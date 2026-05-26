#include "planner_astar.h"

#include <limits.h>
#include <string.h>

/* A* 内存: 全部堆分配, 无 BSS 大数组 */
#include "FreeRTOS.h"
#define PLANNER_POOL_SIZE (NAV_MAP_SIZE * NAV_MAP_SIZE)

static uint8_t  *planner_open   = NULL;              /* 3KB 堆 */
static uint16_t *planner_g_cost = NULL;              /* 6KB 堆 */
static int16_t  *planner_parent = NULL;              /* 6KB 堆 */
static NavPath  planner_reversed_path;               /* 2KB BSS (避免调用方栈压) */

static bool planner_alloc(void)
{
    if (planner_open != NULL && planner_g_cost != NULL && planner_parent != NULL)
        return true;

    if (planner_open == NULL)
        planner_open   = (uint8_t  *)pvPortMalloc(PLANNER_POOL_SIZE * sizeof(uint8_t));
    if (planner_g_cost == NULL)
        planner_g_cost = (uint16_t *)pvPortMalloc(PLANNER_POOL_SIZE * sizeof(uint16_t));
    if (planner_parent == NULL)
        planner_parent = (int16_t *)pvPortMalloc(PLANNER_POOL_SIZE * sizeof(int16_t));

    if (planner_open == NULL || planner_g_cost == NULL || planner_parent == NULL)
    {
        if (planner_open != NULL)   { vPortFree(planner_open);   planner_open   = NULL; }
        if (planner_g_cost != NULL) { vPortFree(planner_g_cost); planner_g_cost = NULL; }
        if (planner_parent != NULL) { vPortFree(planner_parent); planner_parent = NULL; }
        return false;
    }

    return true;
}

static int32_t NavPlanner_IndexOf(NavCell cell)
{
    return (int32_t)cell.y * NAV_MAP_SIZE + cell.x;
}

static NavCell NavPlanner_CellFromIndex(int32_t index)
{
    NavCell cell;
    cell.x = (int16_t)(index % NAV_MAP_SIZE);
    cell.y = (int16_t)(index / NAV_MAP_SIZE);
    return cell;
}

static uint32_t NavPlanner_Heuristic(NavCell a, NavCell b)
{
    int dx = (a.x > b.x) ? (a.x - b.x) : (b.x - a.x);
    int dy = (a.y > b.y) ? (a.y - b.y) : (b.y - a.y);
    int diag = (dx < dy) ? dx : dy;
    int straight = dx + dy - 2 * diag;
    return (uint32_t)(diag * 14 + straight * 10);
}

bool NavPlanner_AStar(const NavGridMap *map, NavCell start, NavCell goal, NavPath *path)
{
    static const int neighbors[8][3] = {
        {1, 0, 10}, {-1, 0, 10}, {0, 1, 10}, {0, -1, 10},
        {1, 1, 14}, {1, -1, 14}, {-1, 1, 14}, {-1, -1, 14}
    };
    int32_t start_index;
    int32_t goal_index;
    int32_t best_index;
    uint32_t best_score;
    uint32_t expanded_count = 0U;

    if ((map == NULL) || (path == NULL))
    {
        return false;
    }
    path->count = 0U;

    if (!planner_alloc()) return false;

    if (!NavGridMap_IsCellTraversable(map, start.x, start.y, true) ||
        !NavGridMap_IsCellTraversable(map, goal.x, goal.y, true))
    {
        return false;
    }

    memset(planner_open, 0, PLANNER_POOL_SIZE * sizeof(uint8_t));
    for (int32_t i = 0; i < PLANNER_POOL_SIZE; ++i)
    {
        planner_g_cost[i] = UINT16_MAX;
        planner_parent[i] = -1;
    }

    start_index = NavPlanner_IndexOf(start);
    goal_index = NavPlanner_IndexOf(goal);
    planner_g_cost[start_index] = 0U;
    planner_open[start_index] = 1U;

    while (true)
    {
        best_index = -1;
        best_score = UINT16_MAX;

        for (int32_t i = 0; i < NAV_MAP_SIZE * NAV_MAP_SIZE; ++i)
        {
            if (planner_open[i] == 0U)
            {
                continue;
            }

            {
                uint32_t f_score = planner_g_cost[i] +
                                   NavPlanner_Heuristic(NavPlanner_CellFromIndex(i), goal);
                if (f_score < best_score)
                {
                    best_score = f_score;
                    best_index = i;
                }
            }
        }

        if (best_index < 0)
        {
            return false;
        }

        if (best_index == goal_index)
        {
            int32_t current = goal_index;
            planner_reversed_path.count = 0U;

            while (current >= 0 && planner_reversed_path.count < NAV_PATH_MAX_POINTS)
            {
                planner_reversed_path.cells[planner_reversed_path.count++] = NavPlanner_CellFromIndex(current);
                if (current == start_index)
                {
                    break;
                }
                current = planner_parent[current];
            }

            if (planner_reversed_path.count == 0U ||
                planner_reversed_path.cells[planner_reversed_path.count - 1U].x != start.x ||
                planner_reversed_path.cells[planner_reversed_path.count - 1U].y != start.y)
            {
                return false;
            }

            path->count = planner_reversed_path.count;
            for (uint16_t i = 0U; i < planner_reversed_path.count; ++i)
            {
                path->cells[i] = planner_reversed_path.cells[planner_reversed_path.count - 1U - i];
            }
            return true;
        }

        planner_open[best_index] = 0U;
        if (++expanded_count > PLANNER_POOL_SIZE)
        {
            return false;
        }
        /* closed 通过 g_cost != UINT16_MAX 隐式判断 */

        {
            NavCell current = NavPlanner_CellFromIndex(best_index);
            for (int i = 0; i < 8; ++i)
            {
                NavCell next = {
                    (int16_t)(current.x + neighbors[i][0]),
                    (int16_t)(current.y + neighbors[i][1])
                };
                int32_t next_index;
                uint32_t candidate_g;

                if (!NavGridMap_IsCellTraversable(map, next.x, next.y, true))
                {
                    continue;
                }

                next_index = NavPlanner_IndexOf(next);
                if (planner_g_cost[next_index] != UINT16_MAX)
                    continue;  /* 已访问过(closed) */

                candidate_g = planner_g_cost[best_index] + (uint32_t)neighbors[i][2];
                if (candidate_g < planner_g_cost[next_index])
                {
                    planner_g_cost[next_index] = candidate_g;
                    planner_parent[next_index] = best_index;
                    planner_open[next_index] = 1U;
                }
            }
        }
    }
}

void NavPlanner_CompressPath(NavPath *path)
{
    NavPath compact;
    int last_dx;
    int last_dy;

    if (path->count <= 2U)
    {
        return;
    }

    compact.count = 0U;
    compact.cells[compact.count++] = path->cells[0];

    last_dx = path->cells[1].x - path->cells[0].x;
    last_dy = path->cells[1].y - path->cells[0].y;

    for (uint16_t i = 1U; i + 1U < path->count; ++i)
    {
        int dx = path->cells[i + 1U].x - path->cells[i].x;
        int dy = path->cells[i + 1U].y - path->cells[i].y;
        if (dx != last_dx || dy != last_dy)
        {
            compact.cells[compact.count++] = path->cells[i];
            last_dx = dx;
            last_dy = dy;
        }
    }

    compact.cells[compact.count++] = path->cells[path->count - 1U];
    *path = compact;
}
