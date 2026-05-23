#include "planner_astar.h"

#include <limits.h>
#include <string.h>

static uint8_t planner_open[NAV_MAP_SIZE * NAV_MAP_SIZE];
static uint8_t planner_closed[NAV_MAP_SIZE * NAV_MAP_SIZE];
static uint32_t planner_g_cost[NAV_MAP_SIZE * NAV_MAP_SIZE];
static int32_t planner_parent[NAV_MAP_SIZE * NAV_MAP_SIZE];

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

    if (!NavGridMap_IsCellTraversable(map, start.x, start.y, true) ||
        !NavGridMap_IsCellTraversable(map, goal.x, goal.y, true))
    {
        return false;
    }

    memset(planner_open, 0, sizeof(planner_open));
    memset(planner_closed, 0, sizeof(planner_closed));
    for (int32_t i = 0; i < NAV_MAP_SIZE * NAV_MAP_SIZE; ++i)
    {
        planner_g_cost[i] = UINT_MAX;
        planner_parent[i] = -1;
    }

    start_index = NavPlanner_IndexOf(start);
    goal_index = NavPlanner_IndexOf(goal);
    planner_g_cost[start_index] = 0U;
    planner_open[start_index] = 1U;

    while (true)
    {
        best_index = -1;
        best_score = UINT_MAX;

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
            NavPath reversed;
            int32_t current = goal_index;
            reversed.count = 0U;

            while (current >= 0 && reversed.count < NAV_PATH_MAX_POINTS)
            {
                reversed.cells[reversed.count++] = NavPlanner_CellFromIndex(current);
                if (current == start_index)
                {
                    break;
                }
                current = planner_parent[current];
            }

            if (reversed.count == 0U || reversed.cells[reversed.count - 1U].x != start.x ||
                reversed.cells[reversed.count - 1U].y != start.y)
            {
                return false;
            }

            path->count = reversed.count;
            for (uint16_t i = 0U; i < reversed.count; ++i)
            {
                path->cells[i] = reversed.cells[reversed.count - 1U - i];
            }
            return true;
        }

        planner_open[best_index] = 0U;
        planner_closed[best_index] = 1U;

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
                if (planner_closed[next_index] != 0U)
                {
                    continue;
                }

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
