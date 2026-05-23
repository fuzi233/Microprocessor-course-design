#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "frontier.h"
#include "grid_map.h"
#include "local_avoidance.h"
#include "path_tracker.h"
#include "planner_astar.h"

typedef struct
{
    NavGridMap truth_map;
    NavGridMap built_map;
    NavPose pose;
    NavPath path;
    NavTrackerState tracker;
    NavCell current_goal;
    bool has_goal;
    NavCell trajectory[4096];
    uint16_t trajectory_count;
} ValidationContext;

static void set_truth_obstacle_rect(NavGridMap *map, int x0, int y0, int x1, int y1)
{
    for (int y = y0; y <= y1; ++y)
    {
        for (int x = x0; x <= x1; ++x)
        {
            if (Nav_CellInBounds(x, y))
            {
                map->occupancy[y][x] = NAV_OCCUPIED_CELL;
            }
        }
    }
}

static void build_truth_world(NavGridMap *map)
{
    NavGridMap_Init(map);

    set_truth_obstacle_rect(map, 0, 0, NAV_MAP_SIZE - 1, 1);
    set_truth_obstacle_rect(map, 0, NAV_MAP_SIZE - 2, NAV_MAP_SIZE - 1, NAV_MAP_SIZE - 1);
    set_truth_obstacle_rect(map, 0, 0, 1, NAV_MAP_SIZE - 1);
    set_truth_obstacle_rect(map, NAV_MAP_SIZE - 2, 0, NAV_MAP_SIZE - 1, NAV_MAP_SIZE - 1);

    set_truth_obstacle_rect(map, 18, 18, 21, 95);
    set_truth_obstacle_rect(map, 21, 58, 55, 61);
    set_truth_obstacle_rect(map, 54, 22, 57, 61);
    set_truth_obstacle_rect(map, 70, 18, 73, 108);
    set_truth_obstacle_rect(map, 73, 84, 112, 87);
    set_truth_obstacle_rect(map, 94, 38, 97, 84);
    set_truth_obstacle_rect(map, 38, 98, 90, 101);
    set_truth_obstacle_rect(map, 43, 36, 49, 42);
    set_truth_obstacle_rect(map, 80, 52, 86, 58);

    map->occupancy[59][21] = NAV_FREE_CELL;
    map->occupancy[60][21] = NAV_FREE_CELL;
    map->occupancy[61][21] = NAV_FREE_CELL;

    map->occupancy[60][55] = NAV_FREE_CELL;
    map->occupancy[60][56] = NAV_FREE_CELL;

    map->occupancy[84][73] = NAV_FREE_CELL;
    map->occupancy[84][74] = NAV_FREE_CELL;

    map->occupancy[99][62] = NAV_FREE_CELL;
    map->occupancy[99][63] = NAV_FREE_CELL;
    map->occupancy[99][64] = NAV_FREE_CELL;
}

static bool ray_hits_obstacle(const NavGridMap *map, float x_mm, float y_mm)
{
    NavCell cell;
    if (!NavGridMap_WorldToCell(x_mm, y_mm, &cell))
    {
        return true;
    }
    return map->occupancy[cell.y][cell.x] == NAV_OCCUPIED_CELL;
}

static void generate_scan(const NavGridMap *truth_map, const NavPose *pose, NavScan *scan, int step)
{
    (void)step;
    scan->count = NAV_SCAN_MAX_BEAMS;
    scan->max_range_mm = 1800.0f;

    for (uint16_t i = 0U; i < scan->count; ++i)
    {
        float angle = -(float)M_PI + (2.0f * (float)M_PI * (float)i) / (float)scan->count;
        float distance = scan->max_range_mm;

        scan->beams[i].angle_rad = angle;
        scan->beams[i].valid = true;

        for (float r = NAV_CELL_SIZE_MM * 0.5f; r <= scan->max_range_mm; r += NAV_CELL_SIZE_MM * 0.5f)
        {
            float wx = pose->x_mm + cosf(pose->theta_rad + angle) * r;
            float wy = pose->y_mm + sinf(pose->theta_rad + angle) * r;
            float dyn_dx = wx - 650.0f;
            float dyn_dy = wy - (200.0f + 120.0f * sinf((float)step * 0.05f));
            bool hit_dynamic = (dyn_dx * dyn_dx + dyn_dy * dyn_dy) < (180.0f * 180.0f);

            if (hit_dynamic || ray_hits_obstacle(truth_map, wx, wy))
            {
                distance = r;
                break;
            }
        }

        scan->beams[i].distance_mm = distance;
    }
}

static void append_trajectory(ValidationContext *ctx)
{
    NavCell cell;
    if (ctx->trajectory_count >= (sizeof(ctx->trajectory) / sizeof(ctx->trajectory[0])))
    {
        return;
    }
    if (!NavGridMap_WorldToCell(ctx->pose.x_mm, ctx->pose.y_mm, &cell))
    {
        return;
    }
    if (ctx->trajectory_count > 0U)
    {
        NavCell last = ctx->trajectory[ctx->trajectory_count - 1U];
        if (last.x == cell.x && last.y == cell.y)
        {
            return;
        }
    }
    ctx->trajectory[ctx->trajectory_count++] = cell;
}

static bool plan_to_frontier(ValidationContext *ctx)
{
    NavFrontierCluster clusters[NAV_FRONTIER_MAX_CLUSTERS];
    NavCell start;
    uint16_t cluster_count;

    if (!NavGridMap_WorldToCell(ctx->pose.x_mm, ctx->pose.y_mm, &start))
    {
        return false;
    }

    cluster_count = NavFrontier_FindClusters(&ctx->built_map,
                                             start,
                                             clusters,
                                             NAV_FRONTIER_MAX_CLUSTERS);
    for (uint16_t i = 0U; i < cluster_count; ++i)
    {
        NavPath new_path;
        if (NavPlanner_AStar(&ctx->built_map, start, clusters[i].representative, &new_path))
        {
            NavPlanner_CompressPath(&new_path);
            ctx->path = new_path;
            ctx->current_goal = clusters[i].representative;
            ctx->has_goal = true;
            NavTracker_Reset(&ctx->tracker);
            return true;
        }
    }

    ctx->has_goal = false;
    ctx->path.count = 0U;
    return false;
}

static void step_vehicle(ValidationContext *ctx, const NavCommand *cmd, float dt_s)
{
    ctx->pose.theta_rad = Nav_NormalizeAngle(ctx->pose.theta_rad + cmd->angular_vel_rads * dt_s);
    ctx->pose.x_mm += cosf(ctx->pose.theta_rad) * cmd->linear_vel_mms * dt_s;
    ctx->pose.y_mm += sinf(ctx->pose.theta_rad) * cmd->linear_vel_mms * dt_s;
}

static void write_ppm(const ValidationContext *ctx, const char *file_name)
{
    FILE *fp = fopen(file_name, "wb");
    if (fp == NULL)
    {
        return;
    }

    fprintf(fp, "P6\n%d %d\n255\n", NAV_MAP_SIZE, NAV_MAP_SIZE);

    for (int y = NAV_MAP_SIZE - 1; y >= 0; --y)
    {
        for (int x = 0; x < NAV_MAP_SIZE; ++x)
        {
            unsigned char rgb[3];
            bool on_path = false;
            bool on_planned_path = false;
            bool is_robot = false;
            bool is_goal = false;

            for (uint16_t i = 0U; i < ctx->trajectory_count; ++i)
            {
                if (ctx->trajectory[i].x == x && ctx->trajectory[i].y == y)
                {
                    on_path = true;
                    break;
                }
            }

            for (uint16_t i = 0U; i < ctx->path.count; ++i)
            {
                if (ctx->path.cells[i].x == x && ctx->path.cells[i].y == y)
                {
                    on_planned_path = true;
                    break;
                }
            }

            {
                NavCell robot_cell;
                if (NavGridMap_WorldToCell(ctx->pose.x_mm, ctx->pose.y_mm, &robot_cell) &&
                    robot_cell.x == x && robot_cell.y == y)
                {
                    is_robot = true;
                }
            }

            is_goal = ctx->has_goal && ctx->current_goal.x == x && ctx->current_goal.y == y;

            if (is_robot)
            {
                rgb[0] = 255U; rgb[1] = 0U; rgb[2] = 0U;
            }
            else if (is_goal)
            {
                rgb[0] = 0U; rgb[1] = 255U; rgb[2] = 0U;
            }
            else if (on_planned_path)
            {
                rgb[0] = 0U; rgb[1] = 0U; rgb[2] = 255U;
            }
            else if (on_path)
            {
                rgb[0] = 255U; rgb[1] = 128U; rgb[2] = 0U;
            }
            else if (ctx->built_map.occupancy[y][x] == NAV_OCCUPIED_CELL)
            {
                rgb[0] = 0U; rgb[1] = 0U; rgb[2] = 0U;
            }
            else if (ctx->built_map.occupancy[y][x] == NAV_FREE_CELL)
            {
                rgb[0] = 255U; rgb[1] = 255U; rgb[2] = 255U;
            }
            else
            {
                rgb[0] = 170U; rgb[1] = 170U; rgb[2] = 170U;
            }

            fwrite(rgb, sizeof(unsigned char), 3U, fp);
        }
    }

    fclose(fp);
}

int main(void)
{
    ValidationContext ctx;
    const float dt_s = 0.12f;
    bool finished = false;

    memset(&ctx, 0, sizeof(ctx));
    build_truth_world(&ctx.truth_map);
    NavGridMap_Init(&ctx.built_map);
    ctx.pose.x_mm = -2200.0f;
    ctx.pose.y_mm = -2200.0f;
    ctx.pose.theta_rad = 0.15f;
    NavTracker_Reset(&ctx.tracker);

    for (int step = 0; step < 900; ++step)
    {
        NavScan scan;
        NavCommand base_cmd;
        NavCommand safe_cmd;
        NavCell current_cell;

        generate_scan(&ctx.truth_map, &ctx.pose, &scan, step);
        NavGridMap_InsertScan(&ctx.built_map, &ctx.pose, &scan);
        NavGridMap_Inflate(&ctx.built_map, 3U);
        append_trajectory(&ctx);

        if (!NavGridMap_WorldToCell(ctx.pose.x_mm, ctx.pose.y_mm, &current_cell))
        {
            printf("Robot moved outside the map at step %d\n", step);
            break;
        }

        if (!ctx.has_goal ||
            ctx.tracker.goal_reached ||
            NavGridMap_IsPathBlocked(&ctx.built_map, &ctx.path, ctx.tracker.target_index))
        {
            finished = !plan_to_frontier(&ctx);
            if (finished)
            {
                printf("Exploration finished at step %d\n", step);
                break;
            }
        }

        base_cmd = NavTracker_Compute(&ctx.pose, &ctx.path, &ctx.tracker, 220.0f, 220.0f, 120.0f);
        safe_cmd = NavAvoidance_Apply(&ctx.pose,
                                      &scan,
                                      &base_cmd,
                                      ctx.tracker.last_target_heading_rad,
                                      320.0f,
                                      180.0f);
        step_vehicle(&ctx, &safe_cmd, dt_s);

        if ((step % 40) == 0)
        {
            printf("step=%d pose=(%.0f, %.0f, %.2f) free=%lu path=%u cmd=(%.0f, %.2f)\n",
                   step,
                   (double)ctx.pose.x_mm,
                   (double)ctx.pose.y_mm,
                   (double)ctx.pose.theta_rad,
                   (unsigned long)NavGridMap_CountKnownFree(&ctx.built_map),
                   (unsigned)ctx.path.count,
                   (double)safe_cmd.linear_vel_mms,
                   (double)safe_cmd.angular_vel_rads);
        }
    }

    write_ppm(&ctx, "nav_validation_result.ppm");
    printf("Wrote nav_validation_result.ppm\n");
    return 0;
}
