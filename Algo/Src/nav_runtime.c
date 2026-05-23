#include "nav_runtime.h"

#include <string.h>

static void NavRuntime_InitScan(NavScan *scan, float max_range_mm)
{
    uint16_t i;

    scan->count = NAV_SCAN_MAX_BEAMS;
    scan->max_range_mm = max_range_mm;
    for (i = 0U; i < NAV_SCAN_MAX_BEAMS; ++i)
    {
        scan->beams[i].angle_rad =
            -(float)M_PI + ((2.0f * (float)M_PI * (float)i) / (float)NAV_SCAN_MAX_BEAMS);
        scan->beams[i].distance_mm = max_range_mm;
        scan->beams[i].valid = false;
    }
}

static void NavRuntime_ResetScanBuffers(NavRuntime *runtime)
{
    NavRuntime_InitScan(&runtime->live_scan, runtime->config.scan_max_range_mm);
    NavRuntime_InitScan(&runtime->pending_scan, runtime->config.scan_max_range_mm);
    NavRuntime_InitScan(&runtime->control_scan, runtime->config.scan_max_range_mm);
    runtime->live_scan_valid_points = 0U;
    runtime->scan_pending = false;
    runtime->control_scan_available = false;
}

static bool NavRuntime_PlanToFrontier(NavRuntime *runtime, NavCell start)
{
    uint16_t i;

    runtime->frontier_count = NavFrontier_FindClusters(&runtime->map,
                                                       start,
                                                       runtime->frontiers,
                                                       NAV_FRONTIER_MAX_CLUSTERS);

    for (i = 0U; i < runtime->frontier_count; ++i)
    {
        NavPath new_path;

        if (!NavPlanner_AStar(&runtime->map, start, runtime->frontiers[i].representative, &new_path))
        {
            continue;
        }

        NavPlanner_CompressPath(&new_path);
        runtime->path = new_path;
        runtime->current_goal = runtime->frontiers[i].representative;
        runtime->has_goal = true;
        NavTracker_Reset(&runtime->tracker);
        return true;
    }

    runtime->path.count = 0U;
    runtime->has_goal = false;
    return false;
}

void NavRuntime_SetDefaultConfig(NavRuntimeConfig *config)
{
    config->scan_max_range_mm = 3000.0f;
    config->inflation_radius_cells = 3U;
    config->lookahead_mm = 220.0f;
    config->cruise_speed_mms = 220.0f;
    config->goal_tolerance_mm = 120.0f;
    config->avoidance_block_distance_mm = 320.0f;
    config->avoidance_emergency_distance_mm = 180.0f;
    config->min_scan_points_for_mapping = 12U;
}

void NavRuntime_Init(NavRuntime *runtime, const NavRuntimeConfig *config)
{
    NavRuntimeConfig local_config;

    if (config == NULL)
    {
        NavRuntime_SetDefaultConfig(&local_config);
        config = &local_config;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->config = *config;
    NavGridMap_Init(&runtime->map);
    NavTracker_Reset(&runtime->tracker);
    NavRuntime_ResetScanBuffers(runtime);
    runtime->status = NAV_RUNTIME_WAITING_FOR_POSE;
}

void NavRuntime_ResetPose(NavRuntime *runtime, float x_mm, float y_mm, float theta_rad)
{
    runtime->pose.x_mm = x_mm;
    runtime->pose.y_mm = y_mm;
    runtime->pose.theta_rad = Nav_NormalizeAngle(theta_rad);
    runtime->pose_initialized = true;
    runtime->odometry_initialized = false;
    runtime->path.count = 0U;
    runtime->has_goal = false;
    NavTracker_Reset(&runtime->tracker);
    runtime->status = NAV_RUNTIME_WAITING_FOR_SCAN;
}

void NavRuntime_UpdatePoseFromOdometry(NavRuntime *runtime,
                                       float yaw_rad,
                                       int32_t left_encoder,
                                       int32_t right_encoder,
                                       float ticks_per_cm)
{
    float distance_mm;
    int32_t delta_left;
    int32_t delta_right;

    if (!runtime->pose_initialized)
    {
        runtime->pose.x_mm = 0.0f;
        runtime->pose.y_mm = 0.0f;
        runtime->pose_initialized = true;
    }

    if (!runtime->odometry_initialized)
    {
        runtime->last_left_encoder = left_encoder;
        runtime->last_right_encoder = right_encoder;
        runtime->pose.theta_rad = Nav_NormalizeAngle(yaw_rad);
        runtime->odometry_initialized = true;
        return;
    }

    delta_left = left_encoder - runtime->last_left_encoder;
    delta_right = right_encoder - runtime->last_right_encoder;
    runtime->last_left_encoder = left_encoder;
    runtime->last_right_encoder = right_encoder;

    if (ticks_per_cm <= 0.0f)
    {
        ticks_per_cm = 100.0f;
    }

    distance_mm = ((float)(delta_left + delta_right) * 0.5f) * (10.0f / ticks_per_cm);

    runtime->pose.theta_rad = Nav_NormalizeAngle(yaw_rad);
    runtime->pose.x_mm += cosf(runtime->pose.theta_rad) * distance_mm;
    runtime->pose.y_mm += sinf(runtime->pose.theta_rad) * distance_mm;
}

bool NavRuntime_IngestScanSample(NavRuntime *runtime,
                                 float angle_rad,
                                 float distance_mm,
                                 bool valid,
                                 bool start_of_scan)
{
    int beam_index;
    float normalized;

    if (start_of_scan && runtime->live_scan_valid_points >= runtime->config.min_scan_points_for_mapping)
    {
        runtime->pending_scan = runtime->live_scan;
        runtime->control_scan = runtime->live_scan;
        runtime->scan_pending = true;
        runtime->control_scan_available = true;
        runtime->completed_scan_count++;
        NavRuntime_InitScan(&runtime->live_scan, runtime->config.scan_max_range_mm);
        runtime->live_scan_valid_points = 0U;
    }

    if (!valid || distance_mm <= 0.0f)
    {
        return runtime->scan_pending;
    }

    normalized = Nav_NormalizeAngle(angle_rad);
    beam_index = (int)(((normalized + (float)M_PI) / (2.0f * (float)M_PI)) * (float)NAV_SCAN_MAX_BEAMS);
    if (beam_index < 0)
    {
        beam_index = 0;
    }
    if (beam_index >= (int)NAV_SCAN_MAX_BEAMS)
    {
        beam_index = NAV_SCAN_MAX_BEAMS - 1;
    }

    if (distance_mm > runtime->config.scan_max_range_mm)
    {
        distance_mm = runtime->config.scan_max_range_mm;
    }

    if (!runtime->live_scan.beams[beam_index].valid)
    {
        runtime->live_scan_valid_points++;
    }

    if (!runtime->live_scan.beams[beam_index].valid ||
        distance_mm < runtime->live_scan.beams[beam_index].distance_mm)
    {
        runtime->live_scan.beams[beam_index].distance_mm = distance_mm;
    }

    runtime->live_scan.beams[beam_index].valid = true;
    return runtime->scan_pending;
}

bool NavRuntime_IngestSectorDistances(NavRuntime *runtime,
                                      const float *distances_mm,
                                      const bool *valid_mask,
                                      uint16_t sector_count)
{
    NavScan sector_scan;
    uint16_t beam;

    if (distances_mm == NULL || sector_count == 0U)
    {
        return false;
    }

    NavRuntime_InitScan(&sector_scan, runtime->config.scan_max_range_mm);

    for (beam = 0U; beam < NAV_SCAN_MAX_BEAMS; ++beam)
    {
        float zero_to_two_pi;
        uint16_t sector_index;

        zero_to_two_pi = sector_scan.beams[beam].angle_rad + (float)M_PI;
        sector_index = (uint16_t)((zero_to_two_pi / (2.0f * (float)M_PI)) * (float)sector_count);
        if (sector_index >= sector_count)
        {
            sector_index = (uint16_t)(sector_count - 1U);
        }

        if (valid_mask != NULL && !valid_mask[sector_index])
        {
            continue;
        }

        sector_scan.beams[beam].valid = true;
        sector_scan.beams[beam].distance_mm = distances_mm[sector_index];
        if (sector_scan.beams[beam].distance_mm <= 0.0f ||
            sector_scan.beams[beam].distance_mm > runtime->config.scan_max_range_mm)
        {
            sector_scan.beams[beam].distance_mm = runtime->config.scan_max_range_mm;
        }
    }

    runtime->control_scan = sector_scan;
    runtime->control_scan_available = true;

    if (!runtime->scan_pending)
    {
        runtime->pending_scan = sector_scan;
        runtime->scan_pending = true;
    }

    return true;
}

bool NavRuntime_Step(NavRuntime *runtime, NavCommand *command)
{
    NavCell start;
    NavCommand base_command;

    if (runtime == NULL || command == NULL)
    {
        return false;
    }

    command->linear_vel_mms = 0.0f;
    command->angular_vel_rads = 0.0f;

    /* Step 1 在适配层完成: NavEmbedAdapter_UpdatePose() */

    /* Step 2 在适配层完成: NavRuntime_IngestScanSample() */

    /* Step 3: 将新激光更新到地图中 */
    /* Step 4: 对障碍物做膨胀 */
    if (runtime->scan_pending)
    {
        NavGridMap_InsertScan(&runtime->map, &runtime->pose, &runtime->pending_scan);
        NavGridMap_Inflate(&runtime->map, runtime->config.inflation_radius_cells);
        runtime->scan_pending = false;
    }

    /* 雷达数据就绪检查: 至少需要一轮扫描才能开始导航 */
    if (!runtime->control_scan_available)
    {
        runtime->status = NAV_RUNTIME_WAITING_FOR_SCAN;
        return false;
    }

    if (!runtime->pose_initialized)
    {
        runtime->status = NAV_RUNTIME_WAITING_FOR_POSE;
        return false;
    }

    if (!NavGridMap_WorldToCell(runtime->pose.x_mm, runtime->pose.y_mm, &start))
    {
        runtime->status = NAV_RUNTIME_WAITING_FOR_POSE;
        return false;
    }

    /* Step 5: 如果没有目标 / 目标已到达 / 路径被阻断 → 重新选 Frontier 并规划 */
    if (!runtime->has_goal ||
        runtime->tracker.goal_reached ||
        NavGridMap_IsPathBlocked(&runtime->map, &runtime->path, runtime->tracker.target_index))
    {
        if (!NavRuntime_PlanToFrontier(runtime, start))
        {
            runtime->status = NAV_RUNTIME_FINISHED;
            return false;
        }
    }

    /* Step 6: 用路径跟踪器生成基础控制命令 */
    base_command = NavTracker_Compute(&runtime->pose,
                                      &runtime->path,
                                      &runtime->tracker,
                                      runtime->config.lookahead_mm,
                                      runtime->config.cruise_speed_mms,
                                      runtime->config.goal_tolerance_mm);

    /* Step 7: 用局部避障模块修正控制命令 */
    /* Step 8: 输出最终线速度和角速度 */
    *command = NavAvoidance_Apply(&runtime->pose,
                                  &runtime->control_scan,
                                  &base_command,
                                  runtime->tracker.last_target_heading_rad,
                                  runtime->config.avoidance_block_distance_mm,
                                  runtime->config.avoidance_emergency_distance_mm);
    runtime->status = NAV_RUNTIME_EXPLORING;
    return true;
}
