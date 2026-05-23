#ifndef NAV_RUNTIME_H
#define NAV_RUNTIME_H

#include "frontier.h"
#include "local_avoidance.h"
#include "path_tracker.h"
#include "planner_astar.h"

typedef struct
{
    float scan_max_range_mm;
    uint8_t inflation_radius_cells;
    float lookahead_mm;
    float cruise_speed_mms;
    float goal_tolerance_mm;
    float avoidance_block_distance_mm;
    float avoidance_emergency_distance_mm;
    uint16_t min_scan_points_for_mapping;
} NavRuntimeConfig;

typedef enum
{
    NAV_RUNTIME_WAITING_FOR_POSE = 0,
    NAV_RUNTIME_WAITING_FOR_SCAN,
    NAV_RUNTIME_EXPLORING,
    NAV_RUNTIME_FINISHED
} NavRuntimeStatus;

typedef struct
{
    NavGridMap map;
    NavPose pose;
    NavPath path;
    NavTrackerState tracker;
    NavCell current_goal;
    NavFrontierCluster frontiers[NAV_FRONTIER_MAX_CLUSTERS];
    NavScan live_scan;
    NavScan pending_scan;
    NavScan control_scan;
    NavRuntimeConfig config;
    uint16_t frontier_count;
    uint16_t live_scan_valid_points;
    uint32_t completed_scan_count;
    int32_t last_left_encoder;
    int32_t last_right_encoder;
    bool pose_initialized;
    bool odometry_initialized;
    bool scan_pending;
    bool control_scan_available;
    bool has_goal;
    NavRuntimeStatus status;
} NavRuntime;

void NavRuntime_SetDefaultConfig(NavRuntimeConfig *config);
void NavRuntime_Init(NavRuntime *runtime, const NavRuntimeConfig *config);
void NavRuntime_ResetPose(NavRuntime *runtime, float x_mm, float y_mm, float theta_rad);
void NavRuntime_UpdatePoseFromOdometry(NavRuntime *runtime,
                                       float yaw_rad,
                                       int32_t left_encoder,
                                       int32_t right_encoder,
                                       float ticks_per_cm);
bool NavRuntime_IngestScanSample(NavRuntime *runtime,
                                 float angle_rad,
                                 float distance_mm,
                                 bool valid,
                                 bool start_of_scan);
bool NavRuntime_IngestSectorDistances(NavRuntime *runtime,
                                      const float *distances_mm,
                                      const bool *valid_mask,
                                      uint16_t sector_count);
bool NavRuntime_Step(NavRuntime *runtime, NavCommand *command);

#endif
