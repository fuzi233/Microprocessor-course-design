#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NAV_MAP_SIZE 56
#define NAV_CELL_SIZE_MM 50.0f
#define NAV_SCAN_MAX_BEAMS 72
#define NAV_SECTOR_COUNT 16
#define NAV_PATH_MAX_POINTS 512
#define NAV_FRONTIER_MAX_CLUSTERS 64

#define NAV_UNKNOWN_CELL (-1)
#define NAV_FREE_CELL 0
#define NAV_OCCUPIED_CELL 100

typedef struct
{
    float x_mm;
    float y_mm;
    float theta_rad;
} NavPose;

typedef struct
{
    float linear_vel_mms;
    float angular_vel_rads;
} NavCommand;

typedef struct
{
    float angle_rad;
    float distance_mm;
    bool valid;
} NavScanBeam;

typedef struct
{
    NavScanBeam beams[NAV_SCAN_MAX_BEAMS];
    uint16_t count;
    float max_range_mm;
} NavScan;

typedef struct
{
    int16_t x;
    int16_t y;
} NavCell;

typedef struct
{
    NavCell cells[NAV_PATH_MAX_POINTS];
    uint16_t count;
} NavPath;

typedef struct
{
    int8_t occupancy[NAV_MAP_SIZE][NAV_MAP_SIZE];
    uint8_t inflated[NAV_MAP_SIZE][NAV_MAP_SIZE];
} NavGridMap;

typedef struct
{
    NavCell centroid;
    NavCell representative;
    uint16_t size;
    float score;
} NavFrontierCluster;

typedef struct
{
    uint16_t target_index;
    bool goal_reached;
    float last_target_heading_rad;
} NavTrackerState;

typedef struct
{
    float sector_min_distance_mm[NAV_SECTOR_COUNT];
    bool sector_blocked[NAV_SECTOR_COUNT];
} NavSectorScan;

static inline float Nav_ClampF(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static inline float Nav_NormalizeAngle(float angle_rad)
{
    while (angle_rad > (float)M_PI)
    {
        angle_rad -= 2.0f * (float)M_PI;
    }
    while (angle_rad < -(float)M_PI)
    {
        angle_rad += 2.0f * (float)M_PI;
    }
    return angle_rad;
}

static inline bool Nav_CellInBounds(int x, int y)
{
    return (x >= 0) && (x < NAV_MAP_SIZE) && (y >= 0) && (y < NAV_MAP_SIZE);
}

#endif
