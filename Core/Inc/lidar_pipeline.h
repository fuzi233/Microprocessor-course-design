#ifndef __LIDAR_PIPELINE_H
#define __LIDAR_PIPELINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "rplidar.h"

#define LIDAR_ANALYSIS_PERIOD_MS         50U
#define LIDAR_RAW_FRAME_CAPACITY         256U
#define LIDAR_SECTOR_COUNT               32U
#define LIDAR_LOCAL_GRID_WIDTH           64U
#define LIDAR_LOCAL_GRID_HEIGHT          64U
#define LIDAR_LOCAL_GRID_RESOLUTION_MM   50U
#define LIDAR_LOCAL_GRID_ORIGIN_VEHICLE  0U

typedef void (*LidarPipelineDebugPrint)(const char *message);

typedef struct
{
  uint16_t offset;
  uint16_t length;
  uint32_t event_count;
  uint32_t timestamp_ms;
} LidarDmaBlockDescriptor;

typedef struct
{
  uint32_t frame_id;
  uint32_t timestamp_ms;
  uint16_t sample_count;
  LidarRawNode samples[LIDAR_RAW_FRAME_CAPACITY];
} LidarRawFrame;

typedef struct
{
  bool valid;
  uint16_t hit_count;
  uint16_t min_distance_mm;
  uint16_t mean_distance_mm;
  uint32_t stale_ms;
} LidarSector32Entry;

typedef struct
{
  uint32_t frame_id;
  uint32_t timestamp_ms;
  uint8_t sector_count;
  LidarSector32Entry sectors[LIDAR_SECTOR_COUNT];
} LidarSector32Frame;

typedef enum
{
  LIDAR_GRID_CELL_UNKNOWN = 0,
  LIDAR_GRID_CELL_FREE = 1,
  LIDAR_GRID_CELL_OCCUPIED = 2
} LidarLocalGridCellState;

typedef struct
{
  uint32_t frame_id;
  uint32_t timestamp_ms;
  uint8_t width;
  uint8_t height;
  uint16_t resolution_mm;
  uint8_t origin_mode;
  uint8_t cells[LIDAR_LOCAL_GRID_HEIGHT][LIDAR_LOCAL_GRID_WIDTH];
} LidarLocalGridFrame;

typedef struct
{
  LidarRawFrame raw_frame;
  LidarSector32Frame sector_frame;
  LidarLocalGridFrame grid_frame;
} LidarParseResult;

bool LidarPipeline_Start(RPLidarContext *ctx, LidarPipelineDebugPrint debug_print);
void LidarPipeline_UartRxEventFromISR(uint16_t size);
void LidarPipeline_DmaHalfCompleteFromISR(void);
void LidarPipeline_DmaCompleteFromISR(void);
bool LidarPipeline_GetLatestRawFrame(LidarRawFrame *out);
bool LidarPipeline_GetLatestSector32Frame(LidarSector32Frame *out);
bool LidarPipeline_GetLatestLocalGridFrame(LidarLocalGridFrame *out);

#ifdef __cplusplus
}
#endif

#endif
