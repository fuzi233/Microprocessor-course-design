#ifndef __LIDAR_PIPELINE_H
#define __LIDAR_PIPELINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "rplidar.h"

/* 简化的MotorSyncSnapshot - 用于兼容旧代码 */
typedef struct {
  int32_t left_encoder;
  int32_t right_encoder;
  float left_speed_mm_s;
  float right_speed_mm_s;
  bool manual_mode;
  uint32_t fault_flags;
  uint32_t control_timestamp_ms;
} MotorSyncSnapshot;

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
  uint32_t block_id;
  uint16_t offset;
  uint16_t length;
  uint32_t dma_timestamp_ms;
  uint32_t sample_timestamp_ms;
  uint32_t valid_sample_count;
  uint32_t parse_error_count;
  bool has_sample;
  RPLidarScanSample latest_sample;
  float min_distance_mm;
  float filtered_min_distance_mm;
  uint32_t total_nodes_extracted;
  bool sector_obstacle[8];
  float sector_min_distance[8];
  bool filtered_sector_obstacle[8];
  float filtered_sector_distance[8];
  uint8_t filter_sample_count[8];
  uint32_t sync_delta_ms;
  MotorSyncSnapshot sync_snapshot;
} LidarParseResult;

/* 阶段2：DMA统计信息 */
typedef struct {
    uint32_t uart_rx_event_count;
    uint32_t dma_event_count;
    uint32_t queue1_drop_count;
    uint32_t parser_block_count;
    uint32_t filter_block_count;
    uint32_t output_result_count;
    uint32_t frame_count;
    uint32_t parse_error_count;
    bool pipeline_started;
} LidarPipelineStats;

bool LidarPipeline_Start(RPLidarContext *ctx, LidarPipelineDebugPrint debug_print);
void LidarPipeline_ResetRuntimeFromTask(void);
void LidarPipeline_UartRxEventFromISR(uint16_t size);
void LidarPipeline_DmaHalfCompleteFromISR(void);
void LidarPipeline_DmaCompleteFromISR(void);
bool LidarPipeline_GetLatestResult(LidarParseResult *result_out);
void LidarPipeline_GetStats(LidarPipelineStats *stats_out);

#ifdef __cplusplus
}
#endif

#endif
