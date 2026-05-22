#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define RPLIDAR_SCAN_SAMPLE_SIZE 5U
#define RPLIDAR_DMA_BUFFER_SIZE 128U
#define RPLIDAR_DECODED_NODE_BUFFER_SIZE 32U
#define RPLIDAR_CMD_STOP        0x25U
#define RPLIDAR_CMD_SCAN        0x20U

#define LIDAR_RAW_NODE_FLAG_START          (1U << 0)
#define LIDAR_RAW_NODE_FLAG_CHECK_OK       (1U << 1)
#define LIDAR_RAW_NODE_FLAG_VALID_DISTANCE (1U << 2)

typedef struct
{
  uint16_t angle_q6;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t flags;
} LidarRawNode;

typedef struct
{
  uint8_t quality;
  bool start_flag;
  bool inverse_start_flag;
  bool check_bit_ok;
  float angle_deg;
  float distance_mm;
  uint16_t raw_angle_q6;
  uint16_t raw_distance_q2;
} RPLidarScanSample;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t dma_buffer[RPLIDAR_DMA_BUFFER_SIZE];
  uint8_t frame_buffer[RPLIDAR_SCAN_SAMPLE_SIZE];
  uint8_t frame_index;
  uint32_t frame_count;
  uint32_t parse_error_count;
  uint16_t decoded_node_count;
  bool sample_ready;
  RPLidarScanSample latest_sample;
  LidarRawNode decoded_nodes[RPLIDAR_DECODED_NODE_BUFFER_SIZE];
} RPLidarContext;

void RPLidar_Init(RPLidarContext *ctx, UART_HandleTypeDef *huart);
HAL_StatusTypeDef RPLidar_SendCommand(RPLidarContext *ctx, uint8_t command);
HAL_StatusTypeDef RPLidar_StartScanReception(RPLidarContext *ctx);
void RPLidar_ProcessBytes(RPLidarContext *ctx, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
