#include "rplidar.h"

#include <string.h>

static bool RPLidar_FrameLooksValid(const uint8_t frame[RPLIDAR_SCAN_SAMPLE_SIZE])
{
  bool start_flag;
  bool inverse_start_flag;
  bool check_bit_ok;

  start_flag = (frame[0] & 0x01U) != 0U;
  inverse_start_flag = (frame[0] & 0x02U) != 0U;
  check_bit_ok = (frame[1] & 0x01U) != 0U;

  return (start_flag != inverse_start_flag) && check_bit_ok;
}

static void RPLidar_DecodeSample(RPLidarContext *ctx, const uint8_t frame[RPLIDAR_SCAN_SAMPLE_SIZE])
{
  RPLidarScanSample *sample = &ctx->latest_sample;
  uint16_t angle_q6;
  uint16_t distance_q2;

  sample->start_flag = (frame[0] & 0x01U) != 0U;
  sample->inverse_start_flag = (frame[0] & 0x02U) != 0U;
  sample->quality = frame[0] >> 2;
  sample->check_bit_ok = (frame[1] & 0x01U) != 0U;

  if ((sample->start_flag == sample->inverse_start_flag) || !sample->check_bit_ok)
  {
    ctx->parse_error_count++;
    return;
  }

  angle_q6 = (uint16_t)(((uint16_t)frame[1] >> 1) | ((uint16_t)frame[2] << 7));
  distance_q2 = (uint16_t)((uint16_t)frame[3] | ((uint16_t)frame[4] << 8));

  sample->raw_angle_q6 = angle_q6;
  sample->raw_distance_q2 = distance_q2;
  sample->angle_deg = (float)angle_q6 / 64.0f;
  sample->distance_mm = (float)distance_q2 / 4.0f;

  ctx->frame_count++;
  ctx->sample_ready = true;
}

void RPLidar_Init(RPLidarContext *ctx, UART_HandleTypeDef *huart)
{
  memset(ctx, 0, sizeof(*ctx));
  ctx->huart = huart;
}

HAL_StatusTypeDef RPLidar_SendCommand(RPLidarContext *ctx, uint8_t command)
{
  uint8_t frame[2];

  frame[0] = 0xA5U;
  frame[1] = command;

  return HAL_UART_Transmit(ctx->huart, frame, sizeof(frame), 100U);
}

HAL_StatusTypeDef RPLidar_StartScanReception(RPLidarContext *ctx)
{
  return HAL_UART_Receive_DMA(ctx->huart, ctx->dma_buffer, sizeof(ctx->dma_buffer));
}

void RPLidar_ProcessBytes(RPLidarContext *ctx, const uint8_t *data, size_t len)
{
  size_t i;

  for (i = 0; i < len; ++i)
  {
    ctx->frame_buffer[ctx->frame_index++] = data[i];
    if (ctx->frame_index == RPLIDAR_SCAN_SAMPLE_SIZE)
    {
      if (RPLidar_FrameLooksValid(ctx->frame_buffer))
      {
        RPLidar_DecodeSample(ctx, ctx->frame_buffer);
        ctx->frame_index = 0U;
      }
      else
      {
        memmove(&ctx->frame_buffer[0], &ctx->frame_buffer[1], RPLIDAR_SCAN_SAMPLE_SIZE - 1U);
        ctx->frame_index = RPLIDAR_SCAN_SAMPLE_SIZE - 1U;
        ctx->parse_error_count++;
      }
    }
  }
}
