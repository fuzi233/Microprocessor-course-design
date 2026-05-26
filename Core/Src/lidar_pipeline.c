#include "lidar_pipeline.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define LIDAR_QUEUE1_LENGTH          48U  /* 阶段2修复: 16→48, 降低ISR丢包 */
#define LIDAR_QUEUE2_LENGTH          12U  /* 增大Parser→Filter缓冲 */
#define LIDAR_QUEUE3_LENGTH          12U  /* 增大Filter→Output缓冲 */
#define LIDAR_PARSER_STACK_WORDS     512U
#define LIDAR_FILTER_STACK_WORDS     512U
#define LIDAR_OUTPUT_STACK_WORDS     512U
#define LIDAR_PARSER_PRIORITY        (tskIDLE_PRIORITY + 5U)
#define LIDAR_OUTPUT_PRIORITY        (tskIDLE_PRIORITY + 4U)
#define LIDAR_FILTER_PRIORITY        (tskIDLE_PRIORITY + 4U)
#define LIDAR_FILTER_WINDOW_SIZE     4U
#define LIDAR_FILTER_STALE_MS        500U

typedef struct
{
  float samples[LIDAR_FILTER_WINDOW_SIZE];
  float sum;
  uint8_t count;
  uint8_t next_index;
  uint32_t last_update_ms;
} LidarSectorFilterState;

static RPLidarContext *s_lidar_ctx;
static QueueHandle_t s_dma_block_queue;
static QueueHandle_t s_parse_result_queue;
static QueueHandle_t s_filtered_result_queue;
static LidarPipelineDebugPrint s_debug_print;
static LidarSectorFilterState s_sector_filters[8];
static volatile uint16_t s_last_dma_pos;
static volatile uint32_t s_dma_event_count;
static volatile uint32_t s_uart_rx_event_count;
static volatile uint32_t s_queue1_drop_count;
static volatile uint32_t s_parser_block_count;
static volatile uint32_t s_filter_block_count;
static volatile uint32_t s_output_result_count;
static LidarParseResult s_latest_result_snapshot;
static volatile bool s_has_latest_result;

static void LidarParserTask(void *argument);
static void LidarFilterTask(void *argument);
static void LidarOutputTask(void *argument);
static void LidarMonitorTask(void *argument);
static void LidarPipeline_SendDescriptorFromISR(uint16_t offset, uint16_t length,
                                                uint32_t timestamp_ms,
                                                BaseType_t *higher_priority_task_woken);
static void LidarPipeline_FilterPushSample(uint8_t sector, float distance_mm, uint32_t timestamp_ms);
static float LidarPipeline_FilterGetAverage(uint8_t sector, uint32_t now_ms, bool *has_value, uint8_t *sample_count);

bool LidarPipeline_Start(RPLidarContext *ctx, LidarPipelineDebugPrint debug_print)
{
  s_lidar_ctx = ctx;
  s_debug_print = debug_print;
  s_last_dma_pos = 0U;
  s_dma_event_count = 0U;
  s_uart_rx_event_count = 0U;
  s_queue1_drop_count = 0U;
  s_parser_block_count = 0U;
  s_filter_block_count = 0U;
  s_output_result_count = 0U;
  s_has_latest_result = false;
  memset(s_sector_filters, 0, sizeof(s_sector_filters));
  memset(&s_latest_result_snapshot, 0, sizeof(s_latest_result_snapshot));

  s_dma_block_queue = xQueueCreate(LIDAR_QUEUE1_LENGTH, sizeof(LidarDmaBlockDescriptor));
  s_parse_result_queue = xQueueCreate(LIDAR_QUEUE2_LENGTH, sizeof(LidarParseResult));
  s_filtered_result_queue = xQueueCreate(LIDAR_QUEUE3_LENGTH, sizeof(LidarParseResult));
  if ((s_dma_block_queue == NULL) || (s_parse_result_queue == NULL)
      || (s_filtered_result_queue == NULL))
  {
    return false;
  }

  if (xTaskCreate(LidarParserTask, "LidarParser", LIDAR_PARSER_STACK_WORDS, NULL,
                  LIDAR_PARSER_PRIORITY, NULL) != pdPASS)
  {
    return false;
  }

  if (xTaskCreate(LidarOutputTask, "LidarOutput", LIDAR_OUTPUT_STACK_WORDS, NULL,
                  LIDAR_OUTPUT_PRIORITY, NULL) != pdPASS)
  {
    return false;
  }

  if (xTaskCreate(LidarFilterTask, "LidarFilter", LIDAR_FILTER_STACK_WORDS, NULL,
                  LIDAR_FILTER_PRIORITY, NULL) != pdPASS)
  {
    return false;
  }

  if (xTaskCreate(LidarMonitorTask, "LidarMon", 256U, NULL,
                  (tskIDLE_PRIORITY + 2U), NULL) != pdPASS)
  {
    return false;
  }

  return true;
}

void LidarPipeline_ResetRuntimeFromTask(void)
{
  if (s_dma_block_queue != NULL)
  {
    xQueueReset(s_dma_block_queue);
  }
  if (s_parse_result_queue != NULL)
  {
    xQueueReset(s_parse_result_queue);
  }
  if (s_filtered_result_queue != NULL)
  {
    xQueueReset(s_filtered_result_queue);
  }

  taskENTER_CRITICAL();
  s_last_dma_pos = 0U;
  s_has_latest_result = false;
  memset(s_sector_filters, 0, sizeof(s_sector_filters));
  memset(&s_latest_result_snapshot, 0, sizeof(s_latest_result_snapshot));
  taskEXIT_CRITICAL();
}

void LidarPipeline_UartRxEventFromISR(uint16_t size)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  uint16_t current_pos;
  uint16_t last_pos;
  uint32_t event_timestamp_ms;

  if ((s_dma_block_queue == NULL) || (s_lidar_ctx == NULL))
  {
    return;
  }

  s_uart_rx_event_count++;
  event_timestamp_ms = HAL_GetTick();

  if (size > RPLIDAR_DMA_BUFFER_SIZE)
  {
    size = RPLIDAR_DMA_BUFFER_SIZE;
  }

  current_pos = (size == RPLIDAR_DMA_BUFFER_SIZE) ? 0U : size;
  last_pos = s_last_dma_pos;

  if (current_pos == last_pos)
  {
    return;
  }

  if (current_pos > last_pos)
  {
    LidarPipeline_SendDescriptorFromISR(last_pos, current_pos - last_pos,
                                        event_timestamp_ms,
                                        &higher_priority_task_woken);
  }
  else
  {
    LidarPipeline_SendDescriptorFromISR(last_pos, RPLIDAR_DMA_BUFFER_SIZE - last_pos,
                                        event_timestamp_ms,
                                        &higher_priority_task_woken);
    if (current_pos > 0U)
    {
      LidarPipeline_SendDescriptorFromISR(0U, current_pos, event_timestamp_ms,
                                          &higher_priority_task_woken);
    }
  }

  s_last_dma_pos = current_pos;
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void LidarPipeline_DmaHalfCompleteFromISR(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  uint32_t event_timestamp_ms;

  if ((s_dma_block_queue == NULL) || (s_lidar_ctx == NULL))
  {
    return;
  }

  s_uart_rx_event_count++;
  event_timestamp_ms = HAL_GetTick();
  LidarPipeline_SendDescriptorFromISR(0U, RPLIDAR_DMA_BUFFER_SIZE / 2U,
                                      event_timestamp_ms,
                                      &higher_priority_task_woken);
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void LidarPipeline_DmaCompleteFromISR(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  uint32_t event_timestamp_ms;

  if ((s_dma_block_queue == NULL) || (s_lidar_ctx == NULL))
  {
    return;
  }

  s_uart_rx_event_count++;
  event_timestamp_ms = HAL_GetTick();
  LidarPipeline_SendDescriptorFromISR(RPLIDAR_DMA_BUFFER_SIZE / 2U,
                                      RPLIDAR_DMA_BUFFER_SIZE / 2U,
                                      event_timestamp_ms,
                                      &higher_priority_task_woken);
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

bool LidarPipeline_GetLatestResult(LidarParseResult *result_out)
{
  bool has_result;

  if (result_out == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  has_result = s_has_latest_result;
  if (has_result)
  {
    *result_out = s_latest_result_snapshot;
  }
  taskEXIT_CRITICAL();

  return has_result;
}

void LidarPipeline_GetStats(LidarPipelineStats *stats_out)
{
  if (stats_out == NULL) return;
  stats_out->uart_rx_event_count = s_uart_rx_event_count;
  stats_out->dma_event_count    = s_dma_event_count;
  stats_out->queue1_drop_count  = s_queue1_drop_count;
  stats_out->parser_block_count = s_parser_block_count;
  stats_out->filter_block_count = s_filter_block_count;
  stats_out->output_result_count = s_output_result_count;
  stats_out->frame_count   = (s_lidar_ctx != NULL) ? s_lidar_ctx->frame_count : 0U;
  stats_out->parse_error_count = (s_lidar_ctx != NULL) ? s_lidar_ctx->parse_error_count : 0U;
  stats_out->pipeline_started = (s_dma_block_queue != NULL);
}

static void LidarPipeline_SendDescriptorFromISR(uint16_t offset, uint16_t length,
                                                uint32_t timestamp_ms,
                                                BaseType_t *higher_priority_task_woken)
{
  LidarDmaBlockDescriptor descriptor;

  if (length == 0U)
  {
    return;
  }

  descriptor.offset = offset;
  descriptor.length = length;
  descriptor.event_count = ++s_dma_event_count;
  descriptor.timestamp_ms = timestamp_ms;
  if (xQueueSendFromISR(s_dma_block_queue, &descriptor, higher_priority_task_woken) != pdPASS)
  {
    s_queue1_drop_count++;
  }
}

static void LidarParserTask(void *argument)
{
  LidarDmaBlockDescriptor descriptor;
  LidarParseResult result;
  uint32_t previous_frame_count;
  uint32_t previous_parse_error_count;
  uint32_t i;
  float angle;
  uint8_t sector;
  MotorSyncSnapshot sync_snapshot;

  (void)argument;

  for (;;)
  {
    if (xQueueReceive(s_dma_block_queue, &descriptor, portMAX_DELAY) == pdPASS)
    {
      previous_frame_count = s_lidar_ctx->frame_count;
      previous_parse_error_count = s_lidar_ctx->parse_error_count;

      result.min_distance_mm = 99999.0f;
      result.total_nodes_extracted = 0U;
      for (i = 0U; i < 8U; i++)
      {
        result.sector_obstacle[i] = false;
        result.sector_min_distance[i] = 99999.0f;
      }

      RPLidar_ProcessBytes(s_lidar_ctx, &s_lidar_ctx->dma_buffer[descriptor.offset],
                           descriptor.length);

      result.valid_sample_count = s_lidar_ctx->frame_count - previous_frame_count;
      result.total_nodes_extracted = result.valid_sample_count;

      if (result.valid_sample_count > 0U)
      {
        angle = s_lidar_ctx->latest_sample.angle_deg;
        if (angle < 0.0f)
        {
          angle += 360.0f;
        }
        sector = (uint8_t)(angle / 45.0f);
        if (sector >= 8U)
        {
          sector = 7U;
        }

        if (s_lidar_ctx->latest_sample.distance_mm > 0.0f)
        {
          result.sector_obstacle[sector] = true;
          result.sector_min_distance[sector] = s_lidar_ctx->latest_sample.distance_mm;
          result.min_distance_mm = s_lidar_ctx->latest_sample.distance_mm;
        }
      }

      result.block_id = descriptor.event_count;
      result.offset = descriptor.offset;
      result.length = descriptor.length;
      result.dma_timestamp_ms = descriptor.timestamp_ms;
      result.sample_timestamp_ms = HAL_GetTick();
      result.parse_error_count = s_lidar_ctx->parse_error_count - previous_parse_error_count;
      result.has_sample = (result.valid_sample_count > 0U);
      result.latest_sample = s_lidar_ctx->latest_sample;
      result.filtered_min_distance_mm = 99999.0f;
      memset(result.filtered_sector_obstacle, 0, sizeof(result.filtered_sector_obstacle));
      memset(result.filtered_sector_distance, 0, sizeof(result.filtered_sector_distance));
      memset(result.filter_sample_count, 0, sizeof(result.filter_sample_count));
      
      // 填充简单的同步数据 (兼容旧代码)
      memset(&sync_snapshot, 0, sizeof(sync_snapshot));
      result.sync_snapshot = sync_snapshot;
      result.sync_delta_ms = 0U;
      s_parser_block_count++;

      (void)xQueueSend(s_parse_result_queue, &result, portMAX_DELAY);
    }
  }
}

static void LidarFilterTask(void *argument)
{
  LidarParseResult result;
  LidarParseResult filtered_result;
  uint32_t i;
  bool has_average;
  uint8_t sample_count;
  float average_distance;
  float latest_angle;
  uint8_t latest_sector;

  (void)argument;

  for (;;)
  {
    if (xQueueReceive(s_parse_result_queue, &result, portMAX_DELAY) == pdPASS)
    {
      filtered_result = result;
      filtered_result.filtered_min_distance_mm = 99999.0f;

      if (result.has_sample && (result.latest_sample.distance_mm > 0.0f))
      {
        latest_angle = result.latest_sample.angle_deg;
        if (latest_angle < 0.0f)
        {
          latest_angle += 360.0f;
        }
        latest_sector = (uint8_t)(latest_angle / 45.0f);
        if (latest_sector >= 8U)
        {
          latest_sector = 7U;
        }
        LidarPipeline_FilterPushSample(latest_sector, result.latest_sample.distance_mm,
                                       result.sample_timestamp_ms);
      }

      for (i = 0U; i < 8U; i++)
      {
        average_distance = LidarPipeline_FilterGetAverage((uint8_t)i, result.sample_timestamp_ms,
                                                          &has_average, &sample_count);
        filtered_result.filter_sample_count[i] = sample_count;
        if (has_average)
        {
          filtered_result.filtered_sector_obstacle[i] = true;
          filtered_result.filtered_sector_distance[i] = average_distance;
          if (average_distance < filtered_result.filtered_min_distance_mm)
          {
            filtered_result.filtered_min_distance_mm = average_distance;
          }
        }
        else
        {
          filtered_result.filtered_sector_obstacle[i] = false;
          filtered_result.filtered_sector_distance[i] = 0.0f;
        }
      }

      if (filtered_result.filtered_min_distance_mm >= 99999.0f)
      {
        filtered_result.filtered_min_distance_mm = result.min_distance_mm;
      }

      s_filter_block_count++;
      (void)xQueueSend(s_filtered_result_queue, &filtered_result, portMAX_DELAY);
    }
  }
}

static void LidarOutputTask(void *argument)
{
  LidarParseResult result;
  LidarParseResult latest_result;
  char line[64];
  TickType_t last_print_time;
  bool has_latest_data;
  float grid_dist[9];  // 3x3 grid: 0-8
  uint8_t i;
  
  // 最大显示距离 (mm)，超过这个距离不显示
  #define MAX_DISPLAY_DISTANCE 3000.0f

  (void)argument;

  last_print_time = xTaskGetTickCount();
  has_latest_data = false;
  memset(&latest_result, 0, sizeof(latest_result));
  memset(grid_dist, 0, sizeof(grid_dist));

  for (;;)
  {
    if (xQueueReceive(s_filtered_result_queue, &result, pdMS_TO_TICKS(100)) == pdPASS)
    {
      latest_result = result;
      has_latest_data = true;
      s_output_result_count++;
      s_lidar_ctx->sample_ready = false;
      taskENTER_CRITICAL();
      s_latest_result_snapshot = result;
      s_has_latest_result = true;
      taskEXIT_CRITICAL();
    }

    // 暂时禁用激光雷达打印输出，避免干扰编码器调试信息
    // if ((xTaskGetTickCount() - last_print_time) >= pdMS_TO_TICKS(3000))
    // {
    //   last_print_time = xTaskGetTickCount();
    // 
    //   if (has_latest_data && (s_debug_print != NULL))
    //   {
    //     // 将8个扇区映射到3x3网格
    //     // 扇区映射:
    //     // 6(270-315°) → 0(左上
    //     // 7(315-360°) → 1(上前
    //     // 0(0-45°) → 2(右上
    //     // 5(225-270°) → 3(左中
    //     // 空 → 4(中心
    //     // 1(45-90°) → 5(右中
    //     // 4(180-225°) → 6(左下
    //     // 3(135-180°) → 7(下后
    //     // 2(90-135°) → 8(右下
    //     
    //     for (i = 0; i < 9; i++)
    //     {
    //       grid_dist[i] = -1.0f;  // -1 表示无数据
    //     }
    //     
    //     // 映射各个扇区
    //     if (latest_result.filtered_sector_obstacle[6]) grid_dist[0] = latest_result.filtered_sector_distance[6];  // 左上
    //     if (latest_result.filtered_sector_obstacle[7]) grid_dist[1] = latest_result.filtered_sector_distance[7];  // 上前
    //     if (latest_result.filtered_sector_obstacle[0]) grid_dist[2] = latest_result.filtered_sector_distance[0];  // 右上
    //     if (latest_result.filtered_sector_obstacle[5]) grid_dist[3] = latest_result.filtered_sector_distance[5];  // 左中
    //     // 中心 grid_dist[4] 中心不使用
    //     if (latest_result.filtered_sector_obstacle[1]) grid_dist[5] = latest_result.filtered_sector_distance[1];  // 右中
    //     if (latest_result.filtered_sector_obstacle[4]) grid_dist[6] = latest_result.filtered_sector_distance[4];  // 左下
    //     if (latest_result.filtered_sector_obstacle[3]) grid_dist[7] = latest_result.filtered_sector_distance[3];  // 下后
    //     if (latest_result.filtered_sector_obstacle[2]) grid_dist[8] = latest_result.filtered_sector_distance[2];  // 右下
    //     
    //     // 第一行
    //     for (i = 0; i < 3; i++)
    //     {
    //       if (grid_dist[i] > 0 && grid_dist[i] < MAX_DISPLAY_DISTANCE)
    //       {
    //         snprintf(line, sizeof(line), "[%4.0f]", (double)grid_dist[i]);
    //       }
    //       else
    //       {
    //         snprintf(line, sizeof(line), "[    ]");
    //       }
    //       s_debug_print(line);
    //     }
    //     s_debug_print("\r\n");
    //     
    //     // 第二行
    //     for (i = 3; i < 6; i++)
    //     {
    //       if (grid_dist[i] > 0 && grid_dist[i] < MAX_DISPLAY_DISTANCE)
    //       {
    //         snprintf(line, sizeof(line), "[%4.0f]", (double)grid_dist[i]);
    //       }
    //       else
    //       {
    //         snprintf(line, sizeof(line), "[    ]");
    //       }
    //       s_debug_print(line);
    //     }
    //     s_debug_print("\r\n");
    //     
    //     // 第三行
    //     for (i = 6; i < 9; i++)
    //     {
    //       if (grid_dist[i] > 0 && grid_dist[i] < MAX_DISPLAY_DISTANCE)
    //       {
    //         snprintf(line, sizeof(line), "[%4.0f]", (double)grid_dist[i]);
    //       }
    //       else
    //       {
    //         snprintf(line, sizeof(line), "[    ]");
    //       }
    //       s_debug_print(line);
    //     }
    //     s_debug_print("\r\n\r\n");
    //   }
    // }
  }
}

static void LidarMonitorTask(void *argument)
{
  // char message[192];
  // uint32_t dma_remaining;
  // uint32_t uart_error;
  // uint32_t uart_state;
  // uint32_t dma_state;

  (void)argument;

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000U));

    // 关闭 LidarMonitor 输出，只保留雷达数据
    // if (s_debug_print != NULL)
    // {
    //   dma_remaining = 0U;
    //   uart_error = 0U;
    //   uart_state = 0U;
    //   dma_state = 0U;
    // 
    //   if ((s_lidar_ctx != NULL) && (s_lidar_ctx->huart != NULL))
    //   {
    //     uart_error = HAL_UART_GetError(s_lidar_ctx->huart);
    //     uart_state = (uint32_t)HAL_UART_GetState(s_lidar_ctx->huart);
    //     if (s_lidar_ctx->huart->hdmarx != NULL)
    //     {
    //       dma_remaining = __HAL_DMA_GET_COUNTER(s_lidar_ctx->huart->hdmarx);
    //       dma_state = (uint32_t)HAL_DMA_GetState(s_lidar_ctx->huart->hdmarx);
    //     }
    //   }
    // 
    //   snprintf(message, sizeof(message),
    //            "LIDAR monitor: rx=%lu q1=%lu drop=%lu parsed=%lu filt=%lu out=%lu ndtr=%lu uerr=0x%lX ustate=%lu dstate=%lu\r\n",
    //            (unsigned long)s_uart_rx_event_count,
    //            (unsigned long)s_dma_event_count,
    //            (unsigned long)s_queue1_drop_count,
    //            (unsigned long)s_parser_block_count,
    //            (unsigned long)s_filter_block_count,
    //            (unsigned long)s_output_result_count,
    //            (unsigned long)dma_remaining,
    //            (unsigned long)uart_error,
    //            (unsigned long)uart_state,
    //            (unsigned long)dma_state);
    //   s_debug_print(message);
    // }
  }
}

static void LidarPipeline_FilterPushSample(uint8_t sector, float distance_mm, uint32_t timestamp_ms)
{
  LidarSectorFilterState *state;

  if (sector >= 8U)
  {
    return;
  }

  state = &s_sector_filters[sector];
  if (state->count < LIDAR_FILTER_WINDOW_SIZE)
  {
    state->samples[state->next_index] = distance_mm;
    state->sum += distance_mm;
    state->count++;
  }
  else
  {
    state->sum -= state->samples[state->next_index];
    state->samples[state->next_index] = distance_mm;
    state->sum += distance_mm;
  }

  state->next_index = (uint8_t)((state->next_index + 1U) % LIDAR_FILTER_WINDOW_SIZE);
  state->last_update_ms = timestamp_ms;
}

static float LidarPipeline_FilterGetAverage(uint8_t sector, uint32_t now_ms, bool *has_value, uint8_t *sample_count)
{
  const LidarSectorFilterState *state;

  *has_value = false;
  *sample_count = 0U;
  if (sector >= 8U)
  {
    return 0.0f;
  }

  state = &s_sector_filters[sector];
  if ((state->count == 0U) || ((now_ms - state->last_update_ms) > LIDAR_FILTER_STALE_MS))
  {
    return 0.0f;
  }

  *has_value = true;
  *sample_count = state->count;
  return state->sum / (float)state->count;
}
