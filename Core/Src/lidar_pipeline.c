#include "lidar_pipeline.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define LIDAR_QUEUE1_LENGTH               16U
#define LIDAR_ANALYZER_STACK_WORDS        768U
#define LIDAR_ANALYZER_PRIORITY           (tskIDLE_PRIORITY + 5U)
#define LIDAR_MIN_VALID_DISTANCE_MM       80U
#define LIDAR_MAX_VALID_DISTANCE_MM       3000U
#define LIDAR_DEGREES_PER_REVOLUTION      360.0f
#define LIDAR_Q6_SCALE                    64.0f
#define LIDAR_PI_F                        3.14159265358979323846f

typedef struct
{
  uint16_t hit_count;
  uint32_t sum_distance_mm;
  uint16_t min_distance_mm;
} LidarSectorAccumulator;

static RPLidarContext *s_lidar_ctx;
static QueueHandle_t s_dma_block_queue;
static volatile uint16_t s_last_dma_pos;
static volatile uint32_t s_dma_event_count;
static volatile uint32_t s_uart_rx_event_count;
static volatile uint32_t s_queue1_drop_count;
static volatile uint32_t s_analyzer_block_count;

static LidarRawFrame s_latest_raw_frame;
static LidarSector32Frame s_latest_sector_frame;
static LidarLocalGridFrame s_latest_grid_frame;
static LidarRawFrame s_work_raw_frame;
static LidarLocalGridFrame s_work_grid_frame;
static LidarSectorAccumulator s_work_sector_acc[LIDAR_SECTOR_COUNT];
static volatile bool s_snapshot_ready;
static uint32_t s_published_frame_id;
static uint32_t s_sector_last_hit_ms[LIDAR_SECTOR_COUNT];

static void LidarAnalyzerTask(void *argument);
static void LidarPipeline_SendDescriptorFromISR(uint16_t offset, uint16_t length,
                                                uint32_t timestamp_ms,
                                                BaseType_t *higher_priority_task_woken);
static void LidarPipeline_ResetRawFrame(LidarRawFrame *frame);
static void LidarPipeline_ResetGridFrame(LidarLocalGridFrame *frame);
static void LidarPipeline_ProcessDecodedNodes(const LidarRawNode *nodes, uint16_t node_count,
                                             uint32_t timestamp_ms, LidarRawFrame *raw_frame,
                                             LidarSectorAccumulator sector_acc[LIDAR_SECTOR_COUNT],
                                             LidarLocalGridFrame *grid_frame);
static void LidarPipeline_PublishSnapshots(uint32_t timestamp_ms, const LidarRawFrame *raw_frame,
                                           const LidarSectorAccumulator sector_acc[LIDAR_SECTOR_COUNT],
                                           const LidarLocalGridFrame *grid_frame);
static uint8_t LidarPipeline_GetSectorIndex(uint16_t angle_q6);
static void LidarPipeline_ProjectNodeToGrid(const LidarRawNode *node, LidarLocalGridFrame *grid_frame);
static void LidarPipeline_DrawRay(LidarLocalGridFrame *grid_frame, int32_t end_x, int32_t end_y,
                                  bool mark_endpoint_occupied);
static void LidarPipeline_SetCellFree(LidarLocalGridFrame *grid_frame, int32_t x, int32_t y);
static void LidarPipeline_SetCellOccupied(LidarLocalGridFrame *grid_frame, int32_t x, int32_t y);
static bool LidarPipeline_IsCellInBounds(int32_t x, int32_t y);

bool LidarPipeline_Start(RPLidarContext *ctx, LidarPipelineDebugPrint debug_print)
{
  (void)debug_print;

  s_lidar_ctx = ctx;
  s_last_dma_pos = 0U;
  s_dma_event_count = 0U;
  s_uart_rx_event_count = 0U;
  s_queue1_drop_count = 0U;
  s_analyzer_block_count = 0U;
  s_snapshot_ready = false;
  s_published_frame_id = 0U;

  memset(&s_latest_raw_frame, 0, sizeof(s_latest_raw_frame));
  memset(&s_latest_sector_frame, 0, sizeof(s_latest_sector_frame));
  memset(&s_latest_grid_frame, 0, sizeof(s_latest_grid_frame));
  memset(s_sector_last_hit_ms, 0, sizeof(s_sector_last_hit_ms));

  s_latest_sector_frame.sector_count = LIDAR_SECTOR_COUNT;
  s_latest_grid_frame.width = LIDAR_LOCAL_GRID_WIDTH;
  s_latest_grid_frame.height = LIDAR_LOCAL_GRID_HEIGHT;
  s_latest_grid_frame.resolution_mm = LIDAR_LOCAL_GRID_RESOLUTION_MM;
  s_latest_grid_frame.origin_mode = LIDAR_LOCAL_GRID_ORIGIN_VEHICLE;

  s_dma_block_queue = xQueueCreate(LIDAR_QUEUE1_LENGTH, sizeof(LidarDmaBlockDescriptor));
  if (s_dma_block_queue == NULL)
  {
    return false;
  }

  if (xTaskCreate(LidarAnalyzerTask, "LidarAnalyzer", LIDAR_ANALYZER_STACK_WORDS, NULL,
                  LIDAR_ANALYZER_PRIORITY, NULL) != pdPASS)
  {
    return false;
  }

  return true;
}

bool LidarPipeline_GetLatestRawFrame(LidarRawFrame *out)
{
  if ((out == NULL) || !s_snapshot_ready)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out = s_latest_raw_frame;
  taskEXIT_CRITICAL();
  return true;
}

bool LidarPipeline_GetLatestSector32Frame(LidarSector32Frame *out)
{
  if ((out == NULL) || !s_snapshot_ready)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out = s_latest_sector_frame;
  taskEXIT_CRITICAL();
  return true;
}

bool LidarPipeline_GetLatestLocalGridFrame(LidarLocalGridFrame *out)
{
  if ((out == NULL) || !s_snapshot_ready)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out = s_latest_grid_frame;
  taskEXIT_CRITICAL();
  return true;
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
    LidarPipeline_SendDescriptorFromISR(last_pos, current_pos - last_pos, event_timestamp_ms,
                                        &higher_priority_task_woken);
  }
  else
  {
    LidarPipeline_SendDescriptorFromISR(last_pos, RPLIDAR_DMA_BUFFER_SIZE - last_pos,
                                        event_timestamp_ms, &higher_priority_task_woken);
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
  LidarPipeline_SendDescriptorFromISR(0U, RPLIDAR_DMA_BUFFER_SIZE / 2U, event_timestamp_ms,
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

static void LidarAnalyzerTask(void *argument)
{
  LidarDmaBlockDescriptor descriptor;
  TickType_t next_publish_tick;
  TickType_t now_tick;
  TickType_t wait_ticks;
  uint32_t publish_timestamp_ms;

  (void)argument;

  LidarPipeline_ResetRawFrame(&s_work_raw_frame);
  LidarPipeline_ResetGridFrame(&s_work_grid_frame);
  memset(s_work_sector_acc, 0, sizeof(s_work_sector_acc));

  next_publish_tick = xTaskGetTickCount() + pdMS_TO_TICKS(LIDAR_ANALYSIS_PERIOD_MS);

  for (;;)
  {
    now_tick = xTaskGetTickCount();
    wait_ticks = (now_tick < next_publish_tick) ? (next_publish_tick - now_tick) : 0U;

    if (xQueueReceive(s_dma_block_queue, &descriptor, wait_ticks) == pdPASS)
    {
      RPLidar_ProcessBytes(s_lidar_ctx, &s_lidar_ctx->dma_buffer[descriptor.offset],
                           descriptor.length);
      LidarPipeline_ProcessDecodedNodes(s_lidar_ctx->decoded_nodes, s_lidar_ctx->decoded_node_count,
                                        descriptor.timestamp_ms, &s_work_raw_frame,
                                        s_work_sector_acc, &s_work_grid_frame);
      s_analyzer_block_count++;
    }

    now_tick = xTaskGetTickCount();
    while (now_tick >= next_publish_tick)
    {
      publish_timestamp_ms = HAL_GetTick();
      LidarPipeline_PublishSnapshots(publish_timestamp_ms, &s_work_raw_frame,
                                     s_work_sector_acc, &s_work_grid_frame);
      LidarPipeline_ResetRawFrame(&s_work_raw_frame);
      LidarPipeline_ResetGridFrame(&s_work_grid_frame);
      memset(s_work_sector_acc, 0, sizeof(s_work_sector_acc));
      next_publish_tick += pdMS_TO_TICKS(LIDAR_ANALYSIS_PERIOD_MS);
    }
  }
}

static void LidarPipeline_ResetRawFrame(LidarRawFrame *frame)
{
  memset(frame, 0, sizeof(*frame));
}

static void LidarPipeline_ResetGridFrame(LidarLocalGridFrame *frame)
{
  memset(frame, 0, sizeof(*frame));
  frame->width = LIDAR_LOCAL_GRID_WIDTH;
  frame->height = LIDAR_LOCAL_GRID_HEIGHT;
  frame->resolution_mm = LIDAR_LOCAL_GRID_RESOLUTION_MM;
  frame->origin_mode = LIDAR_LOCAL_GRID_ORIGIN_VEHICLE;
}

static void LidarPipeline_ProcessDecodedNodes(const LidarRawNode *nodes, uint16_t node_count,
                                             uint32_t timestamp_ms, LidarRawFrame *raw_frame,
                                             LidarSectorAccumulator sector_acc[LIDAR_SECTOR_COUNT],
                                             LidarLocalGridFrame *grid_frame)
{
  uint8_t sector_index;
  uint16_t i;
  LidarSectorAccumulator *sector;
  uint16_t distance_mm;

  for (i = 0U; i < node_count; ++i)
  {
    if (raw_frame->sample_count < LIDAR_RAW_FRAME_CAPACITY)
    {
      raw_frame->samples[raw_frame->sample_count++] = nodes[i];
    }

    sector_index = LidarPipeline_GetSectorIndex(nodes[i].angle_q6);
    distance_mm = nodes[i].distance_mm;

    if ((nodes[i].flags & LIDAR_RAW_NODE_FLAG_VALID_DISTANCE) != 0U)
    {
      sector = &sector_acc[sector_index];
      sector->hit_count++;
      sector->sum_distance_mm += distance_mm;
      if ((sector->min_distance_mm == 0U) || (distance_mm < sector->min_distance_mm))
      {
        sector->min_distance_mm = distance_mm;
      }
      s_sector_last_hit_ms[sector_index] = timestamp_ms;
    }

    LidarPipeline_ProjectNodeToGrid(&nodes[i], grid_frame);
  }
}

static void LidarPipeline_PublishSnapshots(uint32_t timestamp_ms, const LidarRawFrame *raw_frame,
                                           const LidarSectorAccumulator sector_acc[LIDAR_SECTOR_COUNT],
                                           const LidarLocalGridFrame *grid_frame)
{
  uint32_t frame_id;
  uint32_t i;

  frame_id = ++s_published_frame_id;

  taskENTER_CRITICAL();

  s_latest_raw_frame = *raw_frame;
  s_latest_raw_frame.frame_id = frame_id;
  s_latest_raw_frame.timestamp_ms = timestamp_ms;

  memset(&s_latest_sector_frame, 0, sizeof(s_latest_sector_frame));
  s_latest_sector_frame.frame_id = frame_id;
  s_latest_sector_frame.timestamp_ms = timestamp_ms;
  s_latest_sector_frame.sector_count = LIDAR_SECTOR_COUNT;

  for (i = 0U; i < LIDAR_SECTOR_COUNT; ++i)
  {
    if (sector_acc[i].hit_count > 0U)
    {
      s_latest_sector_frame.sectors[i].valid = true;
      s_latest_sector_frame.sectors[i].hit_count = sector_acc[i].hit_count;
      s_latest_sector_frame.sectors[i].min_distance_mm = sector_acc[i].min_distance_mm;
      s_latest_sector_frame.sectors[i].mean_distance_mm =
          (uint16_t)(sector_acc[i].sum_distance_mm / sector_acc[i].hit_count);
      s_latest_sector_frame.sectors[i].stale_ms = 0U;
    }
    else
    {
      s_latest_sector_frame.sectors[i].valid = false;
      s_latest_sector_frame.sectors[i].hit_count = 0U;
      s_latest_sector_frame.sectors[i].min_distance_mm = 0U;
      s_latest_sector_frame.sectors[i].mean_distance_mm = 0U;
      s_latest_sector_frame.sectors[i].stale_ms =
          (s_sector_last_hit_ms[i] == 0U) ? UINT32_MAX : (timestamp_ms - s_sector_last_hit_ms[i]);
    }
  }

  s_latest_grid_frame = *grid_frame;
  s_latest_grid_frame.frame_id = frame_id;
  s_latest_grid_frame.timestamp_ms = timestamp_ms;
  s_snapshot_ready = true;
  taskEXIT_CRITICAL();
}

static uint8_t LidarPipeline_GetSectorIndex(uint16_t angle_q6)
{
  uint32_t sector_index;

  sector_index = ((uint32_t)angle_q6 * LIDAR_SECTOR_COUNT) /
                 (uint32_t)(LIDAR_DEGREES_PER_REVOLUTION * LIDAR_Q6_SCALE);
  if (sector_index >= LIDAR_SECTOR_COUNT)
  {
    sector_index = LIDAR_SECTOR_COUNT - 1U;
  }
  return (uint8_t)sector_index;
}

static void LidarPipeline_ProjectNodeToGrid(const LidarRawNode *node, LidarLocalGridFrame *grid_frame)
{
  bool mark_endpoint_occupied;
  float angle_rad;
  float forward_mm;
  float lateral_mm;
  int32_t end_x;
  int32_t end_y;
  uint16_t projected_distance_mm;

  if ((node->flags & LIDAR_RAW_NODE_FLAG_VALID_DISTANCE) != 0U)
  {
    projected_distance_mm = node->distance_mm;
    mark_endpoint_occupied = true;
  }
  else if ((node->distance_mm == 0U) || (node->distance_mm > LIDAR_MAX_VALID_DISTANCE_MM))
  {
    projected_distance_mm = LIDAR_MAX_VALID_DISTANCE_MM;
    mark_endpoint_occupied = false;
  }
  else
  {
    return;
  }

  angle_rad = ((float)node->angle_q6 / LIDAR_Q6_SCALE) * (LIDAR_PI_F / 180.0f);
  forward_mm = cosf(angle_rad) * (float)projected_distance_mm;
  lateral_mm = sinf(angle_rad) * (float)projected_distance_mm;

  end_x = (int32_t)(LIDAR_LOCAL_GRID_WIDTH / 2U) +
          (int32_t)(lateral_mm / (float)LIDAR_LOCAL_GRID_RESOLUTION_MM);
  end_y = (int32_t)(LIDAR_LOCAL_GRID_HEIGHT / 2U) -
          (int32_t)(forward_mm / (float)LIDAR_LOCAL_GRID_RESOLUTION_MM);

  LidarPipeline_DrawRay(grid_frame, end_x, end_y, mark_endpoint_occupied);
}

static void LidarPipeline_DrawRay(LidarLocalGridFrame *grid_frame, int32_t end_x, int32_t end_y,
                                  bool mark_endpoint_occupied)
{
  int32_t x0;
  int32_t y0;
  int32_t dx;
  int32_t dy;
  int32_t sx;
  int32_t sy;
  int32_t err;
  int32_t e2;

  x0 = (int32_t)(LIDAR_LOCAL_GRID_WIDTH / 2U);
  y0 = (int32_t)(LIDAR_LOCAL_GRID_HEIGHT / 2U);

  dx = (end_x > x0) ? (end_x - x0) : (x0 - end_x);
  dy = (end_y > y0) ? (end_y - y0) : (y0 - end_y);
  sx = (x0 < end_x) ? 1 : -1;
  sy = (y0 < end_y) ? 1 : -1;
  err = dx - dy;

  for (;;)
  {
    if ((x0 == end_x) && (y0 == end_y))
    {
      if (mark_endpoint_occupied)
      {
        LidarPipeline_SetCellOccupied(grid_frame, x0, y0);
      }
      else
      {
        LidarPipeline_SetCellFree(grid_frame, x0, y0);
      }
      break;
    }

    LidarPipeline_SetCellFree(grid_frame, x0, y0);

    e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

static void LidarPipeline_SetCellFree(LidarLocalGridFrame *grid_frame, int32_t x, int32_t y)
{
  if (!LidarPipeline_IsCellInBounds(x, y))
  {
    return;
  }

  if (grid_frame->cells[y][x] != LIDAR_GRID_CELL_OCCUPIED)
  {
    grid_frame->cells[y][x] = LIDAR_GRID_CELL_FREE;
  }
}

static void LidarPipeline_SetCellOccupied(LidarLocalGridFrame *grid_frame, int32_t x, int32_t y)
{
  if (!LidarPipeline_IsCellInBounds(x, y))
  {
    return;
  }

  grid_frame->cells[y][x] = LIDAR_GRID_CELL_OCCUPIED;
}

static bool LidarPipeline_IsCellInBounds(int32_t x, int32_t y)
{
  return (x >= 0) && (x < (int32_t)LIDAR_LOCAL_GRID_WIDTH) &&
         (y >= 0) && (y < (int32_t)LIDAR_LOCAL_GRID_HEIGHT);
}
