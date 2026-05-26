/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mpu6500.h"
#include "oled_ssd1306.h"
#include "button.h"
#include "motor_driver.h"
#include "rplidar.h"
#include "lidar_pipeline.h"
#include "grid_map.h"
#include "planner_astar.h"
#include "car_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ========== 阶段4：FreeRTOS + DMA + Mapping + A* ========== */

/* --- 任务优先级 (0 = idle, 7 = 最高) ---
 * IMU(6) > LidarParser(5, lidar_pipeline内部) > Motor(4) > LidarFilter/Output(4)
 * > Button(3) > Monitor/LidarMon(2) > OLED(1)
 */
#define PRIO_IMU            6U   /* IMU DMA，最高优先 */
#define PRIO_MOTOR          4U   /* 电机 PID 控制 */
#define PRIO_BUTTON         3U   /* 按键轮询 */
#define PRIO_MONITOR        4U   /* 系统监控(提升到4，避免被雷达管线饿死) */
#define PRIO_OLED           1U   /* OLED 显示（最低） */

/* --- 任务栈大小 (words, 1 word = 4 bytes) --- */
#define STACK_IMU           512U
#define STACK_MAPPING       1024U
#define STACK_MOTOR         256U
#define STACK_BUTTON        256U
#define STACK_MONITOR       512U  /* snprintf 需要较大栈 */
#define STACK_OLED          512U  /* SSD1306 帧缓冲 1KB */

/* --- 任务周期 --- */
#define IMU_PERIOD_MS       20U
#define MOTOR_PERIOD_MS     10U
#define BUTTON_PERIOD_MS    10U
#define MONITOR_PERIOD_MS   2000U
#define OLED_PERIOD_MS      200U

/* --- 电机测试参数 --- */
#define MOTOR_TEST_SPEED    (TARGET_TICKS_PER_PERIOD / 2)  /* 半速 */
#define MOTOR_TEST_FWD_MS   2000U  /* 前进时长 */
#define MOTOR_TEST_REV_MS   2000U  /* 后退时长 */
#define MOTOR_TEST_STOP_MS  1500U  /* 停顿时长 */

/* --- Mapping 测试 --- */
#define ENABLE_MAPPING_TEST

/* --- Mapping 参数 --- */
#define MAP_SCAN_BEAMS      72U     /* 每帧扫描光束数 (360°/5°) */
#define MAP_ANGLE_BUCKETS   72U     /* 角度桶数量 */
#define MAP_MIN_VALID_BEAMS 24U     /* 放宽阈值，避免轮询采样时长期达不到插图条件 */
#define MAP_INSERT_PERIOD_MS 1000U  /* 给角度桶更多积累时间 */
#define MAP_STATS_PERIOD_MS  5000U  /* 地图统计输出周期 */

/* --- ADC --- */
#define ADC_BUFFER_SIZE     3U

/* --- IMU --- */
#define IMU_DMA_TIMEOUT_MS  500U

#define FW_DIAG_TAG         "[FW] diag-20260526-1735 map-print-before-astar lidar-hard-restart\r\n"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* ---- 硬件上下文 ---- */
static MPU6500Context g_mpu6500;
static RPLidarContext   g_rplidar;
static CarContext_t     g_car;

/* ---- Mapping (阶段3) ---- */
static NavGridMap       g_grid_map;

/* ---- 导航状态 ---- */
static volatile bool    g_nav_active = false;
static NavPath          g_nav_path;

/* ---- FreeRTOS 对象 ---- */
static SemaphoreHandle_t g_i2c_mutex = NULL;       /* I2C1 总线互斥锁 */
static SemaphoreHandle_t g_uart_mutex = NULL;      /* UART2 发送互斥锁 */
static TaskHandle_t g_task_imu_handle = NULL;       /* IMU 任务句柄 */
static TaskHandle_t g_task_mapping_handle = NULL;   /* Mapping 任务句柄 */
static TaskHandle_t g_task_motor_handle = NULL;     /* 电机测试任务句柄 */
static TaskHandle_t g_task_button_handle = NULL;    /* 按键任务句柄 */
static TaskHandle_t g_task_monitor_handle = NULL;   /* 监控任务句柄 */
static TaskHandle_t g_task_oled_handle = NULL;      /* OLED 任务句柄 */
static volatile uint8_t g_lidar_restart_requested = 0U;
static volatile uint32_t g_lidar_uart_error_flags = 0U;

/* ---- DMA 状态标志 ---- */
static volatile uint8_t g_mpu_dma_busy = 0U;
static uint32_t g_last_mpu_dma_start_ms = 0U;

/* ---- ADC DMA 缓冲区 ---- */
uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

/* ---- 系统运行时间戳（供监控任务使用）---- */
static uint32_t g_boot_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* ---- 任务函数声明 ---- */
static void Task_IMU(void *argument);
static void Task_MotorTest(void *argument);
static void Task_Mapping(void *argument);
static void Task_Button(void *argument);
static void Task_Monitor(void *argument);
static void Task_OLED(void *argument);

/* ---- 辅助函数声明 ---- */
static void Debug_Print(const char *message);
static void Fault_PrintRaw(const char *message);
static void Lidar_RequestRestartFromISR(uint32_t error_flags);
static void Lidar_RestartScanIfNeeded(const char *reason);
static void I2C_MutexInit(void);
static void I2C_Lock(void);
static void I2C_Unlock(void);
static void I2C_UnlockFromISR(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================================================================
 * 调试输出
 * ======================================================================== */

static void Debug_Print(const char *message)
{
    BaseType_t uart_locked = pdFALSE;

    if (message == NULL) {
        return;
    }

    if (g_uart_mutex != NULL) {
        uart_locked = xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(200));
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)message,
                      (uint16_t)strlen(message), 100U);

    if ((g_uart_mutex != NULL) && (uart_locked == pdTRUE)) {
        xSemaphoreGive(g_uart_mutex);
    }
}

static void Fault_PrintRaw(const char *message)
{
    if (message == NULL) {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)message,
                      (uint16_t)strlen(message), 100U);
}

static void Lidar_RequestRestartFromISR(uint32_t error_flags)
{
    g_lidar_uart_error_flags = error_flags;
    g_lidar_restart_requested = 1U;
}

static void Lidar_RestartScanIfNeeded(const char *reason)
{
    char buf[192];
    HAL_StatusTypeDef st;
    HAL_StatusTypeDef stop_st;
    HAL_StatusTypeDef reset_st;
    HAL_StatusTypeDef scan_st;
    uint32_t frames_before = g_rplidar.frame_count;
    uint32_t err_before = HAL_UART_GetError(&huart3);
    LidarPipelineStats stats_before;
    LidarPipeline_GetStats(&stats_before);

    (void)HAL_UART_AbortReceive(&huart3);
    HAL_UART_DMAStop(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    LidarPipeline_ResetRuntimeFromTask();

    memset(g_rplidar.dma_buffer, 0, sizeof(g_rplidar.dma_buffer));
    memset(g_rplidar.frame_buffer, 0, sizeof(g_rplidar.frame_buffer));
    g_rplidar.frame_index = 0U;

    stop_st = RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_STOP);
    HAL_Delay(50);
    reset_st = HAL_OK;  /* 不发送 RESET(会重置雷达波特率) */

    st = RPLidar_StartScanReception(&g_rplidar);
    if (st == HAL_OK) {
        HAL_Delay(10);
        scan_st = RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_SCAN);
        snprintf(buf, sizeof(buf),
                 "[LIDAR] Stream restarted (%s, err=0x%lX/0x%lX, st stop/reset/rx/scan=%d/%d/%d/%d, frames=%lu, rx_evt=%lu)\r\n",
                 reason,
                 (unsigned long)g_lidar_uart_error_flags,
                 (unsigned long)err_before,
                 (int)stop_st,
                 (int)reset_st,
                 (int)st,
                 (int)scan_st,
                 (unsigned long)frames_before,
                 (unsigned long)stats_before.uart_rx_event_count);
        Debug_Print(buf);
    } else {
        snprintf(buf, sizeof(buf),
                 "[LIDAR] Stream restart FAILED (%s, err=0x%lX/0x%lX, st stop/reset/rx=%d/%d/%d, frames=%lu, rx_evt=%lu)\r\n",
                 reason,
                 (unsigned long)g_lidar_uart_error_flags,
                 (unsigned long)err_before,
                 (int)stop_st,
                 (int)reset_st,
                 (int)st,
                 (unsigned long)frames_before,
                 (unsigned long)stats_before.uart_rx_event_count);
        Debug_Print(buf);
    }

    g_lidar_uart_error_flags = 0U;
    g_lidar_restart_requested = 0U;
}

/* ========================================================================
 * I2C 总线互斥锁（OLED + MPU6500 共用 I2C1）
 * ======================================================================== */

static void I2C_MutexInit(void)
{
    /* I2C 总线需要在 Task 和 DMA ISR 之间交接，不能使用从 ISR 释放的 Mutex。 */
    g_i2c_mutex = xSemaphoreCreateBinary();
    if (g_i2c_mutex == NULL) {
        Debug_Print("[FATAL] I2C Mutex create failed!\r\n");
        return;
    }

    xSemaphoreGive(g_i2c_mutex);
}

static void I2C_Lock(void)
{
    if (g_i2c_mutex != NULL) {
        xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    }
}

static void I2C_Unlock(void)
{
    if (g_i2c_mutex != NULL) {
        xSemaphoreGive(g_i2c_mutex);
    }
}

static void I2C_UnlockFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_i2c_mutex != NULL) {
        xSemaphoreGiveFromISR(g_i2c_mutex, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ========================================================================
 * Task_IMU — 周期读取 MPU6500 陀螺仪数据（DMA + I2C）
 * 周期: 20ms  |  优先级: 5（高，数据生产者）
 * ======================================================================== */

static void Task_IMU(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t now;
    (void)argument;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
        now = HAL_GetTick();

        /* 以 IMU 任务周期驱动姿态积分，避免只靠 Monitor/OLED 低频刷新。 */
        MPU6500_UpdateYaw(&g_mpu6500);

        /* DMA 超时检测 */
        if (g_mpu_dma_busy != 0U) {
            if ((now - g_last_mpu_dma_start_ms) > IMU_DMA_TIMEOUT_MS) {
                g_mpu_dma_busy = 0U;
                g_mpu6500.total_error_count++;
                Debug_Print("[IMU] DMA timeout!\r\n");
            } else {
                continue;  /* 等上一次 DMA 完成 */
            }
        }

        /* 跳过未就绪的传感器 */
        if (!g_mpu6500.gyro.who_am_i_ok) {
            continue;
        }

        /* 启动 DMA 读取，成功后清除旧标志 */
        HAL_StatusTypeDef st = MPU6500_StartGyroReadDMA(&g_mpu6500);
        if (st == HAL_OK) {
            g_mpu6500.gyro.data_ready = false;
            g_mpu_dma_busy = 1U;
            g_last_mpu_dma_start_ms = now;
        } else if (st != HAL_BUSY) {
            g_mpu6500.total_error_count++;
        }
    }
}

/* ========================================================================
 * Task_MotorTest — 电机往复测试
 * 周期: 10ms  |  优先级: 4
 *
 * 测试序列: 前进(2s) → 停止(1.5s) → 后退(2s) → 停止(1.5s) → 循环
 * 结果通过编码器和串口输出验证
 * ======================================================================== */

typedef enum {
    MOTOR_PHASE_FWD = 0,
    MOTOR_PHASE_STOP1,
    MOTOR_PHASE_REV,
    MOTOR_PHASE_STOP2,
} MotorTestPhase;

/* ========================================================================
 * Task_Mapping — 栅格地图构建 (阶段3)
 * 周期: 10ms 轮询雷达样本  |  优先级: 4
 *
 * 从 RPLidarContext.latest_sample 持续采集激光点，
 * 按 5° 角度分桶，累积足够数据后插入栅格地图。
 * ======================================================================== */

static void Task_Mapping(void *argument)
{
    /* 角度桶：存储每个 5° 扇区的最新距离 */
    static float    bucket_dist[MAP_ANGLE_BUCKETS];
    static bool     bucket_valid[MAP_ANGLE_BUCKETS];
    static NavPath  s_astar_path;  /* 避免 2KB 级路径数组压在任务栈上 */
    static uint32_t last_frame_count = 0U;
    static uint32_t last_insert_ms  = 0U;
    static uint32_t last_stats_ms   = 0U;
    static uint32_t last_vofa_ms    = 0U;
    static uint32_t last_diag_ms    = 0U;
    static uint32_t insert_attempts = 0U;
    static uint32_t insert_successes = 0U;
    static uint32_t insert_skips = 0U;
    uint32_t        now;
    int             i;
    NavPose         pose;
    char            buf[128];
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(2000)); /* 等雷达数据稳定 */

    memset(bucket_dist, 0, sizeof(bucket_dist));
    memset(bucket_valid, 0, sizeof(bucket_valid));
    NavGridMap_Init(&g_grid_map);

    /* 初始化定时器为当前时刻，避免立即触发插入/统计 */
    last_insert_ms  = HAL_GetTick();
    last_stats_ms   = HAL_GetTick();
    last_vofa_ms    = HAL_GetTick();
    last_diag_ms    = HAL_GetTick();

    Debug_Print("[MAP] Mapping started\r\n");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(13)); /* 13ms 与雷达周期~70ms互质，避免谐波采样 */
        now = HAL_GetTick();

        /* ---- 以下检查不受雷达数据有无影响（即使雷达停转也执行）---- */

        /* 诊断输出（每 2 秒）+ A* 规划（首次 fr>=25 时触发一次）*/
        if ((now - last_diag_ms) >= 2000U) {
            last_diag_ms = now;
            uint32_t map_free = NavGridMap_CountKnownFree(&g_grid_map);
            int bucket_fill = 0;
            for (i = 0; i < MAP_ANGLE_BUCKETS; i++) {
                if (bucket_valid[i]) bucket_fill++;
            }
            snprintf(buf, sizeof(buf), "diag,b=%d/%u,fr=%lu\r\n",
                     bucket_fill, (unsigned int)MAP_ANGLE_BUCKETS,
                     (unsigned long)map_free);
            Debug_Print(buf);

            /* -- A* 测试：首次 fr>=25 触发一次 -- */
            static bool astdone = false;
            if (!astdone && map_free >= 25U) {
                astdone = true;
                Debug_Print("as,go\r\n");

                NavCell sc;
                if (!NavGridMap_WorldToCell(0.0f, 0.0f, &sc)) {
                    Debug_Print("as,no-start\r\n");
                    continue;
                }

                /* 膨胀 + 把 UNKNOWN 临时标为 FREE（8方向射线投射盲区太多）*/
                NavGridMap_Inflate(&g_grid_map, 3U);
                for (int gy = 0; gy < NAV_MAP_SIZE; gy++) {
                    for (int gx = 0; gx < NAV_MAP_SIZE; gx++) {
                        if (g_grid_map.occupancy[gy][gx] != NAV_OCCUPIED_CELL) {
                            g_grid_map.occupancy[gy][gx] = NAV_FREE_CELL;
                        }
                    }
                }
                /* 确保起点+周围可通行（occupancy 和 inflated 都要清）*/
                g_grid_map.occupancy[28][28] = NAV_FREE_CELL;
                g_grid_map.inflated[28][28]  = 0U;
                g_grid_map.inflated[27][28]  = 0U;
                g_grid_map.inflated[29][28]  = 0U;
                g_grid_map.inflated[28][27]  = 0U;
                g_grid_map.inflated[28][29]  = 0U;

                NavCell gc = sc;
                int best = 0;
                for (int y = 0; y < NAV_MAP_SIZE; y++) {
                    for (int x = 0; x < NAV_MAP_SIZE; x++) {
                        if (NavGridMap_IsCellTraversable(&g_grid_map, x, y, true)) {
                            int dx = x - sc.x, dy = y - sc.y, d2 = dx*dx + dy*dy;
                            if (d2 > best) { best = d2; gc.x = (int16_t)x; gc.y = (int16_t)y; }
                        }
                    }
                }
                s_astar_path.count = 0U;
                uint32_t t0 = HAL_GetTick();
                bool ok = NavPlanner_AStar(&g_grid_map, sc, gc, &s_astar_path);
                uint32_t dt = HAL_GetTick() - t0;
                snprintf(buf, sizeof(buf), "as,ok=%u,dt=%lu,raw=%u\r\n",
                         (unsigned int)ok, (unsigned long)dt, (unsigned int)s_astar_path.count);
                Debug_Print(buf);

                if (ok) {
                    uint16_t raw = s_astar_path.count;
                    NavPlanner_CompressPath(&s_astar_path);
                    snprintf(buf, sizeof(buf), "as,path,(%d,%d)->(%d,%d),r%u,c%u\r\n",
                             (int)sc.x,(int)sc.y,(int)gc.x,(int)gc.y,
                             (unsigned int)raw,(unsigned int)s_astar_path.count);
                    Debug_Print(buf);
                    /* 保存路径并激活导航 */
                    g_nav_path = s_astar_path;
                    g_nav_active = true;
                    Debug_Print("nav,start\r\n");
                } else {
                    snprintf(buf, sizeof(buf), "as,fail,(%d,%d)->(%d,%d)\r\n",
                             (int)sc.x,(int)sc.y,(int)gc.x,(int)gc.y);
                    Debug_Print(buf);
                }
            }
        }

        /* VOFA 每 1 秒 */
        if ((now - last_vofa_ms) >= 1000U) {
            last_vofa_ms = now;
            snprintf(buf, sizeof(buf), "vofa,%.1f,%lu\r\n",
                     (double)((float)(xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000.0f),
                     (unsigned long)NavGridMap_CountKnownFree(&g_grid_map));
            Debug_Print(buf);
        }

        /* ---- 以下需要雷达新数据 ---- */
        uint32_t frames = g_rplidar.frame_count;
        if (frames == last_frame_count) continue;
        last_frame_count = frames;

        /* 读取最新样本 */
        RPLidarScanSample *s = &g_rplidar.latest_sample;
        float angle = s->angle_deg;
        bool sample_valid = (s->distance_mm > 1.0f) && (s->distance_mm <= 8000.0f);

        /* 角度归一化到 0~360° */
        if (angle < 0.0f)  angle += 360.0f;
        if (angle >= 360.0f) angle -= 360.0f;

        if (sample_valid) {
            /* 放入对应角度桶 */
            int bucket = (int)(angle / (360.0f / MAP_ANGLE_BUCKETS));
            if (bucket < 0) bucket = 0;
            if (bucket >= (int)MAP_ANGLE_BUCKETS) bucket = MAP_ANGLE_BUCKETS - 1;

            bucket_dist[bucket]  = s->distance_mm;
            bucket_valid[bucket] = true;
        }

        /* 定期插入地图 */
        if ((now - last_insert_ms) >= MAP_INSERT_PERIOD_MS) {
            last_insert_ms = now;
            insert_attempts++;

            /* 统计有效桶数 */
            int valid_count = 0;
            for (i = 0; i < MAP_ANGLE_BUCKETS; i++) {
                if (bucket_valid[i]) valid_count++;
            }

            if (valid_count >= MAP_MIN_VALID_BEAMS) {
                insert_successes++;
                /* 构建 NavScan */
                static NavScan scan;  /* static避免栈溢出(864字节) */
                scan.count = 0;
                scan.max_range_mm = 8000.0f;

                for (i = 0; i < MAP_ANGLE_BUCKETS; i++) {
                    if (bucket_valid[i] && scan.count < NAV_SCAN_MAX_BEAMS) {
                        scan.beams[scan.count].angle_rad =
                            (float)i * (2.0f * (float)M_PI / MAP_ANGLE_BUCKETS);
                        scan.beams[scan.count].distance_mm = bucket_dist[i];
                        scan.beams[scan.count].valid = true;
                        scan.count++;
                    }
                }

                /* 当前阶段先用 IMU yaw 修正车体朝向。
                 * x/y 仍假设基本不变，适合原地旋转和小范围挪动测试。 */
                pose.x_mm = 0.0f;
                pose.y_mm = 0.0f;
                pose.theta_rad = MPU6500_GetYaw(&g_mpu6500) * ((float)M_PI / 180.0f);

                /* ---- 纯整数射线投射 (替代 NavGridMap_InsertScan) ---- */
                /* 预计算 8 方向步进表: N,NE,E,SE,S,SW,W,NW */
                static const int8_t dir_dx[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
                static const int8_t dir_dy[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
                NavCell robot_cell;
                if (NavGridMap_WorldToCell(0.0f, 0.0f, &robot_cell)) {
                    g_grid_map.occupancy[robot_cell.y][robot_cell.x] = NAV_FREE_CELL;
                    for (uint16_t bi = 0; bi < scan.count; bi++) {
                        if (!scan.beams[bi].valid || scan.beams[bi].distance_mm <= 1.0f)
                            continue;
                        /* 桶索引→8方向 (纯整数: (8*i+36)/72 映射到最近45°方向) */
                        int di = ((bi * 8 + 36) / 72) & 7;
                        /* 步数 = 距离 / 格大小 */
                        int steps = (int)(scan.beams[bi].distance_mm / NAV_CELL_SIZE_MM);
                        if (steps < 1) steps = 1;
                        if (steps > 160) steps = 160; /* 8m 上限 */
                        int cx = robot_cell.x, cy = robot_cell.y;
                        for (int s = 1; s < steps; s++) {
                            cx += dir_dx[di];
                            cy += dir_dy[di];
                            if (!Nav_CellInBounds(cx, cy)) break;
                            if (g_grid_map.occupancy[cy][cx] != NAV_OCCUPIED_CELL)
                                g_grid_map.occupancy[cy][cx] = NAV_FREE_CELL;
                        }
                        /* 终点标记为障碍（未超出最大量程时）*/
                        cx += dir_dx[di];
                        cy += dir_dy[di];
                        if (Nav_CellInBounds(cx, cy) &&
                            scan.beams[bi].distance_mm < (scan.max_range_mm - NAV_CELL_SIZE_MM)) {
                            g_grid_map.occupancy[cy][cx] = NAV_OCCUPIED_CELL;
                        }
                    }
                }

                /* 每 4 次插入膨胀一次 */
                static int insert_count = 0;
                if (++insert_count >= 4) {
                    NavGridMap_Inflate(&g_grid_map, 3U);
                    insert_count = 0;
                }

                uint32_t free_now = NavGridMap_CountKnownFree(&g_grid_map);
                snprintf(buf, sizeof(buf),
                         "ins,ok,%ub,fr=%lu\r\n",
                         (unsigned int)scan.count,
                         (unsigned long)free_now);
                Debug_Print(buf);

                /* 清空桶 */
                memset(bucket_valid, 0, sizeof(bucket_valid));
            } else {
                insert_skips++;
                snprintf(buf, sizeof(buf),
                         "[MAPDBG] skip insert: valid=%d/%u frames=%lu last_angle=%.1f dist=%.0f sample_ok=%u\r\n",
                         valid_count,
                         (unsigned int)MAP_MIN_VALID_BEAMS,
                         (unsigned long)g_rplidar.frame_count,
                         (double)angle,
                         (double)s->distance_mm,
                         (unsigned int)sample_valid);
                Debug_Print(buf);
            }
        }

        /* ---- 导航控制: A* 成功后执行 (开环PWM) ---- */
        if (g_nav_active) {
            static uint32_t  nav_phase_ms = 0U;

            /* ---- 路径跟踪: 逐路点导航 ---- */
            static int    nav_wp  = 1;      /* 当前路点 (跳过起点) */
            static float  nav_px  = 0.0f;   /* 里程计 x (mm) */
            static float  nav_py  = 0.0f;   /* 里程计 y (mm) */
            static float  nav_th  = 0.0f;   /* 里程计航向 (rad) */
            static int32_t nav_le = 0;       /* 上次左编码器 */
            static int32_t nav_re = 0;       /* 上次右编码器 */
            static bool   nav_init = false;

            if (!nav_init) {
                nav_init = true;
                nav_wp   = 1;
                nav_px   = 0.0f; nav_py = 0.0f; nav_th = 0.0f;
                nav_le   = MotorDriver_GetEncoderCount(MOTOR_LEFT);
                nav_re   = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
                Debug_Print("nav,go\r\n");
            }

            /* 编码器里程计更新 */
            {
                int32_t le = MotorDriver_GetEncoderCount(MOTOR_LEFT);
                int32_t re = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
                int32_t dl = le - nav_le;
                int32_t dr = re - nav_re;
                nav_le = le; nav_re = re;

                float dist_mm = (float)(dl + dr) * 0.5f * (50.0f / 100.0f); /* 100ticks/cm→0.5mm/tick */
                nav_px += dist_mm * trig_cos(nav_th);
                nav_py += dist_mm * trig_sin(nav_th);
                nav_th += (float)(dr - dl) * 0.5f * (50.0f / 100.0f) / 75.0f; /* wheelbase≈75mm */
                while (nav_th >  (float)M_PI) nav_th -= 2.0f*(float)M_PI;
                while (nav_th < -(float)M_PI) nav_th += 2.0f*(float)M_PI;
            }

            if (nav_wp >= (int)g_nav_path.count) {
                MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
                MotorDriver_StopMotor(MOTOR_BOTH);
                snprintf(buf, sizeof(buf), "nav,done,pose=(%.0f,%.0f)\r\n",
                         (double)nav_px, (double)nav_py);
                Debug_Print(buf);
                g_nav_active = false; nav_init = false;
            } else {
                /* 计算目标方向 (trig_atan2 替代 atan2f) */
                NavCell *wp = &g_nav_path.cells[nav_wp];
                float wx, wy;
                NavGridMap_CellToWorld(*wp, &wx, &wy);
                float dx = wx - nav_px, dy = wy - nav_py;
                float tgt_a = trig_atan2(dy, dx);
                float a_err = tgt_a - nav_th;
                while (a_err >  (float)M_PI) a_err -= 2.0f*(float)M_PI;
                while (a_err < -(float)M_PI) a_err += 2.0f*(float)M_PI;

                /* 平方距离判定到达 (避免 sqrtf) */
                float d2 = dx*dx + dy*dy;

                if (d2 < 22500.0f) {  /* 150² = 22500 mm² */
                    nav_wp++;
                    if (nav_wp < (int)g_nav_path.count) {
                        snprintf(buf, sizeof(buf), "nav,wp%d\r\n", nav_wp);
                        Debug_Print(buf);
                    }
                }

                /* 角度误差 → 差速转向 (修正极性: err>0 目标在左→右轮加速) */
                int32_t turn = (int32_t)(a_err * 60.0f);
                if (turn > 300) turn = 300;
                if (turn < -300) turn = -300;

                /* ---- 自建 PI 速度闭环 (13ms周期, 低增益) ---- */
                static int32_t le_prev = 0, re_prev = 0;
                static float   l_i = 100.0f, r_i = 100.0f;
                static int32_t l_p = 200, r_p = 400;

                int32_t le = MotorDriver_GetEncoderCount(MOTOR_LEFT);
                int32_t re = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
                int32_t ld = le - le_prev; le_prev = le;
                int32_t rd = re - re_prev; re_prev = re;

                /* 目标: 两轮同速 (20 ticks/13ms ≈ 1540 ticks/s) */
                int32_t l_err = 20 - ld;
                int32_t r_err = 20 - rd;
                l_i += (float)l_err * 0.08f;
                r_i += (float)r_err * 0.08f;
                if (l_i >  600.0f) l_i =  600.0f;
                if (l_i < -600.0f) l_i = -600.0f;
                if (r_i >  600.0f) r_i =  600.0f;
                if (r_i < -600.0f) r_i = -600.0f;

                l_p = (int32_t)((float)l_err * 4.0f + l_i);
                r_p = (int32_t)((float)r_err * 4.0f + r_i);
                if (l_p > 800) l_p = 800; if (l_p < -800) l_p = -800;
                if (r_p > 800) r_p = 800; if (r_p < -800) r_p = -800;
                if (l_p >= 0 && l_p < 60) l_p = 60;   /* 最小 PWM 克服静摩擦 */
                if (r_p >= 0 && r_p < 60) r_p = 60;
                if (l_p < 0 && l_p > -60) l_p = -60;
                if (r_p < 0 && r_p > -60) r_p = -60;

                /* 叠加转向: turn>0(目标在左)→右轮加速左轮减速 */
                int32_t l_out = l_p + turn;
                int32_t r_out = r_p - turn;
                if (l_out > 800) l_out = 800; if (l_out < -800) l_out = -800;
                if (r_out > 800) r_out = 800; if (r_out < -800) r_out = -800;
                if (l_out >= 0 && l_out < 60) l_out = 60;
                if (r_out >= 0 && r_out < 60) r_out = 60;

                MotorDriver_SetRawPWM(MOTOR_LEFT,  l_out);
                MotorDriver_SetRawPWM(MOTOR_RIGHT, r_out);

                static uint32_t dbg = 0;
                if ((HAL_GetTick() - dbg) >= 2000U) {
                    dbg = HAL_GetTick();
                    snprintf(buf, sizeof(buf), "pi,ld=%ld,rd=%ld,lp=%ld,rp=%ld,err=%.0f\r\n",
                             (long)ld, (long)rd, (long)l_out, (long)r_out,
                             (double)(a_err * 57.3f));
                    Debug_Print(buf);
                }

                static uint32_t nav_prt = 0;
                if ((HAL_GetTick() - nav_prt) >= 2000U) {
                    nav_prt = HAL_GetTick();
                    snprintf(buf, sizeof(buf), "nav,wp%d,err=%.1f,Lp=%ld,Rp=%ld\r\n",
                             nav_wp, (double)(a_err*57.3f),
                             (long)MotorDriver_GetCurrentPWM(MOTOR_LEFT),
                             (long)MotorDriver_GetCurrentPWM(MOTOR_RIGHT));
                    Debug_Print(buf);
                }
            }
        }

        /* 定期输出地图统计（人类可读） */
        if ((now - last_stats_ms) >= MAP_STATS_PERIOD_MS) {
            last_stats_ms = now;
            uint32_t known_free  = NavGridMap_CountKnownFree(&g_grid_map);
            uint32_t total_cells = NAV_MAP_SIZE * NAV_MAP_SIZE;

            snprintf(buf, sizeof(buf),
                     "[MAP] free=%lu/%lu cells | lidar_frames=%lu\r\n",
                     (unsigned long)known_free,
                     (unsigned long)total_cells,
                     (unsigned long)g_rplidar.frame_count);
            Debug_Print(buf);
        }
    }
}

static void Task_MotorTest(void *argument)
{
    MotorTestPhase phase = MOTOR_PHASE_FWD;
    TickType_t phase_start = xTaskGetTickCount();
    char buf[128];
    (void)argument;

    /* 等待系统就绪 */
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* 确保编码器从 0 开始 */
    MotorDriver_ResetEncoder(MOTOR_BOTH);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 强制重使能编码器定时器（绕过 HAL 状态检查）*/
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_TIM_ENABLE(&htim2);
    __HAL_TIM_ENABLE(&htim3);

    Debug_Print("[MOTOR] Test started\r\n");

    for (;;) {
        uint32_t elapsed = (xTaskGetTickCount() - phase_start) * portTICK_PERIOD_MS;

        switch (phase) {
        case MOTOR_PHASE_FWD:
            MotorDriver_SetTargetSpeed(MOTOR_LEFT,  MOTOR_TEST_SPEED);
            MotorDriver_SetTargetSpeed(MOTOR_RIGHT, MOTOR_TEST_SPEED);
            if (elapsed >= MOTOR_TEST_FWD_MS) {
                phase = MOTOR_PHASE_STOP1;
                phase_start = xTaskGetTickCount();
                Debug_Print("[MOTOR] FWD done, stopping...\r\n");
            }
            break;

        case MOTOR_PHASE_STOP1:
            MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
            MotorDriver_StopMotor(MOTOR_BOTH);
            if (elapsed >= MOTOR_TEST_STOP_MS) {
                phase = MOTOR_PHASE_REV;
                phase_start = xTaskGetTickCount();
                Debug_Print("[MOTOR] REV start\r\n");
            }
            break;

        case MOTOR_PHASE_REV:
            MotorDriver_SetTargetSpeed(MOTOR_LEFT,  -MOTOR_TEST_SPEED);
            MotorDriver_SetTargetSpeed(MOTOR_RIGHT, -MOTOR_TEST_SPEED);
            if (elapsed >= MOTOR_TEST_REV_MS) {
                phase = MOTOR_PHASE_STOP2;
                phase_start = xTaskGetTickCount();
                Debug_Print("[MOTOR] REV done, stopping...\r\n");
            }
            break;

        case MOTOR_PHASE_STOP2:
            MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
            MotorDriver_StopMotor(MOTOR_BOTH);
            if (elapsed >= MOTOR_TEST_STOP_MS) {
                phase = MOTOR_PHASE_FWD;
                phase_start = xTaskGetTickCount();
                Debug_Print("[MOTOR] --- New cycle ---\r\n");
            }
            break;
        }

        /* PID 闭环 */
        MotorDriver_PIDControl(MOTOR_BOTH);

        /* 每秒打印一次状态 */
        if ((elapsed % 1000U) < MOTOR_PERIOD_MS) {
            snprintf(buf, sizeof(buf),
                     "[MOTOR] phase=%u L_enc=%ld R_enc=%ld L_pwm=%ld R_pwm=%ld rawT2=%lu rawT3=%lu\r\n",
                     (unsigned int)phase,
                     (long)MotorDriver_GetEncoderCount(MOTOR_LEFT),
                     (long)MotorDriver_GetEncoderCount(MOTOR_RIGHT),
                     (long)MotorDriver_GetCurrentPWM(MOTOR_LEFT),
                     (long)MotorDriver_GetCurrentPWM(MOTOR_RIGHT),
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim2),
                     (unsigned long)__HAL_TIM_GET_COUNTER(&htim3));
            Debug_Print(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_PERIOD_MS));
    }
}

/* ========================================================================
 * Task_Button — 按键轮询
 * 周期: 10ms  |  优先级: 3
 *
 * 阶段1中仅做基础检测，后续阶段扩展按键功能
 * ======================================================================== */

static void Task_Button(void *argument)
{
    (void)argument;
    /* Button_Task 由 button.c 提供，在 FreeRTOS 任务中运行消抖逻辑 */
    Button_Task(argument);
}

/* ========================================================================
 * Task_Monitor — 系统健康监控 + 串口状态输出
 * 周期: 2000ms  |  优先级: 2
 *
 * 输出内容:
 *   - 系统运行时间
 *   - 各任务栈剩余水位（用于发现栈溢出风险）
 *   - 堆剩余空间
 *   - IMU 状态
 *   - 编码器读数
 * ======================================================================== */

static void Task_Monitor(void *argument)
{
    char buf[256];
    uint32_t last_lidar_frames = 0U;
    uint8_t lidar_stall_count = 0U;
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(3000));  /* 等所有任务启动稳定 */

    Debug_Print("\r\n========== System Monitor Started ==========\r\n");

    for (;;) {
        uint32_t uptime_s = (xTaskGetTickCount() - g_boot_tick)
                            * portTICK_PERIOD_MS / 1000U;

        /* 获取各任务栈高水位（剩余最小栈空间，单位 words） */
        UBaseType_t hw_imu    = g_task_imu_handle
            ? uxTaskGetStackHighWaterMark(g_task_imu_handle) : 0U;
        UBaseType_t hw_map    = g_task_mapping_handle
            ? uxTaskGetStackHighWaterMark(g_task_mapping_handle) : 0U;
        UBaseType_t hw_motor  = g_task_motor_handle
            ? uxTaskGetStackHighWaterMark(g_task_motor_handle) : 0U;
        UBaseType_t hw_btn    = g_task_button_handle
            ? uxTaskGetStackHighWaterMark(g_task_button_handle) : 0U;
        UBaseType_t hw_mon    = g_task_monitor_handle
            ? uxTaskGetStackHighWaterMark(g_task_monitor_handle) : 0U;
        UBaseType_t hw_oled   = g_task_oled_handle
            ? uxTaskGetStackHighWaterMark(g_task_oled_handle) : 0U;

        size_t free_heap = xPortGetFreeHeapSize();

        /* IMU 状态 */
        MPU6500_UpdateYaw(&g_mpu6500);
        float yaw = MPU6500_GetYaw(&g_mpu6500);

        /* 编码器 */
        int32_t enc_l = MotorDriver_GetEncoderCount(MOTOR_LEFT);
        int32_t enc_r = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
        uint32_t raw_t2 = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
        uint32_t raw_t3 = (uint32_t)__HAL_TIM_GET_COUNTER(&htim3);

        /* 雷达 DMA 统计 */
        LidarPipelineStats lidar_stats;
        LidarPipeline_GetStats(&lidar_stats);

        if (g_lidar_restart_requested != 0U) {
            Lidar_RestartScanIfNeeded("uart_error");
            last_lidar_frames = g_rplidar.frame_count;
            lidar_stall_count = 0U;
        } else if (lidar_stats.frame_count == last_lidar_frames) {
            if (lidar_stall_count < 255U) {
                lidar_stall_count++;
            }
            if (lidar_stall_count >= 2U) {
                g_lidar_uart_error_flags = HAL_UART_GetError(&huart3);
                Lidar_RestartScanIfNeeded("frame_stall");
                last_lidar_frames = g_rplidar.frame_count;
                lidar_stall_count = 0U;
            }
        } else {
            last_lidar_frames = lidar_stats.frame_count;
            lidar_stall_count = 0U;
        }

        snprintf(buf, sizeof(buf),
                 "%lus stk:I%luM%luv%luB%luN%luO%lu hp:%u "
                 "yaw:%.1f gz:%.2f e:%lu d:%u "
                 "enc:L%ldR%ld,T2=%lu,T3=%lu "
                 "ld:f%lu pe%lu rx%lu dp%lu "
                 "map:%lu/%u\r\n",
                 (unsigned long)uptime_s,
                 (unsigned long)hw_imu, (unsigned long)hw_map,
                 (unsigned long)hw_motor,
                 (unsigned long)hw_btn, (unsigned long)hw_mon,
                 (unsigned long)hw_oled,
                 (unsigned int)free_heap,
                 (double)yaw,
                 (double)g_mpu6500.gyro.dps_z,
                 (unsigned long)g_mpu6500.total_error_count,
                 (unsigned int)g_mpu_dma_busy,
                 (long)enc_l, (long)enc_r,
                 (unsigned long)raw_t2, (unsigned long)raw_t3,
                 (unsigned long)lidar_stats.frame_count,
                 (unsigned long)lidar_stats.parse_error_count,
                 (unsigned long)lidar_stats.uart_rx_event_count,
                 (unsigned long)lidar_stats.queue1_drop_count,
                 (unsigned long)NavGridMap_CountKnownFree(&g_grid_map),
                 (unsigned int)(NAV_MAP_SIZE * NAV_MAP_SIZE));
        Debug_Print(buf);

        /* 栈空间告警：剩余不足 64 words (256 bytes) 打印警告 */
        if (hw_imu   < 64U) Debug_Print("  [WARN] IMU stack low!\r\n");
        if (g_task_mapping_handle != NULL && hw_map < 64U) Debug_Print("  [WARN] Mapping stack low!\r\n");
        if (g_task_motor_handle != NULL && hw_motor  < 64U) Debug_Print("  [WARN] Motor stack low!\r\n");
        if (hw_btn   < 64U) Debug_Print("  [WARN] Button stack low!\r\n");
        if (hw_mon   < 64U) Debug_Print("  [WARN] Monitor stack low!\r\n");
        if (hw_oled  < 64U) Debug_Print("  [WARN] OLED stack low!\r\n");

        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}

/* ========================================================================
 * Task_OLED — OLED 显示更新
 * 周期: 200ms  |  优先级: 1（最低）
 *
 * 显示内容:
 *   Line 0: 标题 "FreeRTOS Phase 1"
 *   Line 1-2: 系统运行时间
 *   Line 3: IMU yaw
 *   Line 4-5: 编码器
 *   Line 6: 电机测试阶段
 *   Line 7: 堆剩余
 * ======================================================================== */

static void Task_OLED(void *argument)
{
    uint32_t uptime_s;
    float yaw;
    int32_t enc_l, enc_r;
    (void)argument;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(OLED_PERIOD_MS));

        if (!SSD1306_IsReady()) continue;

        uptime_s = (xTaskGetTickCount() - g_boot_tick)
                   * portTICK_PERIOD_MS / 1000U;
        MPU6500_UpdateYaw(&g_mpu6500);
        yaw   = MPU6500_GetYaw(&g_mpu6500);
        enc_l = MotorDriver_GetEncoderCount(MOTOR_LEFT);
        enc_r = MotorDriver_GetEncoderCount(MOTOR_RIGHT);

        SSD1306_Clear();

        SSD1306_Print(0, 0, "Phase 3 Mapping");
        SSD1306_printf(1, 0, "Uptime: %lus", (unsigned long)uptime_s);
        SSD1306_printf(2, 0, "Yaw: %.1f", (double)yaw);
        SSD1306_printf(3, 0, "Enc L:%ld", (long)enc_l);
        SSD1306_printf(4, 0, "Enc R:%ld", (long)enc_r);
        SSD1306_printf(5, 0, "Heap:%u", (unsigned int)xPortGetFreeHeapSize());
        SSD1306_printf(6, 0, "IMU err:%lu",
                       (unsigned long)g_mpu6500.total_error_count);
        SSD1306_Print(7, 0, "Status: OK");

        SSD1306_Update();
    }
}

/* ========================================================================
 * 按键回调 — 阶段1 保留接口，暂不处理
 * ======================================================================== */

static void Button_EventHandler(ButtonId id, ButtonEvent event)
{
    /* 阶段1: 仅响应不处理，后续阶段可在此添加功能 */
    (void)id;
    (void)event;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* FreeRTOS 要求所有优先级位用于抢占 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 中断优先级调整（必须在 FreeRTOS 系统调用优先级上限以下）---- */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);   /* ADC DMA */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);   /* I2C1 RX DMA */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn,       5, 0);   /* I2C1 事件 */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn,       5, 0);   /* I2C1 错误 */

  /* ---- I2C 总线互斥锁 ---- */
  I2C_MutexInit();

  /* ---- UART2 发送互斥锁 ---- */
  g_uart_mutex = xSemaphoreCreateMutex();

  /* ---- OLED 初始化 ---- */
  if (SSD1306_Init(&hi2c1)) {
      SSD1306_SetI2CLockCallbacks(I2C_Lock, I2C_Unlock);
      SSD1306_Clear();
      SSD1306_Print(0, 0, "Phase 1 Boot...");
      SSD1306_Update();
  } else {
      Debug_Print("[ERR] OLED init failed!\r\n");
  }

  /* ---- 按键初始化 ---- */
  Button_Init();
  Button_RegisterCallback(Button_EventHandler);

  /* ---- ADC DMA 启动 ---- */
  memset(adc_dma_buffer, 0, sizeof(adc_dma_buffer));
  if (hadc1.DMA_Handle != NULL) {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE);
  }

  /* ---- MPU6500 初始化 ---- */
  MPU6500_Init(&g_mpu6500, &hi2c1);
  MPU6500_SetI2CLockCallbacks(&g_mpu6500,
                               I2C_Lock, I2C_Unlock, I2C_UnlockFromISR);
  HAL_Delay(50);

  if (MPU6500_ReadWhoAmI(&g_mpu6500) == HAL_OK) {
      Debug_Print("[IMU] MPU6500 found!\r\n");
      if (MPU6500_Configure(&g_mpu6500) == HAL_OK) {
          Debug_Print("[IMU] MPU6500 configured OK\r\n");
      } else {
          Debug_Print("[IMU] MPU6500 configure FAILED!\r\n");
      }
  } else {
      Debug_Print("[IMU] MPU6500 NOT FOUND!\r\n");
  }

  /* DMP 初始化 */
  if (MPU6500_InitDMP(&g_mpu6500) == HAL_OK) {
      Debug_Print("[IMU] DMP init OK\r\n");
  } else {
      Debug_Print("[IMU] DMP init FAILED\r\n");
  }

  /* ---- 电机驱动初始化 ---- */
  MotorDriver_Init();
  Debug_Print("[MOTOR] Driver init OK\r\n");

  /* ---- 激光雷达初始化 (阶段2: DMA传输层) ---- */
  RPLidar_Init(&g_rplidar, &huart3);
  if (LidarPipeline_Start(&g_rplidar, Debug_Print)) {
      Debug_Print("[LIDAR] Pipeline started OK\r\n");
      /* 启动 DMA 循环接收 + 发送 SCAN 命令 */
      if (RPLidar_StartScanReception(&g_rplidar) == HAL_OK) {
          HAL_Delay(10);
          (void)RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_STOP);
          HAL_Delay(10);
          (void)RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_SCAN);
          Debug_Print("[LIDAR] SCAN command sent\r\n");
      } else {
          Debug_Print("[LIDAR] DMA start FAILED!\r\n");
      }
  } else {
      Debug_Print("[LIDAR] Pipeline start FAILED!\r\n");
  }

  /* ---- 记录启动时刻 ---- */
  g_boot_tick = xTaskGetTickCount();

  /* ================================================================
   * 创建 FreeRTOS 任务（阶段3 任务集）
   * ================================================================ */

  BaseType_t ret;
  char heap_msg[96];

  ret = xTaskCreate(Task_IMU, "IMU", STACK_IMU, NULL,
                    PRIO_IMU, &g_task_imu_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] IMU task create failed!\r\n");

#ifdef ENABLE_MAPPING_TEST
  ret = xTaskCreate(Task_Mapping, "Mapping", STACK_MAPPING, NULL,
                    PRIO_MOTOR, &g_task_mapping_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] Mapping task create failed!\r\n");
#else
  ret = xTaskCreate(Task_MotorTest, "Motor", STACK_MOTOR, NULL,
                    PRIO_MOTOR, &g_task_motor_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] Motor task create failed!\r\n");
#endif

  ret = xTaskCreate(Task_Button, "Button", STACK_BUTTON, NULL,
                    PRIO_BUTTON, &g_task_button_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] Button task create failed!\r\n");

  ret = xTaskCreate(Task_Monitor, "Monitor", STACK_MONITOR, NULL,
                    PRIO_MONITOR, &g_task_monitor_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] Monitor task create failed!\r\n");

  ret = xTaskCreate(Task_OLED, "OLED", STACK_OLED, NULL,
                    PRIO_OLED, &g_task_oled_handle);
  if (ret != pdPASS) Debug_Print("[FATAL] OLED task create failed!\r\n");

  snprintf(heap_msg, sizeof(heap_msg),
           "[RTOS] Free heap before scheduler: %u bytes\r\n",
           (unsigned int)xPortGetFreeHeapSize());
  Debug_Print(FW_DIAG_TAG);
  Debug_Print(heap_msg);
  Debug_Print("\r\n========== All tasks created ==========\r\n");
  Debug_Print("Starting FreeRTOS scheduler...\r\n\r\n");

  /* ---- 启动调度器 ---- */
  SSD1306_Clear();
  SSD1306_Print(0, 0, "Phase 1 Ready");
  SSD1306_Print(2, 0, "Starting RTOS...");
  SSD1306_Update();

  vTaskStartScheduler();

  /* 调度器启动失败才走到这里 */
  Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;  /* 降低到 50kHz 提高稳定性 */
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 179;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* PB7现在用作BIN2方向引脚，不配置为TIM4_CH2 */
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PB7 - 电机控制方向引脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ========================================================================
 * HAL 中断回调 — 阶段2: IMU + 雷达 DMA
 * ======================================================================== */

/* ---- I2C DMA 完成（MPU6500 陀螺仪）---- */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        MPU6500_OnGyroReadComplete(&g_mpu6500);
        g_mpu_dma_busy = 0U;
    }
}

/* ---- I2C 错误 ---- */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        MPU6500_UpdateErrorStatsFromISR(&g_mpu6500, HAL_ERROR);
        g_mpu_dma_busy = 0U;
        MPU6500_ResetGyroData(&g_mpu6500);
        I2C_UnlockFromISR();
    }
}

/* ---- 雷达 USART3 DMA 回调 ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3) {
        LidarPipeline_UartRxEventFromISR(Size);
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        LidarPipeline_DmaHalfCompleteFromISR();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        LidarPipeline_DmaCompleteFromISR();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        Lidar_RequestRestartFromISR(HAL_UART_GetError(huart));
    }
}

/* ---- ADC DMA 完成（环形模式，缓冲区自动刷新）---- */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    /* Circular DMA 自动刷新 adc_dma_buffer，无需额外处理 */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  Fault_PrintRaw("[FATAL] Error_Handler entered\r\n");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void vApplicationMallocFailedHook(void)
{
  Fault_PrintRaw("[FATAL] FreeRTOS malloc failed\r\n");
  __disable_irq();
  while (1)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  char msg[96];
  (void)xTask;

  if (pcTaskName == NULL)
  {
    Fault_PrintRaw("[FATAL] Stack overflow in <unknown>\r\n");
  }
  else
  {
    snprintf(msg, sizeof(msg), "[FATAL] Stack overflow in %s\r\n", pcTaskName);
    Fault_PrintRaw(msg);
  }

  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
