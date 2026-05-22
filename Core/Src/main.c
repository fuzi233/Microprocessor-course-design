/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "mpu6500.h"
#include "rplidar.h"
#include "oled_ssd1306.h"
#include "button.h"
#include "motor_driver.h"
#include "lidar_pipeline.h"
#include "car_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 任务优先级 (0=idle, 7=最高) */
#define PRIO_IMU            4U
#define PRIO_BUTTON         3U
#define PRIO_OLED           1U

/* 任务栈大小 (words) */
#define STACK_IMU           512U
#define STACK_BUTTON        512U
#define STACK_OLED          512U

/* ADC 缓冲区 (三路电位器) */
#define ADC_BUFFER_SIZE  3

/* 传感器轮询周期 */
#define IMU_POLL_PERIOD_MS  20U

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
static MPU6500Context g_mpu6500;
static RPLidarContext g_rplidar;
static CarContext_t g_car;
static volatile uint8_t g_mpu_dma_busy = 0U;
static volatile uint8_t g_lidar_rx_active = 0U;

/* I2C 总线互斥信号量（OLED 和 MPU6500 共用 I2C1） */
static SemaphoreHandle_t g_i2c_mutex = NULL;

uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

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
/* ========== 任务函数声明 ========== */
static void Debug_Print(const char *message);
static void Debug_PrintOLEDStatus(void);
static void Button_EventHandler(ButtonId id, ButtonEvent event);
static void IMU_Update_Task(void *argument);
static void OLED_Update_Task(void *argument);
static void PIDControl_Task(void *argument);
static void EncoderOutput_Task(void *argument);
static void YawOutput_Task(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 全局变量：控制阶段
static uint8_t phase = 0;

static int32_t prev_left_enc = 0;
static int32_t prev_right_enc = 0;
static uint32_t prev_time = 0;

static uint32_t last_mpu_data_time = 0;
static uint32_t last_mpu_dma_start_time = 0;
static bool mpu_dma_started = false;

static void Debug_Print(const char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)strlen(message), 100U);
}

static void Debug_PrintOLEDStatus(void)
{
    char msg[96];
    HAL_StatusTypeDef probe_3c;
    HAL_StatusTypeDef probe_3d;

    probe_3c = HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR_PRIMARY, 1U, 20U);
    probe_3d = HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR_SECONDARY, 1U, 20U);

    snprintf(msg, sizeof(msg),
             "OLED diag: ready=%d addr=0x%02X probe3C=%d probe3D=%d i2c_err=0x%08lX state=%lu\r\n",
             SSD1306_IsReady() ? 1 : 0,
             (unsigned int)(SSD1306_GetI2CAddress() >> 1),
             (int)probe_3c,
             (int)probe_3d,
             (unsigned long)HAL_I2C_GetError(&hi2c1),
             (unsigned long)HAL_I2C_GetState(&hi2c1));
    Debug_Print(msg);
}

/* ========== I2C 总线互斥锁 API ========== */

/* 获取 I2C 总线访问权（阻塞等待） */
static void I2C_Lock(void)
{
    if (g_i2c_mutex != NULL)
    {
        xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    }
}

/* 释放 I2C 总线访问权（任务上下文使用） */
static void I2C_Unlock(void)
{
    if (g_i2c_mutex != NULL)
    {
        xSemaphoreGive(g_i2c_mutex);
    }
}

/* 释放 I2C 总线访问权（ISR 上下文使用，如 DMA 回调） */
static void I2C_UnlockFromISR(void)
{
    if (g_i2c_mutex != NULL)
    {
        (void)xSemaphoreGiveFromISR(g_i2c_mutex, pdFALSE);
    }
}

/* 初始化 I2C 互斥信号量 */
static void I2C_MutexInit(void)
{
    g_i2c_mutex = xSemaphoreCreateMutex();
}

static void IMU_Update_Task(void *argument)
{
    TickType_t last_wake;
    uint32_t current_time;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        HAL_StatusTypeDef status;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_POLL_PERIOD_MS));
        
        current_time = HAL_GetTick();

        // 检查 DMA 是否超时（500ms），超时则强制重置
        if (g_mpu_dma_busy != 0U && mpu_dma_started)
        {
            if ((current_time - last_mpu_dma_start_time) > 500U)
            {
                // 超时，尝试恢复
                g_mpu_dma_busy = 0U;
                g_mpu6500.total_error_count++;
                // 可以在这里添加 I2C 复位代码
                Debug_Print("IMU DMA timeout, resetting...\r\n");
            }
            else
            {
                // 还在等待，继续
                continue;
            }
        }

        if (!g_mpu6500.gyro.who_am_i_ok)
        {
            continue;
        }

        /* P1修复：先启动DMA，成功后再清除data_ready标志
         * 避免在清除标志和启动DMA之间的窗口期内，
         * 上一次DMA回调设置data_ready=true导致数据被错误使用 */
        status = MPU6500_StartGyroReadDMA(&g_mpu6500);
        if (status == HAL_OK)
        {
            g_mpu6500.gyro.data_ready = false;  /* 成功启动后清除旧标志 */
            g_mpu_dma_busy = 1U;
            last_mpu_dma_start_time = current_time;
            mpu_dma_started = true;  /* 标记已启动过DMA */
        }
        else if (status != HAL_BUSY)
        {
            g_mpu6500.total_error_count++;
        }
    }
}

/* PID控制任务 - 10ms周期控制电机速度 */
static void PIDControl_Task(void *argument)
{
    (void)argument;
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint32_t phase_time = 0;
    uint8_t phase = 0;
    
    for (;;)
    {
        if (phase == 0)
        {
            if (phase_time == 0)
            {
                int32_t target_per_period = CAR_DEFAULT_SPEED * CAR_CONTROL_PERIOD_MS / 1000;
                MotorDriver_SetTargetSpeed(MOTOR_BOTH, target_per_period);
                
                if (g_mpu6500.gyro.who_am_i_ok)
                {
                    MPU6500_UpdateYaw(&g_mpu6500);
                    g_car.target_heading = MPU6500_GetYaw(&g_mpu6500);
                    g_car.yaw_integral = 0.0f;
                    g_car.last_yaw_error = 0.0f;
                }
            }
            
            if (g_mpu6500.gyro.who_am_i_ok && g_mpu6500.gyro.data_ready)
            {
                MPU6500_UpdateYaw(&g_mpu6500);
                float current_yaw = MPU6500_GetYaw(&g_mpu6500);
                float yaw_error = g_car.target_heading - current_yaw;
                
                if (yaw_error > 180.0f) yaw_error -= 360.0f;
                if (yaw_error < -180.0f) yaw_error += 360.0f;
                
                float dt = CAR_CONTROL_PERIOD_MS / 1000.0f;
                g_car.yaw_integral += yaw_error * dt;
                float derivative = (yaw_error - g_car.last_yaw_error) / dt;
                g_car.last_yaw_error = yaw_error;
                
                float yaw_output = g_car.yaw_kp * yaw_error + g_car.yaw_ki * g_car.yaw_integral + g_car.yaw_kd * derivative;
                
                if (yaw_output > 1000.0f) yaw_output = 1000.0f;
                if (yaw_output < -1000.0f) yaw_output = -1000.0f;
                
                int32_t base_speed = CAR_DEFAULT_SPEED * CAR_CONTROL_PERIOD_MS / 1000;
                int32_t left_speed = base_speed + (int32_t)yaw_output;
                int32_t right_speed = base_speed - (int32_t)yaw_output;
                
                if (left_speed > 5000) left_speed = 5000;
                if (left_speed < 0) left_speed = 0;
                if (right_speed > 5000) right_speed = 5000;
                if (right_speed < 0) right_speed = 0;
                
                MotorDriver_SetTargetSpeed(MOTOR_LEFT, left_speed);
                MotorDriver_SetTargetSpeed(MOTOR_RIGHT, right_speed);
            }
            
            MotorDriver_PIDControl(MOTOR_BOTH);
        }
        else
        {
            MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
            MotorDriver_StopMotor(MOTOR_BOTH);
        }
        
        phase_time += PID_CONTROL_PERIOD_MS;
        
        if ((phase == 0 && phase_time >= 2000) || (phase == 1 && phase_time >= 4000))
        {
            phase = 1 - phase;
            phase_time = 0;
            
            if (phase == 0)
            {
                MotorDriver_ResetEncoder(MOTOR_BOTH);
                if (g_mpu6500.gyro.who_am_i_ok)
                {
                    MPU6500_UpdateYaw(&g_mpu6500);
                    g_car.target_heading = MPU6500_GetYaw(&g_mpu6500);
                    g_car.yaw_integral = 0.0f;
                    g_car.last_yaw_error = 0.0f;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(PID_CONTROL_PERIOD_MS));
    }
}

/* 编码器输出任务 - 每秒输出详细信息 */
static void EncoderOutput_Task(void *argument)
{
    (void)argument;
    char buffer[512];
    
    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(2200)); // 等待 PID 任务完成重置
    
    // 先强制重置编码器，确保清零
    MotorDriver_ResetEncoder(MOTOR_BOTH);
    
    // 记录启动时的编码器值（等一小会确保稳定）
    vTaskDelay(pdMS_TO_TICKS(100));
    int32_t start_left_enc = MotorDriver_GetEncoderCount(MOTOR_LEFT);
    int32_t start_right_enc = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
    
    // 检查并修正异常初始值
    if (abs(start_left_enc) > 10000) start_left_enc = 0;
    if (abs(start_right_enc) > 10000) start_right_enc = 0;
    
    prev_left_enc = start_left_enc;
    prev_right_enc = start_right_enc;
    prev_time = xTaskGetTickCount();
    
    for (;;)
    {
        int32_t left_enc = MotorDriver_GetEncoderCount(MOTOR_LEFT);
        int32_t right_enc = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
        
        // 修正编码器跳变（防止溢出或异常大值）
        if (abs(left_enc - prev_left_enc) > 5000)
        {
            left_enc = prev_left_enc;
        }
        if (abs(right_enc - prev_right_enc) > 5000)
        {
            right_enc = prev_right_enc;
        }
        
        // 获取电机当前PWM
        int32_t left_pwm = MotorDriver_GetCurrentPWM(MOTOR_LEFT);
        int32_t right_pwm = MotorDriver_GetCurrentPWM(MOTOR_RIGHT);
        
        // 获取当前速度（控制周期内的刻度）
        int32_t left_speed_cycle = MotorDriver_GetCurrentSpeed(MOTOR_LEFT);
        int32_t right_speed_cycle = MotorDriver_GetCurrentSpeed(MOTOR_RIGHT);
        
        // 计算从启动以来的总增量
        int32_t left_total = left_enc - start_left_enc;
        int32_t right_total = right_enc - start_right_enc;
        
        // 更准确的每秒速度计算（基于实际时间差）
        uint32_t current_time = xTaskGetTickCount();
        uint32_t time_diff_ms = (current_time - prev_time) * portTICK_PERIOD_MS;
        
        int32_t left_speed_per_sec = 0;
        int32_t right_speed_per_sec = 0;
        
        if (time_diff_ms > 0)
        {
            left_speed_per_sec = ((left_enc - prev_left_enc) * 1000) / time_diff_ms;
            right_speed_per_sec = ((right_enc - prev_right_enc) * 1000) / time_diff_ms;
            
            // 限制速度输出范围，防止异常值
            if (abs(left_speed_per_sec) > 10000) left_speed_per_sec = 0;
            if (abs(right_speed_per_sec) > 10000) right_speed_per_sec = 0;
        }
        
        prev_left_enc = left_enc;
        prev_right_enc = right_enc;
        prev_time = current_time;
        
        snprintf(buffer, sizeof(buffer), 
                 "[%s] Enc: L=%6ld R=%6ld | Total: L=%6ld R=%6ld | "
                 "Spd(sec): L=%5ld R=%5ld | Spd(cycle): L=%5ld R=%5ld | "
                 "PWM: L=%5ld R=%5ld\r\n", 
                 (phase == 0) ? "MOVING" : "STOP  ",
                 (long)left_enc, (long)right_enc,
                 (long)left_total, (long)right_total,
                 (long)left_speed_per_sec, (long)right_speed_per_sec,
                 (long)left_speed_cycle, (long)right_speed_cycle,
                 (long)left_pwm, (long)right_pwm);
        
        // 通过串口输出
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100U);
        
        // 每秒输出一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void YawOutput_Task(void *argument)
{
    (void)argument;
    char buffer[256];
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    for (;;)
    {
        MPU6500_UpdateYaw(&g_mpu6500);
        
        float current_yaw = MPU6500_GetYaw(&g_mpu6500);
        float yaw_error = g_car.target_heading - current_yaw;
        if (yaw_error > 180.0f) yaw_error -= 360.0f;
        if (yaw_error < -180.0f) yaw_error += 360.0f;
        
        snprintf(buffer, sizeof(buffer), 
                 "[YAW] Cur:%.1f Tgt:%.1f Err:%.1f | "
                 "Int:%.1f | GyroZ:%.1f\r\n", 
                 (double)current_yaw, 
                 (double)g_car.target_heading, 
                 (double)yaw_error,
                 (double)g_car.yaw_integral,
                 (double)g_mpu6500.gyro.dps_z);
        
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100U);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void HighFreqDebug_Task(void *argument)
{
    (void)argument;
    char buffer[512];
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    for (;;)
    {
        MotorState_t left_state, right_state;
        MotorDriver_GetMotorState(MOTOR_LEFT, &left_state);
        MotorDriver_GetMotorState(MOTOR_RIGHT, &right_state);
        
        MPU6500_UpdateYaw(&g_mpu6500);
        float current_yaw = MPU6500_GetYaw(&g_mpu6500);
        
        snprintf(buffer, sizeof(buffer),
                 "[DBG] Phase:%s | "
                 "L:Tgt=%d Cur=%d PWM=%d Err=%d | "
                 "R:Tgt=%d Cur=%d PWM=%d Err=%d | "
                 "Yaw:Cur=%.1f Tgt=%.1f\r\n",
                 (phase == 0) ? "MOV" : "STOP",
                 (int)left_state.target_speed, (int)left_state.current_speed,
                 (int)left_state.current_pwm, (int)left_state.last_error,
                 (int)right_state.target_speed, (int)right_state.current_speed,
                 (int)right_state.current_pwm, (int)right_state.last_error,
                 (double)current_yaw, (double)g_car.target_heading);
        
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100U);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* MPU6500 诊断输出任务 - 5秒周期输出诊断信息 */
static void MPUDiagnostic_Task(void *argument)
{
    (void)argument;
    
    // 等待系统初始化完成
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    for (;;)
    {
        // 输出 MPU6500 诊断信息
        MPU6500_DiagnosticPrint(&g_mpu6500, Debug_Print);
        
        // 5秒输出一次
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


/* 按键回调 - 测试期间暂时禁用，避免干扰 */
static void Button_EventHandler(ButtonId id, ButtonEvent event)
{
    (void)id;
    (void)event;
    // 测试期间不处理按键
}

/* OLED 显示更新任务 */
static void OLED_Update_Task(void *argument)
{
    uint32_t now;
    int32_t enc_l, enc_r, pwm_l, pwm_r;
    int32_t speed_l, speed_r;
    float dist_cm, heading;
    CarState_t state;
    bool mpu_ok, lidar_ok;
    const char *state_str;

    (void)argument;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(200));

        if (!SSD1306_IsReady())
        {
            continue;
        }

        now = HAL_GetTick();

        /* ---- 采集数据 ---- */
        enc_l   = MotorDriver_GetEncoderCount(MOTOR_LEFT);
        enc_r   = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
        speed_l = MotorDriver_GetCurrentSpeed(MOTOR_LEFT) * 100;  /* ticks/周期 → ticks/s */
        speed_r = MotorDriver_GetCurrentSpeed(MOTOR_RIGHT) * 100;
        pwm_l   = MotorDriver_GetCurrentPWM(MOTOR_LEFT);
        pwm_r   = MotorDriver_GetCurrentPWM(MOTOR_RIGHT);
        state   = Car_GetState(&g_car);
        dist_cm = Car_GetTraveledDistance(&g_car);
        heading = Car_GetCurrentHeading(&g_car);
        mpu_ok  = g_mpu6500.gyro.who_am_i_ok;
        lidar_ok = (g_lidar_rx_active != 0U);

        /* ---- 状态文字 ---- */
        switch (state) {
        case CAR_STATE_IDLE:           state_str = "IDLE";   break;
        case CAR_STATE_MOVING_FORWARD: state_str = "FWD";    break;
        case CAR_STATE_MOVING_BACKWARD:state_str = "BWD";    break;
        case CAR_STATE_ROTATING:       state_str = "ROT";    break;
        case CAR_STATE_STOPPED:        state_str = "STOP";   break;
        default:                       state_str = "???";    break;
        }

        /* ---- 绘制 ---- */
        SSD1306_Clear();

        /* 第0行: 标题 + 状态 + 传感器图标 */
        SSD1306_printf(0, 0, "AMR %4s %c%c",
                       state_str,
                       mpu_ok  ? 'I' : '-',   /* I=IMU可用 */
                       lidar_ok ? 'L' : '-');  /* L=LiDAR可用 */

        /* 第1行: 编码器 */
        SSD1306_printf(1, 0, "E %+5ld %+5ld", (long)enc_l, (long)enc_r);

        /* 第2行: 速度 (ticks/s) */
        SSD1306_printf(2, 0, "V %+5ld %+5ld", (long)speed_l, (long)speed_r);

        /* 第3行: PWM 输出 */
        SSD1306_printf(3, 0, "P %+4ld %+4ld", (long)pwm_l, (long)pwm_r);

        /* 第4行: 行驶距离 */
        SSD1306_printf(4, 0, "D %+6.1f cm", (double)dist_cm);

        /* 第5行: 航向角 */
        if (mpu_ok)
            SSD1306_printf(5, 0, "Y %+6.1f deg", (double)heading);
        else
            SSD1306_Print(5, 0, "Y -- IMU --");

        /* 第6行: 目标速度 + LiDAR状态 */
        SSD1306_printf(6, 0, "Tg %4d L=%d",
                       TARGET_TICKS_PER_SECOND, lidar_ok ? 1 : 0);

        /* 第7行: 运行时间 */
        SSD1306_printf(7, 0, "T %3lu s", (unsigned long)(now / 1000));

        SSD1306_Update();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 对于 STM32F4xx，PA15 可以直接配置为 TIM2_CH1，
   * 只要在 TIM_Encoder_MspInit 中正确配置 GPIO 复用功能即可，
   * 不需要特殊的 remap 函数 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* FreeRTOS expects all priority bits to be used for preemption. */
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

  /* Keep ADC DMA IRQ at or below the FreeRTOS syscall priority ceiling. */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);

  /* 设置 I2C 和相关 DMA 中断优先级，确保不超过 FreeRTOS 优先级上限 */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);  // I2C1 RX DMA
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);        // I2C1 事件中断
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);        // I2C1 错误中断

  /* ---- 初始化 I2C 总线互斥锁（OLED 和 MPU6500 共用 I2C1）---- */
  I2C_MutexInit();

  /* ---- 初始化OLED ---- */
  if (SSD1306_Init(&hi2c1))
  {
    /* 设置 OLED 的 I2C 锁回调 */
    SSD1306_SetI2CLockCallbacks(I2C_Lock, I2C_Unlock);
    SSD1306_Clear();
    SSD1306_Print(0, 0, "AMR Boot...");
    SSD1306_Update();
  }

  /* ---- 初始化按键 ---- */
  Button_Init();
  Button_RegisterCallback(Button_EventHandler);

  /* ---- 启动ADC DMA ---- */
  adc_dma_buffer[0] = 0;
  adc_dma_buffer[1] = 0;
  adc_dma_buffer[2] = 0;

  if (hadc1.DMA_Handle != NULL)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUFFER_SIZE);
  }

  /* ---- 初始化MPU6500 ---- */
  MPU6500_Init(&g_mpu6500, &hi2c1);
  /* 设置 I2C 总线锁回调（OLED 和 MPU6500 共用 I2C1） */
  MPU6500_SetI2CLockCallbacks(&g_mpu6500, I2C_Lock, I2C_Unlock, I2C_UnlockFromISR);
  HAL_Delay(50);
  // 先检测地址
  if (MPU6500_ReadWhoAmI(&g_mpu6500) == HAL_OK)
  {
    Debug_Print("MPU6500 found!\r\n");
    // 再配置
    if (MPU6500_Configure(&g_mpu6500) == HAL_OK)
    {
      Debug_Print("MPU6500 configured!\r\n");
    }
    else
    {
      Debug_Print("MPU6500 configure failed!\r\n");
    }
  }
  else
  {
    Debug_Print("MPU6500 not found!\r\n");
  }
  
  /* ---- 初始化DMP（数字运动处理器）---- */
  if (MPU6500_InitDMP(&g_mpu6500) == HAL_OK)
  {
    Debug_Print("MPU6500 DMP initialized successfully\r\n");
  }
  else
  {
    Debug_Print("MPU6500 DMP initialization failed\r\n");
  }

  /* ---- 初始化电机驱动 ---- */
  MotorDriver_Init();
  
  /* ---- 初始化小车控制模块 ---- */
  Car_Init(&g_car, &g_mpu6500);

  /* ---- 启动LiDAR Pipeline ---- */
  RPLidar_Init(&g_rplidar, &huart3);
  LidarPipeline_Start(&g_rplidar, Debug_Print);

  if (RPLidar_StartScanReception(&g_rplidar) == HAL_OK)
  {
    g_lidar_rx_active = 1U;
    HAL_Delay(10);
    (void)RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_STOP);
    HAL_Delay(10);
    (void)RPLidar_SendCommand(&g_rplidar, RPLIDAR_CMD_SCAN);
  }

  /* ---- 创建FreeRTOS任务 ---- */

  /* PID控制任务 - 优先级较高 */
  xTaskCreate(PIDControl_Task, "PIDControl", STACK_BUTTON, NULL,
              PRIO_BUTTON, NULL);

  /* IMU 周期采样 */
  xTaskCreate(IMU_Update_Task, "IMU", STACK_IMU, NULL,
              PRIO_IMU, NULL);

  /* 按键消抖 */
  xTaskCreate(Button_Task, "Button", STACK_BUTTON, NULL,
              PRIO_BUTTON, NULL);

  /* OLED 显示更新 */
  xTaskCreate(OLED_Update_Task, "OLED", STACK_OLED, NULL,
              PRIO_OLED, NULL);
              
  /* 编码器输出任务 */
  xTaskCreate(EncoderOutput_Task, "EncoderOut", STACK_BUTTON, NULL,
              PRIO_OLED, NULL);
              
  /* 偏航角输出任务 */
    xTaskCreate(YawOutput_Task, "YawOutput", STACK_BUTTON, NULL,
                PRIO_OLED, NULL);
                
    /* 高频调试输出任务 - 暂时禁用，排查问题
    xTaskCreate(HighFreqDebug_Task, "HiFreqDebug", STACK_BUTTON, NULL,
                PRIO_OLED, NULL);
    */
    
    /* MPU6500 诊断输出任务 */
    xTaskCreate(MPUDiagnostic_Task, "MPUDiag", STACK_BUTTON, NULL,
                PRIO_OLED, NULL);

  /* ---- 启动调度器 ---- */
  SSD1306_Print(0, 0, "AMR Ready");
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

/* ---- UART / LiDAR 回调 (ISR上下文) ---- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    if (Size > 0U)
    {
      LidarPipeline_UartRxEventFromISR(Size);
    }
    g_lidar_rx_active = 1U;
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    LidarPipeline_DmaHalfCompleteFromISR();
    g_lidar_rx_active = 1U;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    LidarPipeline_DmaCompleteFromISR();
    g_lidar_rx_active = 1U;
  }
}

/* ---- I2C / MPU6500 回调 ---- */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    MPU6500_OnGyroReadComplete(&g_mpu6500);
    g_mpu_dma_busy = 0U;
    last_mpu_data_time = HAL_GetTick();  // 记录最后一次收到数据的时间
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    /* P3: 分类统计错误类型 */
    HAL_StatusTypeDef err_status = HAL_ERROR;
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_AF))
    {
      err_status = HAL_ERROR;  // NACK
    }

    MPU6500_UpdateErrorStatsFromISR(&g_mpu6500, err_status);

    g_mpu_dma_busy = 0U;

    /* 错误时清零陀螺仪数据，避免使用旧数据 */
    MPU6500_ResetGyroData(&g_mpu6500);

    /* P0: 错误回调中释放 I2C 总线锁（ISR 上下文） */
    I2C_UnlockFromISR();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    g_lidar_rx_active = 0U;
  }
}

/* ---- ADC DMA完成回调 ---- */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    /* Circular DMA already refreshes adc_dma_buffer in place. */
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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

