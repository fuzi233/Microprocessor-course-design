/*
 * motor_driver.c
 * 基于D157B模块的电机驱动 - PWM直接控制 + 编码器读取 + PID闭环控制
 */

#include "motor_driver.h"
#include <stdbool.h>
#include <stdlib.h>

/* 外部硬件句柄 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* ========== 内部变量 ========== */
static MotorState_t motor_states[2]; // 两个电机的状态

/* ========== 内部辅助函数声明 ========== */
static void apply_pwm_single_polarity(MotorID_t motor, int32_t pwm, bool forward);
static void set_motor_stop_hw(MotorID_t motor);
static int32_t pid_calculate(PIDController_t *pid, int32_t error);

/* ========== 内部辅助函数实现 ========== */

/**
 * @brief  应用PWM和方向控制到硬件 (单极性模式)
 */
static void apply_pwm_single_polarity(MotorID_t motor, int32_t pwm, bool forward)
{
    if (motor == MOTOR_LEFT)
    {
        // 左电机：AIN1=PA8(TIM1_CH1), AIN2=PC7
        // 硬件方向反了，软件中反转
        if (forward)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        }
    }
    else
    {
        // 右电机：BIN1=PB6(TIM4_CH1), BIN2=PB7
        if (forward)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        }
    }
}

/**
 * @brief  设置电机停止 (滑行模式)
 */
static void set_motor_stop_hw(MotorID_t motor)
{
    if (motor == MOTOR_LEFT || motor == MOTOR_BOTH)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    if (motor == MOTOR_RIGHT || motor == MOTOR_BOTH)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }
}

/**
 * @brief  PID控制器计算
 */
static int32_t pid_calculate(PIDController_t *pid, int32_t error)
{
    // 安全检查：如果误差过大，可能是异常情况，重置积分项
    if (error > 500 || error < -500)
    {
        pid->integral = 0;
    }
    
    // 计算P项
    float p_term = pid->Kp * error;
    
    // 计算I项并抗饱和
    pid->integral += error;
    // 更严格的积分项限制
    int32_t integral_max = (pid->output_max * 2) / (pid->Ki > 0.01f ? pid->Ki : 1.0f);
    if (pid->integral > integral_max) pid->integral = integral_max;
    if (pid->integral < -integral_max) pid->integral = -integral_max;
    float i_term = pid->Ki * pid->integral;
    
    // 计算D项
    float d_term = pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    
    // 计算总输出
    int32_t output = (int32_t)(p_term + i_term + d_term);
    
    // 限制输出范围
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;
    
    return output;
}

/* ========== 公开API实现 ========== */

void MotorDriver_Init(void)
{
    // 初始化GPIO状态
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // 先启动编码器定时器
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // 启动编码器定时器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
    
    // 多次清零确保编码器计数器真的为0
    for (int i = 0; i < 3; i++)
    {
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_Delay(1);
    }
    
    // 启动PWM定时器
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    
    // 初始化电机状态和PID控制器
    for (int i = 0; i < 2; i++)
    {
        // 重新读取编码器值来初始化
        if (i == MOTOR_LEFT)
            motor_states[i].last_encoder = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        else
            motor_states[i].last_encoder = -(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
            
        motor_states[i].current_speed = 0;
        motor_states[i].target_speed = 0;
        motor_states[i].current_pwm = 0;
        motor_states[i].last_error = 0;
        
        // PID参数初始化（更稳定的参数）
        motor_states[i].pid.Kp = 5.0f;
        motor_states[i].pid.Ki = 0.1f;
        motor_states[i].pid.Kd = 0.05f;
        motor_states[i].pid.integral = 0;
        motor_states[i].pid.last_error = 0;
        motor_states[i].pid.output_max = MOTOR_PWM_MAX;
    }
    
    // 停止电机
    set_motor_stop_hw(MOTOR_BOTH);
}

void MotorDriver_SetRawPWM(MotorID_t motor, int32_t pwm)
{
    // 如果是BOTH，分别设置两个电机
    if (motor == MOTOR_BOTH)
    {
        MotorDriver_SetRawPWM(MOTOR_LEFT, pwm);
        MotorDriver_SetRawPWM(MOTOR_RIGHT, pwm);
        return;
    }
    
    // 如果PWM为0，停止电机
    if (pwm == 0)
    {
        set_motor_stop_hw(motor);
        return;
    }
    
    // 限制PWM范围
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    
    // 应用PWM和方向
    bool forward = (pwm >= 0);
    int32_t abs_pwm = (pwm >= 0) ? pwm : -pwm;
    
    // 应用PWM
    apply_pwm_single_polarity(motor, abs_pwm, forward);
}

void MotorDriver_StopMotor(MotorID_t motor)
{
    set_motor_stop_hw(motor);
    // 同时重置PID状态
    if (motor == MOTOR_LEFT || motor == MOTOR_BOTH)
    {
        motor_states[MOTOR_LEFT].pid.integral = 0;
        motor_states[MOTOR_LEFT].pid.last_error = 0;
    }
    if (motor == MOTOR_RIGHT || motor == MOTOR_BOTH)
    {
        motor_states[MOTOR_RIGHT].pid.integral = 0;
        motor_states[MOTOR_RIGHT].pid.last_error = 0;
    }
}

int32_t MotorDriver_GetEncoderCount(MotorID_t motor)
{
    if (motor == MOTOR_LEFT)
    {
        // 左电机编码器：TIM3 (PA6, PA7)
        return (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    }
    else if (motor == MOTOR_RIGHT)
    {
        // 右电机编码器：TIM2 (PA15, PB3) - 取反以修正方向
        return -(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    } else {
        // BOTH模式，返回两个的平均值
        int32_t left = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
        int32_t right = -(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        return (left + right) / 2;
    }
}

void MotorDriver_ResetEncoder(MotorID_t motor)
{
    if (motor == MOTOR_LEFT || motor == MOTOR_BOTH) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        motor_states[MOTOR_LEFT].last_encoder = 0;
    }
    if (motor == MOTOR_RIGHT || motor == MOTOR_BOTH) {
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        motor_states[MOTOR_RIGHT].last_encoder = 0;
    }
}

/* ========== PID闭环控制API ========== */

void MotorDriver_SetTargetSpeed(MotorID_t motor, int32_t speed)
{
    if (motor == MOTOR_BOTH)
    {
        MotorDriver_SetTargetSpeed(MOTOR_LEFT, speed);
        MotorDriver_SetTargetSpeed(MOTOR_RIGHT, speed);
        return;
    }
    motor_states[motor].target_speed = speed;
}

int32_t MotorDriver_PIDControl(MotorID_t motor)
{
    if (motor == MOTOR_BOTH)
    {
        int32_t pwm_left = MotorDriver_PIDControl(MOTOR_LEFT);
        int32_t pwm_right = MotorDriver_PIDControl(MOTOR_RIGHT);
        return (pwm_left + pwm_right) / 2;
    }
    
    // 获取当前编码器值并计算速度
    int32_t current_encoder = MotorDriver_GetEncoderCount(motor);
    
    // 处理16位计数器可能的溢出（虽然TIM2/TIM3是32位，但保持安全）
    int32_t delta = current_encoder - motor_states[motor].last_encoder;
    
    // 如果增量异常大，可能是有问题，限制增量
    if (delta > 1000) delta = 1000;
    if (delta < -1000) delta = -1000;
    
    motor_states[motor].current_speed = delta;
    motor_states[motor].last_encoder = current_encoder;
    
    // 计算误差
    int32_t error = motor_states[motor].target_speed - delta;
    motor_states[motor].last_error = error;
    
    // PID计算
    int32_t pwm = pid_calculate(&motor_states[motor].pid, error);
    
    // 安全检查：PWM值限制
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    
    // 保存当前PWM值
    motor_states[motor].current_pwm = pwm;
    
    // 应用PWM
    MotorDriver_SetRawPWM(motor, pwm);
    
    return pwm;
}

int32_t MotorDriver_GetCurrentSpeed(MotorID_t motor)
{
    if (motor == MOTOR_BOTH)
    {
        return (motor_states[MOTOR_LEFT].current_speed + motor_states[MOTOR_RIGHT].current_speed) / 2;
    }
    return motor_states[motor].current_speed;
}

int32_t MotorDriver_GetCurrentPWM(MotorID_t motor)
{
    if (motor == MOTOR_BOTH)
    {
        return (motor_states[MOTOR_LEFT].current_pwm + motor_states[MOTOR_RIGHT].current_pwm) / 2;
    }
    return motor_states[motor].current_pwm;
}

int MotorDriver_GetPIDState(MotorID_t motor, PIDController_t *pid_out)
{
    if (motor == MOTOR_BOTH || !pid_out)
        return -1;
    
    *pid_out = motor_states[motor].pid;
    return 0;
}

int MotorDriver_GetMotorState(MotorID_t motor, MotorState_t *state_out)
{
    if (motor == MOTOR_BOTH || !state_out)
        return -1;
    
    *state_out = motor_states[motor];
    return 0;
}
