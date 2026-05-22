/*
 * motor_driver.h
 * 基于D157B模块的电机驱动 - PWM直接控制 + 编码器读取 + PID闭环控制
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========== 配置参数 ========== */

// PWM配置
#define MOTOR_PWM_MAX               999  // 对应ARR=999

// PID控制配置
#define PID_CONTROL_PERIOD_MS       10   // PID控制周期10ms
#define TARGET_TICKS_PER_SECOND     4000 // 目标速度：每秒4000个编码器刻度
#define TARGET_TICKS_PER_PERIOD     (TARGET_TICKS_PER_SECOND * PID_CONTROL_PERIOD_MS / 1000) // 每个控制周期的目标刻度

/* ========== 类型定义 ========== */

// 电机编号
typedef enum {
    MOTOR_LEFT = 0,   // 左电机
    MOTOR_RIGHT = 1,  // 右电机
    MOTOR_BOTH = 2    // 两个电机
} MotorID_t;

// PID控制器结构体
typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    int32_t integral; // 积分项
    int32_t last_error; // 上次误差
    int32_t output_max; // 输出最大值
} PIDController_t;

// 电机状态结构体
typedef struct {
    int32_t last_encoder; // 上次编码器读数
    int32_t current_speed; // 当前速度（每PID周期的刻度数）
    int32_t target_speed;  // 目标速度
    int32_t current_pwm;   // 当前PWM输出值
    int32_t last_error;    // 上次速度误差
    PIDController_t pid;   // PID控制器
} MotorState_t;

/* ========== 函数声明 ========== */

/* 初始化 */
void MotorDriver_Init(void);

/* ========== 电机控制API ========== */

/**
 * @brief  直接设置电机PWM
 * @param  motor: 电机编号
 * @param  pwm: PWM值，范围 -MOTOR_PWM_MAX 到 +MOTOR_PWM_MAX
 *              正数=前进，负数=后退，0=停止
 */
void MotorDriver_SetRawPWM(MotorID_t motor, int32_t pwm);

/**
 * @brief  停止电机
 * @param  motor: 电机编号
 */
void MotorDriver_StopMotor(MotorID_t motor);

/* ========== 编码器API ========== */

/**
 * @brief  获取编码器计数
 * @param  motor: 电机编号
 * @return 编码器计数值
 */
int32_t MotorDriver_GetEncoderCount(MotorID_t motor);

/**
 * @brief  重置编码器计数
 * @param  motor: 电机编号
 */
void MotorDriver_ResetEncoder(MotorID_t motor);

/* ========== PID闭环控制API ========== */

/**
 * @brief  设置目标速度
 * @param  motor: 电机编号
 * @param  speed: 目标速度（每PID周期的刻度数）
 */
void MotorDriver_SetTargetSpeed(MotorID_t motor, int32_t speed);

/**
 * @brief  运行一次PID控制
 * @param  motor: 电机编号
 * @return 计算出的PWM值
 */
int32_t MotorDriver_PIDControl(MotorID_t motor);

/**
 * @brief  获取当前速度
 * @param  motor: 电机编号
 * @return 当前速度（每PID周期的刻度数）
 */
int32_t MotorDriver_GetCurrentSpeed(MotorID_t motor);

/**
 * @brief  获取当前PWM值
 * @param  motor: 电机编号
 * @return 当前PWM输出值
 */
int32_t MotorDriver_GetCurrentPWM(MotorID_t motor);

/**
 * @brief  获取PID控制器状态
 * @param  motor: 电机编号
 * @param  pid_out: 输出PID控制器状态的指针
 * @return 成功返回0
 */
int MotorDriver_GetPIDState(MotorID_t motor, PIDController_t *pid_out);

/**
 * @brief  获取电机完整状态
 * @param  motor: 电机编号
 * @param  state_out: 输出电机状态的指针
 * @return 成功返回0
 */
int MotorDriver_GetMotorState(MotorID_t motor, MotorState_t *state_out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
