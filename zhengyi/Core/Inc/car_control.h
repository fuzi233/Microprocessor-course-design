/*
 * car_control.h
 * 小车控制封装层 - 提供高级运动控制接口
 * 封装电机、编码器、MPU6500功能
 */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "motor_driver.h"
#include "mpu6500.h"

#define CAR_ENCODER_TICKS_PER_CM       100.0f
#define CAR_WHEEL_BASE_CM              15.0f
#define CAR_DEFAULT_SPEED              4000
#define CAR_ROTATION_SPEED             2000
#define CAR_CONTROL_PERIOD_MS          10
#define CAR_YAW_KP                     2.0f
#define CAR_YAW_KI                     0.0f
#define CAR_YAW_KD                     0.1f
#define CAR_DISTANCE_TOLERANCE_CM      2.0f
#define CAR_ANGLE_TOLERANCE_DEG        3.0f

typedef enum {
    CAR_STATE_IDLE = 0,
    CAR_STATE_MOVING_FORWARD,
    CAR_STATE_MOVING_BACKWARD,
    CAR_STATE_ROTATING,
    CAR_STATE_STOPPED
} CarState_t;

typedef enum {
    CAR_CMD_NONE = 0,
    CAR_CMD_MOVE_DISTANCE,
    CAR_CMD_ROTATE_ANGLE,
    CAR_CMD_STOP
} CarCommand_t;

typedef struct {
    float target_distance_cm;
    float traveled_distance_cm;
    float target_angle_deg;
    float start_angle_deg;
    int32_t target_speed;
    CarState_t state;
    CarCommand_t command;
    bool command_complete;
    bool use_heading_control;
    float target_heading;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float yaw_integral;
    float last_yaw_error;
    uint32_t control_period_ms;
} CarContext_t;

typedef void (*CarDebugPrintFn)(const char *message);

void Car_Init(CarContext_t *ctx, MPU6500Context *mpu_ctx);
void Car_SetDebugPrint(CarDebugPrintFn print_fn);
void Car_SetSpeed(CarContext_t *ctx, int32_t speed_ticks_per_sec);
void Car_SetHeadingPID(CarContext_t *ctx, float kp, float ki, float kd);
bool Car_MoveForward(CarContext_t *ctx, float distance_cm);
bool Car_MoveBackward(CarContext_t *ctx, float distance_cm);
bool Car_Rotate(CarContext_t *ctx, float angle_deg);
void Car_Stop(CarContext_t *ctx);
void Car_EmergencyStop(void);
bool Car_IsCommandComplete(CarContext_t *ctx);
CarState_t Car_GetState(CarContext_t *ctx);
void Car_Update(CarContext_t *ctx);
float Car_GetTraveledDistance(CarContext_t *ctx);
float Car_GetCurrentHeading(CarContext_t *ctx);
int32_t Car_GetEncoderAverage(void);
void Car_ResetEncoders(void);

#ifdef __cplusplus
}
#endif

#endif
