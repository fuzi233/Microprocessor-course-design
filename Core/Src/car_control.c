/*
 * car_control.c
 * 小车控制封装层实现
 */

#include "car_control.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static MPU6500Context *g_mpu_ctx = NULL;
static CarDebugPrintFn g_debug_print = NULL;

void Car_Init(CarContext_t *ctx, MPU6500Context *mpu_ctx)
{
    memset(ctx, 0, sizeof(CarContext_t));
    g_mpu_ctx = mpu_ctx;
    
    ctx->state = CAR_STATE_IDLE;
    ctx->command = CAR_CMD_NONE;
    ctx->target_speed = CAR_DEFAULT_SPEED;
    ctx->control_period_ms = CAR_CONTROL_PERIOD_MS;
    ctx->yaw_kp = CAR_YAW_KP;
    ctx->yaw_ki = CAR_YAW_KI;
    ctx->yaw_kd = CAR_YAW_KD;
    ctx->use_heading_control = true;
    ctx->command_complete = true;
    
    MotorDriver_Init();
    MotorDriver_ResetEncoder(MOTOR_BOTH);
}

void Car_SetDebugPrint(CarDebugPrintFn print_fn)
{
    g_debug_print = print_fn;
}

void Car_SetSpeed(CarContext_t *ctx, int32_t speed_ticks_per_sec)
{
    ctx->target_speed = speed_ticks_per_sec;
}

void Car_SetHeadingPID(CarContext_t *ctx, float kp, float ki, float kd)
{
    ctx->yaw_kp = kp;
    ctx->yaw_ki = ki;
    ctx->yaw_kd = kd;
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

bool Car_MoveForward(CarContext_t *ctx, float distance_cm)
{
    if (ctx->state != CAR_STATE_IDLE && ctx->state != CAR_STATE_STOPPED)
    {
        return false;
    }
    
    MotorDriver_ResetEncoder(MOTOR_BOTH);
    
    ctx->target_distance_cm = distance_cm;
    ctx->traveled_distance_cm = 0.0f;
    ctx->command = CAR_CMD_MOVE_DISTANCE;
    ctx->state = CAR_STATE_MOVING_FORWARD;
    ctx->command_complete = false;
    
    if (g_mpu_ctx != NULL && ctx->use_heading_control)
    {
        MPU6500_UpdateYaw(g_mpu_ctx);
        ctx->target_heading = MPU6500_GetYaw(g_mpu_ctx);
        ctx->yaw_integral = 0.0f;
        ctx->last_yaw_error = 0.0f;
    }
    
    int32_t target_per_period = ctx->target_speed * ctx->control_period_ms / 1000;
    MotorDriver_SetTargetSpeed(MOTOR_BOTH, target_per_period);
    
    return true;
}

bool Car_MoveBackward(CarContext_t *ctx, float distance_cm)
{
    if (ctx->state != CAR_STATE_IDLE && ctx->state != CAR_STATE_STOPPED)
    {
        return false;
    }
    
    MotorDriver_ResetEncoder(MOTOR_BOTH);
    
    ctx->target_distance_cm = distance_cm;
    ctx->traveled_distance_cm = 0.0f;
    ctx->command = CAR_CMD_MOVE_DISTANCE;
    ctx->state = CAR_STATE_MOVING_BACKWARD;
    ctx->command_complete = false;
    
    if (g_mpu_ctx != NULL && ctx->use_heading_control)
    {
        MPU6500_UpdateYaw(g_mpu_ctx);
        ctx->target_heading = MPU6500_GetYaw(g_mpu_ctx);
        ctx->yaw_integral = 0.0f;
        ctx->last_yaw_error = 0.0f;
    }
    
    int32_t target_per_period = ctx->target_speed * ctx->control_period_ms / 1000;
    MotorDriver_SetTargetSpeed(MOTOR_LEFT, -target_per_period);
    MotorDriver_SetTargetSpeed(MOTOR_RIGHT, -target_per_period);
    
    return true;
}

bool Car_Rotate(CarContext_t *ctx, float angle_deg)
{
    if (ctx->state != CAR_STATE_IDLE && ctx->state != CAR_STATE_STOPPED)
    {
        return false;
    }
    
    MotorDriver_ResetEncoder(MOTOR_BOTH);
    
    if (g_mpu_ctx != NULL)
    {
        MPU6500_UpdateYaw(g_mpu_ctx);
        ctx->start_angle_deg = MPU6500_GetYaw(g_mpu_ctx);
    }
    else
    {
        ctx->start_angle_deg = 0.0f;
    }
    
    ctx->target_angle_deg = angle_deg;
    ctx->command = CAR_CMD_ROTATE_ANGLE;
    ctx->state = CAR_STATE_ROTATING;
    ctx->command_complete = false;
    ctx->yaw_integral = 0.0f;
    ctx->last_yaw_error = 0.0f;
    
    int32_t rotation_speed = CAR_ROTATION_SPEED * ctx->control_period_ms / 1000;
    
    if (angle_deg > 0)
    {
        MotorDriver_SetTargetSpeed(MOTOR_LEFT, rotation_speed);
        MotorDriver_SetTargetSpeed(MOTOR_RIGHT, -rotation_speed);
    }
    else
    {
        MotorDriver_SetTargetSpeed(MOTOR_LEFT, -rotation_speed);
        MotorDriver_SetTargetSpeed(MOTOR_RIGHT, rotation_speed);
    }
    
    return true;
}

void Car_Stop(CarContext_t *ctx)
{
    MotorDriver_StopMotor(MOTOR_BOTH);
    MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
    ctx->state = CAR_STATE_STOPPED;
    ctx->command = CAR_CMD_NONE;
    ctx->command_complete = true;
}

void Car_EmergencyStop(void)
{
    MotorDriver_StopMotor(MOTOR_BOTH);
    MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
}

bool Car_IsCommandComplete(CarContext_t *ctx)
{
    return ctx->command_complete;
}

CarState_t Car_GetState(CarContext_t *ctx)
{
    return ctx->state;
}

static float calculate_yaw_correction(CarContext_t *ctx, float current_heading)
{
    float error = ctx->target_heading - current_heading;
    error = normalize_angle(error);
    
    float dt = ctx->control_period_ms / 1000.0f;
    ctx->yaw_integral += error * dt;
    
    float derivative = (error - ctx->last_yaw_error) / dt;
    ctx->last_yaw_error = error;
    
    float output = ctx->yaw_kp * error + ctx->yaw_ki * ctx->yaw_integral + ctx->yaw_kd * derivative;
    
    if (output > 1000.0f) output = 1000.0f;
    if (output < -1000.0f) output = -1000.0f;
    
    return output;
}

void Car_Update(CarContext_t *ctx)
{
    if (ctx->state == CAR_STATE_IDLE || ctx->state == CAR_STATE_STOPPED)
    {
        return;
    }
    
    float current_heading = 0.0f;
    if (g_mpu_ctx != NULL)
    {
        MPU6500_UpdateYaw(g_mpu_ctx);
        current_heading = MPU6500_GetYaw(g_mpu_ctx);
    }
    
    switch (ctx->state)
    {
        case CAR_STATE_MOVING_FORWARD:
        {
            int32_t left_enc = MotorDriver_GetEncoderCount(MOTOR_LEFT);
            int32_t right_enc = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
            int32_t avg_enc = (left_enc + right_enc) / 2;
            
            ctx->traveled_distance_cm = (float)avg_enc / CAR_ENCODER_TICKS_PER_CM;
            
            if (ctx->traveled_distance_cm >= ctx->target_distance_cm - CAR_DISTANCE_TOLERANCE_CM)
            {
                Car_Stop(ctx);
                return;
            }
            
            if (ctx->use_heading_control && g_mpu_ctx != NULL)
            {
                float correction = calculate_yaw_correction(ctx, current_heading);
                int32_t base_speed = ctx->target_speed * ctx->control_period_ms / 1000;
                int32_t left_speed = base_speed + (int32_t)correction;
                int32_t right_speed = base_speed - (int32_t)correction;
                
                if (left_speed > 5000) left_speed = 5000;
                if (left_speed < 0) left_speed = 0;
                if (right_speed > 5000) right_speed = 5000;
                if (right_speed < 0) right_speed = 0;
                
                MotorDriver_SetTargetSpeed(MOTOR_LEFT, left_speed);
                MotorDriver_SetTargetSpeed(MOTOR_RIGHT, right_speed);
            }
            
            MotorDriver_PIDControl(MOTOR_BOTH);
            break;
        }
        
        case CAR_STATE_MOVING_BACKWARD:
        {
            int32_t left_enc = MotorDriver_GetEncoderCount(MOTOR_LEFT);
            int32_t right_enc = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
            int32_t avg_enc = (left_enc + right_enc) / 2;
            
            ctx->traveled_distance_cm = -(float)avg_enc / CAR_ENCODER_TICKS_PER_CM;
            
            if (ctx->traveled_distance_cm >= ctx->target_distance_cm - CAR_DISTANCE_TOLERANCE_CM)
            {
                Car_Stop(ctx);
                return;
            }
            
            if (ctx->use_heading_control && g_mpu_ctx != NULL)
            {
                float correction = calculate_yaw_correction(ctx, current_heading);
                int32_t base_speed = ctx->target_speed * ctx->control_period_ms / 1000;
                int32_t left_speed = -base_speed + (int32_t)correction;
                int32_t right_speed = -base_speed - (int32_t)correction;
                
                if (left_speed > 0) left_speed = 0;
                if (left_speed < -5000) left_speed = -5000;
                if (right_speed > 0) right_speed = 0;
                if (right_speed < -5000) right_speed = -5000;
                
                MotorDriver_SetTargetSpeed(MOTOR_LEFT, left_speed);
                MotorDriver_SetTargetSpeed(MOTOR_RIGHT, right_speed);
            }
            
            MotorDriver_PIDControl(MOTOR_BOTH);
            break;
        }
        
        case CAR_STATE_ROTATING:
        {
            if (g_mpu_ctx != NULL)
            {
                float current_angle = MPU6500_GetYaw(g_mpu_ctx);
                float angle_traveled = current_angle - ctx->start_angle_deg;
                angle_traveled = normalize_angle(angle_traveled);
                
                float remaining = ctx->target_angle_deg - angle_traveled;
                remaining = normalize_angle(remaining);
                
                if (fabsf(remaining) < CAR_ANGLE_TOLERANCE_DEG)
                {
                    Car_Stop(ctx);
                    return;
                }
                
                float rotation_kp = 3.0f;
                int32_t rotation_speed = (int32_t)(rotation_kp * fabsf(remaining));
                
                if (rotation_speed > CAR_ROTATION_SPEED * ctx->control_period_ms / 1000)
                    rotation_speed = CAR_ROTATION_SPEED * ctx->control_period_ms / 1000;
                if (rotation_speed < 200) rotation_speed = 200;
                
                if (remaining > 0)
                {
                    MotorDriver_SetTargetSpeed(MOTOR_LEFT, rotation_speed);
                    MotorDriver_SetTargetSpeed(MOTOR_RIGHT, -rotation_speed);
                }
                else
                {
                    MotorDriver_SetTargetSpeed(MOTOR_LEFT, -rotation_speed);
                    MotorDriver_SetTargetSpeed(MOTOR_RIGHT, rotation_speed);
                }
            }
            
            MotorDriver_PIDControl(MOTOR_BOTH);
            break;
        }
        
        default:
            break;
    }
}

float Car_GetTraveledDistance(CarContext_t *ctx)
{
    return ctx->traveled_distance_cm;
}

float Car_GetCurrentHeading(CarContext_t *ctx)
{
    (void)ctx;
    if (g_mpu_ctx != NULL)
    {
        return MPU6500_GetYaw(g_mpu_ctx);
    }
    return 0.0f;
}

int32_t Car_GetEncoderAverage(void)
{
    int32_t left = MotorDriver_GetEncoderCount(MOTOR_LEFT);
    int32_t right = MotorDriver_GetEncoderCount(MOTOR_RIGHT);
    return (left + right) / 2;
}

void Car_ResetEncoders(void)
{
    MotorDriver_ResetEncoder(MOTOR_BOTH);
}
