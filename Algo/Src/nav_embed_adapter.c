#include "nav_embed_adapter.h"

#include <math.h>

#include "../../Core/Inc/car_control.h"

static int32_t NavEmbedAdapter_RoundToInt(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static void NavEmbedAdapter_UpdatePose(NavEmbedAdapter *adapter, MPU6500Context *mpu_ctx)
{
    float yaw_rad;
    int32_t left_encoder;
    int32_t right_encoder;

    MPU6500_UpdateYaw(mpu_ctx);
    yaw_rad = MPU6500_GetYaw(mpu_ctx) * ((float)M_PI / 180.0f);
    left_encoder = MotorDriver_GetEncoderCount(MOTOR_LEFT);
    right_encoder = MotorDriver_GetEncoderCount(MOTOR_RIGHT);

    NavRuntime_UpdatePoseFromOdometry(&adapter->runtime,
                                      yaw_rad,
                                      left_encoder,
                                      right_encoder,
                                      adapter->ticks_per_cm);
}

static void NavEmbedAdapter_FillSectorSnapshot(const LidarParseResult *parse_result,
                                               float *distances_mm,
                                               bool *valid_mask)
{
    uint16_t i;

    for (i = 0U; i < 8U; ++i)
    {
        if (parse_result->filtered_sector_obstacle[i] &&
            parse_result->filtered_sector_distance[i] > 0.0f)
        {
            distances_mm[i] = parse_result->filtered_sector_distance[i];
            valid_mask[i] = true;
        }
        else if (parse_result->sector_obstacle[i] &&
                 parse_result->sector_min_distance[i] > 0.0f)
        {
            distances_mm[i] = parse_result->sector_min_distance[i];
            valid_mask[i] = true;
        }
        else
        {
            distances_mm[i] = parse_result->filtered_min_distance_mm;
            valid_mask[i] = false;
        }
    }
}

void NavEmbedAdapter_Init(NavEmbedAdapter *adapter, const NavRuntimeConfig *config)
{
    if (adapter == NULL)
    {
        return;
    }

    NavRuntime_Init(&adapter->runtime, config);
    adapter->ticks_per_cm = CAR_ENCODER_TICKS_PER_CM;
    adapter->wheel_base_mm = CAR_WHEEL_BASE_CM * 10.0f;
    adapter->control_period_ms = CAR_CONTROL_PERIOD_MS;
    adapter->last_rplidar_frame_count = 0U;
    adapter->last_lidar_block_id = 0U;
}

void NavEmbedAdapter_ResetPose(NavEmbedAdapter *adapter, float x_mm, float y_mm, float yaw_deg)
{
    if (adapter == NULL)
    {
        return;
    }

    NavRuntime_ResetPose(&adapter->runtime, x_mm, y_mm, yaw_deg * ((float)M_PI / 180.0f));
}

/* ================================================================
 * NavEmbedAdapter_Update — 统一的导航更新入口
 * 严格按照 README 第 3.2 节流程:
 *   1. 用编码器和航向角更新当前位姿
 *   2. 接收并整理激光数据
 *   3. 将新激光更新到地图中
 *   4. 对障碍物做膨胀
 *   5. 检查是否需要重新规划 (无目标/目标到达/路径阻断)
 *   6. 用路径跟踪器生成基础控制命令
 *   7. 用局部避障模块修正控制命令
 *   8. 输出最终 NavCommand (由调用方执行)
 * ================================================================ */
bool NavEmbedAdapter_Update(NavEmbedAdapter *adapter,
                            MPU6500Context *mpu_ctx,
                            RPLidarContext *lidar_ctx,
                            NavCommand *command)
{
    if (adapter == NULL || mpu_ctx == NULL || command == NULL)
    {
        return false;
    }

    /* Step 1: 用编码器和航向角更新当前位姿 */
    NavEmbedAdapter_UpdatePose(adapter, mpu_ctx);

    /* Step 2: 接收并整理激光数据 (从原始 RPLidar 帧获取) */
    if (lidar_ctx != NULL && lidar_ctx->frame_count != adapter->last_rplidar_frame_count)
    {
        NavRuntime_IngestScanSample(&adapter->runtime,
                                    lidar_ctx->latest_sample.angle_deg * ((float)M_PI / 180.0f),
                                    lidar_ctx->latest_sample.distance_mm,
                                    lidar_ctx->latest_sample.check_bit_ok &&
                                    lidar_ctx->latest_sample.distance_mm > 0.0f,
                                    lidar_ctx->latest_sample.start_flag);
        adapter->last_rplidar_frame_count = lidar_ctx->frame_count;
    }

    /* Step 3-8: 总控逻辑 (建图→膨胀→Frontier→A*→跟踪→避障→指令) */
    return NavRuntime_Step(&adapter->runtime, command);
}

bool NavEmbedAdapter_UpdateFromRPLidar(NavEmbedAdapter *adapter,
                                       MPU6500Context *mpu_ctx,
                                       const RPLidarContext *lidar_ctx,
                                       NavCommand *command)
{
    if (adapter == NULL || mpu_ctx == NULL || command == NULL)
    {
        return false;
    }

    NavEmbedAdapter_UpdatePose(adapter, mpu_ctx);

    if (lidar_ctx != NULL && lidar_ctx->frame_count != adapter->last_rplidar_frame_count)
    {
        NavRuntime_IngestScanSample(&adapter->runtime,
                                    lidar_ctx->latest_sample.angle_deg * ((float)M_PI / 180.0f),
                                    lidar_ctx->latest_sample.distance_mm,
                                    lidar_ctx->latest_sample.check_bit_ok &&
                                    lidar_ctx->latest_sample.distance_mm > 0.0f,
                                    lidar_ctx->latest_sample.start_flag);
        adapter->last_rplidar_frame_count = lidar_ctx->frame_count;
    }

    return NavRuntime_Step(&adapter->runtime, command);
}

bool NavEmbedAdapter_UpdateFromLidarPipeline(NavEmbedAdapter *adapter,
                                             MPU6500Context *mpu_ctx,
                                             const LidarParseResult *parse_result,
                                             NavCommand *command)
{
    float distances_mm[8];
    bool valid_mask[8];

    if (adapter == NULL || mpu_ctx == NULL || command == NULL)
    {
        return false;
    }

    NavEmbedAdapter_UpdatePose(adapter, mpu_ctx);

    if (parse_result != NULL && parse_result->block_id != adapter->last_lidar_block_id)
    {
        if (parse_result->has_sample)
        {
            NavRuntime_IngestScanSample(&adapter->runtime,
                                        parse_result->latest_sample.angle_deg * ((float)M_PI / 180.0f),
                                        parse_result->latest_sample.distance_mm,
                                        parse_result->latest_sample.check_bit_ok &&
                                        parse_result->latest_sample.distance_mm > 0.0f,
                                        parse_result->latest_sample.start_flag);
        }

        NavEmbedAdapter_FillSectorSnapshot(parse_result, distances_mm, valid_mask);
        NavRuntime_IngestSectorDistances(&adapter->runtime, distances_mm, valid_mask, 8U);
        adapter->last_lidar_block_id = parse_result->block_id;
    }

    return NavRuntime_Step(&adapter->runtime, command);
}

void NavEmbedAdapter_CommandToMotorTargets(const NavEmbedAdapter *adapter,
                                           const NavCommand *command,
                                           int32_t *left_target,
                                           int32_t *right_target)
{
    float half_wheel_base_mm;
    float mm_to_ticks_per_second;
    float period_scale;
    float left_mm_s;
    float right_mm_s;

    if (adapter == NULL || command == NULL || left_target == NULL || right_target == NULL)
    {
        return;
    }

    half_wheel_base_mm = adapter->wheel_base_mm * 0.5f;
    mm_to_ticks_per_second = adapter->ticks_per_cm / 10.0f;
    period_scale = (float)adapter->control_period_ms / 1000.0f;

    left_mm_s = command->linear_vel_mms - command->angular_vel_rads * half_wheel_base_mm;
    right_mm_s = command->linear_vel_mms + command->angular_vel_rads * half_wheel_base_mm;

    *left_target = NavEmbedAdapter_RoundToInt(left_mm_s * mm_to_ticks_per_second * period_scale);
    *right_target = NavEmbedAdapter_RoundToInt(right_mm_s * mm_to_ticks_per_second * period_scale);
}

void NavEmbedAdapter_ApplyCommand(const NavEmbedAdapter *adapter, const NavCommand *command)
{
    int32_t left_target;
    int32_t right_target;

    if (adapter == NULL || command == NULL)
    {
        return;
    }

    NavEmbedAdapter_CommandToMotorTargets(adapter, command, &left_target, &right_target);
    MotorDriver_SetTargetSpeed(MOTOR_LEFT, left_target);
    MotorDriver_SetTargetSpeed(MOTOR_RIGHT, right_target);
    /* PID由主控制循环统一运行，此处只设置目标速度 */
}

void NavEmbedAdapter_Stop(void)
{
    MotorDriver_SetTargetSpeed(MOTOR_BOTH, 0);
    MotorDriver_StopMotor(MOTOR_BOTH);
}
