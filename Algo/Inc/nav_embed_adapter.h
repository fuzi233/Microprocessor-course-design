#ifndef NAV_EMBED_ADAPTER_H
#define NAV_EMBED_ADAPTER_H

#include "nav_runtime.h"

#include "../../Core/Inc/lidar_pipeline.h"
#include "../../Core/Inc/motor_driver.h"
#include "../../Core/Inc/mpu6500.h"
#include "../../Core/Inc/rplidar.h"

typedef struct
{
    NavRuntime runtime;
    float ticks_per_cm;
    float wheel_base_mm;
    uint32_t control_period_ms;
    uint32_t last_rplidar_frame_count;
    uint32_t last_lidar_block_id;
} NavEmbedAdapter;

void NavEmbedAdapter_Init(NavEmbedAdapter *adapter, const NavRuntimeConfig *config);
void NavEmbedAdapter_ResetPose(NavEmbedAdapter *adapter, float x_mm, float y_mm, float yaw_deg);

/* 统一的导航更新入口，严格按 README 第 3.2 节流程:
 *   位姿更新 → 雷达摄取 → 建图膨胀 → Frontier → A* → 跟踪 → 避障 → NavCommand
 * 内部自动选择数据源: 优先用 LidarPipeline 扇区数据, 无新数据则直接取原始雷达帧 */
bool NavEmbedAdapter_Update(NavEmbedAdapter *adapter,
                            MPU6500Context *mpu_ctx,
                            RPLidarContext *lidar_ctx,
                            NavCommand *command);

bool NavEmbedAdapter_UpdateFromRPLidar(NavEmbedAdapter *adapter,
                                       MPU6500Context *mpu_ctx,
                                       const RPLidarContext *lidar_ctx,
                                       NavCommand *command);
bool NavEmbedAdapter_UpdateFromLidarPipeline(NavEmbedAdapter *adapter,
                                             MPU6500Context *mpu_ctx,
                                             const LidarParseResult *parse_result,
                                             NavCommand *command);
void NavEmbedAdapter_CommandToMotorTargets(const NavEmbedAdapter *adapter,
                                           const NavCommand *command,
                                           int32_t *left_target,
                                           int32_t *right_target);
void NavEmbedAdapter_ApplyCommand(const NavEmbedAdapter *adapter, const NavCommand *command);
void NavEmbedAdapter_Stop(void);

#endif
