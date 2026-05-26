# 算法思路与接口说明

本文档用于总结 `Algo` 目录中自主巡航算法的整体思路、各模块职责，以及它们与现有硬件代码之间的接口关系。

## 1. 总体流程

当前算法链路可以概括为：

`传感器数据 -> 位姿更新 -> 栅格建图 -> Frontier 目标选择 -> A* 全局规划 -> 路径跟踪 -> 局部避障 -> 左右轮速度命令`

其中：

- 传感器数据来自现有硬件程序，包括 IMU、编码器、RPLidar 或 LidarPipeline。
- 算法主体位于 `Algo/Inc` 和 `Algo/Src`。
- 适配现有硬件代码的桥接层位于：
  - `Algo/Inc/nav_runtime.h`
  - `Algo/Src/nav_runtime.c`
  - `Algo/Inc/nav_embed_adapter.h`
  - `Algo/Src/nav_embed_adapter.c`

## 2. 各算法模块思路

### 2.1 地图数据结构

核心数据定义在 `Algo/Inc/nav_types.h`：

- `NavPose`
  - 表示机器人当前位姿
  - 包含 `x_mm`、`y_mm`、`theta_rad`
- `NavScan`
  - 表示一帧激光扫描数据
- `NavCommand`
  - 表示算法输出的速度命令
  - 包含线速度 `linear_vel_mms` 和角速度 `angular_vel_rads`
- `NavGridMap`
  - 表示栅格地图
  - `occupancy` 保存占据状态
  - `inflated` 保存障碍膨胀结果

地图中的每个栅格分为三类：

- `NAV_UNKNOWN_CELL`
  - 未知区域
- `NAV_FREE_CELL`
  - 可通行区域
- `NAV_OCCUPIED_CELL`
  - 障碍物区域

### 2.2 激光建图

相关文件：

- `Algo/Inc/grid_map.h`
- `Algo/Src/grid_map.c`

核心思路：

- 将激光雷达每一束数据投影到二维栅格地图中。
- 从机器人所在位置到激光击中点之间的路径标记为空闲。
- 击中点本身标记为障碍。
- 如果某束激光没有在最大量程内击中障碍，则整条射线都视为空闲。

关键接口：

- `NavGridMap_Init()`
  - 初始化地图
- `NavGridMap_WorldToCell()`
  - 将世界坐标转换为栅格坐标
- `NavGridMap_CellToWorld()`
  - 将栅格坐标转换为世界坐标
- `NavGridMap_InsertScan()`
  - 将一帧激光插入地图

### 2.3 障碍膨胀

相关文件：

- `Algo/Inc/grid_map.h`
- `Algo/Src/grid_map.c`

核心思路：

- 真实小车不是质点，规划时不能只避开“障碍中心”。
- 因此会按安全半径对障碍物做膨胀。
- 膨胀后的区域在规划时视为不可通行，从而让路径离障碍物更远。

关键接口：

- `NavGridMap_Inflate()`
  - 按给定半径对障碍做膨胀
- `NavGridMap_IsCellTraversable()`
  - 判断某栅格是否可通行
- `NavGridMap_IsPathBlocked()`
  - 判断已有路径是否被障碍阻断

### 2.4 Frontier 探索

相关文件：

- `Algo/Inc/frontier.h`
- `Algo/Src/frontier.c`

核心思路：

- Frontier 是“已知空闲区域”和“未知区域”的边界。
- 去 frontier 附近，就有机会看到更多未知环境。
- 算法会先找所有 frontier 栅格，再对相邻 frontier 做聚类。
- 每个聚类会根据大小和距离打分，优先选择信息收益更高的目标。

关键接口：

- `NavFrontier_FindClusters()`
  - 查找 frontier 聚类
  - 输出多个候选探索目标

### 2.5 A* 全局路径规划

相关文件：

- `Algo/Inc/planner_astar.h`
- `Algo/Src/planner_astar.c`

核心思路：

- 在膨胀后的栅格地图上做 A* 搜索。
- 起点为当前机器人所在栅格。
- 终点为选中的 frontier 代表点。
- 使用 8 邻域扩展，允许斜向搜索。
- 启发函数采用对角距离，兼顾效率和效果。

关键接口：

- `NavPlanner_AStar()`
  - 计算从起点到终点的路径
- `NavPlanner_CompressPath()`
  - 压缩路径中的共线点，减少不必要的中间节点

### 2.6 路径跟踪

相关文件：

- `Algo/Inc/path_tracker.h`
- `Algo/Src/path_tracker.c`

核心思路：

- 使用接近 Pure Pursuit 的方式进行路径跟踪。
- 从路径中选择一个前视点。
- 根据当前朝向与目标方向之间的误差计算曲率。
- 根据曲率输出线速度和角速度。
- 转弯越大，线速度越低；接近终点时也会主动减速。

关键接口：

- `NavTracker_Reset()`
  - 重置跟踪器状态
- `NavTracker_Compute()`
  - 根据当前位姿和路径输出基础速度命令

### 2.7 局部避障

相关文件：

- `Algo/Inc/local_avoidance.h`
- `Algo/Src/local_avoidance.c`

核心思路：

- 将激光扫描划分为若干扇区。
- 先判断正前方是否存在近距离障碍。
- 如果没有障碍，则保留全局路径跟踪输出。
- 如果前方有障碍，则在未被阻塞的扇区中选择更安全的方向。
- 如果障碍太近，则直接减速甚至停车。

这个模块不会删除全局规划，只会在执行前对命令进行修正。

关键接口：

- `NavAvoidance_BuildSectors()`
  - 根据激光数据构造扇区障碍信息
- `NavAvoidance_Apply()`
  - 对基础速度命令进行避障修正

## 3. 运行时总控逻辑

相关文件：

- `Algo/Inc/nav_runtime.h`
- `Algo/Src/nav_runtime.c`

这个模块用于把“建图、探索、规划、跟踪、避障”串成一条完整运行链。

### 3.1 主要职责

- 保存当前地图
- 保存当前位姿
- 保存当前路径和目标 frontier
- 接收激光数据
- 检测是否需要重新规划
- 输出最终速度命令

### 3.2 主要流程

每次控制周期内大致执行以下步骤：

1. 用编码器和航向角更新当前位姿
2. 接收并整理激光数据
3. 将新激光更新到地图中
4. 对障碍物做膨胀
5. 如果没有目标，或者目标已到达，或者路径已被阻断，则重新选择 frontier 并重新规划
6. 用路径跟踪器生成基础控制命令
7. 用局部避障模块修正控制命令
8. 输出最终线速度和角速度

### 3.3 关键接口

- `NavRuntime_SetDefaultConfig()`
  - 设置默认参数
- `NavRuntime_Init()`
  - 初始化运行时
- `NavRuntime_ResetPose()`
  - 设置初始位姿
- `NavRuntime_UpdatePoseFromOdometry()`
  - 用编码器和航向更新位姿
- `NavRuntime_IngestScanSample()`
  - 持续接收原始激光点
- `NavRuntime_IngestSectorDistances()`
  - 接收扇区级距离结果
- `NavRuntime_Step()`
  - 执行一轮导航逻辑，输出 `NavCommand`

## 4. 与现有硬件程序的接口关系

相关文件：

- `Algo/Inc/nav_embed_adapter.h`
- `Algo/Src/nav_embed_adapter.c`

该适配层的作用是：

- 不修改现有硬件代码
- 直接复用现有 IMU、编码器、电机、雷达接口
- 将硬件侧数据转换成算法可用的数据结构
- 将算法输出转换成底盘左右轮目标速度

### 4.1 输入接口

来自现有硬件程序的输入包括：

#### 1. IMU 航向

来自：

- `Core/Inc/mpu6500.h`

使用接口：

- `MPU6500_UpdateYaw()`
- `MPU6500_GetYaw()`

用途：

- 获取当前车头航向角
- 更新 `NavPose.theta_rad`

#### 2. 编码器数据

来自：

- `Core/Inc/motor_driver.h`

使用接口：

- `MotorDriver_GetEncoderCount(MOTOR_LEFT)`
- `MotorDriver_GetEncoderCount(MOTOR_RIGHT)`

用途：

- 估计每个控制周期内前进距离
- 更新 `NavPose.x_mm` 和 `NavPose.y_mm`

#### 3. 原始激光点数据

来自：

- `Core/Inc/rplidar.h`

使用数据：

- `RPLidarContext.latest_sample`
- `RPLidarContext.frame_count`

用途：

- 将逐点雷达数据送入 `NavRuntime_IngestScanSample()`

#### 4. 扇区级雷达结果

来自：

- `Core/Inc/lidar_pipeline.h`

使用数据：

- `LidarParseResult.filtered_sector_distance`
- `LidarParseResult.filtered_sector_obstacle`
- `LidarParseResult.sector_min_distance`
- `LidarParseResult.sector_obstacle`

用途：

- 直接构造局部避障所需的扇区距离信息

### 4.2 输出接口

最终输出到现有底盘控制代码的接口来自：

- `Core/Inc/motor_driver.h`

使用接口：

- `MotorDriver_SetTargetSpeed(MOTOR_LEFT, value)`
- `MotorDriver_SetTargetSpeed(MOTOR_RIGHT, value)`
- `MotorDriver_PIDControl(MOTOR_BOTH)`
- `MotorDriver_StopMotor(MOTOR_BOTH)`

用途：

- 将算法输出的线速度和角速度转换成左右轮目标转速
- 交给现有电机 PID 速度闭环执行

## 5. 适配层提供的核心接口

### 5.1 初始化接口

- `NavEmbedAdapter_Init()`
  - 初始化适配器和算法运行时
- `NavEmbedAdapter_ResetPose()`
  - 设置初始位姿

### 5.2 以 RPLidar 原始数据驱动算法

- `NavEmbedAdapter_UpdateFromRPLidar()`
  - 输入：
    - `MPU6500Context`
    - `RPLidarContext`
  - 输出：
    - `NavCommand`

适用于直接处理原始雷达点的情况。

### 5.3 以 LidarPipeline 扇区结果驱动算法

- `NavEmbedAdapter_UpdateFromLidarPipeline()`
  - 输入：
    - `MPU6500Context`
    - `LidarParseResult`
  - 输出：
    - `NavCommand`

适用于现有工程已经先经过 `LidarPipeline` 处理的情况。

### 5.4 命令换算与执行

- `NavEmbedAdapter_CommandToMotorTargets()`
  - 将 `NavCommand` 转换为左右轮目标速度
- `NavEmbedAdapter_ApplyCommand()`
  - 将左右轮目标速度写入现有电机控制接口
- `NavEmbedAdapter_Stop()`
  - 停车

## 6. 当前工程中的实际状态

目前算法代码已经整理完成，并且已经具备与现有硬件接口对接的适配层。

但要让小车真正运行这套算法，还需要主控工程完成两件事：

1. 把 `Algo` 目录加入 STM32 工程编译路径
2. 在现有任务循环中调用 `NavEmbedAdapter_UpdateFromRPLidar()` 或 `NavEmbedAdapter_UpdateFromLidarPipeline()`，并将输出命令交给 `NavEmbedAdapter_ApplyCommand()`

也就是说：

- 现在 `Algo` 代码已经“可以接入”
- 但还没有“真正接入到主循环并烧录运行”

## 7. 推荐测试顺序

### 7.1 离线算法测试

可先使用：

- `Algo/Host/nav_validation_main.c`

验证以下能力是否正常：

- 建图
- frontier 探索
- A* 路径规划
- 路径跟踪
- 局部避障

### 7.2 上车联调测试

建议顺序如下：

1. 车轮架空，先测试命令输出是否平稳
2. 检查姿态角是否正常更新
3. 检查编码器里程是否正常累积
4. 检查激光是否持续进入算法
5. 检查是否能生成 frontier 和路径
6. 最后低速落地测试避障与探索

## 8. 总结

这套 `Algo` 代码采用的是一套经典的轻量级自主巡航方案：

- 用激光构建二维占据栅格地图
- 用 frontier 进行未知区域探索
- 用 A* 做全局路径规划
- 用 Pure Pursuit 风格方法做路径跟踪
- 用扇区法做局部避障
- 用适配层接入现有 IMU、编码器、电机和雷达接口

其优点是：

- 结构清晰
- 模块划分明确
- 易于和现有 STM32 底盘程序集成
- 计算量适中，适合当前差速四轮小车平台

