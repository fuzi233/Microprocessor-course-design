# 板载导航算法离线验证说明

这个目录里放的是一套纯 `C` 的导航算法验证程序，目的是在小车还不在手边时，先把以后要运行在 `NUCLEO-F446RE` 上的算法流程验证通。

## 包含内容

- `Algo/Inc` 和 `Algo/Src`
  - `grid_map.*`：占据栅格建图、射线更新、障碍膨胀
  - `frontier.*`：基于 Frontier 的探索目标选择
  - `planner_astar.*`：在膨胀地图上的 A* 全局规划
  - `path_tracker.*`：Pure Pursuit 路径跟踪
  - `local_avoidance.*`：轻量级扇区法局部避障
- `Algo/Host/nav_validation_main.c`
  - 电脑端离线验证主程序，使用一张模拟地图和模拟激光雷达数据，跑通整套闭环

## 适合当前阶段做什么

你现在已经完成了定位和底层控制部分，那么这套程序主要用于验证下面这些算法是否能协同工作：

1. 建图
2. 探索
3. 全局路径规划
4. 路径跟踪
5. 局部避障

它不会连接真实硬件，而是用模拟场景来代替真实雷达和小车运动。

## 如何编译

这套代码是标准 `C99`，没有第三方依赖。只要你电脑上有 `gcc`、`clang` 或 `Visual Studio` 的 `cl` 编译器，都可以编译。

### 用 GCC 或 Clang

在 `Algo/Host` 目录下执行：

```bash
gcc -std=c99 -O2 -I../Inc nav_validation_main.c ../Src/grid_map.c ../Src/frontier.c ../Src/planner_astar.c ../Src/path_tracker.c ../Src/local_avoidance.c -lm -o nav_validation
```

### 用 Visual Studio 的 cl

在 `Algo/Host` 目录下执行：

```bat
cl /nologo /O2 /I..\Inc nav_validation_main.c ..\Src\grid_map.c ..\Src\frontier.c ..\Src\planner_astar.c ..\Src\path_tracker.c ..\Src\local_avoidance.c
```

## 如何运行

1. 打开终端
2. 切换到目录 [Algo/Host](C:/Users/2023/Desktop/Phase1.1/1.0/Algo/Host)
3. 先编译
4. 再运行生成的可执行文件

### GCC/Clang 编译后运行

```bash
./nav_validation
```

### Visual Studio 编译后运行

```bat
nav_validation_main.exe
```

## 运行后会得到什么

运行成功后会有两类输出：

- 控制台日志
  - 显示当前步数
  - 小车位姿
  - 已探索空闲区域数量
  - 当前路径长度
  - 当前控制命令
- 图像文件
  - `nav_validation_result.ppm`

## 结果图怎么看

生成的 `nav_validation_result.ppm` 颜色含义如下：

- 白色：已知空闲区域
- 灰色：未知区域
- 黑色：建图得到的障碍物
- 橙色：小车实际走过的轨迹
- 蓝色：当前规划路径
- 绿色：当前 frontier 目标
- 红色：最终小车位置

如果系统正常运行，你应该能看到：

1. 地图逐渐从灰色变成白色和黑色
2. 小车轨迹逐步扩展到更多未知区域
3. 路径会随着探索过程不断重规划

## 后续如何移植到 STM32

后面拿到小车后，不需要推翻这套算法，只要把输入输出接到你现有的嵌入式工程即可。

移植顺序建议如下：

1. 用你现有的定位结果填充 `NavPose`
2. 把真实激光雷达数据整理成 `NavScan`
3. 调用 `NavGridMap_InsertScan()` 做建图
4. 调用 `NavGridMap_Inflate()` 做障碍膨胀
5. 用 `NavFrontier_FindClusters()` 选探索目标
6. 用 `NavPlanner_AStar()` 生成路径
7. 用 `NavTracker_Compute()` 生成基础速度命令
8. 用 `NavAvoidance_Apply()` 在发给电机前做局部避障修正

## 建议你现在先做的事

如果你当前还没有编译环境，建议先安装任意一种：

- `Visual Studio`
- `Code::Blocks`
- `MinGW-w64`

然后优先把这套离线验证程序跑起来，确认：

1. 能编译通过
2. 能输出日志
3. 能生成 `nav_validation_result.ppm`

做到这一步，就说明你的核心导航算法框架已经初步打通了，后面只需要再接真实传感器和底盘。
