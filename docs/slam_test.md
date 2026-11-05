# EdgeAI Challenge — 4×4 m SLAM 测试操作指南

**项目目录：** `~/edgeai_ws/src/EdgeAI_Challenge`  
**运行环境：** Ubuntu 24.04 / Pixi / ROS 2 Jazzy / Gazebo / SLAM Toolbox  
**测试场景：** 4×4 m 围墙、中央 1×1 m 障碍物；机器人初始位置 `(-1.3, -1.3)`。

## 1. 载入 4×4 m 场景并开始 SLAM

**文件位置**
- 场景：`src/edgeai_robot_description/worlds/slam_test_4x4.sdf`
- 启动文件：`src/edgeai_robot_description/launch/simulation_slam_test.launch.py`
- RViz 配置：`src/edgeai_robot_description/rviz/realistic_robot.rviz`

**终端 1：Gazebo + 机器人**
```bash
cd ~/edgeai_ws/src/EdgeAI_Challenge
pixi shell
source install/setup.bash
ros2 launch edgeai_robot_description simulation_slam_test.launch.py
```

**终端 2：SLAM Toolbox（另开终端）**
```bash
cd ~/edgeai_ws/src/EdgeAI_Challenge
pixi shell
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**终端 3：RViz2（另开终端）**
```bash
cd ~/edgeai_ws/src/EdgeAI_Challenge
pixi shell
source install/setup.bash
rviz2 -d src/edgeai_robot_description/rviz/realistic_robot.rviz
```

在 RViz2 中设置 `Fixed Frame = map`，`Map Topic = /map`，检查 `Map → Status: OK`。新的对照实验应先关闭旧 Gazebo 和 SLAM 进程，避免沿用上一次的实时地图。

## 2. 运行测试寻路程序

**Python 文件位置**
- `scripts/rotate_scan.py`：原地旋转（有容差）
- `scripts/move_rotate.py`：正方形路线扫描（有容差，约 1° / 2 cm）
- `scripts/move_rotate_no_tolerance.py`：正方形路线扫描（无提前停止容差）

**终端 4：运行完整扫描路线**
```bash
cd ~/edgeai_ws/src/EdgeAI_Challenge
pixi shell
source install/setup.bash
python scripts/move_rotate_no_tolerance.py   --distance 2.6   --linear-speed 0.10   --angular-speed 0.15
```

**路径：** 原地旋转 360° → 前进 2.6 m → 左转 90° → 前进 2.6 m → 左转 90° → 前进 2.6 m → 左转 90° → 前进 2.6 m → 自动停止。

若要复现实验中的**有容差版本**，只需把脚本名换成 `scripts/move_rotate.py`。当前脚本是固定轨迹测试程序，**不属于比赛 Part 1 要求的自主探索算法**。

## 3. 导出并查看地图图像

保持 SLAM Toolbox 运行。运动结束后，在已激活 Pixi 的终端执行：

**保存地图（PGM + YAML）**
```bash
cd ~/edgeai_ws/src/EdgeAI_Challenge
mkdir -p maps
ros2 run nav2_map_server map_saver_cli   -f maps/slam_4x4_no_tolerance   --ros-args -p use_sim_time:=true
```

**直接查看 PGM**
```bash
display maps/slam_4x4_no_tolerance.pgm
```

**可选：转换为 PNG**
```bash
convert maps/slam_4x4_no_tolerance.pgm maps/slam_4x4_no_tolerance.png
xdg-open maps/slam_4x4_no_tolerance.png
```

**检查导出的文件**
```bash
ls -lh maps/slam_4x4_no_tolerance*
```

文件保存于 `~/edgeai_ws/src/EdgeAI_Challenge/maps/`。其中 `.pgm` 与 `.yaml` 应成对保留；PNG 仅用于查看和报告。已有的基准文件还包括 `slam_test_4x4.*` 和 `slam_4x4_full_route.*`。

> **关闭程序：** 先停止运动，再分别在 SLAM、Gazebo、RViz2 的运行终端按 `Ctrl+C`；如果运动程序已自行结束则无需再停止。已保存的文件不会因关闭终端丢失。
