# odin1 用的
## 配置环境

### Docker（推荐）

本项目已经配置 ROS Humble/Jammy 容器，主机不需要直接安装 ROS Humble。

```bash
docker compose build
docker compose run --rm nav26
```

运行仿真导航：

```bash
sudo docker compose run --rm nav26 ros2 launch rm_bringup sentry_bringup.launch.py sim:=true rviz:=true
```

运行真实机器人导航：

```bash
docker compose run --rm nav26 ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav
```

开发模式会挂载当前源码，并用 Docker volume 保存 `build/`、`install/`：

```bash
docker compose --profile dev run --rm nav26-dev
./build.sh
```

无桌面或远程机器可以在容器内用 VNC：

```bash
VNC_PASSWORD='你的密码' docker compose run --rm nav26
./script/start_vnc_rviz.sh
./script/run_in_vnc.sh rviz2
```

更完整的 Docker 说明见 [docker/README.md](docker/README.md)。

### rosdep 

```bash
rosdep init 
rosdep update
rosdep install --from-paths src --ignore-src -r -y  
./build.sh
```

## 快速开始

### bringup启动
我们提供了一个统一的启动入口，支持导航和建图两种模式。

#### 1. 导航模式 (默认)
启动所有核心模块：驱动、定位(LIO+重定位)、Nav2导航栈、地形分析（避障、标记颠簸路段）、决策、通信。这些模块都可以在sentry_bringup.launch.up中修改参数为true或false来控制是否启动
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav
```
*   `rviz:=true` (默认): 打开 RViz 可视化界面。
*   `map:=/path/to/map.yaml`: 指定加载的地图（默认为 `src/rm_bringup/PCD/test4/newMap.yaml`）。

#### Odin1 导航模式（使用 Odin 内置里程计/SLAM/重定位）
统一入口新增 `lidar:=odin1` 与 `odin_mode`：

建议：正式导航优先使用 `relocalization`，`odom/slam` 更适合无先验地图或临时调试场景，长时间运行会出现里程计累计漂移。

```bash
# 1) 内置里程计模式
ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav lidar:=odin1 odin_mode:=odom

# 2) 内置 SLAM 模式
ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav lidar:=odin1 odin_mode:=slam

# 3) 内置重定位模式（需提供 map.bin 绝对路径）
ros2 launch rm_bringup sentry_bringup.launch.py \
	mode:=nav lidar:=odin1 odin_mode:=relocalization \
	odin_relocalization_map:=/abs/path/to/map.bin
```

说明：
- 在 `lidar:=odin1` 下，不再启动外部 LIO 后端，直接使用 Odin 内置定位能力。
- 导航控制器将自动改用 `/odin1/odometry` 作为里程计输入。
- `odom/slam` 模式会自动补发 `map->odom` 静态 TF；`relocalization` 模式依赖 Odin 设备侧发布的重定位 TF。
- 若运行中已出现明显漂移，可在线复位算法：`./set_param.sh algo_reset 1`（在工作区根目录执行）。

#### 2. 建图模式
仅启动 LiDAR 驱动和 LIO 建图后端。
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=mapping
```

### 依次启动

1. 启动雷达驱动,发布点云
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 启动里程计和定位

```bash
ros2 launch rm_bringup slam_mapping_only.launch.py backend:=point_lio # 仅建图
ros2 launch rm_bringup slam_odom_only.launch.py backend:=faster_lio # 仅里程计
ros2 launch rm_bringup slam_and_localize.launch.py backend:=faster_lio # 启动重定位和里程计
```

3. 启动地形分析,输出`/traversability/obstacles`和`/traversability/ground`

```bash
ros2 launch rm_terrain_analysis traversability_pointcloud.launch.py # 动态避障，若不启动就只有静态地图
ros2 launch rm_terrain_analysis region_detector.launch.py # 标记颠簸路段，以调整姿态通过
```

4. 启动导航栈

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py # 仿真模式
```

5. 启动决策:
```bash
ros2 launch rm_decision bt.launch.py # 目前测试了追击
```

6. 启动通信节点

```bash
ros2 launch rm_communication communication_bringup.launch.py
```
