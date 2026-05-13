# Docker 使用说明

这个项目建议在 Ubuntu 26 主机上用 Docker 运行 ROS Humble。容器内部使用官方 ROS Humble/Jammy 环境，主机不需要降级到 Ubuntu 22.04。

## 构建镜像

在项目根目录执行：

```bash
docker compose build
```

镜像名为 `xjtu-nav26:humble`。构建过程中会安装 ROS/Nav2/RViz/VNC 依赖，编译 Livox-SDK2，并执行项目根目录的 `./build.sh`。

如果只是运行当前代码，使用默认服务 `nav26`；它运行的是镜像内已经编译好的工作区。

```bash
docker compose run --rm nav26
```

如果需要边改源码边编译，使用开发服务 `nav26-dev`。它会把当前仓库挂进容器，并用 Docker volume 保存 `build/`、`install/`，避免每次进容器都从零编译。

```bash
docker compose --profile dev run --rm nav26-dev
./build.sh
```

## Ubuntu 26 本机 GUI 运行

如果 Ubuntu 26 主机本身带桌面环境，不需要 VNC。让容器访问主机 X11/XWayland 显示即可。

直接启动项目统一入口的仿真模式：

```bash
sudo docker compose run --rm nav26 ros2 launch rm_bringup sentry_bringup.launch.py sim:=true rviz:=true
```

或者只启动 Nav2 + GVC 仿真导航：

```bash
sudo docker compose run --rm nav26 ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

真实机器人导航入口：

```bash
sudo docker compose run --rm nav26 ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav
```

Odin1 内置里程计模式：

```bash
sudo docker compose run --rm nav26 ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav lidar:=odin1 odin_mode:=odom
```

单独测试 RViz：

```bash
sudo docker compose run --rm nav26 rviz2
```

开发容器里运行同样的命令时，把服务名换成 `nav26-dev`：

```bash
docker compose --profile dev run --rm nav26-dev ros2 launch rm_bringup sentry_bringup.launch.py sim:=true rviz:=true
```

如果 RViz 报 `could not connect to display`，先在主机确认：

```bash
echo $DISPLAY
```

正常情况下不应为空。`docker-compose.yml` 会把主机的 `/tmp/.X11-unix` 和 `/run/user/1000` 挂进容器，并把主机 `DISPLAY` 与 `XAUTHORITY` 传给容器。

## 进入容器 Shell

```bash
docker compose run --rm nav26
```

容器默认已经 source：

- `/opt/ros/humble/setup.bash`
- `/home/zerohour/xjtu_nav26/install/setup.bash`

开发容器：

```bash
docker compose --profile dev run --rm nav26-dev
```

## 远程或无桌面环境使用 VNC

VNC 是备用方案，适合 SSH 远程、OrbStack、服务器无显示器等场景。本机 Ubuntu 26 桌面运行时通常不需要这一节。

进入容器后启动 VNC：

```bash
VNC_PASSWORD='你的密码'
./script/start_vnc_rviz.sh
```

首次启动 VNC 时需要设置 `VNC_PASSWORD`，该密码只用于 VNC 连接。

```bash
VNC_PASSWORD='你的密码' docker compose run --rm nav26
```

如果容器运行在本机 Ubuntu 26 上，VNC 客户端连接：

```text
localhost:5901
```

如果容器运行在远端机器上，请先从本机开 SSH 隧道：

```bash
ssh -L 5901:localhost:5901 <user>@<docker-host>
```

然后连接：

```text
localhost:5901
```

## 在 VNC 中启动仿真导航

在容器内：

```bash
./script/run_in_vnc.sh ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

单独测试 RViz：

```bash
./script/run_in_vnc.sh rviz2
```

## 硬件与网络

`docker-compose.yml` 默认使用：

- `network_mode: host`
- `ipc: host`
- `privileged: true`

这样更适合 ROS 2 DDS 通信、Livox/串口/网口设备访问和 RViz/VNC 使用。若后续要收紧权限，可以再把 `privileged` 改成明确的 `devices` 映射。
