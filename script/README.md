# script 目录说明

本目录放项目常用辅助脚本。当前主要分为两类：VNC/RViz 可视化脚本，以及导航/地图工具脚本。

## VNC + RViz 可视化

OrbStack 的 Ubuntu 虚拟机没有类似 WSLg 的内置图形显示服务，直接运行 `rviz2` 会出现 `could not connect to display`。本项目使用 TigerVNC 提供虚拟桌面，让 RViz、rqt 等 GUI 程序显示在 VNC 桌面中。

### 脚本作用

- `start_vnc_rviz.sh`
  - 启动 VNC 桌面 `:1`，端口为 `5901`。
  - VNC 只监听虚拟机本机地址，需要从 Mac 通过 SSH 隧道访问。
  - 会把 `~/.vnc/xstartup` 指向项目内的 `script/vnc_xstartup.sh`。

- `stop_vnc_rviz.sh`
  - 停止 VNC 桌面 `:1`。

- `run_in_vnc.sh`
  - 为 ROS/GUI 命令设置 VNC 显示环境。
  - 会自动 source `/opt/ros/humble/setup.bash` 和工作区 `install/setup.bash`。
  - 默认设置：
    - `DISPLAY=:1`
    - `LIBGL_ALWAYS_SOFTWARE=1`
    - `QT_X11_NO_MITSHM=1`

- `vnc_xstartup.sh`
  - TigerVNC 的桌面启动脚本。
  - 启动 XFCE 桌面，并设置 RViz 需要的软件 OpenGL 环境。
  - 不直接手动执行，由 `vncserver` 调用。

### 使用流程

在 Ubuntu 虚拟机中启动 VNC：

```bash
cd ~/xjtu_nav26
./script/start_vnc_rviz.sh
```

在 Mac 端开启 SSH 隧道：

```bash
ssh -L 5901:localhost:5901 zerohour@192.168.139.216
```

然后打开 macOS 屏幕共享或 VNC 客户端：

```bash
open vnc://localhost:5901
```

进入 VNC 桌面后，在 Ubuntu 终端运行带 RViz 的 launch：

```bash
cd ~/xjtu_nav26
./script/run_in_vnc.sh ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

如果只想单独测试 RViz：

```bash
cd ~/xjtu_nav26
./script/run_in_vnc.sh rviz2
```

停止 VNC：

```bash
cd ~/xjtu_nav26
./script/stop_vnc_rviz.sh
```

### VNC 密码

VNC 密码保存在用户目录的 `~/.vnc/passwd`，不放入项目仓库。

如需重新设置密码：

```bash
vncpasswd ~/.vnc/passwd
```

设置后重新启动 VNC：

```bash
cd ~/xjtu_nav26
./script/start_vnc_rviz.sh
```

容器中首次启动 VNC 时，也可以用环境变量自动设置密码：

```bash
VNC_PASSWORD='你的密码' ./script/start_vnc_rviz.sh
```

### 常见问题

- `rviz2` 报 `could not connect to display`
  - 请确认命令是通过 `./script/run_in_vnc.sh ...` 启动的。
  - 请确认 VNC 已启动：`vncserver -list` 中应看到 `:1`。

- Mac 无法连接 `localhost:5901`
  - 请确认 Mac 端 SSH 隧道仍在运行。
  - 请确认 Ubuntu 端 VNC 已启动。

- RViz 打开但渲染异常或 OpenGL 报错
  - `run_in_vnc.sh` 默认启用 `LIBGL_ALWAYS_SOFTWARE=1`，一般可避免硬件 OpenGL 问题。
  - 可用下面命令检查 VNC 内 OpenGL：
    ```bash
    DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B
    ```

## Docker

如果主机是 Ubuntu 26，不建议直接在主机系统里硬装 ROS Humble。更稳的方式是使用项目根目录的 Docker 配置，把 ROS Humble 固定在 Ubuntu 22.04/Jammy 容器中运行。

Ubuntu 26 本机带桌面环境时，不需要 VNC，可以直接把容器里的 RViz 显示到主机桌面：

```bash
cd ~/xjtu_nav26
xhost +local:docker
docker compose run --rm nav26 ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

构建镜像：

```bash
cd ~/xjtu_nav26
docker compose build
```

进入容器：

```bash
cd ~/xjtu_nav26
docker compose run --rm nav26
```

容器中启动 VNC：

```bash
VNC_PASSWORD='你的密码'
./script/start_vnc_rviz.sh
```

容器中启动带 RViz 的仿真导航：

```bash
./script/run_in_vnc.sh ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

更完整的 Docker 说明见 `docker/README.md`。

## 导航模块脚本

- `start_nav_modules.sh`
  - 分模块启动 Odin 驱动、静态 TF、地形分析、Nav2、决策与通信。
  - 会记录各进程 PID 到 `log/modular_nav/latest.pids`。
  - 可通过环境变量调整：
    - `MAP_YAML`：地图 yaml 路径
    - `ODIN_MODE`：`odom`、`slam` 或 `relocalization`
    - `RELOCALIZATION_MAP`：Odin 重定位地图路径
    - `START_DECISION`：是否启动决策，默认 `1`
    - `START_COMM`：是否启动通信，默认 `1`
    - `PUBLISH_ODIN_BASE_TF`：是否发布 `odin1_base_link -> base_link` 静态 TF，默认 `1`

示例：

```bash
cd ~/xjtu_nav26
./script/start_nav_modules.sh
```

使用指定地图：

```bash
cd ~/xjtu_nav26
MAP_YAML=~/xjtu_nav26/src/rm_bringup/PCD/RMUL/newMap.yaml ./script/start_nav_modules.sh
```

- `stop_nav_modules.sh`
  - 根据 `log/modular_nav/latest.pids` 停止 `start_nav_modules.sh` 启动的模块。

```bash
cd ~/xjtu_nav26
./script/stop_nav_modules.sh
```

## 地图保存脚本

- `save_grid_map.sh`
  - 调用 Nav2 map saver 保存 2D 栅格地图。
  - 当前脚本中的 `YOUR_MAP_NAME` 是占位名，使用前应改成实际保存路径。

- `save_pcd.sh`
  - 调用 `/map_save` 服务保存点云/地图数据，依赖对应服务已启动。
