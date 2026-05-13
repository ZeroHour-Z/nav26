# TODO

本文件记录项目后续整理与开源化整改事项。当前先作为路线图保留，不代表立即修改现有功能。

## 高优先级

- [ ] 收窄默认推荐运行路径，明确主推的传感器、定位后端、导航入口和仿真入口。
- [ ] 梳理定位后端重复：重点确认 `point_lio` 与 `point_lio_ros2` 是否重复，决定保留、弃用或移入 experimental/vendor。
- [ ] 移除 launch 启动时修改源码/安装目录配置文件的行为，例如 Odin 模式配置应改为 ROS 参数、临时配置文件或节点内参数覆盖。
- [ ] 规范 `livox_ros_driver2` 的 ROS2 包结构，避免在 `Dockerfile` 或 `build.sh` 中临时复制 `package_ROS2.xml` 为 `package.xml`。
- [ ] 区分自研包和第三方包，考虑将外部项目移动到 `src/vendor/` 或改用 `.repos` 管理。

## 项目结构

- [ ] 建立清晰的包分层文档，说明 driver、perception、localization、navigation、decision、communication、bringup、tools 的职责边界。
- [ ] 评估是否将自研包统一命名为 `xjtu_nav_*` 或保持 `rm_*`，避免命名风格混杂。
- [ ] 清理或标记 ROS1 相关入口，例如 `launch_ROS1`，如果项目目标是 ROS2 Humble/Jazzy，应避免误导用户。
- [ ] 清理运行时临时目录和大文件策略，例如 `tmp/`、PCD/map/bin 数据是否应进入 Git LFS 或 release assets。
- [ ] 统一 RViz 配置入口，保留 bringup 的默认配置，各算法包 RViz 配置作为调试样例。

## 构建与依赖

- [ ] 修正各 `package.xml` 的 description、license、maintainer 和依赖声明，尤其是仍含 TODO 的包。
- [ ] 将第三方依赖的来源、版本、补丁和许可证记录到文档中。
- [ ] 将 `build.sh` 的低内存编译策略文档化，并考虑增加可配置参数，如并行度、跳过 experimental 后端。
- [ ] 增加 `colcon test` 或至少基础 launch/package 检查。
- [ ] 增加 GitHub Actions：Docker build、colcon build、脚本语法检查。

## 技术演进

- [ ] 短期继续维护 ROS2 Humble Docker 环境，中期评估迁移 ROS2 Jazzy LTS。
- [ ] 评估将 `global_velocity_controller` 重构为标准 Nav2 controller plugin 或接入 `ros2_control`。
- [ ] 明确 `rm_decision` 与 Nav2 BT Navigator 的职责边界，避免两个行为树系统职责重叠。
- [ ] 通信协议建议定义专用 ROS msg/srv/action，并补充协议版本、CRC、字段文档和测试样例。
- [ ] 对点云高频链路评估 composable nodes 和 intra-process communication，减少点云复制开销。

## 文档与开源化

- [ ] 新增 `docs/architecture.md`：系统架构、数据流、TF 树、主要话题。
- [ ] 新增 `docs/packages.md`：逐包说明、维护状态、推荐/实验/第三方标记。
- [ ] 新增 `docs/quickstart.md`：Docker、仿真、真机、RViz、本机显示和 VNC。
- [ ] 新增 `CONTRIBUTING.md`、`CHANGELOG.md`、`SECURITY.md`。
- [ ] 增加 `.pre-commit-config.yaml`，统一基础格式检查。
