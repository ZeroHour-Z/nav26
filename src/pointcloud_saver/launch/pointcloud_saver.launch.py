from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    _pkg = Path(__file__).resolve().parent.parent
    _tmp = _pkg / "tmp"
    _tmp.mkdir(parents=True, exist_ok=True)
    _session = str(_tmp / f"odin_slam_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pcd")

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/odin1/cloud_slam",
                              description="订阅的点云话题"),
        DeclareLaunchArgument("target_frame", default_value="odom",
                              description="累积点云的目标坐标系"),
        DeclareLaunchArgument("save_dir", default_value=str(_tmp),
                              description="PCD 保存目录（定时/服务保存）"),
        DeclareLaunchArgument("session_pcd_path", default_value=_session,
                              description="退出时写入的会话地图路径（与 Mid360 file_path 语义一致）"),
        DeclareLaunchArgument("merge_voxel_size", default_value="0.10",
                              description="merge 定时器体素下采样大小(m)，0=关闭"),
        DeclareLaunchArgument("merge_interval", default_value="5.0",
                              description="merge 定时器间隔(秒)"),
        DeclareLaunchArgument("save_rgb", default_value="true",
                              description="是否保存 RGB 颜色"),
        DeclareLaunchArgument("pcd_binary", default_value="true",
                              description="PCD 使用二进制格式（与 Mid360 writeBinary 一致）"),
        DeclareLaunchArgument("skip_tf", default_value="true",
                              description="跳过 TF 变换（cloud_slam 已在 odom 坐标系）"),
        DeclareLaunchArgument("auto_start", default_value="true",
                              description="启动后自动开始录制"),
        DeclareLaunchArgument("auto_save_interval", default_value="-1",
                              description="自动保存间隔(秒)，-1=关闭（与 Mid360 interval=-1 一致，依赖退出保存）"),
        DeclareLaunchArgument("auto_save_voxel_size", default_value="0.05",
                              description="自动保存时体素滤波大小(m)"),
        DeclareLaunchArgument("max_stat_points", default_value="150000",
                              description="统计滤波点数上限"),
        DeclareLaunchArgument("process_every_n", default_value="3",
                              description="每 N 帧处理 1 帧（跳帧降负载）"),

        Node(
            package="pointcloud_saver",
            executable="pointcloud_saver_node.py",
            name="pointcloud_saver",
            output="screen",
            parameters=[{
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "save_dir": LaunchConfiguration("save_dir"),
                "session_pcd_path": LaunchConfiguration("session_pcd_path"),
                "merge_voxel_size": LaunchConfiguration("merge_voxel_size"),
                "merge_interval": LaunchConfiguration("merge_interval"),
                "save_rgb": LaunchConfiguration("save_rgb"),
                "pcd_binary": LaunchConfiguration("pcd_binary"),
                "skip_tf": LaunchConfiguration("skip_tf"),
                "auto_start": LaunchConfiguration("auto_start"),
                "auto_save_interval": LaunchConfiguration("auto_save_interval"),
                "auto_save_voxel_size": LaunchConfiguration("auto_save_voxel_size"),
                "max_stat_points": LaunchConfiguration("max_stat_points"),
                "process_every_n": LaunchConfiguration("process_every_n"),
                "auto_publish": True,
                "publish_interval": 2.0,
                "max_points": 10000000,
            }],
        ),
    ])
