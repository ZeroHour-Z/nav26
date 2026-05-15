from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port", default="/dev/ttyACM0")
    baud = LaunchConfiguration("baud", default="115200")
    reopen_interval_ms = LaunchConfiguration("reopen_interval_ms", default="500")
    read_loop_hz = LaunchConfiguration("read_loop_hz", default="200.0")
    tx_hz = LaunchConfiguration("tx_hz", default="100.0")
    odom_topic = LaunchConfiguration("odom_topic", default="/odom")
    cmd_vel_frame = LaunchConfiguration("cmd_vel_frame", default="base_link")
    angular_z_mode = LaunchConfiguration("angular_z_mode", default="yaw_rate")
    yaw_rate_preview_time = LaunchConfiguration("yaw_rate_preview_time", default="0.15")
    smooth_world_velocity = LaunchConfiguration("smooth_world_velocity", default="true")
    world_velocity_filter_tau = LaunchConfiguration("world_velocity_filter_tau", default="0.12")
    world_velocity_accel_limit = LaunchConfiguration("world_velocity_accel_limit", default="1.2")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=port),
            DeclareLaunchArgument("baud", default_value=baud),
            DeclareLaunchArgument(
                "reopen_interval_ms", default_value=reopen_interval_ms
            ),
            DeclareLaunchArgument("read_loop_hz", default_value=read_loop_hz),
            DeclareLaunchArgument("tx_hz", default_value=tx_hz),
            DeclareLaunchArgument("odom_topic", default_value=odom_topic),
            DeclareLaunchArgument("cmd_vel_frame", default_value=cmd_vel_frame),
            DeclareLaunchArgument("angular_z_mode", default_value=angular_z_mode),
            DeclareLaunchArgument("yaw_rate_preview_time", default_value=yaw_rate_preview_time),
            DeclareLaunchArgument("smooth_world_velocity", default_value=smooth_world_velocity),
            DeclareLaunchArgument("world_velocity_filter_tau", default_value=world_velocity_filter_tau),
            DeclareLaunchArgument("world_velocity_accel_limit", default_value=world_velocity_accel_limit),
            Node(
                package="rm_communication",
                executable="serial_rw_node",
                name="serial_rw_node",
                parameters=[
                    {
                        "port": port,
                        "baud": baud,
                        "reopen_interval_ms": reopen_interval_ms,
                        "read_loop_hz": read_loop_hz,
                    }
                ],
            ),
            Node(
                package="rm_communication",
                executable="handler_node",
                name="handler_node",
                parameters=[{
                    "tx_hz": tx_hz,
                    "odom_topic": odom_topic,
                    "cmd_vel_frame": cmd_vel_frame,
                    "angular_z_mode": angular_z_mode,
                    "yaw_rate_preview_time": yaw_rate_preview_time,
                    "smooth_world_velocity": smooth_world_velocity,
                    "world_velocity_filter_tau": world_velocity_filter_tau,
                    "world_velocity_accel_limit": world_velocity_accel_limit,
                }],
            ),
        ]
    )
