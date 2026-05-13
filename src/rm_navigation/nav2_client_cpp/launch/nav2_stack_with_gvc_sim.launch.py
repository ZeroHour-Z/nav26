"""Nav2 + GVC simulator stack launch.

仿真模式：不启动雷达/驱动/定位，由 GVC 内置 2D 仿真器发布
``odom->base_link`` TF，外部静态 TF 提供 ``map->odom``，使
TF 树保持 ``map -> odom -> base_link``，与 Nav2 局部代价地图
``global_frame: odom`` 配置兼容。

可通过参数 `map` 指定要加载的 2D 栅格地图（yaml）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = False
    autostart = True

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("rm_bringup"), "PCD", "RMUC", "map.yaml"],
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='要加载的 2D 栅格地图 yaml 文件路径'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否启动 RViz'
    )
    sim_init_x_arg = DeclareLaunchArgument('sim_init_x', default_value='0.0')
    sim_init_y_arg = DeclareLaunchArgument('sim_init_y', default_value='0.0')
    sim_init_yaw_arg = DeclareLaunchArgument('sim_init_yaw', default_value='0.0')

    map_yaml_param = ParameterValue(LaunchConfiguration('map'), value_type=str)

    default_params = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "nav2_params.yaml"]
    )
    default_bt_xml = PathJoinSubstitution(
        [FindPackageShare("nav2_bt_navigator"),
         "behavior_trees", "navigate_w_replanning_and_recovery.xml"]
    )
    default_nav_to_pose_bt_xml = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "bt.xml"]
    )
    default_nav_through_poses_bt_xml = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "bt_through_poses.xml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "rviz", "nav2_view.rviz"]
    )

    gvc_params = PathJoinSubstitution(
        [FindPackageShare("global_velocity_controller"),
         "config", "controller_params.yaml"]
    )
    gvc_tracker_params = PathJoinSubstitution(
        [FindPackageShare("global_velocity_controller"),
         "config", "tracker_params.yaml"]
    )
    gvc_sim_params = PathJoinSubstitution(
        [FindPackageShare("global_velocity_controller"),
         "config", "simulator_params.yaml"]
    )

    nodes = [
        map_arg,
        rviz_arg,
        sim_init_x_arg,
        sim_init_y_arg,
        sim_init_yaw_arg,
        LogInfo(msg=["[sim] Loading map from: ", LaunchConfiguration('map')]),
    ]

    # 静态 TF: map -> odom （仿真无定位，提供恒等变换）
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_static_map_to_odom",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        )
    )

    nodes.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml_param}],
        )
    )

    nodes.extend([
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[default_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[default_params, {"use_sim_time": use_sim_time}],
            # Nav2 controller 的 cmd_vel 输出被 GVC 取代，这里重映射避免污染 /cmd_vel
            remappings=[("/cmd_vel", "/nav2_cmd_vel")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[default_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                default_params,
                {
                    "use_sim_time": use_sim_time,
                    "default_bt_xml_filename": default_bt_xml,
                    "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                    "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
                },
            ],
        ),
    ])

    node_names = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "map_server",
    ]
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": node_names,
            }],
        )
    )

    # GVC 仿真：发布 odom->base_link TF，父坐标系强制为 odom 以保持单一 TF 父节点
    nodes.append(
        Node(
            package="global_velocity_controller",
            executable="global_velocity_controller_node",
            name="global_velocity_controller",
            output="screen",
            parameters=[
                gvc_params,
                gvc_tracker_params,
                gvc_sim_params,
                {
                    "use_sim_time": use_sim_time,
                    "simulate": True,
                    "sim_tf_parent_frame": "odom",
                    "map_frame": "map",
                    "base_frame": "base_link",
                    "enable_dynamic_lookahead": True,
                    "lookahead_distance": 1.0,
                    "min_lookahead_distance": 0.25,
                    "max_lookahead_distance": 0.8,
                    "curvature_window_distance": 0.8,
                    "cmd_accel_limit_linear": 1.0,
                    "cmd_decel_limit_linear": 2.0,
                    "pose_predict_enabled": False,
                    "sim_init_x": ParameterValue(LaunchConfiguration('sim_init_x'), value_type=float),
                    "sim_init_y": ParameterValue(LaunchConfiguration('sim_init_y'), value_type=float),
                    "sim_init_yaw": ParameterValue(LaunchConfiguration('sim_init_yaw'), value_type=float),
                },
            ],
        )
    )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        )
    )

    return LaunchDescription(nodes)
