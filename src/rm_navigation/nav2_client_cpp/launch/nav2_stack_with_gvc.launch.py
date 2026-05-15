from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo


def generate_launch_description():
    use_sim_time = False
    autostart = True

    gvc_odom_topic_arg = DeclareLaunchArgument(
        'gvc_odom_topic',
        default_value='/odom',
        description='Odometry topic consumed by global_velocity_controller'
    )

    gvc_base_frame_arg = DeclareLaunchArgument(
        'gvc_base_frame',
        default_value='base_link',
        description='Base frame used by global_velocity_controller'
    )

    publish_map_to_odom_tf_arg = DeclareLaunchArgument(
        'publish_map_to_odom_tf',
        default_value='false',
        description='Publish static TF map->odom for odom-only sources'
    )

    use_gvc_arg = DeclareLaunchArgument(
        'use_gvc',
        default_value='false',
        description='Start global_velocity_controller. Default false: Nav2 MPPI publishes /cmd_vel directly.'
    )

    # 默认地图路径
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("rm_bringup"), "PCD", "RMUC", "map.yaml"],
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='要加载的地图 yaml 文件的完整路径'
    )

    map_yaml_param = ParameterValue(LaunchConfiguration('map'), value_type=str)


    default_params = PathJoinSubstitution(
        [FindPackageShare("nav2_client_cpp"), "config", "nav2_params.yaml"]
    )

    default_bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ]
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
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "controller_params.yaml",
        ]
    )
    gvc_tracker_params = PathJoinSubstitution(
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "tracker_params.yaml",
        ]
    )
    gvc_sim_params = PathJoinSubstitution(
        [
            FindPackageShare("global_velocity_controller"),
            "config",
            "simulator_params.yaml",
        ]
    )

    nodes = [
        map_arg,
        gvc_odom_topic_arg,
        gvc_base_frame_arg,
        publish_map_to_odom_tf_arg,
        use_gvc_arg,
        LogInfo(msg=["Loading map from: ", LaunchConfiguration('map')]),
    ]

    # 直接使用 substitution（让 launch 在运行时解析为绝对路径）
    nodes.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_param}
            ],
        )
    )

    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(LaunchConfiguration('publish_map_to_odom_tf')),
        )
    )

    nodes.extend(
        [
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
                condition=UnlessCondition(LaunchConfiguration('use_gvc')),
                parameters=[default_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                condition=IfCondition(LaunchConfiguration('use_gvc')),
                parameters=[default_params, {"use_sim_time": use_sim_time}],
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
                        "odom_topic": LaunchConfiguration('gvc_odom_topic'),
                        "default_bt_xml_filename": default_bt_xml,
                        "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                        "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
                    },
                ],
            ),
        ]
    )

    node_names = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": node_names,
                }
            ],
        )
    )

    # 默认由 Nav2 MPPI 直接发布 /cmd_vel。GVC 保留为显式调试选项。
    nodes.append(
        Node(
            package="global_velocity_controller",
            executable="global_velocity_controller_node",
            name="global_velocity_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration('use_gvc')),
            parameters=[
                gvc_params,
                gvc_tracker_params,
                gvc_sim_params,
                {
                    "use_sim_time": use_sim_time,
                    "simulate": False,
                    "odom_topic": LaunchConfiguration('gvc_odom_topic'),
                    "base_frame": LaunchConfiguration('gvc_base_frame'),
                },
            ],
        )
    )

    # nodes.append(
    #     Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         name="rviz2",
    #         output="screen",
    #         arguments=["-d", default_rviz_config],
    #     )
    # )

    return LaunchDescription(nodes)
