from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import re


def _replace_yaml_line(text: str, key: str, value_literal: str) -> str:
    pattern = rf'(^\s*{re.escape(key)}\s*:\s*).*$'
    new_text, count = re.subn(
        pattern,
        lambda m: m.group(1) + value_literal,
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if count == 0:
        return text + f"\n  {key}: {value_literal}\n"
    return new_text


def _configure_odin_mode(context):
    if LaunchConfiguration('sim').perform(context).lower() == 'true':
        return []
    lidar = LaunchConfiguration('lidar').perform(context)
    if lidar != 'odin1':
        return []

    run_mode = LaunchConfiguration('mode').perform(context)
    odin_mode = LaunchConfiguration('odin_mode').perform(context)
    relocalization_map = LaunchConfiguration('odin_relocalization_map').perform(context)
    mode_to_value = {
        'odom': '0',
        'slam': '1',
        'relocalization': '2',
    }
    custom_map_mode = mode_to_value.get(odin_mode, '0')

    this_file = Path(__file__).resolve()
    source_cfg = this_file.parents[2] / 'odin_ros_driver' / 'config' / 'control_command.yaml'

    target_files = [source_cfg]
    try:
        share_cfg = Path(get_package_share_directory('odin_ros_driver')) / 'config' / 'control_command.yaml'
        if share_cfg not in target_files:
            target_files.append(share_cfg)
    except Exception:
        pass

    updated_files = []
    for cfg in target_files:
        if not cfg.exists():
            continue
        text = cfg.read_text(encoding='utf-8')
        text = _replace_yaml_line(text, 'custom_map_mode', custom_map_mode)
        text = _replace_yaml_line(text, 'relocalization_map_abs_path', f'"{relocalization_map}"')
        cfg.write_text(text, encoding='utf-8')
        updated_files.append(str(cfg))

    if odin_mode == 'relocalization' and not relocalization_map:
        return [
            LogInfo(msg='[sentry_bringup] WARNING: odin_mode=relocalization but odin_relocalization_map is empty.'),
            LogInfo(msg='[sentry_bringup] Please set odin_relocalization_map:=/abs/path/to/map.bin'),
        ]

    if run_mode == 'nav' and odin_mode in ('odom', 'slam'):
        return [
            LogInfo(msg='[sentry_bringup] WARNING: odin_mode=odom/slam in nav mode may accumulate odometry drift.'),
            LogInfo(msg='[sentry_bringup] Recommendation: use odin_mode:=relocalization with odin_relocalization_map:=/abs/path/to/map.bin'),
            LogInfo(msg=f'[sentry_bringup] Odin mode set to {odin_mode} (custom_map_mode={custom_map_mode})'),
            LogInfo(msg='[sentry_bringup] Updated config files: ' + ', '.join(updated_files) if updated_files else '[sentry_bringup] No control_command.yaml found to update'),
        ]

    return [
        LogInfo(msg=f'[sentry_bringup] Odin mode set to {odin_mode} (custom_map_mode={custom_map_mode})'),
        LogInfo(msg='[sentry_bringup] Updated config files: ' + ', '.join(updated_files) if updated_files else '[sentry_bringup] No control_command.yaml found to update'),
    ]


def generate_launch_description():
    # ========================================================================
    # 参数配置
    # ========================================================================
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='nav',
        description='运行模式: "nav" (定位+导航) 或 "mapping" (仅建图)'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='仿真模式: 为 true 时跳过驱动/定位/地形分析，仅启动 Nav2 + GVC 仿真器'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='point_lio',
        description='定位后端: fast_lio, fast_lio, point_lio'
    )

    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='mid360',
        description='雷达类型: mid360 | odin1'
    )

    odin_mode_arg = DeclareLaunchArgument(
        'odin_mode',
        default_value='odom',
        description='odin1内置模式: odom | slam | relocalization'
    )

    odin_relocalization_map_arg = DeclareLaunchArgument(
        'odin_relocalization_map',
        default_value='',
        description='odin1重定位地图绝对路径，仅当 odin_mode=relocalization 生效'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value=PathJoinSubstitution([FindPackageShare("rm_bringup"), "PCD", "RMUC", "Mesh.pcd"]),
        description='3D 点云地图路径 (用于定位)'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([FindPackageShare("rm_bringup"), "PCD", "RMUC", "map.yaml"]),
        description='2D 栅格地图路径 (用于导航)'
    )

    # 子系统开关
    driver_arg = DeclareLaunchArgument(
        'driver', default_value='true', description='启动雷达驱动'
    )
    comm_arg = DeclareLaunchArgument(
        'comm', default_value='false', description='启动通信节点'
    )
    decision_arg = DeclareLaunchArgument(
        'decision', default_value='false', description='启动决策节点'
    )
    terrain_arg = DeclareLaunchArgument(
        'terrain', default_value='false', description='启动地形分析'
    )
    region_detector_arg = DeclareLaunchArgument(
        'region_detector', default_value='true', description='启动区域检测节点'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='启动 RViz'
    )

    # ========================================================================
    # 1. 驱动
    # ========================================================================
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 
                'launch_ROS2', 
                'msg_MID360_launch.py'
            ])
        ),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('driver'), "' == 'true' and '",
                LaunchConfiguration('lidar'), "' == 'mid360'"
            ])
        )
    )

    odin_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odin_ros_driver'),
                'launch',
                'odin1_ros2.launch.py'
            ])
        ),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('driver'), "' == 'true' and '",
                LaunchConfiguration('lidar'), "' == 'odin1'"
            ])
        )
    )

    odin_base_to_robot_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odin1_base_to_base_link',
        arguments=['-0.118', '0.028', '0.2093', '0', '0', '0', 'odin1_base_link', 'base_link'],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('lidar'), "' == 'odin1'"])
        ),
    )

    # ========================================================================
    # 2. 定位与建图
    # ========================================================================
    # 模式: nav -> slam_and_localize (定位 + 里程计)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rm_bringup'), 'launch', 'slam_and_localize.launch.py'])
        ),
        launch_arguments={
            'backend': LaunchConfiguration('backend'),
            'map': LaunchConfiguration('map_pcd'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('lidar'), "' == 'mid360'"
            ])
        )
    )
    
    # 模式: mapping -> slam_mapping_only (建图 + 里程计)
    mapping_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rm_bringup'), 'launch', 'slam_mapping_only.launch.py'])
        ),
        launch_arguments={
            'backend': LaunchConfiguration('backend'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'mapping' and '",
                LaunchConfiguration('lidar'), "' == 'mid360'"
            ])
        )
    )

    # ========================================================================
    # 3. 地形分析 (仅在 nav 模式下)
    # ========================================================================
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_terrain_analysis'), 
                'launch', 
                'traversability_pointcloud.launch.py'
            ])
        ),
        launch_arguments={
            'input_topic': PythonExpression([
                "'/odin1/cloud_slam' if '", LaunchConfiguration('lidar'), "' == 'odin1' else '/cloud_registered_body'"
            ]),
            'rviz': 'false'
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('terrain'), "' == 'true'"
            ])
        )
    )

    # ========================================================================
    # 4. 区域检测 (仅在 nav 模式下，用于检测颠簸区域等)
    # ========================================================================
    region_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_terrain_analysis'), 
                'launch', 
                'region_detector.launch.py'
            ])
        ),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('region_detector'), "' == 'true'"
            ])
        )
    )

    # ========================================================================
    # 5. Nav2 导航栈 (仅在 nav 模式下)
    # ========================================================================
    # 等待地图加载？通常 Nav2 栈会处理自己的生命周期。
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_client_cpp'), 
                'launch', 
                'nav2_stack_with_gvc.launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml'),
            'gvc_odom_topic': PythonExpression([
                "'/odin1/odometry' if '", LaunchConfiguration('lidar'), "' == 'odin1' else '/odom'"
            ]),
            'gvc_base_frame': 'base_link',
            # 所有 odin 模式都先发布静态 map->odom (identity) 到 /tf_static，
            # 保证 map 帧在启动初期就存在，Nav2 costmap 才能初始化。
            # 驱动在 relocalization 成功后会通过 /tf 动态发布 map->odom (已修正方向)，
            # tf2 在 lookup 时会优先用更新的动态 TF，因此重定位的修正不会丢失。
            'publish_map_to_odom_tf': PythonExpression([
                "'true' if '", LaunchConfiguration('lidar'), "' == 'odin1' else 'false'"
            ]),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'nav'"])
        )
    )

    # ========================================================================
    # 5b. 仿真导航栈 (sim=true 时替代 driver/localization/nav)
    # ========================================================================
    sim_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_client_cpp'),
                'launch',
                'nav2_stack_with_gvc_sim.launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map_yaml'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # ========================================================================
    # 6. 决策模块 (行为树)
    # ========================================================================
    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_decision'), 
                'launch', 
                'bt.launch.py'
            ])
        ),
        # Do not open web viewer by default to save resources? 
        # launch_arguments={'use_web_viewer': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('decision'))
    )

    # ========================================================================
    # 7. 通信模块
    # ========================================================================
    comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rm_communication'), 
                'launch', 
                'communication_bringup.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('comm'))
    )

    # ========================================================================
    # 全局可视化
    # ========================================================================
    # 注意: RViz 的启动逻辑已下放到 slam_and_localize 和 slam_mapping_only 中。
    # 它们会根据传入的 'rviz' 参数决定是否启动 RViz。
    # 这样可以复用各模块自带的比较完善的 RViz 配置。

    return LaunchDescription([
        mode_arg,
        sim_arg,
        backend_arg,
        lidar_arg,
        odin_mode_arg,
        odin_relocalization_map_arg,
        map_pcd_arg,
        map_yaml_arg,
        driver_arg,
        comm_arg,
        decision_arg,
        terrain_arg,
        region_detector_arg,
        rviz_arg,

        OpaqueFunction(function=_configure_odin_mode),

        # 真机 / 建图路径
        GroupAction(
            actions=[
                livox_driver,
                odin_driver,
                odin_base_to_robot_base_tf,
                localization_launch,
                mapping_launch,
                terrain_launch,
                region_detector_launch,
                nav_launch,
                decision_launch,
                comm_launch,
            ],
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),

        # 仿真路径：仅 Nav2 + GVC 仿真器
        GroupAction(
            actions=[
                sim_nav_launch,
            ],
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
    ])
