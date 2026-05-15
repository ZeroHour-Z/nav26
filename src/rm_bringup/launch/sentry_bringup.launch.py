from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                               IncludeLaunchDescription, LogInfo, OpaqueFunction,
                               RegisterEventHandler, TimerAction)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import yaml


def _patch_yaml_dict(data: dict, key: str, value) -> dict:
    """Set a nested YAML key to the given value, creating intermediate dicts as needed.
    Returns True if the key was newly created (didn't exist before)."""
    parts = key.split('.')
    d = data
    for part in parts[:-1]:
        if part not in d or not isinstance(d[part], dict):
            d[part] = {}
        d = d[part]
    existed = parts[-1] in d
    d[parts[-1]] = value
    return not existed


def _launch_odin_with_config(context):
    """Patch Odin config for this launch, then launch odin_ros_driver.

    The ROS2 official driver ignores its config_file launch parameter and reads
    config/control_command.yaml directly, so keep driver source code untouched
    and restore the config file when the launch shuts down.
    """
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
    custom_map_mode = int(mode_to_value.get(odin_mode, '0'))

    # Locate config files that the official driver may read. In this workspace
    # host_sdk_sample uses its source-tree path, while helper nodes and installed
    # launches use the package share path.
    this_file = Path(__file__).resolve()
    source_cfg = this_file.parents[2] / 'odin_ros_driver' / 'config' / 'control_command.yaml'
    config_paths = []

    def _add_config_path(path: Path):
        if not path.exists():
            return
        resolved = path.resolve()
        if all(existing.resolve() != resolved for existing in config_paths):
            config_paths.append(path)

    _add_config_path(source_cfg)
    _add_config_path(Path.cwd() / 'src' / 'odin_ros_driver' / 'config' / 'control_command.yaml')
    try:
        _add_config_path(Path(get_package_share_directory('odin_ros_driver')) / 'config' / 'control_command.yaml')
    except Exception:
        pass

    if not config_paths:
        return [
            LogInfo(msg='[sentry_bringup] ERROR: cannot locate control_command.yaml for Odin configuration.')
        ]

    actions = []
    restore_items = []
    try:
        for config_path in config_paths:
            original_text = config_path.read_text(encoding='utf-8')
            data = yaml.safe_load(original_text)
            _patch_yaml_dict(data, 'register_keys.custom_map_mode', custom_map_mode)
            _patch_yaml_dict(data, 'register_keys.relocalization_map_abs_path', relocalization_map)
            if run_mode == 'nav':
                _patch_yaml_dict(data, 'register_keys.use_host_ros_time', 1)

            patched_text = yaml.dump(data, default_flow_style=False, allow_unicode=True)
            if patched_text == original_text:
                actions.append(LogInfo(msg=f'[sentry_bringup] Odin config already matches launch overrides: {config_path}'))
                continue

            config_path.write_text(patched_text, encoding='utf-8')
            restore_items.append((config_path, original_text))
            actions.append(LogInfo(msg=f'[sentry_bringup] Odin config patched for this launch: {config_path}'))
    except Exception as exc:
        for config_path, original_text in restore_items:
            try:
                config_path.write_text(original_text, encoding='utf-8')
            except Exception:
                pass
        return [LogInfo(msg=f'[sentry_bringup] ERROR: failed to patch Odin config: {exc}')]

    if restore_items:
        def _restore_odin_configs(_context):
            restore_actions = []
            for config_path, original_text in restore_items:
                try:
                    config_path.write_text(original_text, encoding='utf-8')
                    restore_actions.append(LogInfo(msg=f'[sentry_bringup] Odin config restored: {config_path}'))
                except Exception as exc:
                    restore_actions.append(LogInfo(msg=f'[sentry_bringup] WARNING: failed to restore Odin config {config_path}: {exc}'))
            return restore_actions

        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=_restore_odin_configs)]
        )))

    actions.append(LogInfo(msg=f'[sentry_bringup] Odin mode: {odin_mode} (custom_map_mode={custom_map_mode})'))
    if run_mode == 'nav':
        actions.append(LogInfo(msg='[sentry_bringup] Odin nav timestamp mode: use_host_ros_time=1'))

    if odin_mode == 'relocalization' and not relocalization_map:
        actions.append(LogInfo(msg='[sentry_bringup] WARNING: odin_mode=relocalization but odin_relocalization_map is empty.'))
        actions.append(LogInfo(msg='[sentry_bringup] Please set odin_relocalization_map:=/abs/path/to/map.bin'))

    if run_mode == 'nav' and odin_mode in ('odom', 'slam'):
        actions.append(LogInfo(msg='[sentry_bringup] WARNING: odin_mode=odom/slam in nav mode may accumulate odometry drift.'))
        actions.append(LogInfo(msg='[sentry_bringup] Recommendation: use odin_mode:=relocalization with odin_relocalization_map:=/abs/path/to/map.bin'))

    # Build odin nodes directly (instead of IncludeLaunchDescription) so the
    # launch-time config patch happens before host_sdk_sample starts.
    odin_pkg = get_package_share_directory('odin_ros_driver')

    # USB power-cycle
    usb_reset_script = os.path.join(odin_pkg, 'script', 'usb_reset_odin.sh')
    if not os.path.isfile(usb_reset_script):
        usb_reset_script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'odin_ros_driver', 'script', 'usb_reset_odin.sh')
    actions.append(ExecuteProcess(
        cmd=['bash', usb_reset_script],
        name='usb_reset_odin',
        output='screen',
    ))

    # host_sdk_sample — the only node that needs the patched config
    # Fix libusb conflict with MVS SDK — prioritize system libusb
    libusb_env = {'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')}
    actions.append(TimerAction(
        period=7.0,
        actions=[Node(
            package='odin_ros_driver',
            executable='host_sdk_sample',
            name='host_sdk_sample',
            output='screen',
            additional_env=libusb_env,
        )]
    ))

    # pcd2depth node (reads original config — custom_map_mode irrelevant to it)
    pcd2depth_config_path = os.path.join(odin_pkg, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    pcd2depth_calib_path = os.path.join(odin_pkg, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path
    actions.append(Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params],
        additional_env=libusb_env,
    ))

    # cloud reprojection node
    reprojection_config_path = os.path.join(odin_pkg, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f)
    reprojection_calib_path = os.path.join(odin_pkg, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path
    actions.append(Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params],
        additional_env=libusb_env,
    ))

    # image overlay node
    overlay_config_path = os.path.join(odin_pkg, 'config', 'control_command.yaml')
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    actions.append(Node(
        package='odin_ros_driver',
        executable='image_overlay_node',
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params],
        additional_env=libusb_env,
    ))

    # RViz for Odin (optional — matches odin1_ros2.launch.py behaviour)
    rviz_config_path = os.path.join(odin_pkg, 'config', 'odin_ros2.rviz')
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_odin',
        output='screen',
        arguments=['-d', rviz_config_path],
    ))
    return actions


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
        description='定位后端: fast_lio, faster_lio, point_lio'
    )

    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='odin1',
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
        'terrain', default_value='true', description='启动地形分析和动态避障'
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
    # 4b. PointCloud2 → LaserScan 转换 (给全局动态避障层用)
    # /traversability/obstacles (PointCloud2, odom) → /traversability/scan (LaserScan, base_link)
    # DynamicScanLayer 会把 LaserScan 直接写入 global costmap，供全局规划绕开动态障碍。
    # ========================================================================
    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='traversability_to_scan',
        output='screen',
        remappings=[('cloud_in', '/traversability/obstacles'),
                    ('scan', '/traversability/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,
            'min_height': -0.3,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745,  # 1 degree
            'scan_time': 0.05,
            'range_min': 0.3,
            'range_max': 10.0,
            'use_inf': True,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'nav' and '",
                LaunchConfiguration('terrain'), "' == 'true'"
            ])
        )
    )

    # ========================================================================
    # 5. Nav2 导航栈 (仅在 nav 模式下)
    # ========================================================================
    # Odin1 需要等待驱动启动后才发布 odom->base_link TF；
    # 延迟 10s 启动 Nav2，避免 local_costmap 初始化时 TF 不可用。
    nav_launch = TimerAction(
        period=PythonExpression([
            "'10.0' if '", LaunchConfiguration('lidar'), "' == 'odin1' else '0.0'"
        ]),
        actions=[
            IncludeLaunchDescription(
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
        ]
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

        OpaqueFunction(function=_launch_odin_with_config),

        # 真机 / 建图路径
        GroupAction(
            actions=[
                livox_driver,
                odin_base_to_robot_base_tf,
                localization_launch,
                mapping_launch,
                terrain_launch,
                region_detector_launch,
                pcl_to_scan_node,
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
