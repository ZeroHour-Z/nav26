
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import subprocess
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')
    
    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )
    
    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )
    
    # USB power-cycle: recover Odin from dirty state after power loss
    # This is equivalent to physical unplug/replug via sysfs authorized=0/1
    usb_reset_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'script', 'usb_reset_odin.sh')
    # Fallback: try installed share directory
    if not os.path.isfile(usb_reset_script):
        usb_reset_script = os.path.join(package_dir, 'script', 'usb_reset_odin.sh')

    usb_reset_action = ExecuteProcess(
        cmd=['bash', usb_reset_script],
        name='usb_reset_odin',
        output='screen',
    )

    # Resolve config_file path at generation time so it works both standalone
    # and when included via IncludeLaunchDescription (avoids LaunchConfiguration
    # scoping issues with TimerAction).
    default_config_file = os.path.join(package_dir, 'config', 'control_command.yaml')

    # Create main node — delayed to allow USB reset to complete
    host_sdk_node = TimerAction(
        period=7.0,  # wait for USB hub power-cycle (3s settle + 2s re-enumerate + margin)
        actions=[Node(
            package='odin_ros_driver',
            executable='host_sdk_sample',
            name='host_sdk_sample',
            output='screen',
           # arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'config_file': default_config_file
            }]
        )]
    )

    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f) 
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # Cloud reprojection node
    reprojection_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f) 
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path 
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',  
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # Image overlay node - overlays reprojected points on camera image
    overlay_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',  
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    # Create RViz2 node - loads specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)  # Add RViz configuration argument
    ld.add_action(usb_reset_action)  # Power-cycle Odin USB before driver starts
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(image_overlay_node)
    ld.add_action(rviz_node)  # Add RViz node
    
    return ld
