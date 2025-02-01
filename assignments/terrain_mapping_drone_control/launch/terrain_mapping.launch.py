from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """Generate launch description for terrain mapping system."""
    
    # Get the path to this package's share directory
    pkg_share = FindPackageShare(package='terrain_mapping_drone_control').find('terrain_mapping_drone_control')
    
    # Path to PX4 SITL launch file
    px4_dir = FindPackageShare(package='px4_ros_com').find('px4_ros_com')
    px4_launch_dir = os.path.join(px4_dir, 'launch')
    
    # Include PX4 SITL launch
    px4_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(px4_launch_dir, 'px4_sitl.launch.py')
        ),
        launch_arguments={
            'world': 'bishop_fault_scarp',  # Custom world for Bishop Fault Scarp
            'verbose': 'true'
        }.items()
    )
    
    # Launch ORBSLAM3 node
    orbslam3_node = Node(
        package='orbslam3_ros2',
        executable='orbslam3_mono_node',
        name='orbslam3_mono',
        output='screen',
        parameters=[{
            'camera_config_file': os.path.join(pkg_share, 'config', 'camera_calibration.yaml'),
            'vocabulary_file': os.path.join(pkg_share, 'config', 'ORBvoc.txt'),
            'use_viewer': True,
            'save_trajectory': True
        }]
    )
    
    # Launch terrain mapping controller
    controller_node = Node(
        package='terrain_mapping_drone_control',
        executable='terrain_mapping_controller',
        name='terrain_mapping_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'terrain_mapping_params.yaml')
        ]
    )
    
    # Create and return launch description
    return LaunchDescription([
        px4_sitl_launch,
        orbslam3_node,
        controller_node
    ]) 