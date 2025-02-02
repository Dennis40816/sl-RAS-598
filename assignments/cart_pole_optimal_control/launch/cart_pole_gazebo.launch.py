from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_cart_pole = FindPackageShare('cart_pole_optimal_control')
    
    # Paths to resources
    model_path = PathJoinSubstitution([pkg_cart_pole, 'models'])
    sdf_path = PathJoinSubstitution([model_path, 'cart_pole', 'model.sdf'])
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Spawn the cart pole
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'cart_pole',
                  '-file', sdf_path],
        output='screen'
    )
    
    # Start the LQR controller
    cart_pole_controller = Node(
        package='cart_pole_optimal_control',
        executable='cart_pole_lqr',
        name='cart_pole_lqr',
        output='screen',
        parameters=[{
            # Physical parameters
            'mass_cart': 1.0,    # Mass of the cart in kg
            'mass_pole': 0.1,    # Mass of the pole in kg
            'pole_length': 1.0,  # Length of the pole in meters
            'gravity': 9.81,     # Gravity constant in m/s^2
            
            # LQR cost weights
            'Q_x': 1000.0,       # Position error cost
            'Q_x_dot': 100.0,    # Velocity cost for damping
            'Q_theta': 2000.0,   # Angle error cost
            'Q_theta_dot': 200.0, # Angular velocity cost
            'R': 1.0,           # Increased to get reasonable forces
            
            # Control limits
            'force_limit': 20.0  # Maximum force in Newtons - reasonable for 1kg cart
        }]
    )
    
    # Bridge to convert Gazebo topics to ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint state bridge
            '/world/empty/model/cart_pole/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Command bridge for cart - changed to force command
            '/model/cart_pole/joint/cart_slider/cmd_force@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        remappings=[
            ('/world/empty/model/cart_pole/joint_state', '/cart_pole/joint_states'),
            ('/model/cart_pole/joint/cart_slider/cmd_force', '/cart_pole/cart_slider_cmd'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        cart_pole_controller,
        bridge
    ]) 