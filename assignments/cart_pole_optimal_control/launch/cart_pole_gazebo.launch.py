from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # path related
    pkg_cart_pole = FindPackageShare('cart_pole_optimal_control')
    model_path = PathJoinSubstitution([pkg_cart_pole, 'models'])
    sdf_path = PathJoinSubstitution([model_path, 'cart_pole', 'model.sdf'])

    '''
    Use local gz(e.g., gz-gardan)
    
    WARNING: Might be Not COMPATIBLE with ros-gz-sim
    '''
    # gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', 'r', 'empty.sdf'],
    #     output='screen'
    # )

    '''
    Use ROS2 default gz (gz-fortress for ROS2 humble)
    
    REQUIRED: 
    sudo apt-get install \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-interfaces
    '''

    # set gz_sim.launch.py path
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(
        ros_gz_sim_share, 'launch', 'gz_sim.launch.py')

    # cmd format: ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            # 'gz_version': '7',      # if you want to use gz garden(but there's a bug)
            'gz_args': '-r empty.sdf',
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',    # explicit world name to avoid error
            '-name', 'cart_pole',
            '-file', sdf_path,
        ],
        output='screen',
        emulate_tty=True,
    )

    cart_pole_controller = Node(
        package='cart_pole_optimal_control',
        executable='cart_pole_lqr',
        name='cart_pole_lqr',
        output='screen',
        parameters=[{
            'mass_cart': 1.0,
            'mass_pole': 0.1,
            'pole_length': 1.0,
            'gravity': 9.81,
            'Q_x': 1.0,
            'Q_theta': 10.0,
            'R': 1.0,
        }],
        emulate_tty=True
    )

    # bridges
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/cart_pole/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/cart_pole/joint/cart_slider/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        remappings=[
            ('/model/cart_pole/joint_state', '/cart_pole/joint_states'),
            ('/model/cart_pole/joint/cart_slider/cmd_vel',
             '/cart_pole/cart_slider_cmd'),
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        cart_pole_controller,
        bridge
    ])
