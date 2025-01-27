from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    controller_gains_arg = DeclareLaunchArgument(
        'controller_gains',
        default_value='{"Kp_linear": 4, "Kd_linear": 0.3, "Kp_angular": 6.5, "Kd_angular": 0.05}',
        description='Controller gains for the boustrophedon controller'
    )

    close_all_after_finished_arg = DeclareLaunchArgument(
        'close_all_after_finished',
        default_value='true',
        description='Shutdown all nodes after controller finishes'
    )

    controller_node = Node(
        package='first_order_boustrophedon_navigator',
        executable='boustrophedon_controller',
        name='boustrophedon_controller',
        # Need explicitly declare parameter type(It's dict which is not allowed)
        parameters=[{
            'controller_gains': ParameterValue(
                LaunchConfiguration('controller_gains'),
                value_type=str
            )
        }]
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )

    on_controller_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[
                LogInfo(
                    msg=["Controller node finished. Shutting down all nodes..."]),
                Shutdown(reason='Controller finished and shutting down all nodes')
            ],
        ),
        condition=IfCondition(LaunchConfiguration('close_all_after_finished'))
    )

    return LaunchDescription([
        # declare args to ros2 param
        controller_gains_arg,
        close_all_after_finished_arg,
        
        # start nodes
        controller_node,
        turtlesim_node,
        
        # register events
        on_controller_exit
    ])
