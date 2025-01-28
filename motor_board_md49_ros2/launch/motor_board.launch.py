from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Enable teleoperation with keyboard'
        ),
        Node(
            package='motor_board_md49_ros2',
            executable='ros_interface',
            name='motor_board_node',
            output='screen',
            parameters=['config/params.yaml']
        ),
        Node(
            condition=LaunchConfiguration('use_teleop') == 'true',
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ]
        )
    ])