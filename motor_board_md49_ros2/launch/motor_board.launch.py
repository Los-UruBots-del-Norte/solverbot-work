import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'motor_board_md49_ros2'

    package_share_dir = get_package_share_directory(package_name)
    params_file = os.path.join(package_share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            output='screen',
            executable='motor_board_node',  # ¡Coincide con el nombre en entry_points!
            name='motor_board_node',
            parameters=[params_file]
        ),
    ])