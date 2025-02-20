from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Executing all the launchers !!']),
        # Lanzador del Fake Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_fake_node'),
                    'launch',
                    'turtlebot3_fake_node.launch.py'
                    ])
                ])
        ),
        # Lanzador del Lidar URG
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node2'),
                    'launch',
                    'urg_node2.launch.py'
                    ])
                ])
            ),
            # Lanzador de Cartographer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ])
        ),
        # Lanzador de Motor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('motor_board_md49_ros2'),
                    'launch',
                    'motor_board.launch.py'
                ])
            ])
        ),
    ])