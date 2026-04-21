# launch/puzzlebot_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('puzzlebot_sim3'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='puzzlebot_sim3',
            executable='trayectory_node',
            name='trajectory_node',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='puzzlebot_sim3',
            executable='PointStabilisation_node',
            name='PointStabilisation_node',
            parameters=[params],
            output='screen'
        ),
        
    ])