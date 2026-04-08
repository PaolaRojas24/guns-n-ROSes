import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def generate_launch_description():

    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )

    static_transform_node_2 = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '0', '--y', '0', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'world', '--child-frame-id', 'map']
                                )
    
    
    puzzle_node = Node(name="puzzlebot",
                            package='puzzlebot_sim_ROS',
                            executable='joint_state_publisher'
                            )
    
    rviz_config = os.path.join(
                            get_package_share_directory('puzzlebot_sim_ROS'),
                            'rviz',
                            'puzzlebot_rviz.rviz'
                            )

    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config]
                    )
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )

    l_d = LaunchDescription([static_transform_node, static_transform_node_2 , rqt_tf_tree_node, rviz_node, puzzle_node])

    return l_d