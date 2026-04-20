#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable
from launch.actions import ExecuteProcess


def generate_launch_description():
 
    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('puzzlebot_sim3'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
    )

    plot_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/pose_sim/pose/position/x', '/pose_sim/pose/position/y'],
                    )
    
    tree_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_tf_tree',
        output='screen',
        arguments=['--standalone', 'rqt_tf_tree']
    )

    graph_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_graph', 'rqt_graph'],
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim3'),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    l_d = LaunchDescription([robot_state_pub_node, plot_node, tree_node, rviz_node, graph_node])

    return l_d
