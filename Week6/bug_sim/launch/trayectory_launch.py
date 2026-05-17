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

    params = os.path.join(
        get_package_share_directory('bug_sim'),
        'config',
        'params.yaml'
    )
 
    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('bug_sim'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()
    
    trayectory_node = Node(
                        package='bug_sim',
                        executable='trayectory_node',
                        name='Trajectory_node',
                        parameters=[params],
                        output='screen',
    )

    control_node = Node(
                        package='bug_sim',
                        executable='PointStabilisation_node',
                        name='control_node',
                        parameters=[params],
                        output='screen',
    )

    localisation_node = Node(
                        package='bug_sim',
                        executable='localisation',
                        name='localisation_node',
                        output='screen',
    )

    joint_state_publisher_node = Node(
                        package='bug_sim',
                        executable='joint_state_pub',
                        name='joint_state_publisher_node',
                        output='screen',
    )

    puzzlebot_sim = Node(
                        package='bug_sim',
                        executable='puzzlebot_sim',
                        name='puzzlebot_sim',
                        output='screen',
    )

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
    )
    
    coords_transform_node = Node(
                        package='bug_sim',
                        executable='coords_transform',
                        name='coords_transform_node',
                        output='screen',
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

    reset_odom = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/reset_odom',
            'std_msgs/msg/Bool', '{data: true}'],
        output='screen',
    )

    l_d = LaunchDescription([
        # Robot 1
        trayectory_node,
        control_node,
        localisation_node,
        joint_state_publisher_node,
        puzzlebot_sim,
        robot_state_pub_node,
        coords_transform_node,
        # Tools
        tree_node,
        reset_odom,
        graph_node,
        ])

    return l_d