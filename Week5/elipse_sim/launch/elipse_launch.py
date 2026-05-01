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
        get_package_share_directory('elipse_sim'),
        'config',
        'params.yaml'
    )
 
    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('elipse_sim'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()


    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [
            '--x', '0', '--y', '0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
            '--frame-id', 'world', '--child-frame-id', 'map'
        ]
    )

    static_transform_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [
            '--x', '2', '--y', '1', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )
    
    trayectory_node = Node(
                        package='elipse_sim',
                        executable='trayectory_node',
                        name='Trajectory_node',
                        parameters=[params],
                        output='screen',
    )

    control_node = Node(
                        package='elipse_sim',
                        executable='PointStabilisation_node',
                        name='control_node',
                        parameters=[params],
                        output='screen',
    )

    localisation_node = Node(
                        package='elipse_sim',
                        executable='localisation',
                        name='localisation_node',
                        output='screen',
    )

    joint_state_publisher_node = Node(
                        package='elipse_sim',
                        executable='joint_state_pub',
                        name='joint_state_publisher_node',
                        output='screen',
    )

    puzzlebot_sim = Node(
                        package='elipse_sim',
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
                        package='elipse_sim',
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

    rviz_config = os.path.join(
        get_package_share_directory('elipse_sim'),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    l_d = LaunchDescription([
        static_transform_node,
        static_transform_node_1,
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
        rviz_node,
        graph_node,
        ])

    return l_d