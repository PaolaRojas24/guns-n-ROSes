import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('puzzlebot_urdf_ROS')
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'puzzlebot.rviz'),
        description='Path to the RViz2 config file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # ── Nodes ─────────────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    puzzlebot_publisher_node = Node(
        package='puzzlebot_urdf_ROS',
        executable='puzzlebot_publisher',
        name='puzzlebot_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)   # ← clean import, no __import__ hack
    )

    rqt_tf_tree_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_tf_tree',
        output='screen',
        arguments=['--standalone', 'rqt_tf_tree']
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        puzzlebot_publisher_node,
        rviz_node,
        rqt_tf_tree_node,  
    ])
