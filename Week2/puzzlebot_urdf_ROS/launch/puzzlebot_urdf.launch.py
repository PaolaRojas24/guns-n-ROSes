import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    # ── Package & file paths ──────────────────────────────────────────────────
    pkg_share = get_package_share_directory('puzzlebot_urdf_ROS')
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    # Read the URDF as a string (required by robot_state_publisher)
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

    # 1. robot_state_publisher
    #    Reads the URDF and publishes static TFs + /robot_description topic.
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

    # 2. joint_state_publisher
    #    Publishes default (zero) joint states so robot_state_publisher can
    #    broadcast all TFs even without a real driver running.
    #    → Replace with joint_state_publisher_gui if you want sliders in RViz.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 3. Your custom TF / marker publisher node
    puzzlebot_publisher_node = Node(
        package='puzzlebot_urdf_ROS',
        executable='puzzlebot_publisher',   # entry-point name in setup.py
        name='puzzlebot_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. RViz2  (conditional on use_rviz argument)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=__import__('launch.conditions', fromlist=['IfCondition'])
                  .IfCondition(use_rviz)
    )

    # ── LaunchDescription ─────────────────────────────────────────────────────
    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        puzzlebot_publisher_node,
        rviz_node,
    ])
