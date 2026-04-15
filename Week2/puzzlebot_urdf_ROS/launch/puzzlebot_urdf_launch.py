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

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.5', '0', '0', '0', '0', 'map', 'odom']
    )

    static_transform_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

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
    
    puzzlebot_joint_state_publisher_node = Node(
    package='puzzlebot_urdf_ROS',
    executable='puzzlebot_joint_state_publisher',
    name='puzzlebot_joint_state_publisher',
    output='screen',
    parameters=[{
        'omega_wheel_r': 2.0,
        'omega_wheel_l': -1.5,  
        'timer_period': 0.05,
        }]
    )

    puzzlebot_odometry_node = Node(
    package='puzzlebot_urdf_ROS',
    executable='puzzlebot_odometry',
    name='puzzlebot_odometry',
    output='screen',
    parameters=[{
        'wheel_radius': 0.05,
        'wheel_base':   0.108,
    }]
)

    rqt_tf_tree_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_tf_tree',
        output='screen',
        arguments=['--standalone', 'rqt_tf_tree']
    )

    rviz_config = os.path.join(
                            get_package_share_directory('puzzlebot_urdf_ROS'),
                            'rviz',
                            'puzzlebot_rviz.rviz'
                            )

    rviz_node = Node(name='rviz',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config]
                    )


    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        static_transform_node,
    	static_transform_node_2,
        robot_state_publisher_node,
        puzzlebot_joint_state_publisher_node,
        puzzlebot_odometry_node,
        rqt_tf_tree_node, 
        rviz_node, 
    ])
