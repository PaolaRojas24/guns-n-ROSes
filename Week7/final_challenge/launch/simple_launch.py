import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


WORLD_CONFIGS = {
    'maze.world': 'maze.yaml',
    'maze2.world': 'maze2.yaml',
    'maze3.world': 'maze3.yaml',
}

DEFAULT_WORLD = 'maze.world'


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    pkg_final_challenge = get_package_share_directory('final_challenge')
    config_file = os.path.join(
        pkg_final_challenge,
        'config',
        WORLD_CONFIGS[world_name]
    )

    pkg_gazebo  = get_package_share_directory('puzzlebot_gazebo')
    params_file = os.path.join(pkg_final_challenge, 'config', 'params.yaml')

    urdf_path = os.path.join(pkg_final_challenge, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ── Gazebo world ──────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_world_launch.py')
        ),
        launch_arguments={
            'world':     world_name,
            'pause':     'false',
            'verbosity': '1',
        }.items()
    )

    # ── Robot spawn ───────────────────────────────────────────────────────────
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_puzzlebot_launch.py')
        ),
        launch_arguments={
            'robot':        'puzzlebot_jetson_lidar_ed',
            'robot_name':   '',
            'x':            '0.0',
            'y':            '0.0',
            'yaw':          '0.0',
            'prefix':       '',
            'lidar_frame':  'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame':    'tof_link',
            'use_sim_time': 'true',
        }.items()
    )

    # ── localisation_node ─────────────────────────────────────────────────────
    localisation_node = Node(
        package='final_challenge',
        executable='localisation',
        name='localisation_node',
        parameters=[params_file, config_file, {'use_sim_time': True}],
        remappings=[
            ('wl', '/VelocityEncL'),
            ('wr', '/VelocityEncR'),
        ],
        output='screen',
    )

    # ── control_node (PID) ────────────────────────────────────────────────────
    control_node = Node(
        package='final_challenge',
        executable='PointStabilisation_node',
        name='control_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── tangent_bug_node ──────────────────────────────────────────────────────
    tangent_bug_node = Node(
        package='final_challenge',
        executable='tangent_bug_node',
        name='tangent_bug_node',
        parameters=[params_file, config_file],
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

    image_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        output='screen'
    )

    # ── camera_node (detección ArUco) ─────────────────────────────────────────
    camera_node = Node(
        package='final_challenge',
        executable='camera_node',
        name='camera_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    rviz_config = os.path.join(
        get_package_share_directory('final_challenge'),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

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
            '--x', '0', '--y', '0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )

    return [
        gazebo_launch,
        robot_launch,
        localisation_node,
        control_node,
        camera_node,
        image_node,
        tangent_bug_node,
        #graph_node,
        #tree_node,
        rviz_node,
        static_transform_node,
        static_transform_node_1,
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=DEFAULT_WORLD,
        description=(
            'Mundo a cargar: obstacle_avoidance_1.world (default), '
            'obstacle_avoidance_2.world, obstacle_avoidance_3.world, '
            'obstacle_avoidance_4.world'
        )
    )
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])
