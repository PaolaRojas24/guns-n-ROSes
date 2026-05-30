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


WORLD_CONFIGS = {
    'maze.world':  'maze.yaml',
    'maze2.world': 'maze2.yaml',
    'maze3.world': 'maze3.yaml',
}

DEFAULT_WORLD = 'maze.world'


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    pkg_final_challenge = get_package_share_directory('final_challenge')
    config_file = os.path.join(
        pkg_final_challenge, 'config', WORLD_CONFIGS[world_name]
    )

    pkg_gazebo  = get_package_share_directory('puzzlebot_gazebo')
    params_file = os.path.join(pkg_final_challenge, 'config', 'params.yaml')
    slam_config = os.path.join(pkg_final_challenge, 'config', 'slam_toolbox.yaml')

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

    # ── Odometría dead-reckoning (odom → base_footprint) ──────────────────────
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

    # ── SLAM Toolbox (publica /map y tf map → odom) ───────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    # ── Misión: secuencia waypoints ───────────────────────────────────────────
    mission_node = Node(
        package='final_challenge',
        executable='mission_node',
        name='mission_node',
        parameters=[params_file, config_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── Planner global: A* sobre el OccupancyGrid de SLAM ─────────────────────
    planner_node = Node(
        package='final_challenge',
        executable='planner_node',
        name='planner_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── Controlador local: Pure Pursuit ───────────────────────────────────────
    controller_node = Node(
        package='final_challenge',
        executable='controller_node',
        name='controller_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── Cámara ArUco ──────────────────────────────────────────────────────────
    camera_node = Node(
        package='final_challenge',
        executable='camera_node',
        name='camera_node',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── SLAM Resetter: limpia el mapa periódicamente / al ver ArUco ───────────
    slam_resetter = Node(
        package='final_challenge',
        executable='slam_resetter',
        name='slam_resetter',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(
        pkg_final_challenge, 'rviz', 'puzzlebot_rviz.rviz'
    )
    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    # ── TF estática world → map. (map → odom lo publica SLAM) ─────────────────
    static_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
            '--frame-id', 'world', '--child-frame-id', 'map',
        ],
    )

    return [
        gazebo_launch,
        robot_launch,
        localisation_node,
        slam_node,
        camera_node,
        mission_node,
        planner_node,
        controller_node,
        slam_resetter,
        rviz_node,
        static_world_map,
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=DEFAULT_WORLD,
        description='Mundo a cargar: maze.world (default), maze2.world, maze3.world'
    )
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])
