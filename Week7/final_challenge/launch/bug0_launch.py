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


WORLD_GOALS = {
    'obstacle_avoidance_1.world': (1.45,   1.20),
    'obstacle_avoidance_2.world': (-1.20,  1.50),
    'obstacle_avoidance_3.world': (0.00,  -2.50),
    'obstacle_avoidance_4.world': (0.00,  -2.45),
}
DEFAULT_WORLD = 'obstacle_avoidance_1.world'


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)
    goal_x, goal_y = WORLD_GOALS.get(world_name, (2.00, -2.00))

    pkg_gazebo  = get_package_share_directory('puzzlebot_gazebo')
    pkg_final_challenge = get_package_share_directory('final_challenge')
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
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('wl', '/VelocityEncL'),
            ('wr', '/VelocityEncR'),
        ],
        output='screen',
    )

    # ── joint_state_pub ───────────────────────────────────────────────────────
    joint_state_pub = Node(
        package='final_challenge',
        executable='joint_state_pub',
        name='joint_state_publisher_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen',
    )

    # ── coords_transform ──────────────────────────────────────────────────────
    coords_transform_node = Node(
        package='final_challenge',
        executable='coords_transform',
        name='coords_transform_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── control_node (PID) ────────────────────────────────────────────────────
    control_node = Node(
        package='final_challenge',
        executable='PointStabilisation_node',
        name='control_node',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('odom', '/ground_truth'),
        ],
        output='screen',
    )

    # ── bug0_node ─────────────────────────────────────────────────────────────
    bug0_node = Node(
        package='final_challenge',
        executable='bug0_node',
        name='bug0_node',
        parameters=[
            params_file,
            {
                'goal_x':       goal_x,
                'goal_y':       goal_y,
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('odom',     '/ground_truth'),
            ('setpoint', 'setpoint'), 
        ],
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

    return [
        gazebo_launch,
        robot_launch,
        localisation_node,
        joint_state_pub,
        robot_state_pub,
        coords_transform_node,
        control_node,
        bug0_node,
        graph_node,
        tree_node,
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
