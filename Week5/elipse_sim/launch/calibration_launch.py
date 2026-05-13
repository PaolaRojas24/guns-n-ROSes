import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('elipse_sim'),
        'config',
        'params.yaml'
    )

    urdf_path = os.path.join(
        get_package_share_directory('elipse_sim'),
        'urdf',
        'puzzlebot.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value=os.path.expanduser('~/calibracion_datos.csv'),
        description='Ruta del CSV de salida'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map']
    )

    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '2', '--y', '1', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    puzzlebot_sim = Node(
        package='elipse_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
    )

    localisation_node = Node(
        package='elipse_sim',
        executable='localisation',
        name='localisation_node',
        parameters=[params],
        output='screen',
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

    coords_transform_node = Node(
        package='elipse_sim',
        executable='coords_transform',
        name='coords_transform_node',
        output='screen',
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    joint_state_pub = Node(
        package='elipse_sim',
        executable='joint_state_pub',
        name='joint_state_publisher_node',
        output='screen',
    )

    data_logger_node = Node(
        package='elipse_sim',
        executable='data_logger',
        name='data_logger_node',
        parameters=[
            params,
            {'output_file': LaunchConfiguration('output_file')}
        ],
        output='screen',
    )

    return LaunchDescription([
        output_file_arg,
        static_tf,
        static_tf_1,
        puzzlebot_sim,
        localisation_node,
        trayectory_node,
        control_node,
        coords_transform_node,
        robot_state_pub,
        joint_state_pub,
        data_logger_node,
    ])
