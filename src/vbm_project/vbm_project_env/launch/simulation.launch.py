import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Define the package names and directories
    vbm_pkg_name = 'vbm_project_env'
    vbm_pkg_share = get_package_share_directory(vbm_pkg_name)

    # Gazebo world path
    world_path = os.path.join(vbm_pkg_share, 'worlds', 'simulation.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items(),
    )

    # Robot description from manipulator xacro file
    robot_urdf_path = os.path.join(vbm_pkg_share, 'description', 'manipulator.urdf.xacro')
    robot_description_content = xacro.process_file(robot_urdf_path).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Entity spawn node for the robot
    spawn_robot_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    # Camera entity spawn node
    camera_urdf_path = os.path.join(vbm_pkg_share, 'urdf', 'camera.urdf')
    spawn_camera_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', camera_urdf_path, '-entity', 'camera'],
        output='screen'
    )

    # Joint state broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Joint trajectory controller
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_robot_entity,
        spawn_camera_entity,
        spawn_broadcaster,
        spawn_controller,
        # Uncomment if RViz is needed
        # node_rviz,
    ])





