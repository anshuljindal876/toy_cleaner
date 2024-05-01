from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "transform_point_cloud", package='capstone_vision', executable='transform_point_cloud.py', output='screen'),
        launch_ros.actions.Node(
            namespace= "process_point_cloud", package='capstone_vision', executable='processPointCloud', output='screen'),
        launch_ros.actions.Node(
            namespace= "clustering_node", package='capstone_vision', executable='clustering_node.py', output='screen'),
        launch_ros.actions.Node(
            namespace= "camera_reading_node", package='capstone_vision', executable='camera_reading_node.py', output='screen'),
    ])
