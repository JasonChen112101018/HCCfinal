from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_controller_node',
            executable='tello_controller_node_main',
            name='tello_controller',
            output='screen'
        ),
        Node(
            package='yolov11_detector_node',
            executable='yolov11_detector_node_main',
            name='yolov11_detector',
            output='screen'
        ),
        Node(
            package='apriltag_detector_node',
            executable='apriltag_detector_node_main',
            name='apriltag_detector',
            output='screen'
        ),
        Node(
            package='navigation_node',
            executable='navigation_node_main',
            name='navigation',
            output='screen'
        ),
        Node(
            package='mission_manager_node',
            executable='mission_manager_node_main',
            name='mission_manager',
            output='screen'
        )
    ])
