from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_controller_node',
            executable='tello_controller_node',
            name='tello_controller',
            output='screen'
        ),
        Node(
            package='yolov11_detector_node',
            executable='yolov11_detector_node',
            name='yolov11_detector',
            output='screen'
        ),
        Node(
            package='apriltag_detector_node',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen'
        ),
        Node(
            package='navigation_node',
            executable='navigation_node',
            name='navigation',
            output='screen'
        ),
        Node(
            package='mission_manager_node',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen'
        ),
    ])
