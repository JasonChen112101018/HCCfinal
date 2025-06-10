from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tello_controller_node', executable='tello_controller'),
        Node(package='yolov11_detector_node', executable='yolov11_detector'),
        Node(package='apriltag_detector_node', executable='apriltag_detector'),
        Node(package='navigation_node', executable='navigation'),
        Node(package='mission_manager_node', executable='mission_manager'),
    ])
