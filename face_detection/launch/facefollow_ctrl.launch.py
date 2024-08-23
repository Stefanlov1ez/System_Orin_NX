from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    diablo_ctrl_node = Node(
        package="diablo_ctrl",
        executable="diablo_ctrl_node"
        )
    detect_node = Node(
        package="face_detection",
        executable="facefollow_ctrl"
        )
    launch_description = LaunchDescription([diablo_ctrl_node, detect_node])
    return launch_description
