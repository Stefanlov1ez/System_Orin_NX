from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    show_image_arg = DeclareLaunchArgument(
        'showImage',
        default_value='false',
        description='Whether to show images'
    )
    theta_node = Node(
        package="receive_theta",
        executable="receiveTheta",
        name="receiveTheta",
        output="screen",
        parameters=[{
            "imuTopicName": "/imu/data",
            "topBottonMargin": 160,
            "imageLatency": 0.21,
            "compQuality": 50,
            "alwaysPubCompImage": False,
            "showImage": LaunchConfiguration('showImage'),
        }]
    )

    detect_node = Node(
        package="face_detection",
        executable="yunet_detection"
        )
    launch_description = LaunchDescription([show_image_arg, theta_node, detect_node])
    return launch_description
