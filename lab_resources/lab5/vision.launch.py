from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the Camera Driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'mjpeg2rgb',
                'camera_info_url': 'file:///root/.ros/camera_info/default_cam.yaml'
            }]
        ),
        # 2. Start the AprilTag Detector
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info')
            ],
            parameters=[
                # Note: In a real launch file, we'd load the yaml file path here
                {'family': '36h11', 'size': 0.05} 
            ]
        )
    ])