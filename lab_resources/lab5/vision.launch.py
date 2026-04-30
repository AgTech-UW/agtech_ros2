from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the tag config (committed to repo)
    tag_config = '/workspaces/agtech_ros2/lab_resources/lab5/tags.yaml'

    return LaunchDescription([
        # 1. CAMERA DRIVER
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'default_cam',
                'camera_info_url':
                    'file:///root/.ros/camera_info/default_cam.yaml',
            }],
        ),

        # 2. APRILTAG DETECTOR
        # Listens to /image_raw/compressed (not /image_raw) for efficiency.
        # This is the same pattern you'll use for the ceiling camera in Lab 9.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            parameters=[
                tag_config,
                {'image_transport': 'compressed'},
            ],
            remappings=[
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
        ),
    ])