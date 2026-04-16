from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera
        Node(
            package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
            parameters=[{
                'video_device': '/dev/video2',
                'image_width': 1920, 'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 30.0,
                'frame_id': 'camera_link',
                'camera_name': 'default_cam',
                'camera_info_url': 'file:///root/.ros/camera_info/default_cam.yaml',
            }],
        ),
        # Tune camera 3 seconds after launch (usb_cam resets on startup)
        TimerAction(
            period=3.0,
            actions=[ExecuteProcess(cmd=['bash', '-c',
                'v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1 && '
                'v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=80 && '
                'v4l2-ctl -d /dev/video2 --set-ctrl=gain=10 && '
                'v4l2-ctl -d /dev/video2 --set-ctrl=brightness=1'
            ])],
        ),
        # AprilTag detector
        Node(
            package='apriltag_ros', executable='apriltag_node', name='apriltag',
            parameters=[{
                'family': '36h11', 'size': 0.150, 'max_hamming': 0,
                'z_up': True, 'decimate': 2.0, 'threads': 4,
                'image_transport': 'compressed',
            }],
            remappings=[('image_rect', '/image_raw'),
                        ('camera_info', '/camera_info')],
        ),
    ])
