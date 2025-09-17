from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package's share directory
    package_share_dir = get_package_share_directory('skysight_360')
    # Construct relative path to camera_info.yaml
    camera_info_path = os.path.join(package_share_dir, 'config', 'camera_info.yaml')

    return LaunchDescription([
        # GSCam node
        Node(
            package='gscam',
            executable='gscam_node',
            name='camera',
            parameters=[
                {'camera_name': 'camera'},
                {'gscam_config': 'v4l2src device=/dev/video4 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! videoconvert'},
                {'frame_id': 'camera_frame'},
                {'camera_info_url': f'file://{camera_info_path}'},
                {'sync_sink': True},
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ],
        ),

        # Image transport compressor
        Node(
            package='image_transport',
            executable='republish',
            name='image_transport_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image_raw'),
                ('out', '/camera/image_raw/compressed'),
            ],
            parameters=[{
                'jpeg_quality': 30,     # default 95, Lower = smaller file size, higher = better quality.
                'png_level': 1,         # default 3, Higher = more compression, slower.
            }]

        ),

        # Image_proc for rectification / color processing
        # Node(
        #     package='image_proc',
        #     executable='rectify_node',
        #     name='rectify',
        #     namespace='camera',
        #     remappings=[
        #         ('image', '/camera/image_raw'),
        #         ('camera_info', '/camera/camera_info'),
        #         ('image_rect', '/camera/image_rect')
        #     ],
        # ),

        Node(
            package='skysight_360',
            executable='yolo_node',
            name='yolo_node',
            # parameters=[{'model_path': '/absolute/path/to/your_model.pt'}]
        )

    ])
