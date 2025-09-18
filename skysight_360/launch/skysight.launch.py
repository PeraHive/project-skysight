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

        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_transport_republisher',
        #     arguments=['raw', 'compressed'],
        #     remappings=[
        #         ('in', '/camera/image_raw'),
        #         ('out', '/camera/image_compressed'),
        #     ],
        #     parameters=[{
        #         'jpeg_quality': 30,     # default 95, Lower = smaller file size, higher = better quality.
        #         'png_level': 1,         # default 3, Higher = more compression, slower.
        #     }]

        # ),

        # Node(
        #     package='image_proc',
        #     executable='crop_decimate_node',
        #     name='decimate_for_yolo',
        #     namespace='camera',
        #     remappings=[
        #         ('in/image_raw', '/camera/image_compressed'),
        #         ('in/camera_info', '/camera/camera_info'),
        #         ('out/image_raw', '/camera/image_resized'),
        #         ('out/camera_info', '/camera/camera_info_resized'),
        #     ],
        #     parameters=[{
        #         'decimation_x': 4,
        #         'decimation_y': 4
        #     }]
        # ),

        Node(
            package='skysight_360',
            executable='preprocess_node',
            name='preprocess_node',
            parameters=[{
                'width': 640,
                'height': 360,
                'rate': 3.0,
                'jpeg_quality': 70
            }]
        ),

        Node(
            package='skysight_360',
            executable='yolo_node',
            name='yolo_node',
        ),

    ])
