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
        # Node(
        #     package='gscam',
        #     executable='gscam_node',
        #     name='camera',
        #     parameters=[
        #         {'camera_name': 'camera'},
        #         {'gscam_config': 'v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! videoconvert'},
        #         {'frame_id': 'camera_frame'},
        #         {'camera_info_url': f'file://{camera_info_path}'},
        #         {'sync_sink': True},
        #     ],
        #     remappings=[
        #         ('/camera/image_raw', '/camera/image_raw'),
        #         ('/camera/camera_info', '/camera/camera_info'),
        #     ],
        # ),
        Node(
            package='gscam',
            executable='gscam_node',
            name='camera',
            parameters=[
                {'camera_name': 'camera'},
                {'gscam_config': 'v4l2src device=/dev/video0 io-mode=2 ! '
                 'image/jpeg,width=1280,height=720,framerate=30/1 ! '
                 'jpegdec ! videoscale ! video/x-raw,width=640,height=360 ! '
                 'videoconvert ! video/x-raw,format=BGR ! videoconvert'},

                {'frame_id': 'camera_frame'},
                {'camera_info_url': f'file://{camera_info_path}'},
                {'sync_sink': True},
                {'preroll': True},
                {'use_gst_timestamps': False},
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ],
            output='screen',
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
                # 'width': 640,
                # 'height': 360,
                'width': 426,
                'height': 240,
                # 'width': 320,
                # 'height': 180,
                'rate': 10.0,
                'jpeg_quality': 50
            }]
        ),

        # Node(
        #     package='skysight_360',
        #     executable='yolo_node',
        #     name='yolo_node',
        # ),
        Node(
            package='skysight_360',
            executable='mobilenet_node',        # entry point from setup.py
            name='mobilenet_node',
            output='screen',
            parameters=[
                # model files auto-resolve from share/skysight_360/models
                # or supply absolute paths if you prefer:
                # {'prototxt_path': '/abs/path/MobileNetSSD_deploy.prototxt'},
                # {'caffemodel_path': '/abs/path/MobileNetSSD_deploy.caffemodel'},

                {'conf_thresh': 0.5},
                {'frame_skip': 0},               # start responsive
                {'person_only': True},           # <--- ONLY person
                {'draw_all_classes': False},     # skip drawing other classes
                {'max_skip_no_person': 5},       # when idle, go up to 1 in 5 frames
                {'no_person_boost_after': 15},   # after 15 frames with no person, increase skip
            ],
        ),

    ])
