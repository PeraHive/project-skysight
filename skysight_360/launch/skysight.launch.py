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
        # --- Camera (GSCam) ---
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

        # --- Preprocess (resize + JPEG compress to CompressedImage) ---
        Node(
            package='skysight_360',
            executable='preprocess_node',
            name='preprocess_node',
            parameters=[{
                'width': 426,
                'height': 240,
                'rate': 10.0,
                'jpeg_quality': 50
            }]
        ),

        # --- Lightweight detector (publishes offsets; ensure it outputs /detected/offset or change below) ---
        Node(
            package='skysight_360',
            executable='mobilenet_node',        # entry point from setup.py
            name='mobilenet_node',
            output='screen',
            parameters=[
                {'conf_thresh': 0.5},
                {'frame_skip': 0},
                {'person_only': True},
                {'draw_all_classes': False},
                {'max_skip_no_person': 5},
                {'no_person_boost_after': 15},
                # If your node uses a param for the output topic, set it here to `/detected/offset`
                # {'offset_topic': '/detected/offset'},
            ],
        ),

        # --- Yaw control with simple-pid (namespaced MAVROS) ---
        Node(
            package='skysight_360',
            executable='track_node',
            name='track_node',
            output='screen',
            parameters=[
                {'uav_ns': '/uav1'},
                {'guided_mode': 'GUIDED'},      # set to 'OFFBOARD' for PX4
                {'target_alt': 3.0},

                # PID (simple-pid)
                {'Kp': 0.002},
                {'Ki': 0.0},
                {'Kd': 0.0},
                {'yaw_rate_limit': 0.7},        # rad/s
                {'pid_sample_time': 0.0},       # run each control tick

                # Vision topic & normalization
                {'offset_topic': '/detected/offset'},
                {'normalize_error': True},
                {'image_width_px': 640.0},
            ],
        ),
    ])
