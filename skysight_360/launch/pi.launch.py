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

        # --- MAVROS node (FCU link) ---
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                # Adjust FCU URL to your setup:
                # - USB: "serial:///dev/ttyAMA0:57600"
                # - UDP (SITL or Pixhawk WiFi): "udp://:14540@127.0.0.1:14557"
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'system_id': 1,
                'component_id': 1,
                'cmd/use_comp_id_system_control': True,
                'conn/heartbeat_mav_type': 'ONBOARD_CONTROLLER',
            }]
        )
    ])
