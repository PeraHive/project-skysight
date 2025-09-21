from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([
     
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

        # --- Foxglove Bridge Node ---
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,  # Default Foxglove port
                'address': '0.0.0.0',  # Listen on all interfaces
                # 'max_qos_depth': 10,
                # 'num_threads': 0,  # 0 = number of CPU cores
                # 'topic_whitelist': [  # Optional: specify which topics to bridge
                #     '/detected/offset',
                #     '/uav1/mavros/state',
                #     '/uav1/mavros/local_position/pose',
                #     # Add other relevant topics here
                # ],
            }]
        ),
    ])