# Sky Sight

This project focuses on developing an intelligent drone platform that integrates computer vision with autonomous flight control. The drone is equipped with an onboard camera system that continuously captures real-time video data, which is processed using vision algorithms to detect paths, obstacles, and objects of interest.

## build
```
colcon build --packages-select skysight_360
```

## Launch Instructions

### 1. Run MAVROS for Multiple UAVs
```bash
ros2 launch perahive_mavros mavros.launch.py uavs:="1,2"
```

### 2. Start the Simulator
```bash
ros2 launch perahive_mavros simulator.launch.py uavs:=1,2 base_port:=14550 bind_ip:=0.0.0.0
```


## 📊 Visualization with Foxglove

### Install Foxglove Bridge
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

### Launch Foxglove Bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Run as a System Service
```bash
sudo systemctl daemon-reload
sudo systemctl enable foxglove_bridge.service
sudo systemctl start foxglove_bridge.service
sudo systemctl restart foxglove_bridge.service
```


## 📌 Notes
- Make sure your UAVs (simulated or real) are properly connected with **MAVROS**.
- Default namespaces are `/uav1`, `/uav2`, `/uav3`.  


## Example Workflow

```bash
# Build the workspace
cd ws
colcon build && source install/setup.bash

# Start MAVROS for 2 UAVs
ros2 launch perahive_mavros mavros.launch.py uavs:="1"

# Run simulator with 2 UAVs
ros2 launch perahive_mavros simulator.launch.py uavs:="1" base_port:=14550 bind_ip:=127.0.0.1

# Start the swarm controlimport rclpy
ros2 launch skysight_360 skysight.launch.py



ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/camera/image_raw -r camera:=/camera


# (Optional) View data in Foxglove
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Run Simulation

```
source ~/Projects/swarm_drone/venv/bin/activate
```

### UAV1 (SYSID 1) → send to 14550 (GCS?) and 14551 (MAVROS uav1)
```
python3 ~/Projects/ardupilot_ws/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -I0 --sysid 1 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
```


## Setup Camera
For manual checks
```
sudo apt install v4l-utils

v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext


sudo apt install mpv
mpv /dev/video4
```

install ros package
```
sudo apt install ros-jazzy-v4l2-camera
sudo apt install ros-jazzy-image-pipeline
```

validating installation
```
ros2 pkg executables image_proc
sudo apt install ros-jazzy-gscam
sudo apt install ros-jazzy-image-transport
```

Intsall YOLO
```
python3 -m pip install ultralytics
