# 🛰️ Sky Sight  

Sky Sight is an **intelligent drone platform** that integrates **computer vision (YOLO)** with **autonomous flight control**.  
The system uses ROS 2 (Jazzy) to combine:  

- **MAVROS** for UAV communication (simulated or real).  
- **YOLO-based object detection** for people/obstacles.  
- **Camera pipelines** for real-time video input, preprocessing, and compression.  
- **Foxglove** for visualization and telemetry monitoring.  

---

## 🔧 Build Instructions

### 1. Clone the workspace
```bash
mkdir -p ~/Projects/perahive/src
cd ~/Projects/perahive/src
git clone <your-repo-url>
```

### 2. Build package
```bash
cd ~/Projects/perahive
colcon build --packages-select skysight_360
source install/setup.bash
```

---

## 🚀 Launch Instructions

### 1. Run MAVROS for Multiple UAVs
This sets up MAVROS instances for each drone (with proper namespaces).  
```bash
ros2 launch perahive_mavros mavros.launch.py uavs:="1"
```

### 2. Start the Simulator
Run SITL for multiple UAVs, binding to a local port:  
```bash
ros2 launch perahive_mavros simulator.launch.py uavs:=1 base_port:=14550 bind_ip:=0.0.0.0

ros2 run mavros mavros_node --ros-args --remap __ns:=/uav4 -p fcu_url:="udp://0.0.0.0:14550@"
```

### 3. Launch Sky Sight Vision Pipeline
Starts the camera capture, preprocessing, and YOLO node:  
```bash
ros2 launch skysight_360 skysight.launch.py
```

---

## 📊 Visualization with Foxglove

### Install Foxglove Bridge
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

### Launch Foxglove Bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### (Optional) Run as a System Service
```bash
sudo systemctl daemon-reload
sudo systemctl enable foxglove_bridge.service
sudo systemctl start foxglove_bridge.service
sudo systemctl restart foxglove_bridge.service
```

---

## 📌 Notes & Tips
- UAVs (simulated or real) must be properly connected via **MAVROS**.  
- Default UAV namespaces are `/uav1`, `/uav2`, `/uav3`.  
- Camera pipeline uses **gscam** → **preprocess (resize/throttle)** → **YOLO**.  
- YOLO outputs:  
  - `/camera/image_yolo` → annotated image stream.  
  - `/person_offset` → offset of detected person (as `geometry_msgs/Point`).  

---

## 🔄 Example Workflow

```bash
# 1. Build the workspace
cd ~/Projects/perahive
colcon build && source install/setup.bash

# 2. Start MAVROS for 1 UAV
ros2 launch perahive_mavros mavros.launch.py uavs:="1"

# 3. Run simulator for UAV1
ros2 launch perahive_mavros simulator.launch.py uavs:="1" base_port:=14550 bind_ip:=0.0.0.0

ros2 run mavros mavros_node  --ros-args --remap __ns:=/uav1 -p fcu_url:="udp://0.0.0.0:14550@"

# 4. Start Sky Sight (camera + YOLO)
ros2 launch skysight_360 skysight.launch.py

# 5. (Optional) View data in Foxglove
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

---

## 🖥️ Run Simulation (ArduPilot SITL)

### Activate Virtual Environment
```bash
source ~/Projects/swarm_drone/venv/bin/activate
```

### Launch UAV1 (SYSID 1)
Send telemetry to MAVROS + GCS:
```bash
python3 ~/Projects/ardupilot_ws/Tools/autotest/sim_vehicle.py   -v ArduCopter -I0 --sysid 1   --out=udp:192.168.7.73:14550 --out=udp:127.0.0.1:14550 
```

For multiple UAVs, increment `-I` and ports.

---

## 🎥 Camera Setup

### Install utilities
```bash
sudo apt install v4l-utils mpv
```

### List cameras
```bash
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

### Preview feed
```bash
mpv /dev/video0
```

---

## 📦 ROS Camera Packages

Install required ROS 2 camera tools:
```bash
sudo apt install ros-jazzy-v4l2-camera
sudo apt install ros-jazzy-image-pipeline
sudo apt install ros-jazzy-gscam
sudo apt install ros-jazzy-image-transport
```

Check executables:
```bash
ros2 pkg executables image_proc
```

---

## 🧠 Install YOLO

Install ultralytics YOLOv8:
```bash
python3 -m pip install ultralytics
pip3 install "numpy<2,>=1.26.0" --break-system-packages
sudo apt-get install -y python3-opencv ros-jazzy-cv-bridge
```

---

## 🔧 Optional: Camera Calibration
For rectification:
```bash
ros2 run camera_calibration cameracalibrator   --size 8x6 --square 0.025   --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

---

## ✅ Summary
- **MAVROS** handles UAV control.  
- **GSCam / preprocess node** streams video efficiently.  
- **YOLOv8** detects persons and outputs both images + offsets.  
- **Foxglove** provides visualization and telemetry.  


 v4l2-ctl --device=/dev/video0 --set-parm=30
 v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=MJPG


 systemctl status foxglove_bridge.service
sudo systemctl restart foxglove_bridge.service
