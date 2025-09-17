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

# Start the swarm control
ros2 launch shadow shadow.launch.py

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