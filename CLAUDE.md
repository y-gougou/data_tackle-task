# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

R550PLUS 三全向轮机器人 (R550PLUS Three Omni-wheel Robot) - a ROS-based robot car project with two main components:

- `turn_on_wheeltec_robot/` - ROS package for sensor data collection, odometry, IMU, and web-based control
- `omi_car/` - Embedded firmware for ARM Cortex MCUs (Keil/ARM development environment)

## Build Commands

### ROS Package
```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
```

### Running Nodes
```bash
# Base robot node
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

# Data collection with CSV output
roslaunch turn_on_wheeltec_robot data_collector.launch

# Web control MVP (browser at http://<robot-ip>:8000)
roslaunch turn_on_wheeltec_robot web_control.launch

# Individual Python nodes
rosrun turn_on_wheeltec_robot current_reader.py
rosrun turn_on_wheeltec_robot data_collector.py
rosrun turn_on_wheeltec_robot cmd_vel_web_adapter.py
```

## Architecture

### ROS Package (`turn_on_wheeltec_robot/`)

```
├── src/                      # C++ nodes
│   ├── wheeltec_robot.cpp    # Main base controller node (serial comm)
│   └── Quaternion_Solution.cpp # IMU quaternion math
├── scripts/                  # Python nodes
│   ├── current_reader.py     # Reads current sensing board via serial
│   ├── data_collector.py     # Subscribes to /odom, /imu, /PowerVoltage, /current_data → CSV
│   ├── cmd_vel_web_adapter.py # Web cmd_vel safety adapter (timeout, estop, rate limiting)
│   └── web_dashboard_server.py # Serves web dashboard static files
├── launch/                   # Launch files
│   ├── base_serial.launch    # Included by turn_on_wheeltec_robot.launch
│   ├── turn_on_wheeltec_robot.launch # Base robot + imu + odom
│   ├── data_collector.launch # Base + current + data collection
│   └── web_control.launch    # Rosbridge + web dashboard + adapter
└── web/
    └── index.html            # Browser-based joystick control + monitoring
```

### Data Flow

1. **Sensor Data**: `wheeltec_robot_node` (C++) ← serial → `/odom`, `/imu`, `/PowerVoltage`
2. **Current Sensing**: `current_reader.py` ← serial → `/current_data`
3. **CSV Recording**: `data_collector.py` subscribes to all sensor topics → CSV log
4. **Web Control**: Browser → WebSocket → rosbridge → `/cmd_vel_web` → `cmd_vel_web_adapter.py` → `/cmd_vel` → base

### Serial Devices

| Device | Default Path | Baud Rate | Purpose |
|--------|--------------|-----------|---------|
| Base controller | `/dev/ttyCH343USB0` | 115200 | Motor control, odometry, IMU |
| Current sensing | `/dev/ttyUSB1` | 115200 | Three-channel current reading |

### Embedded Firmware (`omi_car/`)

- Keil ARM MDK project with CMSIS device files
- Flashed via .hex file (R550PLUS_C50C_大车全向轮_2023.12.04.hex)
- Uses VS Code with EIDE extension for development
- Target: ARM Cortex-M series microcontroller

## Key Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/odom` | nav_msgs/Odometry | publish | Odometry data |
| `/imu` | sensor_msgs/Imu | publish | IMU data |
| `/PowerVoltage` | std_msgs/Float32 | publish | Battery voltage |
| `/current_data` | std_msgs/Float32MultiArray | publish | Three-channel current |
| `/cmd_vel_web` | geometry_msgs/Twist | subscribe | Web velocity command |
| `/cmd_vel` | geometry_msgs/Twist | publish | Final motor velocity |
| `/web/estop` | std_msgs/Bool | subscribe | Emergency stop |
| `/web/heartbeat` | std_msgs/Empty | subscribe | Web heartbeat |
| `/web/control_status` | std_msgs/String | publish | Control status |

## Debug Commands

```bash
# List relevant topics
rostopic list | grep -E "odom|imu|PowerVoltage|current|cmd_vel|web"

# Echo sensor data
rostopic echo /odom
rostopic echo /imu
rostopic echo /PowerVoltage
rostopic echo /current_data
rostopic echo /web/control_status

# Check topic rates
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data

# Serial device check
cat /dev/ttyCH343USB0
cat /dev/ttyUSB1
```

## Web Control Safety

`cmd_vel_web_adapter.py` implements three layers of protection:
1. **Command timeout**: 0.5s no new command → stop
2. **Heartbeat timeout**: 1.0s no heartbeat → stop
3. **Speed limiting**: Configurable max linear/angular velocities
4. **E-stop**: Latched stop via `/web/estop`

## CSV Output Format

`data_collector.py` outputs to `/home/wheeltec/R550PLUS_data_collect/log/`:

New format (with time sync and fault labels):
```
timestamp,frame,x,y,z,vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,current0,current1,current2,fault_label
```

### Fault Labels
| Label | Name | Description |
|-------|------|-------------|
| 0 | normal | Normal state |
| 1 | drive_fault | Drive fault (single wheel blockage) |
| 2 | hub_loss | Omni wheel hub loss/damage |
| 3 | shaft_eccentric | Motor shaft eccentricity |
| 4 | voltage_low | Low battery voltage |

### Data Collection Commands
```bash
# Normal state (label 0)
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0

# Drive fault (label 1)
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=1

# Hub loss (label 2)
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=2

# Shaft eccentric (label 3)
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=3

# Low voltage (label 4)
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=4
```

## Web Control Parameters (web_control.launch)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_timeout` | 0.5 | Command timeout stop threshold (s) |
| `heartbeat_timeout` | 1.0 | Heartbeat timeout stop threshold (s) |
| `max_linear_x` | 1.50 | Max forward/back speed |
| `max_linear_y` | 1.00 | Max lateral speed |
| `max_angular_z` | 3.75 | Max angular speed |
| `response_exponent` | 0.70 | Response curve exponent (lower = more sensitive) |
| `min_linear_ratio` | 0.20 | Minimum forward response ratio outside deadband |
| `min_angular_ratio` | 0.24 | Minimum turn response ratio outside deadband |

## Author

杨鹏 - 燕山大学本科毕业设计
