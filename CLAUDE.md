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

**IMPORTANT**: When running data collection and web control simultaneously, you MUST start them separately to avoid node name conflicts:

```bash
# Terminal 1: Data collection
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0

# Terminal 2: Web control (disable conflicting nodes)
roslaunch turn_on_wheeltec_robot web_control.launch start_base:=false start_current_reader:=false start_data_collector:=false
```

Other commands:
```bash
# Base robot node only
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

# Web control only (includes everything except data collection)
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
│   ├── data_collector.py     # Subscribes to /odom, /imu, /PowerVoltage, /current_data → CSV (uses message_filters for time sync)
│   ├── cmd_vel_web_adapter.py # Web cmd_vel safety adapter + PID cruise control
│   ├── preprocess_data.py      # Data preprocessing (missing values, outliers, normalization)
│   ├── create_sliding_windows.py # Sliding window segmentation
│   ├── validate_dataset.py     # Dataset validation and visualization
│   └── web_dashboard_server.py # Serves web dashboard static files
├── launch/                   # Launch files
│   ├── base_serial.launch    # Included by turn_on_wheeltec_robot.launch
│   ├── turn_on_wheeltec_robot.launch # Base robot + imu + odom
│   ├── data_collector.launch # Data collection launch
│   └── web_control.launch    # Rosbridge + web dashboard + adapter
└── web/
    └── index.html            # Browser-based joystick control + monitoring
```

### Data Flow

1. **Sensor Data**: `wheeltec_robot_node` (C++) ← serial → `/odom`, `/imu`, `/PowerVoltage` @ 20Hz
2. **Current Sensing**: `current_reader.py` ← serial → `/current_data` @ ~5-6Hz
3. **CSV Recording**: `data_collector.py` subscribes independently, records on /odom trigger, stores timestamps for later sync → CSV log with independent timestamps
4. **Web Control**: Browser → WebSocket → rosbridge → `/cmd_vel_web`, `/web/cruise_cmd`, `/web/cruise_enable` → `cmd_vel_web_adapter.py` → `/cmd_vel` → base

### Serial Devices

| Device | Default Path | Baud Rate | Purpose |
|--------|--------------|-----------|---------|
| Base controller | `/dev/ttyCH343USB0` | 115200 | Motor control, odometry, IMU |
| Current sensing | `/dev/ttyUSB0` | 115200 | Three-channel current reading |

### Internal Sampling vs ROS Publishing

- **IMU internal sampling**: 100Hz (MPU6050 chip)
- **ROS topic publish rate**: 20Hz (firmware downsamples)
- **Current sensor**: ~5-6Hz (independent)

This is normal - the firmware does internal filtering at 100Hz but only publishes at 20Hz to save bandwidth.

## Cruise Control

### Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/web/cruise_enable` | std_msgs/Bool | subscribe | Enable/disable cruise mode |
| `/web/cruise_cmd` | geometry_msgs/Twist | subscribe | Set target velocity for cruise |
| `/web/cruise_status` | std_msgs/String | publish | Cruise status feedback |

### Cruise Control Logic

- `/web/cruise_enable:=True` → activates cruise mode
- `/web/cruise_cmd` → sets target velocity (doesn't activate cruise)
- Joystick input (`/cmd_vel_web`) does NOT exit cruise mode
- Cruise can ONLY be exited via:
  1. `/web/cruise_enable:=False`
  2. E-stop (`/web/estop:=True`)
  3. Node shutdown

### Embedded Firmware (`omi_car/`)

- Keil ARM MDK project with CMSIS device files
- Flashed via .hex file (R550PLUS_C50C_大车全向轮_2023.12.04.hex)
- Uses FreeRTOS with tasks: Balance_task (100Hz), MPU6050_task (100Hz), show_task (10Hz), data_task (20Hz)
- Uses VS Code with EIDE extension for development
- Target: ARM Cortex-M series microcontroller

## Key Topics

| Topic | Type | Direction | Purpose | Rate |
|-------|------|-----------|---------|------|
| `/odom` | nav_msgs/Odometry | publish | Odometry data | 20Hz |
| `/imu` | sensor_msgs/Imu | publish | IMU data | 20Hz |
| `/PowerVoltage` | std_msgs/Float32 | publish | Battery voltage | 20Hz |
| `/current_data` | std_msgs/Float32MultiArray | publish | Three-channel current | ~5-6Hz |
| `/cmd_vel_web` | geometry_msgs/Twist | subscribe | Web velocity command | 40Hz |
| `/web/cruise_cmd` | geometry_msgs/Twist | subscribe | Cruise control target speed | - |
| `/web/cruise_enable` | std_msgs/Bool | subscribe | Cruise on/off | - |
| `/web/cruise_status` | std_msgs/String | publish | Cruise status | - |
| `/cmd_vel` | geometry_msgs/Twist | publish | Final motor velocity | 40Hz |
| `/web/estop` | std_msgs/Bool | subscribe | Emergency stop | - |
| `/web/heartbeat` | std_msgs/Empty | subscribe | Web heartbeat | 1Hz |
| `/web/control_status` | std_msgs/String | publish | Control status | - |

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
rostopic echo /web/cruise_status

# Check topic rates
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data

# Send cruise command
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'
rostopic pub /web/cruise_enable std_msgs/Bool '{data: true}'
rostopic pub /web/cruise_enable std_msgs/Bool '{data: false}'

# Serial device check
cat /dev/ttyCH343USB0
cat /dev/ttyUSB0
```

## Web Control Safety

`cmd_vel_web_adapter.py` implements multiple layers of protection:
1. **Command timeout**: 0.5s no new command → stop
2. **Heartbeat timeout**: 1.0s no heartbeat → stop
3. **Speed limiting**: Configurable max linear/angular velocities
4. **E-stop**: Latched stop via `/web/estop`
5. **Cruise priority**: Joystick doesn't override active cruise mode

## CSV Output Format

`data_collector.py` outputs to `/home/wheeltec/R550PLUS_data_collect/log/`:

New format (with independent timestamps for each topic):
```
timestamp,frame,odom_time,imu_time,voltage_time,current_time,x,y,z,vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,current0,current1,current2,fault_label
```

Fields:
- `timestamp`: Record base timestamp (from /odom)
- `frame`: Frame counter
- `odom_time`, `imu_time`, `voltage_time`, `current_time`: Independent timestamps for each topic (for preprocessing sync)
- `x,y,z,vx,vy,vz`: Position and velocity from odometry
- `ax,ay,az,gx,gy,gz`: IMU acceleration and angular velocity
- `voltage`: Battery voltage
- `current0,current1,current2`: Three-channel current
- `fault_label`: Fault label (0-4)

### Fault Labels
| Label | Name | Description |
|-------|------|-------------|
| 0 | normal | Normal state |
| 1 | drive_fault | Drive fault (single wheel blockage) |
| 2 | wheel_slip | Omni wheel hub loss/damage |
| 3 | shaft_eccentric | Motor shaft eccentricity |
| 4 | voltage_low | Low battery voltage |

### Fault Data Collection Commands
```bash
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0  # normal
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=1  # drive_fault
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=2  # wheel_slip
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=3  # shaft_eccentric
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=4  # voltage_low
```

## Data Processing Pipeline

**Important**: `/odom` is 20Hz (not 100Hz). This affects all time-based parameters.

Sliding window parameters:
- Window size: 100 points = **5 seconds** @ 20Hz
- Step size: 50 points = **2.5 seconds**
- Overlap: 50%

Normalization strategy:
- Position/Velocity/Acceleration/Angular velocity/Current: symmetric [-1, 1]
- Voltage: Min-Max [0, 1]

**Time Sync**: `preprocess_data.py` interpolates low-frequency channels (current ~5-6Hz) to odom timeline (20Hz).

### Recommended: One-command pipeline (auto-creates directories)

```bash
# Single file: auto-creates {date}_{fault_label}/raw/ and {date}_{fault_label}/processed/
python process_pipeline.py --csv "/path/to/normal_20260419_154637.csv"

# With fault label and skip first 5 seconds
python process_pipeline.py --csv "xxx.csv" --fault_label 1 --skip_first 5

# Batch process directory
python process_pipeline.py --csv_dir "/path/to/datasets" --fault_label 0
```

### Directory Structure (auto-created)

```
datasets/
└── 2026-04-19_normal/
    ├── raw/
    │   └── normal_20260419_154637.csv
    └── processed/
        ├── processed_*.csv
        ├── X_train.npy, X_test.npy, y_train.npy, y_test.npy
        └── validation_report.txt
```

### Manual step-by-step (alternative)

```bash
# Step 1: Preprocess
python preprocess_data.py --data_dir /path/to/log --pattern '*.csv' --skip_first 5

# Step 2: Sliding windows
python create_sliding_windows.py --data_path /path/to/processed_xxx.csv

# Step 3: Validate
python validate_dataset.py --data_dir /path/to/log
```

## Web Control Parameters (web_control.launch)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `start_base` | true | Start base robot node |
| `start_current_reader` | true | Start current reader node |
| `start_data_collector` | true | Start data collector node |
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
