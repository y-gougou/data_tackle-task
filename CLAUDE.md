# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

R550PLUS 三全向轮机器人 ROS 包，包含传感器数据采集、Web 控制和定速巡航功能。

## 目录结构

```
turn_on_wheeltec_robot/
├── src/                          # C++ 节点
│   ├── wheeltec_robot.cpp        # 底盘控制（串口通信）
│   └── Quaternion_Solution.cpp    # IMU 四元数解算
├── scripts/                      # Python 节点
│   ├── current_reader.py          # 电流传感器读取
│   ├── data_collector.py          # 数据采集（message_filters 时间同步）
│   ├── cmd_vel_web_adapter.py      # Web 控制适配器 + PID 巡航
│   ├── preprocess_data.py          # 数据预处理
│   ├── create_sliding_windows.py   # 滑动窗口分割
│   ├── validate_dataset.py          # 数据集验证
│   ├── process_pipeline.py         # 一键处理流水线
│   ├── reorganize_data.py          # 数据集整理工具
│   └── web_dashboard_server.py      # Web 页面服务
├── launch/                        # 启动文件
│   ├── turn_on_wheeltec_robot.launch    # 底盘 + IMU + 里程计
│   ├── data_collector.launch           # 数据采集
│   ├── web_control.launch              # Web 控制
│   ├── navigation.launch               # 导航
│   └── mapping.launch                  # SLAM 建图
└── web/
    └── index.html                     # 浏览器控制界面
```

## 启动命令

**同时使用数据采集和 Web 控制时，必须分开启动：**

```bash
# 终端 1：底盘控制
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

# 终端 2：数据采集
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0

# 终端 3：Web 控制（禁用冲突节点）
roslaunch turn_on_wheeltec_robot web_control.launch start_base:=false start_current_reader:=false start_data_collector:=false
```

## 串口设备

| 设备 | 路径 | 波特率 | 用途 |
|------|------|--------|------|
| 底盘控制器 | `/dev/ttyCH343USB0` | 115200 | 电机控制、里程计、IMU |
| 电流传感器 | `/dev/ttyUSB0` | 115200 | 三通道电流 |

## 核心话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/odom` | nav_msgs/Odometry | 20Hz | 里程计数据 |
| `/imu` | sensor_msgs/Imu | 20Hz | IMU 数据（固件内部 100Hz 滤波后 20Hz 发布）|
| `/PowerVoltage` | std_msgs/Float32 | 20Hz | 电池电压 |
| `/current_data` | Float32MultiArray | ~5-6Hz | 三通道电流 |
| `/cmd_vel` | geometry_msgs/Twist | 40Hz | 最终电机速度 |
| `/web/cruise_cmd` | geometry_msgs/Twist | - | 巡航目标速度 |
| `/web/cruise_enable` | std_msgs/Bool | - | 巡航开关 |

## 数据采集

### 13 通道底盘数据 + 3 通道电流数据

位置(x,y,z)、速度(vx,vy,vz)、加速度(ax,ay,az)、角速度(gx,gy,gz)、电压、电流(c0,c1,c2)

### 故障标签

| 标签 | 名称 | 说明 |
|------|------|------|
| 0 | normal | 正常 |
| 1 | drive_fault | 驱动异常（单轮堵转） |
| 2 | wheel_slip | 轮子打滑/损坏 |
| 3 | shaft_eccentric | 电机轴偏心 |
| 4 | battery_low | 电池低压 |

### 采集命令
```bash
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0  # 正常
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=1  # 驱动异常
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=2  # 轮子打滑
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=3  # 电机轴偏心
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=4  # 电池低压
```

## 数据处理流水线

### 一键处理（推荐）
```bash
# 单文件：自动创建 {日期}_{故障类型}/raw/ 和 processed/
python process_pipeline.py --csv "/path/to/file.csv"

# 批量处理目录
python process_pipeline.py --csv_dir "/path/to/datasets" --fault_label 0
```

### 手动分步处理
```bash
python preprocess_data.py --data_dir /path/to/log --pattern '*.csv' --skip_first 5
python create_sliding_windows.py --data_path /path/to/processed_xxx.csv
python validate_dataset.py --data_dir /path/to/log
```

### 数据集整理
```bash
# 将混杂的 CSV/NPY 文件整理为规范目录结构
python reorganize_data.py --src "C:\数据集集合"
```

### 滑动窗口参数
- 窗口大小：100 点 = 5 秒 @ 20Hz
- 步长：50 点 = 2.5 秒
- 重叠率：50%

### 归一化策略
- 位置/速度/加速度/角速度/电流：对称归一化 [-1, 1]
- 电压：Min-Max [0, 1]

## 调试命令

```bash
# 查看话题频率
rostopic hz /odom
rostopic hz /current_data

# 发送巡航命令
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'
rostopic pub /web/cruise_enable std_msgs/Bool '{data: true}'

# 串口检查
ls -l /dev/ttyCH343USB0 /dev/ttyUSB0
```

## 固件 (omi_car/)

- Keil ARM MDK 项目，使用 FreeRTOS
- 任务：Balance_task(100Hz)、MPU6050_task(100Hz)、show_task(10Hz)、data_task(20Hz)
- 通过 .hex 文件烧录

## 作者

杨鹏 - 燕山大学本科毕业设计
