# R550PLUS 三全向轮机器人 ROS 包

## 概述

`turn_on_wheeltec_robot` 包含两条可并行使用的能力：

- **传感器数据采集**：13通道底盘数据 + 3通道电流数据，支持 CSV 采集和故障标签
- **Web 控制 + 定速巡航**：浏览器控制机器人，支持 PID 闭环巡航定速

---

## 数据来源

本系统有 **两个独立串口数据源**：

| 数据源 | 串口 | 话题 | 通道数 | 频率 |
|--------|------|------|--------|------|
| 底盘控制器 | `/dev/ttyCH343USB0` | `/odom`, `/imu`, `/PowerVoltage` | 13通道 | 20Hz |
| 电流传感器 | `/dev/ttyUSB1` | `/current_data` | 3通道 | 20Hz |

**13通道底盘数据**：
- 位置：x, y, z（里程计）
- 速度：vx, vy, vz
- 加速度：ax, ay, az（IMU）
- 角速度：gx, gy, gz（IMU）
- 电压：voltage

**3通道电流数据**：
- current0, current1, current2

---

## 系统结构

```
串口设备 1 (ttyCH343USB0)          串口设备 2 (ttyUSB1)
        │                                    │
        ▼                                    ▼
┌───────────────┐                  ┌─────────────────┐
│wheeltec_robot │                  │ current_reader  │
│    _node      │                  │      .py        │
│    (C++)      │                  └────────┬────────┘
└───────┬───────┘                           │
        │  /odom /imu /PowerVoltage         │  /current_data
        └───────────────────┬───────────────┘
                            │
                   ┌────────▼────────┐
                   │ data_collector  │
                   │      .py        │
                   │ message_filters  │
                   │  时间同步        │
                   └─────────────────┘

浏览器 WebSocket
        │
        ▼
  rosbridge_websocket
        │
        ▼
  /cmd_vel_web + /web/cruise_cmd + /web/heartbeat + /web/estop
        │
        ▼
  cmd_vel_web_adapter.py (PID巡航 + 安全保护)
        │
        ▼
      /cmd_vel → wheeltec_robot_node
```

---

## 故障标签说明

| 标签 | 名称 | 说明 |
|------|------|------|
| 0 | normal | 正常状态 |
| 1 | drive_fault | 驱动异常（单轮堵转） |
| 2 | hub_loss | 轮毂丢失/损坏 |
| 3 | shaft_eccentric | 电机轴偏心 |
| 4 | voltage_low | 电池电压偏低 |

---

## 数据采集流程

### 前提条件

```bash
cd ~/ml_robot_ws
catkin_make
source devel/setup.bash
```

### 方式一：一键启动（推荐）

```bash
# 启动底盘 + 电流采集 + 数据采集
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0
```

此 launch 会自动启动：
- `turn_on_wheeltec_robot.launch`（底盘串口节点）
- `current_reader.py`（电流采集节点）
- `data_collector.py`（CSV记录节点）

### 方式二：分开启动

**终端 1 - 启动底盘**：
```bash
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```

**终端 2 - 启动电流采集**：
```bash
rosrun turn_on_wheeltec_robot current_reader.py
```

**终端 3 - 启动数据采集**：
```bash
rosrun turn_on_wheeltec_robot data_collector.py
```

### 启动 Web 控制（控制机器人移动）

**终端 4**：
```bash
roslaunch turn_on_wheeltec_robot web_control.launch
```

浏览器打开：`http://<robot-ip>:8000`

### 故障数据采集命令

```bash
# 正常状态
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0

# 驱动异常（单轮堵转）
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=1

# 轮毂丢失/损坏
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=2

# 电机轴偏心
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=3

# 电池低压
roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=4
```

采集足够时间后按 `Ctrl+C` 停止。

### 定速巡航使用方法

1. 连接 Rosbridge
2. 在"定速巡航"面板设置目标速度（Vx/Vy/Wz）
3. 点击"启动巡航"
4. 机器人自动保持设定速度
5. 点击"停止巡航"或手动接管退出

rostopic 命令示例：
```bash
# 匀速 0.5m/s 前进
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'

# 取消巡航
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{}'
```

---

## 数据预处理流程

采集完成后，在机器人端或本地执行：

### 步骤一：预处理（缺失值/异常值/归一化）

```bash
python ~/ml_robot_ws/src/turn_on_wheeltec_robot/scripts/preprocess_data.py \
    --data_dir /home/wheeltec/R550PLUS_data_collect/log \
    --pattern '*.csv' \
    --normalize symmetric
```

输出：
- `processed_*.csv` — 处理后的数据
- `*_norm_params.pkl` — 归一化参数

### 步骤二：滑动窗口分割

```bash
python ~/ml_robot_ws/src/turn_on_wheeltec_robot/scripts/create_sliding_windows.py \
    --data_path /home/wheeltec/R550PLUS_data_collect/log/processed_xxx.csv
```

输出：
- `X_train.npy`, `X_test.npy` — 特征窗口
- `y_train.npy`, `y_test.npy` — 标签
- `*_info.pkl` — 数据集信息

### 步骤三：验证数据集

```bash
python ~/ml_robot_ws/src/turn_on_wheeltec_robot/scripts/validate_dataset.py \
    --data_dir /home/wheeltec/R550PLUS_data_collect/log
```

---

## 数据说明

### 采样率

- **实际采样频率：20Hz**（由底盘控制器决定）
- 100点滑动窗口 = **5秒**数据
- 50点步长 = **2.5秒**
- 重叠率：50%

### CSV 字段

```
timestamp,frame,x,y,z,vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,current0,current1,current2,fault_label
```

| 字段 | 说明 | 单位 | 来源 |
|------|------|------|------|
| timestamp | ROS 时间戳 | 秒 | - |
| frame | 帧序号 | - | - |
| x, y, z | 里程计位置 | 米 | 底盘串口 |
| vx, vy, vz | 线速度 | m/s | 底盘串口 |
| ax, ay, az | 加速度（IMU） | m/s² | 底盘串口 |
| gx, gy, gz | 角速度（IMU） | rad/s | 底盘串口 |
| voltage | 电池电压 | V | 底盘串口 |
| current0~2 | 三通道电流 | A | 电流传感器 |
| fault_label | 故障标签 | - | - |

### 默认输出目录

- `/home/wheeltec/R550PLUS_data_collect/log/`

### 归一化策略

| 通道类型 | 归一化方法 | 范围 |
|----------|-----------|------|
| 位置/速度/加速度/角速度/电流 | 对称归一化 | [-1, 1] |
| 电压 | Min-Max归一化 | [0, 1] |

---

## 参数配置

### data_collector.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fault_label` | `0` | 故障标签 |

### data_collector.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~fault_label` | `0` | 故障标签 |
| `~output_dir` | `/home/wheeltec/R550PLUS_data_collect/log` | 输出目录 |
| `~rate` | `100` | 配置值（实际20Hz） |
| `~sync_tolerance` | `0.02` | 时间同步容差（秒） |

### cmd_vel_web_adapter.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~max_linear_x` | `0.4` | 最大前进速度 |
| `~max_linear_y` | `0.4` | 最大横移速度 |
| `~max_angular_z` | `1.0` | 最大角速度 |
| `~cmd_timeout` | `0.5` | 命令超时（秒） |
| `~heartbeat_timeout` | `1.0` | 心跳超时（秒） |
| `~response_exponent` | `0.70` | 响应曲线指数 |
| `~min_linear_ratio` | `0.20` | 最小前进比例 |

### web_control.launch

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rosbridge_port` | `9090` | rosbridge 端口 |
| `web_port` | `8000` | Web 页面端口 |
| `publish_rate` | `40.0` | 控制发布频率（Hz） |
| `cmd_timeout` | `0.5` | 命令超时（秒） |
| `max_linear_x` | `1.50` | 最大前进速度 |
| `max_linear_y` | `1.00` | 最大横移速度 |
| `max_angular_z` | `3.75` | 最大角速度 |

---

## 调试命令

```bash
# 查看话题
rostopic list | grep -E "odom|imu|PowerVoltage|current|cmd_vel|web"

# 查看话题频率（确认 20Hz）
rostopic hz /odom
rostopic hz /imu
rostopic hz /current_data

# 查看实时数据
rostopic echo /odom
rostopic echo /imu
rostopic echo /PowerVoltage
rostopic echo /current_data
rostopic echo /web/control_status
rostopic echo /web/cruise_status

# 发送巡航命令
rostopic pub /web/cruise_cmd geometry_msgs/Twist '{linear: {x: 0.5}}'

# 查看数据采集输出
ls -la /home/wheeltec/R550PLUS_data_collect/log/
```

---

## 常见问题

### 1. 电流数据为空
```bash
rostopic echo /current_data
ls -l /dev/ttyUSB1
```
检查串口 `/dev/ttyUSB1` 是否存在，波特率是否正确。

### 2. 数据采集频率低
底盘控制器固定输出 20Hz，无法更改。预处理脚本已按 20Hz 校准。

### 3. 定速巡航速度不准
PID 参数需根据地面情况调整。当前参数适合瓷砖地面。

### 4. 浏览器无法连接
- 确认 `web_control.launch` 已启动
- 确认 `9090` 端口可访问
- 确认页面中 WebSocket 地址是机器人实际 IP

---

## 文件清单

```
turn_on_wheeltec_robot/
├── src/
│   ├── wheeltec_robot.cpp          # 底盘控制节点源码
│   └── Quaternion_Solution.cpp     # IMU四元数解算
├── scripts/
│   ├── current_reader.py            # 电流读取脚本
│   ├── data_collector.py           # 数据采集脚本（message_filters同步）
│   ├── cmd_vel_web_adapter.py      # Web控制适配器（PID巡航）
│   ├── preprocess_data.py           # 数据预处理
│   ├── create_sliding_windows.py   # 滑动窗口分割
│   ├── validate_dataset.py          # 数据集验证
│   └── web_dashboard_server.py     # Web页面服务
├── web/
│   └── index.html                  # Web控制页面（含巡航UI）
├── launch/
│   ├── turn_on_wheeltec_robot.launch # 底盘启动
│   ├── data_collector.launch        # 数据采集启动
│   ├── web_control.launch          # Web控制启动
│   └── include/base_serial.launch   # 底盘串口配置
└── README.md
```

---

## 作者

杨鹏 - 燕山大学本科毕业设计
