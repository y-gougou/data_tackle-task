#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
R550PLUS 三全向轮机器人数据采集节点
数据采集工作流 - 方式二：ROS消息订阅

采集的数据：
- /odom: 机器人里程计 (vx, vy, ω)
- /imu: IMU数据 (加速度, 角速度)
- /PowerVoltage: 电源电压
- /current_data: 电机电流 (3路)

预处理（根据论文建议）：
- 归一化
- 1阶/2阶后向差分
- 滑动窗口生成样本

使用方法：
  rosrun turn_on_wheeltec_robot data_collector.py

或带参数：
  rosrun turn_on_wheeltec_robot data_collector.py _output_dir:=/home/wheeltec/data_collect/log

Web控制模式（不自动开始采集，等待start命令）：
  rosrun turn_on_wheeltec_robot data_collector.py _auto_start:=false
"""

import json
import rospy
import os
import time
import numpy as np
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray, String

class DataCollector:
    """ROS消息订阅方式的数据采集器"""

    # 状态常量
    STATE_IDLE = "idle"
    STATE_RECORDING = "recording"
    STATE_PROCESSING = "processing"

    def __init__(self):
        # 参数
        self.output_dir = rospy.get_param('~output_dir', '/home/wheeltec/R550PLUS_data_collect/log')
        self.rate = rospy.get_param('~rate', 10)
        self.window_size = rospy.get_param('~window_size', 50)
        self.step_size = rospy.get_param('~step_size', 10)
        self.auto_start = rospy.get_param('~auto_start', True)

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # 最新数据缓存
        self.latest_odom = None
        self.latest_imu = None
        self.latest_voltage = None
        self.latest_current = None

        # 录制状态
        self.state = self.STATE_IDLE
        self.raw_file = None
        self.raw_data_file = None
        self.window_data_file = None
        self.data_count = 0
        self.record_start_time = 0
        self.last_print_time = time.time()

        # 数据订阅
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/PowerVoltage', Float32, self.voltage_callback)
        rospy.Subscriber('/current_data', Float32MultiArray, self.current_callback)

        # Web控制：命令与状态
        self.command_topic = rospy.get_param('~command_topic', '/web/data_collect/command')
        self.status_topic = rospy.get_param('~status_topic', '/web/data_collect/status')
        rospy.Subscriber(self.command_topic, String, self.command_callback)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)

        rospy.on_shutdown(self.on_shutdown)
        self.publish_status()

        rospy.loginfo("data_collector ready, output_dir=%s, auto_start=%s", self.output_dir, self.auto_start)

        if self.auto_start:
            self.start_recording()

    def publish_status(self):
        duration = time.time() - self.record_start_time if self.state == self.STATE_RECORDING else 0
        status = json.dumps({
            "state": self.state,
            "count": self.data_count,
            "duration": round(duration, 1),
            "rate": self.rate,
            "file": os.path.basename(self.raw_data_file) if self.raw_data_file else "",
            "output_dir": self.output_dir,
        })
        self.status_pub.publish(String(data=status))

    def command_callback(self, msg):
        cmd = msg.data.strip()
        if cmd.lower().startswith("start"):
            if self.state == self.STATE_IDLE:
                label = ""
                if ":" in cmd:
                    label = cmd.split(":", 1)[1].strip()
                self.start_recording(label=label)
            else:
                rospy.logwarn("Cannot start: state=%s", self.state)
        elif cmd.lower() == "stop":
            if self.state == self.STATE_RECORDING:
                self.stop_recording()
            else:
                rospy.logwarn("Cannot stop: state=%s", self.state)

    def start_recording(self, label=""):
        timestamp = datetime.strftime(datetime.now(), '%Y%m%d_%H%M%S')
        safe_label = "".join(c for c in label if c.isalnum() or c in "-_")
        suffix = ("_%s" % safe_label) if safe_label else ""
        self.raw_data_file = os.path.join(self.output_dir, 'raw_data_%s%s.csv' % (timestamp, suffix))
        self.window_data_file = os.path.join(self.output_dir, 'window_data_%s%s.csv' % (timestamp, suffix))
        self.raw_file = open(self.raw_data_file, 'w')
        self.raw_file.write("timestamp,x,y,z,vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,current0,current1,current2\n")
        self.raw_file.flush()
        self.data_count = 0
        self.record_start_time = time.time()
        self.state = self.STATE_RECORDING
        self.publish_status()
        rospy.loginfo("Recording started: %s", self.raw_data_file)

    def stop_recording(self):
        self.state = self.STATE_PROCESSING
        self.publish_status()
        self.close_raw_file()
        self.preprocess_and_save_windows()
        self.state = self.STATE_IDLE
        self.publish_status()
        rospy.loginfo("Recording stopped, %d samples", self.data_count)

    def on_shutdown(self):
        if self.state == self.STATE_RECORDING:
            self.stop_recording()

    def odom_callback(self, msg):
        """里程计回调"""
        # 提取vx, vy, wz (线速度在child_frame_id坐标系下，z轴取角速度)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z

        # 角速度
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        # 位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.latest_odom = {
            'vx': vx, 'vy': vy, 'vz': vz,
            'wx': wx, 'wy': wy, 'wz': wz,
            'x': x, 'y': y, 'z': z
        }

    def imu_callback(self, msg):
        """IMU回调"""
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        self.latest_imu = {
            'ax': ax, 'ay': ay, 'az': az,
            'gx': gx, 'gy': gy, 'gz': gz
        }

    def voltage_callback(self, msg):
        """电压回调"""
        self.latest_voltage = {'voltage': msg.data}

    def current_callback(self, msg):
        """电流回调"""
        if len(msg.data) >= 3:
            self.latest_current = {
                'ch0': msg.data[0],
                'ch1': msg.data[1],
                'ch2': msg.data[2]
            }

    def collect_and_save_data(self):
        """采集一轮数据并实时写入文件"""
        if self.latest_odom is None:
            return False

        timestamp = rospy.Time.now().to_sec()

        # 里程计
        x = self.latest_odom.get('x', 0)
        y = self.latest_odom.get('y', 0)
        z = self.latest_odom.get('z', 0)
        vx = self.latest_odom.get('vx', 0)
        vy = self.latest_odom.get('vy', 0)
        vz = self.latest_odom.get('vz', 0)

        # IMU
        if self.latest_imu:
            ax = self.latest_imu.get('ax', 0)
            ay = self.latest_imu.get('ay', 0)
            az = self.latest_imu.get('az', 0)
            gx = self.latest_imu.get('gx', 0)
            gy = self.latest_imu.get('gy', 0)
            gz = self.latest_imu.get('gz', 0)
        else:
            ax = ay = az = gx = gy = gz = 0

        # 电压
        voltage = self.latest_voltage.get('voltage', 0) if self.latest_voltage else 0

        # 电流
        if self.latest_current:
            current0 = self.latest_current.get('ch0', 0)
            current1 = self.latest_current.get('ch1', 0)
            current2 = self.latest_current.get('ch2', 0)
        else:
            current0 = current1 = current2 = 0

        # 写入CSV
        # 顺序: timestamp, x, y, z, vx, vy, vz, ax, ay, az, gx, gy, gz, voltage, current0, current1, current2
        line = "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (
            timestamp, x, y, z, vx, vy, vz,
            ax, ay, az, gx, gy, gz, voltage, current0, current1, current2
        )
        self.raw_file.write(line)

        self.data_count += 1
        return True

    def close_raw_file(self):
        """关闭原始数据文件"""
        if self.raw_file:
            self.raw_file.flush()
            self.raw_file.close()
            self.raw_file = None
            rospy.loginfo("原始数据文件已关闭，共 %d 条记录" % self.data_count)

    def preprocess_and_save_windows(self):
        """
        预处理并生成滑动窗口样本
        注意：这里从原始CSV文件读取数据，而不是从内存
        """
        rospy.loginfo("开始预处理数据...")

        # 先读取原始数据（流式读取，避免内存爆炸）
        timestamps = []
        odom_data = []
        imu_data = []
        current_data = []

        rospy.loginfo("读取原始数据文件: %s" % self.raw_data_file)
        with open(self.raw_data_file, 'r') as f:
            header = f.readline()  # 跳过表头
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 17:
                    continue
                timestamps.append(float(parts[0]))
                # x, y, z (索引1,2,3)
                # vx, vy, vz (索引4,5,6)
                odom_data.append([float(parts[4]), float(parts[5]), float(parts[6])])
                # ax, ay, az, gx, gy, gz (索引7,8,9,10,11,12)
                imu_data.append([float(parts[7]), float(parts[8]), float(parts[9]),
                                float(parts[10]), float(parts[11]), float(parts[12])])
                # voltage (索引13)
                # current0,1,2 (索引14,15,16)
                current_data.append([float(parts[14]), float(parts[15]), float(parts[16])])

        num_frames = len(timestamps)
        rospy.loginfo("读取到 %d 帧数据" % num_frames)

        if num_frames < self.window_size:
            rospy.logwarn("数据量不足，窗口大小%d，当前数据%d" % (self.window_size, num_frames))
            return

        # 转换为numpy数组
        odom_array = np.array(odom_data, dtype=np.float32)  # (N, 3)
        imu_array = np.array(imu_data, dtype=np.float32)    # (N, 6)
        current_array = np.array(current_data, dtype=np.float32)  # (N, 3)

        rospy.loginfo("原始数据形状: odom=%s, imu=%s, current=%s" % (
            odom_array.shape, imu_array.shape, current_array.shape))

        # 1. 归一化 (Min-Max)
        def normalize(data):
            min_vals = data.min(axis=0)
            max_vals = data.max(axis=0)
            range_vals = max_vals - min_vals
            range_vals[range_vals == 0] = 1  # 避免除零
            return (data - min_vals) / range_vals, min_vals, max_vals

        odom_norm, odom_min, odom_max = normalize(odom_array)
        imu_norm, imu_min, imu_max = normalize(imu_array)
        current_norm, current_min, current_max = normalize(current_array)

        # 2. 后向差分 (1阶和2阶)
        def compute_backward_diff(data, order=2):
            """计算后向差分"""
            result = [data]
            for _ in range(order):
                diff = np.diff(data, axis=0, prepend=data[0:1])
                result.append(diff)
            return np.concatenate(result, axis=1)

        odom_diff = compute_backward_diff(odom_norm, order=2)  # (N, 9)
        imu_diff = compute_backward_diff(imu_norm, order=2)     # (N, 18)
        current_diff = compute_backward_diff(current_norm, order=2)  # (N, 9)

        rospy.loginfo("差分后数据形状: odom=%s, imu=%s, current=%s" % (
            odom_diff.shape, imu_diff.shape, current_diff.shape))

        # 3. 合并所有特征
        features = np.concatenate([odom_diff, imu_diff, current_diff], axis=1)  # (N, 36)
        rospy.loginfo("合并特征形状: %s" % str(features.shape))

        # 4. 滑动窗口生成样本（流式写入）
        num_samples = (len(features) - self.window_size) // self.step_size + 1
        rospy.loginfo("生成 %d 个滑动窗口样本" % num_samples)

        with open(self.window_data_file, 'w') as f:
            # 头
            header = "sample_id,timestamp,"
            header += ",".join(["f%d" % i for i in range(self.window_size * features.shape[1])])
            header += "\n"
            f.write(header)
            f.flush()

            # 样本
            for i in range(num_samples):
                start_idx = i * self.step_size
                end_idx = start_idx + self.window_size
                window = features[start_idx:end_idx]  # (window_size, 36)

                # 展平
                flat = window.flatten()

                # 写入
                timestamp_start = timestamps[start_idx]
                line = "%d,%.6f," % (i, timestamp_start) + ",".join(["%.6f" % v for v in flat]) + "\n"
                f.write(line)

                if i % 1000 == 0 and i > 0:
                    f.flush()
                    rospy.loginfo("已处理 %d / %d 个样本" % (i, num_samples))

        rospy.loginfo("滑动窗口数据保存到: %s" % self.window_data_file)

        # 保存归一化参数（用于后续推理）
        norm_params_file = os.path.join(self.output_dir, 'norm_params.npz')
        np.savez(norm_params_file,
                 odom_min=odom_min, odom_max=odom_max,
                 imu_min=imu_min, imu_max=imu_max,
                 current_min=current_min, current_max=current_max,
                 window_size=self.window_size,
                 step_size=self.step_size)
        rospy.loginfo("归一化参数保存到: %s" % norm_params_file)

    def run(self):
        """运行采集循环"""
        rate = rospy.Rate(self.rate)
        status_interval = 1.0
        last_status_time = time.time()

        rospy.loginfo("data_collector loop running at %d Hz", self.rate)

        while not rospy.is_shutdown():
            if self.state == self.STATE_RECORDING:
                self.collect_and_save_data()

            now = time.time()
            if now - last_status_time >= status_interval:
                self.publish_status()
                last_status_time = now

                if self.state == self.STATE_RECORDING and now - self.last_print_time >= 5.0:
                    elapsed = now - self.record_start_time
                    rospy.loginfo("Recording... %.0fs, %d samples", elapsed, self.data_count)
                    self.last_print_time = now

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('data_collector', anonymous=True)
    collector = DataCollector()
    collector.run()
