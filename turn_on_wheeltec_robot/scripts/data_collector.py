#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
R550PLUS 三全向轮机器人数据采集节点

功能：
1. 独立接收各话题，保存原始时间戳
2. 基于 /odom 频率（20Hz）触发数据记录
3. 其他话题使用最近一次收到的值
4. CSV包含各话题独立时间戳，供预处理脚本进行时间同步

采集频率：
- 记录频率由 /odom 决定（20Hz）
- /imu, /PowerVoltage 与 /odom 同步（固件20Hz）
- /current_data 独立接收（约5-6Hz），使用最新值填充

故障标签：
  0 - 正常状态
  1 - 驱动异常（单轮堵转）
  2 - 轮子打滑
  3 - 电机轴偏心
  4 - 电池电压偏低

使用方法：
  roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0
"""

import rospy
import csv
import os
from datetime import datetime

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray


class DataCollector:
    """数据采集器 - 基于里程计触发，其他话题使用最新值"""

    # 故障标签名称映射
    FAULT_NAMES = {
        0: 'normal',
        1: 'drive_fault',
        2: 'wheel_slip',
        3: 'shaft_eccentric',
        4: 'voltage_low'
    }

    def __init__(self, fault_label=0):
        """
        初始化数据采集器

        Args:
            fault_label: 故障标签
        """
        # 故障标签
        self.fault_label = rospy.get_param('~fault_label', fault_label)
        self.fault_name = self.FAULT_NAMES.get(self.fault_label, 'unknown')

        # 输出配置
        self.output_dir = rospy.get_param('~output_dir',
            '/home/wheeltec/R550PLUS_data_collect/log')

        # 帧计数
        self.frame_count = 0
        self.csv_file = None
        self.csv_writer = None

        # 存储最新收到的话题数据
        self.latest_data = {
            'odom': None,
            'imu': None,
            'voltage': None,
            'current': None,
            'odom_time': None,
            'imu_time': None,
            'voltage_time': None,
            'current_time': None,
        }

        # 创建输出目录
        self.create_output_dir()

        # 打开CSV文件
        self.open_csv_file()

        # 设置订阅器（独立订阅，不同步）
        self.setup_subscribers()

        rospy.loginfo("数据采集器已启动，故障标签: %d (%s)", self.fault_label, self.fault_name)
        rospy.loginfo("输出目录: %s", self.output_dir)

    def setup_subscribers(self):
        """设置独立订阅器"""
        # /odom 是主触发源，每次收到就记录一行
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/PowerVoltage', Float32, self.voltage_callback)
        rospy.Subscriber('/current_data', Float32MultiArray, self.current_callback)
        rospy.loginfo("独立订阅器已启动 (/odom 触发记录)")

    def odom_callback(self, msg):
        """里程计回调 - 触发数据记录"""
        # 更新时间戳
        self.latest_data['odom'] = msg
        self.latest_data['odom_time'] = msg.header.stamp.to_sec()

        # 记录一行数据（使用 odom 时间戳作为基准）
        self.record_data()

    def imu_callback(self, msg):
        """IMU 回调 - 更新最新值"""
        self.latest_data['imu'] = msg
        self.latest_data['imu_time'] = msg.header.stamp.to_sec()

    def voltage_callback(self, msg):
        """电压回调 - 更新最新值"""
        self.latest_data['voltage'] = msg
        self.latest_data['voltage_time'] = rospy.Time.now().to_sec()

    def current_callback(self, msg):
        """电流回调 - 更新最新值"""
        self.latest_data['current'] = msg
        self.latest_data['current_time'] = rospy.Time.now().to_sec()

    def record_data(self):
        """记录一行数据"""
        try:
            odom = self.latest_data['odom']
            imu = self.latest_data['imu']
            voltage = self.latest_data['voltage']
            current = self.latest_data['current']

            # 基准时间戳
            timestamp = self.latest_data['odom_time']

            # 如果还没有收到其他话题的数据，使用默认值
            if imu is None:
                imu_data = {'x': 0, 'y': 0, 'z': 0, 'gx': 0, 'gy': 0, 'gz': 0}
                imu_time = timestamp
            else:
                imu_data = {
                    'x': imu.linear_acceleration.x,
                    'y': imu.linear_acceleration.y,
                    'z': imu.linear_acceleration.z,
                    'gx': imu.angular_velocity.x,
                    'gy': imu.angular_velocity.y,
                    'gz': imu.angular_velocity.z,
                }
                imu_time = self.latest_data['imu_time']

            if voltage is None:
                voltage_val = 0.0
                voltage_time = timestamp
            else:
                voltage_val = voltage.data
                voltage_time = self.latest_data['voltage_time']

            if current is None:
                current_vals = [0.0, 0.0, 0.0]
                current_time = timestamp
            else:
                current_vals = list(current.data)
                current_time = self.latest_data['current_time']

            # 构建数据行
            row = {
                # 基准时间戳（odom）
                'timestamp': "%.6f" % timestamp,
                # 各话题的原始时间戳（供后续预处理同步）
                'odom_time': "%.6f" % timestamp,
                'imu_time': "%.6f" % imu_time,
                'voltage_time': "%.6f" % voltage_time,
                'current_time': "%.6f" % current_time,
                # 帧计数
                'frame': self.frame_count,
                # 位置
                'x': "%.6f" % odom.pose.pose.position.x,
                'y': "%.6f" % odom.pose.pose.position.y,
                'z': "%.6f" % odom.pose.pose.position.z,
                # 速度
                'vx': "%.6f" % odom.twist.twist.linear.x,
                'vy': "%.6f" % odom.twist.twist.linear.y,
                'vz': "%.6f" % odom.twist.twist.linear.z,
                # 加速度
                'ax': "%.6f" % imu_data['x'],
                'ay': "%.6f" % imu_data['y'],
                'az': "%.6f" % imu_data['z'],
                # 角速度
                'gx': "%.6f" % imu_data['gx'],
                'gy': "%.6f" % imu_data['gy'],
                'gz': "%.6f" % imu_data['gz'],
                # 电压
                'voltage': "%.4f" % voltage_val,
                # 电流
                'current0': "%.4f" % current_vals[0] if len(current_vals) > 0 else "0.0000",
                'current1': "%.4f" % current_vals[1] if len(current_vals) > 1 else "0.0000",
                'current2': "%.4f" % current_vals[2] if len(current_vals) > 2 else "0.0000",
                # 故障标签
                'fault_label': self.fault_label
            }

            # 写入CSV
            self.csv_writer.writerow(row)
            self.csv_file.flush()

            self.frame_count += 1

            # 定期打印采集进度
            if self.frame_count % 1000 == 0:
                rospy.loginfo("已采集 %d 帧数据", self.frame_count)

        except Exception as e:
            rospy.logerr("数据记录错误: %s", str(e))

    def create_output_dir(self):
        """创建输出目录"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo("创建输出目录: %s", self.output_dir)

    def open_csv_file(self):
        """打开新的CSV文件"""
        # 生成文件名：故障类型_时间戳.csv
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "%s_%s.csv" % (self.fault_name, timestamp_str)
        filepath = os.path.join(self.output_dir, filename)

        self.csv_file = open(filepath, 'w')
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'timestamp', 'frame',
                'odom_time', 'imu_time', 'voltage_time', 'current_time',
                'x', 'y', 'z',
                'vx', 'vy', 'vz',
                'ax', 'ay', 'az',
                'gx', 'gy', 'gz',
                'voltage', 'current0', 'current1', 'current2',
                'fault_label'
            ]
        )
        self.csv_writer.writeheader()

        rospy.loginfo("CSV文件已打开: %s", filepath)
        rospy.loginfo("字段: %s", self.csv_writer.fieldnames)

    def shutdown(self):
        """关闭数据采集器"""
        if self.csv_file:
            self.csv_file.close()
        rospy.loginfo("数据采集已停止，共采集 %d 帧数据", self.frame_count)

    def run(self):
        """运行数据采集器"""
        rospy.loginfo("开始数据采集，按 Ctrl+C 停止...")
        rospy.spin()

    def on_shutdown(self):
        """ROS关闭回调"""
        self.shutdown()


def main():
    """主函数"""
    rospy.init_node('data_collector', anonymous=True)

    # 从参数获取故障标签
    fault_label = int(rospy.get_param('~fault_label', 0))

    collector = DataCollector(fault_label=fault_label)

    # 注册关闭钩子
    rospy.on_shutdown(collector.on_shutdown)

    try:
        collector.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
