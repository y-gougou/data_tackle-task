#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
R550PLUS 三全向轮机器人数据采集节点 (带时间同步和故障标签)
修改内容：
1. 使用message_filters实现话题时间同步
2. 添加fault_label标签字段
3. 添加frame序号
4. 优化数据存储格式

故障标签：
  0 - 正常状态
  1 - 驱动异常（单轮堵转）
  2 - 轮毂丢失
  3 - 电机轴偏心
  4 - 电池电压偏低

使用方法：
  roslaunch turn_on_wheeltec_robot data_collector.launch fault_label:=0
"""

import rospy
import csv
import os
import time
from datetime import datetime
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
import message_filters


class SyncedDataCollector:
    """带时间同步的数据采集器"""

    # 故障标签名称映射
    FAULT_NAMES = {
        0: 'normal',
        1: 'drive_fault',
        2: 'hub_loss',
        3: 'shaft_eccentric',
        4: 'voltage_low'
    }

    def __init__(self, fault_label=0):
        """
        初始化数据采集器

        Args:
            fault_label: 故障标签
                0 - 正常状态
                1 - 驱动异常（单轮堵转）
                2 - 轮毂丢失
                3 - 电机轴偏心
                4 - 电池电压偏低
        """
        # 故障标签
        self.fault_label = rospy.get_param('~fault_label', fault_label)
        self.fault_name = self.FAULT_NAMES.get(self.fault_label, 'unknown')

        # 输出配置
        self.output_dir = rospy.get_param('~output_dir',
            '/home/wheeltec/R550PLUS_data_collect/log')

        # 采样率配置
        self.rate = rospy.get_param('~rate', 100)  # Hz
        self.sample_period = 1.0 / self.rate

        # 时间同步阈值（秒）
        self.sync_tolerance = rospy.get_param('~sync_tolerance', 0.02)  # 20ms

        # 帧计数
        self.frame_count = 0
        self.last_odom_time = None
        self.csv_file = None
        self.csv_writer = None

        # 创建输出目录
        self.create_output_dir()

        # 打开CSV文件
        self.open_csv_file()

        # 设置时间同步订阅器
        self.setup_sync_subscribers()

        rospy.loginfo("数据采集器已启动，故障标签: %d (%s)", self.fault_label, self.fault_name)
        rospy.loginfo("采样率: %d Hz，输出目录: %s", self.rate, self.output_dir)

    def setup_sync_subscribers(self):
        """设置时间同步订阅器"""
        # 使用ApproximateTimeSynchronizer实现软同步
        odom_sub = message_filters.Subscriber('/odom', Odometry)
        imu_sub = message_filters.Subscriber('/imu', Imu)
        voltage_sub = message_filters.Subscriber('/PowerVoltage', Float32)
        current_sub = message_filters.Subscriber('/current_data', Float32MultiArray)

        # 同步器配置：队列大小10，时间容差0.02秒
        sync = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, imu_sub, voltage_sub, current_sub],
            queue_size=10,
            slop=self.sync_tolerance
        )
        sync.registerCallback(self.synchronized_callback)

        rospy.loginfo("时间同步订阅器已启动 (容差: %.3fs)", self.sync_tolerance)

    def synchronized_callback(self, odom, imu, voltage, current):
        """
        同步回调函数 - 所有话题数据在同一时间点被处理
        """
        try:
            # 获取ROS时间戳（使用里程计的时间戳作为基准）
            timestamp = odom.header.stamp.to_sec()

            # 检查数据连续性
            if self.last_odom_time is not None:
                dt = timestamp - self.last_odom_time
                # 如果时间间隔异常（超过2倍的采样周期），记录警告
                if dt > 2 * self.sample_period:
                    rospy.logwarn("数据跳跃检测: dt=%.4fs, 期望约%.4fs", dt, self.sample_period)

            self.last_odom_time = timestamp

            # 构建数据行
            row = {
                'timestamp': "%.6f" % timestamp,
                'frame': self.frame_count,
                'x': "%.6f" % odom.pose.pose.position.x,
                'y': "%.6f" % odom.pose.pose.position.y,
                'z': "%.6f" % odom.pose.pose.position.z,
                'vx': "%.6f" % odom.twist.twist.linear.x,
                'vy': "%.6f" % odom.twist.twist.linear.y,
                'vz': "%.6f" % odom.twist.twist.linear.z,
                'ax': "%.6f" % imu.linear_acceleration.x,
                'ay': "%.6f" % imu.linear_acceleration.y,
                'az': "%.6f" % imu.linear_acceleration.z,
                'gx': "%.6f" % imu.angular_velocity.x,
                'gy': "%.6f" % imu.angular_velocity.y,
                'gz': "%.6f" % imu.angular_velocity.z,
                'voltage': "%.4f" % voltage.data,
                'current0': "%.4f" % current.data[0],
                'current1': "%.4f" % current.data[1],
                'current2': "%.4f" % current.data[2],
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
            rospy.logerr("数据采集回调错误: %s", str(e))

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

        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'timestamp', 'frame', 'x', 'y', 'z',
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
    rospy.init_node('synced_data_collector', anonymous=True)

    # 从参数获取故障标签
    fault_label = int(rospy.get_param('~fault_label', 0))

    collector = SyncedDataCollector(fault_label=fault_label)

    # 注册关闭钩子
    rospy.on_shutdown(collector.on_shutdown)

    try:
        collector.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
