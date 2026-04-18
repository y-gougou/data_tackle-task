#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Lightweight robot-side safety adapter for browser teleoperation.

Input topics:
  - /cmd_vel_web (geometry_msgs/Twist)
  - /web/heartbeat (std_msgs/Empty)
  - /web/estop (std_msgs/Bool)

Output topic:
  - /cmd_vel (geometry_msgs/Twist)

This node keeps the existing chassis node untouched while adding a minimal
timeout, estop latch, and speed limiting layer for browser control.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String


class WebCmdVelAdapter(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_web")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.heartbeat_topic = rospy.get_param("~heartbeat_topic", "/web/heartbeat")
        self.estop_topic = rospy.get_param("~estop_topic", "/web/estop")
        self.status_topic = rospy.get_param("~status_topic", "/web/control_status")

        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        self.heartbeat_timeout = rospy.get_param("~heartbeat_timeout", 1.0)

        self.max_linear_x = rospy.get_param("~max_linear_x", 0.4)
        self.max_linear_y = rospy.get_param("~max_linear_y", 0.4)
        self.max_angular_z = rospy.get_param("~max_angular_z", 1.0)
        self.linear_deadband = rospy.get_param("~linear_deadband", 0.02)
        self.angular_deadband = rospy.get_param("~angular_deadband", 0.03)
        self.response_exponent = rospy.get_param("~response_exponent", 0.70)
        self.min_linear_ratio = rospy.get_param("~min_linear_ratio", 0.20)
        self.min_lateral_ratio = rospy.get_param("~min_lateral_ratio", 0.15)
        self.min_angular_ratio = rospy.get_param("~min_angular_ratio", 0.24)

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_heartbeat_time = rospy.Time(0)
        self.have_cmd = False
        self.estop_latched = False
        self.last_status = ""

        self.cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)

        rospy.Subscriber(self.input_topic, Twist, self.cmd_callback)
        rospy.Subscriber(self.heartbeat_topic, Empty, self.heartbeat_callback)
        rospy.Subscriber(self.estop_topic, Bool, self.estop_callback)

        rospy.on_shutdown(self.on_shutdown)
        self.publish_status("idle")

        rospy.loginfo(
            "web_cmd_vel_adapter ready: %s -> %s (cmd_timeout=%.2fs, heartbeat_timeout=%.2fs)",
            self.input_topic,
            self.output_topic,
            self.cmd_timeout,
            self.heartbeat_timeout,
        )

    def clamp(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def zero_twist(self):
        return Twist()

    def shape_axis(self, value, limit, deadband, min_ratio):
        if limit <= 0.0:
            return 0.0

        magnitude = abs(value)
        if magnitude <= 0.0:
            return 0.0

        normalized = min(1.0, magnitude / limit)
        if normalized <= deadband:
            return 0.0

        span = max(1e-6, 1.0 - deadband)
        normalized = (normalized - deadband) / span
        shaped = min_ratio + (1.0 - min_ratio) * pow(normalized, self.response_exponent)
        return shaped * limit if value >= 0.0 else -shaped * limit

    def publish_status(self, status):
        if status != self.last_status:
            self.last_status = status
            self.status_pub.publish(String(data=status))
            rospy.loginfo("web_cmd_vel_adapter status: %s", status)

    def cmd_callback(self, msg):
        limited = Twist()
        limited.linear.x = self.shape_axis(
            self.clamp(msg.linear.x, self.max_linear_x),
            self.max_linear_x,
            self.linear_deadband,
            self.min_linear_ratio,
        )
        limited.linear.y = self.shape_axis(
            self.clamp(msg.linear.y, self.max_linear_y),
            self.max_linear_y,
            self.linear_deadband,
            self.min_lateral_ratio,
        )
        limited.angular.z = self.shape_axis(
            self.clamp(msg.angular.z, self.max_angular_z),
            self.max_angular_z,
            self.angular_deadband,
            self.min_angular_ratio,
        )

        self.last_cmd = limited
        self.last_cmd_time = rospy.Time.now()
        self.have_cmd = True

    def heartbeat_callback(self, _msg):
        self.last_heartbeat_time = rospy.Time.now()

    def estop_callback(self, msg):
        if msg.data:
            self.estop_latched = True
            self.publish_status("estop")
            self.cmd_pub.publish(self.zero_twist())
        else:
            self.estop_latched = False
            self.publish_status("idle")

    def resolve_status_and_command(self):
        now = rospy.Time.now()

        if self.estop_latched:
            return "estop", self.zero_twist()

        if not self.have_cmd:
            return "idle", self.zero_twist()

        if self.heartbeat_timeout > 0.0:
            if self.last_heartbeat_time.to_sec() == 0.0:
                return "waiting_heartbeat", self.zero_twist()
            if (now - self.last_heartbeat_time).to_sec() > self.heartbeat_timeout:
                return "heartbeat_timeout", self.zero_twist()

        if self.cmd_timeout > 0.0 and (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return "cmd_timeout", self.zero_twist()

        return "active", self.last_cmd

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            status, cmd = self.resolve_status_and_command()
            self.publish_status(status)
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def on_shutdown(self):
        self.cmd_pub.publish(self.zero_twist())


if __name__ == "__main__":
    rospy.init_node("web_cmd_vel_adapter", anonymous=False)
    WebCmdVelAdapter().run()
