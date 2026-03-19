#!/usr/bin/env python3
"""
Object follower node.
Subscribes to /detections, drives toward the target object,
stops ~30cm in front of it.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class FollowerNode(Node):
    # States
    SEARCHING = 'SEARCHING'
    APPROACHING = 'APPROACHING'
    ARRIVED = 'ARRIVED'

    def __init__(self):
        super().__init__('follower_node')

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_gain', 0.003)
        self.declare_parameter('stop_bbox_height', 300.0)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('cmd_rate', 10.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.stop_bbox_height = self.get_parameter('stop_bbox_height').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        cmd_rate = self.get_parameter('cmd_rate').value

        # ROS interfaces
        self.sub = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        # State
        self.state = self.SEARCHING
        self.latest_detection = None
        self.last_detection_time = 0.0
        self.image_width = 640

        # Timer for publishing commands
        self.timer = self.create_timer(1.0 / cmd_rate, self.control_loop)

        self.get_logger().info(
            f'Follower started: speed={self.linear_speed} m/s, '
            f'stop_height={self.stop_bbox_height}px')

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.image_width = data.get('image_width', 640)

        if data['detections']:
            self.latest_detection = data['detections'][0]
            self.last_detection_time = time.time()

    def control_loop(self):
        now = time.time()
        target_visible = (
            self.latest_detection is not None
            and (now - self.last_detection_time) < self.lost_timeout
        )

        # State transitions
        if self.state == self.SEARCHING:
            if target_visible:
                self.state = self.APPROACHING
                self.get_logger().info('Target found — approaching')

        elif self.state == self.APPROACHING:
            if not target_visible:
                self.state = self.SEARCHING
                self.get_logger().info('Target lost — searching')
            elif self.latest_detection['bbox_h'] >= self.stop_bbox_height:
                self.state = self.ARRIVED
                self.get_logger().info('Arrived at target')

        elif self.state == self.ARRIVED:
            if not target_visible:
                self.state = self.SEARCHING
                self.get_logger().info('Target lost — searching')
            elif self.latest_detection['bbox_h'] < self.stop_bbox_height * 0.8:
                self.state = self.APPROACHING
                self.get_logger().info('Target moved away — approaching')

        # Compute command
        linear_x = 0.0
        angular_z = 0.0

        if self.state == self.APPROACHING and self.latest_detection:
            det = self.latest_detection
            det_age = now - self.last_detection_time

            # Horizontal centering — proportional controller
            error_x = det['bbox_center_x'] - (self.image_width / 2)
            angular_z = -self.angular_gain * error_x

            # Decay angular command as detection ages — prevents spin-out
            # Fresh detection (<0.3s): full steering
            # Stale detection (>0.3s): drive straight, stop turning
            if det_age > 0.3:
                angular_z = 0.0

            # Forward speed — slow down as we get closer
            approach_ratio = det['bbox_h'] / self.stop_bbox_height
            linear_x = self.linear_speed * max(0.0, 1.0 - approach_ratio) * self.linear_gain

            # Add minimum speed so robot doesn't stall
            if linear_x > 0.01:
                linear_x = max(linear_x, 0.15)

            # Clamp
            linear_x = max(0.0, min(self.linear_speed, linear_x))
            angular_z = max(-0.5, min(0.5, angular_z))

        # Publish
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        # Send stop command before shutting down
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
