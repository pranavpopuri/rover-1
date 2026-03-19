#!/usr/bin/env python3
"""
Simple teleop for Jazzy diff_drive_controller.
Sends TwistStamped messages with real timestamps.

Controls:
  i - forward
  k - stop
  j - turn left
  l - turn right
  , - backward
  u - forward-left
  o - forward-right
  q - quit
"""

import sys
import tty
import termios
import rclpy
from geometry_msgs.msg import TwistStamped

LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 0.5  # rad/s

BINDINGS = {
    'i': (1, 0),    # forward
    ',': (-1, 0),   # backward
    'j': (0, 1),    # turn left
    'l': (0, -1),   # turn right
    'u': (1, 1),    # forward-left
    'o': (1, -1),   # forward-right
    'k': (0, 0),    # stop
}

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    rclpy.init()
    node = rclpy.create_node('teleop_stamped')
    pub = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

    print(__doc__)
    print(f"Speed: linear={LINEAR_SPEED} m/s, angular={ANGULAR_SPEED} rad/s")
    print("Press keys to drive. 'q' to quit.\n")

    try:
        while True:
            key = get_key()
            if key == 'q' or key == '\x03':  # q or Ctrl+C
                break

            if key in BINDINGS:
                lin, ang = BINDINGS[key]
                msg = TwistStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = float(lin) * LINEAR_SPEED
                msg.twist.angular.z = float(ang) * ANGULAR_SPEED
                pub.publish(msg)
                status = "STOP" if lin == 0 and ang == 0 else f"lin={msg.twist.linear.x:.1f} ang={msg.twist.angular.z:.1f}"
                print(f"\r{status}    ", end="", flush=True)
    finally:
        # Send stop
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        pub.publish(msg)
        print("\nStopped.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
