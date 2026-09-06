#!/usr/bin/env python3
"""Publish demo joint states for the SO-101 arm so the model is visible in RViz.

By default the joints follow a slow sinusoidal motion inside their URDF limits.
Set the ``animate`` parameter to ``false`` to hold the model at a fixed pose.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Amplitudes chosen to stay well inside the URDF joint limits.
AMPLITUDES = [0.6, 0.6, 0.6, 0.5, 1.0, 0.45]
OFFSETS = [0.0, 0.4, 0.9, 1.4, 2.0, 2.6]
GRIPPER_BASE = 0.5  # keep the gripper slightly open


class JointStateDemo(Node):
    def __init__(self):
        super().__init__("joint_state_demo")
        self.declare_parameter("animate", True)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.t = 0.0

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        if self.get_parameter("animate").value:
            msg.position = [
                AMPLITUDES[i] * math.sin(self.t + OFFSETS[i]) for i in range(5)
            ]
            msg.position.append(
                GRIPPER_BASE + 0.4 * math.sin(self.t + OFFSETS[5])
            )
        else:
            msg.position = [0.0] * 5 + [GRIPPER_BASE]
        self.t += 0.05
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(JointStateDemo())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
