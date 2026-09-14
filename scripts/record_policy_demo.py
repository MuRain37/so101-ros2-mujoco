#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def decode_image(message):
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, : message.width * 3].reshape(
        message.height, message.width, 3
    )
    return image[:, :, ::-1].copy() if message.encoding == "rgb8" else image.copy()


class PolicyDemoRecorder(Node):
    def __init__(self):
        super().__init__("policy_demo_recorder")
        self.front = None
        self.wrist = None
        self.create_subscription(
            Image, "/d435/color/image_raw", self._on_front, 10
        )
        self.create_subscription(
            Image, "/wrist_cam/image_raw", self._on_wrist, 10
        )

    def _on_front(self, message):
        self.front = decode_image(message)

    def _on_wrist(self, message):
        self.wrist = decode_image(message)


def compose_frame(front, wrist, title, elapsed):
    frame = front.copy()
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (640, 44), (12, 18, 24), -1)
    frame = cv2.addWeighted(overlay, 0.78, frame, 0.22, 0)
    cv2.putText(
        frame,
        title,
        (16, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.circle(frame, (548, 22), 5, (62, 205, 93), -1)
    cv2.putText(
        frame,
        f"LIVE {elapsed:04.1f}s",
        (560, 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (230, 240, 245),
        1,
        cv2.LINE_AA,
    )

    inset = cv2.resize(wrist, (192, 144), interpolation=cv2.INTER_AREA)
    x_offset, y_offset = 438, 326
    cv2.rectangle(
        frame,
        (x_offset - 3, y_offset - 24),
        (x_offset + 195, y_offset + 147),
        (12, 18, 24),
        -1,
    )
    cv2.putText(
        frame,
        "WRIST CAMERA",
        (x_offset + 4, y_offset - 7),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    frame[y_offset : y_offset + 144, x_offset : x_offset + 192] = inset
    return frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = PolicyDemoRecorder()
    deadline = time.monotonic() + 30.0
    while (node.front is None or node.wrist is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.front is None or node.wrist is None:
        raise RuntimeError("timed out waiting for both camera streams")

    writer = cv2.VideoWriter(
        args.output,
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (640, 480),
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot open video output: {args.output}")

    start = time.monotonic()
    next_frame = start
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now >= next_frame:
                writer.write(
                    compose_frame(node.front, node.wrist, args.title, now - start)
                )
                next_frame += 1.0 / args.fps
    finally:
        writer.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
