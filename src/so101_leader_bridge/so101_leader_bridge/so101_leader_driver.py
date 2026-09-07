#!/usr/bin/env python3
"""Publish the real SO-101 leader arm joint states to ROS2.

Reads the 6 Feetech-compatible encoder modules (IDs 1-6) on the serial bus
and publishes ``sensor_msgs/JointState`` so RViz mirrors the physical arm.
Raw encoder values are converted with the LeRobot calibration (same math as
``lerobot/motors/motors_bus.py``) and then mapped linearly onto the URDF
joint limits in radians.
"""

import json
import time
from pathlib import Path

import serial
import rclpy
from rclpy.executors import ExternalShutdownException
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
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# URDF joint limits in radians (so101_new_calib.urdf)
LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

READ_ADDR = 0x38  # Present_Position
READ_LEN = 2
BAUDRATE = 1_000_000

# LeRobot normalization modes for the SO-101 leader
MODE_M100_100 = "range_m100_100"
MODE_0_100 = "range_0_100"
MODES = {
    "shoulder_pan": MODE_M100_100,
    "shoulder_lift": MODE_M100_100,
    "elbow_flex": MODE_M100_100,
    "wrist_flex": MODE_M100_100,
    "wrist_roll": MODE_M100_100,
    "gripper": MODE_0_100,
}


def read_frame(pid):
    """Build a Feetech SCS protocol READ frame for Present_Position."""
    ck = (~(pid + 4 + 2 + READ_ADDR + READ_LEN)) & 0xFF
    return bytes([0xFF, 0xFF, pid, 0x04, 0x02, READ_ADDR, READ_LEN, ck])


def parse_response(buf, expected_id):
    """Extract the payload bytes of the first valid status packet, if any."""
    for i in range(len(buf) - 1):
        if buf[i] == 0xFF and buf[i + 1] == 0xFF and i + 3 < len(buf):
            ln = buf[i + 3]
            total = ln + 4
            if ln == 4 and i + total <= len(buf):
                ck = 0
                for j in range(i + 2, i + total - 1):
                    ck += buf[j]
                if ((~ck & 0xFF) == buf[i + total - 1]
                        and buf[i + 2] == expected_id and buf[i + 4] == 0):
                    payload = buf[i + 5 : i + total - 1]
                    if (payload[0] | payload[1] << 8) <= 4095:
                        return payload
    return None


def normalize(name, raw, calib):
    """Replicate lerobot MotorsBus._normalize for a single motor."""
    min_ = calib["range_min"]
    max_ = calib["range_max"]
    bounded = min(max_, max(min_, raw))
    if MODES[name] == MODE_M100_100:
        return (((bounded - min_) / (max_ - min_)) * 200) - 100
    return ((bounded - min_) / (max_ - min_)) * 100


def norm_to_radians(name, norm):
    """Map LeRobot normalized value (-100..100 or 0..100) onto URDF limits."""
    lo, hi = LIMITS[name]
    if MODES[name] == MODE_0_100:
        return lo + (norm / 100.0) * (hi - lo)
    return lo + ((norm + 100.0) / 200.0) * (hi - lo)


class SO101LeaderDriver(Node):
    def __init__(self):
        super().__init__("so101_leader_driver")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter(
            "calibration_file",
            "~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/zihao_leader_arm.json",
        )
        self.declare_parameter("rate", 50.0)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        rate = self.get_parameter("rate").value
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.ser = None
        self.calibration = self._load_calibration()
        self._fail_count = 0
        self._connect()

    def _load_calibration(self):
        path = Path(self.get_parameter("calibration_file").value).expanduser()
        try:
            with path.open("r", encoding="utf-8") as f:
                calib = json.load(f)
            self.get_logger().info(f"loaded calibration from {path}")
            for name in JOINTS:
                if name not in calib or calib[name]["range_max"] <= calib[name]["range_min"]:
                    raise ValueError(f"invalid calibration for {name}")
            return calib
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"failed to load calibration: {e}")
            raise RuntimeError(f"cannot load valid leader calibration: {path}") from e

    def _connect(self):
        try:
            self.ser = serial.Serial(
                self.get_parameter("port").value, BAUDRATE, timeout=0.003, exclusive=True
            )
            self.get_logger().info(
                f"opened serial port {self.get_parameter('port').value} @ {BAUDRATE}"
            )
        except Exception as e:  # noqa: BLE001
            self.ser = None
            self._fail_count += 1
            if self._fail_count == 1 or self._fail_count % 100 == 0:
                self.get_logger().warn(
                    f"cannot open leader serial port "
                    f"{self.get_parameter('port').value}: {e}; retrying"
                )

    def _read_motor(self, pid):
        """Read one motor's raw position, retrying once on failure."""
        for _ in range(2):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(read_frame(pid))
                deadline = time.monotonic() + 0.02
                buf = b""
                while time.monotonic() < deadline:
                    buf += self.ser.read(max(1, self.ser.in_waiting))
                    data = parse_response(buf, pid)
                    if data is not None:
                        return data[0] | (data[1] << 8)
            except serial.SerialException:
                pass
        return None

    def tick(self):
        if self.ser is None or not self.ser.is_open:
            self._connect()
            return

        positions = []
        ok = True
        for pid in MOTOR_IDS:
            raw = self._read_motor(pid)
            if raw is None:
                ok = False
                break
            positions.append(raw)

        if ok and len(positions) == 6:
            radians = []
            for name, raw in zip(JOINTS, positions):
                if name not in self.calibration:
                    radians.append(0.0)
                    continue
                norm = normalize(name, raw, self.calibration[name])
                radians.append(norm_to_radians(name, norm))
            self._fail_count = 0
        else:
            self._fail_count += 1
            if self._fail_count == 1 or self._fail_count % 200 == 0:
                self.get_logger().warn(
                    f"failed to read motors ({self._fail_count} consecutive failures)"
                )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = radians
        self.pub.publish(msg)

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SO101LeaderDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
