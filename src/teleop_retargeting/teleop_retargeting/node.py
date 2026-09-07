"""Direct SO101 joint forwarding, or absolute-pose retargeting for UR5e."""
import time
from pathlib import Path
import math
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState

from robot_adapters import get_robot
from robot_adapters.models import load_robot_model, load_scene
from mujoco_sim.tasks import create_task
from .controller import PoseRetargeter


def parse_workspace(value):
    """Parse "xmin xmax ymin ymax zmin zmax"."""
    parts = str(value).split()
    if len(parts) != 6:
        raise ValueError(
            "follower_workspace must contain six numbers: "
            "xmin xmax ymin ymax zmin zmax"
        )
    return [float(part) for part in parts]


class RetargetNode(Node):
    def __init__(self):
        super().__init__("teleop_retargeting")
        for name, default in (("robot_id", "auto"), ("task_id", "red_cube_to_red_target"),
                              ("follower_workspace", "0.10 0.45 -0.25 0.25 0.015 0.35"),
                              ("gripper_open_fraction", 1.0),
                              ("rate", 50.0), ("max_joint_speed", 3.0), ("input_timeout", 0.25)):
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        task = create_task(str(value("task_id")))
        self.robot = get_robot(task.resolve_robot(str(value("robot_id"))))
        self.controller = None
        if self.robot.robot_id == "so101":
            # Only load the joint limits; no FK, IK or rate-limited interpolation.
            self.robot.bind(load_robot_model(self.robot))
        else:
            scene = Path(get_package_share_directory("mujoco_sim")) / "mujoco" / task.scene_file
            self.controller = PoseRetargeter(
                load_robot_model(get_robot("so101")),
                load_scene(scene),
                self.robot.robot_id,
                follower_workspace=parse_workspace(value("follower_workspace")),
                gripper_open_fraction=float(value("gripper_open_fraction")),
                max_joint_speed=float(value("max_joint_speed")),
            )
        self.source = get_robot("so101")
        self.leader = self.state = self.header = None
        self.leader_time = self.state_time = 0.0
        self.last_source_stamp = -1
        self.timeout = float(value("input_timeout"))
        rate = float(value("rate"))
        positive = (rate, self.timeout, float(value("max_joint_speed")))
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError("retargeting rate, timeout and speed must be positive")
        gripper_open_fraction = float(value("gripper_open_fraction"))
        if not math.isfinite(gripper_open_fraction) or not 0 < gripper_open_fraction <= 1:
            raise ValueError("gripper_open_fraction must be in (0, 1]")
        self.previous_tick = time.monotonic()
        self.create_subscription(JointState, "/leader/joint_states", self.on_leader, 1)
        self.create_subscription(JointState, "/sim/joint_states", self.on_state, 1)
        self.publisher = self.create_publisher(JointState, "/robot/joint_targets", 1)
        if self.controller is not None:
            self.create_timer(1.0 / rate, self.tick)
        mode = ("direct joint forwarding (no interpolation)" if self.controller is None
                else "absolute desk-frame pose retargeting")
        self.get_logger().info(f"SO101 -> {self.robot.robot_id}: {mode}; waiting for fresh state")

    def on_leader(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= self.last_source_stamp:
            return
        try:
            self.leader = self.source.ordered(msg.name, msg.position)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.last_source_stamp = stamp
        self.leader_time = time.monotonic()
        if self.controller is None and self.header is not None:
            if self.leader_time - self.state_time <= self.timeout:
                self.publish_target(self.robot.clamp(self.leader))

    def on_state(self, msg):
        if not msg.header.frame_id.startswith(self.robot.robot_id + "/epoch/"):
            return
        try:
            state = self.robot.ordered(msg.name, msg.position)
        except ValueError:
            return
        if self.header is None or self.header.frame_id != msg.header.frame_id:
            if self.controller is not None:
                self.controller.reset()
            self.leader = None
        self.state, self.header = state, msg.header
        self.state_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt, self.previous_tick = now - self.previous_tick, now
        if self.leader is None or self.state is None:
            return
        if now - min(self.leader_time, self.state_time) > self.timeout:
            self.controller.reset()
            self.leader = None
            self.get_logger().warning("teleop input stale; holding and awaiting re-anchor",
                                      throttle_duration_sec=2.0)
            return
        try:
            target = self.controller.update(self.leader, self.state, dt)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.publish_target(target)

    def publish_target(self, target):
        msg = JointState()
        msg.header = self.header
        msg.name, msg.position = list(self.robot.joint_names), target.tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RetargetNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
