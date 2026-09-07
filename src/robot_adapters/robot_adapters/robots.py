"""Public state/action vectors are ordered joint angles in radians."""
from abc import ABC, abstractmethod

import numpy as np


class RobotAdapter(ABC):
    robot_id: str
    joint_names: tuple
    model_joints: tuple
    actuators: tuple
    home: tuple
    down_joints: tuple
    minimum_tcp_height: float
    tcp_workspace: tuple
    tcp_site: str
    model_file: str

    def bind(self, model):
        self.model = model
        self.qadr = np.array([model.joint(n).qposadr[0] for n in self.model_joints])
        self.dadr = np.array([model.joint(n).dofadr[0] for n in self.model_joints])
        self.aids = np.array([model.actuator(n).id for n in self.actuators])
        self.limits = np.array([model.joint(n).range for n in self.model_joints])
        self.robot_joint_ids = [model.joint(n).id for n in self.model_joints]
        return self

    def positions(self, data):
        return data.qpos[self.qadr].copy()

    def ordered(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError("joint names/positions must be unique and equally sized")
        values = dict(zip(names, positions))
        try:
            result = np.array([values[n] for n in self.joint_names], dtype=float)
        except KeyError as exc:
            raise ValueError(f"missing joint {exc.args[0]}") from exc
        if not np.isfinite(result).all():
            raise ValueError("joint positions must be finite")
        return result

    def clamp(self, targets):
        return np.clip(targets, self.limits[:, 0], self.limits[:, 1])

    def reset(self, data):
        # Reset all joints in the robot subtree, including passive gripper links.
        base = self.model.body("base").id
        for j in range(self.model.njnt):
            body = int(self.model.jnt_bodyid[j])
            while body and body != base:
                body = int(self.model.body_parentid[body])
            if body != base:
                continue
            q, v = self.model.jnt_qposadr[j], self.model.jnt_dofadr[j]
            data.qpos[q] = self.model.qpos0[q]
            data.qvel[v] = data.qacc_warmstart[v] = data.qfrc_applied[v] = 0
        self.set_positions(data, self.home)
        self.apply(data, np.array(self.home))

    def set_positions(self, data, positions):
        data.qpos[self.qadr] = positions

    @abstractmethod
    def apply(self, data, targets):
        pass

    @abstractmethod
    def gripper_from_leader(self, angle, open_fraction=1.0):
        pass

    def metadata(self):
        return {"robot_id": self.robot_id, "joint_names": list(self.joint_names),
                "joint_units": ["rad"] * len(self.joint_names)}


class SO101Adapter(RobotAdapter):
    robot_id = "so101"
    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                   "wrist_flex", "wrist_roll", "gripper")
    model_joints = joint_names
    actuators = joint_names
    home = (0.0389, -1.7350, 1.5860, 1.1091, -0.0188, -0.1729)
    # A pose whose grasp axis (wrist -> fingertip) is vertical down; used as
    # the orientation reference when retargeting to other arms.
    down_joints = (-0.063, -1.058, 1.059, 1.477, -0.025, -0.1729)
    minimum_tcp_height = 0.011
    # Fixed effective TCP range in the SO101 base frame. Each axis maps
    # linearly onto the configured follower workspace.
    tcp_workspace = (0.151, 0.274, -0.088, 0.078, 0.011, 0.1785)
    tcp_site = "gripperframe"
    model_file = "so101/so101_arm.xml"

    def apply(self, data, targets):
        data.ctrl[self.aids] = targets

    def gripper_from_leader(self, angle, open_fraction=1.0):
        return float(angle)


class UR5eRobotiqAdapter(RobotAdapter):
    robot_id = "ur5e"
    joint_names = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "gripper")
    model_joints = joint_names[:-1] + ("robotiq_right_driver_joint",)
    actuators = ("shoulder_pan", "shoulder_lift", "elbow",
                 "wrist_1", "wrist_2", "wrist_3", "robotiq_fingers_actuator")
    home = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0)
    down_joints = home
    minimum_tcp_height = 0.015
    tcp_site = "tcp"
    model_file = "ur5e/ur5e_robotiq.xml"

    def set_positions(self, data, positions):
        super().set_positions(data, positions)
        left = self.model.joint("robotiq_left_driver_joint").qposadr[0]
        data.qpos[left] = positions[-1]

    def positions(self, data):
        result = super().positions(data)
        left = self.model.joint("robotiq_left_driver_joint").qposadr[0]
        result[-1] = (result[-1] + data.qpos[left]) / 2
        return result

    def apply(self, data, targets):
        controls = np.array(targets, copy=True)
        controls[-1] *= 255.0 / 0.8
        data.ctrl[self.aids] = controls

    def gripper_from_leader(self, angle, open_fraction=1.0):
        leader_openness = np.clip(
            (angle + 0.174533) / (1.74533 + 0.174533), 0, 1
        )
        follower_openness = np.clip(leader_openness / open_fraction, 0, 1)
        return float(0.8 * (1 - follower_openness))


ROBOTS = {"so101": SO101Adapter, "ur5e": UR5eRobotiqAdapter}


def get_robot(robot_id="so101"):
    try:
        return ROBOTS[robot_id]()
    except KeyError as exc:
        raise ValueError(f"unknown robot_id {robot_id!r}; available: {', '.join(ROBOTS)}") from exc
