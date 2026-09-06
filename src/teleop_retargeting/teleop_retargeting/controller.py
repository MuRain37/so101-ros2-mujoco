"""Kinematic retargeting; this module never opens hardware or publishes ROS."""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from robot_core import get_robot


class PoseRetargeter:
    """Map the SO101 leader end effector absolutely onto a fixed follower frame.

    The mapping never depends on the follower's pose when teleop starts:

      follower_x = follower_reference_x + sx * (leader_x - leader_reference_x)
      follower_y = follower_reference_y + sy * (leader_y - leader_reference_y)
      follower_z = follower_min_height
                   + sz * (leader_z - leader_min_height)

    If ``leader_range`` and ``follower_range`` (xmin xmax ymin ymax zmin zmax)
    are both provided, each axis is instead mapped linearly between the two
    boxes and the result is clamped inside the follower box.

    ``leader_reference``/``follower_reference`` are the two arms' fixed home
    grasp-reference positions. ``leader_min_height`` and ``follower_min_height``
    are the grasp-reference heights above each arm's own desk when its
    fingertips just touch the desk, so a leader resting at its lowest usable
    position drives the follower to its own lowest usable position.

    The follower orientation follows the leader absolutely, using each arm's
    own gripper-down pose as the reference: the leader rotation relative to its
    down orientation is applied (scaled by ``orientation_scale``) to the
    follower's down orientation. Thus when the SO101 gripper points straight
    down at the desk, the follower does too, independent of startup pose.
    """

    def __init__(self, leader_model, follower_model, robot_id,
                 position_scale=2.0, orientation_scale=1.0,
                 leader_min_height=0.011, follower_min_height=0.015,
                 leader_range=None, follower_range=None,
                 max_joint_speed=0.8):
        self.source = get_robot("so101").bind(leader_model)
        self.robot = get_robot(robot_id).bind(follower_model)
        self.ld = mujoco.MjData(leader_model)
        self.fd = mujoco.MjData(follower_model)
        self.source.reset(self.ld)
        self.robot.reset(self.fd)
        scale = np.asarray(position_scale, dtype=float)
        if scale.ndim == 0:
            scale = np.full(3, float(scale))
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise ValueError("position_scale must be a positive scalar or length-3 sequence")
        self.scale = scale
        self.orientation_scale = float(orientation_scale)
        if not np.isfinite(self.orientation_scale) or self.orientation_scale < 0:
            raise ValueError("orientation_scale must be finite and non-negative")
        mins = (leader_min_height, follower_min_height)
        if not all(np.isfinite(x) for x in mins) or any(x < 0 for x in mins):
            raise ValueError("leader/follower_min_height must be finite and non-negative")
        self.leader_min = float(leader_min_height)
        self.follower_min = float(follower_min_height)
        self.leader_range = self._parse_range(leader_range, "leader_range")
        self.follower_range = self._parse_range(follower_range, "follower_range")
        if (self.leader_range is None) != (self.follower_range is None):
            raise ValueError("leader_range and follower_range must be provided together")
        self.max_speed = float(max_joint_speed)
        if not np.isfinite(self.max_speed) or self.max_speed <= 0:
            raise ValueError("max_joint_speed must be finite and positive")
        self.source_site = leader_model.site(self.source.tcp_site).id
        self.target_site = follower_model.site(self.robot.tcp_site).id
        leader_home, _ = self._pose(
            self.source, self.ld, np.array(self.source.home), self.source_site)
        follower_home, _ = self._pose(
            self.robot, self.fd, np.array(self.robot.home), self.target_site)
        # Fixed desk-frame references: leader home drives the follower home.
        self.leader_reference = leader_home
        self.follower_reference = follower_home
        self.leader_down_rotation = self._down_rotation(
            self.source, self.ld, self.source_site)
        self.follower_down_rotation = self._down_rotation(
            self.robot, self.fd, self.target_site)
        self.reset()

    @staticmethod
    def _parse_range(value, name):
        if value is None:
            return None
        arr = np.asarray(value, dtype=float)
        if arr.shape != (6,) or not np.isfinite(arr).all():
            raise ValueError(f"{name} must contain six finite numbers "
                             "(xmin xmax ymin ymax zmin zmax)")
        if (arr[1::2] <= arr[::2]).any():
            raise ValueError(f"{name} maxima must be greater than minima")
        return arr

    def reset(self):
        # The absolute mapping needs no re-anchoring; only the IK seed and the
        # speed limiter state are cleared so they follow the live robot.
        self.last = None

    def _pose(self, robot, data, joints, site):
        data.qpos[robot.qadr] = joints
        mujoco.mj_forward(robot.model, data)
        return data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()

    def _down_rotation(self, robot, data, site):
        """FK rotation of the arm when its gripper points vertically down."""
        joints = getattr(robot, "down_joints", None)
        if joints is None:
            joints = robot.home
        return self._pose(robot, data, np.array(joints), site)[1]

    def map_position(self, leader_position):
        """Map a leader grasp-reference position to a follower target position."""
        lp = np.asarray(leader_position, dtype=float)
        if self.leader_range is not None:
            source_min = self.leader_range[::2]
            target_min = self.follower_range[::2]
            gain = ((self.follower_range[1::2] - target_min)
                    / (self.leader_range[1::2] - source_min))
            return np.clip(target_min + gain * (lp - source_min),
                           target_min, self.follower_range[1::2])
        fp = self.follower_reference[:2] + self.scale[:2] * (lp[:2] - self.leader_reference[:2])
        fz = self.follower_min + self.scale[2] * (lp[2] - self.leader_min)
        # A leader cannot usefully go below its desk-contact height, so the
        # follower is clamped to its own lowest grasp height as well.
        fz = max(self.follower_min, fz)
        return np.r_[fp, fz]

    def map_orientation(self, leader_rotation):
        """Map a leader grasp-reference rotation onto the fixed follower frame."""
        relative = Rotation.from_matrix(
            np.asarray(leader_rotation) @ self.leader_down_rotation.T)
        return (Rotation.from_rotvec(
            self.orientation_scale * relative.as_rotvec()).as_matrix()
            @ self.follower_down_rotation)

    def solve(self, position, rotation, seed):
        q = np.array(seed, copy=True)
        n = len(self.robot.joint_names) - 1
        jp = np.zeros((3, self.robot.model.nv))
        jr = np.zeros_like(jp)
        # Balance metre and radian residuals without imposing redundant source joints.
        weight = 0.2
        for _ in range(60):
            p, r = self._pose(self.robot, self.fd, q, self.target_site)
            pe = np.asarray(position) - p
            re = Rotation.from_matrix(rotation @ r.T).as_rotvec()
            if np.linalg.norm(pe) < 0.001 and np.linalg.norm(re) < 0.015:
                return q
            mujoco.mj_jacSite(self.robot.model, self.fd, jp, jr, self.target_site)
            jac = np.vstack((jp[:, self.robot.dadr[:n]], weight * jr[:, self.robot.dadr[:n]]))
            error = np.r_[pe, weight * re]
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 0.015 ** 2 * np.eye(6), error)
            q[:n] += np.clip(delta, -0.15, 0.15)
            q = self.robot.clamp(q)
        raise ValueError("IK target unreachable or unconverged; holding last target")

    def update(self, leader, follower, dt):
        lp, lr = self._pose(self.source, self.ld, leader, self.source_site)
        position = self.map_position(lp)
        rotation = self.map_orientation(lr)
        if self.last is None:
            self.last = self.robot.clamp(follower)
        goal = self.solve(position, rotation, self.last)
        goal[-1] = self.robot.gripper_from_leader(leader[-1])
        step = self.max_speed * max(0.0, min(dt, 0.04))
        self.last += np.clip(goal - self.last, -step, step)
        self.last = self.robot.clamp(self.last)
        return self.last.copy()
