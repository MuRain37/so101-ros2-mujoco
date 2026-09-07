"""Shared FK, workspace mapping, and IK for robot pose retargeting."""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .robots import get_robot


def parse_workspace(value):
    """Parse an xmin/xmax/ymin/ymax/zmin/zmax workspace."""
    parts = str(value).split()
    if len(parts) != 6:
        raise ValueError(
            "follower_workspace must contain six numbers: "
            "xmin xmax ymin ymax zmin zmax"
        )
    return [float(part) for part in parts]


class PoseRetargeter:
    """Map the SO101 TCP pose onto a follower TCP pose in aligned base frames."""

    def __init__(self, leader_model, follower_model, robot_id,
                 follower_workspace=(0.10, 0.45, -0.25, 0.25, 0.015, 0.35),
                 gripper_open_fraction=1.0, max_joint_speed=3.0,
                 source_robot_id="so101"):
        self.source = get_robot(source_robot_id).bind(leader_model)
        self.robot = get_robot(robot_id).bind(follower_model)
        self.ld = mujoco.MjData(leader_model)
        self.fd = mujoco.MjData(follower_model)
        self.source.reset(self.ld)
        self.robot.reset(self.fd)
        leader_workspace = np.asarray(self.source.tcp_workspace, dtype=float)
        self.source_workspace_min = leader_workspace[::2]
        self.source_workspace_max = leader_workspace[1::2]
        workspace = np.asarray(follower_workspace, dtype=float)
        if workspace.shape != (6,) or not np.isfinite(workspace).all():
            raise ValueError(
                "follower_workspace must contain six finite numbers "
                "(xmin xmax ymin ymax zmin zmax)"
            )
        self.workspace_min = workspace[::2]
        self.workspace_max = workspace[1::2]
        if (self.workspace_max <= self.workspace_min).any():
            raise ValueError("follower_workspace maxima must be greater than minima")
        if self.workspace_min[2] < self.robot.minimum_tcp_height:
            raise ValueError(
                "follower_workspace zmin must not be below the robot minimum TCP height"
            )
        self.gripper_open_fraction = float(gripper_open_fraction)
        if not 0 < self.gripper_open_fraction <= 1:
            raise ValueError("gripper_open_fraction must be in (0, 1]")
        self.max_speed = float(max_joint_speed)
        if not np.isfinite(self.max_speed) or self.max_speed <= 0:
            raise ValueError("max_joint_speed must be finite and positive")
        self.source_site = leader_model.site(self.source.tcp_site).id
        self.target_site = follower_model.site(self.robot.tcp_site).id
        self.leader_down_rotation = self._down_rotation(
            self.source, self.ld, self.source_site)
        self.follower_down_rotation = self._down_rotation(
            self.robot, self.fd, self.target_site)
        self.tool_alignment = (
            self.leader_down_rotation.T @ self.follower_down_rotation
        )
        self.reset()

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
        return self._pose(robot, data, np.array(robot.down_joints), site)[1]

    def map_position(self, leader_position):
        """Map the fixed leader workspace onto the configured follower workspace."""
        leader_position = np.asarray(leader_position, dtype=float)
        fraction = (
            (leader_position - self.source_workspace_min)
            / (self.source_workspace_max - self.source_workspace_min)
        )
        return self.workspace_min + np.clip(fraction, 0.0, 1.0) * (
            self.workspace_max - self.workspace_min
        )

    def map_orientation(self, leader_rotation):
        """Map the full leader rotation while correcting fixed TCP-axis differences."""
        return np.asarray(leader_rotation) @ self.tool_alignment

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

    def goal(self, leader, seed):
        """Return the unsmoothed follower joints for one leader pose."""
        lp, lr = self._pose(self.source, self.ld, leader, self.source_site)
        goal = self.solve(
            self.map_position(lp), self.map_orientation(lr), seed
        )
        goal[-1] = self.robot.gripper_from_leader(
            leader[-1], self.gripper_open_fraction
        )
        return goal

    def update(self, leader, follower, dt):
        if self.last is None:
            self.last = self.robot.clamp(follower)
        goal = self.goal(leader, self.last)
        step = self.max_speed * max(0.0, min(dt, 0.04))
        self.last += np.clip(goal - self.last, -step, step)
        self.last = self.robot.clamp(self.last)
        return self.last.copy()
