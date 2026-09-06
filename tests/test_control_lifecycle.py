"""ROS callback lifecycle checks without serial hardware or a viewer."""
import time
from copy import deepcopy
from unittest.mock import Mock
import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from mujoco_sim.simulator import MujocoSimulator
from teleop_retargeting.node import RetargetNode
from robot_core import get_robot


def test_reset_rejects_previous_commands_and_timeout_reprimes():
    rclpy.init(args=["--ros-args", "-p", "task_id:=ur5e_red_cube_to_target"])
    sim = retarget = None
    try:
        sim = MujocoSimulator()
        sim._simulation = Mock()
        msg = JointState()
        msg.name, msg.position = list(sim._robot.joint_names), list(sim._robot.home)
        msg.header.frame_id = sim.command_epoch
        sim._on_joint_state(msg)
        assert sim._simulation.set_targets.call_count == 1
        sim._reset_task("test")
        sim._on_joint_state(msg)
        assert sim._simulation.set_targets.call_count == 1
        msg.header.frame_id = sim.command_epoch
        sim._on_joint_state(msg)
        assert sim._simulation.set_targets.call_count == 2

        retarget = RetargetNode()
        retarget.on_state(msg)
        source = JointState()
        leader = get_robot("so101")
        source.name, source.position = list(leader.joint_names), list(leader.home)
        source.header.stamp.sec = 1
        retarget.on_leader(source)
        retarget.tick()
        assert retarget.controller.last is not None
        assert retarget.leader is not None
        # A duplicate source stamp must not refresh the dropout watchdog.
        retarget.leader_time = time.monotonic() - 1
        retarget.on_leader(source)
        retarget.tick()
        assert retarget.controller.last is None and retarget.leader is None
        source.header.stamp.sec = 2
        retarget.on_leader(source)
        retarget.tick()
        assert retarget.controller.last is not None
        msg = deepcopy(msg)  # ROS delivers a new message, not an in-place mutation.
        msg.header.frame_id = "ur5e/epoch/999"
        retarget.on_state(msg)
        assert retarget.controller.last is None and retarget.leader is None
    finally:
        if retarget is not None:
            retarget.destroy_node()
        if sim is not None:
            sim.destroy_node()
        rclpy.shutdown()


def test_so101_forwards_immediately_without_speed_limit():
    rclpy.init()
    node = None
    try:
        node = RetargetNode()
        node.publisher = Mock()
        assert node.controller is None
        assert len(list(node.timers)) == 0
        state = JointState()
        state.header.frame_id = "so101/epoch/1"
        state.name, state.position = list(node.robot.joint_names), list(node.robot.home)
        node.on_state(state)
        source = deepcopy(state)
        source.header.stamp.sec = 1
        source.position[0] += 1.0
        node.on_leader(source)  # No timer tick needed, including the first sample.
        assert node.publisher.publish.call_count == 1
        command = node.publisher.publish.call_args.args[0]
        assert command.position == source.position
        assert command.header.frame_id == state.header.frame_id
        # Reject duplicate, invalid and stale-state input without emitting commands.
        node.on_leader(source)
        source.header.stamp.sec = 2
        source.position[0] = float("nan")
        node.on_leader(source)
        assert node.publisher.publish.call_count == 1
        source.position[0] = .5
        node.state_time = time.monotonic() - 1
        node.on_leader(source)
        assert node.publisher.publish.call_count == 1
        state = deepcopy(state)
        state.header.frame_id = "so101/epoch/2"
        node.on_state(state)
        source.header.stamp.sec = 3
        node.on_leader(source)
        assert node.publisher.publish.call_count == 2
        assert node.publisher.publish.call_args.args[0].header.frame_id == "so101/epoch/2"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
