"""Run after building and sourcing install/setup.bash; no hardware or GUI."""
from pathlib import Path
from types import SimpleNamespace
import json
import shutil

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_adapters import get_robot
from robot_adapters.models import load_scene, load_robot_model
from mujoco_sim.simulation import MujocoSimulation
from mujoco_sim.tasks import create_task
from teleop_retargeting.controller import PoseRetargeter
from so101_leader_bridge.so101_leader_driver import parse_response

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "src/mujoco_sim/mujoco"


def simulation(robot_id, task_id=None):
    if task_id is None:
        task_id = "ur5e_red_cube_to_target" if robot_id == "ur5e" else "red_cube_to_red_target"
    task = create_task(task_id)
    return MujocoSimulation(SCENES / task.scene_for(robot_id), None, task,
                            np.random.default_rng(42), robot_id)


@pytest.mark.parametrize("robot_id,n", [("so101", 6), ("ur5e", 7)])
def test_simulation_controls_and_reset(robot_id, n):
    sim = simulation(robot_id)
    assert len(sim.joint_positions()) == len(sim.action_positions()) == n
    sim.reset_task()
    targets = np.array(sim.robot.home)
    targets[-1] = sim.robot.limits[-1, 1]
    sim.set_targets(sim.robot.joint_names, targets)
    sim.apply_targets()
    assert sim.data.ctrl[sim.robot.aids[-1]] == pytest.approx(255 if robot_id == "ur5e" else targets[-1])
    assert sim.action_positions()[-1] == pytest.approx(targets[-1])
    for _ in range(500):
        sim.step()
    assert np.isfinite(sim.data.qpos).all()
    assert not any(w.number for w in sim.data.warning)
    sim.reset_task()
    assert sim.action_positions() == list(sim.robot.home)
    assert sim.joint_positions() == pytest.approx(sim.robot.home)
    assert np.max(np.abs(sim.data.qvel)) == 0
    if robot_id == "ur5e":
        cube = sim.data.joint("red_cube_free").qpos[:3]
        assert .205 <= cube[0] <= .295
        assert -.045 <= cube[1] <= .045
        assert cube[2] == pytest.approx(.013)
        assert sim.data.joint("robotiq_left_driver_joint").qpos[0] == 0


@pytest.mark.parametrize("task", ["red_cube_to_red_target", "red_blue_cubes_to_targets", "red_cube_in_drawer"])
def test_so101_tasks_and_unsupported_ur(task):
    sim = simulation("so101", task)
    sim.reset_task()
    with pytest.raises(ValueError):
        create_task(task).scene_for("ur5e")


def test_scene_selects_robot():
    task = create_task("ur5e_red_cube_to_target")
    assert task.resolve_robot() == "ur5e"
    assert task.scene_for() == "tasks/ur5e_red_cube_to_target.xml"
    with pytest.raises(ValueError):
        task.resolve_robot("so101")


def test_ordered_input_and_legacy_metadata():
    from vla_dataset.convert_mcap_to_lerobot import validate_manifest
    robot = get_robot("so101")
    assert robot.ordered(robot.joint_names[::-1], robot.home[::-1]) == pytest.approx(robot.home)
    for names, values in [(robot.joint_names[:-1], robot.home[:-1]),
                          (robot.joint_names, [float("nan")] * 6),
                          (("gripper",) * 6, robot.home)]:
        with pytest.raises(ValueError):
            robot.ordered(names, values)
    validate_manifest({}, robot)
    with pytest.raises(ValueError):
        validate_manifest({}, get_robot("ur5e"))


def test_retarget_direct_pose_speed_and_reset():
    sim = simulation("ur5e")
    controller = PoseRetargeter(load_robot_model(get_robot("so101")), sim.model, "ur5e")
    leader = np.array(controller.source.home)
    follower = np.array(sim.robot.home)
    leader_position, leader_rotation = controller._pose(
        controller.source, controller.ld, leader, controller.source_site)
    expected_position = controller.map_position(leader_position)
    expected_rotation = controller.map_orientation(leader_rotation)
    # The first sample is not anchored to the live pose: it commands the fixed
    # absolute target, but is still bounded by the joint-speed limit.
    moved = controller.update(leader, follower, .02)
    assert np.max(np.abs(moved - follower)) <= controller.max_speed * .02 + 1e-8
    q = follower
    for _ in range(400):
        q = controller.update(leader, q, .04)
    actual_position, actual_rotation = controller._pose(
        controller.robot, controller.fd, q, controller.target_site)
    assert np.linalg.norm(actual_position - expected_position) < .002
    residual = Rotation.from_matrix(expected_rotation @ actual_rotation.T).as_rotvec()
    assert np.linalg.norm(residual) < .05

    # The full leader rotation is mapped through one fixed TCP-axis correction.
    rotated_leader = np.array(leader, copy=True)
    rotated_leader[4] += .4
    rotated_position, rotated_rotation = controller._pose(
        controller.source, controller.ld, rotated_leader, controller.source_site)
    expected_position = controller.map_position(rotated_position)
    expected_rotation = controller.map_orientation(rotated_rotation)
    assert not np.allclose(expected_rotation, controller.follower_down_rotation)
    goal = controller.solve(expected_position, expected_rotation, follower)
    solved_position, solved_rotation = controller._pose(
        controller.robot, controller.fd, goal, controller.target_site)
    assert np.linalg.norm(expected_position - solved_position) < .001
    residual = Rotation.from_matrix(
        expected_rotation @ solved_rotation.T).as_rotvec()
    assert np.linalg.norm(residual) < .015
    q = follower
    for _ in range(400):
        q = controller.update(rotated_leader, q, .04)
    _, actual_rotation = controller._pose(
        controller.robot, controller.fd, q, controller.target_site)
    residual = Rotation.from_matrix(expected_rotation @ actual_rotation.T).as_rotvec()
    assert np.linalg.norm(residual) < .05

    # When the leader points straight down, the follower must point down too.
    down_leader = np.array(controller.source.down_joints)
    assert controller.map_orientation(controller.leader_down_rotation) == \
        pytest.approx(controller.follower_down_rotation)
    q = follower
    for _ in range(400):
        q = controller.update(down_leader, q, .04)
    _, actual_rotation = controller._pose(
        controller.robot, controller.fd, q, controller.target_site)
    residual = Rotation.from_matrix(
        controller.follower_down_rotation @ actual_rotation.T).as_rotvec()
    assert np.linalg.norm(residual) < .05

    # Reset clears only the smoother/IK seed; the absolute target is unchanged.
    controller.reset()
    first = controller.update(leader, follower, .02)
    assert np.max(np.abs(first - follower)) <= controller.max_speed * .02 + 1e-8

    _, rotation = controller._pose(sim.robot, controller.fd, follower, controller.target_site)
    for point in ([.2, -.12, .021], [.2, -.12, .15], [.25, .12, .03]):
        result = controller.solve(point, rotation, follower)
        actual, _ = controller._pose(sim.robot, controller.fd, result, controller.target_site)
        assert np.linalg.norm(actual - point) < .001
    with pytest.raises(ValueError):
        controller.solve([10, 10, 10], rotation, follower)
    assert sim.robot.gripper_from_leader(-.174533) == pytest.approx(.8)
    assert sim.robot.gripper_from_leader(1.74533) == pytest.approx(0)
    half_open = (-.174533 + 1.74533) / 2
    assert sim.robot.gripper_from_leader(half_open, .5) == pytest.approx(0)
    quarter_open = -.174533 + (1.74533 + .174533) / 4
    assert sim.robot.gripper_from_leader(quarter_open, .5) == pytest.approx(.4)


def test_retarget_workspace_mapping_clamping_and_validation():
    sim = simulation("ur5e")
    controller = PoseRetargeter(
        load_robot_model(get_robot("so101")), sim.model, "ur5e")

    assert controller.map_position(controller.source_workspace_min) == \
        pytest.approx(controller.workspace_min)
    assert controller.map_position(controller.source_workspace_max) == \
        pytest.approx(controller.workspace_max)
    midpoint = (
        controller.source_workspace_min + controller.source_workspace_max
    ) / 2
    assert controller.map_position(midpoint) == pytest.approx(
        (controller.workspace_min + controller.workspace_max) / 2
    )

    assert controller.map_position(
        controller.source_workspace_max + 1
    ) == pytest.approx(controller.workspace_max)
    assert controller.map_position(
        controller.source_workspace_min - 1
    ) == pytest.approx(controller.workspace_min)

    leader_model = load_robot_model(get_robot("so101"))
    for fraction in (0, 1.1, float("nan")):
        with pytest.raises(ValueError):
            PoseRetargeter(
                leader_model, sim.model, "ur5e",
                gripper_open_fraction=fraction,
            )
    for workspace in (
        [0.1, 0.2],
        [0.2, 0.1, -0.2, 0.2, 0.015, 0.35],
        [0.1, 0.45, -0.25, 0.25, 0.0, 0.35],
    ):
        with pytest.raises(ValueError):
            PoseRetargeter(
                leader_model, sim.model, "ur5e",
                follower_workspace=workspace,
            )


def test_serial_packet_validation():
    def packet(pid=1, error=0, value=1234):
        payload = [pid, 4, error, value & 255, value >> 8]
        return bytes([255, 255, *payload, (~sum(payload)) & 255])
    good = packet()
    assert parse_response(b"noise" + good, 1) == bytes([210, 4])
    for bad in [good[:-1], packet(2), packet(error=1), packet(value=5000), good[:-1] + b"\x00"]:
        assert parse_response(bad, 1) is None


def test_ur5e_physical_pick_and_place():
    sim = simulation("ur5e")
    sim.reset_task()
    controller = PoseRetargeter(load_robot_model(get_robot("so101")), sim.model, "ur5e")
    q = np.array(sim.robot.home)
    _, rotation = controller._pose(sim.robot, controller.fd, q, controller.target_site)
    cube_start = sim.data.joint("red_cube_free").qpos[:2].copy()
    target = sim.data.geom("red_place_target").xpos[:2].copy()
    stages = [("above", [*cube_start, .17], 0), ("lower", [*cube_start, .026], 0),
              ("grasp", [*cube_start, .026], .8), ("lift", [*cube_start, .17], .8),
              ("transfer", [*target, .17], .8), ("place", [*target, .034], .8),
              ("release", [*target, .034], 0)]
    for label, point, gripper in stages:
        goal = controller.solve(point, rotation, q)
        goal[-1] = gripper
        for _ in range(2000):
            q += np.clip(goal - q, -.0008, .0008)
            sim.set_targets(sim.robot.joint_names, q)
            sim.apply_targets()
            sim.step()
        if label == "lift":
            assert sim.data.joint("red_cube_free").qpos[2] > .12
    cube = sim.data.joint("red_cube_free").qpos[:3]
    assert np.linalg.norm(cube[:2] - target) < .03
    assert cube[2] < .04
    assert not any(w.number for w in sim.data.warning)


def test_policy_shape_and_optional_metadata(tmp_path):
    from robot_policy.act_node import validate_policy
    config = SimpleNamespace(input_features={"observation.state": SimpleNamespace(shape=(6,))},
                             output_features={"action": SimpleNamespace(shape=(6,))})
    validate_policy(config, get_robot("so101"), tmp_path)
    with pytest.raises(ValueError):
        validate_policy(config, get_robot("ur5e"), tmp_path)
    (tmp_path / "robot.json").write_text(json.dumps(get_robot("ur5e").metadata()))
    with pytest.raises(ValueError):
        validate_policy(config, get_robot("so101"), tmp_path)


@pytest.mark.parametrize("robot_id", ["so101", "ur5e"])
def test_scene_outside_source_tree(tmp_path, robot_id):
    # Imitate a copied install where sibling Python-package sources do not exist.
    shutil.copytree(SCENES, tmp_path / "mujoco")
    task_id = "ur5e_red_cube_to_target" if robot_id == "ur5e" else "red_cube_to_red_target"
    model = load_scene(tmp_path / "mujoco" / create_task(task_id).scene_for(robot_id))
    assert model.nu == len(get_robot(robot_id).joint_names)
