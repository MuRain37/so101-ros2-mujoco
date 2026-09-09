"""Optional deployment poses, from checkpoint configuration to simulation reset."""
import json
from pathlib import Path

import numpy as np
import pytest
from launch import LaunchContext
from robot_bringup.validation import configure_initial_pose
from mujoco_sim.simulation import MujocoSimulation
from mujoco_sim.tasks import create_task

ROOT = Path(__file__).resolve().parents[1]


def configure(path, task_id):
    context = LaunchContext()
    context.launch_configurations.update(policy_path=str(path), robot_id="ur5e", task_id=task_id)
    for action in configure_initial_pose(context):
        action.execute(context)
    return context.launch_configurations


def test_missing_pose_uses_default(tmp_path):
    config = configure(tmp_path, "ur5e_red_cube_in_drawer")
    assert config["initial_joint_positions"] == ""


@pytest.mark.parametrize("task_id,expected", [
    ("ur5e_red_blue_cubes_to_targets", [-1.5183004140853882, -1.4680863618850708,
     2.6622369289398193, -3.186650514602661, -1.5933756828308105,
     0.13179104030132294, 0.7821245789527893]),
    ("ur5e_red_cube_in_drawer", [-1.5623246431350708, -1.4814194440841675,
     2.683684825897217, -3.1861424446105957, -1.6042845249176025,
     0.030764548107981682, 0.7816399931907654]),
])
def test_checkpoint_pose_start_and_reset(tmp_path, task_id, expected):
    from robot_adapters import get_robot
    pose = dict(robot_id="ur5e", task_id=task_id, unit="rad",
                joint_names=list(get_robot("ur5e").joint_names), positions=expected[:])
    # Files may order joints differently; public state order must stay unchanged.
    pose["joint_names"].reverse()
    pose["positions"].reverse()
    (tmp_path / "initial_pose.json").write_text(json.dumps(pose))
    config = configure(tmp_path, task_id)
    positions = list(map(float, config["initial_joint_positions"].split()))
    assert positions == expected
    task = create_task(task_id)
    sim = MujocoSimulation(ROOT / "src/mujoco_sim/mujoco" / task.scene_file,
                           None, task, np.random.default_rng(42), "ur5e",
                           initial_joint_positions=positions)
    assert sim.joint_positions() == pytest.approx(expected)
    sim.set_targets(sim.robot.joint_names, sim.robot.home)
    for _ in range(20):
        sim.apply_targets()
        sim.step()
    sim.reset_task()
    assert sim.joint_positions() == pytest.approx(expected)
    assert sim.action_positions() == pytest.approx(expected)
    with pytest.raises(ValueError, match="joint limits"):
        MujocoSimulation(ROOT / "src/mujoco_sim/mujoco" / task.scene_file,
                         None, task, np.random.default_rng(42), "ur5e",
                         initial_joint_positions=[100.] * 7)


@pytest.mark.parametrize("change", [
    {"robot_id": "so101"}, {"task_id": "wrong_task"}, {"unit": "deg"},
    {"positions": [0.]}, {"positions": [float("nan")] * 7},
    {"positions": [True] * 7}, {"joint_names": ["gripper"] * 7},
])
def test_invalid_pose_does_not_fall_back(tmp_path, change):
    from robot_adapters import get_robot
    robot = get_robot("ur5e")
    pose = dict(robot_id="ur5e", task_id="ur5e_red_cube_in_drawer", unit="rad",
                joint_names=list(robot.joint_names), positions=list(robot.home))
    pose.update(change)
    (tmp_path / "initial_pose.json").write_text(json.dumps(pose))
    with pytest.raises(ValueError, match="Invalid initial pose"):
        configure(tmp_path, "ur5e_red_cube_in_drawer")


def test_malformed_pose_does_not_fall_back(tmp_path):
    (tmp_path / "initial_pose.json").write_text("{broken")
    with pytest.raises(ValueError, match="Invalid initial pose"):
        configure(tmp_path, "ur5e_red_cube_in_drawer")
