"""Multi-policy dispatch and ROS lifecycle without downloading model weights."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
import torch
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robot_adapters import get_robot
from mujoco_sim.tasks import create_task
from robot_policy import policy_node as module
from lerobot.policies.pretrained import PreTrainedPolicy


@pytest.mark.parametrize("kind", ["act", "smolvla", "pi0"])
def test_checkpoint_dispatch(tmp_path, monkeypatch, kind):
    task = create_task("ur5e_red_blue_cubes_to_targets")
    config = module.PreTrainedConfig.get_choice_class(kind)(
        input_features={}, output_features={}, device="cpu"
    )
    from lerobot.configs.types import FeatureType, PolicyFeature
    config.input_features = {"observation.state": PolicyFeature(type=FeatureType.STATE, shape=(7,))}
    for camera in task.cameras:
        config.input_features[camera.rgb.observation_key] = PolicyFeature(
            type=FeatureType.VISUAL, shape=(3, camera.height, camera.width)
        )
    config.output_features = {"action": PolicyFeature(type=FeatureType.ACTION, shape=(7,))}
    config.save_pretrained(tmp_path)
    for name in ("model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"):
        (tmp_path / name).touch()
    policy = Mock(config=config)
    klass = Mock()
    klass.from_pretrained.return_value = policy
    factory = Mock(return_value=klass)
    monkeypatch.setattr(module, "get_policy_class", factory)
    base_load = Mock(return_value=policy)
    monkeypatch.setattr(PreTrainedPolicy, "from_pretrained", classmethod(base_load))
    processors = (Mock(), Mock())
    make = Mock(return_value=processors)
    monkeypatch.setattr(module, "make_pre_post_processors", make)
    result = module.load_policy(tmp_path, get_robot("ur5e"), task, torch.device("cpu"))
    assert result == (policy, *processors)
    factory.assert_called_once_with(kind)
    loader = base_load if kind == "pi0" else klass.from_pretrained
    assert loader.call_args.kwargs["config"].device == "cpu"
    assert loader.call_args.kwargs["strict"] is True
    assert make.call_args.kwargs["preprocessor_overrides"] == {"device_processor": {"device": "cpu"}}
    policy.eval.assert_called_once()
    loader.side_effect = ValueError("bad weights")
    with pytest.raises(RuntimeError, match=f"failed to load {kind}.*bad weights"):
        module.load_policy(tmp_path, get_robot("ur5e"), task, "cpu")


@pytest.mark.parametrize("config,expected", [
    ({"type": "other"}, "unsupported policy type"),
    ({"type": "pi0"}, "missing checkpoint file: model.safetensors"),
])
def test_bad_checkpoint(tmp_path, config, expected):
    (tmp_path / "config.json").write_text(json.dumps(config))
    with pytest.raises(RuntimeError, match=expected):
        module.load_policy(tmp_path, get_robot(), create_task("red_cube_to_red_target"), "cpu")


def test_required_local_path(tmp_path):
    with pytest.raises(ValueError, match="policy_path"):
        module.load_policy("", None, None, "cpu")
    with pytest.raises(RuntimeError, match="config.json"):
        module.load_policy(tmp_path / "absent", None, None, "cpu")


@pytest.fixture
def node(monkeypatch):
    policy = Mock(config=SimpleNamespace(type="smolvla", use_amp=True))
    monkeypatch.setattr(module, "load_policy", Mock(return_value=(policy, Mock(), Mock())))
    rclpy.init(args=["--ros-args", "-p", "task_id:=ur5e_red_blue_cubes_to_targets", "-p", "device:=cpu"])
    instance = module.PolicyNode()
    instance._publisher = Mock()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def feed_observations(node, epoch=1):
    msg = JointState()
    msg.header.frame_id = f"ur5e/epoch/{epoch}"
    msg.name = list(node._robot.joint_names)
    msg.position = list(node._robot.home)
    node._on_state(msg)
    for key in node._images:
        node._images[key] = np.zeros((480, 640, 3), dtype=np.uint8)
    return msg


def test_inference_and_epoch_reset(node, monkeypatch):
    predict = Mock(return_value=torch.zeros(1, 7, dtype=torch.bfloat16))
    monkeypatch.setattr(module, "predict_action", predict)
    node._infer()
    predict.assert_not_called()
    msg = feed_observations(node)
    node._infer()
    assert predict.call_args.kwargs["task"] == node._task.language_instruction
    assert predict.call_args.kwargs["use_amp"] is True
    assert node._publisher.publish.call_args.args[0].header.frame_id == "ur5e/epoch/1"
    node._instruction = "Put the red cube in the drawer."
    node._infer()
    assert predict.call_args.kwargs["task"] == node._instruction
    for obj in (node._policy, node._preprocessor, node._postprocessor):
        obj.reset.reset_mock()
    msg.header.frame_id = "ur5e/epoch/2"
    node._on_state(msg)
    for obj in (node._policy, node._preprocessor, node._postprocessor):
        obj.reset.assert_called_once()
    assert all(image is None for image in node._images.values())
    calls = predict.call_count
    node._infer()
    assert predict.call_count == calls


@pytest.mark.parametrize("action", [torch.zeros(1, 6), torch.full((1, 7), float("nan"))])
def test_invalid_action_not_published(node, monkeypatch, action):
    feed_observations(node)
    monkeypatch.setattr(module, "predict_action", Mock(return_value=action))
    node._infer()
    node._publisher.publish.assert_not_called()


def test_service_reset_pauses_and_clears(node, monkeypatch):
    feed_observations(node)
    predict = Mock()
    monkeypatch.setattr(module, "predict_action", predict)
    future = Mock()
    future.done.return_value = False
    node._sim_reset_client = Mock()
    node._sim_reset_client.call_async.return_value = future
    response = node._on_reset_episode(Trigger.Request(), Trigger.Response())
    assert response.success and node._state is None
    assert not node._on_reset_episode(Trigger.Request(), Trigger.Response()).success
    node._infer()
    predict.assert_not_called()
    future.done.return_value = True
    future.result.return_value = Trigger.Response(success=True)
    node._infer()
    assert node._reset_future is None
    assert node._state is None
    predict.assert_not_called()


def test_pi0_strict_loader_rejects_incompatible_weights(tmp_path, monkeypatch):
    from safetensors.torch import save_file
    config = SimpleNamespace(device="cpu", type="pi0")
    monkeypatch.setattr(module.PreTrainedConfig, "from_pretrained", Mock(return_value=config))
    monkeypatch.setattr(module, "validate_policy", Mock())

    class TinyPolicy(torch.nn.Module):
        _load_as_safetensor = PreTrainedPolicy._load_as_safetensor

        def __init__(self, config):
            super().__init__()
            self.config = config
            self.weight = torch.nn.Parameter(torch.zeros(1))

    monkeypatch.setattr(module, "get_policy_class", Mock(return_value=TinyPolicy))
    monkeypatch.setattr(module, "make_pre_post_processors", Mock(return_value=(Mock(), Mock())))
    (tmp_path / "config.json").write_text('{"type":"pi0"}')
    for name in ("policy_preprocessor.json", "policy_postprocessor.json"):
        (tmp_path / name).write_text('{}')
    save_file({"weight": torch.ones(1)}, tmp_path / "model.safetensors")
    policy, _, _ = module.load_policy(tmp_path, None, None, "cpu")
    assert policy.weight.item() == 1
    save_file({"wrong_key": torch.ones(1)}, tmp_path / "model.safetensors")
    with pytest.raises(RuntimeError, match="failed to load pi0"):
        module.load_policy(tmp_path, None, None, "cpu")


@pytest.mark.parametrize("instruction", ["", "Put red: in drawer"])
def test_launch_interface(monkeypatch, instruction):
    import importlib.util
    from pathlib import Path
    from launch.actions import DeclareLaunchArgument
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    path = Path(__file__).resolve().parents[1] / "src/robot_bringup/launch/policy_sim.launch.py"
    spec = importlib.util.spec_from_file_location("policy_launch", path)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    nodes = []
    real_node = launch_module.Node

    def capture(**kwargs):
        nodes.append(kwargs)
        return real_node(**kwargs)

    monkeypatch.setattr(launch_module, "Node", capture)
    description = launch_module.generate_launch_description()
    args = {a.name: a for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    assert args["policy_path"].default_value is None
    context = LaunchContext()
    context.launch_configurations.update({
        "policy_path": "/tmp/checkpoint", "task_id": "ur5e_red_cube_to_target",
        "robot_id": "auto", "device": "cpu", "inference_rate": "30.0", "task_instruction": instruction,
    })
    policy_node = next(n for n in nodes if n["package"] == "robot_policy" and n["executable"] == "policy_inference")
    params = evaluate_parameters(context, normalize_parameters(policy_node["parameters"]))[0]
    assert params["task_instruction"] == instruction
    assert params["policy_path"] == "/tmp/checkpoint"
