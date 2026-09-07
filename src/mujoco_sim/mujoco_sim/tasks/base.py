"""Common interfaces and camera declarations for simulation tasks."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ImageStreamSpec:
    topic: str
    info_topic: str
    observation_key: str
    preview_topic: str | None = None


@dataclass(frozen=True)
class CameraSpec:
    camera_id: str
    mjcf_name: str
    frame_id: str
    width: int = 640
    height: int = 480
    fps: float = 30.0
    rgb: ImageStreamSpec | None = None
    depth: ImageStreamSpec | None = None
    depth_range_m: tuple[float, float] | None = None


def camera_streams(cameras):
    for camera in cameras:
        if camera.rgb is not None:
            yield camera, "rgb", camera.rgb
        if camera.depth is not None:
            yield camera, "depth", camera.depth


def camera_manifest(cameras):
    result = [asdict(camera) for camera in cameras]
    for camera in result:
        if camera["depth_range_m"] is not None:
            camera["depth_range_m"] = list(camera["depth_range_m"])
    return result


def validate_cameras(cameras, primary_camera_id):
    if not cameras:
        raise ValueError("task must declare at least one camera")
    camera_ids = [camera.camera_id for camera in cameras]
    if len(camera_ids) != len(set(camera_ids)):
        raise ValueError("task camera_id values must be unique")
    if primary_camera_id not in camera_ids:
        raise ValueError("primary_camera_id must name a task camera")

    topics, info_topics, keys = [], [], []
    for camera in cameras:
        if not camera.mjcf_name or not camera.frame_id:
            raise ValueError("camera MJCF name and frame ID must not be empty")
        if camera.width < 1 or camera.height < 1:
            raise ValueError("camera width and height must be positive")
        if not math.isfinite(camera.fps) or camera.fps <= 0:
            raise ValueError("camera fps must be finite and positive")
        if camera.rgb is None and camera.depth is None:
            raise ValueError(f"camera {camera.camera_id!r} has no output streams")
        if camera.depth is not None:
            limits = camera.depth_range_m
            if (limits is None or len(limits) != 2
                    or not all(math.isfinite(value) for value in limits)
                    or limits[0] < 0 or limits[1] <= limits[0]):
                raise ValueError(
                    f"camera {camera.camera_id!r} requires a valid depth_range_m"
                )
        for _, _, stream in camera_streams((camera,)):
            if not stream.topic or not stream.info_topic or not stream.observation_key:
                raise ValueError("camera stream topics and observation key must not be empty")
            if not stream.observation_key.startswith("observation.images."):
                raise ValueError(
                    "camera observation keys must start with 'observation.images.'"
                )
            topics.append(stream.topic)
            info_topics.append(stream.info_topic)
            keys.append(stream.observation_key)

    for label, values in (("image topics", topics), ("CameraInfo topics", info_topics),
                          ("observation keys", keys)):
        if len(values) != len(set(values)):
            raise ValueError(f"task camera {label} must be unique")

    primary = next(camera for camera in cameras if camera.camera_id == primary_camera_id)
    if primary.rgb is None:
        raise ValueError("the primary task camera must provide RGB")


class SimulationTask:
    """Base interface implemented by each MuJoCo training task."""

    task_id = ""
    scene_file = ""
    language_instruction = ""
    robot_id = "so101"
    cameras = ()
    primary_camera_id = ""
    reset_source_robot_id = None

    def __init__(self):
        validate_cameras(self.cameras, self.primary_camera_id)

    def resolve_robot(self, requested="auto"):
        if requested not in ("auto", self.robot_id):
            raise ValueError(f"task {self.task_id!r} requires robot {self.robot_id!r}, not {requested!r}")
        return self.robot_id

    def scene_for(self, robot_id="auto"):
        self.resolve_robot(robot_id)
        return self.scene_file

    def reset(self, model, data, rng):
        raise NotImplementedError

    def evaluate(self, model, data):
        """Return a future success result; detection is not implemented yet."""
        return None
