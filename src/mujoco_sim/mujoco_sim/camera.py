"""Task-configured MuJoCo camera rendering and ROS 2 image publication."""

from math import radians, tan

import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .tasks.base import CameraSpec

CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


def rgb_image_array(message: Image) -> np.ndarray:
    if message.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported RGB image encoding: {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, : message.width * 3].reshape(
        message.height, message.width, 3
    ).copy()
    return image if message.encoding == "rgb8" else image[:, :, ::-1].copy()


def depth_image_array(message: Image) -> np.ndarray:
    if message.encoding != "32FC1":
        raise ValueError(f"unsupported depth image encoding: {message.encoding}")
    dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
    if message.step % dtype.itemsize:
        raise ValueError("depth image step is not aligned to float32")
    rows = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, message.step // dtype.itemsize
    )
    return rows[:, : message.width].astype(np.float32, copy=True)


def depth_training_image(message: Image, depth_range_m) -> np.ndarray:
    """Convert metric depth to the three-channel uint8 format ACT expects."""
    near, far = depth_range_m
    depth = depth_image_array(message)
    depth = np.nan_to_num(depth, nan=far, posinf=far, neginf=near)
    gray = np.rint(np.clip((depth - near) / (far - near), 0, 1) * 255).astype(
        np.uint8
    )
    return np.repeat(gray[:, :, None], 3, axis=2)


class MujocoCameraPublisher:
    """Publish the RGB and/or metric depth streams declared by one task camera."""

    def __init__(self, node, spec: CameraSpec) -> None:
        self._node = node
        self.spec = spec
        self._renderer = None
        self._model = None
        self._camera_id = -1
        self._next_publish_time = 0.0
        self._rgb_publisher = None
        self._rgb_info_publisher = None
        self._depth_publisher = None
        self._depth_info_publisher = None

    def start(self, mujoco, model) -> None:
        self._model = model
        self._camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self.spec.mjcf_name
        )
        if self._camera_id < 0:
            raise ValueError(
                f"model does not contain task camera {self.spec.mjcf_name!r}"
            )

        model.vis.global_.offwidth = max(model.vis.global_.offwidth, self.spec.width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, self.spec.height)
        if self.spec.rgb is not None:
            self._rgb_publisher = self._node.create_publisher(
                Image, self.spec.rgb.topic, CAMERA_QOS
            )
            self._rgb_info_publisher = self._node.create_publisher(
                CameraInfo, self.spec.rgb.info_topic, CAMERA_QOS
            )
        if self.spec.depth is not None:
            self._depth_publisher = self._node.create_publisher(
                Image, self.spec.depth.topic, CAMERA_QOS
            )
            self._depth_info_publisher = self._node.create_publisher(
                CameraInfo, self.spec.depth.info_topic, CAMERA_QOS
            )

        self._next_publish_time = 0.0
        self._renderer = mujoco.Renderer(
            model, height=self.spec.height, width=self.spec.width
        )
        outputs = "+".join(
            name for name, stream in (("RGB", self.spec.rgb), ("depth", self.spec.depth))
            if stream is not None
        )
        self._node.get_logger().info(
            f"publishing {self.spec.camera_id} {outputs} at {self.spec.width}x"
            f"{self.spec.height} {self.spec.fps:g} Hz"
        )

    def publish_if_due(self, data, stamp_factory) -> None:
        if self._renderer is None or data.time + 1e-12 < self._next_publish_time:
            return

        self._renderer.update_scene(data, camera=self.spec.mjcf_name)
        color_pixels = self._renderer.render() if self.spec.rgb is not None else None
        depth_pixels = None
        if self.spec.depth is not None:
            self._renderer.enable_depth_rendering()
            try:
                depth_pixels = self._renderer.render()
            finally:
                self._renderer.disable_depth_rendering()

        sec, nanosec = stamp_factory(data.time)
        info = self._make_camera_info(sec, nanosec)
        if color_pixels is not None:
            self._rgb_publisher.publish(
                self._make_image(
                    color_pixels, "rgb8", self.spec.width * 3, sec, nanosec
                )
            )
            self._rgb_info_publisher.publish(info)
        if depth_pixels is not None:
            self._depth_publisher.publish(
                self._make_image(
                    np.asarray(depth_pixels, dtype=np.float32),
                    "32FC1", self.spec.width * 4, sec, nanosec,
                )
            )
            self._depth_info_publisher.publish(info)

        period = 1.0 / self.spec.fps
        while self._next_publish_time <= data.time + 1e-12:
            self._next_publish_time += period

    def reset_timing(self, sim_time: float) -> None:
        self._next_publish_time = float(sim_time)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _make_image(self, pixels, encoding, step, sec, nanosec):
        image = Image()
        image.header.stamp.sec = sec
        image.header.stamp.nanosec = nanosec
        image.header.frame_id = self.spec.frame_id
        image.height = self.spec.height
        image.width = self.spec.width
        image.encoding = encoding
        image.is_bigendian = False
        image.step = step
        image.data = pixels.tobytes()
        return image

    def _make_camera_info(self, sec, nanosec):
        fovy = radians(float(self._model.cam_fovy[self._camera_id]))
        focal = 0.5 * self.spec.height / tan(0.5 * fovy)
        cx = 0.5 * (self.spec.width - 1)
        cy = 0.5 * (self.spec.height - 1)
        info = CameraInfo()
        info.header.stamp.sec = sec
        info.header.stamp.nanosec = nanosec
        info.header.frame_id = self.spec.frame_id
        info.height = self.spec.height
        info.width = self.spec.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info
