"""MuJoCo camera rendering and ROS 2 image publication."""

from math import radians, tan

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MujocoCameraPublisher:
    """Publish a rate-limited RGB or aligned RGB-D stream from one MuJoCo camera."""

    def __init__(
        self,
        node,
        *,
        label,
        parameter_prefix,
        camera_name,
        frame_id,
        color_topic_parameter,
        color_topic,
        color_info_topic_parameter,
        color_info_topic,
        depth_enabled=False,
        depth_topic="",
        depth_info_topic="",
    ) -> None:
        self._node = node
        self._label = label
        self._prefix = parameter_prefix
        self._depth_enabled = depth_enabled
        node.declare_parameter(f"{parameter_prefix}_enabled", True)
        node.declare_parameter(f"{parameter_prefix}_name", camera_name)
        node.declare_parameter(f"{parameter_prefix}_frame_id", frame_id)
        node.declare_parameter(color_topic_parameter, color_topic)
        node.declare_parameter(color_info_topic_parameter, color_info_topic)
        node.declare_parameter(f"{parameter_prefix}_width", 640)
        node.declare_parameter(f"{parameter_prefix}_height", 480)
        node.declare_parameter(f"{parameter_prefix}_publish_rate", 30.0)
        if depth_enabled:
            node.declare_parameter(f"{parameter_prefix}_depth_topic", depth_topic)
            node.declare_parameter(
                f"{parameter_prefix}_depth_info_topic", depth_info_topic
            )

        self._color_topic_parameter = color_topic_parameter
        self._color_info_topic_parameter = color_info_topic_parameter
        self._renderer = None
        self._model = None
        self._camera_id = -1
        self._camera_name = ""
        self._frame_id = ""
        self._width = 0
        self._height = 0
        self._publish_period = 0.0
        self._next_publish_time = 0.0
        self._color_publisher = None
        self._color_info_publisher = None
        self._depth_publisher = None
        self._depth_info_publisher = None

    def start(self, mujoco, model) -> None:
        """Validate configuration and create publishers and an offscreen renderer."""
        if not self._node.get_parameter(f"{self._prefix}_enabled").value:
            self._node.get_logger().info(f"{self._label} rendering is disabled")
            return

        self._model = model
        self._camera_name = self._string_parameter(f"{self._prefix}_name")
        self._frame_id = self._string_parameter(f"{self._prefix}_frame_id")
        self._width = int(
            self._node.get_parameter(f"{self._prefix}_width").value
        )
        self._height = int(
            self._node.get_parameter(f"{self._prefix}_height").value
        )
        publish_rate = float(
            self._node.get_parameter(f"{self._prefix}_publish_rate").value
        )
        if self._width < 1 or self._height < 1:
            raise ValueError(
                f"{self._prefix}_width and {self._prefix}_height must be positive"
            )
        if publish_rate <= 0:
            raise ValueError(f"{self._prefix}_publish_rate must be greater than zero")

        self._camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        if self._camera_id < 0:
            raise ValueError(
                f"model does not contain {self._label} camera {self._camera_name!r}"
            )

        model.vis.global_.offwidth = max(model.vis.global_.offwidth, self._width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, self._height)
        self._color_publisher = self._node.create_publisher(
            Image,
            self._node.get_parameter(self._color_topic_parameter).value,
            CAMERA_QOS,
        )
        self._color_info_publisher = self._node.create_publisher(
            CameraInfo,
            self._node.get_parameter(self._color_info_topic_parameter).value,
            CAMERA_QOS,
        )
        if self._depth_enabled:
            self._depth_publisher = self._node.create_publisher(
                Image,
                self._node.get_parameter(f"{self._prefix}_depth_topic").value,
                CAMERA_QOS,
            )
            self._depth_info_publisher = self._node.create_publisher(
                CameraInfo,
                self._node.get_parameter(f"{self._prefix}_depth_info_topic").value,
                CAMERA_QOS,
            )

        self._publish_period = 1.0 / publish_rate
        self._next_publish_time = 0.0
        self._renderer = mujoco.Renderer(
            model, height=self._height, width=self._width
        )
        stream_kind = "aligned RGB-D" if self._depth_enabled else "RGB"
        self._node.get_logger().info(
            f"publishing {self._label} {stream_kind} at {self._width}x"
            f"{self._height} {publish_rate:g} Hz"
        )

    def publish_if_due(self, data, stamp_factory) -> None:
        """Render and publish a frame when its simulation-time deadline is due."""
        if self._renderer is None or data.time + 1e-12 < self._next_publish_time:
            return

        self._renderer.update_scene(data, camera=self._camera_name)
        color_pixels = self._renderer.render()
        depth_pixels = None
        if self._depth_enabled:
            self._renderer.enable_depth_rendering()
            try:
                depth_pixels = self._renderer.render()
            finally:
                self._renderer.disable_depth_rendering()

        sec, nanosec = stamp_factory(data.time)
        color = self._make_image(
            color_pixels, "rgb8", self._width * 3, sec, nanosec
        )
        self._color_publisher.publish(color)

        if depth_pixels is not None:
            depth = self._make_image(
                depth_pixels, "32FC1", self._width * 4, sec, nanosec
            )
            self._depth_publisher.publish(depth)

        info = self._make_camera_info(sec, nanosec)
        self._color_info_publisher.publish(info)
        if self._depth_info_publisher is not None:
            self._depth_info_publisher.publish(info)

        while self._next_publish_time <= data.time + 1e-12:
            self._next_publish_time += self._publish_period

    def reset_timing(self, sim_time: float) -> None:
        """Publish immediately after the simulation clock moves backwards."""
        self._next_publish_time = float(sim_time)

    def close(self) -> None:
        """Release the renderer's OpenGL resources."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _string_parameter(self, name):
        value = self._node.get_parameter(name).value.strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _make_image(self, pixels, encoding, step, sec, nanosec):
        image = Image()
        image.header.stamp.sec = sec
        image.header.stamp.nanosec = nanosec
        image.header.frame_id = self._frame_id
        image.height = self._height
        image.width = self._width
        image.encoding = encoding
        image.is_bigendian = False
        image.step = step
        image.data = pixels.tobytes()
        return image

    def _make_camera_info(self, sec, nanosec):
        fovy = radians(float(self._model.cam_fovy[self._camera_id]))
        focal = 0.5 * self._height / tan(0.5 * fovy)
        cx = 0.5 * (self._width - 1)
        cy = 0.5 * (self._height - 1)
        info = CameraInfo()
        info.header.stamp.sec = sec
        info.header.stamp.nanosec = nanosec
        info.header.frame_id = self._frame_id
        info.height = self._height
        info.width = self._width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def create_wrist_camera(node):
    """Create the wrist RGB camera with its existing public parameters."""
    return MujocoCameraPublisher(
        node,
        label="wrist camera",
        parameter_prefix="camera",
        camera_name="wrist_cam",
        frame_id="wrist_cam_optical_frame",
        color_topic_parameter="camera_image_topic",
        color_topic="wrist_cam/image_raw",
        color_info_topic_parameter="camera_info_topic",
        color_info_topic="wrist_cam/camera_info",
    )


def create_global_d435(node):
    """Create the fixed D435 with aligned color and metric-depth streams."""
    return MujocoCameraPublisher(
        node,
        label="global D435",
        parameter_prefix="d435",
        camera_name="global_d435_camera",
        frame_id="global_d435_optical_frame",
        color_topic_parameter="d435_color_topic",
        color_topic="d435/color/image_raw",
        color_info_topic_parameter="d435_color_info_topic",
        color_info_topic="d435/color/camera_info",
        depth_enabled=True,
        depth_topic="d435/depth/image_raw",
        depth_info_topic="d435/depth/camera_info",
    )
