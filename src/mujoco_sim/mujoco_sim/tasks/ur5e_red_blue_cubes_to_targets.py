"""UR5e version of the red/blue cubes task."""

from .base import CameraSpec, ImageStreamSpec
from .red_blue_cubes_to_targets import RedBlueCubesToTargetsTask


class UR5eRedBlueCubesToTargetsTask(RedBlueCubesToTargetsTask):
    task_id = "ur5e_red_blue_cubes_to_targets"
    robot_id = "ur5e"
    scene_file = "tasks/ur5e_red_blue_cubes_to_targets.xml"
    reset_source_robot_id = "so101"
    cameras = (
        CameraSpec(
            camera_id="front",
            mjcf_name="global_d435_camera",
            frame_id="global_d435_optical_frame",
            rgb=ImageStreamSpec(
                topic="/d435/color/image_raw",
                info_topic="/d435/color/camera_info",
                observation_key="observation.images.front",
                preview_topic="/d435/color/image_preview/compressed",
            ),
        ),
        CameraSpec(
            camera_id="wrist",
            mjcf_name="wrist_cam",
            frame_id="wrist_cam_optical_frame",
            rgb=ImageStreamSpec(
                topic="/wrist_cam/image_raw",
                info_topic="/wrist_cam/camera_info",
                observation_key="observation.images.wrist",
                preview_topic="/wrist_cam/image_preview/compressed",
            ),
        ),
    )
    primary_camera_id = "front"
