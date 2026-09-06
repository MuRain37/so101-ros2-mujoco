"""Resolve installed, namespaced model resources."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import mujoco


def description_share():
    return Path(get_package_share_directory("robot_description"))


def legacy_assets():
    # Legacy SO101/D435 meshes have unique basenames; new robot assets retain folders.
    return {p.name: p.read_bytes() for p in (description_share() / "meshes").glob("*.stl")}


def load_robot_model(robot):
    return mujoco.MjModel.from_xml_path(
        str(description_share() / "mjcf" / robot.model_file),
        assets=legacy_assets(),
    )


def load_scene(path, assets=None):
    """Resolve cross-package XML references also for non-symlink ROS installs."""
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    for tag in ("include", "model"):
        for element in root.iter(tag):
            reference = element.get("file")
            if not reference:
                continue
            marker = "robot_description/"
            if marker in reference:
                target = description_share() / reference.split(marker, 1)[1]
            else:
                target = path.parent / reference
            element.set("file", str(target.resolve()))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"),
                                        assets=legacy_assets() if assets is None else assets)
