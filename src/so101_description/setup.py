import os
from glob import glob

from setuptools import setup

package_name = "so101_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="murain",
    maintainer_email="murain@example.com",
    description="URDF description and RViz display for the SO-101 leader arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_state_demo = so101_description.joint_state_demo:main",
        ],
    },
)
