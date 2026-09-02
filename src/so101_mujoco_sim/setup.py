from glob import glob
import os

from setuptools import find_packages, setup


package_name = "so101_mujoco_sim"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            [
                "package.xml",
                "README.md",
            ],
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "mujoco"), glob("mujoco/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="murain",
    maintainer_email="murain@example.com",
    description="MuJoCo simulator package for the SO-101 follower arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "so101_mujoco_simulator = so101_mujoco_sim.simulator:main",
            "so101_mujoco_viewer = so101_mujoco_sim.simulator:main",
        ],
    },
)
