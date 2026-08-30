from glob import glob
import os

from setuptools import setup


package_name = "so101_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="murain",
    maintainer_email="murain@example.com",
    description="Launch files for the SO-101 system",
    license="Apache-2.0",
)
