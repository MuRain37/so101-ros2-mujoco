from setuptools import setup


package_name = "so101_leader_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="murain",
    maintainer_email="murain@example.com",
    description="Serial-to-ROS bridge for the SO-101 leader arm",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "so101_leader_driver = so101_leader_bridge.so101_leader_driver:main",
        ],
    },
)
