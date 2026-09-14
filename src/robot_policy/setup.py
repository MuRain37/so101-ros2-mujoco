from setuptools import setup

package_name = "robot_policy"

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
    entry_points={
        "console_scripts": [
            "policy_inference = robot_policy.policy_node:main",
            "robot_policy_gui = robot_policy.policy_gui:main",
        ],
    },
)
