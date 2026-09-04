from setuptools import setup

package_name = "so101_policy"

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
            "so101_act_policy = so101_policy.act_node:main",
            "so101_policy_gui = so101_policy.policy_gui:main",
        ],
    },
)
