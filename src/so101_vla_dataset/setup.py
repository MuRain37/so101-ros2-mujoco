from setuptools import setup

package_name = "so101_vla_dataset"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "so101_dataset_recorder = so101_vla_dataset.recorder:main",
            "so101_dataset_gui = so101_vla_dataset.dataset_gui:main",
            "convert_mcap_to_lerobot = so101_vla_dataset.convert_mcap_to_lerobot:main",
        ],
    },
)
