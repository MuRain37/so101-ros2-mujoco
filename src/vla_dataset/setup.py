from setuptools import setup

package_name = "vla_dataset"

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
            "dataset_recorder = vla_dataset.recorder:main",
            "dataset_gui = vla_dataset.dataset_gui:main",
            "convert_mcap_to_lerobot = vla_dataset.convert_mcap_to_lerobot:main",
        ],
    },
)
