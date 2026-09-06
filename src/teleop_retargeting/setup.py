from setuptools import setup
setup(name="teleop_retargeting", version="0.1.0", packages=["teleop_retargeting"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/teleop_retargeting"]),
                  ("share/teleop_retargeting", ["package.xml"])],
      install_requires=["setuptools", "numpy", "scipy"],
      entry_points={"console_scripts": ["retarget = teleop_retargeting.node:main"]})
