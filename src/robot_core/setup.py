from setuptools import setup
setup(name="robot_core", version="0.1.0", packages=["robot_core"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/robot_core"]),
                  ("share/robot_core", ["package.xml"])],
      install_requires=["setuptools", "numpy"], zip_safe=True)
