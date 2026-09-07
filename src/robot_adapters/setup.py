from setuptools import setup
setup(name="robot_adapters", version="0.1.0", packages=["robot_adapters"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/robot_adapters"]),
                  ("share/robot_adapters", ["package.xml"])],
      install_requires=["setuptools", "numpy"], zip_safe=True)
