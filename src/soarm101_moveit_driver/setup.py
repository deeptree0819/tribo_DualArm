from glob import glob

from setuptools import find_packages, setup

package_name = "soarm101_moveit_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="deeptree",
    maintainer_email="deeptree00@gmail.com",
    description="MoveIt2 direct-drive bridge for the real SO-ARM101 (Feetech/LeRobot).",
    license="BSD-3-Clause",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "moveit_motor_bridge = soarm101_moveit_driver.moveit_motor_bridge:main",
        ],
    },
)
