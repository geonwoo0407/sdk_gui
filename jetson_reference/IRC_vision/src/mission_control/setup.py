from setuptools import find_packages
from setuptools import setup
from glob import glob
import os


package_name = "mission_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="geonwoo",
    maintainer_email="geonwoo0407@gmail.com",
    description="Mission decision and motion coordination for the STEP robot.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_decision_node="
            "mission_control.motion_decision_node:main",
            "motion_command_bridge_node="
            "mission_control.motion_command_bridge_node:main",
            "sdk_motion_stub_node="
            "mission_control.sdk_motion_stub_node:main",
            "motion_executor_node="
            "mission_control.motion_executor_node:main",
            "legacy_motion_executor_adapter="
            "mission_control.legacy_motion_executor_adapter:main",
            "legacy_motion_status_adapter="
            "mission_control.legacy_motion_status_adapter:main",
            "mock_mission_input_node="
            "mission_control.mock_mission_input_node:main",
        ],
    },
)
