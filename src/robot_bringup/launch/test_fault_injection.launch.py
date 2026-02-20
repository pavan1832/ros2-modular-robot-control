"""
robot_bringup/launch/test_fault_injection.launch.py

Launch the full system with IMU fault injection enabled to exercise the
state machine ERROR → RECOVERING → IDLE flow.

Usage:
  ros2 launch robot_bringup test_fault_injection.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "robot.launch.py")
        ),
        launch_arguments={
            "inject_fault": "true",
            "log_level": "debug",
            "auto_lifecycle": "false",
        }.items(),
    )

    lifecycle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "lifecycle.launch.py")
        ),
    )

    return LaunchDescription([
        LogInfo(msg="=== FAULT INJECTION TEST MODE ==="),
        LogInfo(msg="IMU will inject faults after 5 consecutive publishes"),
        robot_launch,
        lifecycle_launch,
    ])
