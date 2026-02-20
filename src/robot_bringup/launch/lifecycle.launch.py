"""
robot_bringup/launch/lifecycle.launch.py

Handles the configure → activate lifecycle sequence for all lifecycle nodes.
Run this after robot.launch.py if auto_lifecycle is disabled, or use it
standalone to bring nodes through their lifecycle manually.

Usage:
  ros2 launch robot_bringup lifecycle.launch.py
  ros2 launch robot_bringup lifecycle.launch.py namespace:=robot1
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace", default_value="robot",
        description="Namespace of the running nodes"
    )

    ns = LaunchConfiguration("namespace")

    # Node FQNs (fully qualified names)
    # These must match the names declared in robot.launch.py

    def lifecycle_set(node_name: str, transition: str, delay: float):
        """Create a timed lifecycle transition command."""
        fqn = f"/{LaunchConfiguration('namespace').variable_name}/{node_name}"
        return TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f"Lifecycle: {transition} → {node_name}"),
                ExecuteProcess(
                    cmd=[
                        "ros2", "lifecycle", "set",
                        f"/robot/{node_name}",
                        transition,
                    ],
                    output="screen",
                    shell=False,
                ),
            ],
        )

    return LaunchDescription([
        ns_arg,
        LogInfo(msg="=== Lifecycle Manager: configuring nodes ==="),

        # Configure all nodes first (t=0s)
        lifecycle_set("imu_sensor_node",          "configure",  0.5),
        lifecycle_set("actuator_controller_node", "configure",  1.0),
        lifecycle_set("state_machine_node",       "configure",  1.5),

        LogInfo(msg="=== Lifecycle Manager: activating nodes ==="),

        # Activate sensor first, then actuator, then state machine
        lifecycle_set("imu_sensor_node",          "activate",   3.0),
        lifecycle_set("actuator_controller_node", "activate",   3.5),
        lifecycle_set("state_machine_node",       "activate",   4.0),

        LogInfo(msg="=== All nodes activated — system running ==="),
    ])
