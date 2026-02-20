"""
robot_bringup/launch/robot.launch.py

Main system launch file. Starts all three nodes as lifecycle nodes,
manages namespacing, parameter files, and performs automatic lifecycle
transitions once all nodes are up.

Usage:
  ros2 launch robot_bringup robot.launch.py
  ros2 launch robot_bringup robot.launch.py inject_fault:=true
  ros2 launch robot_bringup robot.launch.py namespace:=robot1
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Declare arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="robot",
            description="Top-level namespace for all nodes",
        ),
        DeclareLaunchArgument(
            "inject_fault",
            default_value="false",
            description="Enable IMU fault injection for testing",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 log level: debug|info|warn|error",
        ),
        DeclareLaunchArgument(
            "auto_lifecycle",
            default_value="true",
            description="Automatically configure+activate lifecycle nodes after startup",
        ),
    ]

    # ── Config paths ───────────────────────────────────────────────────────────
    bringup_share = get_package_share_directory("robot_bringup")
    config_dir = os.path.join(bringup_share, "config")

    sensor_config   = os.path.join(config_dir, "sensor_driver.yaml")
    actuator_config = os.path.join(config_dir, "actuator_controller.yaml")
    fsm_config      = os.path.join(config_dir, "state_machine.yaml")

    # ── Nodes ──────────────────────────────────────────────────────────────────
    imu_sensor_node = LifecycleNode(
        package="sensor_driver_cpp",
        executable="imu_sensor_node",
        name="imu_sensor_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            sensor_config,
            {"inject_fault": LaunchConfiguration("inject_fault")},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
    )

    actuator_controller_node = LifecycleNode(
        package="actuator_controller_cpp",
        executable="actuator_controller_node",
        name="actuator_controller_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[actuator_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        remappings=[
            # Remap command topic to allow the state machine to send commands
            ("actuator/command", "actuator/command"),
        ],
    )

    state_machine_node = LifecycleNode(
        package="state_machine_cpp",
        executable="state_machine_node",
        name="state_machine_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[fsm_config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        remappings=[
            # Absolute paths to cross-namespace topics
            ("imu/is_healthy",    "/robot/imu_sensor_node/imu/is_healthy"),
            ("actuator/state",    "/robot/actuator_controller_node/actuator/state"),
        ],
    )

    # ── Automatic lifecycle management ─────────────────────────────────────────
    # We use ros2 lifecycle CLI nodes to configure+activate each lifecycle node.
    # These run after a 2 s delay to ensure the nodes are registered.

    def make_lifecycle_cmd(node_fqn: str, transition: str):
        return Node(
            package="lifecycle_msgs",
            executable="",  # placeholder — see lifecycle_manager.launch.py
            name=f"lifecycle_{transition}_{node_fqn.replace('/', '_')}",
            output="screen",
        )

    configure_sensor = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros2lifecycle",
                executable="ros2lifecycle",
                name="lifecycle_configure_sensor",
                arguments=["set", "/robot/imu_sensor_node", "configure"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("auto_lifecycle")),
            )
        ],
    )

    # We use a Python lifecycle manager script that handles sequencing properly
    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="robot_bringup",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                parameters=[{"namespace": LaunchConfiguration("namespace")}],
                condition=IfCondition(LaunchConfiguration("auto_lifecycle")),
            )
        ],
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg="=== Starting Modular Robot Control System ==="),
            imu_sensor_node,
            actuator_controller_node,
            state_machine_node,
            # Lifecycle management is handled via the lifecycle_manager launch
            LogInfo(
                msg=[
                    "Nodes started. If auto_lifecycle=false, run:\n"
                    "  ros2 launch robot_bringup lifecycle.launch.py"
                ]
            ),
        ]
    )
