# ROS 2 Modular Robot Control System

A production-grade, modular ROS 2 system built for Ubuntu 22.04 with ROS 2 Humble. Implements a complete robot control pipeline: sensor driver → state machine → actuator controller, with lifecycle management, safety enforcement, unit tests, and full observability.

---

## Architecture Overview

```
ros2_robot_ws/
└── src/
    ├── robot_interfaces/          # Custom msgs & srvs (no circular deps)
    │   ├── msg/ActuatorCommand.msg
    │   ├── msg/ActuatorState.msg
    │   ├── msg/RobotState.msg
    │   ├── srv/SetActuatorEnabled.srv
    │   └── srv/TriggerStateTransition.srv
    │
    ├── sensor_driver_cpp/         # Lifecycle IMU sensor node
    │   ├── include/sensor_driver_cpp/imu_sensor_node.hpp
    │   ├── src/imu_sensor_node.cpp
    │   ├── src/main.cpp
    │   └── test/test_imu_sensor.cpp
    │
    ├── actuator_controller_cpp/   # Lifecycle actuator controller + safety layer
    │   ├── include/actuator_controller_cpp/
    │   │   ├── actuator_controller_node.hpp
    │   │   └── safety_monitor.hpp
    │   ├── src/actuator_controller_node.cpp
    │   ├── src/safety_monitor.cpp
    │   ├── src/main.cpp
    │   └── test/test_actuator_controller.cpp
    │
    ├── state_machine_cpp/         # Lifecycle FSM node (IDLE→ACTIVE→ERROR)
    │   ├── include/state_machine_cpp/
    │   │   ├── robot_fsm.hpp      ← pure C++, zero ROS deps, fully testable
    │   │   └── state_machine_node.hpp
    │   ├── src/robot_fsm.cpp
    │   ├── src/state_machine_node.cpp
    │   ├── src/main.cpp
    │   └── test/test_robot_fsm.cpp
    │
    └── robot_bringup/             # Launch files + YAML configs
        ├── launch/robot.launch.py
        ├── launch/lifecycle.launch.py
        ├── launch/test_fault_injection.launch.py
        └── config/
            ├── sensor_driver.yaml
            ├── actuator_controller.yaml
            └── state_machine.yaml
```

### Data Flow

```
[imu_sensor_node] ──/robot/imu_sensor_node/imu/data──────────────────┐
                  ──/robot/imu_sensor_node/imu/is_healthy──────────────► [state_machine_node]
                                                                        │        │
[actuator_controller_node] ◄──/robot/actuator/command──────────────────┘        │
                           ──/robot/actuator_controller_node/actuator/state──────►
                           ◄──service: actuator/set_enabled─────────────────────
                                                                                 │
                           [state_machine_node] ──/robot/robot_state────► (operator/GUI)
```

---

## Prerequisites

```bash
# Ubuntu 22.04 with ROS 2 Humble
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2lifecycle \
  ros-humble-lifecycle-msgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential

source /opt/ros/humble/setup.bash
```

---

## Build

```bash
cd ~/ros2_robot_ws

# Install all package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build everything
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the workspace
source install/setup.bash
```

### Build individual packages (faster iteration):

```bash
colcon build --packages-select robot_interfaces
colcon build --packages-select sensor_driver_cpp
colcon build --packages-select actuator_controller_cpp
colcon build --packages-select state_machine_cpp
colcon build --packages-select robot_bringup
```

---

## Running the System

### Option A: Full bringup (all-in-one)

```bash
# Terminal 1 — launch all nodes
source install/setup.bash
ros2 launch robot_bringup robot.launch.py

# Terminal 2 — run lifecycle transitions
source install/setup.bash
ros2 launch robot_bringup lifecycle.launch.py
```

### Option B: Manual node-by-node

```bash
# Terminal 1 — IMU sensor
ros2 run sensor_driver_cpp imu_sensor_node --ros-args \
  --params-file src/robot_bringup/config/sensor_driver.yaml

# Terminal 2 — Actuator controller
ros2 run actuator_controller_cpp actuator_controller_node --ros-args \
  --params-file src/robot_bringup/config/actuator_controller.yaml

# Terminal 3 — State machine
ros2 run state_machine_cpp state_machine_node --ros-args \
  --params-file src/robot_bringup/config/state_machine.yaml

# Terminal 4 — Lifecycle transitions (run each in sequence)
ros2 lifecycle set /robot/imu_sensor_node configure
ros2 lifecycle set /robot/actuator_controller_node configure
ros2 lifecycle set /robot/state_machine_node configure

ros2 lifecycle set /robot/imu_sensor_node activate
ros2 lifecycle set /robot/actuator_controller_node activate
ros2 lifecycle set /robot/state_machine_node activate
```

---

## Unit Tests

### Run all tests

```bash
colcon test --packages-select state_machine_cpp actuator_controller_cpp sensor_driver_cpp
colcon test-result --verbose
```

### Run a specific test binary directly (faster)

```bash
# State machine FSM tests (no ROS needed)
./build/state_machine_cpp/test_robot_fsm

# Safety monitor tests
./build/actuator_controller_cpp/test_actuator_controller

# IMU sensor lifecycle tests
./build/sensor_driver_cpp/test_imu_sensor
```

### Test coverage summary

| Test file | Tests | Coverage |
|---|---|---|
| `test_robot_fsm.cpp` | 18 cases | All FSM transitions, guards, fault counting, callback, timing |
| `test_actuator_controller.cpp` | 11 cases | All SafetyMonitor checks, runtime limit update |
| `test_imu_sensor.cpp` | 5 cases | Lifecycle configure/activate, message publishing |

---

## Observability & Debugging

### Watch topics in real time

```bash
# Robot state machine output
ros2 topic echo /robot/state_machine_node/robot_state

# IMU data stream
ros2 topic echo /robot/imu_sensor_node/imu/data

# Actuator state
ros2 topic echo /robot/actuator_controller_node/actuator/state

# IMU health
ros2 topic echo /robot/imu_sensor_node/imu/is_healthy
```

### Monitor topic rates

```bash
ros2 topic hz /robot/imu_sensor_node/imu/data        # expect ~100 Hz
ros2 topic hz /robot/actuator_controller_node/actuator/state  # expect ~50 Hz
ros2 topic hz /robot/state_machine_node/robot_state  # expect ~10 Hz
```

### Node graph

```bash
ros2 node list
ros2 node info /robot/state_machine_node
ros2 topic list
ros2 service list
```

### Lifecycle state inspection

```bash
ros2 lifecycle list /robot/imu_sensor_node
ros2 lifecycle get /robot/state_machine_node
```

### Structured logging — change log level at runtime

```bash
ros2 param set /robot/imu_sensor_node log_level debug

# Or via command line at launch:
ros2 run sensor_driver_cpp imu_sensor_node --ros-args --log-level debug
```

### ROS 2 logging with mcap (bag recording)

```bash
# Record all robot topics
ros2 bag record -a -o robot_session_$(date +%Y%m%d_%H%M%S)

# Replay and inspect
ros2 bag play robot_session_*/
ros2 bag info robot_session_*/
```

---

## Runtime Operations

### Enable/Disable Actuator via Service

```bash
# Enable
ros2 service call /robot/actuator_controller_node/actuator/set_enabled \
  robot_interfaces/srv/SetActuatorEnabled \
  '{enable: true, reason: "operator enable"}'

# Disable
ros2 service call /robot/actuator_controller_node/actuator/set_enabled \
  robot_interfaces/srv/SetActuatorEnabled \
  '{enable: false, reason: "operator disable"}'
```

### Manual State Machine Override

```bash
# Force to ACTIVE (requires healthy subsystems)
ros2 service call /robot/state_machine_node/trigger_transition \
  robot_interfaces/srv/TriggerStateTransition \
  '{target_state: 1, reason: "operator override"}'

# Emergency stop
ros2 service call /robot/state_machine_node/trigger_transition \
  robot_interfaces/srv/TriggerStateTransition \
  '{target_state: 2, reason: "manual e-stop"}'
```

### Send a Command to the Actuator

```bash
ros2 topic pub --once /robot/actuator_controller_node/actuator/command \
  robot_interfaces/msg/ActuatorCommand \
  '{stamp: {sec: 0, nanosec: 0},
    position_targets: [0.0, 0.5, -0.5, 0.0, 0.5, 0.0],
    velocity_targets: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    torque_limits: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
    gripper_position: 0.5,
    speed_fraction: 0.3,
    source_node: "terminal"}'
```

---

## Fault Injection Testing

```bash
# Launch with fault injection
ros2 launch robot_bringup test_fault_injection.launch.py

# OR toggle fault injection at runtime via parameter
ros2 param set /robot/imu_sensor_node inject_fault true

# Watch the state machine respond
ros2 topic echo /robot/state_machine_node/robot_state \
  --field state_name --field transition_reason --field total_fault_count
```

Expected sequence:
1. `imu_sensor_node` publishes `is_healthy: false`
2. `state_machine_node` detects fault → transitions `ACTIVE → ERROR`
3. After `recovery_wait_sec` (5 s), auto-recovery triggers `ERROR → RECOVERING → IDLE`
4. Disable fault injection: `ros2 param set /robot/imu_sensor_node inject_fault false`
5. State machine returns to IDLE, can be re-activated

---

## Gazebo / RViz Integration

### RViz2 — visualize IMU data

```bash
rviz2 &

# In RViz: Add → By topic → /robot/imu_sensor_node/imu/data → Imu
# Set Fixed Frame to: imu_link
```

### Gazebo — replace mock IMU with real plugin

In your URDF/SDF, replace the `imu_sensor_node` executable with:

```xml
<plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
  <ros>
    <namespace>robot</namespace>
    <remapping>~/out:=imu_sensor_node/imu/data</remapping>
  </ros>
  <initial_orientation_as_reference>false</initial_orientation_as_reference>
  <frame_name>imu_link</frame_name>
  <update_rate>100</update_rate>
</plugin>
```

The `state_machine_node` and `actuator_controller_node` require no changes — they subscribe to the same topic names.

### rqt_graph — full node/topic graph

```bash
rqt_graph
```

### plotjuggler — time series visualization

```bash
sudo apt install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler
# Connect to ROS2 and select topics to plot
```

---

## Common Issues & Solutions

| Problem | Solution |
|---|---|
| `colcon build` fails with missing deps | Run `rosdep install --from-paths src --ignore-src -r -y` |
| Lifecycle node not found | Check `ros2 node list` — node may not have started yet |
| Topics not appearing | Verify lifecycle nodes are in ACTIVE state |
| Safety violation immediately | Check actuator param `command_timeout_ms` — stamp must be recent |
| State machine stuck in ERROR | Disable fault injection, or trigger manual recovery via service |

---

## C++17 & ROS 2 Best Practices Applied

- **Lifecycle nodes** — all three nodes use `rclcpp_lifecycle::LifecycleNode` for safe startup/shutdown sequencing
- **Separated testable logic** — `RobotFsm` and `SafetyMonitor` are pure C++ classes with zero ROS deps, enabling fast unit tests
- **Atomic health flags** — `std::atomic<bool>` for lock-free cross-thread health reads
- **Target-based CMake** — all linking via `target_link_libraries` / `ament_target_dependencies`
- **QoS configuration** — `reliable().transient_local()` on state topics so late joiners get last state immediately
- **Named callback groups** — separate callback groups for command subscription vs. timer to avoid head-of-line blocking
- **Intra-process comms** — `use_intra_process_comms(true)` for zero-copy within same process
- **Parameter descriptors** — typed ranges on all parameters for `ros2 param describe` support
- **Structured logging** — `RCLCPP_*_THROTTLE` to avoid log spam, `RCLCPP_DEBUG` for high-frequency data
