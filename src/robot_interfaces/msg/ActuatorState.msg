# ActuatorState.msg
# Published by the actuator controller reflecting current hardware state.

builtin_interfaces/Time stamp

# Current positions (radians)
float64[] current_positions

# Current velocities (rad/s)
float64[] current_velocities

# Current torques (N·m)
float64[] current_torques

# Gripper position [0.0 = open, 1.0 = closed]
float64 gripper_position

# True if actuator is enabled and accepting commands
bool is_enabled

# True if all safety checks pass
bool is_safe

# Temperature of motor driver (°C) — triggers thermal limit if exceeded
float64 driver_temperature_celsius

# Fault code: 0 = nominal, nonzero = fault category
uint8 fault_code
uint8 FAULT_NONE=0
uint8 FAULT_OVER_TEMPERATURE=1
uint8 FAULT_OVER_CURRENT=2
uint8 FAULT_POSITION_LIMIT=3
uint8 FAULT_COMMUNICATION=4
uint8 FAULT_EMERGENCY_STOP=5
