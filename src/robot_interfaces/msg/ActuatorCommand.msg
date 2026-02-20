# ActuatorCommand.msg
# Command sent to the actuator controller from higher-level nodes.

builtin_interfaces/Time stamp

# Target joint positions in radians (size matches DOF of actuator)
float64[] position_targets

# Target velocities in rad/s (feed-forward)
float64[] velocity_targets

# Target torques in N·m (feed-forward)
float64[] torque_limits

# Normalized gripper position [0.0 = fully open, 1.0 = fully closed]
float64 gripper_position

# Maximum allowable speed fraction [0.0, 1.0] — safety override
float64 speed_fraction

# Command source identifier for audit trail
string source_node
