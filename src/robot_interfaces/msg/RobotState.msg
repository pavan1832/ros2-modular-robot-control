# RobotState.msg
# Published by the state machine node.

builtin_interfaces/Time stamp

# Current FSM state
uint8 state
uint8 STATE_IDLE=0
uint8 STATE_ACTIVE=1
uint8 STATE_ERROR=2
uint8 STATE_RECOVERING=3
uint8 STATE_SHUTDOWN=4

# Human-readable state name
string state_name

# Reason for last state transition
string transition_reason

# Duration in current state (seconds)
float64 time_in_state_sec

# Subsystem health flags
bool sensor_healthy
bool actuator_healthy
bool planner_healthy

# Number of faults encountered since node start
uint32 total_fault_count
