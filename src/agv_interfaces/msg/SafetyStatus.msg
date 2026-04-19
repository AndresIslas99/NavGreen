# Aggregated safety state of the AGV runtime stack.
# Published by agv_safety/safety_supervisor at a fixed rate (default 10 Hz).
#
# Consumers (cmd_vel_gate, dashboard, behavior_executor) MUST treat
# safety_ok=false as a hard stop signal — no further validation needed.

std_msgs/Header header

# Top-level safety verdict. False if any monitored topic missed its deadline,
# any alert is active, or software E-stop is latched.
bool safety_ok

# Software emergency stop currently latched (separate from any hardware E-stop
# circuit).
bool software_estop

# Topics that are currently considered silent (last receipt older than the
# topic's configured deadline). Empty when safety_ok is true.
string[] silent_topics

# Active alert strings for operator display. Each entry is a short
# human-readable reason. Empty when safety_ok is true.
string[] alerts

# One-line reason for the current state (operator-friendly). Empty when ok.
string reason
