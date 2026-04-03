"""
State machine — derives robot state from sensor readings.
Extracted from teleop_server.py for modularity.
"""


def derive_state(e_stop, motor_state, odom_hz, mode, mission_progress, nav_state):
    """Derive the overall robot state from component states."""
    if e_stop:
        return 'e_stop'
    errors = motor_state.get('left_errors', 0) or motor_state.get('right_errors', 0)
    if errors:
        return 'fault'
    armed = motor_state.get('armed', False)
    if odom_hz < 1.0 and not armed:
        return 'idle'
    if mode == 'mapping':
        return 'mapping'
    if mission_progress and mission_progress.get('status') == 'running':
        return 'executing_mission'
    if nav_state and nav_state.get('active', False):
        return 'navigating'
    if armed:
        return 'ready'
    return 'idle'


def allowed_actions(state):
    """Return dict of allowed actions for the current state."""
    return {
        'canTeleop': state in ('ready', 'idle'),
        'canStartMapping': state in ('ready', 'idle'),
        'canStopMapping': state == 'mapping',
        'canSendGoal': state in ('ready', 'navigating'),
        'canExecuteMission': state == 'ready',
        'canSaveMap': state in ('mapping', 'ready', 'idle'),
        'canLoadMap': state in ('ready', 'idle'),
        'canMotorEnable': state not in ('e_stop', 'fault'),
        'canPauseMission': state == 'executing_mission',
        'canCancelNav': state in ('navigating', 'executing_mission'),
    }
