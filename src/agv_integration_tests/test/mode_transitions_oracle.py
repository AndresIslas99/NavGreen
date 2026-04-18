"""Oracle for mode_arbiter transition validation.

Parses the JSON stream from /agv/mode/state and exposes two helpers:
- `record_transition(mode_json_str)`: feed each message in; deduplicates
  consecutive repeats, tracks unique transitions.
- `assert_contains_subsequence(expected_modes, observed_modes)`: checks
  whether `expected_modes` appears as an ordered (non-contiguous)
  subsequence of `observed_modes`. Returns (ok, missing, extras).

Kept ROS-free so it can be unit-tested without a spinning node and
reused by test_waypoint_precision.py.
"""
from __future__ import annotations

import json
from typing import List, Optional, Tuple


class ModeTransitionRecorder:
    """Accumulates unique mode transitions between reset points.

    The recorder is stateful: `begin_waypoint()` clears the transition list
    and starts fresh. `record_transition()` feeds incoming JSON strings
    from /agv/mode/state. `modes_seen()` returns the ordered list of unique
    modes observed since the last begin_waypoint call.
    """

    def __init__(self) -> None:
        self._modes: List[str] = []

    def begin_waypoint(self) -> None:
        self._modes = []

    def record_transition(self, mode_json_str: str) -> Optional[str]:
        """Parse the JSON payload and record the mode if it changed.

        Returns the new mode label if a transition occurred, else None.
        """
        try:
            data = json.loads(mode_json_str)
        except (json.JSONDecodeError, TypeError):
            return None
        if not isinstance(data, dict):
            return None
        mode = data.get("mode")
        if not isinstance(mode, str):
            return None
        if self._modes and self._modes[-1] == mode:
            return None  # same as last; not a transition
        self._modes.append(mode)
        return mode

    def modes_seen(self) -> List[str]:
        return list(self._modes)


def is_subsequence(expected: List[str], observed: List[str]) -> bool:
    """True if `expected` appears as an ordered subsequence of `observed`.

    Elements of expected may be separated by any number of observed items,
    but must appear in the same order.
    """
    it = iter(observed)
    return all(e in it for e in expected)


def diff_subsequence(expected: List[str], observed: List[str]
                      ) -> Tuple[bool, List[str], List[str]]:
    """Return (ok, missing, extras).

    - missing: elements of expected that were never matched.
    - extras: elements of observed beyond what expected consumed. Useful for
      diagnostics when safety_stop or blocked_handoff intervenes.
    """
    observed_copy = list(observed)
    missing = []
    consumed_indices: List[int] = []
    cursor = 0
    for e in expected:
        try:
            idx = observed_copy.index(e, cursor)
        except ValueError:
            missing.append(e)
            continue
        consumed_indices.append(idx)
        cursor = idx + 1
    extras = [m for i, m in enumerate(observed_copy) if i not in consumed_indices]
    ok = len(missing) == 0
    return ok, missing, extras
