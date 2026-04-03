"""
Persistent event log — stores events to JSONL file with in-memory ring buffer.
Extracted from teleop_server.py for modularity.
"""

import json
import os
import time
from collections import deque

MAX_EVENTS = 500


class EventLog:
    def __init__(self, data_dir):
        self.file_path = os.path.join(data_dir, 'events.jsonl')
        self.entries = deque(maxlen=MAX_EVENTS)
        self.pending = []  # events queued for WS broadcast
        self._load()

    def _load(self):
        """Load existing events from disk."""
        if not os.path.exists(self.file_path):
            return
        try:
            with open(self.file_path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        self.entries.append(json.loads(line))
        except Exception:
            pass

    def emit(self, severity, subsystem, text):
        """Add an event and persist to disk."""
        entry = {
            'timestamp': time.time(),
            'severity': severity,
            'subsystem': subsystem,
            'text': text,
        }
        self.entries.append(entry)
        self.pending.append(entry)
        try:
            with open(self.file_path, 'a') as f:
                f.write(json.dumps(entry) + '\n')
        except Exception:
            pass
        return entry

    def get_entries(self, limit=100, offset=0):
        """Return recent entries (newest first)."""
        entries = list(self.entries)
        entries.reverse()
        return entries[offset:offset + limit]

    def pop_pending(self):
        """Return and clear pending events for WS broadcast."""
        events = self.pending[:]
        self.pending.clear()
        return events

    def clear(self):
        """Clear all events."""
        self.entries.clear()
        self.pending.clear()
        try:
            open(self.file_path, 'w').close()
        except Exception:
            pass
