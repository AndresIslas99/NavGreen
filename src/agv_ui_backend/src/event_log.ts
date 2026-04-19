// Persistent event log — appends to JSONL file, keeps last N in memory

import * as fs from 'fs';
import * as path from 'path';

export interface LogEntry {
  timestamp: number;
  severity: 'info' | 'warn' | 'crit';
  subsystem: string;
  text: string;
}

const MAX_EVENTS = 500;

export type EventListener = (entry: LogEntry) => void;

export class EventLog {
  private entries: LogEntry[] = [];
  private filePath: string;
  private pending: LogEntry[] = []; // events to push to WS clients
  private listeners: EventListener[] = [];

  constructor(dataDir: string) {
    this.filePath = path.join(dataDir, 'events.jsonl');
    this.load();
  }

  private load(): void {
    try {
      if (fs.existsSync(this.filePath)) {
        const lines = fs.readFileSync(this.filePath, 'utf-8').trim().split('\n');
        for (const line of lines.slice(-MAX_EVENTS)) {
          if (line.trim()) {
            this.entries.push(JSON.parse(line));
          }
        }
      }
    } catch { /* ignore */ }
  }

  emit(severity: LogEntry['severity'], subsystem: string, text: string): LogEntry {
    const entry: LogEntry = {
      timestamp: Date.now() / 1000,
      severity,
      subsystem,
      text,
    };
    this.entries.push(entry);
    if (this.entries.length > MAX_EVENTS) {
      this.entries = this.entries.slice(-MAX_EVENTS);
    }
    this.pending.push(entry);

    // Persist
    try {
      fs.appendFileSync(this.filePath, JSON.stringify(entry) + '\n');
    } catch { /* ignore */ }

    // Notify listeners (e.g., telemetry store)
    for (const listener of this.listeners) {
      try { listener(entry); } catch { /* ignore */ }
    }

    return entry;
  }

  onEvent(listener: EventListener): void {
    this.listeners.push(listener);
  }

  getEntries(limit = 100, offset = 0): LogEntry[] {
    const reversed = [...this.entries].reverse();
    return reversed.slice(offset, offset + limit);
  }

  popPending(): LogEntry[] {
    const events = [...this.pending];
    this.pending = [];
    return events;
  }

  clear(): void {
    this.entries = [];
    this.pending = [];
    try {
      fs.writeFileSync(this.filePath, '');
    } catch { /* ignore */ }
  }
}
