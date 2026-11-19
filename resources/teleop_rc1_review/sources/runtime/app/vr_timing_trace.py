"""Bounded, asynchronous VR timing trace. Never writes from the control loop.

All interval/latency calculations use the target PC's monotonic clock. Client
timestamps are annotations only: these clocks cannot measure one-way latency.
"""
import collections
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid


class TimingTrace:
    def __init__(self, directory=None, capacity=4096):
        self.session = uuid.uuid4().hex
        self._lock = threading.Lock()
        self._events = collections.deque(maxlen=capacity)
        self._counts = collections.Counter()
        self._gaps = collections.defaultdict(lambda: collections.deque(maxlen=capacity))
        self._last = {}
        self._queue = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self._observer = None
        self.dropped = 0
        self.writer_errors = 0
        self.observer_errors = 0
        self._directory = Path(directory) if directory else None
        self._thread = threading.Thread(target=self._write, name="vr-trace-writer", daemon=True)
        self._thread.start()

    def set_observer(self, observer):
        self._observer = observer

    def emit(self, stage, **fields):
        record = dict(fields, stage=stage, session=self.session,
                      host_monotonic_ns=time.monotonic_ns())
        with self._lock:
            now = record["host_monotonic_ns"]
            if stage in self._last:
                self._gaps[stage].append((now - self._last[stage]) / 1e6)
            self._last[stage] = now
            self._counts[stage] += 1
            self._events.append(record)
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1
        return record

    @staticmethod
    def _stats(values):
        ordered = sorted(v for v in values if isinstance(v, (int, float)) and math.isfinite(v))
        if not ordered:
            return {"count": 0}
        def percentile(p):
            return ordered[round((len(ordered)-1)*p)]
        return {"count": len(ordered), "p50": percentile(.5), "p95": percentile(.95),
                "p99": percentile(.99), "max": ordered[-1]}

    def snapshot(self):
        with self._lock:
            events = list(self._events)
            counts = dict(self._counts)
            gaps = {k: self._stats(list(v)) for k, v in self._gaps.items()}
        durations = {}
        for key in ("receive_to_callback_ms", "queue_wait_ms", "solve_ms", "target_age_ms",
                    "tick_lateness_ms", "compute_ms", "event_loop_lag_ms"):
            durations[key] = self._stats([e[key] for e in events if key in e])
        return {"session": self.session, "counts": counts, "interval_ms": gaps,
                "duration_ms": durations, "retained_events": len(events),
                "queue_dropped": self.dropped, "writer_errors": self.writer_errors,
                "observer_errors": self.observer_errors,
                "clock": "target_pc_monotonic", "one_way_network_latency_available": False,
                "events_file": str(self._directory / "timing.jsonl") if self._directory else None}

    def events(self, limit=100):
        with self._lock:
            return list(self._events)[-max(1, min(200, int(limit))):]

    def _write(self):
        # This thread alone handles JSON, filesystem rotation and capture callbacks.
        while not self._stop.is_set() or not self._queue.empty():
            try:
                first = self._queue.get(timeout=.2)
            except queue.Empty:
                continue
            batch = [first]
            while len(batch) < 256:
                try:
                    batch.append(self._queue.get_nowait())
                except queue.Empty:
                    break
            if self._directory:
                try:
                    self._directory.mkdir(parents=True, exist_ok=True)
                    path = self._directory / "timing.jsonl"
                    if path.exists() and path.stat().st_size >= 16*1024*1024:
                        for i in (2, 1):
                            old = self._directory / f"timing.{i}.jsonl"
                            if old.exists():
                                old.replace(self._directory / f"timing.{i+1}.jsonl")
                        path.replace(self._directory / "timing.1.jsonl")
                    with path.open("a", encoding="utf-8") as stream:
                        for record in batch:
                            stream.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
                except Exception:
                    self.writer_errors += 1
            observer = self._observer
            if observer:
                for record in batch:
                    if record["stage"] not in ("ik_candidate", "smooth_shadow", "smooth_publish"):
                        continue
                    try:
                        observer("vr/" + record["stage"], record)
                    except Exception:
                        self.observer_errors += 1

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
