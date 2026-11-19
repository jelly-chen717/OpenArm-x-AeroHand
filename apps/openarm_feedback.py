"""Feedback snapshots and bounded worker lifecycle, independent of MuJoCo and CAN."""
from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class Sample:
    motor_id: int
    position: float
    velocity: float
    received_at: float
    state: int = 0


class FeedbackCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.batches = {}
        self.sequence = 0

    def update(self, side, samples, error=None):
        if side not in ('left', 'right'):
            raise ValueError('Invalid side')
        samples = tuple(samples)
        with self.lock:
            self.sequence += 1
            self.batches[side] = (samples, error, self.sequence)

    def snapshot(self, side, now, max_age, max_skew):
        if not all(math.isfinite(v) and v > 0 for v in (max_age, max_skew)):
            raise ValueError('Invalid feedback thresholds')
        with self.lock:
            samples, error, seq = self.batches.get(side, ((), 'no feedback', 0))
        reason = error
        if not reason and (len(samples) != 7 or {s.motor_id for s in samples} != set(range(1, 8))):
            reason = 'incomplete feedback'
        if not reason and any(not all(math.isfinite(v) for v in
                (s.position, s.velocity, s.received_at)) for s in samples):
            reason = 'nonfinite feedback'
        age = skew = None
        if not reason:
            stamps = [s.received_at for s in samples]
            age, skew = now - min(stamps), max(stamps) - min(stamps)
            if now < max(stamps) or age > max_age:
                reason = 'stale or future feedback'
            elif skew > max_skew:
                reason = 'sampling skew exceeded'
            elif any(s.state not in (0, 1) for s in samples):
                reason = 'motor fault'
        return dict(side=side, valid=not bool(reason), reason=reason, age_s=age,
                    skew_s=skew, sequence=seq, samples=samples)


class FeedbackWorker:
    """Source is opened, read and closed exclusively on its own worker thread."""
    def __init__(self, source_factory, side, cache, period=.03, reconnect_delay=.5):
        if not math.isfinite(period) or period <= 0:
            raise ValueError('Invalid period')
        self.source_factory, self.side, self.cache, self.period = source_factory, side, cache, period
        if not math.isfinite(reconnect_delay) or reconnect_delay <= 0:
            raise ValueError('Invalid reconnect delay')
        self.reconnect_delay = reconnect_delay
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='feedback-'+side)
        self.error = None

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                with self.source_factory() as source:
                    while not self.stop_event.is_set():
                        started = time.monotonic()
                        self.cache.update(self.side, source.read())
                        self.error = None
                        self.stop_event.wait(max(0, self.period-(time.monotonic()-started)))
            except Exception as exc:
                self.error = str(exc)
                # Reopen after failures, publishing no samples from the old connection.
                self.cache.update(self.side, (), self.error)
                self.stop_event.wait(self.reconnect_delay)

    def stop(self, timeout=4):
        self.stop_event.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            raise RuntimeError('Feedback worker did not stop: '+self.side)


class PcanFeedbackSource:
    """Query-only adapter. Never invokes motor enable, MIT, disable or parameter writes."""
    def __init__(self, side, channel, timeout=.1):
        from openarm_driver import OpenArmDriver
        self.driver = OpenArmDriver(side, channel, timeout)

    def __enter__(self):
        try:
            self.driver.__enter__()
        except BaseException:
            self.driver.__exit__(None, None, None)
            raise
        return self

    def read(self):
        result = []
        for i in range(1, 8):
            s = self.driver.query_motor(i)
            result.append(Sample(i, s.position_rad, s.velocity_rad_s, s.received_at, s.state))
        return result

    def __exit__(self, *args):
        return self.driver.__exit__(*args)
