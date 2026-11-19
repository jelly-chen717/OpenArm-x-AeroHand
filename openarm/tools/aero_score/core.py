from __future__ import annotations

from concurrent.futures import Future
from dataclasses import dataclass, field
from pathlib import Path
import queue
import re
import sys
import threading
import time

from tools.aero_piano_test.core import AeroTransport, FINGERS, number

# The strict parser is shared unchanged; this import does not load IK/MuJoCo.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skills"))
from paino_skill.score import parse_score

SIDES = ("left", "right")
LABELS = {"left": "左手", "right": "右手"}
CHANNEL = {"index": 3, "middle": 4, "ring": 5, "pinky": 6}
CHARS = {"left": {3: "4", 4: "3", 5: "2", 6: "1"},
         "right": {3: "7", 4: "8", 5: "9", 6: "x"}}
MAX_LATE_NS = 50_000_000


@dataclass(frozen=True)
class Event:
    time_us: int
    lines: tuple[int, ...]
    changes: tuple[tuple[int, bool], ...]


@dataclass(frozen=True)
class FingerScore:
    events: dict[str, tuple[Event, ...]]
    used: dict[str, tuple[int, ...]]
    duration_us: int
    ignored_moves: int
    note_count: int

    @property
    def sides(self):
        return tuple(side for side in SIDES if self.used[side])


def compile_score(text):
    parsed = parse_score(text, finger_count=4)
    grouped = {s: {} for s in SIDES}
    moves = notes = 0
    for action in parsed.actions:
        if action["kind"] == "move":
            moves += 1
            continue  # Explicit finger rehearsal: never call any arm endpoint.
        down = action["direction"] == "D"
        for finger in action["fingers"]:
            side, ch = finger["side"], CHANNEL[finger["finger"]]
            values, lines = grouped[side].setdefault(action["time_us"], ({}, set()))
            if ch in values and values[ch] != down:
                raise ValueError(f"第 {action['line']} 行：同一时刻反转同一手指，实机不能表示零时长击键")
            values[ch] = down
            lines.add(action["line"])
            notes += int(down)
    events = {s: tuple(Event(t, tuple(sorted(lines)), tuple(sorted(values.items())))
                       for t, (values, lines) in sorted(grouped[s].items())) for s in SIDES}
    used = {s: tuple(sorted({ch for e in events[s] for ch, _ in e.changes})) for s in SIDES}
    if not any(used.values()):
        raise ValueError("乐谱没有手指动作")
    return FingerScore(events, used, parsed.duration_us, moves, notes)


def validate_calibration(values, channels):
    result = {}
    for ch in channels:
        row = values.get(ch, values.get(str(ch), {}))
        try:
            home = number(row.get("home", ""), f"{FINGERS[ch]}抬起", 0, 90)
            press = number(row.get("press", ""), f"{FINGERS[ch]}按下", 0, 90)
        except (ValueError, TypeError) as exc:
            raise ValueError(f"请先设置{FINGERS[ch]}的抬起/按下位置") from exc
        if home == press:
            raise ValueError(f"{FINGERS[ch]}的抬起与按下位置不能相同")
        result[ch] = {"home": home, "press": press}
    return result


def validate_ports(settings):
    ports = []
    for side, item in settings.items():
        if item["demo"]:
            continue
        port = item["port"].strip().upper()
        if not re.fullmatch(r"COM[1-9]\d*", port):
            raise ValueError(f"{LABELS[side]}需要真实 COM 口；未接设备时请选择软件模拟")
        ports.append(port)
    if len(ports) != len(set(ports)):
        raise ValueError("左右手不能连接同一个真实 COM 口")


class HandLink:
    """One thread owns one port for its complete lifetime."""
    def __init__(self, side, emit, factory=AeroTransport):
        self.side, self.emit, self.factory = side, emit, factory
        self.bus = None
        self.jobs = queue.Queue()
        self.shutdown = threading.Event()
        self.lock = threading.Lock()
        self.pending = False
        self.thread = threading.Thread(target=self._loop, name=f"aero-score-{side}", daemon=True)
        self.thread.start()

    def submit(self, callback):
        with self.lock:
            if self.pending or self.shutdown.is_set():
                raise RuntimeError(f"{LABELS[self.side]}正在执行其他任务")
            self.pending = True
        future = Future()
        self.jobs.put((callback, future))
        return future

    def connect(self, demo, port):
        if self.bus is not None:
            self.bus.close()
        self.bus = self.factory(demo=demo)
        feedback = self.bus.connect(port)
        self.emit("feedback", (self.side, feedback))
        self.emit("connected", (self.side, demo, port))

    def disconnect(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self.emit("disconnected", self.side)

    def feedback(self, telemetry=True):
        data = self.bus.read(telemetry=telemetry)
        self.emit("feedback", (self.side, data))
        return data

    def _loop(self):
        try:
            while not self.shutdown.is_set():
                try:
                    callback, future = self.jobs.get(timeout=.2)
                except queue.Empty:
                    if self.bus and self.bus.connected and not self.bus.fault:
                        try:
                            self.feedback()
                        except Exception as exc:
                            self.emit("fault", (self.side, str(exc)))
                    continue
                try:
                    value = callback(self)
                except Exception as exc:
                    if self.bus and self.bus.fault:
                        self.emit("fault", (self.side, str(exc)))
                    with self.lock:
                        self.pending = False
                    future.set_exception(exc)
                else:
                    if self.bus and self.bus.fault:
                        self.emit("fault", (self.side, "连接故障；请检查实物并重新连接"))
                    with self.lock:
                        self.pending = False
                    future.set_result(value)
        finally:
            if self.bus:
                self.bus.close()


@dataclass(frozen=True)
class Prepared:
    baseline: tuple[int, ...]
    packets: tuple[tuple[Event, tuple[int, ...]], ...]
    release: tuple[int, ...]


def prepare(link, score, calibration, tolerance):
    bus, side = link.bus, link.side
    if not bus or not bus.connected or bus.fault:
        raise ValueError(f"{LABELS[side]}尚未连接或已故障")
    cal = validate_calibration(calibration, score.used[side])
    tolerance = number(tolerance, "抬起姿态容差", .1, 5)
    measured = link.feedback()
    for ch, row in cal.items():
        if abs(measured["q"][ch] - row["home"]) > tolerance:
            raise ValueError(f"{LABELS[side]}{FINGERS[ch]}未处于抬起位置，请先点击“抬起已配置手指”")
    raw = list(bus.raw)
    baseline = tuple(raw)
    home_raw = {ch: bus.target(ch, row["home"]) for ch, row in cal.items()}
    press_raw = {ch: bus.target(ch, row["press"]) for ch, row in cal.items()}
    packets = []
    for event in score.events[side]:
        for ch, down in event.changes:
            raw[ch] = press_raw[ch] if down else home_raw[ch]
        packets.append((event, tuple(raw)))
    release = list(baseline)
    for ch, value in home_raw.items():
        release[ch] = value
    return Prepared(baseline, tuple(packets), tuple(release))


@dataclass
class Gate:
    start_ns: int = 0
    go: threading.Event = field(default_factory=threading.Event)
    ready: dict = field(default_factory=lambda: {s: threading.Event() for s in SIDES})


def execute(link, prepared, duration_us, gate, stop, abort, record,
            clock=time.monotonic_ns, wait=None):
    """Absolute timestamps; no interpolation/dwell insertion or arm operations."""
    wait = wait or stop.wait
    bus = link.bus
    sent = 0
    outcome = "completed"
    gate.ready[link.side].set()
    try:
        while not gate.go.is_set():
            if stop.is_set() or abort.is_set():
                outcome = "cancelled-before-start"
                return outcome
            wait(.005)
        last_poll = clock()
        end = gate.start_ns + duration_us * 1000
        while True:
            if abort.is_set():
                outcome = "aborted-no-release"
                break
            if stop.is_set():
                outcome = "stopped"
                break
            now = clock()
            if sent < len(prepared.packets):
                event, raw = prepared.packets[sent]
                due = gate.start_ns + event.time_us * 1000
                if now >= due:
                    if now - due > MAX_LATE_NS:
                        raise TimeoutError(f"{LABELS[link.side]}第 {event.lines} 行迟到超过 50 ms")
                    if sent + 1 < len(prepared.packets):
                        next_event = prepared.packets[sent + 1][0]
                        if now >= gate.start_ns + next_event.time_us * 1000:
                            raise TimeoutError(f"{LABELS[link.side]}已错过多个时刻；不集中补发")
                    # No side effects occur between this cancellation check and IO.
                    if stop.is_set() or abort.is_set():
                        continue
                    bus.write(list(raw))
                    finished = clock()
                    record(dict(side=link.side, demo=bus.demo, kind="hand", lines=event.lines,
                                scheduled_us=event.time_us, sent_us=(now-gate.start_ns)//1000,
                                finished_us=(finished-gate.start_ns)//1000,
                                changes=event.changes))
                    sent += 1
                    if finished - due > MAX_LATE_NS:
                        raise TimeoutError(f"{LABELS[link.side]}串口发送完成超过 50 ms")
                    continue
            else:
                due = end
                if now >= end:
                    break
            # Read only positions during playback. Avoid blocking a near-future
            # event when possible, but never let an active stream lose feedback.
            age = now - last_poll
            if age >= 200_000_000 and (due - now > 60_000_000 or age >= 800_000_000):
                link.feedback(telemetry=False)
                last_poll = clock()
                continue
            wait(min(.002, max(0., (due - now) / 1e9)))
    except Exception:
        outcome = "fault"
        stop.set()  # A fault on either hand cancels the other hand immediately.
        raise
    finally:
        # On healthy stop/timing fault release only this score's fingers. A
        # failed connection or explicit hard abort must never trigger blind IO.
        if sent and outcome in ("stopped", "fault") and not abort.is_set() and not bus.fault:
            try:
                link.feedback(telemetry=False)
                if not abort.is_set():
                    bus.write(list(prepared.release))
                    record(dict(side=link.side, demo=bus.demo, kind="release", outcome=outcome))
            except Exception as exc:
                record(dict(side=link.side, demo=bus.demo, kind="release-failed", error=str(exc)))
        record(dict(side=link.side, demo=bus.demo, kind="end", outcome=outcome, packets_sent=sent))
    return outcome


def run_dual(links, score, calibrations, tolerance, stop, abort, record):
    prep_futures = {side: links[side].submit(
        lambda link, s=side: prepare(link, score, calibrations[s], tolerance)) for side in score.sides}
    prepared = {side: future.result(timeout=5) for side, future in prep_futures.items()}
    if stop.is_set() or abort.is_set():
        return {s: "cancelled-before-start" for s in score.sides}
    gate = Gate()
    futures = {}
    try:
        for side in score.sides:
            futures[side] = links[side].submit(lambda link, s=side: execute(
                link, prepared[s], score.duration_us, gate, stop, abort, record))
        deadline = time.monotonic() + 3
        for side in score.sides:
            while not gate.ready[side].wait(.01):
                if stop.is_set() or abort.is_set() or time.monotonic() > deadline:
                    raise RuntimeError("双手调度未就绪，已取消")
        if stop.is_set() or abort.is_set():
            return {s: "cancelled-before-start" for s in score.sides}
        gate.start_ns = time.monotonic_ns() + 200_000_000
        gate.go.set()
        results, errors = {}, []
        for side, future in futures.items():
            try:
                results[side] = future.result()
            except Exception as exc:
                errors.append(f"{LABELS[side]}：{exc}")
        if errors:
            raise RuntimeError("；".join(errors))
        return results
    finally:
        # Prevent a cancelled preparation/start from leaving a waiter armed.
        if not gate.go.is_set():
            stop.set()
            gate.go.set()
        for future in futures.values():
            try:
                future.result(timeout=2)
            except Exception:
                pass
