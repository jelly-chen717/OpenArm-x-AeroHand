"""Aero hand control for the loopback console. Each port has one worker."""
from concurrent.futures import wait
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openarm"))
from tools.aero_piano_test.core import AeroTransport, FINGERS, number
from tools.aero_score.core import CHANNEL, Event, FingerScore, HandLink, LABELS, compile_score, validate_ports
from openarm_piano_air import CHORD_SCORE as JIN_SCORE


class ConsoleHands:
    def __init__(self, hardware, journal, directory=ROOT / "configs", factory=AeroTransport):
        self.hardware, self.journal = hardware, journal
        self.directory, self.factory = directory, factory
        self.defaults = json.loads((ROOT / "openarm/tools/aero_score/config.json").read_text(
            encoding="utf-8"))["sides"]
        self.lock = threading.RLock()
        self.operations = {side: threading.Lock() for side in LABELS}
        self.links = {}
        self.targets = {}
        self.stopping = {side: threading.Event() for side in LABELS}
        self.closed = False
        self.score = dict(active=False, message="等待排练")
        self.score_watch = None
        self.score_client = None
        self.rows = {}
        for side, item in self.defaults.items():
            real_default = hardware and side == "right"
            self.rows[side] = dict(connected=False, session=None, fault=None, feedback=None,
                                   demo=not real_default, port=item["port"] if real_default else "")
            self.rows[side]["calibration"] = {}

    def _path(self, side, demo):
        return self.directory / f"aero_monitor_{side}_{'simulation' if demo else 'real'}.json"

    def _calibration(self, side, demo):
        path = self._path(side, demo)
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
        return deepcopy(self.defaults[side]["simulation" if demo else "real"])

    def snapshot(self):
        with self.lock:
            return deepcopy(self.rows)

    def score_status(self, client):
        with self.lock:
            if self.score["active"] and client == self.score_client:
                self.score_watch.touch()
            return dict(self.score)

    def start_score(self, payload, client, watch):
        score = compile_score(JIN_SCORE.read_text(encoding="utf-8-sig"))
        return self.schedule_score(score, payload.get("sessions"), client, watch)

    def schedule_score(self, score, sessions, client, watch, *, run_id=None, arm_clock=None):
        reserved = []
        positions = {}
        with self.lock:
            try:
                if self.closed or self.score["active"]:
                    raise ValueError("服务正在关闭或已有指法排练。")
                for side in score.sides:
                    if not self.operations[side].acquire(blocking=False):
                        raise ValueError(f"{LABELS[side]}正在处理其他指令。")
                    reserved.append(side)
                    row = self.rows[side]
                    if (not row["connected"] or self.stopping[side].is_set() or row["fault"]
                            or not isinstance(sessions, dict) or sessions.get(side) != row["session"]):
                        raise ValueError(f"请先连接{LABELS[side]}并确认连接状态。")
                    positions[side] = {}
                    for ch in score.used[side]:
                        cal = row["calibration"].get(str(ch), {})
                        try:
                            positions[side][ch] = {key: number(cal.get(key), key, 0, 90)
                                                   for key in ("press", "home")}
                        except (TypeError, ValueError) as exc:
                            raise ValueError(f"请先记录{LABELS[side]}{FINGERS[ch]}的按下位和抬起位。") from exc
                run_id = run_id or uuid.uuid4().hex
                self.journal.record("hand.score.start", JIN_SCORE.stem, request_id=run_id,
                                    data=dict(notes=score.note_count, ignored_moves=score.ignored_moves,
                                              demo={s: self.rows[s]["demo"] for s in score.sides}))
                self.score_client, self.score_watch = client, watch
                threading.Thread(target=self._play_score, args=(score, positions, watch, run_id, arm_clock),
                                 daemon=True, name="console-hand-score").start()
                self.score = dict(active=True, id=run_id, message="排练中", elapsed_s=0,
                                  sides=score.sides,
                                  duration_s=score.duration_us / 1e6)
                return dict(self.score)
            except Exception:
                for side in reserved:
                    self.operations[side].release()
                raise

    def stop_score(self, run_id):
        with self.lock:
            if not self.score["active"] or self.score["id"] != run_id:
                raise ValueError("该次指法排练已结束或已变更。")
            self.score_watch.stop("已停止排练")
            return dict(self.score)

    def _play_score(self, score, positions, watch, run_id, arm_clock=None):
        held = {s: set() for s in score.sides}
        timeline = {}
        for side in score.sides:
            for event in score.events[side]:
                timeline.setdefault(event.time_us, {})[side] = event

        def send(link, changes, releasing=False):
            side = link.side
            raw = list(self.targets[side])
            for ch, down in changes:
                raw[ch] = link.bus.target(ch, positions[side][ch]["press" if down else "home"])
            if not releasing and watch.is_set():
                return
            held[side].update(ch for ch, down in changes if down)
            link.bus.write_position_frame(raw)
            self.targets[side] = raw
            for ch, down in changes:
                if not down:
                    held[side].discard(ch)

        message = "排练完成"
        try:
            if arm_clock is not None:
                while not arm_clock.done() and not watch.wait(.01):
                    pass
                if arm_clock.cancelled():
                    message = "试弹未提交"
                    return
            start = arm_clock.result() if arm_clock is not None and arm_clock.done() else time.monotonic()
            timeline.setdefault(score.duration_us, {})
            for time_us, events in sorted(timeline.items()):
                while not watch.is_set():
                    elapsed = time.monotonic() - start
                    with self.lock:
                        self.score["elapsed_s"] = min(elapsed, score.duration_us / 1e6)
                        faults = [self.rows[s]["fault"] for s in score.sides if self.rows[s]["fault"]]
                    if faults:
                        raise OSError("、".join(faults))
                    remaining = time_us / 1e6 - elapsed
                    if remaining <= 0:
                        break
                    watch.event.wait(min(remaining, .1))
                if watch.is_set():
                    message = watch.reason
                    break
                futures = []
                try:
                    for side, event in events.items():
                        futures.append(self.links[side].submit(
                            lambda link, changes=event.changes: send(link, changes)))
                finally:
                    # Both workers must finish before cleanup inspects the held fingers.
                    wait(futures)
                for future in futures:
                    future.result()
                if watch.is_set():
                    message = watch.reason
                    break
                if events:
                    self.journal.record("hand.score.frame", JIN_SCORE.stem, request_id=run_id,
                                        data=dict(time_us=time_us, sent_s=time.monotonic() - start,
                                                  changes={s: e.changes for s, e in events.items()}))
        except Exception as exc:
            message = str(exc)
        finally:
            for side, channels in held.items():
                if channels:
                    changes = tuple((ch, False) for ch in channels)
                    try:
                        self.links[side].submit(lambda link, c=changes: send(link, c, True)).result()
                    except Exception as exc:
                        message += f" · {LABELS[side]}抬起未确认：{exc}"
            with self.lock:
                self.score.update(active=False, message=message)
                for side in score.sides:
                    self.operations[side].release()
                self.journal.record("hand.score.end", message, request_id=run_id)

    def _event(self, kind, payload):
        if kind not in ("feedback", "fault"):
            return
        side, value = payload
        with self.lock:
            if kind == "feedback":
                self.rows[side].update(feedback=value, received_at=time.time())
            else:
                self.rows[side]["fault"] = value
                self.journal.record("hand.fault", LABELS[side] + " " + value, level="ERROR")

    def _disconnect(self, side):
        with self.lock:
            link = self.links.get(side)
            if not link:
                return
            link.shutdown.set()
        link.thread.join()  # worker closes the serial port before releasing the reservation
        with self.lock:
            self.links.pop(side)
            self.targets.pop(side, None)
            self.rows[side].update(connected=False, session=None, feedback=None,
                                   received_at=None, fault=None)

    def _connect(self, side, payload):
        demo, port = payload.get("demo"), payload.get("port", "")
        if type(demo) is not bool or not isinstance(port, str) or (not self.hardware and not demo):
            raise ValueError("离线服务仅支持模拟手。请选择有效的连接方式。")
        port = port.strip().upper()
        with self.lock:
            if side in self.links:
                raise ValueError("该手已连接，请先停止并断开。")
            settings = {s: self.rows[s] for s in self.links}
            settings[side] = dict(demo=demo, port=port)
            validate_ports(settings)
            cal = self._calibration(side, demo)
            self.rows[side].update(demo=demo, port=port, calibration=cal, fault=None)
            self.stopping[side].clear()
            link = HandLink(side, self._event, self.factory)
            self.links[side] = link
        def open_port(worker):
            worker.connect(demo, port)
            self.targets[side] = list(worker.bus.raw)
        try:
            link.submit(open_port).result()
        except Exception:
            self._disconnect(side)
            raise
        with self.lock:
            self.rows[side].update(connected=True, session=uuid.uuid4().hex)

    def _finger(self, worker, action, ch, payload):
        side, bus = worker.side, worker.bus
        feedback = worker.feedback(telemetry=False)
        row = self.rows[side]
        key = str(ch)
        if action in ("save_home", "save_press"):
            degrees = round(number(feedback["q"][ch], "反馈角度", 0, 90), 3)
            cal = deepcopy(row["calibration"])
            cal[key][action.removeprefix("save_")] = degrees
            path = self._path(side, row["demo"])
            temp = path.with_suffix(f".{uuid.uuid4().hex}.tmp")
            try:
                temp.write_text(json.dumps(cal, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                os.replace(temp, path)
            finally:
                temp.unlink(missing_ok=True)
            with self.lock:
                row["calibration"] = cal
            return degrees
        degrees = payload.get("degrees") if action == "move" else row["calibration"][key][action]
        if degrees in (None, "") or isinstance(degrees, bool):
            raise ValueError("请先填写目标角度或记录该指位置。")
        degrees = number(degrees, "目标角度", 0, 90)
        raw = list(self.targets[side])
        raw[ch] = bus.target(ch, degrees)
        if self.stopping[side].is_set():
            raise ValueError("手部操作已取消。")
        bus.write_position_frame(raw)
        self.targets[side] = raw
        worker.feedback()
        return degrees

    def action(self, payload):
        side, action = payload.get("side"), payload.get("action")
        if not isinstance(side, str) or side not in LABELS:
            raise ValueError("请选择左手或右手。")
        if action == "disconnect":
            with self.lock:
                if self.rows[side]["connected"] and payload.get("session") == self.rows[side]["session"]:
                    self.stopping[side].set()
                    if self.score["active"] and side in self.score["sides"]:
                        self.score_watch.stop("手部连接已停止")
        lock = self.operations[side]
        if not lock.acquire(blocking=action == "disconnect"):
            raise ValueError("该手正在处理上一条指令。")
        try:
            if self.closed:
                raise ValueError("服务正在关闭。")
            row = self.rows[side]
            details = dict(side=side, action=action)
            if action == "connect":
                self._connect(side, payload)
            else:
                if not row["connected"] or payload.get("session") != row["session"]:
                    raise ValueError("手部连接已变更，请刷新状态。")
                if action == "disconnect":
                    self._disconnect(side)
                elif action in ("move", "home", "press", "save_home", "save_press"):
                    ch = payload.get("channel")
                    if type(ch) is not int or ch not in FINGERS:
                        raise ValueError("请选择食指至小指。")
                    degrees = self.links[side].submit(
                        lambda worker: self._finger(worker, action, ch, payload)).result()
                    details.update(channel=ch, degrees=degrees)
                else:
                    raise ValueError("未知手部操作。")
            if row["feedback"]:
                details["feedback_deg"] = row["feedback"]["q"]
            details.update(demo=row["demo"], port=row["port"])
            self.journal.record("hand." + action, LABELS[side], data=details)
            return self.snapshot()[side]
        except (ImportError, RuntimeError, TypeError) as exc:
            raise ValueError(str(exc)) from exc
        finally:
            lock.release()

    def disconnect_all(self):
        with self.lock:
            for signal in self.stopping.values():
                signal.set()
            if self.score["active"]:
                self.score_watch.stop("控制台停止")
        for side in LABELS:
            with self.operations[side]:
                self._disconnect(side)

    def close(self):
        self.closed = True
        self.disconnect_all()
