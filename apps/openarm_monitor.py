"""Loopback-only OpenArm console. A single worker owns the selected transport.

Laboratory profile (2026-11-19): the console lets an operator enable, retarget and
hold all 14 axes without stationary/tracking/travel thresholds aborting the run.
Retained interlocks are correctness and stop-confirmation checks; the full list is
in docs/project/OpenArm防护删除_总纲_20261125.md.
"""
import argparse
from collections import deque
from concurrent.futures import Future
from contextlib import closing
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid
import webbrowser

from drive_openarm_from_sim import execute
from openarm_driver import DriverError, OpenArmDriver
from openarm_monitor_hands import ConsoleHands
from openarm_joint_mapping import load_mapping
from openarm_monitor_motion import KEYS, SIDES, MonitorTransport, MotionPlanner, VirtualDrive, grouped
from openarm_piano_origin import PianoFingerProjection, load_origin
from openarm_piano_performance import SCOPES, fingering_preview, performance_options
from openarm_piano_sync import PianoClock
from openarm_piano_wrist import WristTrajectory
from openarm_monitor_modes import DRIVE_MODES, MODE_CODES, DriveProfile, ModeRegisters

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "monitor-ui"
CHANNELS = {"left": 0x51, "right": 0x52}
FEEDBACK_AGE = .75  # UI cache limit; execution samples have their own age notes.
HEARTBEAT_TIMEOUT = 3.
PERIOD = .04
ENDPOINT_TOLERANCE = .015  # Logged as an advisory, not an abort condition.
MAX_LATENESS = .08  # Logged as an advisory, not an abort condition.


class Journal:
    def __init__(self, directory, mode):
        self.id = uuid.uuid4().hex
        self.mode = mode
        self.path = directory / f"{mode}-{self.id}.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", encoding="utf-8")
        self.events = deque(maxlen=400)
        self.sequence = 0
        self.lock = threading.Lock()
        self.error = None

    def record(self, event, message="", *, level="INFO", request_id=None, data=None, important=False):
        row = dict(timestamp=datetime.now(timezone.utc).isoformat(), event=event,
                   level=level, mode=self.mode, request_id=request_id, message=message,
                   data=data, important=important)
        with self.lock:
            try:
                self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                self.stream.flush()
            except OSError as exc:
                self.error = str(exc)
                raise
            if event != "motion.sample":
                self.sequence += 1
                self.events.append(dict(row, sequence=self.sequence))

    def since(self, sequence):
        with self.lock:
            return [row for row in self.events if row["sequence"] > sequence]


class OperatorWatch:
    """The browser running a finite motion must keep its status connection alive."""
    def __init__(self):
        self.event = threading.Event()
        self.lock = threading.Lock()
        self.last_seen = time.monotonic()
        self.reason = None

    def touch(self):
        with self.lock:
            if not self._expired():
                self.last_seen = time.monotonic()

    def stop(self, reason):
        with self.lock:
            if not self.event.is_set():
                self.reason = reason
                self.event.set()

    def is_set(self):
        with self.lock:
            return self._expired()

    def _expired(self):
        if not self.event.is_set() and time.monotonic() - self.last_seen > HEARTBEAT_TIMEOUT:
            self.reason = "操控页面连接超时"
            self.event.set()
        return self.event.is_set()

    def wait(self, duration):
        return self.event.wait(duration) or self.is_set()


@dataclass
class Job:
    plan: dict
    client: str
    watch: OperatorWatch
    state: str = "QUEUED"
    active: bool = True
    sample: dict | None = None
    message: str = ""
    stop_confirmed: bool = False
    piano_exit_id: str | None = None
    piano_exited: bool = False

    def __post_init__(self):
        self.session_id = self.id

    @property
    def id(self):
        return self.plan["request_id"]

    def public(self):
        return dict(request_id=self.id, session_id=self.session_id, active=self.active, state=self.state,
                    message=self.message, stop_confirmed=self.stop_confirmed,
                    stop_reason=self.watch.reason, duration_s=self.plan["duration"],
                    target_deg=grouped(self.plan["start"] | self.plan["goal"], math.degrees),
                    sample=self.sample, piano_trial=self.plan.get("piano_trial"),
                    piano_exiting=self.piano_exit_id == self.id, piano_exited=self.piano_exited)


class Conflict(RuntimeError):
    pass


class Monitor:
    def __init__(self, mode, channels, log_directory, *, driver_factory=OpenArmDriver, profile_path=None,
                 inventory_path=None):
        self.mode, self.channels = mode, channels
        self.hardware = mode == "hardware"
        self.driver_factory = driver_factory
        self.version, self.mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json",
                                                  hardware=self.hardware)
        self.planner = MotionPlanner(self.version, self.mappings, virtual=not self.hardware)
        self.piano_performance_scores = performance_options()
        try:
            points, anchors, _ = load_origin()
            self.piano_origin = {s: dict(anchor=anchors[s].key, note=anchors[s].note,
                                        physical_key=anchors[s].physical_key, keys=sorted(points[s])) for s in SIDES}
            self.piano_projection = PianoFingerProjection.load(anchors)
        except (ValueError, KeyError, IndexError, TypeError, OSError):
            self.piano_origin = None
            self.piano_projection = None
        self.limits_deg = {side: [[math.degrees(v) for v in self.mappings[side, j].limits]
                                  for j in range(1, 8)] for side in SIDES}
        self.journal = Journal(log_directory, mode)
        self.lock = threading.Lock()
        self.closing = threading.Event()
        self.wake = threading.Event()
        self.worker = threading.Thread(target=self._run, name="openarm-monitor")
        self.drivers = {}  # Accessed exclusively on the worker thread.
        self.reconnect_at = dict.fromkeys(SIDES, 0.)
        self.positions = self.planner.home.copy()
        self.feedback = {}
        self.errors = dict.fromkeys(SIDES, "等待反馈")
        self.preview = None
        self.job = None
        if (profile_path or inventory_path) and not self.hardware:
            raise ValueError("实机协议配置仅用于 hardware 服务。")
        self.drive_profile = DriveProfile(profile_path, self.version, channels) if profile_path else None
        self.mode_registers = self.drive_profile
        if inventory_path and not self.mode_registers:
            self.mode_registers = ModeRegisters(inventory_path, self.version, channels)
        self.drive_mode = None if self.hardware else "MIT"
        self.axis_modes = {}
        self.next_mode_read = dict.fromkeys(SIDES, 0.)
        self.mode_error = None
        self.expected_modes = None
        self.mode_request = None
        self.revision = 0
        self.continuation = None
        self.hands = ConsoleHands(self.hardware, self.journal)
        self.journal.record("service.ready", "实机服务已启动" if self.hardware else "离线仿真服务已启动",
                            important=True, data=dict(mode=mode, mapping_version=self.version,
                                                      channels=channels if self.hardware else None))

    def start(self):
        self.worker.start()

    def close(self):
        with self.lock:
            self.closing.set()
            if self.job and self.job.active:
                self.job.watch.stop("本机服务关闭")
        self.wake.set()
        self.hands.close()
        self.worker.join()
        self.journal.stream.close()

    def _feedback_view(self):
        """Call with the state lock held."""
        now = time.monotonic()
        arms, positions = {}, {}
        for side in SIDES:
            rows, ages = [], []
            for j in range(1, 8):
                key = side, j
                row = self.feedback.get(key)
                age = now - row["received_at"] if row is not None else None
                if age is None or not 0 <= age <= FEEDBACK_AGE:
                    rows.append(None)
                    continue
                q = self.mappings[key].to_model(row["position_rad"])
                v = self.mappings[key].velocity_to_model(row["velocity_rad_s"])
                positions[key] = q
                ages.append(age)
                rows.append(dict(position_deg=math.degrees(q), velocity_deg_s=math.degrees(v),
                                 torque_nm=row.get("torque_nm"), mos_c=row.get("mos_c"),
                                 rotor_c=row.get("rotor_c"), state=row["state"],
                                 age_ms=round(age * 1000)))
            fresh = len(ages) == 7
            disabled = fresh and all(row["state"] == 0 for row in rows)
            ready = disabled and all(abs(row["velocity_deg_s"]) <= math.degrees(.1) for row in rows)
            reason = self.errors[side]
            if not reason:
                if not fresh:
                    reason = "反馈缺失或超时"
                elif any(row["state"] not in (0, 1) for row in rows):
                    reason = "电机故障"
            arms[side] = dict(fresh=fresh, disabled=disabled, ready=ready, reason=reason,
                              age_ms=round(max(ages) * 1000) if ages else None, joints=rows)
        return arms, positions

    def _idle(self, arms):
        """Service idle with fresh feedback. Disabled/stationary state is not required."""
        return (not self.closing.is_set() and not self.journal.error and not self.mode_request
                and not (self.job and self.job.active)
                and all(arm["fresh"] for arm in arms.values()))

    def _stable(self, arms):
        """All 14 axes disabled and stationary. Only motor-mode switching needs this."""
        return all(arm["ready"] and not arm["reason"] for arm in arms.values())

    def _can_prepare(self, arms, client):
        if not self._motion_supported():
            return False
        if self._idle(arms):
            return True
        job = self.job
        if (not job or not job.active or job.state != "HOLDING_TARGET" or job.client != client
                or job.watch.is_set() or job.piano_exit_id or self.continuation or self.journal.error):
            return False
        return all(arm["fresh"] for arm in arms.values())

    def _motion_supported(self):
        # The uniform 14-axis readback decides which control mode the console runs.
        # Every mode sends its own target frames and reaches enable/disable through
        # the base-ID special frames (see MonitorTransport); a commissioned drive
        # profile only overrides addressing or the POS_FORCE current fraction, so
        # enabling and moving in POS_VEL must not wait for one.
        return bool(self.drive_mode and not self.mode_error)

    def _start_ready(self, arms, client):
        """Fresh 14-axis feedback and either idle or held by this client.

        The recipe enables every axis it commands, so a prior “全部使能” is not a
        precondition. Only feedback availability and job ownership are checked.
        """
        if not self._motion_supported() or self.journal.error:
            return False
        if not all(arm["fresh"] and not arm["reason"] for arm in arms.values()):
            return False
        if self._idle(arms):
            return True
        job = self.job
        return bool(job and job.active and job.state == "HOLDING_TARGET"
                    and job.client == client and not job.watch.is_set()
                    and not job.piano_exit_id and self.continuation is None)

    def _start_plan(self, source, current, holding):
        if source == "piano-origin":
            return self.planner.prepare_piano_origin(current, holding=holding)
        if source == "zero-pose":
            return self.planner.prepare_zero_pose(current, holding=holding)
        if source == "start-return":
            return self.planner.prepare_start_return(current, holding=holding)
        if source == "zero-return":
            return self.planner.prepare_zero_return(current, holding=holding)
        return self.planner.prepare_start_pose(current, holding=holding)

    def _check_continuation_axes(self, plan):
        if self.job and self.job.active and set(plan["keys"]) != set(self.job.plan["keys"]):
            raise Conflict("接续轴集合与当前保持不一致，请先停止。")

    def start_pose_motion(self, source, client):
        """One-click both-arm start recipe: full approach or quick return."""
        with self.lock:
            arms, current = self._feedback_view()
            if not self._start_ready(arms, client):
                raise Conflict("等待 14 轴反馈就绪后再执行起始位。")
            revision = self.revision
            if self.job and self.job.active:
                holding = self.job.plan["goal"]
            else:
                # An enabled-but-idle service still has all 14 axes live. Keep
                # every enabled axis in the plan so the stop path owns them too.
                holding = dict(current)
        plan, request = self._start_plan(source, current, holding)
        with self.lock:
            arms, _ = self._feedback_view()
            if revision != self.revision or not self._start_ready(arms, client):
                raise Conflict("控制台状态已变化，请重新操作。")
            self._check_continuation_axes(plan)
            if source == "zero-pose":
                event, message = "motion.zero_pose", "0位：双臂回到 J7 +15°，其余 0°"
            elif source == "start-return":
                event, message = "motion.start_return", "回0—起始位：0位 → J1+15°/J2−45° → J1−30°/J4+90° → 起始位"
            elif source == "zero-return":
                event, message = "motion.zero_return", ("逆方向返回：当前位置 → 起始位 → J1−30°/J2−45°/J4+90° → "
                                                        "J1+15°/J2−45° → 0位（到位后保持使能）")
            elif source == "piano-origin":
                event, message = "motion.piano_origin", plan["steps"][0][0]
            else:
                event, message = "motion.start_pose", "起始位：双臂直接回起始位"
            self.journal.record(event, message, request_id=plan["request_id"], important=True,
                                data=dict(source=source, trajectory=request,
                                          sequence=[dict(name=name, duration_s=seconds,
                                                         right_deg=[math.degrees(target[("right", j)])
                                                                    for j in range(1, 8)],
                                                         left_deg=[math.degrees(target[("left", j)])
                                                                   for j in range(1, 8)])
                                                    for name, seconds, target in plan.get("steps", [])]))
            if plan.get("notice"):
                self.journal.record("motion.notice", plan["notice"], level="WARN",
                                    request_id=plan["request_id"])
            self.preview = None
            self.revision += 1
            if self.job and self.job.active:
                self.continuation = plan
                result = dict(Job(plan, client, self.job.watch).public(),
                              session_id=self.job.session_id)
            else:
                self.job = Job(plan, client, OperatorWatch())
                result = self.job.public()
        self.wake.set()
        return result

    def status(self, after, client):
        with self.lock:
            if self.job and self.job.active and self.job.client == client:
                self.job.watch.touch()
            arms, positions = self._feedback_view()
            ready = self._can_prepare(arms, client)
            idle, stable = self._idle(arms), self._stable(arms)
            drive_mode, revision = self.drive_mode, self.revision
            pending_mode = self.mode_request
            axis_modes = grouped(self.axis_modes)
            switchable = idle and stable and (not self.hardware or
                         (self.mode_registers is not None and len(self.axis_modes) == 14))
            motion_supported = self._motion_supported()
            start_ready = self._start_ready(arms, client)
            all_enabled = all(row is not None and row["state"] == 1
                              for arm in arms.values() for row in arm["joints"])
            mode_error = self.mode_error
            job = self.job.public() if self.job else None
        points, wrist_rpy = self.planner.end_effector_feedback(positions)
        for side, arm in arms.items():
            endpoints = points.get(side, {}) if not arm["reason"] else {}
            arm["endpoints_mm"] = endpoints
            arm["wrist_rpy_deg"] = wrist_rpy.get(side) if endpoints else None
            arm["piano_finger_keys"] = (self.piano_projection.keys_at(side, endpoints["wrist"][1])
                                       if endpoints and self.piano_projection else {})
        return dict(instance=self.journal.id, mode=self.mode, mapping_version=self.version,
                    limits_deg=self.limits_deg, arms=arms, hands=self.hands.snapshot(),
                    hand_score=self.hands.score_status(client), can_prepare=ready, run=job,
                    drive_mode=drive_mode, revision=revision, can_enable_all=idle and motion_supported,
                    axis_modes=axis_modes, mode_error=mode_error, motion_supported=motion_supported,
                    all_enabled=all_enabled, can_start_pose=start_ready,
                    piano_trial_available=True, piano_origin=self.piano_origin,
                     piano_fingering_scores=list(self.piano_performance_scores), piano_trial_scopes=SCOPES,
                     piano_performance_scores=self.piano_performance_scores,
                    pending_mode=pending_mode,
                    can_switch_mode=switchable,
                    drive_modes=() if self.hardware and not self.mode_registers else DRIVE_MODES,
                    log_error=self.journal.error, events=self.journal.since(after))

    def prepare(self, payload, client):
        with self.lock:
            arms, current = self._feedback_view()
            if not self._can_prepare(arms, client):
                raise Conflict("当前模式或反馈状态不允许预览位置目标。")
            revision = self.revision
            held_plan = self.job.plan if self.job and self.job.active else None
            holding = held_plan["goal"] if held_plan else None
            if self.preview and self.preview[1] == client:
                self.preview = None
        if payload.get("mode") in ("piano-trial", "piano-performance"):
            plan, request = self.planner.prepare_piano_trial(payload, current, held_plan)
        else:
            plan, request = self.planner.prepare(payload, current, holding=holding)
        with self.lock:
            arms, _ = self._feedback_view()
            if revision != self.revision or not self._can_prepare(arms, client):
                raise Conflict("控制台状态已变化，请重新预览。")
            details = dict(input=payload, trajectory=request, notice=plan.get("notice"))
            if "piano_trial" in plan:
                details["piano_trial"] = plan["piano_trial"]
            self.journal.record("motion.preview", "目标已通过预检", request_id=plan["request_id"],
                                data=details, important=True)
            if plan.get("notice"):
                self.journal.record("motion.notice", plan["notice"], level="WARN",
                                    request_id=plan["request_id"])
            self.preview = plan, client
        motion = request["points"]
        peak = max(abs(a - b) for a, b in zip(motion[0]["q"], motion[1]["q"]))
        result = dict(request_id=plan["request_id"], duration_s=plan["duration"],
                      revision=revision, command_joints=[f"{side} J{j}" for side, j in plan["commanded"]],
                      target_deg=grouped(current | plan["goal"], math.degrees),
                      peak_speed_deg_s=math.degrees(1.875 * peak / motion[1]["t"]))
        if "piano_trial" in plan:
            result["piano_trial"] = plan["piano_trial"]
            trajectory = plan["trajectory"]
            tracks = getattr(trajectory, "arms", (trajectory,))
            result["peak_speed_deg_s"] = math.degrees(max(
                arm.peak_bounds(t0, q0, q1, t1-t0)[0] for arm in tracks
                for (t0, q0), (t1, q1) in zip(arm.points, arm.points[1:])))
        return result

    def play_piano(self, payload, client):
        preview = self.prepare(dict(mode="piano-performance", score_id=payload.get("score_id")), client)
        return self.start_motion(preview["request_id"], client, payload.get("sessions"))

    def exit_piano(self, request_id):
        with self.lock:
            job = self.job
            if not job or not job.active or job.watch.is_set():
                raise Conflict("该次演奏已结束或已变更。")
            plan = self.continuation or job.plan
            if request_id != plan["request_id"] or not plan.get("piano_trial", {}).get("performance"):
                raise Conflict("该次演奏已结束或已变更。")
            if job.piano_exit_id != request_id and not (job.id == request_id and job.piano_exited):
                job.piano_exit_id = request_id
                self.hands.exit_score(request_id)
                self.preview = None
                self.revision += 1
            return dict(request_id=request_id)

    def start_motion(self, request_id, client, sessions=None):
        with self.lock:
            arms, current = self._feedback_view()
            if not self._can_prepare(arms, client):
                raise Conflict("设备未就绪或已有任务在运行。")
            plan, owner = self.preview or ({}, None)
            if client != owner or request_id != plan.get("request_id"):
                raise Conflict("预览已失效或已执行，请重新预览。")
            if "piano_trial" in plan:
                self._check_continuation_axes(plan)
            watch = self.job.watch if self.job and self.job.active else OperatorWatch()
            if plan.get("trial_fingers") and plan["trial_fingers"].sides:
                clock = Future()
                trial = plan["piano_trial"]
                piano_clock = PianoClock(plan, self.planner.endpoints) if trial["performance"] else None
                try:
                    self.hands.schedule_score(plan["trial_fingers"], sessions, client, watch,
                                               run_id=request_id, arm_clock=clock, piano_clock=piano_clock,
                                               title=trial.get("score_name"))
                except ValueError as exc:
                    # Hand availability never blocks the arm: run the wrist motion
                    # only and record why the fingers were skipped.
                    clock.cancel()
                    trial["finger_notice"] = str(exc)
                    self.journal.record("hand.score.skipped",
                                        f"指法未启动，仅执行机械臂：{exc}",
                                        level="WARN", request_id=request_id, important=True)
                else:
                    plan["trial_clock"] = clock
                    if piano_clock is not None:
                        plan["piano_clock"] = piano_clock
            try:
                self.journal.record("motion.accepted", "已提交执行，正在复核起点", request_id=request_id,
                                    important=True)
            except OSError:
                if "trial_clock" in plan:
                    plan.pop("trial_clock").cancel()
                raise
            self.preview = None
            self.revision += 1
            if self.job and self.job.active:
                self.continuation = plan
                return dict(Job(plan, client, self.job.watch).public(), session_id=self.job.session_id)
            self.job = Job(plan, client, watch)
            result = self.job.public()
        self.wake.set()
        return result

    def switch_mode(self, name, originals=None):
        with self.lock:
            if name not in DRIVE_MODES:
                raise ValueError("未知的电机模式。")
            if self.hardware and not self.mode_registers:
                raise Conflict("实机切换需要完整的 C02 盘点。")
            arms = self._feedback_view()[0]
            if not self._idle(arms) or not self._stable(arms):
                raise Conflict("请先停止，等待双臂静止及 14 轴失能回读后切换。")
            if self.hardware:
                if len(self.axis_modes) != 14:
                    raise Conflict("等待 14 轴模式回读。")
                current = grouped(self.axis_modes)
                if originals is not None and originals != current:
                    raise Conflict("模式已变化，请按最新读回重新确认。")
                self.expected_modes = self.axis_modes.copy()
                self.mode_request = name
                self.drive_mode = None
                self.wake.set()
            else:
                self.journal.record("drive.mode", f"离线驱动模式已切换至 {name}", important=True,
                                    data=dict(previous=self.drive_mode, drive_mode=name))
                self.drive_mode = name
            self.preview = None
            self.revision += 1
            return dict(drive_mode=self.drive_mode, pending_mode=self.mode_request, revision=self.revision)

    def _switch_hardware(self, name):
        confirmed = None
        error = None
        try:
            axes = self.mode_registers.switch(self.drivers, name, self.journal.record, self.closing,
                                              self.expected_modes)
            self.journal.record("drive.mode", f"14 轴 {name} 模式已读回，全部失能", important=True,
                                data=dict(drive_mode=name, axes=axes))
            confirmed = name
        except (DriverError, OSError, KeyError) as exc:
            error = str(exc)
            self.journal.record("drive.mode_failed", str(exc), level="ERROR", important=True)
        finally:
            with self.lock:
                self.drive_mode = confirmed
                self.mode_error = error
                self.axis_modes = dict.fromkeys(KEYS, MODE_CODES[name]) if confirmed else {}
                self.expected_modes = None
                self.mode_request = None
                self.feedback.clear()
                self.revision += 1

    def enable_all(self, client):
        with self.lock:
            arms, current = self._feedback_view()
            if not self._motion_supported() or not self._idle(arms):
                raise Conflict("全部使能需要服务空闲且 14 轴反馈齐全。")
            revision = self.revision
        plan = self.planner.enable_plan(current)
        with self.lock:
            arms, _ = self._feedback_view()
            if revision != self.revision or not self._motion_supported() or not self._idle(arms):
                raise Conflict("使能条件已变化，请重新操作。")
            self.journal.record("drive.enable_all", "已受理双臂 14 轴使能", request_id=plan["request_id"],
                                important=True, data=dict(drive_mode=self.drive_mode))
            self.preview = None
            self.revision += 1
            self.job = Job(plan, client, OperatorWatch())
            result = self.job.public()
        self.wake.set()
        return result

    def stop_motion(self, request_id):
        with self.lock:
            pending_id = self.continuation and self.continuation["request_id"]
            if (not self.job or not self.job.active or not request_id
                    or request_id not in (self.job.id, self.job.session_id, pending_id)):
                raise Conflict("该任务已结束或已变更。")
            # Signal first, even if the log volume is unavailable.
            self.job.watch.stop("操作员请求停止")
            self.preview = None
            self.revision += 1
            return self.job.public()

    def _set_side(self, side, rows, reason):
        with self.lock:
            previous = self.errors[side]
            self.errors[side] = reason
            if reason and self.hardware:
                for j in range(1, 8):
                    self.axis_modes.pop((side, j), None)
            if reason and self.hardware and self.drive_mode:
                self.drive_mode = None
                self.preview = None
                self.revision += 1
            for j in range(1, 8):
                self.feedback.pop((side, j), None)
            self.feedback.update(rows)
        if previous != reason:
            if reason is None:
                event, level = "feedback.online", "INFO"
            elif rows:
                event, level = "feedback.fault", "ERROR"
            else:
                event, level = "feedback.offline", "WARN"
            self.journal.record(event,
                                f"{'左臂' if side == 'left' else '右臂'} · {reason or '反馈在线'}",
                                level=level, data=dict(side=side), important=True)

    def _virtual_rows(self, target=None):
        velocity = {}
        states = set()
        if target:
            keys, row = target
            velocity = dict(zip(keys, row["target_velocity"]))
            self.positions.update(zip(keys, row["measured"]))
            states = set(keys)
        now = time.monotonic()
        return {key: dict(position_rad=self.mappings[key].to_motor(self.positions[key]),
                          velocity_rad_s=self.mappings[key].velocity_to_motor(velocity.get(key, 0.)),
                          state=int(key in states), received_at=now) for key in KEYS}

    def _close_driver(self, side):
        driver = self.drivers.pop(side, None)
        if driver is not None:
            try:
                driver.__exit__(None, None, None)
            except (DriverError, OSError) as exc:
                self.journal.record("feedback.close_error", str(exc), level="ERROR", important=True)

    def _idle_feedback(self):
        if not self.hardware:
            rows = self._virtual_rows()
            for side in SIDES:
                self._set_side(side, {k: v for k, v in rows.items() if k[0] == side}, None)
            return
        for side in SIDES:
            if time.monotonic() < self.reconnect_at[side]:
                continue
            try:
                if side not in self.drivers:
                    self.drivers[side] = self.driver_factory(side, self.channels[side], .1).__enter__()
                arm = self.drivers[side]
                batch = {(side, j): asdict(arm.query_motor(j)) for j in range(1, 8)}
                if arm.bus.status():
                    raise DriverError("CAN 总线状态异常")
                faults = [f"J{j} 状态 {row['state']}" for (_, j), row in batch.items()
                          if row["state"] not in (0, 1)]
                first_read = (side, 1) not in self.axis_modes
                if not faults and (first_read or time.monotonic() >= self.next_mode_read[side]):
                    modes = (self.mode_registers.read_arm_modes(side, arm) if self.mode_registers and first_read else
                             {(side, j): arm.read_parameter(j, 10) for j in range(1, 8)})
                    if any(code not in MODE_CODES.values() for code in modes.values()):
                        raise DriverError("电机模式读回无效。")
                    with self.lock:
                        if any(key in self.axis_modes and self.axis_modes[key] != code for key, code in modes.items()):
                            self.preview = None
                            self.revision += 1
                        self.axis_modes.update(modes)
                        if len(self.axis_modes) == 14 and not self.mode_request:
                            values = set(self.axis_modes.values())
                            self.drive_mode = next((n for n, c in MODE_CODES.items() if values == {c}), None)
                            if self.drive_mode is not None:
                                # A uniform 14-axis readback is the ground truth: a stale failure
                                # latch must not keep enable/preview blocked (P13 recovery path).
                                self.mode_error = None
                    self.next_mode_read[side] = time.monotonic() + 2
                self._set_side(side, batch, "、".join(faults) if faults else None)
            except (DriverError, OSError) as exc:
                self._close_driver(side)
                self._set_side(side, {}, str(exc))
                self.reconnect_at[side] = time.monotonic() + .75

    def _update_run(self, job, transport, row):
        kind = row["type"]
        if kind == "sample":
            keys = job.plan["keys"]
            sample = dict(elapsed_s=row["t"],
                          **{label: grouped({key: math.degrees(q) for key, q in zip(keys, row[field])})
                             for label, field in (("command_deg", "target"),
                                                  ("measured_deg", "measured"), ("error_deg", "error"))},
                          max_error_deg=math.degrees(max(map(abs, row["error"]))))
            if "wall_elapsed_s" in row:
                sample.update(sync_delay_s=max(0., row["wall_elapsed_s"] - row["t"]),
                              piano_sync=row.get("piano_sync"))
            if self.hardware:
                feedback = transport.feedback.copy()
            else:
                feedback = self._virtual_rows((keys, row))
                for key in keys:
                    feedback[key]["velocity_rad_s"] = transport.velocity[key]
            with self.lock:
                self.feedback = feedback
                job.sample = sample
            self.journal.record("motion.sample", request_id=job.id,
                                data=dict(**row, feedback=[dict(side=side, motor_id=j,
                                            **{k: v for k, v in feedback[side, j].items()
                                               if k not in ("motor_id", "online")})
                                            for side, j in KEYS if (side, j) in feedback]))
            return
        if kind == "state":
            if row["state"] == "EXECUTING" and "trial_clock" in job.plan:
                job.plan["trial_clock"].set_result(row["started_at"])
            with self.lock:
                job.state = row["state"]
                job.stop_confirmed = job.state == "STOPPED" and not self.hardware
                exited = job.state == "HOLDING_TARGET" and row["outcome"] == "INTERRUPTED"
                if exited:
                    job.plan["goal"].update(zip(job.plan["keys"], row["target"]))
                    job.piano_exit_id = None
                    job.piano_exited = True
            if row["state"] == "HOLDING_TARGET":
                event, message = (("motion.piano_exit", "已退出演奏，双臂保持当前位置") if exited else
                                  ("drive.holding", "已到位，关节持续使能保持位置"))
                self.journal.record(event, message,
                                    important=True, request_id=job.id, data=dict(drive_mode=self.drive_mode))
        if kind == "advisory":
            # Thresholds no longer stop the run; they are recorded for the operator.
            self.journal.record("motion.advisory", row["message"], level="WARN",
                                request_id=job.id, data=row)
            return
        if kind == "piano_sync":
            self.journal.record("piano.sync.wait" if row["waiting"] else "piano.sync.ready",
                                "等待抬指或腕位反馈" if row["waiting"] else "同步条件已满足",
                                request_id=job.id, data=row)
            return
        self.journal.record(f"motion.{kind}", row.get("state", row.get("error", "")),
                            request_id=job.id, data=row,
                            level="ERROR" if kind == "fault" or row.get("state") == "STOP_UNCONFIRMED" else "INFO")

    def _move(self, job, transport, current):
        # A new session starts at the measured pose. On continuation the planner
        # refreshes moving axes and retains the setpoints of axes still holding.
        trajectory, request = self.planner.revalidate(job.plan, current)
        if self.hardware:
            transport.start = current
            transport.validate_trajectory(trajectory)
        self.journal.record("motion.preflight", "执行前复核通过", request_id=job.id,
                            data=dict(trajectory=request, drive_mode=self.drive_mode, endpoint_tolerance_rad=ENDPOINT_TOLERANCE,
                                      period_s=PERIOD, max_tracking_error_rad=None,
                                      max_lateness_s=MAX_LATENESS,
                                      joints=[dict(side=key[0], motor_id=key[1], **settings)
                                              for key, settings in transport.joint_settings.items()]
                                      if self.hardware else []))
        def continuation():
            with self.lock:
                plan = self.continuation
                if not plan or job.watch.is_set():
                    return None
                self.continuation = None
                job.plan = plan
                job.piano_exited = False
                _, measured = self._feedback_view()
                if set(measured) != set(KEYS):
                    # Stale feedback is adopted instead of failing the continuation:
                    # use every cached sample regardless of age, then the plan start.
                    measured = {key: self.mappings[key].to_model(row["position_rad"])
                                for key, row in self.feedback.items() if key in KEYS}
                    missing = [key for key in KEYS if key not in measured]
                    measured.update({key: plan["start"].get(key, plan["goal"].get(key, 0.))
                                     for key in missing})
                    self.journal.record("motion.continuation_adopted",
                                        "接续时反馈不完整，已按缓存反馈与计划起点继续",
                                        level="WARN", request_id=job.id,
                                        data=dict(missing=[f"{side} J{j}" for side, j in missing]))
                upcoming, _ = self.planner.revalidate(plan, measured)
                if self.hardware:
                    transport.continue_from_hold(plan)
                else:
                    transport.speed = plan["velocity"]
                job.state = "EXECUTING"
                job.sample = None
                # The accepted continuation gets its own execution window,
                # just as a cold start does.
                return replace(upcoming, expires_at=time.time() + plan["duration"] + 10)

        def configure_cycle(track, elapsed):
            transport.speed_overrides = {key: speed for arm in getattr(track, "arms", ())
                                         if isinstance(arm, WristTrajectory)
                                         for key, speed in arm.speeds_at(elapsed).items()}

        return execute(trajectory, self.mappings, transport,
                       lambda row: self._update_run(job, transport, row), cancel=job.watch,
                       period=PERIOD, max_lateness=MAX_LATENESS, max_error=None,
                       endpoint_tolerance=ENDPOINT_TOLERANCE, hold_after=True,
                       next_trajectory=continuation, hold_requested=lambda: job.piano_exit_id == job.id,
                       piano_clock=lambda: job.plan.get("piano_clock"), configure_cycle=configure_cycle)

    def _hardware_run(self, job):
        def warn(message):
            self.journal.record("motion.advisory", message, level="WARN", request_id=job.id)
        transport = MonitorTransport(self.mappings, job.plan, self.channels, self.drivers, job.watch,
                                     self.drive_mode, self.drive_profile, warn=warn, mode_registers=self.mode_registers)
        with transport:
            current = transport.start_pose()
            try:
                return self._move(job, transport, current)
            finally:
                try:
                    self.journal.record("motion.disable_evidence", request_id=job.id,
                                        data=transport.stop_records)
                finally:
                    try:
                        reports = transport.require_disabled()
                    except (DriverError, OSError):
                        with self.lock:
                            self.feedback.clear()
                            job.state, job.stop_confirmed = "STOP_UNCONFIRMED", False
                        raise
                    else:
                        with self.lock:
                            self.feedback = {(arm["side"], row["motor_id"]): row
                                             for arm in reports for row in arm["motors"]}
                            job.stop_confirmed = True
                        self.journal.record("motion.after", "双臂失能回读已确认",
                                            request_id=job.id, data=reports)

    def _run_job(self, job):
        outcome, detail = "CANCELLED", ""
        try:
            if not job.watch.is_set():
                with self.lock:
                    job.state = "PREFLIGHT"
                if self.hardware:
                    outcome = self._hardware_run(job)
                else:
                    transport = VirtualDrive(self.mappings, self.positions, self.drive_mode, job.plan["velocity"])
                    outcome = self._move(job, transport, self.positions.copy())
        except Exception as exc:
            outcome, detail = "FAULT", str(exc)
        finally:
            with self.lock:
                if "trial_clock" in job.plan or "trial_clock" in (self.continuation or {}):
                    job.watch.stop("试弹执行已结束")
            if not self.hardware:
                with self.lock:
                    self.feedback = self._virtual_rows()
            with self.lock:
                self.continuation = None
                self.preview = None
                self.revision += 1
                if job.state == "STOP_UNCONFIRMED" and not job.stop_confirmed:
                    outcome = "STOP_UNCONFIRMED"
                if self.drive_profile and outcome in ("FAULT", "STOP_UNCONFIRMED"):
                    self.drive_mode = None
                    self.mode_error = "请重新选择模式并确认 14 轴回读。"
                label = {"COMPLETED": "已到位，失能回读确认" if self.hardware else "离线执行已到位",
                         "CANCELLED": "执行已停止" if job.stop_confirmed else "指令已取消",
                         "FAULT": "执行失败",
                         "STOP_UNCONFIRMED": "停止未确认，请使用现场急停或断电"}[outcome]
                if outcome == "CANCELLED" and job.watch.reason:
                    detail = job.watch.reason
                job.message = f"{label} · {detail}" if detail else label
                try:
                    self.journal.record(f"motion.{outcome.lower()}", job.message,
                                        request_id=job.id, important=True,
                                        level="INFO" if outcome == "COMPLETED" else
                                        "WARN" if outcome == "CANCELLED" else "ERROR")
                finally:
                    job.state, job.active = outcome, False

    def _run(self):
        try:
            while not self.closing.is_set():
                with self.lock:
                    job = self.job if self.job and self.job.active else None
                    mode_request = self.mode_request
                if mode_request:
                    self._switch_hardware(mode_request)
                elif job:
                    self._run_job(job)
                else:
                    self._idle_feedback()
                self.wake.wait(.15)
                self.wake.clear()
        except Exception as exc:
            with self.lock:
                self.feedback.clear()
                self.closing.set()
                self.errors = dict.fromkeys(SIDES, str(exc))
            if not self.journal.error:
                self.journal.record("service.fault", str(exc), level="ERROR", important=True)
        finally:
            for side in tuple(self.drivers):
                self._close_driver(side)


def create_server(monitor, port):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            self.request.settimeout(5)
            super().setup()

        def log_message(self, *_):
            pass

        @property
        def origin(self):
            return f"http://127.0.0.1:{self.server.server_port}"

        def _send(self, code, body, mime="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self._headers(code, mime, len(body), None)
            self._write(body)

        def _headers(self, code, mime, size, filename):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(size))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy",
                             "default-src 'self'; frame-ancestors 'none'; object-src 'none'; base-uri 'none'")
            if filename:
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.end_headers()

        def _write(self, data):
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return False
            return True

        def _download_log(self):
            path = monitor.journal.path
            with monitor.journal.lock:
                remaining = path.stat().st_size
            self._headers(200, "application/x-ndjson; charset=utf-8", remaining, path.name)
            with path.open("rb") as stream:
                while remaining:
                    chunk = stream.read(min(65536, remaining))
                    if not chunk or not self._write(chunk):
                        break
                    remaining -= len(chunk)

        def _host_ok(self):
            return self.headers.get("Host") == self.origin.removeprefix("http://")

        def _client(self):
            value = self.headers.get("X-OpenArm-Client", "")
            return value if len(value) == 32 and all(c in "0123456789abcdef" for c in value) else None

        def do_GET(self):
            if not self._host_ok():
                self._send(403, dict(error="仅支持本机 127.0.0.1 访问。"))
                return
            url = urlsplit(self.path)
            if url.path == "/api/status":
                try:
                    after = int(parse_qs(url.query).get("after", ["0"])[0])
                    if after < 0:
                        raise ValueError("日志游标无效。")
                    self._send(200, monitor.status(after, self._client()))
                except ValueError as exc:
                    self._send(400, dict(error=str(exc)))
                return
            if url.path == "/api/logs":
                self._download_log()
                return
            files = {"/": ("index.html", "text/html"), "/index.html": ("index.html", "text/html"),
                     "/app.js": ("app.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
            if url.path not in files:
                self._send(404, dict(error="请求的资源不存在。"))
                return
            name, mime = files[url.path]
            self._send(200, (UI / name).read_bytes(), mime + "; charset=utf-8")

        def do_POST(self):
            client = self._client()
            if not self._host_ok() or self.headers.get("Origin") != self.origin or not client:
                self._send(403, dict(error="指令只能从本机控制台页面提交。"))
                return
            request_id = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192 or self.headers.get_content_type() != "application/json":
                    raise ValueError("需要有效的 JSON 指令。")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("指令必须为 JSON 对象。")
                request_id = payload.get("request_id")
                request_id = request_id if isinstance(request_id, str) and len(request_id) == 32 else None
                if self.path == "/api/prepare":
                    result = monitor.prepare(payload, client)
                elif self.path == "/api/execute":
                    result = monitor.start_motion(request_id, client, payload.get("sessions"))
                elif self.path == "/api/piano/play":
                    result = monitor.play_piano(payload, client)
                elif self.path == "/api/piano/exit":
                    result = monitor.exit_piano(request_id)
                elif self.path == "/api/stop":
                    result = monitor.stop_motion(request_id)
                    monitor.hands.stop_active_score()
                elif self.path == "/api/hand":
                    result = monitor.hands.action(payload)
                elif self.path == "/api/hand-score/start":
                    result = monitor.hands.start_score(payload, client, OperatorWatch())
                elif self.path == "/api/hand-score/stop":
                    result = monitor.hands.stop_score(payload.get("request_id"))
                elif self.path == "/api/enable-all":
                    result = monitor.enable_all(client)
                elif self.path == "/api/start/zero":
                    result = monitor.start_pose_motion("zero-pose", client)
                elif self.path == "/api/start/return":
                    result = monitor.start_pose_motion("start-return", client)
                elif self.path == "/api/start/zero-return":
                    result = monitor.start_pose_motion("zero-return", client)
                elif self.path == "/api/start/pose":
                    result = monitor.start_pose_motion("start-pose", client)
                elif self.path == "/api/start/piano-origin":
                    result = monitor.start_pose_motion("piano-origin", client)
                elif self.path == "/api/piano/fingering":
                    result = fingering_preview(payload.get("scope"), payload.get("score_id", "jin-chords"),
                                               monitor.mappings)
                elif self.path == "/api/drive-mode":
                    result = monitor.switch_mode(payload.get("drive_mode"), payload.get("original_modes"))
                else:
                    self._send(404, dict(error="请求的接口不存在。"))
                    return
            except (Conflict, ValueError, OverflowError) as exc:
                try:
                    monitor.journal.record("motion.rejected", str(exc), request_id=request_id,
                                           level="WARN" if isinstance(exc, Conflict) else "ERROR",
                                           data=dict(route=self.path), important=True)
                except OSError:
                    pass
                self._send(409 if isinstance(exc, Conflict) else 400, dict(error=str(exc)))
            except OSError as exc:
                self._send(503, dict(error=str(exc)))
            else:
                self._send(200, result)

    class Server(ThreadingHTTPServer):
        def server_bind(self):
            if monitor.hardware and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                # Windows' SO_REUSEADDR allows two processes to listen on the same port.
                self.allow_reuse_address = False
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            super().server_bind()

    return Server(("127.0.0.1", port), Handler)


def open_running_monitor(port, mode):
    try:
        with closing(HTTPConnection("127.0.0.1", port, timeout=1)) as connection:
            connection.request("GET", "/api/status")
            response = connection.getresponse()
            if response.status != 200:
                return False
            status = json.load(response)
    except (OSError, HTTPException, ValueError):
        return False
    if (not isinstance(status, dict) or status.get("mode") != mode or
            not status.get("instance") or not status.get("mapping_version")):
        return False
    url = f"http://127.0.0.1:{port}/"
    print(f"OpenArm monitor ({mode}) is already running: {url}", flush=True)
    webbrowser.open(url)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("virtual", "hardware"), default="virtual")
    parser.add_argument("--port", type=int, help="Defaults to 8091 (virtual) or 8092 (hardware)")
    parser.add_argument("--left-channel", type=lambda s: int(s, 0), default=CHANNELS["left"])
    parser.add_argument("--right-channel", type=lambda s: int(s, 0), default=CHANNELS["right"])
    parser.add_argument("--open", action="store_true", help="Open the console in the default browser")
    parser.add_argument("--reuse-running", action="store_true",
                        help="With --open, reopen an existing console in the same mode")
    parser.add_argument("--drive-profile", type=Path, help="Commissioned hardware protocol profile tied to a C02 inventory")
    parser.add_argument("--mode-inventory", type=Path, help="C02 backup for disabled mode-register switching")
    parser.add_argument("--log-directory", type=Path, default=ROOT / "logs/openarm/monitor",
                        help="JSONL session directory (defaults to logs/openarm/monitor)")
    args = parser.parse_args()
    if args.left_channel == args.right_channel:
        parser.error("Use different CAN channels for the left and right arms")
    default_port = 8092 if args.mode == "hardware" else 8091
    port = args.port if args.port is not None else default_port
    reuse = args.open and args.reuse_running and port != 0
    if reuse and open_running_monitor(port, args.mode):
        return
    inventory = args.mode_inventory
    if args.mode == "hardware" and not inventory and not args.drive_profile:
        config = ROOT / "configs/openarm_position_mode.json"
        reference = json.loads(config.read_text(encoding="utf-8")).get("monitor_inventory")
        if reference and (config.parent / reference).is_file():
            inventory = config.parent / reference
    monitor = Monitor(args.mode, dict(left=args.left_channel, right=args.right_channel),
                      args.log_directory, profile_path=args.drive_profile, inventory_path=inventory)
    try:
        server = create_server(monitor, port)
    except OSError:
        monitor.journal.stream.close()
        # Another launch can bind the port between the initial check and this bind.
        if reuse and open_running_monitor(port, args.mode):
            return
        raise
    with server:
        monitor.start()
        try:
            url = f"http://127.0.0.1:{server.server_port}/"
            print(f"OpenArm monitor ({args.mode}): {url}", flush=True)
            print(f"JSONL: {monitor.journal.path}", flush=True)
            if args.open:
                webbrowser.open(url)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            monitor.close()


if __name__ == "__main__":
    main()
