"""Loopback-only OpenArm console. A single worker owns the selected transport."""
import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid
import webbrowser

from drive_openarm_from_sim import execute
from openarm_driver import DriverError, OpenArmDriver
from openarm_joint_mapping import load_mapping
from openarm_monitor_motion import KEYS, SIDES, MonitorTransport, MotionPlanner, VirtualDrive, grouped
from openarm_monitor_modes import DRIVE_MODES, DriveProfile

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "monitor-ui"
CHANNELS = {"left": 0x51, "right": 0x52}
FEEDBACK_AGE = .75  # UI cache limit. Execution uses its own stricter limits.
HEARTBEAT_TIMEOUT = 3.
PERIOD = .04
ENDPOINT_TOLERANCE = .015
MAX_LATENESS = .08


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
                    sample=self.sample)


class Conflict(RuntimeError):
    pass


class Monitor:
    def __init__(self, mode, channels, log_directory, *, driver_factory=OpenArmDriver, profile_path=None):
        self.mode, self.channels = mode, channels
        self.hardware = mode == "hardware"
        self.driver_factory = driver_factory
        self.version, self.mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json",
                                                  hardware=self.hardware)
        self.planner = MotionPlanner(self.version, self.mappings, virtual=not self.hardware)
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
        if profile_path and not self.hardware:
            raise ValueError("实机协议配置仅用于 hardware 服务。")
        self.drive_profile = DriveProfile(profile_path, self.version, channels) if profile_path else None
        self.drive_mode = None if self.drive_profile else "MIT"
        self.mode_request = None
        self.revision = 0
        self.continuation = None
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
        if not self.drive_mode:
            return False
        if self._idle(arms):
            return True
        job = self.job
        if (not job or not job.active or job.state != "HOLDING_TARGET" or job.client != client
                or job.watch.is_set() or self.continuation or self.journal.error):
            return False
        return all(arm["fresh"] for arm in arms.values())

    def status(self, after, client):
        with self.lock:
            if self.job and self.job.active and self.job.client == client:
                self.job.watch.touch()
            arms, positions = self._feedback_view()
            ready = self._can_prepare(arms, client)
            idle, stable = self._idle(arms), self._stable(arms)
            drive_mode, revision = self.drive_mode, self.revision
            pending_mode = self.mode_request
            job = self.job.public() if self.job else None
        points = self.planner.endpoints(positions)
        for side, arm in arms.items():
            arm["endpoints_mm"] = points.get(side, {}) if not arm["reason"] else {}
        return dict(instance=self.journal.id, mode=self.mode, mapping_version=self.version,
                    limits_deg=self.limits_deg, arms=arms, can_prepare=ready, run=job,
                    drive_mode=drive_mode, revision=revision, can_enable_all=idle and bool(drive_mode),
                    pending_mode=pending_mode,
                    can_switch_mode=idle and stable and (not self.hardware or bool(self.drive_profile)),
                    drive_modes=("MIT",) if self.hardware and not self.drive_profile else DRIVE_MODES,
                    log_error=self.journal.error, events=self.journal.since(after))

    def prepare(self, payload, client):
        with self.lock:
            arms, current = self._feedback_view()
            if not self._can_prepare(arms, client):
                raise Conflict("当前模式或反馈状态不允许预览位置目标。")
            revision = self.revision
            holding = self.job.plan["goal"] if self.job and self.job.active else None
            if self.preview and self.preview[1] == client:
                self.preview = None
        plan, request = self.planner.prepare(payload, current, holding=holding)
        with self.lock:
            arms, _ = self._feedback_view()
            if revision != self.revision or not self._can_prepare(arms, client):
                raise Conflict("控制台状态已变化，请重新预览。")
            self.journal.record("motion.preview", "目标已通过预检", request_id=plan["request_id"],
                                data=dict(input=payload, trajectory=request, notice=plan.get("notice")),
                                important=True)
            if plan.get("notice"):
                self.journal.record("motion.notice", plan["notice"], level="WARN",
                                    request_id=plan["request_id"])
            self.preview = plan, client
        motion = request["points"]
        peak = max(abs(a - b) for a, b in zip(motion[0]["q"], motion[1]["q"]))
        return dict(request_id=plan["request_id"], duration_s=plan["duration"],
                    expires_in_s=round(plan["deadline"] - time.monotonic()),
                    revision=revision, command_joints=[f"{side} J{j}" for side, j in plan["commanded"]],
                    target_deg=grouped(current | plan["goal"], math.degrees),
                    peak_speed_deg_s=math.degrees(1.875 * peak / plan["duration"]))

    def start_motion(self, request_id, client):
        with self.lock:
            arms, current = self._feedback_view()
            if not self._can_prepare(arms, client):
                raise Conflict("设备未就绪或已有任务在运行。")
            plan, owner = self.preview or ({}, None)
            if (client != owner or request_id != plan.get("request_id")
                    or time.monotonic() >= plan["deadline"]):
                raise Conflict("预览已失效或已执行，请重新预览。")
            self.journal.record("motion.accepted", "已提交执行，正在复核起点", request_id=request_id,
                                important=True)
            self.preview = None
            self.revision += 1
            if self.job and self.job.active:
                self.continuation = plan
                return dict(Job(plan, client, self.job.watch).public(), session_id=self.job.session_id)
            self.job = Job(plan, client, OperatorWatch())
            result = self.job.public()
        self.wake.set()
        return result

    def switch_mode(self, name):
        with self.lock:
            if name not in DRIVE_MODES:
                raise ValueError("未知的电机模式。")
            if self.hardware and not self.drive_profile:
                raise Conflict("实机驱动目前使用 MIT。其他模式须先核实各轴固件、首目标和停止寻址。")
            arms = self._feedback_view()[0]
            if not self._idle(arms) or not self._stable(arms):
                raise Conflict("请先停止，等待双臂静止及 14 轴失能回读后切换。")
            if self.hardware:
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
        try:
            axes = self.drive_profile.switch(self.drivers, name, self.journal.record, self.closing)
            self.journal.record("drive.mode", f"14 轴 {name} 模式已读回，全部失能", important=True,
                                data=dict(drive_mode=name, axes=axes))
            confirmed = name
        except (DriverError, OSError, KeyError) as exc:
            self.journal.record("drive.mode_failed", str(exc), level="ERROR", important=True)
        finally:
            with self.lock:
                self.drive_mode = confirmed
                self.mode_request = None
                self.feedback.clear()
                self.revision += 1

    def enable_all(self, client):
        with self.lock:
            arms, current = self._feedback_view()
            if not self.drive_mode or not self._idle(arms):
                raise Conflict("全部使能需要服务空闲且 14 轴反馈齐全。")
            revision = self.revision
        plan = self.planner.enable_plan(current)
        with self.lock:
            arms, _ = self._feedback_view()
            if revision != self.revision or not self._idle(arms):
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
            if reason and self.drive_profile and self.drive_mode:
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
            with self.lock:
                job.state = row["state"]
                job.stop_confirmed = job.state == "STOPPED" and not self.hardware
            if row["state"] == "HOLDING_TARGET":
                self.journal.record("drive.holding", "已到位，关节持续使能保持位置",
                                    important=True, request_id=job.id, data=dict(drive_mode=self.drive_mode))
        if kind == "advisory":
            # Thresholds no longer stop the run; they are recorded for the operator.
            self.journal.record("motion.advisory", row["message"], level="WARN",
                                request_id=job.id, data=row)
            return
        self.journal.record(f"motion.{kind}", row.get("state", row.get("error", "")),
                            request_id=job.id, data=row,
                            level="ERROR" if kind == "fault" or row.get("state") == "STOP_UNCONFIRMED" else "INFO")

    def _move(self, job, transport, current):
        # The accepted target is re-planned from the measured pose. A drift between
        # preview and execution is adopted instead of invalidating the request.
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
                _, measured = self._feedback_view()
                if set(measured) != set(KEYS):
                    raise DriverError("接续执行时反馈已过期。")
                upcoming, _ = self.planner.revalidate(plan, measured)
                if self.hardware:
                    transport.continue_from_hold(plan)
                else:
                    transport.speed = plan["velocity"]
                job.state = "EXECUTING"
                job.sample = None
                # The 60 s preview lease was checked on submission. Give the accepted
                # trajectory its own execution window, just as a cold start does.
                return replace(upcoming, expires_at=time.time() + plan["duration"] + 10)

        return execute(trajectory, self.mappings, transport,
                       lambda row: self._update_run(job, transport, row), cancel=job.watch,
                       period=PERIOD, max_lateness=MAX_LATENESS, max_error=None,
                       endpoint_tolerance=ENDPOINT_TOLERANCE, hold_after=True,
                       next_trajectory=continuation)

    def _hardware_run(self, job):
        def warn(message):
            self.journal.record("motion.advisory", message, level="WARN", request_id=job.id)
        transport = MonitorTransport(self.mappings, job.plan, self.channels, self.drivers, job.watch,
                                     self.drive_mode, self.drive_profile, warn=warn)
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
                    result = monitor.start_motion(request_id, client)
                elif self.path == "/api/stop":
                    result = monitor.stop_motion(request_id)
                elif self.path == "/api/enable-all":
                    result = monitor.enable_all(client)
                elif self.path == "/api/drive-mode":
                    result = monitor.switch_mode(payload.get("drive_mode"))
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

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("virtual", "hardware"), default="virtual")
    parser.add_argument("--port", type=int, help="Defaults to 8091 (virtual) or 8092 (hardware)")
    parser.add_argument("--left-channel", type=lambda s: int(s, 0), default=CHANNELS["left"])
    parser.add_argument("--right-channel", type=lambda s: int(s, 0), default=CHANNELS["right"])
    parser.add_argument("--open", action="store_true", help="Open the console in the default browser")
    parser.add_argument("--drive-profile", type=Path, help="Commissioned hardware protocol profile tied to a C02 inventory")
    args = parser.parse_args()
    if args.left_channel == args.right_channel:
        parser.error("Use different CAN channels for the left and right arms")
    monitor = Monitor(args.mode, dict(left=args.left_channel, right=args.right_channel),
                      ROOT / "logs/openarm/monitor", profile_path=args.drive_profile)
    default_port = 8092 if args.mode == "hardware" else 8091
    port = args.port if args.port is not None else default_port
    with create_server(monitor, port) as server:
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
