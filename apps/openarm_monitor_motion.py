"""Model previews and PCAN transport for the local OpenArm monitor.

Laboratory profile (2026-11-19): previews no longer refuse a target because of a
velocity/acceleration percentage, a model-contact scan or a measured start drift,
and the hardware transport no longer aborts on feedback thresholds. Joint position
limits from the V1 model are still applied to the requested target. The removed
and retained guards are listed in
docs/project/OpenArm控制台防护项清单_20261119.md.
"""
from contextlib import nullcontext
from dataclasses import asdict
import math
import struct
from threading import Lock
import time
import uuid

import mujoco
import numpy as np

from openarm_app.kinematics import OpenArmIK
from openarm_app.kinematics.openarm_ik import IKConvergenceError
from drive_openarm_from_sim import VirtualTransport
from openarm_driver import DEFAULT_KD, DEFAULT_KP, DriverError
from openarm_execution import PcanTransport, rated_limits
from openarm_joint_mapping import CONVENTION_VERSION, finite
from openarm_monitor_modes import MODE_CODES, position_frame, velocity_frame
from openarm_trajectory import Trajectory

SIDES = ("left", "right")
KEYS = tuple((side, joint) for side in SIDES for joint in range(1, 8))
START_ERROR = .01  # Model rad, only used to size the per-axis travel allowance.
TRAVEL_MARGIN = .05  # Model rad beyond the planned displacement.
MAX_ACCELERATION = 1.  # Model rad/s², planning reference recorded in the request.
PREVIEW_TTL = 60.
FINGERS = {"thumb": "th", "index": "if", "middle": "mf", "ring": "rf", "little": "pf"}


def grouped(values, convert=lambda value: value):
    return {side: [convert(values.get((side, j))) for j in range(1, 8)] for side in SIDES}


class VirtualDrive(VirtualTransport):
    """Ideal unloaded axes, with mode-specific setpoints and motor-space speed limits."""
    def __init__(self, mappings, initial, mode, speed):
        super().__init__(mappings, initial)
        self.mode = mode
        self.speed = speed
        self.previous = time.monotonic()
        self.last_dispatch = None

    def command(self, targets):
        if self.mode == "MIT":
            return super().command(targets)
        if not self.enabled:
            raise RuntimeError("Transport disabled")
        now = time.monotonic()
        dt = max(now - self.previous, 1e-6)
        self.previous = now
        frames = {}
        for key, (p, v) in targets.items():
            mapping = self.mappings[key]
            limit = abs(mapping.velocity_to_motor(self.speed))
            current = self.position[key]
            if self.mode == "VEL":
                # Outer position loop on the host. The simulated drive receives velocity only.
                can_id, payload = velocity_frame(key[1], p - current, v, limit)
                step = struct.unpack("<f", payload)[0] * dt
            else:
                can_id, payload = position_frame(self.mode, mapping, p, self.speed, .1)
                if self.mode == "POS_VEL":
                    target, limit = struct.unpack("<ff", payload)
                else:
                    target, speed_code, _ = struct.unpack("<fHH", payload)
                    limit = speed_code / 100
                step = max(-limit * dt, min(limit * dt, target - current))
            self.position[key] = current + step
            self.velocity[key] = step / dt
            frames[f"{key[0]}:{key[1]}"] = [can_id, payload.hex()]
        self.last_dispatch = dict(drive_mode=self.mode, simulated_frames=frames)


class MotionPlanner:
    def __init__(self, version, mappings, *, virtual):
        self.version, self.mappings = version, mappings
        self.virtual = virtual
        right = OpenArmIK()
        self.arms = {"right": right, "left": OpenArmIK(side="left", model=right.setup.model)}
        for side, arm in self.arms.items():
            for j, limits in enumerate(arm.limits, 1):
                mapping = mappings[side, j]
                if (mapping.coordinate_convention != CONVENTION_VERSION
                        or not np.allclose(mapping.limits, limits, rtol=0, atol=1e-9)):
                    raise ValueError(f"V1 model and joint mapping disagree for {side} J{j}")
        self.home = {key: float(self.arms[key[0]].home[key[1] - 1]) for key in KEYS}
        if virtual:
            from paino_skill.scene import build_scene
            self.model, self.plan_data, _ = build_scene()
        else:
            self.model, self.plan_data = right.setup.model, mujoco.MjData(right.setup.model)
            self.plan_data.qpos[:] = right.setup.data.qpos
        self.feedback_data = mujoco.MjData(self.model)
        self.feedback_data.qpos[:] = self.plan_data.qpos
        self.qindices = {key: int(self.model.joint(mapping.joint_name).qposadr[0])
                         for key, mapping in mappings.items()}
        self.base_site = self.model.site("arm_origin").id
        self.wrists = {side: self.model.site(f"{side}_ee_control_point").id for side in SIDES}
        self.tips = {side: {finger: self.model.geom(f"aero_{side}_{short}_tip").id
                            for finger, short in FINGERS.items()} for side in SIDES} if virtual else {}
        self.plan_lock = Lock()
        self.feedback_lock = Lock()

    def endpoints(self, positions):
        with self.feedback_lock:
            return self._endpoints(self.feedback_data, positions)

    def _endpoints(self, data, positions):
        for key, q in positions.items():
            data.qpos[self.qindices[key]] = q
        mujoco.mj_kinematics(self.model, data)
        base = data.site_xpos[self.base_site]
        rotation = data.site_xmat[self.base_site].reshape(3, 3)
        result = {}
        for side in SIDES:
            if all((side, j) in positions for j in range(1, 8)):
                points = {"wrist": data.site_xpos[self.wrists[side]]}
                points.update({name: data.geom_xpos[geom] for name, geom in self.tips.get(side, {}).items()})
                result[side] = {name: (1000 * rotation.T @ (point - base)).tolist()
                                for name, point in points.items()}
        return result

    def _ik(self, side, current, endpoint, target):
        if endpoint != "wrist" and endpoint not in self.tips.get(side, {}):
            raise ValueError("实机 IK 使用手腕控制点。指尖需要实测的手指姿态和安装标定。")
        if not isinstance(target, list) or len(target) != 3:
            raise ValueError("每只臂需要填写 X、Y、Z 三个坐标。")
        desired = np.asarray([finite(coord) for coord in target]) / 1000
        seed = [current[side, j] for j in range(1, 8)]
        arm = self.arms[side]
        initial = self._endpoints(self.plan_data, current)[side]
        # Hold the wrist orientation. In the virtual scene the Aero fingers
        # stay in a fixed posture, giving a rigid offset from the wrist.
        pose = arm.forward(seed)
        pose[:3] += desired - np.asarray(initial[endpoint]) / 1000
        try:
            q = arm.inverse(pose, seed, max_iterations=120, position_tolerance=.001,
                            orientation_tolerance=.003)
        except IKConvergenceError as exc:
            raise ValueError(f"{'左臂' if side == 'left' else '右臂'} IK 无解或目标不可达。{exc}") from exc
        values = {(side, j): float(value) for j, value in enumerate(q, 1)}
        return values

    def prepare(self, payload, current, *, holding=None):
        sides = SIDES if payload.get("arm") == "both" else (payload.get("arm"),)
        mode, targets = payload.get("mode"), payload.get("targets")
        if (any(s not in SIDES for s in sides) or mode not in ("joint", "ik")
                or not isinstance(targets, dict) or set(targets) != set(sides)):
            raise ValueError("控制模式、控制臂或目标格式无效。")
        duration, ratio = finite(payload.get("duration_s")), finite(payload.get("speed_percent"))
        if not .05 <= duration <= 600 or not 1 <= ratio <= 100:
            raise ValueError("运动时间须为 0.05–600 秒，速度上限须为 1–100%。")
        endpoint = payload.get("endpoint", "wrist")
        if not isinstance(endpoint, str):
            raise ValueError("末端类型无效。")
        target = {}
        with self.plan_lock:
            for side in sides:
                if mode == "ik":
                    target.update(self._ik(side, current, endpoint, targets[side]))
                else:
                    values = targets[side]
                    if not isinstance(values, list) or len(values) != 7:
                        raise ValueError("每只臂需要七个关节输入，留空跳过该轴。")
                    target.update({(side, j): math.radians(finite(q))
                                   for j, q in enumerate(values, 1) if q is not None})
            self._check_pose(target)
            goal = dict(target)
            if not goal and holding is None:
                raise ValueError("请至少填写一个有位移的目标。")
            commanded = tuple(key for key in KEYS if key in goal)
            notice = None
            if holding is not None:
                if not set(commanded).issubset(holding):
                    notice = "目标包含仍在失能的轴，该轴不会被使能。"
                goal = holding | goal
            keys = tuple(key for key in KEYS if key in goal)
            speed = min(rated_limits(key[1])["max_speed"] * self.mappings[key].scale
                        for key in keys) * ratio / 100
            plan = dict(request_id=uuid.uuid4().hex, start=dict(current), goal=goal, keys=keys,
                        duration=duration, velocity=speed, deadline=time.monotonic()+PREVIEW_TTL)
            plan["commanded"] = commanded
            plan["notice"] = notice
            plan["trajectory"], request = self._validate(plan, current)
        return plan, request

    def enable_plan(self, current):
        # A stationary trajectory takes over at measured angles, including nonzero axes.
        plan = dict(request_id=uuid.uuid4().hex, start=dict(current), goal=dict(current), keys=KEYS,
                    duration=.04, velocity=.1, deadline=time.monotonic()+PREVIEW_TTL, enable_all=True)
        plan["commanded"] = KEYS
        plan["notice"] = None
        with self.plan_lock:
            plan["trajectory"], request = self._validate(plan, current)
        return plan

    def revalidate(self, plan, current):
        """Rebuild from the measured pose. A drifted start is adopted, not rejected."""
        with self.plan_lock:
            plan["start"] = dict(current)
            if plan.get("enable_all"):
                plan["goal"] = dict(current)
            trajectory, request = self._validate(plan, current)
        return trajectory, request

    def _check_pose(self, positions):
        for key, q in positions.items():
            lo, hi = self.mappings[key].limits
            if not lo <= finite(q) <= hi:
                side, j = key
                raise ValueError(f"{'左臂' if side == 'left' else '右臂'} J{j} 超出关节限位 "
                                 f"[{math.degrees(lo):.2f}, {math.degrees(hi):.2f}]°。")

    def _validate(self, plan, start):
        goal, keys, duration = plan["goal"], plan["keys"], plan["duration"]
        self._check_pose(goal)
        finish = start | goal
        peak = max((abs(finish[key]-start[key]) for key in keys), default=0.)
        # A short requested time is allowed to need more speed or acceleration than
        # the rated percentage. The requirement replaces the cap instead of failing.
        velocity = max(plan["velocity"], 1.875*peak/duration*1.000001)
        acceleration = max(MAX_ACCELERATION, (10/math.sqrt(3))*peak/duration**2*1.000001)
        plan["velocity"], plan["acceleration"] = velocity, acceleration
        points = ((0., tuple(start[key] for key in keys)),
                  (duration, tuple(finish[key] for key in keys)))
        request = dict(request_id=plan["request_id"], mapping_version=self.version,
                       expires_at=time.time()+duration+10, source="monitor-ui",
                       joints=[dict(side=side, motor_id=j) for side, j in keys],
                       initial_model_position=[start[key] for key in keys],
                       points=[dict(t=t, q=list(q)) for t, q in points],
                       limits=dict(velocity_rad_s=velocity, acceleration_rad_s2=acceleration,
                                   start_error_rad=0.))
        trajectory = Trajectory(plan["request_id"], keys, points, request["expires_at"])
        plan["trajectory"] = trajectory
        return trajectory, request


class MonitorTransport(PcanTransport):
    """Reuse the already-owned CAN handles and keep observing passive axes."""
    def __init__(self, mappings, plan, channels, arms, cancel, mode="MIT", profile=None, warn=None):
        keys, start = plan["keys"], plan["start"]
        travel = {key: (abs(plan["goal"][key] - start[key]) + START_ERROR + TRAVEL_MARGIN)
                       / mappings[key].scale for key in keys}
        settings = {key: dict(kp=DEFAULT_KP[key[1]-1], kd=DEFAULT_KD[key[1]-1],
                              max_travel=travel[key], max_temperature=50., max_age=.15,
                              **rated_limits(key[1])) for key in keys}
        super().__init__(mappings, keys, channels, joint_settings=settings, batch_mit=True,
                         driver_factory=lambda side, *_: nullcontext(arms[side]), warn=warn)
        self.start = start
        self.cancel = cancel
        self.profile = profile
        self.mode, self.mode_code = mode, MODE_CODES[mode]
        if not profile and mode != "MIT":
            raise DriverError("实机模式缺少已核验的驱动配置。")
        self.speed = plan["velocity"]
        self.dispatch_mode = f"batch_{mode.lower()}_send_then_query"
        self.passive_keys = [key for key in KEYS if key not in keys]
        self.passive_index = 0
        self.feedback = {}

    def _sample(self, reports):
        """Take the measured pose as the start. Stationary and stale checks are gone."""
        self.feedback = {(report["side"], row["motor_id"]): row
                         for report in reports for row in report["motors"]}
        return {key: self.mappings[key].to_model(row["position_rad"])
                for key, row in self.feedback.items()}

    def start_pose(self):
        if self.profile:
            if any(code != self.mode_code for code in self.profile.read_modes(self.arms).values()):
                raise DriverError("14 轴实际模式与控制台选择不一致。")
        return self._sample(self.snapshot_arms())

    def continue_from_hold(self, plan):
        self.speed = plan["velocity"]
        for key in self.keys:
            origin = self.states[key].position_rad
            self.origins[key] = origin
            goal = self.mappings[key].to_motor(plan["goal"][key])
            self.joint_settings[key]["max_travel"] = (
                abs(goal - origin) + (START_ERROR + TRAVEL_MARGIN) / self.mappings[key].scale)
        self.validate_trajectory(plan["trajectory"])

    def enable(self):
        super().enable(cancel=self.cancel)

    def _command_frame(self, key, p, v):
        if self.mode == "MIT":
            return super()._command_frame(key, p, v)
        mapping = self.mappings[key]
        if self.mode == "VEL":
            return velocity_frame(key[1], p - self.states[key].position_rad, v,
                                  abs(mapping.velocity_to_motor(self.speed)))
        return position_frame(self.mode, mapping, p, self.speed, self.profile.current)

    def _enable_axis(self, key, state):
        if self.mode == "MIT":
            return super()._enable_axis(key, state)
        arm, j = self.arms[key[0]], key[1]
        self.states[key] = state
        first = self._command_frame(key, state.position_rad, 0.)
        arm.bus.send(*first)
        arm.bus.send(self.profile.special_id(key, self.mode), b"\xff" * 7 + b"\xfc")
        arm.bus.send(*first)
        return arm.wait_motor_state(j, 1)

    def _disable_axis(self, key):
        if self.mode == "MIT":
            return super()._disable_axis(key)
        arm = self.arms[key[0]]
        arm.bus.send(self.profile.special_id(key, self.mode), b"\xff" * 7 + b"\xfd")
        return arm.wait_motor_state(key[1], 0)

    def command(self, targets):
        super().command(targets)
        self.feedback.update({key: asdict(self.states[key]) for key in self.keys})
        for _ in range(min(2, len(self.passive_keys))):
            key = self.passive_keys[self.passive_index]
            state = self.arms[key[0]].query_motor(key[1])
            self.feedback[key] = asdict(state)
            q = self.mappings[key].to_model(state.position_rad)
            if state.state != 0 or abs(q-self.start[key]) > START_ERROR:
                self._note('passive', f"{key[0]} J{key[1]} 非指令轴状态 {state.state}，"
                                      f"相对起点 {q-self.start[key]:+.4f} rad", key)
            self.passive_index = (self.passive_index+1) % len(self.passive_keys)
