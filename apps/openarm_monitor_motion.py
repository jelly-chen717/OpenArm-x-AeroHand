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
from openarm_driver import DEFAULT_KD, DEFAULT_KP, DriverError, LIMITS, motor_type
from openarm_execution import PcanTransport, rated_limits
from openarm_joint_mapping import CONVENTION_VERSION, LIMIT_TOLERANCE, finite
from openarm_monitor_modes import MODE_CODES, position_frame, velocity_frame
from openarm_start_pose import return_start_steps, start_pose_step, zero_step
from openarm_piano_air import attacks, compile_air
from openarm_trajectory import Trajectory

SIDES = ("left", "right")
KEYS = tuple((side, joint) for side in SIDES for joint in range(1, 8))
START_ERROR = .01  # Model rad, only used to size the per-axis travel allowance.
TRAVEL_MARGIN = .05  # Model rad beyond the planned displacement.
MAX_ACCELERATION = 1.  # Model rad/s², planning reference recorded in the request.
POS_FORCE_CURRENT_FRACTION = .1  # Per-unit current limit used without a commissioned profile.
FINGERS = {"thumb": "th", "index": "if", "middle": "mf", "ring": "rf", "little": "pf"}
# Fixed interior samples explore other joint configurations on every repeatable preview.
IK_SEED_FRACTIONS = np.random.default_rng(0).uniform(.1, .9, (8, 7))


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
        # Keep multi-start solves off the lock used by execution revalidation.
        self.kinematics_lock = Lock()
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
        desired_mm = np.asarray([finite(coord) for coord in target])
        keys = [(side, j) for j in range(1, 8)]
        measured = {key: current[key] for key in keys}
        self._check_pose(measured)
        reference = np.asarray(list(measured.values()))
        arm = self.arms[side]
        lower, upper = arm.limits.T
        # Project encoder roundoff at a hard stop using the monitor's existing limit check.
        seed = np.clip(reference, lower, upper)
        pose = arm.forward(seed)
        pose[:3] = desired_mm / 1000
        if endpoint != "wrist":
            # The fixed virtual fingers keep their offset as the wrist translates.
            model_current = current | dict(zip(keys, seed))
            initial = self._endpoints(self.plan_data, model_current)[side]
            pose[:3] += (np.asarray(initial["wrist"]) - initial[endpoint]) / 1000

        seeds = [seed, arm.home, (lower + upper) / 2]
        swivel = np.array([0, 0, 1, 0, -1, 0, 0])
        seeds.extend(seed + swivel * radians for radians in np.deg2rad([-45, -20, 20, 45]))
        seeds.extend(lower + IK_SEED_FRACTIONS * (upper - lower))
        tried, candidates = set(), []
        for start in seeds:
            start = np.clip(start, lower, upper)
            angles = tuple(start)
            if angles in tried:
                continue
            tried.add(angles)
            try:
                q = arm.inverse(pose, start, max_iterations=120, position_tolerance=.001,
                                orientation_tolerance=.003)
            except IKConvergenceError:
                continue
            candidates.append(q)
            if np.array_equal(q, seed):
                break  # The closest possible in-limit posture already meets the target.
        if not candidates:
            name = "左臂" if side == "left" else "右臂"
            xyz = ", ".join(f"{value:.2f}" for value in desired_mm)
            raise ValueError(f"{name} IK 尝试了 {len(tried)} 个初始姿态，保持当前手腕朝向时未找到收敛解。"
                             f"目标 XYZ 为 ({xyz}) mm。")
        q = min(candidates, key=lambda candidate: np.linalg.norm(candidate - reference))
        return dict(zip(keys, map(float, q)))

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
        if mode == "ik":
            with self.kinematics_lock:
                for side in sides:
                    target.update(self._ik(side, current, endpoint, targets[side]))
        else:
            for side in sides:
                values = targets[side]
                if not isinstance(values, list) or len(values) != 7:
                    raise ValueError("每只臂需要七个关节输入，留空跳过该轴。")
                target.update({(side, j): math.radians(finite(q))
                               for j, q in enumerate(values, 1) if q is not None})
        with self.plan_lock:
            self._check_pose(target)
            goal = dict(target)
            if not goal and holding is None:
                raise ValueError("请至少填写一个有位移的目标。")
            commanded = tuple(key for key in KEYS if key in goal)
            notice = None
            hold_targets = {}
            if holding is not None:
                if not set(commanded).issubset(holding):
                    notice = "目标包含仍在失能的轴，该轴不会被使能。"
                goal = holding | goal
                hold_targets = {key: previous for key, previous in holding.items()
                                if goal[key] == previous}
            keys = tuple(key for key in KEYS if key in goal)
            speed = min(rated_limits(key[1])["max_speed"] * self.mappings[key].scale
                        for key in keys) * ratio / 100
            plan = dict(request_id=uuid.uuid4().hex, goal=goal, keys=keys,
                        duration=duration, velocity=speed, hold_targets=hold_targets)
            plan["commanded"] = commanded
            plan["notice"] = notice
            plan["trajectory"], request = self._validate(plan, current)
        return plan, request

    def enable_plan(self, current):
        # A stationary trajectory takes over at measured angles, including nonzero axes.
        plan = dict(request_id=uuid.uuid4().hex, goal=dict(current), keys=KEYS,
                    duration=.04, velocity=.1, enable_all=True)
        plan["commanded"] = KEYS
        plan["notice"] = None
        with self.plan_lock:
            plan["trajectory"], request = self._validate(plan, current)
        return plan

    def revalidate(self, plan, current):
        """Refresh moving axes from feedback while keeping unchanged targets continuous."""
        with self.plan_lock:
            if plan.get("enable_all"):
                plan["goal"] = dict(current)
            trajectory, request = self._validate(plan, current)
        return trajectory, request

    def _check_pose(self, positions):
        # The measured start pose is checked too; a joint on its hard stop reads
        # a step below the limit, which LIMIT_TOLERANCE absorbs. Targets further
        # outside the model range are still refused.
        for key, q in positions.items():
            q, (lo, hi) = finite(q), self.mappings[key].limits
            if not lo - LIMIT_TOLERANCE <= q <= hi + LIMIT_TOLERANCE:
                side, j = key
                raise ValueError(f"{'左臂' if side == 'left' else '右臂'} J{j} 超出关节限位 "
                                 f"[{math.degrees(lo):.2f}, {math.degrees(hi):.2f}]°，"
                                 f"本次 {math.degrees(q):+.3f}°。")

    def _validate(self, plan, current):
        goal, keys = plan["goal"], plan["keys"]
        # Rebasing a loaded holding axis on its encoder would release its position error.
        start = current | plan.get("hold_targets", {})
        plan["start"] = start
        steps = plan.get("steps")
        if steps:
            points = [(0., tuple(start[key] for key in keys))]
            elapsed = 0.
            for _, seconds, target in steps:
                elapsed += seconds
                points.append((elapsed, tuple(target.get(key, start[key]) for key in keys)))
        else:
            finish = start | goal
            points = [(0., tuple(start[key] for key in keys)),
                      (plan["duration"], tuple(finish[key] for key in keys))]
        for _, point in points:
            self._check_pose(dict(zip(keys, point)))
        # Every segment is rest-to-rest. The plan takes the largest requirement
        # of any segment, so a short requested time is never refused here.
        velocity, acceleration = plan["velocity"], MAX_ACCELERATION
        for (t0, q0), (t1, q1) in zip(points, points[1:]):
            span = t1-t0
            peak = max((abs(b-a) for a, b in zip(q0, q1)), default=0.)
            velocity = max(velocity, 1.875*peak/span*1.000001)
            acceleration = max(acceleration, (10/math.sqrt(3))*peak/span**2*1.000001)
        plan["velocity"], plan["acceleration"] = velocity, acceleration
        plan["duration"] = points[-1][0]
        request = dict(request_id=plan["request_id"], mapping_version=self.version,
                       expires_at=time.time()+plan["duration"]+10, source="monitor-ui",
                       joints=[dict(side=side, motor_id=j) for side, j in keys],
                       initial_model_position=[start[key] for key in keys],
                       points=[dict(t=t, q=list(q)) for t, q in points],
                       limits=dict(velocity_rad_s=velocity, acceleration_rad_s2=acceleration,
                                   start_error_rad=0.))
        trajectory = Trajectory(plan["request_id"], keys, points, request["expires_at"])
        plan["trajectory"] = trajectory
        plan["travel"] = {key: max(abs(point[index]-start[key]) for _, point in points)
                          for index, key in enumerate(keys)}
        return trajectory, request

    def _prepare_steps(self, steps, current, *, holding, source):
        """Plan one or more absolute waypoints, then hold the final pose."""
        with self.plan_lock:
            full_steps = []
            for step in steps:
                target = {}
                for side, degrees in (("right", step.right_deg), ("left", step.left_deg)):
                    target.update({(side, j): math.radians(q)
                                   for j, q in enumerate(degrees, 1) if q is not None})
                full_steps.append((step.name, step.duration_s, target))
            goal = dict(full_steps[-1][2])
            if holding is not None:
                goal = dict(holding) | goal
            keys = tuple(key for key in KEYS if key in goal
                         or any(key in target for _, _, target in full_steps))
            commanded = tuple(key for key in keys
                              if any(key in target for _, _, target in full_steps))
            notice = None
            if holding is not None and not set(commanded).issubset(holding):
                notice = "目标包含仍在失能的轴，该轴不会被使能。"
            hold_targets = {}
            if holding is not None:
                hold_targets = {key: previous for key, previous in holding.items()
                                if goal[key] == previous}
            # Give every waypoint the full commanded set. Held axes stay at their
            # previous setpoint in every segment and are never released.
            full_steps = [(name, seconds, dict(holding or {}) | target)
                          for name, seconds, target in full_steps]
            speed = min(rated_limits(key[1])["max_speed"] * self.mappings[key].scale
                        for key in keys) * .2
            plan = dict(request_id=uuid.uuid4().hex, goal=goal, keys=keys,
                        duration=sum(seconds for _, seconds, _ in full_steps),
                        velocity=speed, hold_targets=hold_targets, steps=full_steps,
                        commanded=commanded, notice=notice, source=source)
            plan["trajectory"], request = self._validate(plan, current)
        return plan, request

    def prepare_start_return(self, current, *, holding=None):
        """Both arms: zero, J3 +/-80, J3 +/-60 with J4 +60, then the start pose."""
        return self._prepare_steps(return_start_steps(), current, holding=holding,
                                   source="start-return")

    def prepare_start_pose(self, current, *, holding=None):
        """Shortcut: both arms straight to J4 +90 and J3 0 without the approach."""
        return self._prepare_steps((start_pose_step(),), current, holding=holding,
                                   source="start-pose")

    def prepare_zero_pose(self, current, *, holding=None):
        """Shortcut: both arms straight to J1..J7 = 0 without the start approach."""
        return self._prepare_steps((zero_step(),), current, holding=holding,
                                   source="zero-pose")

    def prepare_piano_trial(self, payload, current, held_plan):
        from openarm_piano_trial import compile_trial
        steps, info, fingers = compile_trial(payload, held_plan)
        holding = held_plan["goal"] if held_plan else None
        plan, request = self._prepare_steps(steps, current, holding=holding, source="piano-trial")
        plan.update(piano_trial=info, trial_fingers=fingers)
        return plan, request

    def prepare_piano_origin(self, current, *, holding):
        from openarm_piano_origin import origin_step
        return self._prepare_steps((origin_step(),), current, holding=holding, source="piano-origin")

    def prepare_piano_air(self, payload, current, *, calibration_path=None, held_plan=None):
        with self.kinematics_lock:
            air = compile_air(self.arms, scope=payload.get("scope"), speed=payload.get("speed"),
                              score_id=payload.get("score_id", "jin"),
                              calibration_path=calibration_path)
        with self.plan_lock:
            steps = []
            previous = 0.
            for stamp, wrists in air.pop("waypoints"):
                target = {(side, joint): q for side, values in wrists.items()
                          for joint, q in enumerate(values, 1)}
                steps.append(("起位" if not steps else "《槿》空弹", stamp - previous, target))
                previous = stamp
            held_air = (held_plan or {}).get("piano_air", {})
            start_target = steps[0][2]
            from_score_start = (air["scope"] != "start" and held_air.get("scope") == "start"
                                and held_air["score_id"] == air["score_id"]
                                and held_air["source"] == air["source"]
                                and held_plan["goal"] == start_target)
            if from_score_start:
                steps.pop(0)
                air["prepare_s"] = 0.
                previous = air["score_us"] / 1e6 / air["speed"]
                air["duration_s"] = previous
            air["from_score_start"] = from_score_start
            score_attacks = tuple(attacks(air["finger_events"]))
            speed = min(rated_limits(key[1])["max_speed"] * self.mappings[key].scale
                        for key in KEYS) * .2
            plan = dict(request_id=uuid.uuid4().hex, goal=steps[-1][2], keys=KEYS,
                        duration=previous, velocity=speed,
                        hold_targets=start_target if from_score_start else {}, steps=steps,
                        commanded=KEYS, source="piano-air", piano_air=air,
                        piano_onsets=tuple(onset for onset, _ in score_attacks),
                        piano_attacks=tuple(notes for _, notes in score_attacks))
            _, request = self._validate(plan, current)
            return plan, request


class MonitorTransport(PcanTransport):
    """Reuse the already-owned CAN handles and keep observing passive axes."""
    def __init__(self, mappings, plan, channels, arms, cancel, mode="MIT", profile=None, warn=None,
                 mode_registers=None):
        keys, start = plan["keys"], plan["start"]
        travel = {key: (plan.get("travel", {}).get(key, abs(plan["goal"][key] - start[key]))
                       + START_ERROR + TRAVEL_MARGIN) / mappings[key].scale for key in keys}
        settings = {key: dict(kp=DEFAULT_KP[key[1]-1], kd=DEFAULT_KD[key[1]-1],
                              max_travel=travel[key], max_temperature=50., max_age=.15,
                              **rated_limits(key[1])) for key in keys}
        super().__init__(mappings, keys, channels, joint_settings=settings, batch_mit=True,
                         driver_factory=lambda side, *_: nullcontext(arms[side]), warn=warn)
        self.start = start
        self.cancel = cancel
        self.profile = profile
        self.mode_registers = mode_registers or profile
        self.mode, self.mode_code = mode, MODE_CODES[mode]
        self.current_fraction = profile.current if profile is not None else POS_FORCE_CURRENT_FRACTION
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
        if self.mode_registers:
            # The console keeps running in its selected mode; a different or
            # unreadable motor mode is recorded, never used to refuse the motion.
            try:
                modes = self.mode_registers.read_modes(self.arms)
            except DriverError as exc:
                self._note("mode_readback", f"运动起点模式读回失败，按控制台模式继续：{exc}")
            else:
                wrong = [f"{side} J{j}" for (side, j), code in modes.items() if code != self.mode_code]
                if wrong:
                    self._note("mode_readback",
                               f"运动起点模式与控制台选择不一致，按控制台模式继续：{'、'.join(wrong)}")
        return self._sample(self.snapshot_arms())

    def continue_from_hold(self, plan):
        self.speed = plan["velocity"]
        for key in self.keys:
            origin = self.states[key].position_rad
            self.origins[key] = origin
            travel = plan.get("travel", {}).get(key, abs(plan["goal"][key] - plan["start"][key]))
            self.joint_settings[key]["max_travel"] = (
                travel + (START_ERROR + TRAVEL_MARGIN) / self.mappings[key].scale)
        self.validate_trajectory(plan["trajectory"])

    def enable(self):
        super().enable(cancel=self.cancel)

    def special_id(self, key):
        """CAN ID of the 0xFF×7+0xFC/0xFD enable and disable frame.

        OpenArm's own CAN library sends this frame to the motor's base command ID
        in every control mode; only the target frames take the mode offset. A
        commissioned profile may still record a per-axis ``mode`` override for
        firmware that answers on the offset ID instead.
        """
        return self.profile.special_id(key, self.mode) if self.profile is not None else key[1]

    def _command_frame(self, key, p, v):
        if self.mode == "MIT":
            return super()._command_frame(key, p, v)
        mapping = self.mappings[key]
        if self.mode == "VEL":
            return velocity_frame(key[1], p - self.states[key].position_rad, v,
                                  abs(mapping.velocity_to_motor(self.speed)))
        _, speed_range, _ = LIMITS[motor_type(key[1])]
        if abs(mapping.velocity_to_motor(self.speed)) > speed_range:
            self._note("protocol_range", f"{key[0]} J{key[1]} 目标速度超过协议上限，"
                                         f"按 {abs(mapping.velocity_to_model(speed_range)):.4f} rad/s 发送", key)
        return position_frame(self.mode, mapping, p, self.speed, self.current_fraction)

    def _enable_axis(self, key, state):
        if self.mode == "MIT":
            return super()._enable_axis(key, state)
        arm, j = self.arms[key[0]], key[1]
        self.states[key] = state
        first = self._command_frame(key, state.position_rad, 0.)
        arm.bus.send(*first)
        arm.bus.send(self.special_id(key), b"\xff" * 7 + b"\xfc")
        arm.bus.send(*first)
        return arm.wait_motor_state(j, 1)

    def _disable_axis(self, key):
        if self.mode == "MIT":
            return super()._disable_axis(key)
        arm = self.arms[key[0]]
        arm.bus.send(self.special_id(key), b"\xff" * 7 + b"\xfd")
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
