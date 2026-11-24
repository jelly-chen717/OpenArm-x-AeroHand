"""Preflight + monotonic, absolute-time, independent-arm score execution."""
import time
import uuid

import mujoco
import numpy as np

from openarm_app.trajectory import JointTrajectoryPlanner
from openarm_app.trajectory.quintic import quintic
from openarm_app.joint_convention import SIGNS
from paino_skill.score import PHYSICAL, WHITES, parse_score


class ScoreRunner:
    def __init__(self, engine, clock=time.monotonic_ns):
        self.engine, self.clock = engine, clock
        self.status, self.error = "EMPTY", None
        self.stop_reason = None
        self.prepared = None
        self.position_cache = {}
        self.offset = self.elapsed_us = 0
        self.motions, self.indices, self.held = {}, {}, set()
        self.lateness_us = 50_000
        self.last_line = None
        self.max_tick_gap_us = self.max_lateness_us = 0
        self.rate = 1.
        self.score_ns = 0.

    @property
    def active(self):
        return self.status in ("RUNNING", "PAUSED")

    def snapshot(self):
        return {"status": self.status, "error": self.error, "stop_reason": self.stop_reason,
                "elapsed_us": self.elapsed_us, "rate": self.rate,
                "duration_us": self.prepared["score"].duration_us if self.prepared else 0,
                "action_index": self.offset, "line": self.last_line,
                "moving": sorted(self.motions), "indices": dict(self.indices),
                "held": sorted(self.held), "lateness_limit_us": self.lateness_us,
                "max_tick_gap_us": self.max_tick_gap_us, "max_lateness_us": self.max_lateness_us}

    def _contacts(self, data, side):
        p, result = self.engine.piano, {}
        for c in data.contact:
            a, b = int(c.geom1), int(c.geom2)
            if a in p.pads and b in p.key_geoms:
                pad, key, normal = a, b, -c.frame[2]
            elif b in p.pads and a in p.key_geoms:
                pad, key, normal = b, a, c.frame[2]
            else:
                continue
            hand, finger = p.pads[pad]
            if hand == side and normal >= .6 and c.dist < -.0006:
                result.setdefault(finger, set()).add(p.key_geoms[key])
        return result

    def hand_position(self, side, index):
        """One wrist pose, four separate strokes; every pad is contact-checked."""
        key = (side, index)
        if key in self.position_cache:
            return self.position_cache[key]
        e, p = self.engine, self.engine.piano
        index_key = index + (3 if side == "left" else 0)
        midi = WHITES[index_key-1]
        hover_offset = e.scene_config.get("score_hover_clearance_m", 0.)
        errors = []
        # Keep successful existing positions unchanged. Only failed positions
        # try alternative IK seeds, continuation, then small wrist adjustments.
        for pitch in (None, .04, -.04, .08, -.08, .12, -.12, .18, -.18):
            try:
                if pitch is None:
                    base, _ = p.play_target(side, "index", midi, hover_offset=hover_offset)
                else:
                    seed = np.array(p.calibration["center_q_rad"][side])
                    seed[5] += SIGNS[side][5] * pitch * (1 if side == "right" else -1)
                    e.arms[side]._joints(seed)
                    base, _ = p._solve_play_basis(side, "index", midi, seed, hover_offset=hover_offset)
                for q in self._aligned_candidates(side, index, base, errors):
                    try:
                        result = self._validate_position(side, index, q)
                        result["wrist_seed_offset_rad"] = pitch or 0.
                        self.position_cache[key] = result
                        return result
                    except (ValueError, RuntimeError) as exc:
                        errors.append(str(exc))
            except (ValueError, RuntimeError) as exc:
                errors.append(str(exc))
        reason = errors[-1] if errors else "没有可用候选姿态"
        raise ValueError(f"多初值、分步对齐及小幅腕部调整后仍未找到合法四指姿态：{reason}")

    def _aligned_candidates(self, side, index, base, errors):
        """All endpoint solves delegate to official IK; no changes to limits."""
        e, p = self.engine, self.engine.piano
        arm, d = e.arms[side], p.scratch
        d.qpos[:] = e.home_qpos
        d.qpos[e.scene_qindices[side]] = base
        mujoco.mj_forward(e.model, d)
        start = arm.forward(base)
        target = start.copy()
        delta = np.array([0, -.0045 if side == "left" else .0045, 0])
        target[:3] += d.site("arm_origin").xmat.reshape(3, 3).T @ delta
        baseline = np.array(p.calibration["center_q_rad"][side])
        alternate = baseline.copy()
        alternate[1] = SIGNS[side][1] * (.6 if side == "right" else -.6)
        seeds = [base, baseline, e.joints[side]]
        neighbors = sorted(((abs(i-index), pos["q"]) for (s, i), pos in self.position_cache.items()
                            if s == side), key=lambda item: item[0])
        if neighbors:
            seeds.append(neighbors[0][1])
        seeds.append(alternate)
        tried = []
        for seed in seeds:
            if any(np.allclose(seed, q, atol=1e-9, rtol=0) for q in tried):
                continue
            tried.append(seed)
            try:
                yield arm.inverse(target, seed, position_tolerance=.00015,
                                  orientation_tolerance=np.deg2rad(.1))
            except (ValueError, RuntimeError) as exc:
                errors.append(str(exc))
        # Six 0.75 mm increments are offline IK aids, NOT additional timed
        # robot actions; the score still uses its original arm_time.
        try:
            q = base.copy()
            for fraction in np.linspace(1/6, 1, 6):
                waypoint = target.copy()
                waypoint[:3] = start[:3] + fraction*(target[:3]-start[:3])
                q = arm.inverse(waypoint, q, position_tolerance=.00015,
                                orientation_tolerance=np.deg2rad(.1))
            yield q
        except (ValueError, RuntimeError) as exc:
            errors.append(str(exc))

    def _validate_position(self, side, index, q):
        """Reject every candidate unless all original contact checks pass."""
        e, p = self.engine, self.engine.piano
        d = p.scratch
        e.arms[side]._joints(q)
        d.qpos[:] = e.home_qpos
        d.qpos[e.scene_qindices[side]] = q
        mujoco.mj_forward(e.model, d)
        if self._contacts(d, side):
            raise ValueError(f"{side} index={index} 抬指姿态仍触键")
        strokes = {}
        for i, finger in enumerate(PHYSICAL[4][side]):
            midi = WHITES[index+i-1]
            top_z = d.site(f"key_{midi}_top").xpos[2]
            lo, hi = 0., .6
            for _ in range(17):
                amount = (lo+hi)/2
                p._set_finger(d, side, finger, amount)
                mujoco.mj_forward(e.model, d)
                if p._pad_bottom(d, side, finger)[2] > top_z-.003:
                    lo = amount
                else:
                    hi = amount
            strokes[finger] = hi
            p._set_finger(d, side, finger, hi)
            mujoco.mj_forward(e.model, d)
            hits = self._contacts(d, side)
            if hits != {finger: {midi}}:
                raise ValueError(f"{side} index={index} {finger} 未能独立接触白键 {index+i}，实际 {hits}")
            p._set_finger(d, side, finger, 0)
        # Verify the complete chord in this common wrist pose too.
        for finger, amount in strokes.items():
            p._set_finger(d, side, finger, amount)
        mujoco.mj_forward(e.model, d)
        expected = {f: {WHITES[index+i-1]} for i, f in enumerate(PHYSICAL[4][side])}
        if self._contacts(d, side) != expected:
            raise ValueError(f"{side} index={index} 四指同时触键复核失败")
        result = {"q": q.copy(), "strokes": strokes}
        return result

    def prepare(self, text, finger_count, velocity=2., acceleration=20., automatic_motion=False):
        e = self.engine
        if self.active or e.state in ("EXECUTING", "PAUSED") or e.piano.busy:
            raise ValueError("请先完成或停止当前动作")
        score = parse_score(text, finger_count=finger_count)
        if finger_count != 4:
            raise ValueError("解析器支持五指；当前 Aero 演奏端仅启用四指，尚无拇指触键标定")
        if any(v != 0 for side in e.piano.values.values() for v in side.values()):
            raise ValueError("预检前请松开全部手指")
        for value in (velocity, acceleration):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError("速度和加速度上限必须为有限正数")
        if not isinstance(automatic_motion, bool):
            raise ValueError("automatic_motion 必须为布尔值")
        peak_velocity = peak_acceleration = 0.
        starts = {s: e.joints[s].copy() for s in e.SIDES}
        last = {s: q.copy() for s, q in starts.items()}
        plans, positions = {}, {}
        for i, a in enumerate(score.actions):
            if a["kind"] != "move":
                continue
            side, index, duration = a["side"], a["index"], a["duration_us"]/1e6
            try:
                pose = self.hand_position(side, index)
                q = pose["q"]
                limits = e.arms[side].limits
                planner = JointTrajectoryPlanner(limits[:, 0], limits[:, 1])
                delta = float(np.max(np.abs(q-last[side])))
                required_v = 1.875*delta/duration
                required_a = (10/np.sqrt(3))*delta/duration**2
                peak_velocity = max(peak_velocity, required_v)
                peak_acceleration = max(peak_acceleration, required_a)
                move_v = max(velocity, required_v*(1+1e-10)) if automatic_motion else velocity
                move_a = max(acceleration, required_a*(1+1e-10)) if automatic_motion else acceleration
                tr = planner.plan(last[side], q, [move_v]*7, [move_a]*7,
                                  duration=duration, sample_period=min(1/60, duration))
                if tr.time[-1] > duration+1e-9:
                    delta = float(np.max(np.abs(q-last[side])))
                    required_v = 1.875*delta/duration
                    required_a = (10/np.sqrt(3))*delta/duration**2
                    raise ValueError(f"指定 {duration:.6f} s 不满足速度/加速度上限；至少需 {tr.time[-1]:.6f} s，未延长时间。"
                                     f"按原时间需要峰值 {required_v:.4f} rad/s、{required_a:.4f} rad/s²")
                plans[i] = {"q0": last[side].copy(), "q1": q.copy()}
                positions[side, index] = pose
                last[side] = q.copy()
            except (ValueError, RuntimeError) as exc:
                raise ValueError(f"第 {a['line']} 行，{side} index={index} 运动预检失败：{exc}") from exc
        token = uuid.uuid4().hex
        self.prepared = {"token": token, "score": score, "starts": starts,
                         "plans": plans, "positions": positions, "automatic_motion": automatic_motion,
                         "peak_velocity": peak_velocity, "peak_acceleration": peak_acceleration,
                         "velocity_limit": velocity, "acceleration_limit": acceleration}
        self.status, self.error, self.elapsed_us, self.offset = "READY", None, 0, 0
        self.stop_reason = None
        return {"token": token, **score.summary(), "motion_check": "passed", "lateness_limit_us": self.lateness_us,
                "automatic_motion": automatic_motion, "peak_velocity_rad_s": peak_velocity,
                "peak_acceleration_rad_s2": peak_acceleration}

    def validate_rate(self, rate):
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not np.isfinite(rate) or not .5 <= rate <= 2:
            raise ValueError("演奏倍速必须为 0.5–2.0 的有限数值")
        if self.prepared and not self.prepared["automatic_motion"]:
            if (self.prepared["peak_velocity"]*rate > self.prepared["velocity_limit"]*(1+1e-9) or
                    self.prepared["peak_acceleration"]*rate**2 > self.prepared["acceleration_limit"]*(1+1e-9)):
                raise ValueError("该倍速超过手动运动上限，请启用自动仿真参数后重新预检")
        return float(rate)

    def start(self, token, rate=1.):
        e = self.engine
        if self.active or e.state in ("EXECUTING", "PAUSED") or e.piano.busy:
            raise ValueError("当前动作尚未结束")
        if not isinstance(token, str) or not token or not self.prepared or token != self.prepared["token"]:
            raise ValueError("请先完成整份文件预检")
        if any(not np.allclose(e.joints[s], q, atol=1e-9, rtol=0) for s, q in self.prepared["starts"].items()):
            raise ValueError("预检后手臂姿态已改变，请重新预检")
        if any(v != 0 for values in (e.piano.values, e.piano.targets) for side in values.values() for v in side.values()):
            raise ValueError("启动前必须松开全部手指")
        self.rate = self.validate_rate(rate)
        self.score_ns = 0.
        e.piano.stop()
        self.status, self.error, self.offset, self.elapsed_us = "RUNNING", None, 0, 0
        self.motions, self.indices, self.held = {}, {}, set()
        self.start_ns = self.last_ns = self.clock()
        self.max_tick_gap_us = self.max_lateness_us = 0
        e.state, e.motion_id = "EXECUTING", uuid.uuid4().hex
        e.interpolation_steps, e.completed_steps = None, 0
        e.duration, e.elapsed = self.prepared["score"].duration_us/1e6, 0.
        self.tick()

    def pause(self):
        if self.status != "RUNNING":
            return
        self.tick()
        if self.status != "RUNNING":
            return
        self.status = self.engine.state = "PAUSED"
        self.engine.piano.paused = True
        for velocity in self.engine.velocities.values():
            velocity[:] = 0

    def resume(self):
        if self.status != "PAUSED":
            return
        self.last_ns = self.clock()
        self.status, self.engine.state = "RUNNING", "EXECUTING"
        self.engine.piano.paused = False
        self._move_positions(self.elapsed_us)

    def set_rate(self, rate):
        rate = self.validate_rate(rate)
        if not self.active:
            raise ValueError("当前没有运行或暂停的文本演奏")
        if self.status == "RUNNING":
            self.tick()
        if not self.active:
            return
        self.rate = rate
        if self.status == "RUNNING":
            self._move_positions(self.elapsed_us)

    def stop_if_current(self, token, reason):
        if reason not in ("user_stop", "cancel_start", "page_closed"):
            raise ValueError("无效的演奏停止原因")
        if (not isinstance(token, str) or not token or self.status not in ("READY", "RUNNING", "PAUSED")
                or not self.prepared or self.prepared["token"] != token):
            return False
        self.stop(reason=reason)
        return True

    def stop(self, error=None, *, reason="stop_request"):
        self.status = "ERROR" if error else "STOPPED"
        self.error = error
        self.stop_reason = "scheduler_error" if error else reason
        if self.prepared:
            self.prepared["token"] = None
        self.motions.clear()
        self.held.clear()
        e = self.engine
        e.piano.stop()
        for dq in e.velocities.values():
            dq[:] = 0
        e.state, e.active_arms = "STOPPED", []
        e.refresh_model()

    def _move_positions(self, now):
        e = self.engine
        for side, m in list(self.motions.items()):
            if now >= m["end"] and (now-m["end"])/self.rate > self.lateness_us:
                raise ValueError(f"{side} 未按时到位，迟到 {(now-m['end'])/1000:.1f} ms")
            u = min(1., (now-m["start"])/(m["end"]-m["start"]))
            pos, vel, _ = quintic(u)
            delta = m["q1"]-m["q0"]
            e.joints[side] = m["q0"]+pos*delta
            e.velocities[side] = self.rate*vel*delta/((m["end"]-m["start"])/1e6)
            if u >= 1:
                e.joints[side] = m["q1"].copy()
                e.velocities[side][:] = 0
                del self.motions[side]
        e.active_arms = list(self.motions)

    def tick(self):
        if self.status != "RUNNING":
            return
        e, p, prepared = self.engine, self.engine.piano, self.prepared
        current = self.clock()
        self.score_ns += max(0, current-self.last_ns)*self.rate
        now = int(self.score_ns//1000)
        try:
            gap = (current-self.last_ns)//1000
            self.max_tick_gap_us = max(self.max_tick_gap_us, gap)
            if gap > self.lateness_us:
                raise ValueError(f"调度间隔 {gap/1000:.1f} ms 超过 50 ms，已停止，不集中补发过期动作")
            wall_end = p.time + (current-self.last_ns)/1e9
            self.last_ns, self.elapsed_us = current, now
            actions = prepared["score"].actions
            while self.offset < len(actions) and actions[self.offset]["time_us"] <= now:
                stamp = actions[self.offset]["time_us"]
                lateness = (now-stamp)/self.rate
                self.max_lateness_us = max(self.max_lateness_us, lateness)
                end = self.offset
                while end < len(actions) and actions[end]["time_us"] == stamp:
                    end += 1
                if lateness > self.lateness_us:
                    raise ValueError("动作截止时间已错过，已停止，不补发")
                # Preserve ordered note transitions crossed within one normal
                # scheduler tick, including sub-tick notes at higher rates.
                p.time = wall_end - lateness/1e6
                self._move_positions(stamp)
                while self.offset < end:
                    a = actions[self.offset]
                    self.last_line = a["line"]
                    if a["kind"] == "move":
                        side = a["side"]
                        if side in self.motions:
                            raise ValueError(f"{side} 前一移动未完成")
                        plan = prepared["plans"][self.offset]
                        self.motions[side] = {**plan, "start": stamp, "end": stamp+a["duration_us"]}
                        self.indices[side] = a["index"]
                        e.goals[side] = plan["q1"].copy()
                    else:
                        for f in a["fingers"]:
                            side, finger = f["side"], f["finger"]
                            value = 0.
                            if a["direction"] == "D":
                                if side in self.motions:
                                    raise ValueError(f"{side} 未到位，禁止按键")
                                value = prepared["positions"][side, self.indices[side]]["strokes"][finger]
                                self.held.add(f["id"])
                            else:
                                self.held.discard(f["id"])
                            # Ideal servo: atomic same-timestamp finger updates;
                            # no hidden 0.1 s wait or automatic note-off.
                            p.values[side][finger] = p.targets[side][finger] = value
                        # Commit the complete hand instruction as one update.
                        # A later U in this same timestamp still observes D first.
                        e.refresh_model()
                    self.offset += 1
            p.time = wall_end
            self._move_positions(now)
            e.elapsed = min(e.duration, now/1e6)
            e.refresh_model()
            if now >= prepared["score"].duration_us and self.offset == len(actions) and not self.motions:
                self.status, e.state = "COMPLETED", "COMPLETED"
        except Exception as exc:
            self.stop(f"第 {self.last_line or 1} 行附近：{exc}")
