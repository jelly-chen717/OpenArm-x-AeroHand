"""Ideal Aero joint servos and contact-driven piano keys, with note events.

No whole-robot dynamics or hardware control. MuJoCo's finger-pad/key contacts
drive key travel; only measured contact depression emits performance notes.
"""
from collections import deque
import json
import uuid

import mujoco
import numpy as np

from openarm_app.trajectory.quintic import quintic
from openarm_app.joint_convention import CONVENTION_VERSION, SIGNS
from paino_skill.scene import PIANO

FINGERS = {"index": "if", "middle": "mf", "ring": "rf", "pinky": "pf", "thumb": "th"}
WHITE_PITCHES = (0, 2, 4, 5, 7, 9, 11)


def keyboard_layout(cfg):
    result, white = [], 0
    for midi in range(cfg["first_midi"], cfg["first_midi"] + cfg["key_count"]):
        is_white = midi % 12 in WHITE_PITCHES
        if is_white:
            white += 1
        result.append({"midi": midi, "note": note_name(midi), "white": is_white,
                       "white_key": white if is_white else None, "after_white": white})
    return result


def note_name(midi):
    return ("C", "C♯", "D", "D♯", "E", "F", "F♯", "G", "G♯", "A", "A♯", "B")[midi % 12] + str(midi//12-1)


class PianoPerformance:
    def __init__(self, engine):
        self.engine, self.model = engine, engine.model
        self.layout = keyboard_layout(engine.scene_config)
        self.midis = {k["midi"] for k in self.layout}
        self.travel = engine.scene_config["key_travel_m"]
        self.keys = {n: self.model.joint(f"key_{n}_press").qposadr[0] for n in self.midis}
        self.key_geoms = {i: int(self.model.geom(i).name.split("_")[1]) for i in range(self.model.ngeom)
                          if self.model.geom(i).name.startswith("key_")}
        self.pads, self.finger_joints = {}, {}
        self.values = {s: {f: 0.0 for f in FINGERS} for s in engine.SIDES}
        self.targets = {s: dict(v) for s, v in self.values.items()}
        for side in engine.SIDES:
            for finger, short in FINGERS.items():
                self.pads[self.model.geom(f"aero_{side}_{short}_tip").id] = (side, finger)
                suffixes = ("cmc_flex", "mcp", "ip") if finger == "thumb" else ("mcp_flex", "pip", "dip")
                scales = (.4, .45, .45) if finger == "thumb" else (.65, .325, .325)
                self.finger_joints[side, finger] = [
                    (self.model.joint(f"aero_{side}_{side}_{finger}_{suffix}").qposadr[0], scale)
                    for suffix, scale in zip(suffixes, scales)]
        self.depths = {n: 0.0 for n in self.midis}
        self.pressed = set()
        self.events = deque(maxlen=512)
        self.event_id, self.time = 0, 0.0
        self.session = uuid.uuid4().hex
        self.sequence = None
        self.paused = False
        self.scratch = mujoco.MjData(self.model)
        self.calibration = json.loads((PIANO / "calibration.json").read_text(encoding="utf-8"))
        if self.calibration.get("coordinate_convention") != CONVENTION_VERSION:
            raise ValueError("Piano calibration coordinate convention mismatch")
        self.pose_cache = {}

    @property
    def busy(self):
        return self.sequence is not None and self.sequence["phase"] not in ("done", "failed", "stopped")

    def snapshot(self, after=0):
        return {"session": self.session, "time_s": self.time, "last_event_id": self.event_id,
                "arm_state": self.engine.state,
                "events": [e for e in self.events if e["id"] > after],
                "pressed": sorted(self.pressed), "depths_m": {str(n): d for n, d in self.depths.items() if d > 0},
                "fingers": {s: dict(v) for s, v in self.values.items()},
                "busy": self.busy, "paused": self.paused,
                "score": self.engine.score.snapshot() if getattr(self.engine, "score", None) else None,
                "sequence": dict(self.sequence) if self.sequence else None}

    def summary(self):
        result = self.snapshot(self.event_id)
        result.pop("events")
        return result

    def _set_finger(self, data, side, finger, amount):
        for address, scale in self.finger_joints[side, finger]:
            data.qpos[address] = amount * scale

    def refresh(self, data):
        for side in self.values:
            for finger, amount in self.values[side].items():
                self._set_finger(data, side, finger, amount)
        # Measure penetration against unpressed keys, then move their actual
        # slide joints by that depth. This is an ideal contact follower.
        for address in self.keys.values():
            data.qpos[address] = 0
        mujoco.mj_forward(self.model, data)
        depths = dict.fromkeys(self.midis, 0.0)
        sources = {}
        for c in data.contact:
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.key_geoms and g2 in self.pads:
                key_geom, pad, normal_z = g1, g2, c.frame[2]
            elif g2 in self.key_geoms and g1 in self.pads:
                key_geom, pad, normal_z = g2, g1, -c.frame[2]
            else:
                continue
            # Top-surface contact only: side/bottom collisions must not sound.
            if normal_z < .6 or c.dist >= 0:
                continue
            midi = self.key_geoms[key_geom]
            depth = min(self.travel, max(0.0, -float(c.dist)/normal_z))
            if depth > depths[midi]:
                depths[midi], sources[midi] = depth, self.pads[pad]
        for midi, depth in depths.items():
            data.qpos[self.keys[midi]] = depth
            was = midi in self.pressed
            if not was and depth >= .0006:
                self.pressed.add(midi)
                self._event("on", midi, sources[midi], depth)
            elif was and depth <= .0002:
                self.pressed.remove(midi)
                self._event("off", midi, None, depth)
        self.depths = depths
        if any(depths.values()):
            mujoco.mj_forward(self.model, data)

    def _event(self, kind, midi, source, depth):
        self.event_id += 1
        self.events.append({"id": self.event_id, "type": kind, "midi": midi,
                            "time_s": self.time, "velocity": float(np.clip(.35+depth/self.travel*.65, .35, 1)),
                            "source": "aero_contact", "hand": source[0] if source else None,
                            "finger": source[1] if source else None})
        if (kind == "on" and self.busy and self.sequence["midi"] == midi
                and self.sequence["phase"] in ("pressing", "releasing")
                and source == (self.sequence["hand"], self.sequence["finger"])):
            self.sequence["hit"] = True

    def set_finger(self, side, finger, amount):
        if self.busy:
            raise ValueError("触键动作执行中，请完成或停止后调整手指")
        if side not in self.values or finger not in FINGERS:
            raise ValueError("请选择左/右手及有效手指")
        if isinstance(amount, bool) or not isinstance(amount, (int, float)) or not np.isfinite(amount) or not 0 <= amount <= 1:
            raise ValueError("手指弯曲量必须为 0–1")
        self.targets[side][finger] = float(amount)

    def stop(self):
        if self.busy:
            self.sequence["phase"] = "stopped"
        for side in self.values:
            for finger in FINGERS:
                self.values[side][finger] = self.targets[side][finger] = 0.0
        self.paused = False

    def _pad_bottom(self, data, side, finger):
        gid = self.model.geom(f"aero_{side}_{FINGERS[finger]}_tip").id
        point = data.geom_xpos[gid].copy()
        point[2] -= np.abs(data.geom_xmat[gid].reshape(3, 3)[2]) @ self.model.geom_size[gid]
        return point

    def play_target(self, side, finger, midi, *, hover_offset=0.):
        """Use official wrist IK with a measured finger-pad offset; no new IK core."""
        cache_key = (side, finger, midi) if hover_offset == 0 else (side, finger, midi, hover_offset)
        if cache_key in self.pose_cache:
            return self.pose_cache[cache_key]
        baseline = np.array(self.calibration["center_q_rad"][side])
        error = ""
        # A small wrist-pitch change can reach raised black keys while keeping
        # fingers facing forward. Every candidate still uses official V1 IK.
        candidates = [(pitch, .012) for pitch in (0., .18, -.18, .35, -.35)]
        if midi % 12 not in WHITE_PITCHES:
            # A lower piano may put the original black-key target just beyond
            # reach. Retry 2 mm closer to the player, still on the key surface.
            candidates.extend((pitch, .014) for pitch in (.18, 0., -.18))
        for pitch, black_front_offset in candidates:
            seed = baseline.copy()
            seed[5] += SIGNS[side][5] * pitch * (1 if side == "right" else -1)
            try:
                self.engine.arms[side]._joints(seed)
                result = self._solve_play_basis(side, finger, midi, seed, hover_offset=hover_offset,
                                                black_front_offset=black_front_offset)
                self.pose_cache[cache_key] = result
                return result
            except (RuntimeError, ValueError) as exc:
                error = str(exc)
        raise ValueError(f"{note_name(midi)} 当前手指未找到可达触键姿态：{error}")

    def _solve_play_basis(self, side, finger, midi, seed, *, hover_offset=0., black_front_offset=.012):
        e, d = self.engine, self.scratch
        d.qpos[:] = e.home_qpos
        d.qpos[e.scene_qindices[side]] = seed
        mujoco.mj_forward(self.model, d)
        rest = self._pad_bottom(d, side, finger)
        # Find a small, downward finger stroke within the original joint limits.
        stroke = None
        for amount in np.linspace(.05, .8, 31):
            self._set_finger(d, side, finger, amount)
            mujoco.mj_forward(self.model, d)
            contact_point = self._pad_bottom(d, side, finger)
            if rest[2]-contact_point[2] >= .012:
                stroke = float(amount)
                break
        if stroke is None:
            raise ValueError("此姿态下该手指无法向下触键，请选食指/中指/无名指/小指")
        key_top = d.site(f"key_{midi}_top").xpos.copy()
        # TXT can retain its previous hover plane after lowering the piano.
        # ScoreRunner separately recalibrates every stroke against REAL keys.
        key_top[2] += hover_offset
        if midi % 12 not in WHITE_PITCHES:
            # Contact the front part of a black key, within its top surface.
            key_top[0] -= black_front_offset
        # Put the flexed fingertip 4 mm into the top; open finger hovers above it.
        delta = key_top - contact_point
        delta[2] -= .004
        origin_rotation = d.site("arm_origin").xmat.reshape(3, 3)
        target = e.arms[side].forward(seed)
        target[:3] += origin_rotation.T @ delta
        alternate = seed.copy()
        alternate[1] = SIGNS[side][1] * (.6 if side == "right" else -.6)
        candidates, error = [], ""
        for initial in (e.joints[side], seed, alternate):
            try:
                candidates.append(e.arms[side].inverse(target, initial, position_tolerance=.00015,
                                                       orientation_tolerance=np.deg2rad(.1)))
            except (RuntimeError, ValueError) as exc:
                error = str(exc)
        if not candidates:
            raise ValueError(f"{note_name(midi)} 超出当前手姿的已解算范围：{error}")
        q = min(candidates, key=lambda q: np.linalg.norm(q-e.joints[side]))
        d.qpos[e.scene_qindices[side]] = q
        mujoco.mj_forward(self.model, d)
        actual = self._pad_bottom(d, side, finger)
        if np.linalg.norm(actual - (key_top + [0, 0, -.004])) > .0008:
            raise ValueError("触键指尖位置复核未通过")
        return q, stroke

    def start_play(self, request):
        e = self.engine
        if self.busy or e.state in ("EXECUTING", "PAUSED"):
            raise ValueError("动作执行中，请完成或停止后再弹奏")
        midi, side, finger = request.get("midi"), request.get("arm", "auto"), request.get("finger", "index")
        if isinstance(midi, bool) or not isinstance(midi, int) or midi not in self.midis:
            raise ValueError("音符必须为 MIDI 21–108（A0–C8）")
        if side == "auto":
            side = "left" if midi < 60 else "right"
        if side not in e.SIDES or finger not in FINGERS:
            raise ValueError("手或手指选择无效")
        q, stroke = self.play_target(side, finger, midi)
        # The normal planner validates count/velocity/acceleration atomically.
        e.command({**request, "action": "move_joints", "arm": side, "q": q})
        self.stop()
        self.sequence = {"id": uuid.uuid4().hex, "phase": "moving", "midi": midi,
                         "hand": side, "finger": finger, "stroke": stroke, "phase_time": 0., "hit": False}

    def advance(self, dt):
        self.time += dt
        if self.paused:
            return
        if self.busy:
            seq = self.sequence
            if seq["phase"] == "moving":
                if self.engine.state != "COMPLETED":
                    return
                seq["phase"], seq["phase_time"] = "pressing", 0.0
            seq["phase_time"] += dt
            t = seq["phase_time"]
            if t < .12:
                value = float(quintic(min(1., t/.12))[0]) * seq["stroke"]
            elif t < .30:
                value = seq["stroke"]
            elif t < .45:
                seq["phase"] = "releasing"
                value = (1-float(quintic(min(1., (t-.30)/.15))[0])) * seq["stroke"]
            else:
                value = 0.0
                seq["phase"] = "done" if seq["hit"] else "failed"
            self.values[seq["hand"]][seq["finger"]] = value
        else:
            for side in self.values:
                for finger in FINGERS:
                    value, target = self.values[side][finger], self.targets[side][finger]
                    self.values[side][finger] += float(np.clip(target-value, -3*dt, 3*dt))
