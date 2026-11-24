"""Persistent ideal MuJoCo arm, controlled over a loopback-only JSON API."""
import argparse
from contextlib import nullcontext
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import urlparse, parse_qs

import mujoco
import numpy as np
from openarm_app.kinematics import OpenArmIK
from openarm_app.trajectory import JointTrajectoryPlanner
from openarm_app.trajectory.quintic import quintic
from openarm_app.trajectory.timing import INTERPOLATION_PERIOD_S, interpolation_steps as validate_steps
from openarm_app.validation import positive, vector


class ArmServerState:
    SIDES = ("right", "left")

    def __init__(self, scene="default", *, piano_config=None):
        if scene not in ("default", "piano"):
            raise ValueError("Unknown scene")
        self.scene = scene
        self.lock = threading.RLock()
        self.arm = OpenArmIK()
        # Share immutable model geometry, but keep official IK state/data separate.
        self.arms = {"right": self.arm,
                     "left": OpenArmIK(self.arm.xml, side="left", model=self.arm.setup.model)}
        self.model = self.arm.setup.model
        self.model.vis.quality.shadowsize = 2048
        self.data = mujoco.MjData(self.model)
        self.home_qpos = self.arm.setup.data.qpos.copy()
        self.resolver = self.arm.setup.joint_resolver
        limits = np.vstack([self.arms[side].limits for side in self.SIDES])
        self.planner = JointTrajectoryPlanner(limits[:, 0], limits[:, 1], joint_count=14)
        self.joints = {s: self.arms[s].home.copy() for s in self.SIDES}
        self.velocities = {s: np.zeros(7) for s in self.SIDES}
        self.goals = {s: self.joints[s].copy() for s in self.SIDES}
        self.starts = {s: self.joints[s].copy() for s in self.SIDES}
        self.active_arms = []
        self.elapsed = self.duration = 0.0
        self.interpolation_steps = None
        self.completed_steps = 0
        self.state = "IDLE"
        self.motion_id = None
        self.piano = None
        self.score = None
        if scene == "piano":
            from paino_skill.scene import build_scene
            self.model, self.data, self.scene_config = build_scene(piano_config)
            self.home_qpos = self.data.qpos.copy()
        self.scene_qindices = {s: np.array([self.model.joint(f"openarm_{s}_joint{i}").qposadr[0]
                                           for i in range(1, 8)]) for s in self.SIDES}
        self.scene_dindices = {s: np.array([self.model.joint(f"openarm_{s}_joint{i}").dofadr[0]
                                           for i in range(1, 8)]) for s in self.SIDES}
        if scene == "piano":
            from paino_skill.performance import PianoPerformance
            self.piano = PianoPerformance(self)
            from paino_skill.score_runner import ScoreRunner
            self.score = ScoreRunner(self)
        self.refresh_model()

        from openarm_app.visualization import geometry_ids
        self.visual_ids = geometry_ids(self.model)
        self.visual_id = uuid.uuid4().hex
        self._visual_model = None

    def display_model(self):
        if self._visual_model is None:
            from openarm_app.visualization import visual_model
            self._visual_model = {"id": self.visual_id, **visual_model(self.model, self.visual_ids)}
        return self._visual_model

    # Preserve the original right-arm attributes for existing software checks.
    @property
    def q(self):
        return self.joints["right"]

    @q.setter
    def q(self, value):
        self.joints["right"] = value

    @property
    def dq(self):
        return self.velocities["right"]

    @property
    def goal(self):
        return self.goals["right"]

    @goal.setter
    def goal(self, value):
        self.goals["right"] = value

    def refresh_model(self):
        self.data.qpos[:] = self.home_qpos
        self.data.qvel[:] = 0
        for side in self.SIDES:
            self.data.qpos[self.scene_qindices[side]] = self.joints[side]
            self.data.qvel[self.scene_dindices[side]] = self.velocities[side]
        if self.piano:
            self.piano.refresh(self.data)
        else:
            mujoco.mj_forward(self.model, self.data)

    def snapshot(self):
        from openarm_app.visualization import visual_state
        arms = {s: {"q": self.joints[s].tolist(), "dq": self.velocities[s].tolist(),
                    "pose": self.arms[s].forward(self.joints[s]).tolist(),
                    "goal": self.goals[s].tolist(), "home": self.arms[s].home.tolist(),
                    "limits": self.arms[s].limits.tolist()} for s in self.SIDES}
        return {"api_version": 2, "mode": "ideal", "state": self.state, "scene": self.scene,
                "visual": {"id": self.visual_id, **visual_state(self.model, self.data, self.visual_ids)},
                **arms["right"],  # Legacy top-level fields always refer to right.
                "arms": arms, "active_arms": list(self.active_arms),
                "joint_order": [f"{s}_joint{i}" for s in self.SIDES for i in range(1, 8)],
                "q14": np.concatenate([self.joints[s] for s in self.SIDES]).tolist(),
                "motion_id": self.motion_id,
                "elapsed_s": self.elapsed, "duration_s": self.duration,
                "interpolation_steps": self.interpolation_steps,
                "completed_steps": self.completed_steps,
                "interpolation_period_s": INTERPOLATION_PERIOD_S,
                "piano": self.piano.summary() if self.piano else None,
                "score": self.score.snapshot() if self.score else None,
                "progress": min(1.0, self.elapsed/self.duration) if self.duration else 0.0}

    def _limits14(self, request, name, default, selected):
        count = 7*len(selected)
        values = vector(request.get(name, [default]*count), count, name)
        if len(selected) == 2:
            return values
        return np.concatenate([values if side in selected else np.full(7, default)
                               for side in self.SIDES])

    @staticmethod
    def _targets(request, field, selected):
        values = request.get(field)
        if len(selected) == 1:
            return {selected[0]: values}
        if not isinstance(values, dict) or set(values) != {"right", "left"}:
            raise ValueError(f"For both arms, {field} must contain exactly right and left")
        return values

    def command(self, request):
        if not isinstance(request, dict):
            raise ValueError("Request must be an object")
        action = request.get("action")
        if action == "piano_score_stop":
            if not self.score:
                raise ValueError("请启动钢琴仿真场景")
            return {"stopped": self.score.stop_if_current(request.get("token"), request.get("reason", "user_stop"))}
        if self.score and self.score.active:
            if action == "stop":
                self.score.stop()
                return self.snapshot()
            if action in ("pause", "piano_score_pause"):
                self.score.pause()
                return self.snapshot()
            if action in ("resume", "piano_score_resume"):
                self.score.resume()
                return self.snapshot()
            if action == "piano_score_rate":
                self.score.set_rate(request.get("rate"))
                return self.snapshot()
            raise ValueError("文本演奏中，仅允许暂停、继续、倍速与停止")
        if action in ("piano_score_pause", "piano_score_resume", "piano_score_rate"):
            raise ValueError("当前没有运行或暂停的文本演奏")
        if action in ("piano_score_prepare", "piano_score_start"):
            if not self.score:
                raise ValueError("请启动钢琴仿真场景")
            if action == "piano_score_prepare":
                try:
                    return self.score.prepare(request.get("text"), request.get("finger_count"),
                                              request.get("velocity_limit", 2.), request.get("acceleration_limit", 20.),
                                              request.get("automatic_motion", False))
                except (ValueError, RuntimeError):
                    if not self.score.active:
                        self.score.prepared = None
                        self.score.status = "INVALID"
                    raise
            self.score.start(request.get("token"), request.get("rate", 1.))
            return self.snapshot()
        if action in ("piano_play", "piano_finger", "piano_release"):
            if not self.piano:
                raise ValueError("请启动88键钢琴场景")
            if action == "piano_play":
                self.piano.start_play(request)
            elif action == "piano_finger":
                self.piano.set_finger(request.get("arm"), request.get("finger"), request.get("amount"))
            else:
                if self.piano.busy:
                    raise ValueError("触键动作中请使用停止按钮")
                self.piano.stop()
            self.refresh_model()
            return self.snapshot()
        arm = request.get("arm", "right")
        if arm not in ("right", "left", "both"):
            raise ValueError("arm must be right, left or both")
        selected = list(self.SIDES) if arm == "both" else [arm]
        if action == "pause":
            if self.piano:
                self.piano.paused = True
            if self.state == "EXECUTING":
                self.state = "PAUSED"
                for velocity in self.velocities.values():
                    velocity[:] = 0
        elif action == "resume":
            if self.piano:
                self.piano.paused = False
            if self.state == "PAUSED":
                self.state = "EXECUTING"
        elif action == "stop":
            if self.score and self.score.prepared:
                self.score.prepared["token"] = None
                self.score.status = "STOPPED"
            if self.piano:
                self.piano.stop()
            self.state = "STOPPED"
            for velocity in self.velocities.values():
                velocity[:] = 0
        elif action in ("move_joints", "move_pose", "home", "preview_pose"):
            if self.state in ("EXECUTING", "PAUSED") or (self.piano and self.piano.busy):
                raise ValueError("Arm is busy; complete or stop the current motion first")
            goals = {s: self.joints[s].copy() for s in self.SIDES}
            if action == "home":
                for side in selected:
                    goals[side] = self.arms[side].home.copy()
            else:
                is_pose = action in ("move_pose", "preview_pose")
                targets = self._targets(request, "pose" if is_pose else "q", selected)
                for side in selected:
                    goals[side] = (self.arms[side].inverse(targets[side], self.joints[side]) if is_pose
                                   else self.arms[side]._joints(targets[side]))
            if action == "preview_pose":
                qs = {s: goals[s].tolist() for s in selected}
                poses = {s: self.arms[s].forward(goals[s]).tolist() for s in selected}
                return {"arm": arm, "q_goal": qs if arm == "both" else qs[arm],
                        "fk_pose": poses if arm == "both" else poses[arm]}
            steps = validate_steps(request["interpolation_steps"]) if "interpolation_steps" in request else None
            trajectory = self.planner.plan(
                np.concatenate([self.joints[s] for s in self.SIDES]),
                np.concatenate([goals[s] for s in self.SIDES]),
                self._limits14(request, "velocity_limits", .2, selected),
                self._limits14(request, "acceleration_limits", .4, selected),
                duration=request.get("duration"),
                sample_period=INTERPOLATION_PERIOD_S if steps is not None else .05,
                interpolation_steps=steps)
            # State changes only after all validation, IK and planning succeed.
            self.starts = {s: self.joints[s].copy() for s in self.SIDES}
            self.goals = goals
            self.active_arms = selected
            self.duration = float(trajectory.time[-1])
            self.elapsed = 0.0
            self.interpolation_steps = steps
            self.completed_steps = 0
            self.motion_id = uuid.uuid4().hex
            self.state = "EXECUTING"
        else:
            raise ValueError(f"Unknown action: {action}")
        self.refresh_model()
        return self.snapshot()

    def advance(self, dt):
        positive(dt, "dt")
        if self.score and self.score.active:
            self.score.tick()
            return
        if self.state == "EXECUTING":
            if self.interpolation_steps is not None:
                if not np.isclose(dt, INTERPOLATION_PERIOD_S, rtol=0, atol=1e-12):
                    raise ValueError("Fixed-step motion requires the fixed interpolation period")
                self.completed_steps += 1
                self.elapsed = self.completed_steps * INTERPOLATION_PERIOD_S
            else:
                self.elapsed = min(self.duration, self.elapsed+dt)
            s, ds, _ = quintic(self.elapsed/self.duration)
            for side in self.SIDES:
                delta = self.goals[side]-self.starts[side]
                self.joints[side] = self.starts[side]+s*delta
                self.velocities[side] = ds*delta/self.duration
            if self.elapsed >= self.duration:
                for side in self.SIDES:
                    self.joints[side] = self.goals[side].copy()
                    self.velocities[side][:] = 0
                self.state = "COMPLETED"
        if self.piano:
            self.piano.advance(dt)
        self.refresh_model()


def create_server(engine, port=8769):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, code, payload):
            body = json.dumps(payload, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            try:
                self.end_headers()
                self.wfile.write(body)
            except ConnectionError:
                pass

        def do_GET(self):
            if urlparse(self.path).path == "/piano/state":
                try:
                    after = int(parse_qs(urlparse(self.path).query).get("after", ["0"])[0])
                    if after < 0 or not engine.piano:
                        raise ValueError("Invalid piano event request")
                    with engine.lock:
                        result = engine.piano.snapshot(after)
                    return self.respond(200, {"ok": True, "result": result})
                except ValueError as exc:
                    return self.respond(400, {"ok": False, "error": str(exc)})
            if self.path == "/visual/model":
                with engine.lock:
                    result = engine.display_model()
                return self.respond(200, {"ok": True, "result": result})
            if self.path != "/state":
                return self.respond(404, {"ok": False, "error": "Unknown endpoint"})
            with engine.lock:
                result = engine.snapshot()
            self.respond(200, {"ok": True, "result": result})

        def do_POST(self):
            if self.path != "/command":
                return self.respond(404, {"ok": False, "error": "Unknown endpoint"})
            # Browser pages are not control clients. No CORS and no form requests.
            if self.headers.get("Origin") or self.headers.get_content_type() != "application/json":
                return self.respond(403, {"ok": False, "error": "Use a local JSON control client"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= (2_000_000 if engine.scene == "piano" else 16384):
                    raise ValueError("Invalid request size")
                self.connection.settimeout(3)
                request = json.loads(self.rfile.read(size))
                with engine.lock:
                    result = engine.command(request)
                self.respond(200, {"ok": True, "result": result})
            except (ValueError, TypeError, RuntimeError, OSError) as exc:
                self.respond(400, {"ok": False, "error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description="Persistent ideal MuJoCo simulation server")
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--scene", choices=("default", "piano"), default="default")
    parser.add_argument("--initial-state", type=Path,
                        help="Restore saved ideal arm positions before opening the viewer")
    args = parser.parse_args()
    engine = ArmServerState(scene=args.scene)
    if args.initial_state:
        try:
            saved = json.loads(args.initial_state.read_text(encoding="utf-8"))
            if saved.get("scene") != args.scene or saved.get("mode") != "ideal":
                raise ValueError("Initial state must match the ideal simulation scene")
            engine.command({"action": "move_joints", "arm": "both",
                            "q": {s: saved["arms"][s]["q"] for s in engine.SIDES}})
            # Initialize before opening the viewer or accepting any clients.
            engine.advance(max(engine.duration, 1e-6))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.exit(2, f"Cannot restore initial state: {exc}\n")
    try:
        server = create_server(engine, args.port)
    except OSError as exc:
        parser.exit(2, f"Cannot listen on 127.0.0.1:{args.port}: {exc}\n")
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    if args.scene == "piano":
        from paino_skill.scene import PIANO
        logroot = PIANO / "logs"
    else:
        logroot = Path("logs")
    logdir = logroot/f"server-{time.strftime('%Y%m%d-%H%M%S')}"
    logdir.mkdir(parents=True, exist_ok=True)
    print(f"READY http://127.0.0.1:{server.server_port} | ideal mode | waiting for controller", flush=True)
    control_stop = threading.Event()
    def score_control():
        # Render/HTTP latency must not become cumulative score timing latency.
        while not control_stop.is_set():
            tick = time.perf_counter()
            with engine.lock:
                if engine.score and engine.score.active:
                    engine.score.tick()
            control_stop.wait(max(0, 1/120-(time.perf_counter()-tick)))
    control_worker = threading.Thread(target=score_control, daemon=True, name="piano-score-clock")
    control_worker.start()
    try:
        if args.headless:
            viewer_context = nullcontext(None)
        else:
            from openarm_app.graphics import record_graphics
            import mujoco.viewer
            record_graphics(logdir)
            # The viewer owns a copy. Its synchronization never locks the score
            # executor, and it cannot race with updates to the control data.
            viewer_data, display_snapshot = mujoco.MjData(engine.model), mujoco.MjData(engine.model)
            mujoco.mj_copyData(viewer_data, engine.model, engine.data)
            viewer_context = mujoco.viewer.launch_passive(engine.model, viewer_data)
        with viewer_context as viewer:
            if viewer and args.scene == "piano":
                with viewer.lock():
                    viewer.cam.lookat[:] = [.2, 0, .42]
                    viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.75, 135, -25
                    viewer.opt.geomgroup[3] = 0
                    viewer.opt.sitegroup[:] = 0
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = False
            while viewer is None or viewer.is_running():
                tick = time.perf_counter()
                with engine.lock:
                    if not (engine.score and engine.score.active):
                        engine.advance(INTERPOLATION_PERIOD_S)
                    if viewer:
                        mujoco.mj_copyData(display_snapshot, engine.model, engine.data)
                if viewer:
                    with viewer.lock():
                        mujoco.mj_copyData(viewer_data, engine.model, display_snapshot)
                    viewer.sync()
                time.sleep(max(0, INTERPOLATION_PERIOD_S-(time.perf_counter()-tick)))
    except KeyboardInterrupt:
        pass
    finally:
        control_stop.set()
        control_worker.join(timeout=2)
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
        with engine.lock:
            if engine.state in ("EXECUTING", "PAUSED"):
                engine.command({"action": "stop"})
            (logdir/"final-state.json").write_text(json.dumps(engine.snapshot(), indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
