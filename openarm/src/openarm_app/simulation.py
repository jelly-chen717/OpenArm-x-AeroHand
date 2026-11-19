"""Ideal position replay (default) or optional MuJoCo dynamics, never hardware."""
import argparse
import csv
import json
from pathlib import Path
from queue import SimpleQueue
import time

import mujoco
import numpy as np
from openarm_app.kinematics import OpenArmIK, pose_error
from openarm_app.validation import positive


def load_trajectory(path, limits):
    expected = ["time_s"] + [f"{field}{i}_{unit}" for field, unit in
                (("q", "rad"), ("dq", "rad_s"), ("ddq", "rad_s2")) for i in range(1, 8)]
    with open(path, encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        if next(reader, None) != expected:
            raise ValueError("CSV header must match the planner's 22-column export")
        values = np.asarray(list(reader), dtype=float)
    if values.ndim != 2 or values.shape[0] < 2 or values.shape[1] != 22 or not np.all(np.isfinite(values)):
        raise ValueError("CSV requires at least two finite 22-column rows")
    if values[0, 0] != 0 or np.any(np.diff(values[:, 0]) <= 0) or values[-1, 0] > 3600:
        raise ValueError("Time must start at zero, strictly increase, and finish within 3600 seconds")
    q = values[:, 1:8]
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError("CSV positions exceed model limits")
    return values[:, 0], q


class Simulation:
    def __init__(self, path, settle_timeout=3.0, gravity_compensation=False, mode="ideal"):
        if mode not in ("ideal", "dynamic"):
            raise ValueError("mode must be ideal or dynamic")
        if mode == "ideal" and gravity_compensation:
            raise ValueError("Ideal position mode needs no gravity compensation; use --mode dynamic for that comparison")
        self.mode = mode
        self.arm = OpenArmIK()
        self.model = self.arm.setup.model
        self.model.vis.quality.shadowsize = 2048
        self.gravity_compensation = bool(gravity_compensation)
        self.gravity_data = mujoco.MjData(self.model) if self.gravity_compensation else None
        # Separate data prevents FK checks from overwriting the physical state.
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = self.arm.setup.data.qpos
        self.home_qpos = self.data.qpos.copy()
        self.data.ctrl[:] = self.arm.setup.data.ctrl
        self.times, self.positions = load_trajectory(path, self.arm.limits)
        self.resolver = self.arm.setup.joint_resolver
        self.indices = self.resolver.arm_qpos_indices("right")
        self.dofs = self.resolver.arm_dof_indices("right")
        self.resolver.set_qpos(self.data.qpos, np.r_[self.positions[0], self.arm.gripper], "right")
        self.target_pose = self.arm.forward(self.positions[-1])
        mujoco.mj_forward(self.model, self.data)
        self.dt = float(self.model.opt.timestep)
        self.timeout = positive(settle_timeout, "settle_timeout")
        self.warmup = 1.0
        self.status = "RUNNING"
        self.hold = 0.0
        self.peak = np.zeros(7)
        self.rows = []
        self.steps = 0
        self.max_contacts = 0
        self.limit_violation = False
        self.last = None

    def step(self):
        if self.status != "RUNNING":
            return
        # Ideal replay evaluates the commanded state at the new sample time.
        sample_time = self.data.time + (self.dt if self.mode == "ideal" else 0)
        trajectory_time = np.clip(sample_time-self.warmup, 0, self.times[-1])
        command = np.array([np.interp(trajectory_time, self.times, self.positions[:, i]) for i in range(7)])
        self.resolver.set_ctrl(self.data.ctrl, np.r_[command, self.arm.gripper], "right")
        previous_time = self.data.time
        if self.mode == "ideal":
            # User's ideal servo assumption: q_actual == q_command. This branch
            # advances kinematics only; gravity/torque/contact do not drive motion.
            self.data.qpos[:] = self.home_qpos
            self.resolver.set_qpos(self.data.qpos, np.r_[command, self.arm.gripper], "right")
            self.data.qvel[:] = 0
            if self.warmup < sample_time < self.warmup+self.times[-1]:
                segment = min(np.searchsorted(self.times, trajectory_time, side="right")-1, len(self.times)-2)
                self.data.qvel[self.dofs] = ((self.positions[segment+1]-self.positions[segment]) /
                                            (self.times[segment+1]-self.times[segment]))
            self.data.time = sample_time
            mujoco.mj_forward(self.model, self.data)
        elif self.gravity_data is not None:
            # At zero velocity qfrc_bias contains gravity only. Apply its opposite
            # effect as ideal feedforward; this is explicitly a diagnostic variant.
            self.gravity_data.qpos[:] = self.data.qpos
            self.gravity_data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.gravity_data)
            self.data.qfrc_applied[:] = self.gravity_data.qfrc_bias
        if self.mode == "dynamic":
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        if (not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel))
                or self.data.time <= previous_time or np.any(self.data.warning.number)):
            self.status = "FAULT"
            return
        q, dq = self.data.qpos[self.indices].copy(), self.data.qvel[self.dofs].copy()
        error = q-command
        self.peak = np.maximum(self.peak, np.abs(error))
        self.max_contacts = max(self.max_contacts, int(self.data.ncon))
        self.limit_violation |= bool(np.any(q < self.arm.limits[:, 0]-1e-4) or
                                     np.any(q > self.arm.limits[:, 1]+1e-4))
        # Recompute FK through the official solver's separate data, not simulated qpos.
        actual_pose = self.arm.kin.fk("right", np.r_[q, self.arm.gripper])
        pe, oe = pose_error(actual_pose, self.target_pose)
        self.last = dict(actual=q, error=error, velocity=dq, position_error=pe, orientation_error=oe)
        self.rows.append(np.r_[self.data.time, trajectory_time, command, q, error, dq, actual_pose, pe, oe, self.data.ncon])
        finished_commands = trajectory_time >= self.times[-1]
        joint_tolerance = 1e-12 if self.mode == "ideal" else np.deg2rad(1)
        reached = (np.max(np.abs(q-self.positions[-1])) <= joint_tolerance
                   and np.max(np.abs(dq)) <= .02 and pe <= .002 and oe <= np.deg2rad(2))
        self.hold = self.hold+self.dt if finished_commands and reached else 0.0
        if self.hold >= .25 and not self.limit_violation:
            self.status = "PASSED"
        elif self.data.time >= self.warmup+self.times[-1]+self.timeout:
            self.status = "NOT_REACHED"

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        header = ["simulation_time_s", "trajectory_time_s"]
        header += [f"{field}{i}" for field in ("target_rad_", "actual_rad_", "error_rad_", "actual_velocity_rad_s_") for i in range(1, 8)]
        header += ["ee_x", "ee_y", "ee_z", "ee_qw", "ee_qx", "ee_qy", "ee_qz",
                   "final_target_position_error_m", "final_target_orientation_error_rad", "contacts"]
        with (directory/"tracking.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self.rows)
        report = {"status": self.status, "steps": self.steps, "simulation_time_s": float(self.data.time),
                  "peak_joint_error_deg": np.rad2deg(self.peak).tolist(),
                  "joint_limit_violation": self.limit_violation, "max_contacts": self.max_contacts,
                  "mode": self.mode,
                  "controller": ("ideal position servo: actual positions assigned to commands; no dynamics integration"
                                 if self.mode == "ideal" else "official MJCF position actuators, original gains; NOT Damiao POS_VEL"),
                  "validation_scope": ("kinematic trajectory playback only; zero joint tracking error is assumed, not measured motor capability"
                                       if self.mode == "ideal" else "MuJoCo model dynamics; not Damiao motor verification"),
                  "gravity_compensation": self.gravity_compensation,
                  "acceptance": ("after final command: <=1e-12 rad joint error" if self.mode == "ideal" else "after final command: <=1 deg joint error") + ", <=0.02 rad/s, <=2 mm and <=2 deg EE error for 0.25 s; no joint limit violation",
                  "command_interpolation": "linear interpolation between CSV position samples; dq/ddq not fed to actuators"}
        if self.last:
            report.update(final_joint_error_deg=np.rad2deg(self.last["actual"]-self.positions[-1]).tolist(),
                          final_position_error_mm=self.last["position_error"]*1000,
                          final_orientation_error_deg=float(np.rad2deg(self.last["orientation_error"])))
        (directory/"summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report


def run(sim, headless, graphics_directory=None):
    if headless:
        while sim.status == "RUNNING":
            sim.step()
        return
    from openarm_app.graphics import record_graphics
    directory = graphics_directory or Path("logs")/f"graphics-{time.time_ns()}"
    record_graphics(directory)
    import mujoco.viewer
    events = SimpleQueue()
    print(f"Mode: {sim.mode}. SPACE: start/pause/resume | Q: stop/exit. Completion holds the final pose until you close the window. Initial state is paused.", flush=True)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=events.put) as viewer:
        paused = True
        result_saved = False
        exit_requested = False
        while viewer.is_running():
            started = time.perf_counter()
            while not events.empty():
                key = events.get()
                if key == 32 and sim.status == "RUNNING":
                    paused = not paused
                    print("PAUSED" if paused else "RUNNING", flush=True)
                elif key in (81, 113):
                    if sim.status == "RUNNING":
                        sim.status = "STOPPED"
                    exit_requested = True
            if exit_requested:
                break
            if not paused and sim.status == "RUNNING":
                with viewer.lock():
                    for _ in range(max(1, round(1/60/sim.dt))):
                        sim.step()
            viewer.sync()
            if sim.status != "RUNNING" and not result_saved:
                report = sim.save(directory)
                result_saved = True
                print(json.dumps(report, indent=2), flush=True)
                print(f"Result saved: {Path(directory).resolve()}\nHolding final pose. Q or close window to exit.", flush=True)
            time.sleep(max(0, 1/60-(time.perf_counter()-started)))
        if sim.status == "RUNNING":
            sim.status = "STOPPED"
        print("Simulation finished:", sim.status, flush=True)


def main():
    parser = argparse.ArgumentParser(description="Offline MuJoCo ideal position replay or dynamic comparison")
    parser.add_argument("--csv", default="trajectory.csv")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--output", default=None)
    parser.add_argument("--settle-timeout", type=float, default=3)
    parser.add_argument("--mode", choices=("ideal", "dynamic"), default="ideal")
    parser.add_argument("--gravity-compensation", action="store_true",
                        help="Diagnostic comparison: ideal body gravity compensation, not motor capability")
    args = parser.parse_args()
    directory = args.output or str(Path("logs")/time.strftime("simulation-%Y%m%d-%H%M%S"))
    try:
        sim = Simulation(args.csv, args.settle_timeout, args.gravity_compensation, args.mode)
        try:
            run(sim, args.headless, directory)
        except KeyboardInterrupt:
            if sim.status == "RUNNING":
                sim.status = "STOPPED"
        report = sim.save(directory)
        print(json.dumps(report, indent=2))
        print("Logs:", str(Path(directory).resolve()))
        return 0 if sim.status == "PASSED" else 2
    except (ValueError, OSError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
