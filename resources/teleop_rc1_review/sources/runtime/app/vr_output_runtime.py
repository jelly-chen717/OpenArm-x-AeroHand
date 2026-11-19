"""Optional output stage for the existing VR worker, not a second controller.

off/shadow preserve the existing publisher path. Verified active mode
replaces grip-held VR arm positions, through that same ROS publisher. UMI,
absolute moves, gripper Joy and A/X continue using their existing code paths.
source mode uses the original IK publication cadence with a short convex filter;
the experimental OTG pursuit limits and its tracking-error latch do not own it.
"""
import collections
import json
import math
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit

from vr_command_smoother import ArmSmoother, SourcePositionFilter, finite_vector
from vr_timing_trace import TimingTrace
from vr_joint_limits import ARM_NAMES, named_limits, verify_loaded_model
from vr_tracking_guard import TrackingGuard
import hashlib


class VROutputRuntime:
    def __init__(self, worker, config_path=None, trace_directory="/logs/vr_output",
                 start_thread=True, clock=time.monotonic_ns):
        self.worker = worker
        self.clock = clock
        self.config_path = Path(config_path or Path(__file__).with_name("vr_smoothing_config.json"))
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        if self.config.get("schema_version") != 1 or self.config.get("period_s") != .01:
            raise ValueError("Expected v1 100Hz VR output configuration")
        self.period = .01
        self.max_age_ns = int(float(self.config["target_max_age_s"])*1e9)
        if not 0 < self.max_age_ns <= 250_000_000:
            raise ValueError("Invalid VR target freshness limit")
        self.mode = self.config["mode"]
        if self.mode not in ("off", "shadow", "active", "source"):
            raise ValueError("Invalid VR output mode")
        self._trial_armed = False
        self._trial_anchor = None
        self._trial_started_ns = None
        self._trial_halted = None
        self._protection_halted = None
        self._activation_scope = "not_activated"
        self._verified_limits = None
        self._tracking = (TrackingGuard(self.config["continuous_operation"])
                          if self.config.get("continuous_operation", {}).get("enabled") else None)
        self._input_observation = {"received_ns": None, "both_valid": False,
                                   "all_released": False, "sides": {}}
        if self.mode == "active":
            self._activation_scope = self._require_profile()
            self._trial_armed = self._activation_scope == "bounded_unloaded_trial_not_hardware_certification"
        if self.mode == "source":
            self._require_source_filter()
            self._activation_scope = "original_ik_cadence_short_filter"
        self._source_filter = None
        self._source_epoch = None
        self.source_published = 0
        self.source_filter_age_ms = None
        self.source_boundary_return_count = 0
        self.source_last_boundary_returns = []
        self._core = None
        self._pending = None
        self._epoch = None
        self._feedback = collections.deque(maxlen=30)
        # Observe the *existing bridge's final position commands*, including
        # A/X. Feedback may differ under gravity; it is not the held command.
        self._bridge_commands = {side: None for side in ("left", "right")}
        self._command_observers = []
        self.last_reason = "no_target"
        self.last_error = None
        self.tick_count = 0
        self.published = 0
        self.shadow_count = 0
        self.received = 0
        self.rejected_candidates = 0
        self.last_rejected_candidate = None
        self.projected_candidates = 0
        self.last_projected_candidate = None
        self.projection_tolerance = float(self.config.get("ik_boundary_tolerance_rad", 0))
        if not math.isfinite(self.projection_tolerance) or not 0 <= self.projection_tolerance <= math.pi/180:
            raise ValueError("IK boundary tolerance must be between zero and one degree")
        self._token = secrets.token_urlsafe(32)
        self._stop = threading.Event()
        self._thread = None
        self.trace = TimingTrace(trace_directory)
        if start_thread:
            self._thread = threading.Thread(target=self._run, name="vr-output-100hz", daemon=True)
            self._thread.start()

    def attach_command_observer(self, node, message_type=None):
        """Two read-only subscriptions on the existing node, no new publisher."""
        if self._command_observers:
            raise ValueError("Bridge command observer already attached")
        if message_type is None:
            from std_msgs.msg import Float64MultiArray
            message_type = Float64MultiArray
        for side in ("left", "right"):
            subscription = node.create_subscription(message_type,
                f"/{side}_forward_position_controller/commands",
                lambda msg, side=side: self.record_bridge_command(side, msg.data), 10)
            self._command_observers.append(subscription)

    def record_bridge_command(self, side, positions):
        if side not in self._bridge_commands or self._stop.is_set():
            return False
        try:
            values = finite_vector(positions, 7, "bridge command")
            limits = self.config["profile"]["joints"]
            for i, value in enumerate(values, 1):
                limit = limits[f"openarm_{side}_joint{i}"]
                if not limit["min_position"]-1e-8 <= value <= limit["max_position"]+1e-8:
                    raise ValueError("Observed bridge command outside named model limits")
        except (ValueError, TypeError, KeyError) as exc:
            with self.worker._state_lock:
                self._bridge_commands[side] = None
            self.trace.emit("bridge_command_rejected", side=side, reason=str(exc))
            return False
        received = self.clock()
        with self.worker._state_lock:
            self._bridge_commands[side] = {"q": tuple(values), "received_ns": received}
        self.trace.emit("bridge_command_observed", side=side, q=values, received_ns=received)
        return True

    def _held_command_seed_locked(self, measured):
        seed = list(measured)
        names = self.worker.robot.actuated_joint_names
        ages = {}
        for side in ("left", "right"):
            command = self._bridge_commands[side]
            if command is None:
                raise ValueError("Awaiting original bridge hold command for "+side+
                                 "; use existing A/X reset with grips released")
            # The controller holds its last command while grips are released.
            # Do not expire that target simply because no new command is sent.
            ages[side] = (self.clock()-command["received_ns"])/1e9
            for i, value in enumerate(command["q"], 1):
                seed[names.index(f"openarm_{side}_joint{i}")] = value
        indices = [names.index(name) for name in ARM_NAMES]
        error = max(abs(seed[i]-measured[i]) for i in indices)
        protection = self.config.get("continuous_operation") or self.config.get("commissioning_trial", {})
        allowed = protection.get("max_tracking_error_rad", .06)
        if error > allowed:
            raise ValueError("Held command and feedback disagree beyond the tracking envelope")
        self.trace.emit("smooth_start_seed", seed_source="observed_original_bridge_command",
                        q=seed, measured_q=list(measured), command_age_s=ages,
                        max_command_feedback_error_rad=error, epoch=self.worker._epoch)
        return seed

    def _require_profile(self):
        profile = self.config["profile"]
        commissioned = (profile.get("validated_for_hardware") is True
                and isinstance(profile.get("commissioning_evidence"), str)
                and bool(profile["commissioning_evidence"].strip()))
        trial = self.config.get("commissioning_trial", {})
        continuous = self.config.get("continuous_operation", {})
        if continuous.get("enabled") and trial.get("enabled"):
            raise ValueError("Continuous operation and bounded trials are mutually exclusive")
        if not commissioned and not trial.get("enabled") and not continuous.get("enabled"):
            raise ValueError("Active output requires a commissioned per-joint limit profile")
        try:
            self._verified_limits = verify_loaded_model(self.worker.robot, profile)
        except (OSError, AttributeError, KeyError) as exc:
            raise ValueError("Loaded model/profile is unavailable: "+str(exc)) from exc
        if commissioned:
            return "hardware_commissioned"
        # Evidence verifies named command limits. Physical maximum capability
        # remains unclaimed in both continuous operation and bounded trials.
        proof = continuous if continuous.get("enabled") else trial
        try:
            raw = Path(proof["offline_evidence_path"]).read_bytes()
        except OSError as exc:
            raise ValueError("Commissioning evidence is unavailable: "+str(exc)) from exc
        if hashlib.sha256(raw).hexdigest() != proof["offline_evidence_sha256"]:
            raise ValueError("Commissioning evidence changed")
        evidence = json.loads(raw)
        payload = {"joints": profile["joints"], "model_urdf_sha256": profile["model_urdf_sha256"]}
        digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        scope = "offline_check_for_command_limits" if continuous.get("enabled") else "offline_check_for_bounded_unloaded_trial"
        if (evidence.get("profile_sha256") != digest or evidence.get("passed") is not True
                or evidence.get("scope") != scope
                or evidence.get("joints_checked") != list(ARM_NAMES)):
            raise ValueError("Commissioning evidence does not cover the named model/profile")
        tracking = proof.get("max_tracking_error_rad")
        if type(tracking) not in (float, int) or not math.isfinite(tracking) or not 0 < tracking <= .08:
            raise ValueError("Invalid command tracking protection")
        if any(row["max_velocity"] > 1 or row["max_acceleration"] > 2 or row["max_jerk"] > 20
               for row in self._verified_limits):
            raise ValueError("Command dynamics exceed the reviewed conservative limits")
        if continuous.get("enabled"):
            return "verified_command_limits_continuous"
        for key, cap in (("max_duration_s", 90), ("max_excursion_rad", .15),
                         ("max_target_feedback_error_rad", .15), ("max_tracking_error_rad", .08)):
            value = trial.get(key)
            if type(value) not in (float, int) or not math.isfinite(value) or not 0 < value <= cap:
                raise ValueError("Invalid bounded commissioning envelope: "+key)
        return "bounded_unloaded_trial_not_hardware_certification"

    def _require_source_filter(self):
        # Verify only named model limits here. The experimental OTG dynamic
        # envelope is deliberately not presented as the original robot's cap.
        self._verified_limits=verify_loaded_model(self.worker.robot,self.config['profile'])
        SourcePositionFilter(self.worker.robot.actuated_joint_names,**self.config.get('source_filter',{}))
        tolerance=float(self.config.get('source_feedback_boundary_tolerance_rad',0))
        if not math.isfinite(tolerance) or not 0 <= tolerance <= math.radians(2):
            raise ValueError('Source feedback boundary tolerance must be between zero and two degrees')
        self.source_feedback_boundary_tolerance=tolerance

    def _offer_source_locked(self,q,initial_q,epoch,timing,active):
        """Filter one current VR IK result; publish through the existing worker.

        Original motor permission, grip/source epoch and input loss gates remain
        authoritative. No timer, target-following excursion gate or automatic
        enable is added. Model violations and stale/nonfinite data hold output.
        """
        w=self.worker;now=self.clock();timing=dict(timing or {})
        received=timing.get('received_ns',now)
        if (not active or w._input_source!='vr' or epoch!=w._epoch
                or not w.state_container.get('motor_output_enabled',False)
                or w.state_container.get('motor_rearm',True)
                or not w.state_container.get('deadman_active',False)):
            self.cancel_locked('source_gate_or_grip_changed');return True
        if not 0<=now-received<=self.max_age_ns:
            self.cancel_locked('input_expired_during_ik');return True
        try:
            measured=w._feedback_snapshot_locked(.25)
            names=list(w.robot.actuated_joint_names)
            feedback=finite_vector(measured['q'],len(names),'feedback')
            goal=finite_vector(q,len(names),'IK candidate')
            limits=self.config['profile']['joints'];projected=[];boundary_returns=[]
            for name in ARM_NAMES:
                i=names.index(name);bound=limits[name]
                if not bound['min_position']<=feedback[i]<=bound['max_position']:
                    nearest=max(bound['min_position'],min(bound['max_position'],feedback[i]))
                    overrun=abs(feedback[i]-nearest)
                    if overrun>self.source_feedback_boundary_tolerance+1e-12:
                        raise ValueError('measured_joint_outside_model:'+name)
                    # A valid command inside the unchanged model interval can
                    # bring a slightly overshooting servo back. Rejecting that
                    # command would strand both arms at the held boundary.
                    # Never clamp the measured value or expand a target limit.
                    boundary_returns.append(dict(joint=name,feedback_rad=feedback[i],
                        boundary_rad=nearest,overrun_rad=overrun))
                value=max(bound['min_position'],min(bound['max_position'],goal[i]))
                if abs(value-goal[i])>self.projection_tolerance:
                    raise ValueError('IK_candidate_outside_model:'+name)
                if value!=goal[i]:projected.append(name)
                goal[i]=value
            if self._source_filter is None or self._source_epoch!=epoch:
                self._source_filter=SourcePositionFilter(names,**self.config.get('source_filter',{}))
                # Retain the held bridge command during initial filter filling.
                # Do not reintroduce the OTG's static-error regrip lock here.
                seed=goal[:];complete=True
                for side in ('left','right'):
                    command=self._bridge_commands[side]
                    if command is None:complete=False;break
                    for i,v in enumerate(command['q'],1):seed[names.index(f'openarm_{side}_joint{i}')]=v
                self._source_filter.reset(seed if complete else None,now)
                self._source_epoch=epoch
            filtered,self.source_filter_age_ms=self._source_filter.update(goal,now)
            # Convex averaging cannot expand the model interval. Keep an
            # explicit publication check for future implementation changes.
            for name in ARM_NAMES:
                bound=limits[name];i=names.index(name);value=filtered[i]
                if not bound['min_position']-1e-9<=value<=bound['max_position']+1e-9:
                    raise ValueError('filtered_candidate_outside_model:'+name)
                # Normalize convex-average roundoff only; the allowance for
                # measured feedback never applies to commanded positions.
                filtered[i]=max(bound['min_position'],min(bound['max_position'],value))
            for item in boundary_returns:
                command=filtered[names.index(item['joint'])]
                if ((command-item['feedback_rad'])*(item['boundary_rad']-item['feedback_rad'])<=0):
                    raise ValueError('source_boundary_return_not_inward:'+item['joint'])
            w.publish_smoothed_locked(filtered)
            self.source_published+=1
            self.source_boundary_return_count+=bool(boundary_returns)
            self.source_last_boundary_returns=boundary_returns
            self.last_reason='source_filtered';self.last_error=None
            self.trace.emit('source_filter_publish',**timing,epoch=epoch,q=filtered,raw_target=goal,
                joint_names=names,projected_joints=projected,filter_history_age_ms=self.source_filter_age_ms,
                feedback_boundary_returns=boundary_returns,
                source_target_age_ms=(now-received)/1e6)
        except (ValueError,RuntimeError,IndexError,TypeError) as exc:
            self.cancel_locked('source_invalid_or_stale_hold');self.last_error=str(exc)
            self.trace.emit('source_filter_hold',reason=str(exc),epoch=epoch)
        return True

    def _output_check_locked(self, target=None):
        continuous = self._activation_scope == "verified_command_limits_continuous"
        if not self._trial_armed and not continuous:
            return True
        if self._trial_halted or self._protection_halted:
            return False
        now = self.clock()
        trial = self.config.get("commissioning_trial", {})
        protection = self.config["continuous_operation"] if continuous else trial
        try:
            snapshot = self.worker._feedback_snapshot_locked(.25)
            names = self.worker.robot.actuated_joint_names
            measured = finite_vector(snapshot["q"], len(names), "feedback")
            indices = [names.index(name) for name in ARM_NAMES]
            limits = self.config["profile"]["joints"]
            for name, index in zip(ARM_NAMES, indices):
                if not limits[name]["min_position"] <= measured[index] <= limits[name]["max_position"]:
                    raise ValueError("measured_joint_outside_model:"+name)
            if self._trial_armed and self._trial_anchor is None:
                history = [q for stamp, q in self._feedback if now-stamp <= 150_000_000]
                if len(history) < 10 or now-self._feedback[-1][0] > 30_000_000:
                    self.last_reason = "trial_awaiting_stationary_feedback"
                    return False
                if any(max(q[i] for q in history)-min(q[i] for q in history) > .002 for i in indices):
                    self.last_reason = "trial_awaiting_stationary_feedback"
                    return False
                self._trial_anchor = tuple(measured)
                self._trial_started_ns = now
                self.trace.emit("trial_started", mode=self.mode, anchor=list(measured),
                                max_duration_s=trial["max_duration_s"])
            if self._trial_armed and now-self._trial_started_ns > trial["max_duration_s"]*1e9:
                raise ValueError("trial_time_elapsed")
            if self._trial_armed and any(abs(measured[i]-self._trial_anchor[i]) > trial["max_excursion_rad"] for i in indices):
                raise ValueError("trial_measured_excursion")
            if target is not None:
                target = [float(v) for v in target]
                if len(target) != len(names) or not all(math.isfinite(v) for v in target):
                    raise ValueError("trial_nonfinite_target")
                for name, index in zip(ARM_NAMES, indices):
                    if not limits[name]["min_position"] <= target[index] <= limits[name]["max_position"]:
                        raise ValueError("trial_target_outside_model:"+name)
                    if self._trial_armed and abs(target[index]-self._trial_anchor[index]) > trial["max_excursion_rad"]:
                        raise ValueError("trial_target_excursion:"+name)
                    if self._trial_armed and abs(target[index]-measured[index]) > trial["max_target_feedback_error_rad"]:
                        raise ValueError("trial_target_feedback_error:"+name)
            if continuous and self.mode == "active":
                reference = self._tracking.reference
                if reference is None and self._core is not None:
                    reference = self._core.q
                if reference is not None:
                    before = self._tracking.phase
                    phase = self._tracking.observe(reference, measured, names, indices, now,
                        self._stationary_locked(now),
                        command_velocity=self._core.v if self._core is not None else None,
                        measured_velocity=self._feedback_velocity_locked(now))
                    if phase != before:
                        self.trace.emit("tracking_phase", **self._tracking.snapshot())
                    if phase == "latched":
                        raise ValueError("tracking_error_persistent")
                    if phase != "normal":
                        self.cancel_locked("tracking_"+phase)
                        return False
            elif self._core is not None and self.mode == "active":
                if any(abs(self._core.q[i]-measured[i]) > protection["max_tracking_error_rad"] for i in indices):
                    raise ValueError("command_tracking_error" if continuous else "trial_command_tracking_error")
            return True
        except (ValueError, RuntimeError, IndexError) as exc:
            if continuous:
                self._protection_halted = str(exc)
                self.cancel_locked("protection_hold:"+str(exc))
                self.trace.emit("protection_hold", mode=self.mode, reason=str(exc),
                                tracking=self._tracking.snapshot() if self._tracking else None)
            else:
                self._trial_halted = str(exc)
                self.cancel_locked("bounded_trial_hold:"+str(exc))
                self.trace.emit("trial_hold", mode=self.mode, reason=str(exc))
            return False

    def _feedback_velocity_locked(self, now):
        """Short measured-position history; stale/gapped samples give no allowance.

        Independent of the commanded velocity. A dropped packet or stalled
        joint must never be mistaken for progress from a requested trajectory.
        """
        history = [(stamp,q) for stamp,q in self._feedback if 0 <= now-stamp <= 80_000_000]
        if (len(history) < 5 or now-history[-1][0] > 30_000_000
                or history[-1][0]-history[0][0] < 40_000_000
                or any(not 0 < b[0]-a[0] <= 30_000_000 for a,b in zip(history,history[1:]))):
            return None
        dt=(history[-1][0]-history[0][0])/1e9
        velocity=[(b-a)/dt for a,b in zip(history[0][1],history[-1][1])]
        if len(velocity) != len(self.worker.robot.actuated_joint_names) or not all(math.isfinite(v) for v in velocity):
            return None
        return velocity

    def _stationary_locked(self, now):
        history = [q for stamp, q in self._feedback if 0 <= now-stamp <= 150_000_000]
        if len(history) < 10 or not self._feedback or not 0 <= now-self._feedback[-1][0] <= 30_000_000:
            return False
        indices = [self.worker.robot.actuated_joint_names.index(n) for n in ARM_NAMES]
        return all(max(q[i] for q in history)-min(q[i] for q in history) <= .002 for i in indices)

    def _recovery_issue_locked(self, manual=False):
        if self.worker._input_source != "vr":
            return "select_vr_source"
        observation = self._input_observation
        if (observation["received_ns"] is None
                or not 0 <= self.clock()-observation["received_ns"] <= self.max_age_ns
                or not observation["both_valid"]):
            return "restore_both_controller_poses"
        if not observation["all_released"]:
            return "release_all_controller_buttons"
        try:
            snap = self.worker._feedback_snapshot_locked(.25)
            measured = finite_vector(snap["q"], len(self.worker.robot.actuated_joint_names), "feedback")
            for name in ARM_NAMES:
                value = measured[self.worker.robot.actuated_joint_names.index(name)]
                limit = self.config["profile"]["joints"][name]
                if not limit["min_position"] <= value <= limit["max_position"]:
                    return "measured_joint_outside_model"
            if not self._stationary_locked(self.clock()):
                return "wait_for_stationary_feedback"
            seed = self._held_command_seed_locked(measured)
            names = self.worker.robot.actuated_joint_names
            # Release recovery requires the guard's stationary qualification:
            # tight hysteresis, or a longer stable residual strictly below the
            # unchanged soft stop threshold. Neither route advances a command.
            # Explicit, acknowledged recovery also stays below that stop limit.
            # It uses the *current observed bridge command*, never the old
            # collision reference, and still cannot publish or enable motion.
            errors = [abs(seed[names.index(n)]-measured[names.index(n)]) for n in ARM_NAMES]
            if (any(error >= self._tracking.limit for error in errors) if manual
                    else any(not self._tracking.release_error_allowed(error) for error in errors)):
                return "held_command_feedback_error_not_settled"
        except (ValueError, RuntimeError):
            return "held_command_or_feedback_unavailable"
        return None

    def _finish_recovery_locked(self, method):
        # Invalidate in-flight IK, discard old targets, and require a new grip.
        # No motor enable or IK gate is changed, and no position is published.
        self._protection_halted = None
        release_basis = self._tracking.release_basis
        self._tracking.reset()
        self.worker._epoch += 1
        self.worker._clear_ingresses_locked()
        self.last_reason = "recovered_regrip_required"
        self.trace.emit("tracking_recovered", method=method, release_basis=release_basis,
                        epoch=self.worker._epoch)

    def recover(self, hardware):
        """Explicit webpage acknowledgement; validated against current read-only health."""
        with self.worker._state_lock:
            if not self._tracking or self._protection_halted not in ("command_tracking_error", "tracking_error_persistent"):
                raise ValueError("No recoverable tracking latch")
            if (hardware.get("online") != 16 or hardware.get("enabled") != 16
                    or hardware.get("faulted") != 0 or not hardware.get("feedback_fresh")
                    or hardware.get("busy") or hardware.get("busy_external")
                    or hardware.get("hardware_states") != {
                        "openarm_left_hardware_interface": 3, "openarm_right_hardware_interface": 3}):
                raise ValueError("Healthy manually enabled hardware feedback is required")
            issue = self._recovery_issue_locked(manual=True)
            if issue:
                raise ValueError(issue)
            self._finish_recovery_locked("web_acknowledgement")
        return self.snapshot()

    def note_input(self, payload):
        timing = payload.get("_oa_timing", {}) if isinstance(payload, dict) else {}
        now = self.clock()
        received = timing.get("host_received_monotonic_ns", now)
        # Only the server creates _oa_timing. Clamp malformed internal values;
        # client timestamps are never a motion deadline or a ROS/TF timestamp.
        if type(received) is not int or not 0 <= now-received < 60_000_000_000:
            received = now
        with self.worker._state_lock:
            self.received += 1
            token = {"input_sequence": self.received, "received_ns": received,
                     "callback_ns": now}
        sides = {d.get("handedness"): bool(d.get("gripPose"))
                 for d in payload.get("devices", []) if isinstance(d, dict)
                 and d.get("role") == "controller"}
        observations = {}
        for side in ("left", "right"):
            devices = [d for d in payload.get("devices", []) if isinstance(d, dict)
                       and d.get("role") == "controller" and d.get("handedness") == side]
            device = devices[0] if len(devices) == 1 else {}
            gamepad = device.get("gamepad")
            buttons = gamepad.get("buttons", []) if isinstance(gamepad, dict) else []
            valid_buttons = (isinstance(buttons, list) and 2 <= len(buttons) <= 16 and all(isinstance(b, dict)
                and type(b.get("pressed")) is bool and type(b.get("value")) in (float, int)
                and math.isfinite(b["value"]) and 0 <= b["value"] <= 1 for b in buttons))
            diag = device.get("input_diagnostic") or {}
            observations[side] = {"pose_valid": bool(device.get("gripPose")),
                "buttons_valid": valid_buttons,
                # XR controllers may expose capacitive/auxiliary value=1 while
                # pressed=False. Only trigger (0) and squeeze (1) are analog
                # release gates; every actual pressed button still blocks.
                "released": valid_buttons and all(not b["pressed"] for b in buttons)
                    and all(b["value"] < .1 for b in buttons[:2]),
                "pressed_buttons": [i for i,b in enumerate(buttons) if b["pressed"]] if valid_buttons else None,
                "trigger_grip_values": [b["value"] for b in buttons[:2]] if valid_buttons else None,
                "collector_reason": str(diag.get("reason", "legacy_collector"))[:80],
                "source_count": diag.get("source_count") if type(diag.get("source_count")) is int else None,
                "visibility": str(diag.get("visibility", "unknown"))[:32]}
        with self.worker._state_lock:
            self._input_observation = {"received_ns": received,
                "both_valid": all(d["pose_valid"] and d["buttons_valid"] for d in observations.values()),
                "all_released": all(d["released"] for d in observations.values()), "sides": observations}
            if (self._tracking and self._tracking.phase == "awaiting_release"
                    and not self._protection_halted
                    and self._tracking.release_ready_ns is not None
                    and self._tracking.release_ready_ns <= received
                    and self._recovery_issue_locked() is None):
                self._finish_recovery_locked("fresh_released_controllers")
        self.trace.emit("xr_receive", **token, valid_grips=sides,
                        controller_observation=observations,
                        client_timestamp_unix_ms=timing.get("client_timestamp_unix_ms"),
                        receive_to_callback_ms=(now-received)/1e6)
        return token

    def feedback_locked(self):
        # Called only after the worker updated a complete measured snapshot.
        try:
            snap = self.worker._feedback_snapshot_locked(.25)
            self._feedback.append((self.clock(), tuple(snap["q"])))
            self.trace.emit("joint_feedback", feedback_sequence=snap["feedback_seq"],
                            epoch=self.worker._epoch, q=snap["q"],
                            joint_names=snap["joint_names"],
                            feedback_received_ns=snap["received_monotonic_ns"])
        except (ValueError, RuntimeError):
            pass

    def cancel_locked(self, reason):
        had_target = self._pending is not None or self._core is not None or self._source_filter is not None
        if self._tracking and self._tracking.phase == "normal":
            self._tracking.clear_pending()
        self._pending = None
        self._core = None
        self._epoch = None
        self._source_filter = None
        self._source_epoch = None
        self.last_reason = reason
        if had_target:
            self.trace.emit("cancel", reason=reason, epoch=self.worker._epoch)

    def offer_locked(self, q, initial_q, epoch, timing, active):
        """Return True when this stage owns publication (including active errors)."""
        if self.mode == 'source':
            return self._offer_source_locked(q,initial_q,epoch,timing,active)
        takeover = self.mode == "active"
        if takeover and active and self._activation_scope == "verified_command_limits_continuous":
            # An upstream IK candidate is not an issued hardware command.
            # Project only small solver residuals onto the original goal limits,
            # before trajectory generation. Never clip a published command or
            # expand hardware limits. Larger invalid goals retain the old age.
            try:
                candidate = finite_vector(q, len(self.worker.robot.actuated_joint_names), "IK candidate")
            except (ValueError, TypeError, OverflowError):
                candidate = None  # Existing malformed/nonfinite protection below.
            if candidate is not None:
                violations = []
                for name in ARM_NAMES:
                    index = self.worker.robot.actuated_joint_names.index(name)
                    limit = self.config["profile"]["joints"][name]
                    if not limit["min_position"] <= candidate[index] <= limit["max_position"]:
                        violations.append(dict(joint=name, target_rad=candidate[index],
                            lower_rad=limit["min_position"], upper_rad=limit["max_position"],
                            projected_rad=max(limit["min_position"], min(limit["max_position"], candidate[index]))))
                if violations and all(abs(v["target_rad"]-v["projected_rad"]) <= self.projection_tolerance for v in violations):
                    for v in violations:
                        candidate[self.worker.robot.actuated_joint_names.index(v["joint"])] = v["projected_rad"]
                    q = candidate
                    self.projected_candidates += 1
                    self.last_projected_candidate = dict(received_ns=self.clock(), violations=violations)
                    self.trace.emit("ik_candidate_projected", **self.last_projected_candidate, epoch=epoch)
                    violations = []
                if violations:
                    # Measured-limit, feedback-age and actual tracking protection
                    # still run, even when every candidate is rejected.
                    if not self._output_check_locked():
                        return True
                    self.rejected_candidates += 1
                    self.last_rejected_candidate = dict(received_ns=self.clock(), violations=violations)
                    self.last_reason = "ik_candidate_rejected:"+violations[0]["joint"]
                    self.trace.emit("ik_candidate_rejected", **self.last_rejected_candidate,
                        epoch=epoch, previous_valid_target_retained=self._pending is not None,
                        previous_valid_received_ns=self._pending["received_ns"] if self._pending else None)
                    return True
        if not self._output_check_locked(q):
            # In an explicitly armed comparison, shadow/raw and active use the
            # same excursion/tracking envelope. A refusal is never a raw bypass.
            return True
        if self.mode == "off" or not active:
            self.cancel_locked("off" if self.mode == "off" else "ik_inactive")
            return takeover
        now = self.clock()
        timing = dict(timing or {})
        received_ns = timing.get("received_ns", now)
        if now-received_ns > self.max_age_ns:
            self.cancel_locked("input_expired_during_ik")
            return takeover
        try:
            if self._core is None or self._epoch != epoch:
                core = ArmSmoother(self.worker.robot.actuated_joint_names, self.config["profile"], self.period,
                                   tracking=self.config.get("streaming_tracking"))
                seed = initial_q
                if takeover:
                    # A new grip/source epoch may not assume zero dynamics while
                    # the arm is moving. Require fresh measured rest before seed.
                    measured = self.worker._feedback_snapshot_locked(.25)
                    history = [q for stamp, q in self._feedback if now-stamp <= 150_000_000]
                    if len(history) < 10 or not self._feedback or now-self._feedback[-1][0] > 30_000_000:
                        raise ValueError("Awaiting fresh stationary feedback for active initialization")
                    arm_indices = sum(core.groups, [])
                    if any(max(q[i] for q in history)-min(q[i] for q in history) > .002
                           for i in arm_indices):
                        raise ValueError("Arm still moving; active initialization held")
                    # Continue from the command that the original controller is
                    # holding. Starting from measured q inserts the static servo
                    # error as an instantaneous new position command at regrip.
                    seed = self._held_command_seed_locked(measured["q"])
                core.reset(seed)
                self._core = core
                self._epoch = epoch
            self._core.retarget(q)
            self._pending = {"epoch": epoch, "timing": timing, "q": tuple(float(v) for v in q),
                             "received_ns": received_ns, "offered_ns": now}
            self.last_reason = "tracking_shadow" if self.mode == "shadow" else "tracking_active"
            self.last_error = None
        except Exception as exc:
            self.cancel_locked("smoothing_error_hold")
            self.last_error = str(exc)
            self.trace.emit("smooth_error", error=str(exc), mode=self.mode, epoch=epoch)
        return takeover

    def tick_locked(self, lateness_ms=0):
        """One tick under the *same* epoch lock as admission and publication."""
        if self.mode == 'source':
            return  # Original IK callbacks own publication; no second cadence.
        if self._tracking and self._tracking.reference is not None and not self._protection_halted:
            self._output_check_locked()
        pending = self._pending
        if not pending or self.mode == "off":
            return
        w = self.worker
        now = self.clock()
        if (w._input_source != "vr" or pending["epoch"] != w._epoch
                or not w.state_container.get("motor_output_enabled", False)
                or w.state_container.get("motor_rearm", True)
                or not w.state_container.get("deadman_active", False)):
            self.cancel_locked("source_gate_or_grip_changed")
            return
        if now-pending["received_ns"] > self.max_age_ns:
            self.cancel_locked("target_stale_hold")
            return
        if not self._output_check_locked(pending["q"]):
            return
        started = self.clock()
        try:
            if self.mode == "active":
                w._feedback_snapshot_locked(.25)
            q, v, a = self._core.step()
            self.tick_count += 1
            fields = dict(pending["timing"], epoch=w._epoch, mode=self.mode,
                          profile=self.config["profile"]["id"], q=q, velocity=v, acceleration=a,
                          joint_names=list(w.robot.actuated_joint_names),
                          raw_target=list(pending["q"]),
                          target_age_ms=(now-pending["received_ns"])/1e6,
                          tick_lateness_ms=max(0, lateness_ms),
                          compute_ms=(self.clock()-started)/1e6)
            if self.mode == "active":
                # Uses the existing worker's publisher only, never a CAN client.
                # Finger positions in the message are not a new Joy command.
                w.publish_smoothed_locked(q)
                self.published += 1
                self.trace.emit("smooth_publish", **fields)
            else:
                self.shadow_count += 1
                self.trace.emit("smooth_shadow", **fields)
        except Exception as exc:
            self.cancel_locked("smoothing_error_hold")
            self.last_error = str(exc)
            self.trace.emit("smooth_error", error=str(exc), mode=self.mode, epoch=w._epoch)

    def _run(self):
        deadline = time.monotonic()+self.period
        while not self._stop.wait(max(0, deadline-time.monotonic())):
            now = time.monotonic()
            with self.worker._state_lock:
                self.tick_locked((now-deadline)*1000)
            # Skip missed deadlines; never burst old samples to catch up.
            deadline = time.monotonic()+self.period

    def set_mode(self, mode):
        if mode not in ("off", "shadow", "active", "source"):
            raise ValueError("Mode must be off, shadow, active or source")
        with self.worker._state_lock:
            if (self.worker.state_container.get("motor_output_enabled", False)
                    or self.worker.state_container.get("deadman_active", False)
                    or self.worker.state_container.get("active", False)):
                raise ValueError("Change output mode only with motor/IK output gate closed and grips released")
            if mode == 'source':
                self._require_source_filter()
                scope='original_ik_cadence_short_filter'
            elif mode == "active":
                scope = self._require_profile()
                # Verify local extension and profile before taking ownership.
                ArmSmoother(self.worker.robot.actuated_joint_names, self.config["profile"], self.period,
                            tracking=self.config.get("streaming_tracking"))
            elif mode == "shadow" and self.config.get("commissioning_trial", {}).get("enabled"):
                scope = self._require_profile()
            else:
                scope = "not_activated"
            self.cancel_locked("mode_changed")
            self._activation_scope = scope
            self._trial_armed = self._activation_scope == "bounded_unloaded_trial_not_hardware_certification"
            self._trial_anchor = None
            self._trial_started_ns = None
            self._trial_halted = None
            self._protection_halted = None
            if self._tracking:
                self._tracking.reset()
            self.mode = mode
            self.trace.emit("mode_changed", mode=mode)
        return self.snapshot()

    def snapshot(self, include_token=False):
        with self.worker._state_lock:
            result = {"mode": self.mode, "profile": self.config["profile"]["id"],
                      "hardware_profile_validated": self.config["profile"]["validated_for_hardware"],
                      "activation_scope": self._activation_scope,
                      "continuous_operation": self._activation_scope in ("verified_command_limits_continuous", "original_ik_cadence_short_filter"),
                      "source_following": {"enabled":self.mode=='source',
                          "publication":"original_ik_cadence", "filter":self.config.get('source_filter'),
                          "published":self.source_published,"history_age_ms":self.source_filter_age_ms,
                          "feedback_boundary_tolerance_rad":self.source_feedback_boundary_tolerance,
                          "boundary_return_publishes":self.source_boundary_return_count,
                          "last_boundary_returns":self.source_last_boundary_returns,
                          "otg_velocity_acceleration_jerk_caps_applied":False,
                          "otg_tracking_error_lock_applied":False,
                          "original_motor_and_input_gates_unchanged":True} if self.mode=='source' else None,
                      "protection": {"halted": self._protection_halted,
                                     "max_tracking_error_rad": self.config.get("continuous_operation", {}).get("max_tracking_error_rad") if self.mode!='source' else None,
                                     "tracking": self._tracking.snapshot() if self._tracking and self.mode!='source' else None},
                      "controller_input": dict(self._input_observation,
                          age_s=((self.clock()-self._input_observation["received_ns"])/1e9
                                 if self._input_observation["received_ns"] is not None else None)),
                      "input_source": self.worker._input_source,
                      "motion_permission": bool(self.worker.state_container.get("motor_output_enabled", False)),
                      "named_limits_verified": self._verified_limits is not None,
                      "limits": self._verified_limits,
                      "trial": {"armed": self._trial_armed,
                                "started": self._trial_started_ns is not None,
                                "halted": self._trial_halted,
                                "elapsed_s": (self.clock()-self._trial_started_ns)/1e9 if self._trial_started_ns else None},
                      "handoff_seed": "observed_original_bridge_command",
                      "bridge_command_observer": {
                          side: {"ready": command is not None,
                                 "age_s": (self.clock()-command["received_ns"])/1e9 if command else None}
                          for side, command in self._bridge_commands.items()},
                      "command_period_s": self.period if self.mode!='source' else None, "last_reason": self.last_reason,
                      "blocking_reason": (self._protection_halted or
                          ("tracking_"+self._tracking.phase if self._tracking and self._tracking.phase != "normal" else self.last_reason)),
                      "last_error": self.last_error, "pending_target": self._pending is not None,
                      "rejected_ik_candidates": self.rejected_candidates,
                      "last_rejected_ik_candidate": self.last_rejected_candidate,
                      "projected_ik_candidates": self.projected_candidates,
                      "last_projected_ik_candidate": self.last_projected_candidate,
                      "ik_boundary_tolerance_rad": self.projection_tolerance,
                      "streaming_tracking": self.config.get("streaming_tracking", {}) if self.mode!='source' else None,
                      "shadow_ticks": self.shadow_count, "active_publishes": self.published+self.source_published,
                      "takeover": self.mode in ("active", "source"),
                      "startup_mode": self.config["mode"], "mode_changes_persisted": False,
                      "raw_path_unchanged": self.mode in ("off", "shadow")}
        result["timing"] = self.trace.snapshot()
        if include_token:
            result["csrf_token"] = self._token
        return result

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.trace.close()


def install(app, runtime):
    import asyncio
    from fastapi import HTTPException, Request
    from fastapi.routing import APIRoute

    def check_origin(request):
        origin = request.headers.get("origin")
        fetch_site = request.headers.get("sec-fetch-site")
        if fetch_site == "cross-site" or (origin and
                (urlsplit(origin).scheme, urlsplit(origin).netloc) !=
                (request.url.scheme, request.url.netloc)):
            raise HTTPException(403, "Same-origin request required")

    async def status(request: Request):
        check_origin(request)
        from fastapi.responses import JSONResponse
        return JSONResponse(runtime.snapshot(include_token=True), headers={"Cache-Control": "no-store"})

    async def events(request: Request, limit: int = 100):
        check_origin(request)
        return {"events": runtime.trace.events(limit)}

    async def mode(request: Request):
        check_origin(request)
        if not secrets.compare_digest(request.headers.get("x-vr-output-token", ""), runtime._token):
            raise HTTPException(403, "VR output token required")
        # Bound the body even for chunked requests, before JSON decoding.
        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > 256:
                raise HTTPException(413, "Body too large")
        try:
            value = json.loads(body)
            if not isinstance(value, dict) or set(value) != {"mode"}:
                raise ValueError("Expected only mode")
            return runtime.set_mode(value["mode"])
        except (ValueError, TypeError, KeyError) as exc:
            raise HTTPException(409, str(exc))

    async def recover(request: Request):
        check_origin(request)
        if not secrets.compare_digest(request.headers.get("x-vr-output-token", ""), runtime._token):
            raise HTTPException(403, "VR output token required")
        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > 256:
                raise HTTPException(413, "Body too large")
        try:
            value = json.loads(body)
            if not isinstance(value, dict) or set(value) != {"acknowledge_clear"} or value["acknowledge_clear"] is not True:
                raise ValueError("Explicit clear-workspace acknowledgement required")
            from motor_web import proxy
            hardware, code = await asyncio.to_thread(proxy, "state")
            if code != 200:
                raise ValueError("Hardware status unavailable")
            return runtime.recover(hardware)
        except (ValueError, TypeError, KeyError) as exc:
            raise HTTPException(409, str(exc))

    for path, handler, methods in (("status", status, ["GET"]), ("events", events, ["GET"]),
                                   ("mode", mode, ["POST"]), ("recover", recover, ["POST"])):
        app.router.routes.insert(0, APIRoute("/api/teleop/vr-output/"+path, handler, methods=methods))
    # Host receive_text() can itself be delayed by a busy ASGI loop. Observe
    # that separately; a fast callback alone does not exonerate the server loop.
    loop_task = None

    async def watch_event_loop():
        loop = asyncio.get_running_loop()
        while True:
            deadline = loop.time()+.02
            await asyncio.sleep(.02)
            runtime.trace.emit("event_loop_heartbeat",
                               event_loop_lag_ms=max(0, (loop.time()-deadline)*1000))

    async def start_watch():
        nonlocal loop_task
        loop_task = asyncio.create_task(watch_event_loop())

    async def shutdown():
        if loop_task:
            loop_task.cancel()
            try:
                await loop_task
            except asyncio.CancelledError:
                pass
        await asyncio.to_thread(runtime.close)

    app.router.add_event_handler("startup", start_watch)
    app.router.add_event_handler("shutdown", shutdown)
