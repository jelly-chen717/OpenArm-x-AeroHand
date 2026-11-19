"""Tracking pause policy; observes commands/feedback and never publishes motion."""
import math

WRIST_NAMES = frozenset(f"openarm_{side}_joint{j}" for side in ("left", "right") for j in (5, 6, 7))


class TrackingGuard:
    def __init__(self, settings):
        self.limit = settings["max_tracking_error_rad"]
        self.resume_limit = settings["tracking_resume_error_rad"]
        self.timeout_ns = int(settings["tracking_settle_timeout_s"] * 1e9)
        self.stable_ns = int(settings["tracking_settle_stable_s"] * 1e9)
        self.hard_limit = settings.get("max_tracking_error_hard_rad", self.limit)
        self.debounce_ns = int(settings.get("tracking_error_debounce_s", 0) * 1e9)
        # A held arm may settle above the tighter hysteresis threshold because
        # of load/servo bias. A separate, longer stationary window can permit
        # release/regrip strictly BELOW the existing soft stop threshold.
        # This never subtracts a learned bias from the live tracking error.
        self.settled_release_limit = settings.get("tracking_settled_release_error_rad", self.resume_limit)
        self.settled_release_ns = int(settings.get("tracking_settled_release_stable_s",
                                                   settings["tracking_settle_stable_s"]) * 1e9)
        self.wrist_lag_s = settings.get("tracking_dynamic_wrist_lag_s", 0)
        if not (0 < self.resume_limit < self.limit <= .08
                and self.limit <= self.hard_limit <= .08
                and self.resume_limit <= self.settled_release_limit <= self.limit
                and 0 <= self.wrist_lag_s <= .05
                and 0 <= self.debounce_ns <= 150_000_000
                and 0 < self.stable_ns <= self.settled_release_ns < self.timeout_ns <= 1_000_000_000):
            raise ValueError("Invalid tracking pause/recovery settings")
        self.reset()

    def reset(self):
        self.phase = "normal"
        self.reference = None
        self.since_ns = None
        self.settled_since_ns = None
        self.residual_since_ns = None
        self.release_ready_ns = None
        self.release_basis = None
        self.detail = None
        self._soft_since = {}
        self.pending = []
        self.dynamic_thresholds = []

    def clear_pending(self):
        """A released grip/source epoch must not inherit a soft-error timer."""
        self._soft_since.clear()
        self.pending = []
        self.dynamic_thresholds = []

    def soft_limit(self, name, error, command_velocity=None, measured_velocity=None):
        """Only actual same-direction wrist progress permits bounded servo lag.

        Error is signed command minus feedback. A stalled/opposing joint, a
        stopped reference, an ahead-of-reference joint, or missing velocity
        uses the unchanged static threshold. This is not torque compensation.
        """
        if not self.wrist_lag_s or name not in WRIST_NAMES:
            return self.limit
        if (command_velocity is None or measured_velocity is None
                or not all(math.isfinite(v) for v in (error, command_velocity, measured_velocity))
                or command_velocity*measured_velocity <= 0
                or error*command_velocity <= 0
                or min(abs(command_velocity), abs(measured_velocity)) < .02):
            return self.limit
        return min(self.hard_limit, self.limit+self.wrist_lag_s*
                   min(abs(command_velocity), abs(measured_velocity)))

    def observe(self, command, measured, names, indices, now, stationary=False,
                command_velocity=None, measured_velocity=None):
        """Hold the *last published* command; do not follow newer IK targets.

        A brief soft-limit crossing is tolerated while outputs remain bounded.
        The hard limit pauses immediately; sustained soft error also pauses.
        Durations are per joint, not a timer reset by another joint becoming
        the largest error. This is not a collision detector.
        """
        if self.phase == "latched":
            return self.phase
        reference = self.reference if self.reference is not None else command
        if len(reference) != len(measured) or not all(math.isfinite(v) for v in reference):
            raise ValueError("Invalid held tracking reference")
        index = max(indices, key=lambda i: abs(reference[i] - measured[i]))
        error = abs(reference[index] - measured[index])
        self.detail = dict(joint=names[index], command_rad=reference[index],
                           feedback_rad=measured[index], error_rad=error)
        if self.phase == "normal":
            self.pending = []
            self.dynamic_thresholds = []
            persistent = []
            for i in indices:
                current_error = abs(reference[i]-measured[i])
                v_cmd = command_velocity[i] if command_velocity is not None and len(command_velocity)==len(names) else None
                v_meas = measured_velocity[i] if measured_velocity is not None and len(measured_velocity)==len(names) else None
                threshold = self.soft_limit(names[i], reference[i]-measured[i], v_cmd, v_meas)
                if threshold > self.limit:
                    self.dynamic_thresholds.append(dict(joint=names[i], threshold_rad=threshold,
                        command_velocity_rad_s=v_cmd, measured_velocity_rad_s=v_meas))
                if current_error > threshold:
                    since = self._soft_since.setdefault(i, now)
                    elapsed = max(0, now-since)
                    self.pending.append(dict(joint=names[i],error_rad=current_error,
                                             threshold_rad=threshold,elapsed_s=elapsed/1e9))
                    if elapsed >= self.debounce_ns:
                        persistent.append(i)
                else:
                    self._soft_since.pop(i, None)
            if error <= self.hard_limit and not persistent:
                return self.phase
            if error <= self.hard_limit:
                index = max(persistent, key=lambda i: now-self._soft_since[i])
                error = abs(reference[index]-measured[index])
                self.detail = dict(joint=names[index],command_rad=reference[index],
                                   feedback_rad=measured[index],error_rad=error)
            self.pending = []
            self.dynamic_thresholds = []
            self._soft_since.clear()
            self.reference = tuple(reference)
            self.since_ns = now
            self.phase = "waiting_feedback"
        if error <= self.resume_limit and stationary:
            if self.settled_since_ns is None:
                self.settled_since_ns = now
        else:
            self.settled_since_ns = None
        if (self.settled_release_limit > self.resume_limit
                and error < self.settled_release_limit and stationary):
            if self.residual_since_ns is None:
                self.residual_since_ns = now
        else:
            self.residual_since_ns = None
        tight_ready = (self.settled_since_ns is not None
                       and now-self.settled_since_ns >= self.stable_ns)
        residual_ready = (self.residual_since_ns is not None
                          and now-self.residual_since_ns >= self.settled_release_ns)
        if tight_ready or residual_ready:
            self.phase = "awaiting_release"
            if self.release_ready_ns is None:
                self.release_ready_ns = now
            self.release_basis = "tight_hysteresis" if tight_ready else "settled_residual"
        else:
            self.release_ready_ns = None
            self.release_basis = None
            self.phase = "waiting_feedback"
            if error > self.resume_limit and now - self.since_ns >= self.timeout_ns:
                self.phase = "latched"
        return self.phase

    def release_error_allowed(self, error):
        """Recovery rechecks the current held command, with no motion output."""
        if self.phase == "awaiting_release" and self.release_basis == "settled_residual":
            return error < self.settled_release_limit
        return error <= self.resume_limit

    def snapshot(self):
        return dict(phase=self.phase, detail=self.detail,
                    paused_since_ns=self.since_ns, threshold_rad=self.limit,
                    hard_threshold_rad=self.hard_limit,
                    error_debounce_s=self.debounce_ns/1e9,
                    pending_soft_errors=list(self.pending),
                    dynamic_wrist_lag_s=self.wrist_lag_s,
                    dynamic_thresholds=list(self.dynamic_thresholds),
                    resume_threshold_rad=self.resume_limit,
                    settled_release_threshold_rad=self.settled_release_limit,
                    settled_release_stable_s=self.settled_release_ns/1e9,
                    release_basis=self.release_basis, release_ready_ns=self.release_ready_ns,
                    settle_timeout_s=self.timeout_ns / 1e9)
