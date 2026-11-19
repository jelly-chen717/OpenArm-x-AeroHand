"""One small current-relative wrist motion for onsite acceptance."""

from contextlib import ExitStack
from dataclasses import asdict
import json
import time

from check_openarm_enable import require_disabled
from openarm_driver import DriverError, OpenArmDriver, ROOT


AMPLITUDE = 0.06
LEG_SECONDS = 3.0
KP, KD = 10.0, 2.0


def smooth_target(start, end, elapsed):
    u = min(1.0, max(0.0, elapsed / LEG_SECONDS))
    return start + (end - start) * (10 * u**3 - 15 * u**4 + 6 * u**5)


def verify(state, origin, target):
    if state.state != 1:
        raise DriverError(f"Wrist state {state.state}")
    if abs(state.position_rad - origin) > 0.12:
        raise DriverError("Travel exceeded 0.12 rad")
    if abs(state.velocity_rad_s) > 0.3:
        raise DriverError("Speed exceeded 0.3 rad/s")
    if abs(state.torque_nm) > 1.0:
        raise DriverError("Feedback torque exceeded 1 Nm")
    if abs(state.position_rad - target) > 0.08:
        raise DriverError("Tracking error exceeded 0.08 rad")
    if max(state.mos_c, state.rotor_c) >= 50:
        raise DriverError("Temperature reached 50 C")


def main():
    path = ROOT / "logs/openarm" / f"wrist_demo_{time.time_ns()}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    result = {"side": "right", "channel": "0x51", "motor_id": 7,
              "amplitude_rad": AMPLITUDE, "leg_seconds": LEG_SECONDS,
              "kp": KP, "kd": KD, "samples": [], "passed": False}
    print(f"Log: {path}", flush=True)
    try:
        with ExitStack() as stack:
            arms = [stack.enter_context(OpenArmDriver(side)) for side in ("right", "left")]
            result["before"] = require_disabled(arms)
            arm = arms[0]
            baseline = arm.query_motor(7)
            origin = baseline.position_rad
            result["origin_rad"] = origin
            if max(baseline.mos_c, baseline.rotor_c) >= 50 or abs(baseline.velocity_rad_s) > 0.1:
                raise DriverError("Wrist must start stationary and below 50 C")
            # Model J7 nominal limits, with 0.1 rad margin. The actual test
            # remains relative to the measured motor position.
            if abs(origin) + AMPLITUDE > 1.470796:
                raise DriverError("Current wrist region is outside nominal J7 test margin")
            try:
                state = arm.enable_motor(7, kp=KP, kd=KD)
                verify(state, origin, origin)
                start = origin
                for offset in (AMPLITUDE, 0.0, -AMPLITUDE, 0.0):
                    end = origin + offset
                    print(f"Target offset {offset:+.3f} rad", flush=True)
                    began = time.monotonic()
                    while True:
                        elapsed = time.monotonic() - began
                        target = smooth_target(start, end, elapsed)
                        state = arm.command_mit(7, target, 0, KP, KD)
                        result["samples"].append({"target_rad": target, **asdict(state)})
                        verify(state, origin, target)
                        if elapsed >= LEG_SECONDS:
                            break
                        time.sleep(0.02)
                    if abs(state.position_rad - end) > 0.015:
                        raise DriverError("Wrist did not reach endpoint within 0.015 rad")
                    start = end
                result["return_error_rad"] = abs(state.position_rad - origin)
                result["passed"] = True
            finally:
                result["disable_confirmed"] = False
                errors = []
                for _ in range(3):
                    try:
                        result["disabled"] = asdict(arm.disable_motor(7))
                        result["disable_confirmed"] = True
                        break
                    except (DriverError, OSError) as exc:
                        errors.append(str(exc))
                if not result["disable_confirmed"]:
                    result["passed"] = False
                    raise DriverError(f"Wrist disable UNCONFIRMED: {errors}")
            result["after"] = require_disabled(arms)
    except BaseException as exc:
        result["passed"] = False
        result["error"] = str(exc)
        raise
    finally:
        path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"PASS: returned within {result['return_error_rad']:.6f} rad; all joints disabled", flush=True)


if __name__ == "__main__":
    main()
