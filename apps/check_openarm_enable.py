"""Sequential fixed-arm enable/hold/disable verification for fourteen joints."""

from contextlib import ExitStack
from dataclasses import asdict
import json
import math
import time

from openarm_driver import DriverError, LIMITS, OpenArmDriver, ROOT, motor_type


DURATION = 0.5
PERIOD = 0.02
KP, KD = 1.0, 0.2
MAX_TRAVEL, MAX_SPEED, MAX_TEMPERATURE = 0.05, 0.5, 50


def require_disabled(arms):
    reports = [arm.snapshot() for arm in arms]
    for report in reports:
        if (report["online"] != 7 or report["bus_status"] != "0x0"
                or any(m.get("state") != 0 for m in report["motors"])):
            raise DriverError(f"Expected seven disabled joints: {report}")
    return reports


def verify_state(state, origin):
    if state.state != 1:
        raise DriverError(f"Expected ENABLED, received {state.state}")
    if abs(state.position_rad - origin) > MAX_TRAVEL:
        raise DriverError("Joint position changed by more than 0.05 rad")
    if abs(state.velocity_rad_s) > MAX_SPEED:
        raise DriverError("Joint speed exceeded 0.5 rad/s")
    if max(state.mos_c, state.rotor_c) >= MAX_TEMPERATURE:
        raise DriverError("Joint temperature reached 50 C")


def check_joint(arm, motor_id, record):
    parameters = {rid: arm.read_parameter(motor_id, rid) for rid in (10, 21, 22, 23)}
    record["parameters"] = parameters
    expected = (1, *LIMITS[motor_type(motor_id)])
    if any(not math.isclose(parameters[rid], value, rel_tol=1e-5, abs_tol=1e-5)
           for rid, value in zip((10, 21, 22, 23), expected)):
        raise DriverError(f"Motor mode or protocol ranges differ: {parameters}")
    baseline = arm.query_motor(motor_id)
    record["baseline"] = asdict(baseline)
    if baseline.state != 0 or max(baseline.mos_c, baseline.rotor_c) >= MAX_TEMPERATURE:
        raise DriverError("Expected disabled motor below 50 C")
    record["samples"] = []
    try:
        state = arm.enable_motor(motor_id, kp=KP, kd=KD)
        verify_state(state, baseline.position_rad)
        record["samples"].append(asdict(state))
        deadline = time.monotonic() + DURATION
        while time.monotonic() < deadline:
            state = arm.command_mit(motor_id, baseline.position_rad, 0, KP, KD)
            record["samples"].append(asdict(state))
            verify_state(state, baseline.position_rad)
            time.sleep(min(PERIOD, max(0, deadline - time.monotonic())))
    finally:
        record["disable_confirmed"] = False
        failures = []
        for _ in range(3):
            try:
                record["disabled"] = asdict(arm.disable_motor(motor_id))
                record["disable_confirmed"] = True
                break
            except (DriverError, OSError) as exc:
                failures.append(str(exc))
        if not record["disable_confirmed"]:
            raise DriverError(f"Disable UNCONFIRMED: {failures}")
    record["max_travel_rad"] = max(abs(s["position_rad"] - baseline.position_rad)
                                    for s in record["samples"])
    record["passed"] = True


def main():
    path = ROOT / "logs/openarm" / f"enable_check_{time.time_ns()}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    result = {"duration_s": DURATION, "kp": KP, "kd": KD, "joints": [], "passed": False}
    print(f"Log: {path}", flush=True)
    try:
        with ExitStack() as stack:
            arms = [stack.enter_context(OpenArmDriver(side)) for side in ("right", "left")]
            result["before"] = require_disabled(arms)
            for arm in arms:
                for motor_id in range(1, 8):
                    record = {"side": arm.side, "motor_id": motor_id, "passed": False}
                    result["joints"].append(record)
                    try:
                        check_joint(arm, motor_id, record)
                    except BaseException as exc:
                        record["error"] = str(exc)
                        raise
                    record["all_disabled_after"] = require_disabled(arms)
                    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
                    print(f"PASS {arm.side} J{motor_id}: enabled, "
                          f"{len(record['samples'])} samples, disabled; "
                          f"max travel {record['max_travel_rad']:.6f} rad", flush=True)
            result["after"] = require_disabled(arms)
            result["passed"] = True
    except BaseException as exc:
        result["error"] = str(exc)
        result["notes"] = getattr(exc, "__notes__", [])
        raise
    finally:
        path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
