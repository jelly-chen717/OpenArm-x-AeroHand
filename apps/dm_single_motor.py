"""DM-J4310P-2EC single-motor bench tests using the bundled USB2CAN SDK.

Examples (close DMTool first):
    python apps/dm_single_motor.py ports
    python apps/dm_single_motor.py monitor --port COM9
    python apps/dm_single_motor.py mit-speed --port COM9
    python apps/dm_single_motor.py position --port COM9
    python apps/dm_single_motor.py mit-position --port COM9

Motion commands are intended for a fixed, unloaded motor. Ctrl+C triggers
best-effort disable. A disconnected bus requires a physical stop mechanism.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SDK_PATH = ROOT / "达妙电机/motor-sdk-master/Python例程/u2can/DM_CAN.py"
FIELDS = (
    "elapsed_s", "event", "mode", "target_position", "target_velocity", "kp",
    "kd", "feedforward_nm", "position_rad", "velocity_rad_s", "torque_nm",
    "unwrapped_position_rad", "state", "mos_c", "rotor_c", "feedback_sequence", "feedback_age_s",
    "raw_feedback",
)


class TestFailure(RuntimeError):
    """A test or communication check failed."""


def load_sdk():
    spec = importlib.util.spec_from_file_location("single_motor_dm_sdk", SDK_PATH)
    if spec is None or spec.loader is None:
        raise TestFailure(f"SDK unavailable: {SDK_PATH}")
    sdk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sdk)
    return sdk


def make_controller(sdk, serial_port, motor):
    """Reuse SDK encoding/parsing and intercept only validated feedback frames."""
    class BenchController(sdk.MotorControl):
        def __init__(self):
            self.sequence = 0
            self.received_at = 0.0
            self.unwrapped_position = 0.0
            self.feedback_gap = 0.0
            self.raw_feedback = b""
            self.mos_c = None
            self.rotor_c = None
            self.pending_register = None
            super().__init__(serial_port)
            self.send_data_frame = self.send_data_frame.copy()
            self.addMotor(motor)

        def recv(self):
            incoming = self.data_save + self.serial_.read_all()
            packets = self._MotorControl__extract_packets(incoming)
            for packet in packets:
                can_id = int.from_bytes(packet[3:7], "little")
                payload = packet[7:15]
                if packet[1] != 0x11 or can_id != motor.MasterID:
                    continue
                is_parameter = (
                    self.pending_register is not None
                    and payload[:2] == motor.SlaveID.to_bytes(2, "little")
                    and payload[2] in (0x33, 0x55)
                    and payload[3] == self.pending_register
                )
                if is_parameter:
                    self._MotorControl__process_set_param_packet(payload, can_id, packet[1])
                    continue
                if payload[0] & 0x0F != motor.SlaveID:
                    continue
                previous_position = float(motor.getPosition())
                self._MotorControl__process_packet(payload, can_id, packet[1])
                position = float(motor.getPosition())
                now = time.monotonic()
                if self.sequence:
                    delta = (position - previous_position + 12.5) % 25.0 - 12.5
                    self.unwrapped_position += delta
                    self.feedback_gap = now - self.received_at
                else:
                    self.unwrapped_position = position
                self.raw_feedback = bytes(payload)
                self.mos_c, self.rotor_c = payload[6], payload[7]
                self.received_at = now
                self.sequence += 1

        def recv_set_param_data(self):
            self.recv()

    return BenchController()


def read_parameter(controller, motor, register):
    motor.temp_param_dict.pop(int(register), None)
    controller.pending_register = int(register)
    try:
        controller._MotorControl__read_RID_param(motor, register)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            controller.recv_set_param_data()
            if int(register) in motor.temp_param_dict:
                break
            time.sleep(0.002)
        value = motor.temp_param_dict.get(int(register))
    finally:
        controller.pending_register = None
    if value is None or not math.isfinite(float(value)):
        raise TestFailure(f"Parameter read failed: {register.name}")
    return value


def wait_feedback(controller, previous_sequence, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        controller.recv()
        if controller.sequence > previous_sequence:
            return
        time.sleep(0.002)
    raise TestFailure(f"No fresh motor feedback within {timeout:.3f} s")


def poll(controller, motor, timeout):
    controller.recv()
    previous = controller.sequence
    controller.refresh_motor_status(motor)
    wait_feedback(controller, previous, timeout)


def check_feedback(controller, motor, args, origin=None, expected_state=None):
    state = int(motor.getError())
    if state not in (0, 1):
        raise TestFailure(f"Motor fault state 0x{state:X}")
    if expected_state is not None and state != expected_state:
        raise TestFailure(f"Expected state {expected_state}, received {state}")
    if time.monotonic() - controller.received_at > args.feedback_timeout:
        raise TestFailure("Stale motor feedback")
    if max(controller.mos_c, controller.rotor_c) >= args.temperature_limit:
        raise TestFailure("Bench temperature threshold reached")
    if abs(float(motor.getVelocity())) > args.speed_limit:
        raise TestFailure("Bench speed threshold exceeded")
    if origin is not None:
        if controller.feedback_gap * args.speed_limit >= 12.5:
            raise TestFailure("Feedback gap too long to resolve position wrap")
        if abs(controller.unwrapped_position - origin) > args.travel_limit:
            raise TestFailure("Bench travel threshold exceeded")


class CsvLog:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", encoding="utf-8-sig", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.started = time.monotonic()
        self.last_print = -1.0
        self.target = {}
        self.mode = None

    def write(self, event, controller, motor):
        elapsed = time.monotonic() - self.started
        row = dict.fromkeys(FIELDS, "")
        row.update(self.target)
        row.update(elapsed_s=elapsed, event=event, mode=self.mode)
        if controller.sequence:
            row.update(
                position_rad=float(motor.getPosition()),
                unwrapped_position_rad=controller.unwrapped_position,
                velocity_rad_s=float(motor.getVelocity()),
                torque_nm=float(motor.getTorque()), state=int(motor.getError()),
                mos_c=controller.mos_c, rotor_c=controller.rotor_c,
                feedback_sequence=controller.sequence,
                feedback_age_s=time.monotonic() - controller.received_at,
                raw_feedback=controller.raw_feedback.hex(" "),
            )
        self.writer.writerow(row)
        self.file.flush()
        if event != "sample" or elapsed - self.last_print >= 0.5:
            print(f"{elapsed:7.2f}s {event}  q={row['position_rad']}  "
                  f"v={row['velocity_rad_s']}  state={row['state']}")
            self.last_print = elapsed

    def close(self):
        self.file.close()


def send_target(controller, motor, sdk, mode, target, log):
    log.target = target
    if mode == sdk.Control_Type.POS_VEL:
        controller.control_Pos_Vel(motor, target["target_position"], target["target_velocity"])
    else:
        controller.controlMIT(
            motor, target["kp"], target["kd"], target["target_position"],
            target["target_velocity"], target["feedforward_nm"],
        )


def run_segment(controller, motor, sdk, mode, target, duration, args, origin, log,
                settle=False):
    """Send a bounded segment; completion uses fresh feedback only."""
    started = time.monotonic()
    next_tick = started
    stable_since = None
    while time.monotonic() - started < duration:
        time.sleep(max(0.0, next_tick - time.monotonic()))
        controller.recv()
        previous = controller.sequence
        send_target(controller, motor, sdk, mode, target, log)
        wait_feedback(controller, previous, args.feedback_timeout)
        check_feedback(controller, motor, args, origin, expected_state=1)
        log.write("sample", controller, motor)
        if settle:
            at_target = abs(float(motor.getPosition()) - target["target_position"]) <= 0.01
            slow = abs(float(motor.getVelocity())) <= 0.05
            if at_target and slow:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    return
            else:
                stable_since = None
        # Skip missed ticks instead of emitting a burst of old commands.
        next_tick = max(next_tick + args.period, time.monotonic())
    if settle:
        raise TestFailure("Target did not settle within the segment timeout")


def disable_and_confirm(controller, motor, args, log):
    """Always attempt disable, including after an interrupted enable command."""
    for _ in range(3):
        try:
            controller.recv()
            previous = controller.sequence
            controller.disable(motor)
            wait_feedback(controller, previous, args.feedback_timeout)
            if int(motor.getError()) == 0:
                log.write("disabled_confirmed", controller, motor)
                return True
        except (OSError, TestFailure, ValueError) as exc:
            print(f"Disable attempt: {exc}", file=sys.stderr)
    print("DISABLE UNCONFIRMED. Use the physical stop/power control.", file=sys.stderr)
    return False


def motion_test(controller, motor, sdk, args, log):
    poll(controller, motor, args.feedback_timeout)
    check_feedback(controller, motor, args, expected_state=0)
    mode = sdk.Control_Type.POS_VEL if args.command == "position" else sdk.Control_Type.MIT
    active_mode = read_parameter(controller, motor, sdk.DM_variable.CTRL_MODE)
    if active_mode != mode:
        raise TestFailure(
            f"Motor mode={active_mode}, test requires {int(mode)}. "
            "Set the mode in DMTool while disabled, then close DMTool and retry."
        )
    log.mode = int(mode)
    poll(controller, motor, args.feedback_timeout)
    check_feedback(controller, motor, args, expected_state=0)
    origin = float(motor.getPosition())
    controller.unwrapped_position = origin
    if args.command != "mit-speed" and abs(origin) + args.amplitude + 0.05 >= 12.5:
        raise TestFailure("Test region crosses feedback wrap boundary; reposition first")
    target = dict(target_position=origin, target_velocity=0.0,
                  kp=0.0, kd=args.kd, feedforward_nm=0.0)
    if args.command == "position":
        target["target_velocity"] = args.velocity
    if args.command == "mit-position":
        target["kp"] = args.kp
    log.write("preflight_ok", controller, motor)
    disable_ok = False
    try:
        # Seed a benign target while disabled before enabling.
        previous = controller.sequence
        send_target(controller, motor, sdk, mode, target.copy(), log)
        wait_feedback(controller, previous, args.feedback_timeout)
        check_feedback(controller, motor, args, origin, expected_state=0)
        previous = controller.sequence
        controller.enable(motor)
        wait_feedback(controller, previous, args.feedback_timeout)
        check_feedback(controller, motor, args, origin, expected_state=1)
        log.write("enabled", controller, motor)
        if args.command == "mit-speed":
            for velocity, duration in (
                (0.0, 1.0), (args.velocity, args.duration), (0.0, 1.5),
                (-args.velocity, args.duration), (0.0, 1.5),
            ):
                target["target_velocity"] = velocity
                run_segment(controller, motor, sdk, mode, target.copy(), duration,
                            args, origin, log)
                if velocity and abs(float(motor.getVelocity()) - velocity) > max(0.15, abs(velocity) * 0.3):
                    raise TestFailure("Actual velocity did not track the MIT speed target")
            if abs(float(motor.getVelocity())) > 0.05:
                raise TestFailure("Motor did not slow to the stop threshold")
        else:
            for _ in range(args.cycles):
                for offset in (args.amplitude, 0.0, -args.amplitude, 0.0):
                    target["target_position"] = origin + offset
                    run_segment(controller, motor, sdk, mode, target.copy(), 5.0,
                                args, origin, log, settle=True)
        log.write("test_completed", controller, motor)
    finally:
        disable_ok = disable_and_confirm(controller, motor, args, log)
    if not disable_ok:
        raise TestFailure("Test ended without a confirmed disabled state")


def run_device(args, sdk, serial_module):
    serial_port = serial_module.Serial(
        port=None, baudrate=921600, timeout=0.02, write_timeout=0.2,
    )
    serial_port.port = args.port
    stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000000:09d}"
    log = CsvLog(args.log_dir / f"{stamp}_{args.command}.csv")
    print(f"CSV: {log.file.name}")
    controller = None
    motor = sdk.Motor(sdk.DM_Motor_Type.DM4310, args.motor_id, args.master_id)
    try:
        controller = make_controller(sdk, serial_port, motor)
        controller.change_limit_param(sdk.DM_Motor_Type.DM4310, 12.5, 30.0, 10.0)
        parameters = {}
        for name in ("ESC_ID", "MST_ID", "PMAX", "VMAX", "TMAX", "CTRL_MODE", "TIMEOUT"):
            parameters[name] = read_parameter(controller, motor, getattr(sdk.DM_variable, name))
        print(f"Parameters: {parameters}")
        for name, expected in (
            ("ESC_ID", args.motor_id), ("MST_ID", args.master_id),
            ("PMAX", 12.5), ("VMAX", 30.0), ("TMAX", 10.0),
        ):
            if not math.isclose(parameters[name], expected, rel_tol=1e-5, abs_tol=1e-5):
                raise TestFailure(f"{name}={parameters[name]}, expected {expected}")
        log.mode = parameters["CTRL_MODE"]
        poll(controller, motor, args.feedback_timeout)
        check_feedback(controller, motor, args)
        log.write("connected", controller, motor)
        if args.command == "monitor":
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                started = time.monotonic()
                poll(controller, motor, args.feedback_timeout)
                check_feedback(controller, motor, args)
                log.write("sample", controller, motor)
                time.sleep(max(0.0, args.period - (time.monotonic() - started)))
        else:
            if parameters["TIMEOUT"] == 0:
                print("CAN TIMEOUT=0: link loss may leave the last command active. "
                      "Keep physical stop/power control accessible.")
            motion_test(controller, motor, sdk, args, log)
    except BaseException as exc:
        if controller is not None:
            log.write(f"exit_{type(exc).__name__}: {exc}", controller, motor)
        raise
    finally:
        serial_port.close()
        log.close()


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("ports", "monitor", "position", "mit-speed", "mit-position"))
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=0x01)
    parser.add_argument("--master-id", type=lambda value: int(value, 0), default=0x11)
    parser.add_argument("--duration", type=float, default=None,
                        help="monitor duration (default 10 s), or each speed leg (default 2 s)")
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--feedback-timeout", type=float, default=0.2)
    parser.add_argument("--amplitude", type=float, default=0.05, help="position offset in rad")
    parser.add_argument("--velocity", type=float, default=None,
                        help="positive speed magnitude; position default 0.1, MIT speed default 0.5")
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--travel-limit", type=float, default=None,
                        help="distance from start, rad; default scales with speed and duration")
    parser.add_argument("--speed-limit", type=float, default=None,
                        help="abort threshold, rad/s; default max(2, 1.3 * target speed)")
    parser.add_argument("--temperature-limit", type=float, default=50.0, help="bench abort threshold, C")
    parser.add_argument("--log-dir", type=Path, default=ROOT / "logs/single_motor")
    return parser


def validate_args(parser, args):
    if args.duration is None:
        args.duration = 10.0 if args.command == "monitor" else 2.0
    if args.velocity is None:
        args.velocity = 0.1 if args.command == "position" else 0.5
    if args.speed_limit is None:
        args.speed_limit = max(2.0, 1.3 * args.velocity)
    if args.travel_limit is None:
        args.travel_limit = 2.0
        if args.command == "mit-speed":
            args.travel_limit = max(2.0, args.velocity * (args.duration + 1.5) * 1.3)
    bounds = {
        "duration": (0.1, 120.0 if args.command == "monitor" else 3.0),
        "period": (0.02, 0.1), "feedback_timeout": (0.05, 0.5),
        "amplitude": (0.005, 0.1), "velocity": (0.05, 30.0),
        "kp": (0.1, 5.0), "kd": (0.1, 2.0),
        "travel_limit": (0.1, 200.0), "speed_limit": (0.1, 45.0),
        "temperature_limit": (30.0, 60.0),
    }
    for name, (lower, upper) in bounds.items():
        value = getattr(args, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f"--{name.replace('_', '-')} must be in [{lower}, {upper}]")
    if not 1 <= args.cycles <= 10:
        parser.error("--cycles must be in [1, 10]")
    if not 1 <= args.motor_id <= 15 or not 1 <= args.master_id <= 0x7FF:
        parser.error("motor ID must be 1..15; master ID must be 1..0x7FF")
    if args.motor_id == args.master_id:
        parser.error("motor ID and master ID must differ")
    if args.period >= args.feedback_timeout:
        parser.error("feedback timeout must exceed the control period")
    if args.amplitude >= args.travel_limit:
        parser.error("amplitude must be smaller than the travel limit")
    if args.command == "mit-speed":
        if args.speed_limit <= args.velocity:
            parser.error("speed limit must exceed the target velocity")
        if args.feedback_timeout * args.speed_limit >= 12.5:
            parser.error("reduce feedback timeout to resolve position wrap at this speed limit")


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    try:
        import serial
        if args.command == "ports":
            from serial.tools import list_ports
            for port in list_ports.comports():
                print(f"{port.device}: {port.description}")
            return 0
        run_device(args, load_sdk(), serial)
    except KeyboardInterrupt:
        print("Interrupted. Check the disable confirmation above.")
        return 130
    except (TestFailure, OSError, ValueError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        if isinstance(exc, ImportError):
            print("Install dependencies: python -m pip install pyserial numpy", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
