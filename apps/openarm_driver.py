"""Windows OpenArm V1 CAN FD driver using the installed PCAN-Basic DLL.

Protocol references are under resources/openarm_control_extract/openarm_can.
Units are motor radians, radians/second and Nm. Use one driver owner per bus.

Laboratory profile (2026-11-19): MIT setpoints saturate instead of raising, and
enable/command no longer pre-read the mode, require disabled feedback, or
auto-disable after one missed reply. Frame/adapter/timeout checks remain because
they detect an unusable transport, not a motion-threshold excursion.
"""

from __future__ import annotations

import argparse
import ctypes as ct
from contextlib import ExitStack
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
import struct
import time


ROOT = Path(__file__).resolve().parents[1]
# 80 MHz / 80 = 1 Mbps, 80 MHz / 16 = 5 Mbps. Both sample at 75%.
BIT_TIMING = (
    b"f_clock_mhz=80,nom_brp=1,nom_tseg1=59,nom_tseg2=20,nom_sjw=1,"
    b"data_brp=1,data_tseg1=11,data_tseg2=4,data_sjw=2"
)
DEFAULT_CHANNELS = {"right": 0x51, "left": 0x52}
MOTOR_TYPES = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310")
LIMITS = {"DM8009": (12.5, 45.0, 54.0), "DM4340": (12.5, 10.0, 28.0),
          "DM4310": (12.5, 30.0, 10.0)}
DEFAULT_KP = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0)
DEFAULT_KD = (2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5)
STATE_NAMES = {0: "DISABLED", 1: "ENABLED", 8: "OVERVOLTAGE", 9: "UNDERVOLTAGE",
               10: "OVERCURRENT", 11: "MOS_OVERHEAT", 12: "COIL_OVERHEAT",
               13: "COMMUNICATION_LOST", 14: "OVERLOAD"}
FD_BRS = 0x04 | 0x08
QUEUE_EMPTY = 0x20


class DriverError(RuntimeError):
    """Communication, validation or motor feedback failure."""


class CanFrame(ct.Structure):
    _fields_ = [("can_id", ct.c_uint32), ("kind", ct.c_uint8),
                ("dlc", ct.c_uint8), ("data", ct.c_uint8 * 64)]


class PcanBus:
    """One native PCAN channel, with explicit FD timing and checked cleanup."""

    def __init__(self, channel):
        from openarm_bus_lock import ChannelLease
        self.channel = channel
        self.lease = ChannelLease(channel)
        self.opened = False
        if sys.platform != "win32":
            raise DriverError("PCAN-Basic requires Windows")
        self.dll = ct.WinDLL("PCANBasic.dll")
        signatures = {
            "CAN_InitializeFD": [ct.c_ushort, ct.c_char_p],
            "CAN_Uninitialize": [ct.c_ushort],
            "CAN_GetStatus": [ct.c_ushort],
            "CAN_SetValue": [ct.c_ushort, ct.c_ubyte, ct.c_void_p, ct.c_uint],
            "CAN_ReadFD": [ct.c_ushort, ct.POINTER(CanFrame), ct.POINTER(ct.c_uint64)],
            "CAN_WriteFD": [ct.c_ushort, ct.POINTER(CanFrame)],
            "CAN_GetErrorText": [ct.c_uint32, ct.c_ushort, ct.c_char_p],
        }
        for name, arguments in signatures.items():
            function = getattr(self.dll, name)
            function.argtypes = arguments
            function.restype = ct.c_uint32

    def check(self, status, operation):
        if status:
            message = ct.create_string_buffer(256)
            self.dll.CAN_GetErrorText(status, 0, message)
            raise DriverError(f"PCAN 0x{self.channel:X} {operation}: "
                              f"0x{status:X} {message.value.decode(errors='replace')}")

    def open(self):
        if self.opened:
            raise DriverError("Channel already open")
        try:
            self.lease.acquire()
        except OSError as exc:
            raise DriverError(f'PCAN 0x{self.channel:X} owned by another OpenArm process') from exc
        try:
            off = ct.c_uint32(0)
            # Explicitly clear listen-only inherited from previous diagnostics.
            self.check(self.dll.CAN_SetValue(self.channel, 0x08, ct.byref(off), 4), "listen-only off")
            self.check(self.dll.CAN_InitializeFD(self.channel, BIT_TIMING), "initialize FD")
            self.opened = True
            self.check(self.dll.CAN_GetStatus(self.channel), "initial status")
        except BaseException:
            self.close()
            raise

    def close(self):
        try:
            if self.opened:
                self.check(self.dll.CAN_Uninitialize(self.channel), "close")
        finally:
            self.opened = False
            self.lease.release()

    def status(self):
        return int(self.dll.CAN_GetStatus(self.channel))

    def send(self, can_id, payload):
        if not self.opened:
            raise DriverError("Channel closed")
        if not 0 <= can_id <= 0x7FF or (len(payload) != 8
                                         and not (0x201 <= can_id <= 0x207 and len(payload) == 4)):
            raise ValueError("Invalid motor frame ID or length")
        self.check(self.status(), "before transmit")
        frame = CanFrame(can_id=can_id, kind=FD_BRS, dlc=len(payload))
        frame.data[:len(payload)] = payload
        self.check(self.dll.CAN_WriteFD(self.channel, ct.byref(frame)), "write FD")

    def receive(self):
        if not self.opened:
            raise DriverError("Channel closed")
        frame, timestamp = CanFrame(), ct.c_uint64()
        status = self.dll.CAN_ReadFD(self.channel, ct.byref(frame), ct.byref(timestamp))
        if status == QUEUE_EMPTY:
            return None
        self.check(status, "read FD")
        # PCAN status notifications and error frames are not motor feedback.
        if frame.kind & (0x40 | 0x80):
            self.check(self.status(), "bus notification")
            return (frame.can_id, frame.kind, b"")
        if frame.dlc > 8:
            return (frame.can_id, frame.kind, b"")
        return frame.can_id, frame.kind, bytes(frame.data[:frame.dlc])


@dataclass(frozen=True)
class MotorState:
    motor_id: int
    motor_type: str
    state: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_c: int
    rotor_c: int
    received_at: float
    raw: str


def motor_type(motor_id):
    if type(motor_id) is not int or not 1 <= motor_id <= 8:
        raise ValueError("Motor ID must be 1..8 (8 is the optional gripper)")
    return "DM4310" if motor_id == 8 else MOTOR_TYPES[motor_id - 1]


def decode_feedback(can_id, kind, data):
    motor_id = can_id - 0x10
    if kind & (0x01 | 0x02 | 0x40 | 0x80) or not kind & 0x04:
        return None
    if not 1 <= motor_id <= 8 or len(data) != 8 or data[0] & 15 != motor_id:
        return None
    model = motor_type(motor_id)
    pmax, vmax, tmax = LIMITS[model]
    return MotorState(
        motor_id, model, data[0] >> 4,
        int.from_bytes(data[1:3], "big") / 65535 * (2 * pmax) - pmax,
        ((data[3] << 4) | (data[4] >> 4)) / 4095 * (2 * vmax) - vmax,
        (((data[4] & 15) << 8) | data[5]) / 4095 * (2 * tmax) - tmax,
        data[6], data[7], time.monotonic(), data.hex(),
    )


def encode_mit(motor_id, position, velocity, kp, kd, torque):
    """Pack a MIT frame. Out-of-range setpoints saturate to the protocol limit.

    Formerly a slightly excessive velocity or gain raised ValueError and the
    caller stopped the run. Saturation keeps the frame valid and still sends it.
    """
    pmax, vmax, tmax = LIMITS[motor_type(motor_id)]
    values = (position, velocity, kp, kd, torque)
    ranges = ((-pmax, pmax, 16), (-vmax, vmax, 12), (0, 500, 12),
              (0, 5, 12), (-tmax, tmax, 12))
    packed = []
    for value, (lower, upper, bits) in zip(values, ranges):
        value = lower if not math.isfinite(value) else min(max(value, lower), upper)
        packed.append(int((value - lower) / (upper - lower) * ((1 << bits) - 1)))
    p, v, k, d, t = packed
    return bytes((p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k >> 8),
                  k & 255, d >> 4, ((d & 15) << 4) | (t >> 8), t & 255))


class OpenArmDriver:
    """Synchronous single-owner arm connection. Opening only configures the adapter.

    PCAN handles are explicitly selectable until physical port mapping is verified.
    Closing releases the adapter. Use disable_motor explicitly when required.
    """

    def __init__(self, side="right", channel=None, timeout=0.2, bus=None):
        if side not in DEFAULT_CHANNELS:
            raise ValueError("side must be right or left")
        if not math.isfinite(timeout) or not 0 < timeout <= 5:
            raise ValueError("timeout must be finite and in (0, 5] seconds")
        self.side = side
        self.timeout = timeout
        self.bus = bus if bus is not None else PcanBus(channel or DEFAULT_CHANNELS[side])
        self.states = {}

    def __enter__(self):
        self.bus.open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.bus.close()

    def drain(self):
        # Bounded draining also handles a continuously active external sender.
        for _ in range(4096):
            frame = self.bus.receive()
            if frame is None:
                return
        raise DriverError("Receive queue did not drain")

    def query_motor(self, motor_id):
        motor_type(motor_id)
        self.states.pop(motor_id, None)
        # A newly initialized adapter may lose its first request. Retry once,
        # with a new deadline and receive drain, only after a timeout.
        for attempt in (1, 2):
            self.drain()
            self.bus.send(0x7FF, bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0)))
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                frame = self.bus.receive()
                if frame is None:
                    time.sleep(0.001)
                    continue
                if frame[0] != 0x10 + motor_id:
                    continue
                state = decode_feedback(*frame)
                if state is not None:
                    self.states[motor_id] = state
                    return state
        raise DriverError(f"{self.side} motor {motor_id}: feedback timeout after {attempt} attempts")

    def read_parameter(self, motor_id, register, *, include_raw=False):
        motor_type(motor_id)
        if type(register) is not int or not 0 <= register <= 255:
            raise ValueError("Register must be 0..255")
        self.drain()
        self.bus.send(0x7FF, bytes((motor_id, 0, 0x33, register, 0, 0, 0, 0)))
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            frame = self.bus.receive()
            if frame is None:
                time.sleep(0.001)
                continue
            can_id, kind, data = frame
            if (can_id != 0x10 + motor_id or kind & 0xC3 or not kind & 0x04
                    or len(data) != 8 or data[:4] != bytes((motor_id, 0, 0x33, register))):
                continue
            integer = 7 <= register <= 10 or 13 <= register <= 16 or 35 <= register <= 36
            value = struct.unpack("<I" if integer else "<f", data[4:])[0]
            if not math.isfinite(value):
                raise DriverError(f"Invalid register {register} value")
            if include_raw:
                return value, data.hex()
            return value
        raise DriverError(f"{self.side} motor {motor_id}: register {register} timeout")

    def write_disabled_mode(self, motor_id, code):
        if type(code) is not int or code not in (1, 2, 3, 4):
            raise ValueError("Unknown CTRL_MODE")
        self.bus.send(0x7FF, bytes((motor_id, 0, 0x55, 10)) + struct.pack("<I", code))
        # A parameter-write ACK cannot satisfy read_parameter's independent 0x33 check.
        for _ in range(3):
            if self.read_parameter(motor_id, 10) == code:
                return
            time.sleep(.005)
        raise DriverError(f"{self.side} J{motor_id}: CTRL_MODE readback mismatch")

    def snapshot(self, include_gripper=False):
        motors = []
        for motor_id in range(1, 9 if include_gripper else 8):
            try:
                state = self.query_motor(motor_id)
                row = asdict(state)
                row.update(online=True, state_name=STATE_NAMES.get(state.state, "UNKNOWN"))
                row["age_s"] = time.monotonic() - state.received_at
                row["error"] = None
            except DriverError as exc:
                row = {"motor_id": motor_id, "online": False, "error": str(exc)}
                self.states.pop(motor_id, None)
            motors.append(row)
            if self.bus.status():
                break
        return {"side": self.side, "channel": hex(self.bus.channel),
                "bus_status": hex(self.bus.status()), "motors": motors,
                "expected_motors": 8 if include_gripper else 7,
                "online": sum(row["online"] for row in motors)}

    def enable_motor(self, motor_id, kp=None, kd=None):
        """Seed the measured angle and enable. The caller decides what to do next.

        The former pre-checks (CTRL_MODE readback, DISABLED feedback required) and
        the automatic disable on a missed reply are gone; the returned feedback is
        the observed transition instead.
        """
        motor_type(motor_id)
        if kp is None:
            kp = DEFAULT_KP[motor_id - 1] if motor_id <= 7 else 5.0
        if kd is None:
            kd = DEFAULT_KD[motor_id - 1] if motor_id <= 7 else 0.1
        state = self.query_motor(motor_id)
        hold = encode_mit(motor_id, state.position_rad, 0.0, kp, kd, 0.0)
        self.bus.send(motor_id, hold)
        self.bus.send(motor_id, b"\xff" * 7 + b"\xfc")
        self.bus.send(motor_id, hold)
        return self.wait_motor_state(motor_id, 1)

    def disable_motor(self, motor_id):
        motor_type(motor_id)
        self.bus.send(motor_id, b"\xff" * 7 + b"\xfd")
        return self.wait_motor_state(motor_id, 0)

    def wait_motor_state(self, motor_id, expected):
        # Command replies from the target change can still be in flight. Observe
        # the transition for a bounded time and report whatever was read last.
        deadline = time.monotonic() + self.timeout
        state = self.query_motor(motor_id)
        while state.state != expected and time.monotonic() < deadline:
            time.sleep(0.002)
            state = self.query_motor(motor_id)
        return state

    def command_mit(self, motor_id, position, velocity, kp, kd, torque=0.0):
        """Send one MIT command and return the following feedback.

        Caller supplies calibrated motor-space targets and owns trajectory timing.
        Protocol ranges saturate in encode_mit. A mode readback, a freshness gate
        and an automatic disable after an unexpected motor state are not applied.
        """
        payload = encode_mit(motor_id, position, velocity, kp, kd, torque)
        self.bus.send(motor_id, payload)
        return self.query_motor(motor_id)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "monitor"))
    parser.add_argument("--side", choices=("right", "left", "both"), default="both")
    parser.add_argument("--right-channel", type=lambda v: int(v, 0), default=0x51)
    parser.add_argument("--left-channel", type=lambda v: int(v, 0), default=0x52)
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in (args.timeout, args.duration, args.period)):
        parser.error("Timing values must be positive and finite")
    if args.timeout > 5 or args.right_channel == args.left_channel:
        parser.error("timeout must be <=5 and left/right channels must differ")
    sides = ("right", "left") if args.side == "both" else (args.side,)
    path = args.log or ROOT / "logs/openarm" / f"status_{time.time_ns()}.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    healthy = True
    with path.open("x", encoding="utf-8") as log, ExitStack() as stack:
        print(f"Log: {path}")
        arms = []
        for side in sides:
            channel = getattr(args, f"{side}_channel")
            try:
                arms.append(stack.enter_context(OpenArmDriver(side, channel, args.timeout)))
            except (DriverError, OSError) as exc:
                healthy = False
                line = json.dumps({"side": side, "error": str(exc)}, ensure_ascii=False)
                print(line, flush=True)
                log.write(line + "\n")
        deadline = time.monotonic() + args.duration
        while arms:
            bus_failed = False
            for arm in arms:
                report = arm.snapshot(args.gripper)
                report["time_unix"] = time.time()
                line = json.dumps(report, ensure_ascii=False)
                print(line, flush=True)
                log.write(line + "\n")
                log.flush()
                healthy &= (report["online"] == report["expected_motors"]
                            and report["bus_status"] == "0x0"
                            and all(m.get("state") in (0, 1) for m in report["motors"]))
                bus_failed |= report["bus_status"] != "0x0"
            if args.command == "status" or bus_failed or time.monotonic() >= deadline:
                break
            time.sleep(min(args.period, max(0, deadline - time.monotonic())))
    return 0 if healthy else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
