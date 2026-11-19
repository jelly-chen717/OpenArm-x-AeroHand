"""Offline protocol and cleanup regressions. Never opens a physical port."""

import contextlib
import io
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

import dm_single_motor as bench


@contextlib.contextmanager
def memory_log():
    buffer = io.StringIO()
    with patch.object(Path, "open", return_value=buffer):
        log = bench.CsvLog(bench.ROOT / "apps/test.csv")
    try:
        yield log, buffer
    finally:
        log.close()


def feedback_packet(can_id, payload):
    return b"\xaa\x11\x08" + can_id.to_bytes(4, "little") + payload + b"\x55"


class FakeSerial:
    def __init__(self):
        self.is_open = False
        self.incoming = b""
        self.writes = []
        self.enabled = False
        self.position = 0.0
        self.mode = 1
        self.drop_feedback = False

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def read_all(self):
        data, self.incoming = self.incoming, b""
        return data

    def write(self, frame):
        self.writes.append(bytes(frame))
        can_id = int.from_bytes(frame[13:15], "little")
        data = frame[21:29]
        if self.drop_feedback:
            return len(frame)
        if can_id == 0x7FF and data[2] == 0x33:
            rid = data[3]
            integers = {7: 0x11, 8: 1, 9: 0, 10: self.mode}
            floats = {21: 12.5, 22: 30.0, 23: 10.0}
            value = struct.pack("<I", integers[rid]) if rid in integers else struct.pack("<f", floats[rid])
            payload = data[:4] + value
        else:
            if data == b"\xff" * 7 + b"\xfc":
                self.enabled = True
            if data == b"\xff" * 7 + b"\xfd":
                self.enabled = False
            q = int((self.position + 12.5) / 25 * 65535)
            payload = bytes([0x11 if self.enabled else 0x01, q >> 8, q & 255,
                             0x7F, 0xF7, 0xFF, 30, 29])
        self.incoming += feedback_packet(0x11, payload)
        return len(frame)


class BenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = bench.load_sdk()

    def setUp(self):
        self.serial = FakeSerial()
        self.motor = self.sdk.Motor(self.sdk.DM_Motor_Type.DM4310, 1, 0x11)
        with contextlib.redirect_stdout(io.StringIO()):
            self.controller = bench.make_controller(self.sdk, self.serial, self.motor)
        parser = bench.make_parser()
        self.args = parser.parse_args(["mit-speed"])
        bench.validate_args(parser, self.args)

    def test_mit_encoding_preserves_kd(self):
        self.controller.controlMIT(self.motor, 0, 1, 0, 0.5, 0)
        frame = self.serial.writes[-1]
        self.assertEqual(frame[13:15], b"\x01\x00")
        self.assertEqual(frame[21:29], bytes.fromhex("7f ff 82 10 00 33 37 ff"))

    def test_real_feedback_tracks_state_temperature_and_freshness(self):
        bench.poll(self.controller, self.motor, 0.05)
        self.assertEqual(self.controller.sequence, 1)
        self.assertEqual(self.controller.mos_c, 30)
        self.assertEqual(self.controller.rotor_c, 29)
        self.assertLess(abs(self.motor.getPosition()), 0.001)
        bench.check_feedback(self.controller, self.motor, self.args, expected_state=0)

    def test_split_frame_and_wrong_id(self):
        payload = bytes.fromhex("01 7f ff 7f f7 ff 20 1f")
        self.serial.incoming = feedback_packet(0x12, payload)
        self.controller.recv()
        self.assertEqual(self.controller.sequence, 0)
        frame = feedback_packet(0x11, payload)
        self.serial.incoming = b"noise" + frame[:9]
        self.controller.recv()
        self.assertEqual(self.controller.sequence, 0)
        self.serial.incoming = frame[9:]
        self.controller.recv()
        self.assertEqual(self.controller.sequence, 1)

    def test_parameter_reply_does_not_become_feedback(self):
        value = bench.read_parameter(self.controller, self.motor, self.sdk.DM_variable.PMAX)
        self.assertEqual(value, 12.5)
        self.assertEqual(self.controller.sequence, 0)

    def test_missing_parameter_cannot_reuse_cached_value(self):
        self.motor.temp_param_dict[21] = 12.5
        self.serial.drop_feedback = True
        with self.assertRaises(bench.TestFailure):
            bench.read_parameter(self.controller, self.motor, self.sdk.DM_variable.PMAX)

    def test_old_feedback_cannot_satisfy_poll(self):
        bench.poll(self.controller, self.motor, 0.05)
        self.serial.drop_feedback = True
        with self.assertRaises(bench.TestFailure):
            bench.poll(self.controller, self.motor, 0.01)

    def test_exception_after_enable_disables_motor(self):
        with memory_log() as (log, buffer):
            with patch.object(bench, "run_segment", side_effect=bench.TestFailure("injected")):
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(bench.TestFailure, "injected"):
                        bench.motion_test(self.controller, self.motor, self.sdk, self.args, log)
            self.assertIn("disabled_confirmed", buffer.getvalue())
        self.assertFalse(self.serial.enabled)
        self.assertEqual(self.serial.writes[-1][21:29], b"\xff" * 7 + b"\xfd")

    def test_interrupt_during_enable_also_disables(self):
        original_enable = self.controller.enable

        def interrupted_enable(motor):
            original_enable(motor)
            raise KeyboardInterrupt()

        with memory_log() as (log, _):
            with patch.object(self.controller, "enable", side_effect=interrupted_enable):
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(KeyboardInterrupt):
                        bench.motion_test(self.controller, self.motor, self.sdk, self.args, log)
        self.assertFalse(self.serial.enabled)

    def test_wrong_mode_never_enables(self):
        self.serial.mode = 2
        with memory_log() as (log, _):
            with self.assertRaisesRegex(bench.TestFailure, "requires 1"):
                bench.motion_test(self.controller, self.motor, self.sdk, self.args, log)
        self.assertFalse(any(frame[21:29] == b"\xff" * 7 + b"\xfc" for frame in self.serial.writes))

    def test_successful_motion_path_confirms_disable(self):
        def follow_target(controller, motor, sdk, mode, target, *args, **kwargs):
            motor.state_dq = target["target_velocity"]

        with memory_log() as (log, buffer):
            with patch.object(bench, "run_segment", side_effect=follow_target) as segment:
                with contextlib.redirect_stdout(io.StringIO()):
                    bench.motion_test(self.controller, self.motor, self.sdk, self.args, log)
            self.assertEqual(segment.call_count, 5)
            self.assertIn("test_completed", buffer.getvalue())
            self.assertIn("disabled_confirmed", buffer.getvalue())
        self.assertFalse(self.serial.enabled)

    def test_stationary_motor_fails_velocity_test_and_disables(self):
        with memory_log() as (log, _):
            with patch.object(bench, "run_segment"):
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(bench.TestFailure, "track"):
                        bench.motion_test(self.controller, self.motor, self.sdk, self.args, log)
        self.assertFalse(self.serial.enabled)

    def test_segment_sends_and_logs_fresh_feedback(self):
        self.controller.enable(self.motor)
        bench.poll(self.controller, self.motor, 0.05)
        target = dict(target_position=0.0, target_velocity=0.0, kp=1.0, kd=1.0,
                      feedforward_nm=0.0)
        with memory_log() as (log, buffer):
            with contextlib.redirect_stdout(io.StringIO()):
                bench.run_segment(self.controller, self.motor, self.sdk,
                                  self.sdk.Control_Type.MIT, target, 0.65,
                                  self.args, 0.0, log, settle=True)
            self.assertIn("sample", buffer.getvalue())
        self.assertGreater(self.controller.sequence, 5)

    def test_failed_disable_is_explicit(self):
        self.serial.drop_feedback = True
        self.args.feedback_timeout = 0.01
        with memory_log() as (log, _):
            with contextlib.redirect_stderr(io.StringIO()) as output:
                result = bench.disable_and_confirm(self.controller, self.motor, self.args, log)
            self.assertFalse(result)
            self.assertIn("DISABLE UNCONFIRMED", output.getvalue())

    def test_monitor_normal_entry_point_with_fake_port(self):
        parser = bench.make_parser()
        args = parser.parse_args(["monitor", "--duration", "0.1"])
        bench.validate_args(parser, args)
        buffer = io.StringIO()
        buffer.name = "memory.csv"
        with patch.object(Path, "open", return_value=buffer):
            with patch.object(Path, "mkdir"):
                with contextlib.redirect_stdout(io.StringIO()):
                    serial_module = type("SerialModule", (), {"Serial": lambda **kwargs: self.serial})
                    bench.run_device(args, self.sdk, serial_module)
        self.assertFalse(self.serial.is_open)
        self.assertFalse(any(frame[21:29] == b"\xff" * 7 + b"\xfc" for frame in self.serial.writes))

    def test_invalid_values_rejected_before_connect(self):
        for option, value in (("--velocity", "31"), ("--kd", "nan"),
                              ("--master-id", "1"), ("--duration", "inf")):
            parser = bench.make_parser()
            args = parser.parse_args(["mit-speed", option, value])
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    bench.validate_args(parser, args)

    def test_watchdog_faults_and_travel_limits(self):
        bench.poll(self.controller, self.motor, 0.05)
        with self.assertRaisesRegex(bench.TestFailure, "travel"):
            bench.check_feedback(self.controller, self.motor, self.args, origin=5.0)
        self.motor.state_err = 8
        with self.assertRaisesRegex(bench.TestFailure, "fault"):
            bench.check_feedback(self.controller, self.motor, self.args)

    def test_ten_rad_s_command_has_consistent_limits(self):
        parser = bench.make_parser()
        args = parser.parse_args(["mit-speed", "--velocity", "10", "--kd", "2", "--duration", "2"])
        bench.validate_args(parser, args)
        self.assertEqual(args.speed_limit, 13.0)
        self.assertGreater(args.travel_limit, 20.0)

    def test_position_wrap_is_accumulated_in_both_directions(self):
        self.serial.position = 12.4
        bench.poll(self.controller, self.motor, 0.05)
        start = self.controller.unwrapped_position
        self.serial.position = -12.4
        bench.poll(self.controller, self.motor, 0.05)
        self.assertAlmostEqual(self.controller.unwrapped_position - start, 0.2, delta=0.001)
        self.serial.position = 12.4
        bench.poll(self.controller, self.motor, 0.05)
        self.assertAlmostEqual(self.controller.unwrapped_position, start, delta=0.001)


if __name__ == "__main__":
    unittest.main()
