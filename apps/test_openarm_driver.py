"""Offline regressions for Windows FD timing and OpenArm protocol handling."""

import contextlib
import io
import struct
import unittest
from pathlib import Path
from unittest.mock import MagicMock, mock_open, patch

import openarm_driver as driver


def feedback(motor_id, state=0):
    return (0x10 + motor_id, driver.FD_BRS,
            bytes((motor_id | (state << 4), 0x80, 0, 0x80, 0, 0, 25, 26)))


class FakeBus:
    channel = 0x51

    def __init__(self):
        self.opened = False
        self.frames = []
        self.sent = []
        self.enabled = False
        self.drop = False
        self.bus_status = 0
        self.mode = 1

    def open(self):
        self.opened = True

    def close(self):
        self.opened = False

    def status(self):
        return self.bus_status

    def send(self, can_id, payload):
        self.sent.append((can_id, payload))
        if can_id == 0x7FF:
            if not self.drop:
                if payload[2] == 0x33:
                    self.frames.append((0x10 + payload[0], driver.FD_BRS,
                                        payload[:4] + struct.pack("<I", self.mode)))
                else:
                    self.frames.append(feedback(payload[0], int(self.enabled)))
        elif payload == b"\xff" * 7 + b"\xfc":
            self.enabled = True
        elif payload == b"\xff" * 7 + b"\xfd":
            self.enabled = False

    def receive(self):
        return self.frames.pop(0) if self.frames else None


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()
        self.arm = driver.OpenArmDriver(bus=self.bus, timeout=0.005)

    def test_exact_reference_timing(self):
        values = dict(p.split("=") for p in driver.BIT_TIMING.decode().split(","))
        for prefix, bitrate, sjw in (("nom", 1000000, 1), ("data", 5000000, 2)):
            brp, t1, t2 = (int(values[f"{prefix}_{key}"]) for key in ("brp", "tseg1", "tseg2"))
            self.assertEqual(80000000 / brp / (1 + t1 + t2), bitrate)
            self.assertEqual((1 + t1) / (1 + t1 + t2), 0.75)
            self.assertEqual(int(values[f"{prefix}_sjw"]), sjw)

    def test_mode_write_requires_readback_not_disabled_state(self):
        with self.assertRaisesRegex(driver.DriverError, "readback"):
            self.arm.write_disabled_mode(6, 3)
        write = (0x7ff, bytes((6, 0, 0x55, 10, 3, 0, 0, 0)))
        self.assertIn(write, self.bus.sent)
        self.assertEqual(self.bus.sent[-1], (0x7ff, bytes((6, 0, 0x33, 10, 0, 0, 0, 0))))
        self.bus.mode = 3
        self.arm.write_disabled_mode(6, 3)
        self.bus.sent.clear()
        self.bus.enabled = True
        # No disabled-state precheck remains; the write is attempted and its
        # independent readback is the only thing that can reject it.
        with self.assertRaisesRegex(driver.DriverError, "readback"):
            self.arm.write_disabled_mode(6, 2)
        self.assertTrue(any(frame[1][2] == 0x55 for frame in self.bus.sent))

    def test_joint_models_and_separate_scales(self):
        self.assertEqual(driver.motor_type(1), "DM8009")
        self.assertEqual(driver.motor_type(3), "DM4340")
        self.assertEqual(driver.motor_type(7), "DM4310")
        for mid, velocity, torque in ((1, 45, 54), (3, 10, 28), (5, 30, 10)):
            state = driver.decode_feedback(0x10 + mid, driver.FD_BRS,
                                           bytes((mid, 255, 255, 255, 255, 255, 20, 21)))
            self.assertEqual(state.velocity_rad_s, velocity)
            self.assertEqual(state.torque_nm, torque)

    def test_notifications_and_bad_frames_are_rejected(self):
        for frame in ((0x11, 0x80, bytes(8)), (0x11, 0x40, bytes(8)),
                      (0x11, 0, feedback(1)[2]), (0x11, 5, feedback(1)[2]),
                      (0x12, driver.FD_BRS, feedback(1)[2]),
                      (0x11, driver.FD_BRS, bytes(7))):
            self.assertIsNone(driver.decode_feedback(*frame))

    def test_mit_protocol_endpoints(self):
        self.assertEqual(driver.encode_mit(1, -12.5, -45, 0, 0, -54), bytes(8))
        self.assertEqual(driver.encode_mit(1, 12.5, 45, 500, 5, 54), b"\xff" * 8)
        self.assertEqual(driver.encode_mit(1, 0, 0, 0, 0, 0).hex(), "7fff7ff0000007ff")

    def test_out_of_range_mit_saturates_instead_of_raising(self):
        self.assertEqual(driver.encode_mit(3, 0, 11, 1, 1, 0),
                         driver.encode_mit(3, 0, 10, 1, 1, 0))
        self.assertEqual(driver.encode_mit(3, float("nan"), 0, 1, 1, 0),
                         driver.encode_mit(3, -12.5, 0, 1, 1, 0))
        self.assertEqual(driver.encode_mit(3, 0, 0, 501, 1, 0),
                         driver.encode_mit(3, 0, 0, 500, 1, 0))

    def test_snapshot_only_sends_reference_refresh(self):
        with self.arm:
            report = self.arm.snapshot()
        self.assertEqual(report["online"], 7)
        self.assertFalse(self.bus.opened)
        self.assertEqual(self.bus.sent, [(0x7FF, bytes((mid, 0, 0xCC, 0, 0, 0, 0, 0)))
                                         for mid in range(1, 8)])

    def test_old_queued_feedback_does_not_satisfy_query(self):
        self.bus.frames.append(feedback(1))
        self.bus.drop = True
        with self.assertRaisesRegex(driver.DriverError, "timeout"):
            self.arm.query_motor(1)

    def test_first_request_timeout_retries_once(self):
        send = self.bus.send

        def drop_first(can_id, payload):
            self.bus.drop = not self.bus.sent
            send(can_id, payload)

        self.bus.send = drop_first
        self.assertEqual(self.arm.query_motor(1).motor_id, 1)
        self.assertEqual(len(self.bus.sent), 2)

    def test_missing_motor_does_not_reuse_cached_state(self):
        self.arm.query_motor(1)
        self.bus.drop = True
        report = self.arm.snapshot()
        self.assertEqual(report["online"], 0)
        self.assertNotIn(1, self.arm.states)

    def test_bus_error_stops_snapshot_requests(self):
        self.bus.bus_status = 0x40000
        report = self.arm.snapshot()
        self.assertEqual(len(report["motors"]), 1)
        self.assertEqual(report["bus_status"], "0x40000")

    def test_explicit_enable_mit_disable(self):
        with self.arm:
            self.assertEqual(self.arm.enable_motor(1).state, 1)
            self.assertEqual(self.arm.command_mit(1, 0, 0, 1, 1).state, 1)
            self.assertEqual(self.arm.disable_motor(1).state, 0)

    def test_enable_seeds_current_target_around_enable(self):
        baseline = self.arm.query_motor(1)
        self.arm.enable_motor(1, kp=1.0, kd=0.2)
        expected = driver.encode_mit(1, baseline.position_rad, 0, 1.0, 0.2, 0)
        commands = [(cid, payload) for cid, payload in self.bus.sent if cid == 1]
        self.assertEqual(commands[:3], [(1, expected), (1, b"\xff" * 7 + b"\xfc"), (1, expected)])

    def test_enable_ignores_in_flight_disabled_response(self):
        disabled = driver.decode_feedback(*feedback(1, 0))
        enabled = driver.decode_feedback(*feedback(1, 1))
        with patch.object(self.arm, "query_motor", side_effect=[disabled, disabled, enabled]):
            self.assertEqual(self.arm.enable_motor(1, kp=1, kd=0.2).state, 1)

    def test_mit_command_has_no_fresh_enabled_precheck(self):
        # The frame is sent and the resulting feedback is returned even when the
        # motor reports disabled. Tracking decisions belong to the caller.
        self.assertEqual(self.arm.command_mit(1, 0, 0, 1, 1).state, 0)
        self.arm.query_motor(1)
        self.assertEqual(self.arm.command_mit(1, 0, 0, 1, 1).state, 0)
        self.assertTrue(any(cid == 1 for cid, _ in self.bus.sent))

    def test_enable_timeout_does_not_auto_disable(self):
        original_send = self.bus.send

        def send(can_id, payload):
            original_send(can_id, payload)
            if payload == b"\xff" * 7 + b"\xfc":
                self.bus.drop = True

        self.bus.send = send
        with self.assertRaises(driver.DriverError):
            self.arm.enable_motor(1)
        self.assertNotIn((1, b"\xff" * 7 + b"\xfd"), self.bus.sent)
        self.assertTrue(self.bus.enabled)

    def test_wrong_mode_does_not_block_enable(self):
        self.bus.mode = 2
        self.assertEqual(self.arm.enable_motor(1).state, 1)
        self.assertFalse(any(cid == 0x7FF and payload[2] == 0x33
                             for cid, payload in self.bus.sent))

    def test_parameter_reply_is_separate_from_motor_state(self):
        self.assertEqual(self.arm.read_parameter(1, 10), 1)
        self.assertEqual(self.arm.states, {})

    def test_cli_normal_entry_point(self):
        opened = mock_open()
        with patch.object(Path, "open", opened), patch.object(Path, "mkdir"):
            with patch.object(driver, "PcanBus", return_value=self.bus):
                with contextlib.redirect_stdout(io.StringIO()):
                    code = driver.main(["status", "--side", "right", "--log", "memory.jsonl"])
            self.assertEqual(code, 0)
            saved = "".join(call.args[0] for call in opened().write.call_args_list)
            self.assertIn('"online": 7', saved)
        self.assertFalse(self.bus.opened)


class NativeTransportTests(unittest.TestCase):
    def tearDown(self):
        self.bus.close()

    def setUp(self):
        self.dll = MagicMock()
        for name in ("CAN_SetValue", "CAN_InitializeFD", "CAN_Uninitialize",
                     "CAN_GetStatus", "CAN_WriteFD", "CAN_GetErrorText"):
            getattr(self.dll, name).return_value = 0
        self.dll.CAN_ReadFD.return_value = driver.QUEUE_EMPTY
        with patch.object(driver.sys, "platform", "win32"):
            with patch.object(driver.ct, "WinDLL", return_value=self.dll):
                self.bus = driver.PcanBus(0x51)

    def test_native_open_uses_exact_timing_and_closes(self):
        self.bus.open()
        self.dll.CAN_InitializeFD.assert_called_once_with(0x51, driver.BIT_TIMING)
        self.bus.close()
        self.dll.CAN_Uninitialize.assert_called_once_with(0x51)

    def test_status_failure_after_open_closes_channel(self):
        self.dll.CAN_GetStatus.return_value = 0x40000
        with self.assertRaises(driver.DriverError):
            self.bus.open()
        self.assertFalse(self.bus.opened)
        self.dll.CAN_Uninitialize.assert_called_once()

    def test_close_failure_releases_channel_lease(self):
        self.bus.open()
        self.dll.CAN_Uninitialize.return_value = 0x1400
        with self.assertRaises(driver.DriverError):
            self.bus.close()
        self.assertFalse(self.bus.opened)
        self.assertIsNone(self.bus.lease.stream)

    def test_native_send_uses_fd_brs(self):
        self.bus.open()
        self.bus.send(0x7FF, bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)))
        frame = self.dll.CAN_WriteFD.call_args.args[1]._obj
        self.assertEqual((frame.can_id, frame.kind, frame.dlc), (0x7FF, 0x0C, 8))
        self.assertEqual(bytes(frame.data[:8]), bytes((1, 0, 0xCC, 0, 0, 0, 0, 0)))

    def test_error_status_prevents_transmit(self):
        self.bus.open()
        self.dll.CAN_GetStatus.return_value = 0x10
        with self.assertRaises(driver.DriverError):
            self.bus.send(1, bytes(8))
        self.dll.CAN_WriteFD.assert_not_called()


if __name__ == "__main__":
    unittest.main()
