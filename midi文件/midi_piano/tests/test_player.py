import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from unittest.mock import patch
import uuid

import mido

from midi_piano.devices import SerialHand, SimArm, SimHand, VirtualClock, validate_hardware
from midi_piano.executor import execute
from midi_piano.midi_reader import read_song, streams, load
from midi_piano.planner import ROOT, MAPPING, build_plan, read_config, validate_config


class PlayerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scratch = ROOT / "tests" / (".scratch-" + uuid.uuid4().hex)
        cls.scratch.mkdir()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.scratch)

    def setUp(self):
        self.cfg = read_config()

    def midi(self, events, other=None, tempo=None, kind=1):
        midi = mido.MidiFile(type=kind, ticks_per_beat=480)
        midi.tracks.append(mido.MidiTrack(tempo or [mido.MetaMessage("set_tempo", tempo=500000)]))
        midi.tracks.append(mido.MidiTrack(events))
        if other:
            midi.tracks.append(mido.MidiTrack(other))
        path = self.scratch / (uuid.uuid4().hex + ".mid")
        midi.save(path)
        return path

    @staticmethod
    def melody(pitches=(60,62,64), duration=480):
        return [msg for pitch in pitches for msg in (
            mido.Message("note_on", note=pitch, velocity=80),
            mido.Message("note_off", note=pitch, time=duration))]

    def fixture(self, pitches=(60,62,64)):
        song = read_song(self.midi(self.melody(pitches)))
        return build_plan(song, self.cfg)

    def test_examples_and_seven_notes(self):
        ode = read_song(ROOT / "examples" / "ode_to_joy.mid")
        self.assertEqual(len(ode.notes), 30)
        self.assertAlmostEqual(ode.end_seconds, 32 * 60 / 90, places=4)
        scale = read_song(ROOT / "examples" / "seven_note_scale.mid")
        self.assertEqual({n.pitch for n in scale.notes}, set(MAPPING))

    def test_tempo_change_from_unselected_track(self):
        path = self.midi(self.melody((60,), 960), tempo=[
            mido.MetaMessage("set_tempo", tempo=500000),
            mido.MetaMessage("set_tempo", tempo=1000000, time=480)])
        self.assertAlmostEqual(read_song(path).notes[0].duration, 1.5)

    def test_rest_velocity_zero_and_repeat(self):
        path = self.midi([mido.Message("note_on", note=60, velocity=50, time=480),
                          mido.Message("note_on", note=60, velocity=0, time=480),
                          mido.Message("note_on", note=60, velocity=70, time=480),
                          mido.Message("note_off", note=60, time=480)])
        notes = read_song(path).notes
        self.assertEqual([(n.start, n.duration, n.velocity) for n in notes], [(0.5,0.5,50),(1.5,0.5,70)])

    def test_same_tick_note_off_sorted_before_on(self):
        path = self.midi([mido.Message("note_on", note=60),
                          mido.Message("note_on", note=62, time=480),
                          mido.Message("note_off", note=60),
                          mido.Message("note_off", note=62, time=480)])
        self.assertEqual([n.pitch for n in read_song(path).notes], [60,62])

    def test_track_selection(self):
        path = self.midi(self.melody(), other=self.melody((67,)))
        with self.assertRaisesRegex(ValueError, "唯一"):
            read_song(path)
        self.assertEqual(len(streams(load(path))), 2)
        self.assertEqual(read_song(path, track=2).notes[0].pitch, 67)

    def test_channel_selection(self):
        events = self.melody((60,)) + [m.copy(channel=1) for m in self.melody((62,))]
        path = self.midi(events)
        with self.assertRaisesRegex(ValueError, "唯一"):
            read_song(path)
        self.assertEqual(read_song(path, channel=2).notes[0].pitch, 62)

    def test_chord_and_overlap_rejected(self):
        for delay in (0, 240):
            with self.subTest(delay=delay):
                path = self.midi([mido.Message("note_on", note=60), mido.Message("note_on", note=64, time=delay)])
                with self.assertRaisesRegex(ValueError, "和弦|重叠"):
                    read_song(path)

    def test_missing_and_orphan_off_rejected(self):
        for events in ([mido.Message("note_on", note=60)], [mido.Message("note_off", note=60)]):
            with self.assertRaises(ValueError):
                read_song(self.midi(events))

    def test_transpose_and_out_of_range(self):
        path = self.midi(self.melody((72,)))
        with self.assertRaisesRegex(ValueError, "超出七音"):
            build_plan(read_song(path), self.cfg)
        self.assertEqual(build_plan(read_song(path, transpose=-12), self.cfg)["notes"][0]["pitch"], 60)
        with self.assertRaisesRegex(ValueError, "超出七音"):
            self.fixture((61,))

    def test_pedal_warning_bend_and_drums(self):
        song = read_song(self.midi([mido.Message("control_change", control=64, value=127)] + self.melody()))
        self.assertTrue(song.warnings)
        with self.assertRaisesRegex(ValueError, "弯音"):
            read_song(self.midi([mido.Message("pitchwheel", pitch=100)] + self.melody()))
        with self.assertRaisesRegex(ValueError, "打击乐"):
            read_song(self.midi([m.copy(channel=9) for m in self.melody()]))

    def test_corrupt_type2_and_tempo_conflict(self):
        bad = self.scratch / "bad.mid"
        bad.write_bytes(b"not midi")
        with self.assertRaises(ValueError):
            read_song(bad)
        with self.assertRaisesRegex(ValueError, "Type"):
            read_song(self.midi(self.melody(), kind=2))
        with self.assertRaisesRegex(ValueError, "冲突"):
            read_song(self.midi([mido.MetaMessage("set_tempo", tempo=600000)] + self.melody()))

    def test_type0_supported_smpte_rejected(self):
        midi = mido.MidiFile(type=0, ticks_per_beat=480)
        midi.tracks.append(mido.MidiTrack(self.melody()))
        path = self.scratch / "type0.mid"
        midi.save(path)
        self.assertEqual(len(read_song(path).notes), 3)
        data = bytearray(path.read_bytes())
        data[12:14] = bytes((0xE7, 40))
        path.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "SMPTE"):
            read_song(path)

    def test_group_mapping_and_release_before_move(self):
        plan = self.fixture(tuple(MAPPING))
        self.assertEqual([n["group"] for n in plan["notes"]], list("AAABBBC"))
        self.assertEqual(plan["group_changes"], 2)
        for i, action in enumerate(plan["actions"]):
            if action["kind"] == "move":
                self.assertIn(plan["actions"][i - 1]["kind"], ("prepare", "release"))
        self.assertEqual(sum(a["kind"] == "move" for a in self.fixture()["actions"]), 1)

    def test_global_scale_and_motion_budget(self):
        plan = self.fixture((60,65,71,60))
        self.assertGreater(plan["time_scale"], 1)
        for a, b in zip(plan["notes"], plan["notes"][1:]):
            self.assertAlmostEqual(b["onset"] - a["onset"], (b["source_start"] - a["source_start"]) * plan["time_scale"])
            self.assertGreaterEqual(b["onset"] - a["onset"] - a["hold"] + 1e-8, 2 + .15 + .05 + .12)
        self.assertTrue(all(b["start"] + 1e-8 >= a["start"] + a["duration"] for a,b in zip(plan["actions"], plan["actions"][1:])))

    def test_slow_request_preserved(self):
        plan = build_plan(read_song(self.midi(self.melody())), self.cfg, speed=.1)
        self.assertAlmostEqual(plan["effective_speed"], .1)

    def test_config_and_speed_validation(self):
        for value in (float("nan"), float("inf"), 0, -1, True):
            with self.assertRaises(ValueError):
                build_plan(read_song(self.midi(self.melody())), self.cfg, value)
        for value in (float("nan"), True, 100):
            cfg = copy.deepcopy(self.cfg)
            cfg["notes"]["60"]["press_degrees"] = value
            with self.assertRaises(ValueError):
                validate_config(cfg)

    def test_virtual_execution_releases_every_note(self):
        plan = self.fixture(tuple(MAPPING))
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        execute(plan, self.cfg, hand, arm, clock, lambda _: None)
        self.assertEqual([e[2] for e in hand.events if e[1] == "press"], list(MAPPING))
        self.assertTrue(hand.released)
        self.assertTrue(arm.stopped)
        self.assertAlmostEqual(clock.now(), plan["total_seconds"])

    def test_arrival_timeout_never_presses(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        with patch.object(arm, "wait_until_arrived", side_effect=TimeoutError("arrival")):
            with self.assertRaises(TimeoutError):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
        self.assertFalse(any(e[1] == "press" for e in hand.events))
        self.assertTrue(arm.stopped)

    def test_move_failure_stops_before_press(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        with patch.object(arm, "move_to", side_effect=IOError("move failed")):
            with self.assertRaises(IOError):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
        self.assertFalse(any(e[1] == "press" for e in hand.events))
        self.assertTrue(arm.stopped)

    def test_release_failure_still_closes_devices(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        original = hand.release
        def release(duration):
            if not hand.released:
                raise IOError("release failed")
            original(duration)
        with patch.object(hand, "release", side_effect=release), patch.object(hand, "close") as close_hand, patch.object(arm, "close") as close_arm:
            with self.assertRaises(IOError):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
            close_hand.assert_called_once()
            close_arm.assert_called_once()

    def test_no_arrival_ack_never_presses(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        with patch.object(arm, "wait_until_arrived", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "未确认"):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
        self.assertFalse(any(e[1] == "press" for e in hand.events))

    def test_press_failure_and_interrupt_cleanup(self):
        for error in (IOError("write failed"), KeyboardInterrupt()):
            clock = VirtualClock()
            hand = SimHand(clock)
            arm = SimArm(clock, hand)
            def fail(*args):
                hand.released = False
                raise error
            with patch.object(hand, "press", side_effect=fail), patch.object(hand, "close") as close:
                with self.assertRaises(type(error)):
                    execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
                close.assert_called_once()
            self.assertTrue(hand.released)
            self.assertTrue(arm.stopped)

    def test_late_action_stops_without_following_notes(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        original = hand.press
        def slow(pitch, duration):
            original(pitch, duration)
            clock.sleep(1)
        with patch.object(hand, "press", side_effect=slow):
            with self.assertRaises(TimeoutError):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
        self.assertEqual(sum(e[1] == "press" for e in hand.events), 1)
        self.assertTrue(hand.released)

    def test_cleanup_continues_if_stop_fails(self):
        clock = VirtualClock()
        hand = SimHand(clock)
        arm = SimArm(clock, hand)
        with patch.object(arm, "stop", side_effect=IOError("stop")), patch.object(hand, "close") as close:
            with self.assertRaises(RuntimeError):
                execute(self.fixture(), self.cfg, hand, arm, clock, lambda _: None)
            close.assert_called_once()

    def test_hardware_gate_even_when_config_filled(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["hand"].update(port="COM999", rest_pose=[10] * 7)
        for entry in cfg["notes"].values():
            entry["press_degrees"] = 20
        cfg["arm"].update(backend="pretend_ready", poses={g: [0] * 7 for g in "ABC"}, timing_is_measured=True)
        with patch("serial.Serial") as serial:
            for config in (self.cfg, cfg):
                with self.assertRaisesRegex(RuntimeError, "未打开任何设备"):
                    validate_hardware(config)
            serial.assert_not_called()

    def calibrated_hand(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["hand"].update(port="COM999", rest_pose=[10] * 7)
        for n in cfg["notes"].values():
            n["press_degrees"] = 20
        return SerialHand(cfg, VirtualClock())

    def test_hand_conversion_only_changes_selected_motor(self):
        hand = self.calibrated_hand()
        for pitch, (_, finger) in MAPPING.items():
            changed = [i for i,(a,b) in enumerate(zip(hand.targets["rest"], hand.targets[pitch])) if abs(a-b) > 1e-8]
            self.assertEqual(changed, [{"index":3,"middle":4,"ring":5}[finger]])

    def test_serial_partial_write_and_invalid_read(self):
        from unittest.mock import Mock
        hand = self.calibrated_hand()
        hand.ser = Mock()
        hand.ser.write.return_value = 1
        with self.assertRaises(IOError):
            hand.send(hand.targets["rest"])
        hand.ser.write.return_value = 16
        hand.ser.read.return_value = b"bad"
        with self.assertRaises(IOError):
            hand.read_pose()
        self.assertIsNone(hand.pose)

    def test_cli_fast_and_run_blocked(self):
        output = self.scratch / "smoke.plan.json"
        cmd = [sys.executable, str(ROOT / "player.py")]
        result = subprocess.run(cmd + ["simulate", "--fast", "--output", str(output)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(json.loads(output.read_text(encoding="utf-8"))["notes"]), 30)
        result = subprocess.run(cmd + ["run", "--output", str(output)], capture_output=True)
        self.assertEqual(result.returncode, 1)


if __name__ == "__main__":
    unittest.main()
