import threading
import unittest
from unittest.mock import patch

from midi_piano.planner import ROOT, read_config, build_plan
from midi_piano.midi_reader import read_song
from midi_piano.devices import SimHand, SimArm, VirtualClock, StoppableClock, validate_hardware
from midi_piano.executor import execute


class DualTests(unittest.TestCase):
    def setUp(self):
        self.cfg=read_config(ROOT/'config_four_ab.json')
        self.song=read_song(ROOT/'examples/seven_note_scale.mid')
        self.plan=build_plan(self.song,self.cfg,.5)

    def test_two_positions_and_four_fingers(self):
        mapping={n['pitch']:(n['group'],n['finger']) for n in self.plan['notes']}
        self.assertEqual(mapping[65],('A','pinky'))
        self.assertEqual(mapping[71],('B','ring'))
        self.assertEqual({g for g,_ in mapping.values()},{'A','B'})
        clock=VirtualClock(); hand=SimHand(clock); arm=SimArm(clock,hand)
        execute(self.plan,self.cfg,hand,arm,clock,lambda _:None)
        self.assertEqual([e[2] for e in hand.events if e[1]=='press'],[n.pitch for n in self.song.notes])
        self.assertTrue(hand.released and arm.stopped)

    def test_budget_and_release_before_every_move(self):
        self.assertLessEqual(self.plan['effective_speed'],.5)
        actions=self.plan['actions']
        for a,b in zip(actions,actions[1:]):
            self.assertLessEqual(a['start']+a['duration'],b['start']+1e-8)
            if b['kind']=='move': self.assertIn(a['kind'],('prepare','release'))

    def test_stop_during_wait_move_and_press(self):
        for stage in ('wait','move','press'):
            with self.subTest(stage=stage):
                event=threading.Event(); clock=StoppableClock(event)
                hand=SimHand(clock); arm=SimArm(clock,hand)
                if stage=='wait': event.set()
                target=arm if stage=='move' else hand
                method='move_to' if stage=='move' else 'press'
                original=getattr(target,method)
                def interrupt(*args):
                    event.set(); return original(*args)
                with patch.object(target,method,side_effect=interrupt) if stage!='wait' else patch.object(hand,'connect'):
                    with self.assertRaises(InterruptedError): execute(self.plan,self.cfg,hand,arm,clock,lambda _:None)
                self.assertTrue(hand.released and arm.stopped)
                self.assertLessEqual(sum(e[1]=='press' for e in hand.events),1)

    def test_failed_arrival_at_b_blocks_b_notes(self):
        clock=VirtualClock(); hand=SimHand(clock); arm=SimArm(clock,hand)
        original=arm.wait_until_arrived
        def arrive(group,timeout):
            if group=='B': raise TimeoutError('B arrival failed')
            original(group,timeout)
        with patch.object(arm,'wait_until_arrived',side_effect=arrive):
            with self.assertRaises(TimeoutError): execute(self.plan,self.cfg,hand,arm,clock,lambda _:None)
        self.assertFalse(any(e[1]=='press' and e[2] in (67,69,71) for e in hand.events))
        self.assertTrue(hand.released and arm.stopped)

    def test_hardware_gate(self):
        with self.assertRaisesRegex(RuntimeError,'未打开任何设备'): validate_hardware(self.cfg)
