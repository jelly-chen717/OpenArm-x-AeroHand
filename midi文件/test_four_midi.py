import json
import unittest
from types import SimpleNamespace
import piano_four_midi as player

class FourMidiTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((player.ROOT/'piano_four_config.json').read_text(encoding='utf-8'))
        self.song=player.read_song(player.ROOT/'玛丽有只小羊羔.mid')

    def test_calibration_and_independent_fingers(self):
        player.hand.validate(self.cfg,True)
        targets,_=player.hand.build_targets(self.cfg)
        for finger in player.hand.FINGERS:
            changed=[i for i,(a,b) in enumerate(zip(targets['rest'],targets[finger])) if abs(a-b)>1e-8]
            self.assertEqual(changed,[player.hand.NAMES.index(finger)])

    def test_tempo_and_release_room(self):
        for speed in (0.5,1.056,1.5):
            events,scale=player.plan(self.song,self.cfg['timing'],speed)
            self.assertEqual(len(events),26)
            self.assertAlmostEqual(scale,1/speed)
            overhead=sum(self.cfg['timing'][k] for k in ('down_seconds','up_seconds','gap_seconds'))
            for a,b in zip(events,events[1:]):
                self.assertLessEqual(a['start']+a['hold']+overhead,b['start']+1e-8)

    def test_reject_unsupported_pitch_before_connection(self):
        song=SimpleNamespace(notes=[SimpleNamespace(pitch=67,duration=1)])
        with self.assertRaises(ValueError):player.plan(song,self.cfg['timing'],0.5)

    def test_short_note_forces_uniform_slowdown(self):
        song=SimpleNamespace(notes=[SimpleNamespace(pitch=60,start=0,duration=.01)])
        events,scale=player.plan(song,self.cfg['timing'],1)
        self.assertGreater(scale,1)
        self.assertGreaterEqual(events[0]['hold'],.05)

if __name__=='__main__':unittest.main()
