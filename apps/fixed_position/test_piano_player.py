import copy
import json
import unittest

import piano_player as player


class PlayerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((player.ROOT / "piano_config.json").read_text(encoding="utf-8"))

    def test_placeholders_rejected_before_connection(self):
        self.cfg["port"] = "COM5"
        self.cfg["rest_pose"] = dict.fromkeys(player.NAMES)
        self.cfg["press"] = dict.fromkeys(player.FINGERS)
        with self.assertRaisesRegex(ValueError, "位置尚未填写"):
            player.validate(self.cfg, True)
        self.assertEqual(len(player.validate(self.cfg)), 10)

    def test_invalid_values(self):
        for value in (float("nan"), True, -1, 91):
            cfg = copy.deepcopy(self.cfg)
            cfg["press"]["index"] = value
            with self.assertRaises(ValueError):
                player.validate(cfg)

    def test_order_release_and_timing(self):
        now = [0.0]
        sent = []
        def sleep(seconds):
            now[0] += seconds
        motion = player.Motion(lambda p: sent.append(tuple(p)), [0], 100, lambda: now[0], sleep)
        targets = {"rest": [0], "index": [1], "middle": [2], "ring": [3]}
        player.play(motion, targets, self.cfg["timing"], player.FINGERS, 2)
        extremes = [v[0] for v in sent if v[0] in (0, 1, 2, 3)]
        self.assertEqual(extremes, [1, 0, 2, 0, 3, 0] * 2)
        self.assertAlmostEqual(now[0], 3.0)
        self.assertEqual(motion.pose, [0])

    def test_conversion_keeps_other_fingers_fixed(self):
        self.cfg["rest_pose"] = dict.fromkeys(player.NAMES, 10)
        self.cfg["press"] = dict.fromkeys(player.FINGERS, 20)
        targets, _ = player.build_targets(self.cfg)
        for finger in player.FINGERS:
            changed = [i for i, (a, b) in enumerate(zip(targets["rest"], targets[finger])) if abs(a-b) > 1e-8]
            self.assertEqual(changed, [player.NAMES.index(finger)])

    def test_write_error_stops_motion(self):
        def fail(pose):
            raise IOError("disconnected")
        motion = player.Motion(fail, [0], 100, lambda: 0, lambda _: None)
        with self.assertRaises(IOError):
            motion.move([1], 0.1)
        self.assertEqual(motion.pose, [0])


if __name__ == "__main__":
    unittest.main()
