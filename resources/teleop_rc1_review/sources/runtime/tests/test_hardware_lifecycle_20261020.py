import importlib.util
from datetime import datetime, timezone, timedelta
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ops" / "hardware_lifecycle_20261020.py"
SPEC = importlib.util.spec_from_file_location("hardware_lifecycle", SOURCE)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def status(sides=()):
    return {"mode": "original_standby", "ros_domain_id": 94, "joy": {}, "can": {"motors": [
        {"channel": channel, "id": i, "state": int(side in sides), "age_ms": 10,
         "position_rad": i / 100} for channel, side in (("can0", "right"), ("can1", "left"))
        for i in range(1, 9)]}}


class Fake:
    def __init__(self):
        self.hardware = dict.fromkeys(m.HARDWARE, 1)
        self.controllers = {}
        self.events = []
        self.sides = set()
        self.drift = False
        self.fail_left = False
        self.quiet = True

    def assert_single_manager(self):
        self.events.append(("single",))

    def inventory(self):
        return dict(self.hardware), dict(self.controllers)

    def bridge_false(self):
        self.events.append(("bridge_false",))

    def establish_quiet(self, duration):
        self.events.append(("quiet", duration))

    def assert_quiet(self):
        m.require(self.quiet, "Command flow")

    def status(self):
        return status(self.sides)

    def initial_proof(self):
        return {"historical_query_not_live": True, "age_s": 2}

    def command_counts(self):
        return dict.fromkeys(m.COMMAND_TOPICS, 0)

    def load(self, name):
        self.events.append(("load", name))
        self.controllers[name] = "unconfigured"

    def configure(self, name):
        self.events.append(("configure", name))
        self.controllers[name] = "inactive"

    def unload(self, name):
        self.events.append(("unload", name))
        self.controllers.pop(name)

    def hardware_state(self, name, state):
        self.events.append(("hardware", name, state))
        if self.fail_left and name == m.HARDWARE[1] and state == 3:
            raise RuntimeError("left activation failure")
        self.hardware[name] = state
        side = "right" if name == m.HARDWARE[0] else "left"
        if state == 3:
            self.sides.add(side)
        else:
            self.sides.discard(side)

    def wait_feedback(self, sides, quiet, timeout):
        return m.validate_can(quiet(), sides)

    def switch(self, activate, deactivate):
        self.events.append(("switch", tuple(activate), tuple(deactivate)))
        for name in activate:
            self.controllers[name] = "active"
        for name in deactivate:
            self.controllers[name] = "inactive"

    def wait_joints(self, timeout):
        self.events.append(("joints",))
        return dict.fromkeys(m.JOINTS, 0.0)

    def current_joints(self, max_age):
        joints = dict.fromkeys(m.JOINTS, 0.0)
        joints[m.JOINTS[0]] = 0.11 if self.drift else 0
        return joints

    def observe_hold(self, duration, check):
        self.events.append(("hold", duration))
        check()


def run(action="initialize-enable", io=None):
    io = io or Fake()
    receipt = {"events": []}
    return io, m.execute(action, io, receipt)


class LifecycleTests(unittest.TestCase):
    def test_environment_is_fixed(self):
        m.validate_domain({"ROS_DOMAIN_ID": "94", "ROS_LOCALHOST_ONLY": "1"})
        for environment in ({}, {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"},
                            {"ROS_DOMAIN_ID": "94", "ROS_LOCALHOST_ONLY": "0"}):
            with self.assertRaises(RuntimeError):
                m.validate_domain(environment)

    def test_success_has_inactive_preparation_direct_activation_and_no_targets(self):
        io, receipt = run()
        self.assertTrue(receipt["success"], receipt)
        hardware_events = [event for event in io.events if event[0] == "hardware"]
        self.assertEqual(hardware_events, [("hardware", name, 3) for name in m.HARDWARE])
        last_prepare = max(i for i, event in enumerate(io.events) if event[0] == "configure")
        first_hw = min(i for i, event in enumerate(io.events) if event[0] == "hardware")
        self.assertLess(last_prepare, first_hw)
        self.assertEqual(io.events[1], ("bridge_false",))
        self.assertLess(io.events.index(("switch", (m.JSB,), ())), io.events.index(("joints",)))
        self.assertLess(io.events.index(("joints",)), io.events.index(("switch", m.COMMAND_CONTROLLERS, ())))
        self.assertEqual(len(receipt["final"]["joints"]), 16)
        self.assertFalse(receipt["bridge_enable_requested"])
        self.assertEqual(receipt["command_counts_before"], receipt["command_counts_after"])

    def test_unknown_controller_refused_without_touching_hardware(self):
        io = Fake()
        io.controllers["other"] = "active"
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertFalse(any(e[0] in ("hardware", "load", "switch") for e in io.events))

    def test_existing_hw_active_refused(self):
        io = Fake()
        io.hardware[m.HARDWARE[0]] = 3
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertFalse(any(e[0] == "hardware" for e in io.events))

    def test_command_flow_prevents_enable(self):
        io = Fake()
        io.quiet = False
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertFalse(any(e[0] == "hardware" for e in io.events))

    def test_left_failure_rolls_back_only_attempted_hardware(self):
        io = Fake()
        io.fail_left = True
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertEqual(io.hardware, dict.fromkeys(m.HARDWARE, 1))
        self.assertEqual(io.controllers, {})
        self.assertIn(("hardware", m.HARDWARE[0], 1), io.events)
        self.assertFalse(any(e[0] == "hardware" and e[2] == 2 for e in io.events))

    def test_hold_drift_deactivates_controllers_before_hw1(self):
        io = Fake()
        io.drift = True
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertIn("drift", receipt["error"])
        switch_off = io.events.index(("switch", (), m.COMMAND_CONTROLLERS))
        jsb_off = io.events.index(("switch", (), (m.JSB,)))
        hw_off = io.events.index(("hardware", m.HARDWARE[0], 1))
        self.assertLess(switch_off, jsb_off)
        self.assertLess(jsb_off, hw_off)
        self.assertTrue(receipt["shutdown"]["verified_hardware_unconfigured"])
        self.assertEqual(io.controllers, {})

    def test_disable_exact_known_controllers_no_position_publication(self):
        io = Fake()
        io.hardware = dict.fromkeys(m.HARDWARE, 3)
        io.controllers = dict.fromkeys(m.CONTROLLERS, "active")
        io.sides = {"left", "right"}
        io, receipt = run("disable", io)
        self.assertTrue(receipt["success"], receipt)
        self.assertEqual(io.hardware, dict.fromkeys(m.HARDWARE, 1))
        self.assertEqual(io.controllers, {})
        self.assertEqual(io.events[1], ("bridge_false",))
        self.assertFalse(any(e[0] in ("configure", "load") for e in io.events))

    def test_shutdown_attempts_second_arm_when_first_rpc_fails(self):
        io = Fake()
        io.hardware = dict.fromkeys(m.HARDWARE, 3)
        io.controllers = dict.fromkeys(m.CONTROLLERS, "active")
        original = io.hardware_state

        def fail_right(name, state):
            if name == m.HARDWARE[0] and state == 1:
                io.events.append(("failed_hw1", name))
                raise RuntimeError("injected shutdown RPC failure")
            return original(name, state)

        io.hardware_state = fail_right
        io, receipt = run("disable", io)
        self.assertFalse(receipt["success"])
        self.assertIn(("hardware", m.HARDWARE[1], 1), io.events)
        self.assertFalse(receipt["shutdown"]["verified_hardware_unconfigured"])

    def test_failed_load_not_present_is_not_unloaded(self):
        io = Fake()

        def failed_load(name):
            raise RuntimeError("load RPC failed before resource creation")

        io.load = failed_load
        io, receipt = run(io=io)
        self.assertFalse(receipt["success"])
        self.assertNotIn("rollback_error", receipt)
        self.assertFalse(any(e[0] == "unload" for e in io.events))

    def test_stale_fault_disabled_or_duplicate_motor_is_not_enabled(self):
        for mutate in (lambda s: s["can"]["motors"][0].update(age_ms=251),
                       lambda s: s["can"]["motors"][0].update(state=8),
                       lambda s: s["can"]["motors"][0].update(state=0),
                       lambda s: s["can"]["motors"].pop(),
                       lambda s: s["can"]["motors"][0].update(position_rad=float("nan"))):
            value = status({"left", "right"})
            mutate(value)
            with self.assertRaises(RuntimeError):
                m.validate_can(value, {"left", "right"})

    def test_fresh_pressed_grip_or_home_refused(self):
        for index in (1, 4):
            buttons = [0] * 6
            buttons[index] = 1
            with self.assertRaises(RuntimeError):
                m.validate_joy({"right": {"age_ms": 0, "buttons": buttons}})
        m.validate_joy({"right": {"age_ms": 2000, "buttons": [1] * 6}})

    def test_joint_state_requires_all_finite_names(self):
        good = SimpleNamespace(name=list(m.JOINTS), position=[0.0] * 16)
        self.assertEqual(len(m.finite_joints(good)), 16)
        for invalid in (SimpleNamespace(name=good.name[:-1], position=[0.0] * 15),
                        SimpleNamespace(name=good.name, position=[float("nan")] * 16)):
            with self.assertRaises(RuntimeError):
                m.finite_joints(invalid)

    def test_finger_motor_drift_measured_in_radians(self):
        base = status({"left", "right"})["can"]["motors"]
        changed = status({"left", "right"})["can"]["motors"]
        changed[7]["position_rad"] += 0.11
        with self.assertRaisesRegex(RuntimeError, "Motor hold drift"):
            m.validate_drift(dict.fromkeys(m.JOINTS, 0), dict.fromkeys(m.JOINTS, 0), base, changed)

    def test_disabled_query_is_bounded_historical_proof(self):
        now = datetime.now(timezone.utc)
        record = {"time_utc": (now - timedelta(seconds=2)).isoformat(), "query_only": True,
                  "enable_frames_sent": 0, "motion_frames_sent": 0, "zero_frames_sent": 0,
                  "online": 16, "disabled": 16, "enabled": 0, "faulted": 0,
                  "motors": status()["can"]["motors"]}
        self.assertTrue(m.validate_query_record(record, now)["historical_query_not_live"])
        for seconds in (31, -1):
            record["time_utc"] = (now - timedelta(seconds=seconds)).isoformat()
            with self.assertRaises(RuntimeError):
                m.validate_query_record(record, now)

    def test_partial_feedback_does_not_promote_inactive_stale_side(self):
        record = status({"right"})
        for row in record["can"]["motors"]:
            if row["channel"] == "can1":
                row.update(state=None, age_ms=20000, position_rad=None)
        m.validate_can(record, {"right"}, active_only=True)
        with self.assertRaises(RuntimeError):
            m.validate_can(record, {"right", "left"})
        record["can"]["motors"][8].update(state=1)
        with self.assertRaises(m.UnsafeFeedback):
            m.validate_can(record, {"right"}, active_only=True)


if __name__ == "__main__":
    unittest.main()
