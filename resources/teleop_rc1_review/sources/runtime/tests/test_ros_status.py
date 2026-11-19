"""Offline evidence/HTTP routing tests; no ROS, CAN, network, or process startup."""

import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


status = load_module("original_ros_status_test", ROOT / "app" / "ros_status.py")
supervisor = load_module("original_supervisor_test", ROOT / "ops" / "supervisor.py")


def idle_ros():
    return {"hardware": {name: {"state_id": 1, "state": "unconfigured"} for name in status.HARDWARE},
            "hardware_seen": 99.9, "controllers": {}, "controllers_seen": 99.9}


def disabled_can():
    return {"motors": [{"channel": channel, "id": motor, "state": 0, "age_ms": 10,
                        "position_rad": 0.1} for channel in ("can0", "can1") for motor in range(1, 9)]}


class OriginalStatusTests(unittest.TestCase):
    def test_status_node_creation_respects_readonly_rclpy_node_properties(self):
        class FakeNode:
            def __init__(self, name):
                self._test_name = name
                self._test_clients = []
                self._test_subscriptions = []
                self._test_timers = []

            # rclpy Node reserves these read-only collection properties. A
            # wrapper assigning one must fail this construction smoke test.
            clients = property(lambda self: ("rclpy-owned clients",))
            publishers = property(lambda self: ())
            subscriptions = property(lambda self: ())
            timers = property(lambda self: ())
            guards = property(lambda self: ())
            services = property(lambda self: ())
            waitables = property(lambda self: ())

            def create_client(self, service, endpoint):
                client = mock.Mock(name=endpoint)
                client.service_is_ready.return_value = False
                self._test_clients.append((service, endpoint))
                return client

            def create_subscription(self, message, topic, callback, qos):
                self._test_subscriptions.append((message, topic, callback, qos))
                return mock.Mock()

            def create_timer(self, period, callback):
                self._test_timers.append((period, callback))
                return mock.Mock()

        class FakeService:
            class Request:
                pass

        modules = {}
        for package in ("rclpy", "controller_manager_msgs", "geometry_msgs", "sensor_msgs", "std_msgs", "trajectory_msgs"):
            modules[package] = types.ModuleType(package)
            modules[package].__path__ = []
        definitions = {
            "rclpy.node": {"Node": FakeNode},
            "controller_manager_msgs.srv": {"ListControllers": FakeService, "ListHardwareComponents": FakeService},
            "geometry_msgs.msg": {"PoseStamped": type("PoseStamped", (), {})},
            "sensor_msgs.msg": {"JointState": type("JointState", (), {}), "Joy": type("Joy", (), {})},
            "std_msgs.msg": {"Float64MultiArray": type("Float64MultiArray", (), {})},
            "trajectory_msgs.msg": {"JointTrajectory": type("JointTrajectory", (), {})},
        }
        for name, attributes in definitions.items():
            module = types.ModuleType(name)
            for key, value in attributes.items():
                setattr(module, key, value)
            modules[name] = module
        with mock.patch.dict(sys.modules, modules):
            node = status.create_ros_node(status.StatusStore())
        self.assertEqual(node.clients, ("rclpy-owned clients",))
        self.assertEqual(set(node.status_clients), {"hardware", "controllers"})
        self.assertEqual(node.pending, {})
        self.assertEqual(len(node._test_clients), 2)
        self.assertEqual(len(node._test_subscriptions), 9)
        self.assertEqual(len(node._test_timers), 1)
        # Both clients are read-only list services; no publisher, lifecycle
        # setter, enable service, hardware observer, or executor was created.
        self.assertEqual({entry[1] for entry in node._test_clients}, {
            "/controller_manager/list_hardware_components", "/controller_manager/list_controllers"})

    def test_unobserved_motors_remain_unknown_despite_idle_ros_and_old_all_off(self):
        history = {"time_utc": "2026-10-19T06:04:57Z", "disabled": 16, "enabled": 0}
        result = status.build_status(idle_ros(), {}, 100, history)
        self.assertEqual(result["state"], "unknown")
        self.assertEqual((result["online"], result["disabled"], result["enabled"]), (0, 0, 0))
        self.assertIsNone(result["output_enabled"])
        self.assertIsNone(result["hardware_connected"])
        self.assertFalse(result["motor_state_verified"])
        self.assertTrue(result["driver_quiescent"])
        self.assertEqual(result["last_motor_check"], history)
        self.assertTrue(all(row["position_deg"] is None for row in result["joints"]))

    def test_full_fresh_disabled_feedback_is_required_for_live_sixteen_off(self):
        result = status.build_status(idle_ros(), disabled_can(), 100)
        self.assertEqual(result["state"], "disabled")
        self.assertEqual(result["disabled"], 16)
        self.assertEqual(result["online"], 16)
        self.assertIs(result["output_enabled"], False)
        self.assertTrue(result["motor_state_verified"])
        self.assertTrue(result["hardware_connected"])
        self.assertEqual(len(result["joints"]), 14)
        self.assertFalse(result["control_available"])
        self.assertIsNone(result["initialized"])

    def test_stale_can_counts_and_positions_cannot_be_reused(self):
        can = disabled_can()
        can.update(enabled=16, disabled=0, online=16)
        for motor in can["motors"]:
            motor.update(state=1, age_ms=900)
        result = status.build_status(idle_ros(), can, 100)
        self.assertEqual(result["online"], 0)
        self.assertEqual(result["enabled"], 0)
        self.assertIsNone(result["output_enabled"])
        self.assertTrue(all(row["position_deg"] is None for row in result["joints"]))

    def test_either_fresh_active_hardware_or_enabled_motor_reports_output(self):
        ros = idle_ros()
        ros["hardware"][status.HARDWARE[0]]["state_id"] = 3
        self.assertIs(status.build_status(ros, {}, 100)["output_enabled"], True)
        ros["hardware_seen"] = 94.0
        self.assertIsNone(status.build_status(ros, {}, 100)["output_enabled"])
        can = disabled_can()
        can["motors"][0]["state"] = 1
        result = status.build_status(idle_ros(), can, 100)
        self.assertIs(result["output_enabled"], True)
        self.assertTrue(result["known_motor_enabled"])
        self.assertEqual(result["state"], "mixed")

    def test_quiescence_and_startup_require_two_fresh_hardware_states_and_no_controllers(self):
        ros = idle_ros()
        current = status.build_status(ros, {}, 100)
        self.assertTrue(current["driver_quiescent"])
        self.assertTrue(supervisor.standby_ready(current))
        for change in ("active", "stale_hardware", "stale_controllers", "loaded", "missing"):
            altered = copy.deepcopy(ros)
            if change == "active":
                altered["hardware"][status.HARDWARE[0]]["state_id"] = 3
            elif change == "stale_hardware":
                altered["hardware_seen"] = 94
            elif change == "stale_controllers":
                altered["controllers_seen"] = 94
            elif change == "loaded":
                altered["controllers"] = {"right_forward_position_controller": "inactive"}
            else:
                del altered["hardware"][status.HARDWARE[0]]
            with self.subTest(change=change):
                result = status.build_status(altered, {}, 100)
                self.assertFalse(result["driver_quiescent"])
                self.assertFalse(supervisor.standby_ready(result))

    def test_get_routes_and_all_posts_are_readonly_without_opening_server(self):
        snapshot = mock.Mock(return_value={"mode": "original_standby"})
        handler_class = status.make_handler(snapshot)
        handler = handler_class.__new__(handler_class)
        handler.reply = mock.Mock()
        for path in ("/status", "/state", "/state?poll=1"):
            handler.path = path
            handler_class.do_GET(handler)
            handler.reply.assert_called_with(200, {"mode": "original_standby"})
        snapshot.reset_mock()
        for path in ("/set", "/initialize", "/enable", "/anything"):
            handler.path = path
            handler_class.do_POST(handler)
            args = handler.reply.call_args.args
            self.assertEqual(args[0], 409)
            self.assertFalse(args[1]["control_available"])
            self.assertTrue(handler.close_connection)
        snapshot.assert_not_called()

    def test_metrics_do_not_turn_cached_messages_into_current_traffic(self):
        result = status.rate([97.0, 98.1, 98.2], 200, 100.0)
        self.assertAlmostEqual(result["hz"], 10.0)
        self.assertEqual(result["count"], 200)
        self.assertAlmostEqual(result["age_ms"], 1800)
        self.assertEqual(status.rate([97.0], 200, 100.0)["hz"], 0)


if __name__ == "__main__":
    unittest.main()
