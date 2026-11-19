"""Read-only ROS graph discovery timing, using no ROS or hardware imports."""
import importlib.util
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ops" / "hardware_lifecycle_20261020.py"
SPEC = importlib.util.spec_from_file_location("lifecycle_manager_discovery", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ROOT_MANAGER = ("controller_manager", "/")


class FakeDiscovery:
    def __init__(self, observations):
        self.observations = observations
        self.now = 0.
        self.spins = []
        self.graph_reads = []

    def clock(self):
        return self.now

    def get_node_names_and_namespaces(self):
        self.graph_reads.append(self.now)
        return self.observations(self.now)

    def spin(self, duration):
        self.spins.append(duration)
        self.now += duration

    def wait(self):
        MODULE.await_single_root_manager(self, self.spin, clock=self.clock)


class ManagerDiscoveryTests(unittest.TestCase):
    def test_empty_graph_then_root_at_one_point_three_seconds_is_admitted(self):
        graph = FakeDiscovery(lambda now: [ROOT_MANAGER] if now >= 1.3 else [])
        graph.wait()
        self.assertGreaterEqual(graph.now, 1.3)
        self.assertLess(graph.now, 1.351)
        self.assertGreater(len(graph.graph_reads), 1)
        self.assertTrue(all(0 < duration <= .05 for duration in graph.spins))

    def test_always_empty_expires_at_monotonic_five_seconds(self):
        graph = FakeDiscovery(lambda _now: [])
        with self.assertRaisesRegex(RuntimeError, "5-second discovery wait"):
            graph.wait()
        self.assertAlmostEqual(graph.now, 5.)
        self.assertLessEqual(max(graph.spins), .05)

    def test_duplicate_root_or_root_with_nonroot_refuses_without_spin(self):
        for rows in ([ROOT_MANAGER, ROOT_MANAGER],
                     [ROOT_MANAGER, ("controller_manager", "/other")]):
            graph = FakeDiscovery(lambda _now: rows)
            with self.subTest(rows=rows), self.assertRaisesRegex(RuntimeError, "exactly one root"):
                graph.wait()
            self.assertEqual(graph.spins, [])

    def test_nonroot_manager_is_not_treated_as_empty_discovery(self):
        graph = FakeDiscovery(lambda _now: [("controller_manager", "/other")])
        with self.assertRaisesRegex(RuntimeError, "exactly one root"):
            graph.wait()
        self.assertEqual(graph.spins, [])

    def test_late_duplicate_or_nonroot_refuses_immediately_when_observed(self):
        for rows in ([ROOT_MANAGER, ROOT_MANAGER], [("controller_manager", "/other")]):
            graph = FakeDiscovery(lambda now: rows if now >= .2 else [])
            with self.subTest(rows=rows), self.assertRaisesRegex(RuntimeError, "exactly one root"):
                graph.wait()
            self.assertAlmostEqual(graph.now, .2)

    def test_unrelated_nodes_do_not_look_like_a_manager(self):
        graph = FakeDiscovery(lambda now: [("status", "/"), ("ik_worker", "/")]
                              + ([ROOT_MANAGER] if now >= .1 else []))
        graph.wait()
        self.assertAlmostEqual(graph.now, .1)

    def test_root_ready_needs_no_fixed_sleep(self):
        graph = FakeDiscovery(lambda _now: [ROOT_MANAGER])
        graph.wait()
        self.assertEqual(graph.now, 0.)
        self.assertEqual(graph.spins, [])

    def test_a_previous_success_is_not_cached_as_current_ownership(self):
        graph = FakeDiscovery(lambda _now: [ROOT_MANAGER])
        graph.wait()
        graph.observations = lambda _now: []
        with self.assertRaisesRegex(RuntimeError, "5-second discovery wait"):
            graph.wait()
        self.assertAlmostEqual(graph.now, 5.)


if __name__ == "__main__":
    unittest.main()
