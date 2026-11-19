"""Exercise the viewer lifecycle without opening a graphics window."""
from contextlib import nullcontext
from types import SimpleNamespace
import pytest
import mujoco.viewer
import openarm_app.graphics
import openarm_app.simulation as module


@pytest.mark.parametrize("terminal", ["PASSED", "NOT_REACHED", "FAULT"])
@pytest.mark.parametrize("exit_method", ["q", "close"])
def test_finished_viewer_holds_and_saves_before_exit(monkeypatch, tmp_path, terminal, exit_method):
    counts = {"steps": 0, "saves": 0, "frames": 0}
    sim = SimpleNamespace(mode="ideal", model=None, data=None, dt=1, status="RUNNING")

    def step():
        counts["steps"] += 1
        sim.status = terminal

    def save(directory):
        assert directory == tmp_path
        counts["saves"] += 1
        return {"status": sim.status}

    sim.step, sim.save = step, save

    class Viewer:
        def __init__(self, callback):
            self.callback = callback
        def __enter__(self):
            self.callback(32)
            return self
        def __exit__(self, *args):
            pass
        def lock(self):
            return nullcontext()
        def is_running(self):
            if counts["frames"]:
                assert counts["saves"] == 1  # Saved while the window is still open.
                assert counts["steps"] == 1
            if counts["frames"] > 5:
                pytest.fail("Viewer did not exit")
            return exit_method == "q" or counts["frames"] < 3
        def sync(self):
            counts["frames"] += 1
            if counts["frames"] == 2:
                self.callback(32)  # Space after completion must not restart.
            if counts["frames"] == 3 and exit_method == "q":
                self.callback(81)

    monkeypatch.setattr(openarm_app.graphics, "record_graphics", lambda directory: None)
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda *a, key_callback: Viewer(key_callback))
    monkeypatch.setattr(module.time, "sleep", lambda duration: None)
    module.run(sim, False, tmp_path)
    assert counts == {"steps": 1, "saves": 1, "frames": 3}
    assert sim.status == terminal  # Exiting must not turn a completed result into STOPPED.
