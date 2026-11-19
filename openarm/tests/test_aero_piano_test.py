"""Real installed SDK against an in-memory serial device; no hardware IO."""
import struct
import threading
import time

import pytest

from tools.aero_piano_test.core import AeroTransport, Finger, Plan, Runner, groups


@pytest.fixture
def serial_device(monkeypatch):
    sdk = pytest.importorskip("aero_open_sdk.aero_hand")

    class Serial:
        def __init__(self, *args, **kwargs):
            self.raw = [12345, 23456, 34567, 4567, 5678, 6789, 7890]
            self.frames = []
            self.response = b""
            self.closed = False
            self.fail_write = False
            self.fail_read = False
        def reset_input_buffer(self):
            self.response = b""
        def reset_output_buffer(self):
            pass
        def write(self, data):
            if self.fail_write:
                from serial import SerialTimeoutException
                raise SerialTimeoutException("test timeout")
            self.frames.append(struct.unpack("<2B7H", data))
            opcode, _, *payload = self.frames[-1]
            if opcode == sdk.CTRL_POS:
                self.raw = payload
            else:
                values = self.raw if opcode == sdk.GET_POS else [25] * 7
                self.response = struct.pack("<2B7H", opcode, 0, *values)
            return len(data)
        def flush(self):
            pass
        def read(self, count):
            if self.fail_read:
                return b""
            data, self.response = self.response[:count], self.response[count:]
            return data
        def close(self):
            self.closed = True

    monkeypatch.setattr(sdk, "Serial", Serial)
    bus = AeroTransport()
    bus.connect("FAKE_NOT_A_COM_PORT")
    yield bus, bus.hand.ser, sdk
    bus.close()


def test_connect_is_read_only_and_uses_actual_sdk(serial_device):
    bus, serial, sdk = serial_device
    assert {frame[0] for frame in serial.frames} == {sdk.GET_POS, sdk.GET_CURR, sdk.GET_TEMP, sdk.GET_VEL}
    assert bus.raw == serial.raw
    assert len(bus.last["q"]) == 7


def test_single_finger_preserves_other_channels_exactly(serial_device):
    bus, serial, sdk = serial_device
    baseline = list(serial.raw)
    runner = Runner(bus, threading.Event(), lambda *args: None)
    runner.begin()
    runner.move({3: 10.}, {3: 0.05})
    commands = [f[2:] for f in serial.frames if f[0] == sdk.CTRL_POS]
    assert commands
    for command in commands:
        assert command[:3] == tuple(baseline[:3])
        assert command[4:] == tuple(baseline[4:])
    assert commands[-1][3] == bus.target(3, 10.)
    assert set(f[0] for f in serial.frames) <= {sdk.CTRL_POS, sdk.GET_POS, sdk.GET_CURR, sdk.GET_TEMP, sdk.GET_VEL}


def test_stop_interrupts_press_and_returns_home(serial_device):
    bus, serial, sdk = serial_device
    stop = threading.Event()
    runner = Runner(bus, stop, lambda *args: None)
    original = bus.write
    home_raw = bus.target(3, 5)
    triggered = False
    def write(raw):
        nonlocal triggered
        original(raw)
        if raw[3] > home_raw + 100 and not triggered:
            triggered = True
            stop.set()
    bus.write = write
    runner.run(Plan((Finger(3, 5, 20, 0.05),), press_dwell=0, home_dwell=0, cycles=3))
    assert triggered
    assert serial.raw[3] == home_raw


def test_feedback_failure_never_sends_home_blindly(serial_device):
    bus, serial, sdk = serial_device
    original = bus.write
    def write(raw):
        original(raw)
        serial.fail_read = True
    bus.write = write
    runner = Runner(bus, threading.Event(), lambda *args: None)
    with pytest.raises(OSError, match="反馈缺失"):
        runner.run(Plan((Finger(3, 5, 20, 0.05),), cycles=1))
    count = sum(f[0] == sdk.CTRL_POS for f in serial.frames)
    assert count > 0
    assert bus.fault
    with pytest.raises(OSError):
        bus.write([0] * 7)
    assert count == sum(f[0] == sdk.CTRL_POS for f in serial.frames)


def test_official_sdk_write_timeout_is_not_swallowed(serial_device):
    bus, serial, sdk = serial_device
    serial.fail_write = True
    with pytest.raises(OSError, match="写入超时"):
        bus.write(list(bus.raw))
    assert bus.fault


def test_stale_feedback_blocks_motion(serial_device):
    bus, serial, sdk = serial_device
    bus.last["time"] = time.monotonic() - 2
    with pytest.raises(OSError, match="未更新"):
        bus.write(list(bus.raw))
    assert not any(f[0] == sdk.CTRL_POS for f in serial.frames)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, 91])
def test_bad_angles_rejected(serial_device, value):
    bus, _, _ = serial_device
    with pytest.raises(ValueError):
        bus.target(3, value)
    with pytest.raises(ValueError):
        Finger(3, 0, value)


def test_thumb_not_exposed(serial_device):
    bus, _, _ = serial_device
    with pytest.raises(ValueError):
        bus.target(2, 10)
    with pytest.raises(ValueError):
        Finger(2, 0, 10)


def test_negative_transition_and_same_positions_rejected():
    with pytest.raises(ValueError):
        Finger(3, 1, 2, -0.01)
    with pytest.raises(ValueError):
        Plan((Finger(3, 1, 1),))


def test_direct_target_sends_once_without_added_delay(serial_device, monkeypatch):
    bus, serial, sdk = serial_device
    stop = threading.Event()
    runner = Runner(bus, stop, lambda *args: None)
    runner.begin()
    before = len(serial.frames)
    def forbidden_wait(*args):
        pytest.fail("Zero transition must not add a wait")
    monkeypatch.setattr(stop, "wait", forbidden_wait)
    runner.move({3: 20}, {3: 0})
    runner.move({3: 5}, {3: 0})
    frames = serial.frames[before:]
    assert len(frames) == 2
    assert all(frame[0] == sdk.CTRL_POS for frame in frames)
    assert frames[0][5] == bus.target(3, 20)
    assert frames[1][5] == bus.target(3, 5)


def test_direct_target_still_polls_and_faults(serial_device):
    bus, serial, sdk = serial_device
    runner = Runner(bus, threading.Event(), lambda *args: None)
    runner.begin()
    serial.fail_read = True
    runner.last_poll -= 0.3
    before = len(serial.frames)
    with pytest.raises(OSError):
        runner.move({3: 20}, {3: 0})
    assert bus.fault
    assert not any(frame[0] == sdk.CTRL_POS for frame in serial.frames[before:])


def test_direct_target_requires_nonzero_dwell():
    with pytest.raises(ValueError, match="停留"):
        Plan((Finger(3, 5, 20, 0),), press_dwell=0, home_dwell=0)


def test_modes_only_use_selected_fingers():
    fingers = (Finger(3, 1, 2), Finger(5, 3, 4))
    assert list(groups(Plan(fingers, "逐指轮流", cycles=3))) == [{3}, {5}, {3}]
    assert list(groups(Plan(fingers, "全部一起", cycles=2))) == [{3, 5}, {3, 5}]
    assert all(g and g <= {3, 5} for g in groups(Plan(fingers, "随机组合", cycles=20)))


def test_demo_never_opens_serial(monkeypatch):
    sdk = pytest.importorskip("aero_open_sdk.aero_hand")
    def forbidden(*a, **kw):
        pytest.fail("Demo tried to open serial")
    monkeypatch.setattr(sdk, "Serial", forbidden)
    bus = AeroTransport(demo=True)
    bus.connect("COM8")
    runner = Runner(bus, threading.Event(), lambda *args: None)
    runner.run(Plan((Finger(3, 1, 2, 0.05),), press_dwell=0, home_dwell=0, cycles=1))
    assert bus.raw[3] == bus.target(3, 1)
    bus.close()


def test_gui_demo_connect_capture_cycle_and_close(tmp_path, monkeypatch):
    # Run the standalone app in its own interpreter, as launched in production.
    # This also keeps Tcl's native lifetime separate from the simulation UI tests.
    pytest.importorskip("aero_open_sdk.aero_hand")
    import subprocess
    import sys
    from pathlib import Path
    code = (
        "import runpy,sys,pytest; from pathlib import Path; "
        "namespace=runpy.run_path(sys.argv[1]); "
        "patch=pytest.MonkeyPatch(); "
        "namespace['_gui_demo_flow'](Path(sys.argv[2]),patch); patch.undo()"
    )
    result = subprocess.run([sys.executable, "-c", code, __file__, str(tmp_path)],
                            cwd=Path(__file__).resolve().parents[1], capture_output=True,
                            text=True, encoding="utf-8", errors="replace", timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr


def _gui_demo_flow(tmp_path, monkeypatch):
    """Exercise the actual Tk callbacks and worker without a serial device."""
    sdk = pytest.importorskip("aero_open_sdk.aero_hand")
    import tkinter as tk
    from tools.aero_piano_test import gui
    monkeypatch.setattr(gui, "ROOT", tmp_path)
    def forbidden(*a, **kw):
        pytest.fail("GUI demo tried to open serial")
    monkeypatch.setattr(sdk, "Serial", forbidden)
    root = tk.Tk()
    root.withdraw()
    app = gui.App(root, demo=True, config_path=tmp_path / "demo.json")
    def pump_until(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        assert predicate(), app.status.get()
    try:
        assert not app.connected and not app.armed.get()
        assert all(row["home"].get() == row["press"].get() == "" for row in app.rows.values())
        app.submit("connect", "COM8")
        pump_until(lambda: app.connected and not app.busy)
        assert not app.armed.get()
        app.capture(3, "home")
        app.rows[3]["press"].set(str(float(app.rows[3]["home"].get()) + 1))
        app.set_transition(0)
        assert app.rows[3]["transition"].get() == "0"
        assert app.rows[4]["transition"].get() == "0.5"
        app.set_transition(0.05)
        app.press_dwell.set("0")
        app.home_dwell.set("0")
        app.cycles.set("1")
        app.armed.set(True)
        app.cycle()
        pump_until(lambda: not app.busy)
        assert "1（" in app.counter.get()
        assert app.connected
        logs = list((tmp_path / "logs" / "aero_piano_test").glob("*_demo.csv"))
        assert len(logs) == 1
        assert "completed" in logs[0].read_text(encoding="utf-8-sig")
        # Stop must also release a manually held finger after its move completed.
        app.one(3, "press")
        pump_until(lambda: not app.busy)
        app.stop_and_home()
        pump_until(lambda: not app.busy)
        assert abs(app.feedback["q"][3] - float(app.rows[3]["home"].get())) < .01
        app.hard_stop()
        pump_until(lambda: not app.connected)
        assert not app.armed.get()
    finally:
        app.close()
        app.thread.join(timeout=2)
        assert not app.thread.is_alive()
        for callback in root.tk.splitlist(root.tk.call("after", "info")):
            root.after_cancel(callback)
        try:
            root.destroy()
        except tk.TclError:
            pass
        del app, root
        import gc
        gc.collect()
