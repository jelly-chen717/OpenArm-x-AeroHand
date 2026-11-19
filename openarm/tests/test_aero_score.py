from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from tools.aero_score.core import (CHARS, Event, FingerScore, Gate, HandLink, Prepared,
    compile_score, execute, prepare, run_dual, validate_calibration, validate_ports)


TEXT = """left_arm_move(1,0.01)
right_arm_move(24,0.01)
sleep(0.05)
hand(D,1234789x)
sleep(0.1)
hand(U,1234789x)
sleep(0.05)
"""
CAL = {ch: dict(home=0, press=20) for ch in (3, 4, 5, 6)}


def test_parser_maps_both_hands_and_retains_timing():
    score = compile_score(TEXT)
    assert score.ignored_moves == 2
    assert score.duration_us == 200000
    assert score.note_count == 8
    assert score.sides == ("left", "right")
    for side in score.sides:
        assert [e.time_us for e in score.events[side]] == [50000, 150000]
        assert score.events[side][0].changes == tuple((ch, True) for ch in (3, 4, 5, 6))
    assert CHARS["left"] == {3: "4", 4: "3", 5: "2", 6: "1"}


def test_same_timestamp_groups_preserve_changes():
    score = compile_score(TEXT.replace("hand(D,1234789x)", "hand(D,4)\nhand(D,3)\nhand(D,12789x)"))
    assert len(score.events["left"]) == 2
    assert len(score.events["left"][0].lines) == 3


def test_no_zero_duration_reversal_or_five_fingers():
    with pytest.raises(ValueError, match="同一时刻"):
        compile_score(TEXT.replace("sleep(0.1)", "sleep(0)"))
    with pytest.raises(ValueError, match="不允许"):
        compile_score(TEXT.replace("1234789x", "12345789x"))
    with pytest.raises(ValueError, match="未知指令"):
        compile_score("__import__('os').system('echo bad')")


def test_ports_and_calibration_are_not_guessed():
    validate_ports({"left": dict(demo=True, port="COM1"), "right": dict(demo=False, port="COM8")})
    with pytest.raises(ValueError, match="同一个"):
        validate_ports({s: dict(demo=False, port="com8") for s in ("left", "right")})
    with pytest.raises(ValueError, match="真实 COM"):
        validate_ports({"left": dict(demo=False, port="")})
    with pytest.raises(ValueError):
        validate_calibration({}, (3,))


def test_right_real_launcher_imports_user_positions_without_left_real_copy(tmp_path):
    import json
    from tools.aero_score.gui import defaults, right_real_setup
    source = tmp_path / "tools" / "aero_piano_test" / "config.json"
    source.parent.mkdir(parents=True)
    positions = {"3": dict(home="8", press="30"), "4": dict(home="0", press="20"),
                 "5": dict(home="10", press="35"), "6": dict(home="15", press="40")}
    source.write_text(json.dumps(dict(port="COM8", fingers=positions)), encoding="utf-8")
    settings = defaults(tmp_path)
    settings["sides"]["right"]["demo"] = True
    settings["sides"]["left"]["simulation"]["3"] = dict(home="0", press="0")
    actual = right_real_setup(settings, tmp_path)["sides"]
    assert actual["right"]["demo"] is False
    assert actual["right"]["port"] == "COM8"
    assert actual["right"]["real"] == positions
    assert actual["left"]["demo"] is True
    assert all(row == dict(home="", press="") for row in actual["left"]["real"].values())
    validate_calibration(actual["left"]["simulation"], (3, 4, 5, 6))


class Clock:
    def __init__(self):
        self.ns = 10_000_000_000
    def now(self):
        return self.ns
    def wait(self, seconds):
        self.ns += max(1, round(seconds * 1e9))


class Bus:
    demo = True
    fault = False
    def __init__(self, on_write=None):
        self.writes = []
        self.on_write = on_write
    def write(self, raw):
        if self.fault:
            raise OSError("feedback lost")
        self.writes.append(tuple(raw))
        if self.on_write:
            self.on_write(raw)


def fixture_stream():
    base = (10, 20, 30, 0, 0, 0, 0)
    down = (10, 20, 30, 200, 0, 0, 0)
    return Prepared(base, ((Event(50000, (4,), ((3, True),)), down),
                           (Event(150000, (6,), ((3, False),)), base)), base)


def test_absolute_schedule_and_thumb_preservation():
    clock, bus = Clock(), Bus()
    link = SimpleNamespace(side="right", bus=bus, feedback=lambda **kw: None)
    gate = Gate(start_ns=clock.now())
    gate.go.set()
    events = []
    assert execute(link, fixture_stream(), 200000, gate, threading.Event(), threading.Event(),
                   events.append, clock.now, clock.wait) == "completed"
    assert [e["sent_us"] for e in events if e["kind"] == "hand"] == [50000, 150000]
    assert all(raw[:3] == (10, 20, 30) for raw in bus.writes)


def test_late_schedule_is_not_replayed():
    clock, bus = Clock(), Bus()
    link = SimpleNamespace(side="left", bus=bus, feedback=lambda **kw: None)
    gate = Gate(start_ns=clock.now()-160_000_000)
    gate.go.set()
    stop = threading.Event()
    with pytest.raises(TimeoutError):
        execute(link, fixture_stream(), 200000, gate, stop, threading.Event(), lambda e: None,
                clock.now, clock.wait)
    assert stop.is_set()
    assert bus.writes == []


@pytest.mark.parametrize("hard", [False, True])
def test_stop_releases_but_abort_does_not(hard):
    clock, stop, abort = Clock(), threading.Event(), threading.Event()
    def request_stop(raw):
        stop.set()
        if hard:
            abort.set()
    bus = Bus(request_stop)
    gate = Gate(start_ns=clock.now())
    gate.go.set()
    link = SimpleNamespace(side="right", bus=bus, feedback=lambda **kw: None)
    result = execute(link, fixture_stream(), 200000, gate, stop, abort, lambda e: None, clock.now, clock.wait)
    assert result == ("aborted-no-release" if hard else "stopped")
    assert len(bus.writes) == (1 if hard else 2)
    if not hard:
        assert bus.writes[-1] == fixture_stream().release


def test_real_sdk_dual_demo_never_opens_serial(monkeypatch):
    sdk = pytest.importorskip("aero_open_sdk.aero_hand")
    def forbidden(*a, **kw):
        pytest.fail("Demo must not open COM1 or any serial port")
    monkeypatch.setattr(sdk, "Serial", forbidden)
    links = {s: HandLink(s, lambda *args: None) for s in ("left", "right")}
    try:
        for link in links.values():
            link.submit(lambda l: l.connect(True, "COM1")).result(timeout=3)
        score = compile_score(TEXT)
        with pytest.raises(ValueError, match="未处于抬起"):
            links["left"].submit(lambda l: prepare(l, score, CAL, 2)).result(timeout=3)
        def home(link):
            link.feedback()
            raw = list(link.bus.raw)
            for ch in CAL:
                raw[ch] = link.bus.target(ch, 0)
            link.bus.write(raw)
        for link in links.values():
            link.submit(home).result(timeout=3)
        events = []
        result = run_dual(links, score, {s: CAL for s in links}, 2, threading.Event(), threading.Event(), events.append)
        assert result == {s: "completed" for s in links}
        sent = [e for e in events if e["kind"] == "hand"]
        assert len(sent) == 4 and all(e["demo"] for e in sent)
        assert {e["scheduled_us"] for e in sent} == {50000, 150000}
    finally:
        for link in links.values():
            link.shutdown.set()
        for link in links.values():
            link.thread.join(timeout=2)
            assert not link.thread.is_alive()


def test_gui_demo_in_own_process(tmp_path):
    pytest.importorskip("aero_open_sdk.aero_hand")
    import subprocess
    import sys
    code = "import runpy,sys; from pathlib import Path; runpy.run_path(sys.argv[1])['_gui_demo'](Path(sys.argv[2]))"
    result = subprocess.run([sys.executable, "-c", code, __file__, str(tmp_path)],
                            cwd=Path(__file__).resolve().parents[1], capture_output=True,
                            text=True, encoding="utf-8", errors="replace", timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("fail_right", [False, True])
def test_mixed_mode_uses_one_real_port_and_stops_peer(monkeypatch, fail_right):
    """Official SDK talks to a protocol fake; left simulation never opens COM1."""
    import struct
    sdk = pytest.importorskip("aero_open_sdk.aero_hand")
    from serial import SerialTimeoutException
    opened = []
    class Serial:
        def __init__(self, port, *args, **kwargs):
            opened.append(port)
            self.raw = [100, 200, 300, 0, 0, 0, 0]
            self.frames = []
            self.response = b""
            self.closed = False
        def reset_input_buffer(self):
            self.response = b""
        def reset_output_buffer(self):
            pass
        def write(self, data):
            opcode, _, *raw = struct.unpack("<2B7H", data)
            if opcode == sdk.CTRL_POS and fail_right and raw[3] > 100:
                raise SerialTimeoutException("right disconnected during note")
            self.frames.append((opcode, tuple(raw)))
            if opcode == sdk.CTRL_POS:
                self.raw = raw
            else:
                values = self.raw if opcode == sdk.GET_POS else [0]*7
                self.response = struct.pack("<2B7H", opcode, 0, *values)
            return len(data)
        def flush(self):
            pass
        def read(self, count):
            data, self.response = self.response[:count], self.response[count:]
            return data
        def close(self):
            self.closed = True
    monkeypatch.setattr(sdk, "Serial", Serial)
    links = {s: HandLink(s, lambda *args: None) for s in ("left", "right")}
    stop, abort, records = threading.Event(), threading.Event(), []
    try:
        links["left"].submit(lambda link: link.connect(True, "COM1")).result(timeout=3)
        links["right"].submit(lambda link: link.connect(False, "COM8")).result(timeout=3)
        serial = links["right"].bus.hand.ser
        assert opened == ["COM8"]
        def left_home(link):
            link.feedback()
            raw = list(link.bus.raw)
            raw[3:] = [0]*4
            link.bus.write(raw)
        links["left"].submit(left_home).result(timeout=3)
        score = compile_score(TEXT)
        if fail_right:
            with pytest.raises(RuntimeError, match="写入超时"):
                run_dual(links, score, {s: CAL for s in links}, 2, stop, abort, records.append)
            assert stop.is_set()
            assert links["right"].bus.fault
            assert not any(e["kind"] == "hand" and e["scheduled_us"] == 150000 for e in records)
            left_notes = [e for e in records if e["side"] == "left" and e["kind"] == "hand"]
            if left_notes:
                assert any(e["side"] == "left" and e["kind"] == "release" for e in records)
        else:
            result = run_dual(links, score, {s: CAL for s in links}, 2, stop, abort, records.append)
            assert all(value == "completed" for value in result.values())
            assert all(e["demo"] == (e["side"] == "left") for e in records)
            assert serial.raw[:3] == [100, 200, 300]
        assert set(opcode for opcode, _ in serial.frames) <= {sdk.GET_POS, sdk.GET_CURR, sdk.GET_TEMP, sdk.GET_VEL, sdk.CTRL_POS}
    finally:
        abort.set()
        stop.set()
        for link in links.values():
            link.shutdown.set()
        for link in links.values():
            link.thread.join(timeout=2)
            assert not link.thread.is_alive()


def _gui_demo(tmp_path):
    import tkinter as tk
    import time
    import gc
    from tools.aero_score import gui
    import aero_open_sdk.aero_hand as sdk
    def forbidden(*a, **kw):
        raise AssertionError("No serial in GUI demo")
    sdk.Serial = forbidden
    gui.ROOT = tmp_path
    root = tk.Tk()
    root.withdraw()
    app = gui.App(root, config_path=tmp_path/"dual-demo.json", all_demo=True)
    def pump(predicate):
        deadline = time.monotonic()+8
        while not predicate() and time.monotonic()<deadline:
            root.update()
            time.sleep(.01)
        assert predicate(), app.progress.get()
    try:
        assert app.inspect().ignored_moves == 2
        for side in ("left", "right"):
            app.connect(side)
            pump(lambda s=side: app.connected[s] and not app.busy)
            app.home_side(side)
            pump(lambda: not app.busy)
        app.text.delete("1.0", "end")
        app.text.insert("1.0", TEXT)
        app.start()
        pump(lambda: not app.busy)
        assert app.progress.get().startswith("完成")
        reports = list((tmp_path/"logs"/"aero_score").glob("*.json"))
        assert len(reports) == 1
        app.disconnect("left")
        pump(lambda: not app.busy)
        app.demo["left"].set(False)
        app.change_mode("left")
        assert all(r["home"].get() == r["press"].get() == "" for r in app.rows["left"].values())
    finally:
        app.abort.set()
        app.stop.set()
        if app.task_thread:
            app.task_thread.join(timeout=2)
        for link in app.links.values():
            link.shutdown.set()
            link.thread.join(timeout=2)
        for callback in root.tk.splitlist(root.tk.call("after", "info")):
            root.after_cancel(callback)
        root.destroy()
        del app, root
        gc.collect()
