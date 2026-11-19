"""Exercise arm selection and asynchronous preview routing in a withdrawn Tk window."""
import tkinter as tk
import gc
import sys
import pytest
from openarm_app.controller_ui import Controller


@pytest.fixture(scope="module")
def tk_root():
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        if sys.platform == "win32":
            pytest.fail(f"Tk initialization failed on Windows: {exc}")
        pytest.skip(f"Tk display unavailable: {exc}")
    root.withdraw()
    yield root
    root.destroy()
    # Tcl objects must be finalized on this thread, before later HTTP workers
    # can trigger cyclic GC and block waiting for a destroyed Tk event loop.
    del root
    gc.collect()


@pytest.fixture
def ui(tk_root):
    # One Tcl interpreter, like the application; each test gets fresh widgets.
    root = tk.Toplevel(tk_root)
    root.withdraw()
    controller = Controller(root, 8769)
    controller.submit = lambda *args: None  # No background requests in GUI checks.
    yield controller
    # Cancel scheduled polling before destroying the interpreter's widgets.
    for callback in root.tk.splitlist(root.tk.call("after", "info")):
        root.after_cancel(callback)
    controller.close()


def snapshot():
    return {"api_version": 2, "state": "IDLE", "progress": 0,
            "arms": {s: {"q": [float(i)]*7, "pose": [i,0,0,1,0,0,0]}
                     for i, s in enumerate(("right", "left"))}}


def test_selection_and_command_payloads(ui):
    ui.results.put(("poll", snapshot(), None))
    ui.pump()
    assert ui.read(ui.joints["left"]) == [1]*7
    assert all(e.instate(["disabled"]) for e in ui.entries["left"])
    sent = []
    ui.client.command = lambda action, **values: sent.append((action, values))
    ui.submit = lambda label, function: function()
    ui.arm_selection.set("both")
    ui.update_selection()
    assert all(e.instate(["!disabled"]) for e in ui.entries["left"])
    ui.send("move_joints", q=ui.read_selected(ui.joints))
    assert sent[-1] == ("move_joints", {"arm":"both", "q":{"right":[0]*7, "left":[1]*7}})
    ui.arm_selection.set("left")
    ui.send("home")
    assert sent[-1] == ("home", {"arm":"left"})
    ui.send("stop")
    assert sent[-1] == ("stop", {})


def test_preview_keeps_response_arm_when_user_changes_selection(ui):
    ui.results.put(("poll", snapshot(), None))
    ui.pump()
    ui.arm_selection.set("right")
    ui.results.put(("preview_pose", {"arm":"left", "q_goal":[.2]*7}, None))
    ui.pump()
    assert ui.read(ui.joints["left"]) == [.2]*7
    assert ui.read(ui.joints["right"]) == [0]*7
    ui.results.put(("preview_pose", {"arm":"both", "q_goal":{"right":[.3]*7,"left":[.4]*7}}, None))
    ui.pump()
    assert ui.read(ui.joints["right"]) == [.3]*7
    assert ui.read(ui.joints["left"]) == [.4]*7


def test_old_server_cannot_receive_left_command(ui):
    ui.results.put(("poll", {"state":"IDLE"}, None))
    ui.pump()
    assert ui.snapshot is None
    assert "版本过旧" in ui.status.get()
    sent = []
    ui.submit = lambda *args: sent.append(args)
    ui.arm_selection.set("left")
    ui.send("home")
    assert sent == []


def test_both_arm_fields_fit_window(ui):
    ui.root.update_idletasks()
    assert ui.root.winfo_reqwidth() <= 940
    assert ui.root.winfo_reqheight() <= 760
