"""Regressions for Windows venv launchers leaving HTTP children alive."""
import importlib.util
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest


SPEC = importlib.util.spec_from_file_location(
    "start_simulation", Path(__file__).resolve().parents[1] / "tools/start_simulation.py"
)
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


@pytest.mark.skipif(sys.platform != "win32", reason="Windows venv process tree")
@pytest.mark.parametrize("abrupt_exit", [False, True], ids=["explicit-stop", "launcher-killed"])
def test_stop_releases_descendant_listener(tmp_path, abrupt_exit):
    port_file = tmp_path / "port.txt"
    stop_file = tmp_path / "stop"
    child = (
        "import socket,time; from pathlib import Path; "
        "s=socket.socket(); s.bind(('127.0.0.1',0)); s.listen(); "
        f"Path({str(port_file)!r}).write_text(str(s.getsockname()[1])); "
        f"stop=Path({str(stop_file)!r}); deadline=time.monotonic()+30\n"
        "while time.monotonic() < deadline and not stop.exists():\n"
        "    time.sleep(.05)"
    )
    parent = (
        "import runpy, subprocess\n"
        f"if {abrupt_exit!r}:\n"
        f"    runpy.run_path({SPEC.origin!r})['bind_windows_service_lifetime']()\n"
        f"p=subprocess.Popen([{sys.executable!r},'-c',{child!r}]); p.wait()"
    )
    # Address the actual launcher interpreter so kill() cannot target just its venv stub.
    process = subprocess.Popen([sys._base_executable, "-c", parent])
    try:
        deadline = time.monotonic() + 10
        while not port_file.exists() and time.monotonic() < deadline:
            time.sleep(.05)
        assert port_file.exists(), "Descendant failed to start"
        port = int(port_file.read_text())
        with socket.create_connection(("127.0.0.1", port), timeout=1):
            pass
        if abrupt_exit:
            process.kill()
            process.wait(timeout=5)
        else:
            launcher.stop_process(process)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            with socket.socket() as probe:
                probe.settimeout(.2)
                if probe.connect_ex(("127.0.0.1", port)) != 0:
                    break
            time.sleep(.05)
        else:
            pytest.fail("Descendant still listening after the launcher exited")
    finally:
        stop_file.touch()
        launcher.stop_process(process)
