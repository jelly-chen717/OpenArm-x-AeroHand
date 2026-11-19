"""Run a controller and a separate headless MuJoCo server process end-to-end."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from openarm_app.sim_client import SimClient


def main():
    root = Path(__file__).resolve().parents[1]
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    # Run the base interpreter directly so cleanup targets the real child, not
    # the Windows venv redirector. Use exactly this project's installed packages.
    paths = [str(root/"src"), str(root/".venv/Lib/site-packages")]
    bootstrap = f"import sys,runpy; sys.path[:0]={paths!r}; runpy.run_module('openarm_app.sim_server',run_name='__main__')"
    log = root/"logs/remote-process-check.log"
    log.parent.mkdir(exist_ok=True)
    with log.open("w", encoding="utf-8") as output:
        process = subprocess.Popen([sys._base_executable, "-c", bootstrap, "--headless", "--port", str(port)],
                                   cwd=root, stdout=output, stderr=subprocess.STDOUT)
        client = SimClient(port, timeout=3)
        try:
            deadline = time.monotonic()+30
            while True:
                try:
                    initial = client.state()
                    break
                except RuntimeError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("Server startup failed; see "+str(log))
                    time.sleep(.1)
            q = initial["q"].copy()
            q[0] += .1
            sent = client.command("move_joints", q=q)
            assert sent["state"] == "EXECUTING"
            assert client.command("pause")["state"] == "PAUSED"
            held = client.state()["q"]
            time.sleep(.15)
            assert client.state()["q"] == held
            client.command("resume")

            def completed():
                limit = time.monotonic()+10
                while time.monotonic() < limit:
                    result = client.state()
                    if result["state"] == "COMPLETED":
                        return result
                    time.sleep(.05)
                raise AssertionError("Motion did not complete")

            assert completed()["q"] == q
            pose = [.232704,-.150961,-.177241,.653241,.033270,-.752708,.074834]
            before = client.state()["q"]
            preview = client.command("preview_pose", pose=pose)
            assert client.state()["q"] == before
            client.command("move_pose", pose=pose)
            endpoint = completed()
            assert max(abs(a-b) for a,b in zip(endpoint["q"], preview["q_goal"])) < 1e-6
            client.command("home")
            assert completed()["q"] == initial["home"]
            client.command("move_joints", q=q)
            stopped = client.command("stop")
            time.sleep(.15)
            assert client.state()["q"] == stopped["q"]
            # Left-only commands must hold the right arm at its current state.
            right_held = client.state()["q"]
            left = initial["arms"]["left"]["home"].copy()
            left[0] -= .1
            client.command("move_joints", arm="left", q=left)
            result = completed()
            assert result["arms"]["left"]["q"] == left
            assert result["q"] == right_held
            goals = {"right": q, "left": left.copy()}
            goals["left"][1] += .1
            sent = client.command("move_joints", arm="both", q=goals)
            assert sent["active_arms"] == ["right", "left"]
            client.command("pause")
            paused = client.state()["q14"]
            time.sleep(.15)
            assert client.state()["q14"] == paused
            client.command("resume")
            result = completed()
            assert all(result["arms"][s]["q"] == goals[s] for s in goals)
            targets = {s: result["arms"][s]["pose"] for s in goals}
            client.command("home", arm="both")
            result = completed()
            assert all(result["arms"][s]["q"] == initial["arms"][s]["home"] for s in goals)
            preview = client.command("preview_pose", arm="both", pose=targets)
            assert client.state()["q14"] == result["q14"]
            client.command("move_pose", arm="both", pose=targets)
            result = completed()
            for side in goals:
                assert max(abs(a-b) for a,b in zip(result["arms"][side]["q"], preview["q_goal"][side])) < 1e-6
            # Exercise the user-facing CLI with the left-arm selection too.
            cli = subprocess.run([sys.executable, "-m", "openarm_app.sim_client", "--port", str(port),
                                  "--arm", "left", "home"], cwd=root, capture_output=True, text=True, timeout=10)
            assert cli.returncode == 0, cli.stderr
            assert json.loads(cli.stdout)["active_arms"] == ["left"]
            completed()
            assert process.poll() is None
            print(json.dumps({"result":"PASSED", "controller_pid":os.getpid(), "server_pid":process.pid,
                              "checks":["legacy right commands", "left move holding right", "dual joints",
                                        "dual pause/resume", "dual IK preview", "dual pose move",
                                        "dual home", "left CLI", "stop", "server persists"]}, indent=2))
        finally:
            process.terminate()
            process.wait(timeout=10)


if __name__ == "__main__":
    main()
