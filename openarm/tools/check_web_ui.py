"""Launch isolated services and run tools/check_web.cjs (requires Playwright)."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler


def port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main():
    root = Path(__file__).resolve().parents[1]
    ports = [port(), port()]
    paths = [str(root/"src"), str(root/"skills"), str(root/".venv/Lib/site-packages")]
    children = []
    (root/"logs").mkdir(exist_ok=True)
    with (root/"logs/web-check.log").open("w") as output:
        try:
            for module, args in (("sim_server", ["--headless", "--port", str(ports[0]), "--scene", os.environ.get("OPENARM_TEST_SCENE", "default")]),
                                  ("web_controller", ["--no-browser", "--port", str(ports[1]), "--sim-port", str(ports[0])])):
                code = f"import sys,runpy;sys.path[:0]={paths!r};runpy.run_module('openarm_app.{module}',run_name='__main__')"
                children.append(subprocess.Popen([sys._base_executable, "-c", code, *args], cwd=root, stdout=output, stderr=subprocess.STDOUT))
            url = f"http://127.0.0.1:{ports[1]}"
            deadline = time.monotonic()+45
            while True:
                try:
                    with build_opener(ProxyHandler({})).open(url+"/api/state", timeout=4):
                        break
                except OSError:
                    if time.monotonic()>deadline:
                        raise RuntimeError("Services failed to start; see logs/web-check.log")
                    time.sleep(.2)
            subprocess.run(["node", "tools/check_web.cjs"], cwd=root,
                           env={**os.environ, "OPENARM_WEB_URL":url}, check=True, timeout=100)
        finally:
            for child in children:
                child.terminate()
                child.wait(timeout=10)


if __name__ == "__main__":
    main()
