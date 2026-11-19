"""Password-free local HTML controller, forwarding only to loopback simulation."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import webbrowser
from openarm_app.sim_client import SimClient

ASSETS = Path(__file__).with_name("web")


def create_web_server(port=8088, sim_port=8769):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def local_request(self):
            host = f"127.0.0.1:{self.server.server_port}"
            return (self.headers.get("Host") == host and
                    self.headers.get("Origin") in (None, f"http://{host}"))

        def reply(self, status, payload, mime="application/json; charset=utf-8"):
            body = payload if isinstance(payload, bytes) else json.dumps(payload, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self.local_request():
                return self.reply(403, {"error": "Only the local controller origin is accepted"})
            assets = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "text/javascript"),
                      "/style.css": ("style.css", "text/css")}
            for filename in ("viewer.js", "pose_math.js", "lib/three.module.js", "lib/three.core.js", "lib/OrbitControls.js"):
                assets["/"+filename] = (filename, "text/javascript")
            if self.path == "/api/model":
                try:
                    return self.reply(200, SimClient(sim_port).visual_model())
                except (RuntimeError, ValueError, OSError) as exc:
                    return self.reply(503, {"error": str(exc)})
            if self.path in assets:
                file, mime = assets[self.path]
                return self.reply(200, (ASSETS/file).read_bytes(), mime+"; charset=utf-8")
            if self.path != "/api/state":
                return self.reply(404, {"error": "Not found"})
            try:
                state = SimClient(sim_port, timeout=3).state()
                if state.get("api_version") != 2:
                    raise RuntimeError("请重启新版双臂仿真服务")
                self.reply(200, state)
            except (RuntimeError, ValueError, OSError) as exc:
                self.reply(503, {"error": str(exc)})

        def do_POST(self):
            if not self.local_request() or self.headers.get_content_type() != "application/json":
                return self.reply(403, {"error": "Use the local JSON controller"})
            if self.path != "/api/command":
                return self.reply(404, {"error": "Not found"})
            try:
                self.connection.settimeout(3)
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 16384:
                    raise ValueError("Invalid request size")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("Command must be an object")
                action = payload.pop("action", None)
                if action not in ("move_joints", "move_pose", "preview_pose", "home", "pause", "resume", "stop"):
                    raise ValueError("Unknown action")
                client = SimClient(sim_port)
                if client.state().get("api_version") != 2:
                    raise ValueError("请重启新版双臂仿真服务")
                self.reply(200, client.command(action, **payload))
            except (RuntimeError, ValueError, TypeError, OSError) as exc:
                self.reply(400, {"error": str(exc)})

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description="OpenArm local HTML controller")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--sim-port", type=int, default=8769)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    try:
        server = create_web_server(args.port, args.sim_port)
    except OSError as exc:
        parser.exit(2, f"Cannot start web controller: {exc}\n")
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"OpenArm HTML controller: {url} | simulation: 127.0.0.1:{args.sim_port}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
