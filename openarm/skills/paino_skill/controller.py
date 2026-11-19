"""Local 88-key Aero performance, contact audio, and CSV positioning UI."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import re
import secrets
from pathlib import Path
import threading
import webbrowser
from urllib.parse import urlparse, parse_qs

from openarm_app.sim_client import SimClient
from openarm_app.web_controller import ASSETS as COMMON_ASSETS
from openarm_app.trajectory.timing import (
    INTERPOLATION_PERIOD_S, MAX_INTERPOLATION_STEPS,
    interpolation_steps,
)
from paino_skill.points import CSV_PATH, load_table
from paino_skill.scene import PIANO
from paino_skill.performance import keyboard_layout
from paino_skill.score import parse_score


# Ideal simulation settings, not measured or calibrated hardware capabilities.
DEFAULT_STEPS = 120
DEFAULT_VELOCITY = 2.0
DEFAULT_ACCELERATION = 20.0
MUSIC_DIR = PIANO / "music_data"
STANDALONE_UI = PIANO.parents[2] / "piano_standalone.html"


def music_files():
    """Rescan local TXT songs; never expose files outside the music directory."""
    if not MUSIC_DIR.is_dir():
        return []
    root = MUSIC_DIR.resolve()
    return sorted((p for p in MUSIC_DIR.iterdir()
                   if p.is_file() and p.suffix.lower() == ".txt" and p.resolve().parent == root),
                  key=lambda p: p.name.casefold())


def motion_limit(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label}必须是有限正数")
    return float(value)


def key_target(table, key):
    if isinstance(key, bool) or not isinstance(key, int) or not 1 <= key <= 52:
        raise ValueError("白键编号必须是 1–52 的整数")
    regions = table["calibration"]["regions"]
    side = next((s for s, (lo, hi) in regions.items() if lo <= key <= hi), None)
    if side is None:
        raise ValueError("20–23 号白键尚未分配机械臂")
    row = next((r for r in table["rows"] if r["arm"] == side and r["key"] == key), None)
    if row is None or row["status"] != "ready":
        raise ValueError("该键位没有通过 IK 验证的点位，请查看点位表")
    return side, row["q"]


def create_server(port=8089, sim_port=8769, *, score_only=False):
    movement_lock = threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def local(self):
            host = f"127.0.0.1:{self.server.server_port}"
            return self.headers.get("Host") == host and self.headers.get("Origin") in (None, f"http://{host}")

        def reply(self, code, value, mime="application/json; charset=utf-8", *, csp=None):
            raw = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", csp or "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            try:
                self.end_headers()
                self.wfile.write(raw)
            except ConnectionError:
                pass

        def do_GET(self):
            if not self.local():
                return self.reply(403, {"error": "仅允许本机页面访问"})
            assets = {"/": (PIANO / "web/index.html", "text/html"),
                      "/app.js": (PIANO / "web/app.js", "text/javascript"),
                      "/style.css": (PIANO / "web/style.css", "text/css")}
            assets["/audio.js"] = (PIANO / "web/audio.js", "text/javascript")
            for name in ("viewer.js", "lib/three.module.js", "lib/three.core.js", "lib/OrbitControls.js"):
                assets["/"+name] = (COMMON_ASSETS / name, "text/javascript")
            if score_only:
                assets = {
                    "/": (PIANO / "web/score.html", "text/html"),
                    "/score.html": (PIANO / "web/score.html", "text/html"),
                    "/score.js": (PIANO / "web/score.js", "text/javascript"),
                    "/score.css": (PIANO / "web/score.css", "text/css"),
                    "/audio.js": (PIANO / "web/audio.js", "text/javascript"),
                    "/scores/moon.txt": (PIANO / "scores/moon.txt", "text/plain"),
                }
            try:
                parsed = urlparse(self.path)
                if score_only and parsed.path in ("/", "/piano_standalone.html") and STANDALONE_UI.is_file():
                    nonce = secrets.token_urlsafe(24)
                    html = STANDALONE_UI.read_text(encoding="utf-8")
                    # The offline file embeds all WAVs. The local service can
                    # load them on demand without transferring 100+ MB per visit.
                    asset_tag = '<script id="assets" type="application/json">'
                    asset_start = html.index(asset_tag) + len(asset_tag)
                    asset_end = html.index("</script>", asset_start)
                    bundled = json.loads(html[asset_start:asset_end])
                    bundled["samples"] = {}
                    html = html[:asset_start] + json.dumps(bundled, ensure_ascii=False) + html[asset_end:]
                    bridge = (PIANO / "web/standalone-sim.js").read_text(encoding="utf-8")
                    module_end = html.rfind("</script>")
                    if module_end < 0 or '<script type="module">' not in html:
                        raise ValueError("新版前端缺少预期的 module 脚本入口")
                    html = html[:module_end] + "\n" + bridge + "\n" + html[module_end:]
                    html = html.replace('<script type="module">', f'<script type="module" nonce="{nonce}">')
                    policy = (f"default-src 'self' data: blob:; script-src 'self' data: 'nonce-{nonce}'; "
                              "style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'")
                    return self.reply(200, html.encode("utf-8"), "text/html; charset=utf-8", csp=policy)
                if score_only and parsed.path == "/api/music":
                    return self.reply(200, {"songs": [{"id": p.name, "name": p.stem} for p in music_files()]})
                if score_only and parsed.path == "/api/music/text":
                    name = parse_qs(parsed.query).get("id", [""])[0]
                    song = next((p for p in music_files() if p.name == name), None)
                    if song is None:
                        return self.reply(404, {"error": "歌曲不存在，请刷新歌单"})
                    if song.stat().st_size > 512000:
                        raise ValueError("歌曲文本超过 512 KB")
                    return self.reply(200, {"name": song.stem, "text": song.read_text(encoding="utf-8-sig")})
                sample = re.fullmatch(r"/samples/(\d{1,3})\.wav", self.path)
                if sample:
                    midi = int(sample[1])
                    if not 21 <= midi <= 108:
                        return self.reply(404, {"error": "音符超出采样范围"})
                    path = PIANO / "samples" / f"tone ({midi-20}).wav"
                    if not path.is_file():
                        return self.reply(404, {"error": f"缺少采样：{path.name}"})
                    return self.reply(200, path.read_bytes(), "audio/wav")
                if self.path in assets:
                    p, mime = assets[self.path]
                    return self.reply(200, p.read_bytes(), mime+"; charset=utf-8")
                if score_only and urlparse(self.path).path not in ("/api/keyboard", "/api/piano"):
                    return self.reply(404, {"error": "Not found"})
                if self.path == "/api/table":
                    return self.reply(200, load_table())
                if self.path == "/api/keyboard":
                    cfg = json.loads((PIANO / "scene.json").read_text(encoding="utf-8"))
                    return self.reply(200, {"keys": keyboard_layout(cfg), "size_m": cfg["size_m"]})
                if urlparse(self.path).path == "/api/piano":
                    after = int(parse_qs(urlparse(self.path).query).get("after", ["0"])[0])
                    if after < 0:
                        raise ValueError("事件编号无效")
                    return self.reply(200, SimClient(sim_port, timeout=3).piano_state(after))
                if self.path == "/api/timing":
                    return self.reply(200, {"period_s": INTERPOLATION_PERIOD_S,
                                            "default_steps": DEFAULT_STEPS,
                                            "default_velocity": DEFAULT_VELOCITY,
                                            "default_acceleration": DEFAULT_ACCELERATION,
                                            "max_steps": MAX_INTERPOLATION_STEPS})
                if self.path == "/points.csv":
                    load_table()
                    return self.reply(200, CSV_PATH.read_bytes(), "text/csv; charset=utf-8")
                if self.path == "/api/model":
                    return self.reply(200, SimClient(sim_port).visual_model())
                if self.path == "/api/state":
                    return self.reply(200, SimClient(sim_port, timeout=3).state())
                self.reply(404, {"error": "Not found"})
            except (OSError, ValueError, RuntimeError, KeyError) as exc:
                self.reply(503, {"error": str(exc)})

        def do_POST(self):
            if not self.local() or self.headers.get_content_type() != "application/json":
                return self.reply(403, {"error": "仅允许本机 JSON 请求"})
            if self.path not in (("/api/move", "/api/score") if score_only else ("/api/move",)):
                return self.reply(404, {"error": "Not found"})
            locked = False
            try:
                self.connection.settimeout(3)
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= (2_000_000 if self.path == "/api/score" else 4096):
                    raise ValueError("请求大小无效")
                request = json.loads(self.rfile.read(size))
                if not isinstance(request, dict):
                    raise ValueError("请求格式无效")
                action = request.get("action")
                if score_only and self.path == "/api/move" and action != "stop":
                    raise ValueError("文本演奏服务仅支持演奏与停止；调试动作请使用调试服务")
                if self.path == "/api/score":
                    if action == "parse":
                        return self.reply(200, parse_score(request.get("text"), finger_count=request.get("finger_count")).summary())
                    client = SimClient(sim_port, timeout=120)
                    if action == "prepare":
                        return self.reply(200, client.command("piano_score_prepare", text=request.get("text"),
                            finger_count=request.get("finger_count"), velocity_limit=request.get("velocity_limit", 2.),
                            acceleration_limit=request.get("acceleration_limit", 20.),
                            automatic_motion=request.get("automatic_motion", score_only)))
                    if action == "start":
                        options = {"token": request.get("token")}
                        if "rate" in request:
                            options["rate"] = request["rate"]
                        return self.reply(200, client.command("piano_score_start", **options))
                    if action in ("pause", "resume"):
                        return self.reply(200, client.command("piano_score_" + action))
                    if action == "rate":
                        return self.reply(200, client.command("piano_score_rate", rate=request.get("rate")))
                    raise ValueError("未知文本演奏操作")
                if action not in ("key", "center", "stop", "pause", "resume", "play", "finger", "release"):
                    raise ValueError("未知动作")
                client = SimClient(sim_port)
                state = client.state()
                if state.get("api_version") != 2 or state.get("scene") != "piano" or state.get("mode") != "ideal":
                    raise ValueError("请连接新版钢琴理想仿真场景")
                if action in ("stop", "pause", "resume"):
                    return self.reply(200, client.command(action))
                if action == "finger":
                    return self.reply(200, client.command("piano_finger", arm=request.get("arm"),
                                                        finger=request.get("finger"), amount=request.get("amount")))
                if action == "release":
                    return self.reply(200, client.command("piano_release"))
                locked = movement_lock.acquire(blocking=False)
                if not locked or state["state"] in ("EXECUTING", "PAUSED") or (state.get("piano") or {}).get("busy"):
                    raise ValueError("动作执行中，请完成或停止后再点击；不会排队")
                steps = interpolation_steps(request.get("interpolation_steps", DEFAULT_STEPS))
                velocity = motion_limit(request.get("velocity_limit", DEFAULT_VELOCITY), "速度上限")
                acceleration = motion_limit(request.get("acceleration_limit", DEFAULT_ACCELERATION), "加速度上限")
                if action == "play":
                    return self.reply(200, client.command("piano_play", arm=request.get("arm", "auto"),
                                      midi=request.get("midi"), finger=request.get("finger", "index"),
                                      interpolation_steps=steps, velocity_limits=[velocity]*7,
                                      acceleration_limits=[acceleration]*7))
                table = load_table()
                if action == "key":
                    side, q = key_target(table, request.get("key"))
                else:
                    side, q = "both", table["calibration"]["center_q_rad"]
                count = 14 if side == "both" else 7
                result = client.command("move_joints", arm=side, q=q, interpolation_steps=steps,
                                        velocity_limits=[velocity]*count,
                                        acceleration_limits=[acceleration]*count)
                self.reply(200, result)
            except (OSError, ValueError, TypeError, RuntimeError, KeyError) as exc:
                self.reply(400, {"error": str(exc)})
            finally:
                if locked:
                    movement_lock.release()
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description="OpenArm piano debug controller")
    parser.add_argument("--port", type=int, default=8089)
    parser.add_argument("--sim-port", type=int, default=8769)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    server = create_server(args.port, args.sim_port)
    url = f"http://127.0.0.1:{server.server_port}"
    print(url, flush=True)
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
