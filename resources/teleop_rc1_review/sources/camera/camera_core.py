"""Bounded latest-frame RGB preview, loopback only; never talks to robots."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlsplit

SERIAL = "401522071685"
HEADER = struct.Struct("<4sIIQQI")
MAX_JPEG = 512 * 1024
FRESH_NS = 1_000_000_000

def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        part = stream.read(size - len(data))
        if not part:
            raise EOFError("Native camera pipe closed")
        data.extend(part)
    return bytes(data)

class LatestFrames:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw = None
        self.jpeg = None
        self.error = ""
        self.capture_times = deque(maxlen=64)
        self.encode_times = deque(maxlen=32)

    def put_raw(self, seq, timestamp_ns, data):
        if len(data) != 640 * 480 * 3:
            raise ValueError("Unexpected RGB size")
        with self.lock:
            self.raw = (seq, timestamp_ns, data)
            self.capture_times.append(timestamp_ns)

    def get_raw(self):
        with self.lock:
            return self.raw

    def put_jpeg(self, seq, timestamp_ns, data):
        if not (4 <= len(data) <= MAX_JPEG and
                data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9")):
            raise ValueError("Invalid JPEG")
        with self.lock:
            self.jpeg = (seq, timestamp_ns, data)
            self.error = ""
            self.encode_times.append(time.monotonic_ns())

    @staticmethod
    def fps(times):
        return ((len(times)-1)*1e9/(times[-1]-times[0])
                if len(times)>1 and times[-1]>times[0] else 0.0)

    def status(self):
        with self.lock:
            item = self.jpeg
            error = self.error
            capture = tuple(self.capture_times)
            encoded = tuple(self.encode_times)
        age = max(0, (time.monotonic_ns()-item[1])/1e6) if item else None
        available = bool(item and age < FRESH_NS/1e6 and not error)
        return {"available": available, "serial": SERIAL,
                "model": "D435", "seq": item[0] if item else -1,
                "age_ms": round(age, 1) if age is not None else None,
                "capture_fps": round(self.fps(capture), 1),
                "preview_fps": round(self.fps(encoded), 1),
                "width": 640, "height": 480,
                "error": error or ("" if available else "相机画面暂不可用")}

    def snapshot(self, after=-1):
        with self.lock:
            item, error = self.jpeg, self.error
        if not item or error or time.monotonic_ns()-item[1] >= FRESH_NS:
            return None
        return item if item[0] > after else ()

    def fail(self, message):
        with self.lock:
            self.error = message

def handler_for(store):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        def log_message(self, *args):
            pass
        def reply(self, code, body=b"", mime="application/json", extras=None):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in (extras or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            try:
                if body: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
        def do_GET(self):
            self.connection.settimeout(2)
            parsed = urlsplit(self.path)
            if parsed.path == "/status":
                self.reply(200, json.dumps(store.status(), ensure_ascii=False).encode())
                return
            if parsed.path != "/snapshot.jpg":
                self.reply(404, b'{"error":"not found"}')
                return
            try:
                after = int(parse_qs(parsed.query).get("after", ["-1"])[0])
                if not -1 <= after < 2**63: raise ValueError()
            except (ValueError, IndexError):
                self.reply(400, b'{"error":"invalid sequence"}')
                return
            item = store.snapshot(after)
            if item is None:
                self.reply(503, '{"error":"相机画面暂不可用"}'.encode())
            elif not item:
                self.reply(304)
            else:
                seq, timestamp, jpeg = item
                self.reply(200, jpeg, "image/jpeg",
                           {"X-Frame-Seq":seq,
                            "X-Frame-Age-Ms":f"{max(0,(time.monotonic_ns()-timestamp)/1e6):.1f}"})
    return Handler

def main():
    import fcntl
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8768)
    args = parser.parse_args()
    lock = open(Path(__file__).with_name("camera-service.lock"), "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    stop = threading.Event()
    worker_failed = threading.Event()
    store = LatestFrames()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(store))
    server.daemon_threads = True
    proc = subprocess.Popen([str(args.native), SERIAL], stdout=subprocess.PIPE,
                            bufsize=0)
    def receive():
        try:
            while not stop.is_set():
                magic, w, h, seq, timestamp, size = HEADER.unpack(read_exact(proc.stdout, HEADER.size))
                if magic != b"RSV1" or (w,h,size)!=(640,480,640*480*3):
                    raise ValueError("Invalid native RGB framing")
                store.put_raw(seq, timestamp, read_exact(proc.stdout,size))
        except (EOFError, ValueError, OSError) as e:
            if not stop.is_set():
                store.fail("相机连接中断")
                print(str(e), flush=True)
    def encode_frames():
        import cv2
        import numpy as np
        cv2.setNumThreads(1)
        last = -1
        while not stop.is_set():
            start = time.monotonic()
            raw = store.get_raw()
            if raw and raw[0] != last:
                seq, timestamp, data = raw
                rgb = np.frombuffer(data,dtype=np.uint8).reshape(480,640,3)
                ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),
                                       [cv2.IMWRITE_JPEG_QUALITY,80])
                if ok:
                    store.put_jpeg(seq,timestamp,jpeg.tobytes())
                    last = seq
            stop.wait(max(0,1/15-(time.monotonic()-start)))
    def encode():
        try:
            encode_frames()
        except Exception as exc:
            store.fail("预览编码暂不可用")
            print("Preview encoder stopped:",type(exc).__name__,flush=True)
            worker_failed.set()
            stop.set()
    workers = [threading.Thread(target=receive,daemon=True),
               threading.Thread(target=encode,daemon=True),
               threading.Thread(target=server.serve_forever,daemon=True)]
    for worker in workers: worker.start()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.set())
    failed = False
    try:
        while not stop.wait(.2):
            if proc.poll() is not None:
                failed = True
                break
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        if proc.poll() is None:
            proc.terminate()
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait(timeout=3)
        proc.stdout.close()
        for worker in workers: worker.join(timeout=2)
        lock.close()
    return 1 if failed or worker_failed.is_set() else 0

if __name__ == "__main__":
    raise SystemExit(main())
