"""Three serial-bound latest-frame previews. No robot or ROS interfaces."""
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import parse_qs,urlsplit
from camera_core import LatestFrames,read_exact,HEADER,FRESH_NS

IDS=("d435","d405_1","d405_2")
EXPECTED_SERIALS=("401522071685","409122273067","409122272257")

# Preview-only recovery: at most three retries per camera in ten minutes.
# A camera that has never supplied a valid frame still requires USB hotplug.
RECOVERY_WINDOW_S=600.0
RECOVERY_DELAYS_S=(5.0,15.0,60.0)

def reconnect_reason(received_frame,failed_since,attempts,now,usb_changed,usb_present):
    """Pure decision helper; never opens a device or resets a USB port."""
    attempts[:]=[stamp for stamp in attempts if 0<=now-stamp<RECOVERY_WINDOW_S]
    if usb_changed and usb_present:
        return "usb-hotplug"
    if not received_frame or failed_since is None or len(attempts)>=len(RECOVERY_DELAYS_S):
        return None
    if now-failed_since>=RECOVERY_DELAYS_S[len(attempts)]:
        return "capture-retry"
    return None

def load_config(path):
    specs=json.loads(Path(path).read_text(encoding="utf-8"))
    if tuple(specs)!=IDS:raise ValueError("Expected the three verified camera IDs")
    if tuple(specs[i]["serial"] for i in IDS)!=EXPECTED_SERIALS:
        raise ValueError("Unexpected serial mapping: re-inspect SDK before changing")
    if [specs[i]["model"] for i in IDS]!=["D435","D405","D405"]:
        raise ValueError("Unexpected camera model")
    return specs

class CameraFrames(LatestFrames):
    def __init__(self,camera,spec):
        super().__init__()
        self.camera,self.spec=camera,spec
    def status(self):
        info=super().status()
        info.update(camera=self.camera,serial=self.spec["serial"],model=self.spec["model"])
        return info
    def reset(self):
        with self.lock:
            self.raw=None
            self.jpeg=None
            self.capture_times.clear()
            self.encode_times.clear()
            self.error="相机连接中"
    def headers(self,item=None):
        headers={"X-Camera-Id":self.camera,"X-Camera-Serial":self.spec["serial"],
                 "X-Frame-Width":"640","X-Frame-Height":"480"}
        if item:
            headers.update({"X-Frame-Seq":str(item[0]),
                            "X-Frame-Age-Ms":f"{max(0,(time.monotonic_ns()-item[1])/1e6):.1f}"})
        return headers

def handler_for(stores):
    class Handler(BaseHTTPRequestHandler):
        protocol_version="HTTP/1.1"
        def setup(self):
            super().setup()
            import socket
            self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.connection.settimeout(2)
        def log_message(self,*_):pass
        def reply(self,code,body=b"",mime="application/json",headers=None):
            self.send_response(code)
            self.send_header("Content-Type",mime)
            self.send_header("Content-Length",str(len(body)))
            self.send_header("Cache-Control","no-store")
            for key,value in (headers or {}).items():self.send_header(key,str(value))
            self.end_headers()
            try:
                if body:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            parsed=urlsplit(self.path)
            queries=parse_qs(parsed.query,keep_blank_values=True)
            if parsed.path not in ["/status","/snapshot.jpg","/cameras"]:
                self.reply(404,b'{"error":"not found"}');return
            if parsed.path=="/cameras":
                if queries:self.reply(400,b'{"error":"invalid query"}');return
                self.reply(200,json.dumps({"cameras":[stores[i].status() for i in IDS]},ensure_ascii=False).encode())
                return
            allowed={"camera"} if parsed.path=="/status" else {"camera","after"}
            try:
                if set(queries)-allowed or any(len(v)!=1 for v in queries.values()):raise ValueError()
                camera=queries.get("camera",["d435"])[0]
                if camera not in stores:raise ValueError()
                after=int(queries.get("after",["-1"])[0])
                if not -1<=after<2**63:raise ValueError()
            except ValueError:
                self.reply(400,b'{"error":"invalid camera or sequence"}');return
            store=stores[camera]
            if parsed.path=="/status":
                self.reply(200,json.dumps(store.status(),ensure_ascii=False).encode());return
            with store.lock:
                item,error=store.jpeg,store.error
            if not item or error or time.monotonic_ns()-item[1]>=FRESH_NS:
                self.reply(503,'{"error":"相机暂不可用"}'.encode(),headers=store.headers());return
            if item[0]<=after:
                self.reply(304,headers=store.headers(item));return
            self.reply(200,item[2],"image/jpeg",store.headers(item))
    return Handler

class CameraRuntime:
    def __init__(self,store,native,stop):
        self.store,self.stop=store,stop
        self.store.reset()
        self.local_stop=threading.Event()
        self.received_frame=False
        self.failed_since=None
        self.proc=subprocess.Popen([str(native),store.spec["serial"]],stdout=subprocess.PIPE,bufsize=0)
        self.workers=[threading.Thread(target=self.receive,daemon=True),
                      threading.Thread(target=self.encode,daemon=True)]
        for worker in self.workers:worker.start()
    def receive(self):
        try:
            while not self.stop.is_set() and not self.local_stop.is_set():
                magic,w,h,seq,timestamp,size=HEADER.unpack(read_exact(self.proc.stdout,HEADER.size))
                if (magic,w,h,size)!=(b"RSV1",640,480,640*480*3):raise ValueError("Unexpected RGB framing")
                self.store.put_raw(seq,timestamp,read_exact(self.proc.stdout,size))
                self.received_frame=True
        except (EOFError,ValueError,OSError) as error:
            if not self.stop.is_set():
                self.store.fail("相机连接中断")
                print(self.store.camera,type(error).__name__,flush=True)
                self.local_stop.set()
    def encode(self):
        import cv2,numpy as np
        last=-1
        try:
            while not self.stop.is_set() and not self.local_stop.is_set():
                begin=time.monotonic()
                raw=self.store.get_raw()
                if raw and raw[0]!=last:
                    seq,timestamp,data=raw
                    rgb=np.frombuffer(data,dtype=np.uint8).reshape(480,640,3)
                    ok,jpeg=cv2.imencode(".jpg",cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),
                                         [cv2.IMWRITE_JPEG_QUALITY,75])
                    if ok:self.store.put_jpeg(seq,timestamp,jpeg.tobytes());last=seq
                self.stop.wait(max(0,1/15-(time.monotonic()-begin)))
        except Exception as error:
            self.store.fail("画面暂不可用")
            self.local_stop.set()
            print(self.store.camera,"encoder",type(error).__name__,flush=True)
    def close(self):
        self.local_stop.set()
        if self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=3)
        self.proc.stdout.close()
        for worker in self.workers:worker.join(timeout=2)

def usb_camera_fingerprint(root=Path("/sys/bus/usb/devices")):
    """Read only these camera USB identities; restart only after hotplug evidence."""
    result=[]
    for device in root.iterdir():
        try:
            vendor=(device/"idVendor").read_text().strip()
            product=(device/"idProduct").read_text().strip()
            if vendor!="8086" or product not in ["0b07","0b5b"]:continue
            result.append((device.name,product,(device/"devnum").read_text().strip()))
        except OSError:continue
    return tuple(sorted(result))

def main():
    import fcntl
    import cv2
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser()
    parser.add_argument("--native",type=Path,required=True)
    parser.add_argument("--config",type=Path,default=Path(__file__).with_name("cameras.json"))
    parser.add_argument("--port",type=int,default=8768)
    parser.add_argument("--only",choices=["all","d405"],default="all")
    args=parser.parse_args()
    specs=load_config(args.config)
    lock=open(Path(__file__).with_name(f"camera-service-{args.port}.lock"),"a")
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stop=threading.Event()
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:stop.set())
    stores={i:CameraFrames(i,specs[i]) for i in IDS}
    runtimes={}
    retry_history={camera:[] for camera in IDS}
    server=ThreadingHTTPServer(("127.0.0.1",args.port),handler_for(stores))
    server.daemon_threads=True
    http_thread=threading.Thread(target=server.serve_forever,daemon=True)
    try:
        fingerprint=usb_camera_fingerprint()
        for camera in IDS:
            if args.only=="d405" and camera=="d435":continue
            runtimes[camera]=CameraRuntime(stores[camera],args.native,stop)
        http_thread.start()
        print("SERIAL_BOUND_CAMERA_PREVIEW_READY",args.port,flush=True)
        while not stop.wait(1):
            current=usb_camera_fingerprint()
            changed=current!=fingerprint
            fingerprint=current
            for camera,runtime in list(runtimes.items()):
                if runtime.proc.poll() is not None or runtime.local_stop.is_set():
                    runtime.store.fail("相机连接中断")
                    now=time.monotonic()
                    if runtime.failed_since is None:
                        runtime.failed_since=now
                    reason=reconnect_reason(runtime.received_frame,runtime.failed_since,
                                            retry_history[camera],now,changed,bool(current))
                    if reason is not None:
                        # Preserve the existing hotplug recovery; soft recovery
                        # is bounded and cannot retry a never-seen/missing camera.
                        if reason=="usb-hotplug":retry_history[camera].clear()
                        else:retry_history[camera].append(now)
                        runtime.close()
                        if not stop.is_set():
                            print("CAMERA_PREVIEW_RECONNECT",camera,reason,
                                  "attempts",len(retry_history[camera]),flush=True)
                            runtimes[camera]=CameraRuntime(stores[camera],args.native,stop)
    finally:
        stop.set()
        if http_thread.is_alive():server.shutdown()
        server.server_close()
        for runtime in runtimes.values():runtime.close()
        if http_thread.ident is not None:http_thread.join(timeout=2)
        lock.close()
    return 0

if __name__=="__main__":raise SystemExit(main())
