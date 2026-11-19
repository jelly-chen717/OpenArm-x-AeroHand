"""VR/UMI raw session recorder: observes existing data, never commands hardware.

ROS published targets remain candidates. These files are not qualified training
episodes, synchronized sensor measurements, or evidence of accepted CAN targets.
"""
from collections import Counter
import asyncio
import copy
import datetime
import hashlib
import http.client
import json
import os
from pathlib import Path
import queue
import secrets
import shutil
import threading
import time
import uuid

MODES = ('vr', 'umi')
ACTIVE = ('starting', 'recording', 'stopping', 'saving')
CAMERAS = ('d435', 'd405_1', 'd405_2')
from station_network import same_origin
SCHEMA = 'openarm.teleop.raw_capture.v1'


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def atomic(path, data):
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, allow_nan=False, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def camera_request(camera, image=False):
    connection = http.client.HTTPConnection('127.0.0.1', 8768, timeout=.8)
    try:
        path = '/snapshot.jpg' if image else '/status'
        connection.request('GET', path + '?camera=' + camera)
        response = connection.getresponse()
        body = response.read(2 * 1024 * 1024 + 1)
        if response.status != 200 or len(body) > 2 * 1024 * 1024:
            raise ValueError('相机缓存不可用：' + camera)
        if not image:
            return json.loads(body)
        if not body.startswith(b'\xff\xd8'):
            raise ValueError('相机图像格式无效：' + camera)
        seq = int(response.getheader('X-Frame-Seq'))
        age_ms = float(response.getheader('X-Frame-Age-Ms'))
        if not 0 <= age_ms < 1000:
            raise ValueError('相机画面已过期：' + camera)
        return body, seq, age_ms
    finally:
        connection.close()


class CaptureManager:
    def __init__(self, root, source, umi_status=lambda: {}, include_rgb=True,
                 camera=camera_request, clock=time.monotonic, reserve_bytes=1024**3):
        self.root = Path(root)
        self.source = source
        self.umi_status = umi_status
        self.include_rgb = include_rgb
        self.camera = camera
        self.clock = clock
        self.reserve_bytes = reserve_bytes
        self.lock = threading.RLock()
        self.token = secrets.token_hex(24)
        self.session = None
        self.latest = {}
        self.camera_health = {}
        self.umi_health = {}
        self.previous_buttons = {'left': None, 'right': None}
        self.last_xr_at = None
        self.states = {mode: {'mode': mode, 'state': 'idle', 'session_id': None,
                              'elapsed_s': 0, 'error': None} for mode in MODES}
        self.closed = threading.Event()

    def observe(self, topic, data):
        now = self.clock()
        with self.lock:
            self.latest[topic] = (now, data)
            session = self.session
            if (not session or session['stop'].is_set()
                    or self.states[session['mode']]['state'] != 'recording'):
                return
            if topic == 'xr_state' and session['mode'] != 'vr':
                return
            row = {'host_received_monotonic_ns': int(now * 1e9),
                   'host_received_unix_ns': time.time_ns(), 'topic': topic,
                   'data': copy.deepcopy(data)}
            try:
                session['queue'].put_nowait(row)
            except queue.Full:
                session['error'] = '写入队列已满，记录不完整'
                session['stop'].set()

    def observe_xr(self, _pose, data):
        """Observe authorized server XR frames before control-source admission."""
        now = self.clock()
        devices = {d.get('handedness'): d for d in (data or {}).get('devices', [])
                   if d.get('role') == 'controller' and d.get('handedness') in ('left', 'right')}
        edges = {}
        with self.lock:
            if self.last_xr_at is None or now - self.last_xr_at > 1:
                self.previous_buttons = {'left': None, 'right': None}
            self.last_xr_at = now
            for side in ('left', 'right'):
                buttons = devices.get(side, {}).get('gamepad', {}).get('buttons', [])
                if len(buttons) <= 5:
                    self.previous_buttons[side] = None
                    continue
                button = buttons[5]
                pressed = bool(button.get('pressed')) if isinstance(button, dict) else bool(button)
                edges[side] = pressed and self.previous_buttons[side] is False
                self.previous_buttons[side] = pressed
        self.observe('xr_state', data)
        # B always wins over Y in the same input frame; first held buttons need release.
        if edges.get('right'):
            self.request('vr', 'stop', 'right_B')
        elif edges.get('left') and self.previous_buttons['right'] is False:
            self.request('vr', 'start', 'left_Y')

    def observer_failed(self, message):
        with self.lock:
            if self.session:
                self.session['error'] = message
                self.session['stop'].set()

    def reasons(self, mode):
        reasons = []
        if self.source() != mode:
            reasons.append('请先选择 ' + mode.upper() + ' 控制源')
        now = self.clock()
        joint = self.latest.get('/joint_states')
        if not joint or now - joint[0] > 1 or len(joint[1].get('position', [])) < 16:
            reasons.append('等待双臂新鲜关节反馈')
        if mode == 'vr':
            frame = self.latest.get('xr_state')
            if not frame or now - frame[0] > 1:
                reasons.append('等待头显输入')
            else:
                hands = {d.get('handedness') for d in frame[1].get('devices', [])
                         if d.get('role') == 'controller' and d.get('gripPose') is not None}
                if not {'left', 'right'} <= hands:
                    reasons.append('等待左右实体手柄定位')
        elif not self.umi_health.get('transport', {}).get('healthy'):
            reasons.append('等待 UMI 定位数据')
        if self.include_rgb:
            for name in CAMERAS:
                if not self.camera_health.get(name, {}).get('available'):
                    reasons.append('等待相机：' + name)
        return reasons

    def status(self):
        with self.lock:
            result = {}
            for mode in MODES:
                row = copy.deepcopy(self.states[mode])
                if self.session and self.session['mode'] == mode:
                    row['elapsed_s'] = round(max(0, self.clock() - self.session['started']), 1)
                    if self.session.get('stopped') is not None:
                        row['elapsed_s'] = round(max(0, self.session['stopped']-self.session['started']), 1)
                    row['events'] = self.session['events']
                    row['frames'] = dict(self.session['frames'])
                    if self.session['stop'].is_set() and row['state'] == 'recording':
                        row['state'] = 'stopping'
                row['blockers'] = self.reasons(mode)
                row['ready'] = not row['blockers']
                row['include_rgb'] = self.include_rgb
                row['training_eligible'] = False
                result[mode] = row
            return {'schema': SCHEMA, 'csrf_token': self.token, 'modes': result,
                    'active_mode': self.session['mode'] if self.session else None,
                    'storage_root': str(self.root), 'updated_utc': utc()}

    def history(self, mode):
        if mode not in MODES:
            raise ValueError('采集类型无效')
        parent = self.root / mode
        if not parent.is_dir() or parent.is_symlink():
            return []
        result = []
        for folder in sorted(parent.iterdir(), key=lambda p: p.name, reverse=True)[:1000]:
            manifest = folder / 'manifest.json'
            if folder.is_symlink() or manifest.is_symlink() or not manifest.is_file():
                continue
            try:
                if manifest.stat().st_size > 65536:
                    continue
                row = json.loads(manifest.read_text())
                if row.get('schema') != SCHEMA or row.get('mode') != mode or row.get('session_id') != folder.name:
                    continue
                result.append({k: row.get(k) for k in ['mode', 'state', 'session_id', 'elapsed_s',
                    'events', 'frames', 'error', 'finished_utc', 'output', 'training_eligible']})
                if len(result) >= 20:
                    break
            except (ValueError, OSError):
                continue
        return result

    def request(self, mode, action, trigger='web'):
        if mode not in MODES or action not in ('start', 'stop'):
            raise ValueError('采集操作无效')
        with self.lock:
            if action == 'stop':
                if self.session and self.session['mode'] == mode:
                    self.session['reason'] = trigger
                    self.session.setdefault('stopped', self.clock())
                    self.session['stop'].set()
                    self.states[mode]['state'] = 'stopping'
                return 200, self.status()
            if self.session:
                if self.session['mode'] == mode:
                    return 200, self.status()
                raise ValueError('另一种采集正在进行，请先结束并保存')
            blockers = self.reasons(mode)
            if blockers:
                self.states[mode].update(state='error', error='；'.join(blockers))
                return 409, self.status()
            session = {'mode': mode, 'stop': threading.Event(), 'queue': queue.Queue(2048),
                       'started': self.clock(), 'reason': trigger, 'events': 0,
                       'frames': Counter(), 'topics': Counter(), 'error': None, 'bytes': 0}
            self.session = session
            self.states[mode] = {'mode': mode, 'state': 'starting', 'session_id': None,
                                 'elapsed_s': 0, 'error': None}
            worker = threading.Thread(target=self._record, args=(session,), daemon=True)
            session['thread'] = worker
            worker.start()
            return 202, self.status()

    def _images(self, session, camera, path):
        previous = None
        failed_at = None
        folder = path / 'images' / camera
        folder.mkdir(parents=True)
        with (folder / 'frames.jsonl').open('x', encoding='utf-8') as index:
            while not session['stop'].is_set():
                started = self.clock()
                try:
                    body, seq, age_ms = self.camera(camera, True)
                    if session['stop'].is_set():
                        break
                    failed_at = None
                    if previous is None or seq != previous:
                        previous = seq
                        number = session['frames'][camera]
                        target = folder / f'{number:08d}.jpg'
                        target.write_bytes(body)
                        index.write(json.dumps({'file': target.name, 'camera_seq': seq,
                            'server_frame_age_ms': age_ms, 'host_received_monotonic_ns': int(self.clock()*1e9),
                            'sha256': hashlib.sha256(body).hexdigest()}) + '\n')
                        session['frames'][camera] += 1
                        session['bytes'] += len(body)
                except Exception as error:
                    failed_at = self.clock() if failed_at is None else failed_at
                    if self.clock() - failed_at >= 3:
                        session['error'] = '相机持续断流：' + camera + ' (' + type(error).__name__ + ')'
                        session['stop'].set()
                session['stop'].wait(max(0, 1/15 - (self.clock() - started)))
            index.flush()
            os.fsync(index.fileno())

    def _record(self, session):
        mode = session['mode']
        path = None
        images = []
        try:
            if session['stop'].is_set():
                session['cancelled'] = True
                return
            self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
            if self.root.is_symlink() or shutil.disk_usage(self.root).free < self.reserve_bytes:
                raise ValueError('记录目录不可用或剩余空间不足')
            identity = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:12]
            parent = self.root / mode
            parent.mkdir(exist_ok=True, mode=0o700)
            if parent.is_symlink():
                raise ValueError('采集目录不能是符号链接')
            path = parent / identity
            path.mkdir(mode=0o700)
            with self.lock:
                self.states[mode].update(session_id=identity, state='recording', output=str(path))
                initial_state = copy.deepcopy({topic: {'host_received_monotonic_ns': int(row[0]*1e9),
                    'data': row[1]} for topic, row in self.latest.items()
                    if topic == '/joint_states' or (topic == 'xr_state' and mode == 'vr')})
            atomic(path / 'metadata.json', {'schema': SCHEMA, 'mode': mode, 'started_utc': utc(),
                'started_monotonic_ns': int(session['started']*1e9), 'include_rgb': self.include_rgb,
                'start_trigger': session['reason'],
                'initial_observed_state': initial_state,
                'camera_identities': self.camera_health, 'coordinate_system': 'ROS FLU after existing XR conversion',
                'timestamps': 'ROS source stamps and host receive clocks are separate; cameras use cached JPEG receipt',
                'targets': 'published ROS candidates; actual accepted boundary unverified',
                'training_eligible': False, 'synchronized_training_episode': False})
            if self.include_rgb:
                for camera in CAMERAS:
                    thread = threading.Thread(target=self._images, args=(session, camera, path), daemon=True)
                    images.append(thread)
                    thread.start()
            last_check = self.clock()
            with (path / 'events.jsonl').open('x', encoding='utf-8') as output:
                while not session['stop'].is_set() or not session['queue'].empty():
                    try:
                        row = session['queue'].get(timeout=.1)
                        encoded = json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n'
                        output.write(encoded)
                        session['events'] += 1
                        session['topics'][row['topic']] += 1
                        session['bytes'] += len(encoded.encode())
                    except queue.Empty:
                        pass
                    now = self.clock()
                    if now - last_check >= 1:
                        last_check = now
                        output.flush()
                        if self.source() != mode:
                            session['error'] = '控制源已切换，本段记录结束'
                        elif mode == 'vr' and now-self.latest.get('xr_state', (-1e20, {}))[0] > 3:
                            session['error'] = '头显输入中断，本段记录结束'
                        elif now-self.latest.get('/joint_states', (-1e20, {}))[0] > 3:
                            session['error'] = '关节反馈中断，本段记录结束'
                        elif shutil.disk_usage(self.root).free < self.reserve_bytes:
                            session['error'] = '磁盘余量不足，本段记录结束'
                        elif session['bytes'] >= 10 * 1024**3 or now-session['started'] >= 3600:
                            session['error'] = '已达到单段记录上限（1 小时或 10 GB）'
                        if session['error']:
                            session['stop'].set()
                        if mode == 'umi' and not session['stop'].is_set():
                            self.observe('umi_runtime_status', self.umi_health)
                output.flush()
                os.fsync(output.fileno())
            with self.lock:
                self.states[mode]['state'] = 'saving'
            for thread in images:
                thread.join(timeout=3)
            if any(thread.is_alive() for thread in images):
                raise RuntimeError('画面写入尚未结束')
            if session['events'] < 1:
                raise ValueError('未收到记录数据，不能标为已保存')
            if self.include_rgb and any(session['frames'][camera] < 1 for camera in CAMERAS):
                raise ValueError('三路画面记录不完整，原始文件保留')
        except Exception as error:
            session['error'] = str(error)
            session['stop'].set()
            for thread in images:
                thread.join(timeout=3)
        finally:
            result = {'mode': mode, 'state': 'error' if session['error'] else 'saved',
                      'session_id': path.name if path else None, 'output': str(path) if path else None,
                      'elapsed_s': round(session.get('stopped', self.clock())-session['started'], 1), 'events': session['events'],
                      'frames': dict(session['frames']), 'error': session['error'],
                      'topic_counts': dict(session['topics']),
                      'finished_utc': utc(), 'stop_reason': session['reason'],
                      'training_eligible': False, 'schema': SCHEMA}
            if session.get('cancelled'):
                result.update(state='idle', stop_reason='cancelled_before_start')
            if path:
                try:
                    atomic(path / 'manifest.json', result)
                except Exception:
                    result.update(state='error', error='封存失败，原始文件保留在 ' + str(path))
            with self.lock:
                self.states[mode] = result
                if self.session is session:
                    self.session = None

    def monitor(self):
        while not self.closed.is_set():
            try:
                self.umi_health = self.umi_status()
            except Exception:
                self.umi_health = {}
            if self.include_rgb:
                for camera in CAMERAS:
                    try:
                        self.camera_health[camera] = self.camera(camera)
                    except Exception:
                        self.camera_health[camera] = {'available': False}
            self.closed.wait(1)

    def close(self):
        self.closed.set()
        with self.lock:
            session = self.session
            if session:
                session['reason'] = 'service_shutdown'
                session['stop'].set()
        if session:
            session['thread'].join(timeout=8)


def install(app, node, teleop, worker, umi_runtime):
    """Add observers and recording routes to the existing node; no new publishers."""
    from fastapi import Request
    from fastapi.responses import JSONResponse
    from fastapi.routing import APIRoute
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    paths = {'/api/capture/teleop/status', '/api/capture/teleop/action'}
    if paths.intersection(getattr(route, 'path', '') for route in app.routes):
        raise RuntimeError('Teleop capture routes already installed')
    # /logs is the existing persistent writable mount; /test stays read-only.
    manager = CaptureManager(Path('/logs/teleop_capture'), worker.get_input_source, umi_runtime.status)
    qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    topics = {'/joint_states': JointState, '/joint_trajectory': JointTrajectory,
              '/left_forward_position_controller/commands': Float64MultiArray,
              '/right_forward_position_controller/commands': Float64MultiArray,
              '/left_gripper_controller/joint_trajectory': JointTrajectory,
              '/right_gripper_controller/joint_trajectory': JointTrajectory}
    def observe(topic, msg):
        try:
            manager.observe(topic, message_to_ordereddict(msg))
        except Exception:
            # Recording failures must never break a control callback or publish a target.
            manager.observer_failed('数据观察失败，记录不完整')
    def observe_xr(pose, state):
        try:
            manager.observe_xr(pose, state)
        except Exception:
            manager.observer_failed('VR 数据观察失败，记录不完整')
    for topic, msg_type in topics.items():
        node.create_subscription(msg_type, topic, lambda msg, topic=topic: observe(topic, msg), qos)
    teleop.subscribe(observe_xr)
    monitor = threading.Thread(target=manager.monitor, daemon=True)
    monitor.start()

    async def status(request: Request):
        if not same_origin(request, optional=True):
            return JSONResponse({'error': 'same_origin_required'}, status_code=403)
        return JSONResponse(await asyncio.to_thread(manager.status), headers={'Cache-Control': 'no-store'})

    async def action(request: Request):
        if (not same_origin(request) or
            request.headers.get('content-type', '').split(';')[0] != 'application/json' or
            not secrets.compare_digest(request.headers.get('x-openarm-capture-token', ''), manager.token)):
            return JSONResponse({'error': 'same_origin_token_required'}, status_code=403)
        body = bytearray()
        async for chunk in request.stream():
            if len(body)+len(chunk)>128:
                return JSONResponse({'error': 'invalid_body_size'}, status_code=400)
            body.extend(chunk)
        if not body:
            return JSONResponse({'error': 'invalid_body_size'}, status_code=400)
        try:
            data = json.loads(body)
            if not isinstance(data, dict) or set(data) != {'mode', 'action'}:
                raise ValueError('采集参数无效')
            code, result = manager.request(data['mode'], data['action'])
            return JSONResponse(result, status_code=code, headers={'Cache-Control': 'no-store'})
        except (ValueError, TypeError) as error:
            return JSONResponse({'error': str(error)}, status_code=409)

    async def history(request: Request):
        if not same_origin(request, optional=True):
            return JSONResponse({'error': 'same_origin_required'}, status_code=403)
        try:
            rows = await asyncio.to_thread(manager.history, request.path_params['mode'])
            return JSONResponse({'records': rows}, headers={'Cache-Control': 'no-store'})
        except ValueError as error:
            return JSONResponse({'error': str(error)}, status_code=400)

    app.router.routes[0:0] = [APIRoute('/api/capture/teleop/status', status, methods=['GET']),
                             APIRoute('/api/capture/teleop/action', action, methods=['POST']),
                             APIRoute('/api/capture/teleop/{mode}/records', history, methods=['GET'])]
    app.router.add_event_handler('shutdown', manager.close)
    return manager
