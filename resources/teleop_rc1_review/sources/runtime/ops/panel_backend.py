# 项目中文注释（2026-10-29）：网页操作的内部服务与队列。
# 调用与职责：127.0.0.1:8781 接收鉴权请求；通过单工作线程提交原有操作并保存回执。
# 运行边界：显式修改请求可能触发电机生命周期；状态轮询不触发。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Authenticated loopback API and one-worker queue; executor owns all control.

No ROS/Docker/CAN imports or actions occur here.  An OperationExecutor must
revalidate physical prerequisites and own the shared host transaction lock.
"""

import argparse
import copy
from datetime import datetime, timezone
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import signal
import threading
import uuid


ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
REQUEST_ID = re.compile(r'[A-Za-z0-9._:-]{1,96}\Z')
MAX_BODY = 512


# 注释：生成 UTC 时间字符串，用于回执与日志关联。
def utc():
    return datetime.now(timezone.utc).isoformat()


# 注释：解析 JSON 对象并拒绝重复字段，避免请求含义不一致。
def unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('Duplicate JSON field')
        value[key] = item
    return value


# 注释：本模块的 RequestError 定义；用途见文件说明与下方原实现。
class RequestError(Exception):
    # 注释：初始化 RequestError 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


# 注释：原实现说明：At most one active operation and one coalesced priority-disable.
class JobQueue:
    """At most one active operation and one coalesced priority-disable.

    The journal records receipts; it is never interpreted as physical state.
    Worker restart marks unfinished records interrupted, without replaying them.
    """
    # 注释：初始化 JobQueue 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self, executor, root, history_limit=None):
        self.executor = executor
        self.path = Path(root) / 'logs/panel_jobs.json'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.history_limit = history_limit
        self.condition = threading.Condition(threading.RLock())
        self.jobs = []
        self.request_ids = {}
        self.active = None
        self.pending_disable = None
        self.stopping = False
        self.journal_error = None
        if self.path.exists():
            saved = json.loads(self.path.read_text(encoding='utf-8'))
            if saved.get('schema_version') != 1 or not isinstance(saved.get('jobs'), list):
                raise RuntimeError('Invalid panel job journal; no operation was started')
            rows = saved['jobs'] if history_limit is None else saved['jobs'][-history_limit:]
            for row in rows:
                job = copy.deepcopy(row)
                if job.get('phase') in ('queued', 'running', 'cancelling'):
                    job.update(phase='interrupted', finished_utc=utc(),
                               error='Backend restarted; operation was not replayed')
                self.jobs.append(job)
                for identity in job.get('request_ids', []):
                    self.request_ids[identity] = job
        self._persist()
        self.worker = threading.Thread(target=self._work, name='openarm-panel-worker', daemon=True)
        self.worker.start()

    # 注释：生成可返回网页的任务摘要，避免暴露内部执行对象。
    def _public(self, job):
        if job is None:
            return None
        return copy.deepcopy({key: value for key, value in job.items() if key != '_cancel'})

    # 注释：保存任务状态及请求标识，支持重复请求核对。
    def _persist(self):
        """Called under the queue lock after startup; never includes credentials."""
        data = {'schema_version': 1, 'updated_utc': utc(),
                'jobs': [self._public(job) for job in self.jobs]}
        raw = (json.dumps(data, allow_nan=False, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
        temporary = self.path.with_name(self.path.name + '.tmp-' + str(os.getpid()))
        try:
            descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(descriptor, 'wb') as stream:
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(self.path)
        except OSError:
            self.journal_error = 'Panel job journal could not be persisted'
            raise

    # 注释：读取现有操作执行器的状态，不启动新硬件操作。
    def _telemetry(self):
        try:
            value = self.executor.status()
            if not isinstance(value, dict):
                raise ValueError('Invalid executor state')
            json.dumps(value, allow_nan=False)
            return copy.deepcopy(value)
        except Exception:
            return {'can_initialize': False, 'can_enable': False, 'can_disable': False,
                    'busy_external': False, 'initialized': None, 'bridge_enabled': False,
                    'telemetry_error': 'Executor status unavailable'}

    # 注释：返回任务状态与原执行器的只读硬件摘要，供 4446 电机面板轮询。
    def state(self):
        physical = self._telemetry()
        with self.condition:
            current = self.active
            phase = ('cancelling' if self.pending_disable and current else
                     current['phase'] if current else
                     'external_operation' if physical.get('busy_external') else 'idle')
            last = self.jobs[-1] if self.jobs else None
            busy = bool(current or self.pending_disable or physical.get('busy_external') or self.stopping)
            return {**physical, 'busy': busy, 'phase': phase,
                    'current_operation': self._public(current),
                    'pending_disable': self._public(self.pending_disable),
                    'last_operation': self._public(last),
                    'error': self.journal_error or (last.get('error') if last else None)
                             or physical.get('error', '')}

    # 注释：为固定动作创建带唯一请求标识的队列任务。
    def _new_job(self, action, identity):
        job = {'id': uuid.uuid4().hex, 'action': action, 'request_ids': [identity],
               'phase': 'queued', 'submitted_utc': utc(), 'started_utc': None,
               'finished_utc': None, 'error': None, 'receipt': None,
               '_cancel': threading.Event()}
        self.jobs.append(job)
        self.request_ids[identity] = job
        # Keep active/pending records even if a caller configures a tiny limit.
        while self.history_limit is not None and len(self.jobs) > self.history_limit:
            removable = next((row for row in self.jobs if row is not self.active
                              and row is not self.pending_disable and row is not job), None)
            if removable is None:
                break
            self.jobs.remove(removable)
            for key in removable['request_ids']:
                self.request_ids.pop(key, None)
        return job

    # 注释：在锁内处理重复请求、排队和取消，保持同一时刻一个硬件事务。
    def submit(self, action, request_id=None):
        if action not in ('prepare', 'enable', 'disable', 'park', 'open-vr', 'close-vr'):
            raise RequestError(400, 'Unknown operation')
        if request_id is not None and (not isinstance(request_id, str) or not REQUEST_ID.fullmatch(request_id)):
            raise RequestError(400, 'Invalid request_id')
        identity = request_id or uuid.uuid4().hex
        physical = self._telemetry()
        with self.condition:
            if identity in self.request_ids:
                existing = self.request_ids[identity]
                if existing['action'] != action:
                    raise RequestError(409, 'request_id already belongs to another action')
                return 200, self._public(existing)
            if self.stopping or self.journal_error:
                raise RequestError(503, 'Backend is stopping or its journal is unavailable')
            same_stop = (self.pending_disable if self.pending_disable and self.pending_disable['action']==action
                         else self.active if self.active and self.active['action']==action else None)
            if action in ('disable','close-vr','park') and same_stop:
                job = same_stop
                job['request_ids'].append(identity)
                self.request_ids[identity] = job
                self._persist()
                return 202, self._public(job)
            if action=='close-vr' and (self.pending_disable and self.pending_disable['action']=='disable'
                                      or self.active and self.active['action']=='disable'):
                raise RequestError(409, 'Hardware OFF is already pending; VR close cannot replace it')
            if action=='close-vr' and self.active and self.active['action']!='open-vr':
                raise RequestError(409, 'VR close does not cancel a motor operation')
            stop_vr=action=='close-vr' and self.active and self.active['action']=='open-vr'
            if action != 'disable' and not stop_vr and (self.active or self.pending_disable or physical.get('busy_external')):
                raise RequestError(409, 'Another operation is running')
            permitted = {'prepare':'can_initialize','enable':'can_enable','disable':'can_disable',
                         'park':'can_park','open-vr':'can_open_vr','close-vr':'can_close_vr'}[action]
            # An explicit stop may cooperatively cancel an active/external job.
            may_stop_running = action == 'disable' and (self.active or physical.get('busy_external')) or stop_vr
            if not may_stop_running and physical.get(permitted) is not True:
                raise RequestError(409, 'Current state does not permit this operation')
            if action=='disable' and self.pending_disable and self.pending_disable['action']=='close-vr':
                self.pending_disable.update(phase='cancelled',finished_utc=utc(),
                                            error='Superseded by explicit hardware OFF')
                self.pending_disable=None
            job = self._new_job(action, identity)
            if self.active:
                self.pending_disable = job
                self.active['phase'] = 'cancelling'
                self.active['_cancel'].set()
            else:
                self.active = job
            try:
                self._persist()
            except OSError:
                # No unjournaled operation is dispatched. Existing work retains
                # its cooperative cancel signal, but no extra action is launched.
                if self.active is job:
                    self.active = None
                if self.pending_disable is job:
                    self.pending_disable = None
                job.update(phase='failed', error=self.journal_error, finished_utc=utc())
                raise RequestError(503, self.journal_error)
            self.condition.notify_all()
            return 202, self._public(job)

    # 注释：单工作线程消费队列，执行固定操作并记录成功、取消或失败。
    def _work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.active is not None or self.stopping)
                if self.active is None:
                    return
                job = self.active
                job['phase'] = 'running' if not job['_cancel'].is_set() else 'cancelling'
                job['started_utc'] = utc()
                try:
                    self._persist()
                except OSError:
                    job.update(phase='failed', error=self.journal_error, finished_utc=utc())
                    self.active = None
                    self.condition.notify_all()
                    continue
            try:
                if job['_cancel'].is_set() and job['action'] != 'disable':
                    receipt = {'success': False, 'cancelled': True, 'error': 'Cancelled before execution'}
                else:
                    receipt = self.executor.run(job['action'], job['_cancel'])
                if not isinstance(receipt, dict):
                    raise RuntimeError('Executor returned no receipt')
                json.dumps(receipt, allow_nan=False)
                error = None if receipt.get('success') is True else str(receipt.get('error') or 'Executor did not verify success')
                phase = 'complete' if error is None else 'cancelled' if job['_cancel'].is_set() else 'failed'
            except Exception as exception:
                receipt, error, phase = None, str(exception), 'failed'
            with self.condition:
                job.update(receipt=receipt, error=error, phase=phase, finished_utc=utc())
                self.active = self.pending_disable
                self.pending_disable = None
                try:
                    self._persist()
                except OSError:
                    # A successful control action stays recorded in memory; do
                    # not replay it or launch the pending operation on journal loss.
                    if self.active is not None:
                        self.active.update(phase='failed', error=self.journal_error, finished_utc=utc())
                        self.active = None
                self.condition.notify_all()

    # 注释：释放当前对象持有的线程、ROS 或网络资源。
    def close(self, wait=True):
        with self.condition:
            self.stopping = True
            if self.active:
                self.active['_cancel'].set()
            self.condition.notify_all()
        if wait:
            self.worker.join()


# 注释：为指定队列与鉴权信息创建 HTTP 处理器。
def handler_type(queue, token):
    # 注释：本模块的 handler_type.Handler 定义；用途见文件说明与下方原实现。
    class Handler(BaseHTTPRequestHandler):
        # 注释：处理或抑制 HTTP 访问日志，不改变接口业务结果。
        def log_message(self, *_):
            pass

        # 注释：序列化 JSON 响应，设置状态码及缓存/长度等 HTTP 头。
        def reply(self, status, data):
            if status >= 400 and self.command in ('POST', 'PUT', 'PATCH', 'DELETE') and not getattr(self, 'body_read', False):
                # A bounded rejected request must be consumed before closing,
                # otherwise Windows clients may see TCP reset instead of 4xx.
                self.close_connection = True
                lengths = self.headers.get_all('Content-Length', [])
                if len(lengths) == 1 and lengths[0].isdecimal() and int(lengths[0]) <= 4096:
                    self.body_read = True
                    try:
                        self.connection.settimeout(.2)
                        self.rfile.read(int(lengths[0]))
                    except OSError:
                        pass
            raw = json.dumps(data, allow_nan=False, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(raw)

        # 注释：校验内部 API 的调用凭据，不把公网来源视为可信。
        def authenticated(self):
            provided = self.headers.get_all('X-OpenArm-Panel-Token', [])
            return len(provided) == 1 and hmac.compare_digest(provided[0].encode('utf-8'), token.encode('ascii'))

        # 注释：处理 HTTP GET；按固定路由返回状态、拒绝请求或交给已有处理器。
        def do_GET(self):
            if not self.authenticated():
                return self.reply(403, {'error': 'Forbidden'})
            if self.path != '/state':
                return self.reply(404, {'error': 'Unknown endpoint'})
            self.reply(200, queue.state())

        # 注释：处理 HTTP POST；按固定路由返回状态、拒绝请求或交给已有处理器。
        def do_POST(self):
            self.close_connection = True
            if not self.authenticated():
                return self.reply(403, {'error': 'Forbidden'})
            if self.path not in ('/initialize', '/set'):
                return self.reply(404, {'error': 'Unknown endpoint'})
            try:
                lengths = self.headers.get_all('Content-Length', [])
                if self.headers.get('Transfer-Encoding') or len(lengths) != 1 or not lengths[0].isdecimal():
                    raise RequestError(400, 'One Content-Length is required')
                length = int(lengths[0])
                if length > MAX_BODY:
                    raise RequestError(413, 'Body exceeds 512 bytes')
                if self.headers.get_content_type() != 'application/json':
                    raise RequestError(415, 'application/json required')
                self.connection.settimeout(2)
                self.body_read = True
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise RequestError(400, 'Incomplete body')
                body = json.loads(raw.decode('utf-8'), object_pairs_hook=unique_object)
                if not isinstance(body, dict):
                    raise RequestError(400, 'JSON object required')
                if self.path == '/initialize':
                    if body:
                        raise RequestError(400, 'initialize accepts only an empty object')
                    action, identity = 'prepare', None
                else:
                    if set(body)<={'enabled','request_id'} and type(body.get('enabled')) is bool:
                        action='enable' if body['enabled'] else 'disable'
                    elif set(body)<={'vr_enabled','request_id'} and type(body.get('vr_enabled')) is bool:
                        action='open-vr' if body['vr_enabled'] else 'close-vr'
                    elif set(body)<={'park','request_id'} and body.get('park') is True:
                        action='park'
                    else:
                        raise RequestError(400, 'set requires exactly one enabled:boolean or vr_enabled:boolean and optional request_id')
                    identity=body.get('request_id')
                status, operation = queue.submit(action, identity)
                self.reply(status, {'operation': operation})
            except RequestError as error:
                self.reply(error.status, {'error': str(error)})
            except (ValueError, UnicodeError, TimeoutError):
                self.reply(400, {'error': 'Invalid or incomplete JSON body'})
            except OSError:
                self.reply(503, {'error': 'Panel operation journal unavailable'})

    return Handler


# 注释：该文件的显式执行入口；127.0.0.1:8781 接收鉴权请求；通过单工作线程提交原有操作并保存回执。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--port',type=int,default=8781)
    args = parser.parse_args()
    token = (args.root / 'state/panel_api_token').read_text(encoding='ascii').strip()
    if not 32 <= len(token) <= 256 or not token.isascii():
        raise RuntimeError('Invalid panel API credential file')
    from panel_operations import OperationExecutor
    queue = JobQueue(OperationExecutor(args.root), args.root)
    if not 1024<=args.port<=65535:
        parser.error('Invalid local API port')
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_type(queue, token))

    # 注释：响应退出请求或停止当前所属子进程，具体边界见调用处。
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        queue.close()


if __name__ == '__main__':
    main()
