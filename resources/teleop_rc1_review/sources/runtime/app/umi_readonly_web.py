"""Bounded GET-only adapter to the existing paused UMI diagnostic backend.

Legacy services expect Origin 4443. This adapter translates ONLY server-side
diagnostic GETs after checking the incoming canonical origin; it never forwards
actions, cookies or authorization. Public prepare/start/stop remain unavailable.
"""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import http.client
import json
import re
import socket
import ssl
import threading
import time

from fastapi import Request
from fastapi.responses import JSONResponse, Response

from station_network import same_origin
LEGACY_DIAGNOSTIC_ORIGIN = 'https://192.168.31.220:4443'
HEADERS = {'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff'}
JSON_LIMIT = 256 * 1024
IMAGE_LIMIT = 2 * 1024 * 1024
TOTAL_TIMEOUT_S = 2.0
_POOL = ThreadPoolExecutor(max_workers=4, thread_name_prefix='umi-readonly')
_SLOTS = threading.BoundedSemaphore(4)


class BusyError(OSError):
    pass


async def bounded_read(*args):
    # A private pool and pre-submit admission prevent UMI from filling the
    # default executor used by shared motor operations. There is no work queue.
    if not _SLOTS.acquire(blocking=False):
        raise BusyError('UMI diagnostic readers busy')
    def work():
        try:
            return read_internal(*args)
        finally:
            _SLOTS.release()
    try:
        future = _POOL.submit(work)
    except BaseException:
        _SLOTS.release()
        raise
    return await asyncio.shield(asyncio.wrap_future(future))


def allowed_origin(request):
    return same_origin(request, optional=True)


def read_internal(port, path, limit, expected_type, frame_headers=False):
    if (port, path) not in ((8445, '/api/umi/diagnostics'), (8446, '/api/umi/session')):
        if port != 8445 or not re.fullmatch(
            r'/api/umi/cameras/s20008[56]/(?:left|right|tracking)\.jpg\?seq=[0-9]{1,19}', path
        ):
            raise ValueError('UMI diagnostic path is not whitelisted')
    context = ssl.create_default_context(cafile='/state/tls/cert.pem')
    deadline = time.monotonic() + TOTAL_TIMEOUT_S
    active_socket = None
    timer = None
    connection = None
    try:
        # Fixed numeric IP: no DNS, redirects, proxy environment or forwarded credentials.
        active_socket = socket.create_connection(('192.168.31.220', port), timeout=TOTAL_TIMEOUT_S)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('UMI total request deadline')
        active_socket = context.wrap_socket(active_socket, server_hostname='192.168.31.220',
                                             do_handshake_on_connect=False)
        target = active_socket
        def abort():
            # shutdown also unblocks response-file readers retaining the socket.
            try:target.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            target.close()
        timer = threading.Timer(remaining, abort)
        timer.daemon = True
        timer.start()
        active_socket.settimeout(remaining)
        active_socket.do_handshake()
        connection = http.client.HTTPConnection('192.168.31.220', port, timeout=remaining)
        connection.sock = active_socket
        connection.request('GET', path, headers={'Origin': LEGACY_DIAGNOSTIC_ORIGIN,
            'Accept': expected_type, 'Connection': 'close'})
        response = connection.getresponse()
        if response.status != 200 or response.headers.get_content_type() != expected_type:
            raise ValueError('Unexpected UMI diagnostic response')
        length = response.getheader('Content-Length', '')
        if response.getheader('Transfer-Encoding') or not length.isdecimal() or not 0 < int(length) <= limit:
            raise ValueError('UMI diagnostic response is too large')
        image_headers = None
        if frame_headers:
            if expected_type != 'image/jpeg':
                raise ValueError('Frame headers are only available for camera frames')
            image_headers = {key: response.getheader(key) for key in (
                'X-UMI-Frame-Seq', 'X-UMI-Sample-Age-Ms', 'X-UMI-TTL-Ms', 'X-UMI-Run-Id')}
            seq = image_headers['X-UMI-Frame-Seq'] or ''
            age = image_headers['X-UMI-Sample-Age-Ms'] or ''
            ttl = image_headers['X-UMI-TTL-Ms'] or ''
            run = image_headers['X-UMI-Run-Id'] or ''
            if (not re.fullmatch(r'(?:0|[1-9][0-9]{0,18})', seq)
                    or seq != path.rsplit('seq=', 1)[1]
                    or not re.fullmatch(r'(?:0|[1-9][0-9]*)(?:\.[0-9]+)?', age)
                    or len(age) > 24
                    or not re.fullmatch(r'[1-9][0-9]{0,3}', ttl)
                    or not 0 < int(ttl) <= 2000 or float(age) >= int(ttl)
                    or not re.fullmatch(r'[0-9a-f]{32}_[0-9]{1,20}_[0-9]{1,10}', run)):
                raise ValueError('Invalid UMI frame identity or freshness headers')
            if '/tracking.jpg?' in path:
                clock = response.getheader('X-UMI-Clock-Id') or ''
                if not re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', clock):
                    raise ValueError('Tracking frame requires a boot clock identity')
                image_headers['X-UMI-Clock-Id'] = clock
        data = response.read(int(length) + 1)
        if len(data) != int(length) or time.monotonic() > deadline:
            raise ValueError('UMI incomplete response or deadline')
        return (data, image_headers) if frame_headers else data
    finally:
        if timer is not None:timer.cancel()
        if connection is not None:connection.close()
        if active_socket is not None:active_socket.close()


def error_response(message='UMI 诊断后台暂不可达'):
    return JSONResponse({'error': 'umi_diagnostic_backend_unavailable', 'message': message,
                         'available': False, 'robot_commands': False,
                         'real_control_ready': False, 'training_eligible': False},
                        status_code=503, headers=HEADERS)


async def json_snapshot(request, port, path):
    if not allowed_origin(request):
        return JSONResponse({'error': 'Canonical origin required'}, status_code=403, headers=HEADERS)
    try:
        raw = await bounded_read(port, path, JSON_LIMIT, 'application/json')
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError('Expected a UMI snapshot object')
        if port == 8446:
            if value.get('schema') != 'oa.umi.session.v1' or value.get('diagnostic_only') is not True:
                raise ValueError('Expected the existing diagnostic-only UMI session')
            if (value.get('real_control_ready') is not False or value.get('can_start') is not False
                    or not isinstance(value.get('readiness'), dict)
                    or value['readiness'].get('can_start') is not False
                    or value.get('hardware_ack') is not False or value.get('training_eligible') is not False):
                raise ValueError('Diagnostic adapter cannot expose a motion-ready session')
        elif value.get('robot_commands') is not False:
            raise ValueError('Expected a read-only UMI diagnostic source')
        return JSONResponse(value, headers=HEADERS)
    except (OSError, ValueError, http.client.HTTPException):
        return error_response()


async def diagnostics(request: Request):
    return await json_snapshot(request, 8445, '/api/umi/diagnostics')


async def session(request: Request):
    return await json_snapshot(request, 8446, '/api/umi/session')


async def camera(serial: str, eye: str, request: Request):
    if not allowed_origin(request):
        return JSONResponse({'error': 'Canonical origin required'}, status_code=403, headers=HEADERS)
    query = str(request.url.query)
    if serial not in ('s200085', 's200086') or eye not in ('left', 'right', 'tracking') or not re.fullmatch(r'seq=[0-9]{1,19}', query):
        return JSONResponse({'error': 'Invalid UMI image request'}, status_code=400, headers=HEADERS)
    try:
        path = '/api/umi/cameras/' + serial + '/' + eye + '.jpg?' + query
        data, frame_meta = await bounded_read(8445, path, IMAGE_LIMIT, 'image/jpeg', True)
        if not data.startswith(b'\xff\xd8'):
            raise ValueError('Expected a JPEG frame')
        return Response(data, media_type='image/jpeg', headers={**HEADERS, **frame_meta})
    except (OSError, ValueError, http.client.HTTPException):
        return error_response('UMI 当前图像不可用，未返回历史帧')


async def status(request: Request):
    if not allowed_origin(request):
        return JSONResponse({'error': 'Canonical origin required'}, status_code=403, headers=HEADERS)
    result = await diagnostics(request)
    connected = result.status_code == 200
    return JSONResponse({
        'schema': 'oa.station.umi_integration.v1', 'scope': 'web_integration_only',
        'frontend_mounted': True, 'backend_connected': connected,
        'backend_state': 'read_only_diagnostics' if connected else 'unavailable',
        'hardware_state': None, 'real_control_ready': False, 'motion_connected': False,
        'control_source_switch': 'not_implemented', 'actions_forwarded': False,
        'message': 'UMI 只读诊断已接入；真机遥操尚未开放。' if connected else 'UMI 诊断后台暂不可达。',
    }, headers=HEADERS)
