# 项目中文注释（2026-10-29）：保留的内部代理函数与历史网页实现。
# 调用与职责：当前 motor_web 仍导入 panel_proxy；旧 4443/4447 HTTP 启动入口已退役。
# 运行边界：不能把仍被导入的代理模块当作无用文件删除。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Preserved Chinese console; explicit test-panel mode uses a fixed host API."""
import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import ssl
from urllib.parse import urlsplit
from urllib.request import urlopen, Request
from urllib.error import HTTPError

ROOT = Path('/test/legacy_ui')
APP = Path('/test/app')
STATE = Path('/state')
PANEL_MODE = False
SOURCE_MODE = False
PANEL_PORT = 4447
PANEL_BACKEND = 'http://127.0.0.1:8781'
REMOVED = ('motor-toggle.js', 'motor-layout.js', 'vr-operation-view.', 'vr-start-guide.',
           'vr-view-mirror.', 'capture-panel.js', 'trajectory-record-panel.js',
           'umi-station-bootstrap.', 'umi-teleop-bootstrap.')


# 注释：本模块的 console_html 定义；用途见文件说明与下方原实现。
def console_html(source, panel_mode=False, source_mode=False):
    # 注释：本模块的 console_html.script 定义；用途见文件说明与下方原实现。
    def script(match):
        removed=tuple(name for name in REMOVED if not source_mode or name not in ('motor-toggle.js','motor-layout.js'))
        return '' if any(name in match.group(0) for name in removed) else match.group(0)
    source = re.sub(r'<script\b[^>]*src=[^>]*>\s*</script>', script, source, flags=re.I)
    scripts='<script defer src="/legacy-readonly.js"></script>'
    if source_mode:
        references=re.findall(r'<script\b[^>]*\bsrc\s*=\s*["\']([^"\']+)["\']',source,flags=re.I)
        has_motor_toggle=any(not urlsplit(value).netloc and urlsplit(value).path in ('/motor-toggle.js','motor-toggle.js','./motor-toggle.js')
                             for value in references)
        scripts='<script>window.__OPENARM_ARCHIVE_SOURCE__=true;</script>'+scripts
        if not has_motor_toggle:scripts+='<script defer src="/motor-toggle.js"></script>'
    elif panel_mode:
        scripts='<script>window.__OPENARM_TEST_PANEL__=true;</script>'+scripts+'<script defer src="/test-motor-panel.js"></script>'
        link=('<a id="oa-motor-control-link" href="/motor-control" '
              'style="position:fixed;top:12px;right:16px;z-index:2147483647;'
              'padding:10px 16px;border:1px solid #fff8;border-radius:8px;'
              'background:#087b68;color:#fff;font:650 14px/1.4 system-ui;'
              'text-decoration:none;box-shadow:0 2px 8px #0002">双臂控制 ↗</a>')
        source=source.replace('</body>',link+'</body>')
    return source.replace('</head>',scripts+'</head>')


# 注释：原实现说明：A standalone shell for the same explicit motor/VR panel and API.
def motor_control_html():
    """A standalone shell for the same explicit motor/VR panel and API."""
    return '''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>双臂控制</title><style>
*{box-sizing:border-box}html{background:#f4f7fa;overflow-y:auto}
body{max-width:640px;margin:0 auto;padding:28px 16px 40px;color:#132b40;font:14px/1.6 system-ui,sans-serif}
nav{display:flex;flex-wrap:wrap;gap:12px;margin-bottom:24px}a{color:#087b68;text-decoration:none}
a:hover{text-decoration:underline}a:focus-visible{outline:3px solid #71c6ff;outline-offset:4px}
h1{margin:0 0 8px;font-size:28px;line-height:1.3}p{margin:0;color:#637788}
ol{margin:16px 0 0;padding-left:22px}li+li{margin-top:6px}
#oa-test-motor-panel{position:static!important;width:100%!important;max-width:100%;margin-top:20px}
</style><script defer src="/test-motor-panel.js"></script></head>
<body><nav aria-label="页面导航"><a href="/">← 返回控制台</a>
<a href="https://192.168.31.220:4446/">进入 VR 场景 ↗</a></nav>
<main><h1>双臂控制</h1><p>电机使能与 VR 运动分别控制。</p>
<ol><li>先点击“使能电机（保持当前位置）”，等待双臂进入使能保持。</li>
<li>头显进入 VR 后，再点击“开启 VR 运动”，开始手柄遥操。</li></ol></main>
</body></html>'''


# 注释：复用内部 8781 的固定端点和本机令牌；当前 4446 电机 API 仍依赖此函数。
def panel_proxy(path,body=None):
    token=(STATE/'panel_api_token').read_text(encoding='ascii').strip()
    request=Request(PANEL_BACKEND+path,data=body,
                    headers={'X-OpenArm-Panel-Token':token,'Content-Type':'application/json'})
    try:
        with urlopen(request,timeout=3) as response:
            return response.status,response.read()
    except HTTPError as error:
        with error:
            return error.code,error.read()


# 注释：本模块的 Handler 定义；用途见文件说明与下方原实现。
class Handler(SimpleHTTPRequestHandler):
    # 注释：初始化 Handler 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    # 注释：发送已有字节内容及正确的内容类型和长度。
    def send_bytes(self, body, content_type, status=200):
        if status>=400 and self.command in ('POST','PUT','PATCH','DELETE') and not getattr(self,'body_read',False):
            # Drain bounded rejected bodies before closing so clients receive
            # the rejection instead of a TCP reset from unread request data.
            self.close_connection=True
            lengths=self.headers.get_all('Content-Length',[])
            if len(lengths)==1 and lengths[0].isdecimal() and int(lengths[0])<=4096:
                try:
                    self.connection.settimeout(.2)
                    self.rfile.read(int(lengths[0]))
                except OSError:
                    pass
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    # 注释：处理 HTTP GET；按固定路由返回状态、拒绝请求或交给已有处理器。
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/motor-control':
            if SOURCE_MODE:
                self.send_response(302);self.send_header('Location','/');self.send_header('Content-Length','0');self.end_headers()
            elif PANEL_MODE:
                self.send_bytes(motor_control_html().encode(),'text/html; charset=utf-8')
            else:
                self.send_bytes(b'Not found','text/plain; charset=utf-8',404)
        elif (PANEL_MODE or SOURCE_MODE) and path == '/api/motors':
            try:
                code,data=panel_proxy('/state')
                self.send_bytes(data,'application/json; charset=utf-8',code)
            except (OSError,ValueError):
                self.send_bytes('{"state":"unknown","feedback_fresh":false,"can_initialize":false,"can_enable":false,"can_disable":false,"error":"使能服务暂不可达"}'.encode(),
                                'application/json; charset=utf-8',503)
        elif path == '/original-vr':
            self.send_response(302)
            self.send_header('Location', 'https://192.168.31.220:4446/')
            self.send_header('Content-Length', '0')
            self.end_headers()
        elif path in ('/api/original/status', '/api/motor/state', '/api/teleop/diagnostics'):
            try:
                with urlopen('http://127.0.0.1:8767/status', timeout=2) as response:
                    data = json.load(response)
                self.send_bytes(json.dumps(data).encode(), 'application/json')
            except (OSError, ValueError):
                self.send_bytes(b'{"mode":"original_standby","ready":false,"output_enabled":null}',
                                'application/json', 503)
        elif path == '/ws' or self.headers.get('Upgrade', '').lower() == 'websocket':
            self.send_bytes(b'Chinese console is read-only. Use /original-vr.', 'text/plain', 409)
        elif path.startswith('/api/'):
            self.send_bytes(b'{"detail":"This console has no control backend"}', 'application/json', 404)
        elif path in ('/', '/index.html'):
            self.send_bytes(console_html((ROOT/'index.html').read_text(),PANEL_MODE,SOURCE_MODE).encode(), 'text/html; charset=utf-8')
        elif path == '/legacy-readonly.js':
            self.send_bytes((APP/'legacy-readonly.js').read_bytes(), 'text/javascript; charset=utf-8')
        elif SOURCE_MODE and path == '/motor-toggle.js':
            self.send_bytes((APP/'motor-toggle.js').read_bytes(),'text/javascript; charset=utf-8')
        elif SOURCE_MODE and path == '/test-motor-panel.js':
            self.send_bytes(b'Archived test panel','text/plain',410)
        elif PANEL_MODE and path == '/test-motor-panel.js':
            self.send_bytes((APP/'test-motor-panel.js').read_bytes(),'text/javascript; charset=utf-8')
        else:
            super().do_GET()

    do_HEAD = do_GET

    # 注释：处理 HTTP POST；按固定路由返回状态、拒绝请求或交给已有处理器。
    def do_POST(self):
        path=urlsplit(self.path).path
        if (PANEL_MODE or SOURCE_MODE) and path in ('/api/motors','/api/motors/initialize'):
            self.close_connection=True
            host=self.headers.get('Host','')
            allowed={name+':'+str(PANEL_PORT) for name in ('192.168.31.220','127.0.0.1','localhost')}
            if host not in allowed or self.headers.get('Origin')!='https://'+host:
                return self.send_bytes(b'{"error":"Same-origin HTTPS request required"}','application/json',403)
            lengths=self.headers.get_all('Content-Length',[])
            if self.headers.get('Transfer-Encoding') or len(lengths)!=1 or not lengths[0].isdecimal():
                return self.send_bytes(b'{"error":"Invalid content length"}','application/json',400)
            length=int(lengths[0])
            if not 0<length<=512:
                return self.send_bytes(b'{"error":"Invalid body size"}','application/json',413)
            if self.headers.get_content_type()!='application/json':
                return self.send_bytes(b'{"error":"JSON required"}','application/json',415)
            try:
                self.connection.settimeout(3)
                body=self.rfile.read(length)
                self.body_read=True
                if len(body)!=length:
                    return self.send_bytes(b'{"error":"Incomplete body"}','application/json',400)
                if SOURCE_MODE and not path.endswith('/initialize'):
                    try:payload=json.loads(body)
                    except (ValueError,UnicodeError):
                        return self.send_bytes(b'{"error":"Invalid JSON"}','application/json',400)
                    if not isinstance(payload,dict) or type(payload.get('enabled')) is not bool or set(payload)-{'enabled','request_id'}:
                        return self.send_bytes(b'{"error":"Only the existing motor switch is available"}','application/json',400)
                code,data=panel_proxy('/initialize' if path.endswith('/initialize') else '/set',body)
                if SOURCE_MODE and code<300:
                    state_code,data=panel_proxy('/state')
                    if state_code>=400:code=state_code
                return self.send_bytes(data,'application/json; charset=utf-8',code)
            except (OSError,ValueError):
                return self.send_bytes('{"error":"操作结果未知，请核对状态，不要重复使能"}'.encode(),
                                       'application/json; charset=utf-8',503)
        self.send_bytes('{"detail":"中文控制台仅查看，请从原版 VR 入口访问"}'.encode(),
                        'application/json; charset=utf-8', 409)

    # 注释：处理 HTTP PUT；按固定路由返回状态、拒绝请求或交给已有处理器。
    def do_PUT(self):
        self.send_bytes(b'{"error":"Only explicit POST actions are supported"}','application/json',405)

    do_PATCH = do_PUT
    do_DELETE = do_PUT


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path)
    parser.add_argument('--port',type=int,default=4443)
    parser.add_argument('--motor-panel',action='store_true')
    args=parser.parse_args()
    if args.project_root:
        ROOT=args.project_root/'legacy_ui'
        APP=args.project_root/'app'
        STATE=args.project_root/'state'
    PANEL_MODE=args.motor_panel
    SOURCE_MODE=(STATE/'source_control.json').is_file()
    PANEL_PORT=args.port
    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(str(STATE/'tls/cert.pem'),str(STATE/'tls/key.pem'))
    server.socket = context.wrap_socket(server.socket, server_side=True)
    server.serve_forever()
