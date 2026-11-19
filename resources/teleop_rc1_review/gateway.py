#!/usr/bin/env python3
"""HTTPS UI facade. Reuses one existing backend; contains no robot driver.

Mutations require the exact public origin before forwarding. The internal
origin is rewritten only after that check. WebSocket bytes remain unchanged.
"""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import select
import socket
import ssl
import threading
from urllib.parse import urlsplit

POST_PATHS={'/api/motors','/api/motors/initialize','/api/motors/park',
            '/api/capture/teleop/action','/api/umi/motion/action'}
HOP={'connection','keep-alive','proxy-authenticate','proxy-authorization',
     'te','trailer','transfer-encoding','upgrade','content-length','content-encoding'}


def local_hosts():
    hosts={'localhost','127.0.0.1'}
    try:
        import fcntl,struct
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            for _,name in socket.if_nameindex():
                try:
                    raw=fcntl.ioctl(s.fileno(),0x8915,struct.pack('256s',name.encode()[:15]))
                    hosts.add(socket.inet_ntoa(raw[20:24]))
                except OSError:pass
    except ImportError:pass
    return hosts


def valid_origin(headers,port,extra=()):
    hosts=headers.get_all('Host',[])
    origins=headers.get_all('Origin',[])
    if len(hosts)!=1 or len(origins)!=1:return False
    origin=urlsplit(origins[0])
    try:
        return (origin.scheme=='https' and origin.netloc==hosts[0]
                and origin.port==port and origin.hostname in local_hosts()|set(extra)
                and not origin.path and not origin.query and not origin.fragment
                and origin.username is None and origin.password is None
                and headers.get('Sec-Fetch-Site')!='cross-site')
    except ValueError:return False


def handler(settings):
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        rbufsize=0

        def log_message(self,fmt,*args):
            # Do not log tokens, bodies or query strings.
            print(json.dumps({'client':self.client_address[0],'method':self.command,
                              'path':urlsplit(self.path).path}),flush=True)

        def reply(self,code,raw,kind='application/json'):
            self.send_response(code);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Permissions-Policy','xr-spatial-tracking=(self)')
            self.end_headers();self.wfile.write(raw)

        def error(self,code,msg):
            self.close_connection=True
            self.reply(code,json.dumps({'error':msg},ensure_ascii=False).encode())

        def do_GET(self):
            path=urlsplit(self.path).path
            if self.headers.get('Origin') and not valid_origin(self.headers,settings.port,settings.allowed_host):
                return self.error(403,'Origin rejected')
            if self.headers.get('Upgrade','').lower()=='websocket':
                if path!='/ws' or not valid_origin(self.headers,settings.port,settings.allowed_host):
                    return self.error(403,'WebSocket origin rejected')
                return self.websocket()
            files={'/':('index.html','text/html; charset=utf-8'),
                   '/index.html':('index.html','text/html; charset=utf-8'),
                   '/style.css':('style.css','text/css; charset=utf-8'),
                   '/app.js':('app.js','text/javascript; charset=utf-8'),
                   '/xr-shell.js':('xr-shell.js','text/javascript; charset=utf-8')}
            if path in files:
                name,kind=files[path]
                return self.reply(200,(settings.web/name).read_bytes(),kind)
            if path=='/healthz':
                return self.reply(200,json.dumps({'service':'RobotControl UI','port':settings.port,
                    'backend':settings.upstream,'new_can_writer':False}).encode())
            # The old optional desktop mirror points at a different machine's
            # 8444 service. It is not part of the portable VR control product.
            if path=='/vr-view-mirror.r19.js':
                return self.reply(200,b'/* Desktop mirror is not loaded by RobotControl VR. */','text/javascript')
            # Old VR HTML/assets are served through the same origin and backend.
            if path in ('/xr','/xr/'):
                return self.proxy('/',inject=True)
            return self.proxy(self.path)

        def do_POST(self):
            if not valid_origin(self.headers,settings.port,settings.allowed_host):
                return self.error(403,'必须从当前 RobotControl 页面操作')
            if urlsplit(self.path).path not in POST_PATHS:return self.error(404,'Unknown action')
            lengths=self.headers.get_all('Content-Length',[])
            if self.headers.get('Transfer-Encoding') or len(lengths)!=1 or not lengths[0].isdecimal():
                return self.error(400,'Content-Length required')
            size=int(lengths[0])
            if not 0<size<=4096:return self.error(413,'Invalid body size')
            if self.headers.get_content_type()!='application/json':return self.error(415,'JSON required')
            self.connection.settimeout(8)
            raw=b''
            while len(raw)<size:
                piece=self.rfile.read(size-len(raw))
                if not piece:break
                raw+=piece
            if len(raw)!=size:return self.error(400,'Incomplete body')
            # Source endpoint is shared internally; this product exposes VR only.
            if urlsplit(self.path).path=='/api/umi/motion/action':
                try:
                    payload=json.loads(raw)
                    if payload!={'action':'source','source':'vr'}:return self.error(400,'VR source only')
                except ValueError:return self.error(400,'Invalid JSON')
            return self.proxy(self.path,raw)

        def proxy(self,path,body=None,inject=False):
            upstream=urlsplit(settings.upstream)
            if not path.startswith('/') or path.startswith('//'):return self.error(400,'Invalid path')
            # TLS is locally pinned by the fixed loopback upstream, never a client URL.
            conn=http.client.HTTPSConnection(upstream.hostname,upstream.port,
                      context=ssl._create_unverified_context(),timeout=8)
            headers={k:v for k,v in self.headers.items() if k.lower() not in HOP|{
                'host','origin','accept-encoding','forwarded','x-forwarded-host','x-forwarded-proto','x-forwarded-for'}}
            headers['Host']=upstream.netloc;headers['Accept-Encoding']='identity'
            if self.headers.get('Origin') or body is not None:headers['Origin']=settings.upstream
            headers['Sec-Fetch-Site']='same-origin'
            try:
                conn.request('POST' if body is not None else 'GET',path,body=body,headers=headers)
                r=conn.getresponse();data=r.read(64*1024*1024+1)
                if len(data)>64*1024*1024:return self.error(502,'Backend response too large')
                kind=r.getheader('Content-Type','application/octet-stream')
                if inject and r.status==200 and 'text/html' in kind:
                    back=b'<a href="/" style="position:fixed;right:12px;bottom:12px;z-index:2147483640;background:#078373;color:white;padding:10px 16px;border-radius:8px;text-decoration:none">RobotControl</a><script defer src="/xr-shell.js"></script>'
                    data=data.replace(b'</body>',back+b'</body>')
                self.reply(r.status,data,kind)
            except (OSError,http.client.HTTPException):self.error(503,'控制后台暂不可用，请等待服务恢复')
            finally:conn.close()

        def websocket(self):
            upstream=urlsplit(settings.upstream)
            raw_sock=socket.create_connection((upstream.hostname,upstream.port),timeout=8)
            upstream_sock=ssl._create_unverified_context().wrap_socket(raw_sock,server_hostname=upstream.hostname)
            self.close_connection=True
            try:
                lines=['GET /ws HTTP/1.1','Host: '+upstream.netloc,'Origin: '+settings.upstream,
                       'Connection: Upgrade','Upgrade: websocket']
                for name in ('Sec-WebSocket-Key','Sec-WebSocket-Version','Sec-WebSocket-Protocol','Sec-WebSocket-Extensions'):
                    if self.headers.get(name):lines.append(name+': '+self.headers[name])
                upstream_sock.sendall(('\r\n'.join(lines)+'\r\n\r\n').encode('ascii'))
                response=b''
                while b'\r\n\r\n' not in response:
                    piece=upstream_sock.recv(1)
                    if not piece:raise OSError('Backend closed during handshake')
                    response+=piece
                    if len(response)>32768:raise OSError('Oversized handshake')
                self.connection.sendall(response)
                if not response.startswith(b'HTTP/1.1 101'):return
                self.connection.settimeout(15);upstream_sock.settimeout(15)
                peers=[self.connection,upstream_sock]
                while True:
                    ready=[s for s in peers if isinstance(s,ssl.SSLSocket) and s.pending()]
                    if not ready:ready,_,_=select.select(peers,[],[],30)
                    for source in ready:
                        data=source.recv(65536)
                        if not data:return
                        (upstream_sock if source is self.connection else self.connection).sendall(data)
            except OSError:pass
            finally:upstream_sock.close()
    return Handler


def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=4448)
    p.add_argument('--bind',default='0.0.0.0');p.add_argument('--web',type=Path,required=True)
    p.add_argument('--cert',type=Path,required=True);p.add_argument('--key',type=Path,required=True)
    p.add_argument('--upstream',default='https://127.0.0.1:4446')
    p.add_argument('--allowed-host',action='append',default=[])
    args=p.parse_args()
    u=urlsplit(args.upstream)
    if u.scheme!='https' or u.hostname!='127.0.0.1' or u.port==args.port:
        p.error('Upstream must be a separate loopback HTTPS service')
    server=ThreadingHTTPServer((args.bind,args.port),handler(args));server.daemon_threads=True
    ctx=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);ctx.minimum_version=ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(args.cert,args.key);server.socket=ctx.wrap_socket(server.socket,server_side=True)
    server.serve_forever()


if __name__=='__main__':main()
