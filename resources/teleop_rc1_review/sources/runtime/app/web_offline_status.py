# 项目中文注释（2026-10-29）：无 CAN 时的 Web 状态。
# 调用与职责：缺少任一 CAN 接口时提供只读离线状态，使网页可启动。
# 运行边界：电机状态保持未知；插回 CAN 不自动启动驱动。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Truthful Web-only status. No ROS, CAN, device access or control endpoints."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import time


# 注释：构造纯 Web 的未知硬件状态；不把零个反馈误写成 16 个电机均已关闭。
def status():
    return {
        'mode': 'original_web_offline', 'web_only': True,
        'ros_control_started': False, 'status_generated_monotonic_s': time.monotonic(),
        'state': 'unknown', 'ready': False, 'control_available': False, 'read_only': True,
        'hardware_connected': None, 'motor_state_verified': False,
        'online': 0, 'enabled': 0, 'disabled': 0, 'faulted': 0, 'motors': [], 'joints': [],
        'hardware': {}, 'hardware_states': {}, 'hardware_age_ms': None,
        'controllers': {}, 'controllers_age_ms': None,
        'driver_quiescent': True, 'known_motor_enabled': None, 'output_enabled': False,
        'metrics': {}, 'joy': {}, 'busy': False,
        'can': {'receive_only': True, 'interfaces': {
            name: {'exists': (Path('/sys/class/net')/name).exists()}
            for name in ('can0','can1')}},
        'phase': '机械臂未连接 · 仅 Web',
        'error': '本次启动未加载机械臂驱动。连接 USB-CAN 后，先由管理员重启服务，再按原流程手动初始化和使能。',
    }


# 注释：本模块的 Handler 定义；用途见文件说明与下方原实现。
class Handler(BaseHTTPRequestHandler):
    # 注释：处理 HTTP GET；按固定路由返回状态、拒绝请求或交给已有处理器。
    def do_GET(self):
        if self.path!='/status':
            self.send_error(404);return
        data=json.dumps(status(),ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(data)))
        self.end_headers();self.wfile.write(data)

    # 注释：处理或抑制 HTTP 访问日志，不改变接口业务结果。
    def log_message(self,*args):pass


if __name__=='__main__':
    ThreadingHTTPServer(('127.0.0.1',8767),Handler).serve_forever()
