# 项目中文注释（2026-10-29）：CAN 接收与链路观察。
# 调用与职责：解析 Netlink 链路信息和 CAN-FD 电机反馈，按新鲜度生成双臂快照。
# 运行边界：AF_CAN 只接收；Netlink 查询不是 CAN 电机指令。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Receive-only CAN observer. No CAN transmission, enable, or link mutation."""
import select
import socket
import struct
import threading
import time


# 注释：解析按四字节对齐的 Netlink 属性序列。
def attributes(data):
    result = {}
    while len(data) >= 4:
        length, kind = struct.unpack_from('=HH', data)
        if length < 4 or length > len(data):
            break
        result[kind & 0x3fff] = data[4:length]
        data = data[(length+3) & ~3:]
    return result


# 注释：用只读 Netlink 查询 CAN 接口、控制模式和时序，不改变链路。
def inspect_can_links():
    # RTM_GETLINK is read-only. The base image need not install iproute2 or
    # receive NET_ADMIN. Values come from the host network namespace kernel.
    rows = {name: {'exists': False} for name in ['can0', 'can1']}
    with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as s:
        s.settimeout(1)
        s.bind((0, 0))
        body = struct.pack('=BBHiII', socket.AF_UNSPEC, 0, 0, 0, 0, 0)
        s.send(struct.pack('=IHHII', 16+len(body), 18, 0x301, 1, 0)+body)
        done = False
        while not done:
            packet = s.recv(65536)
            while len(packet) >= 16:
                length, kind, _, _, _ = struct.unpack_from('=IHHII', packet)
                if length < 16 or length > len(packet):
                    raise OSError('Invalid netlink response')
                payload = packet[16:length]
                packet = packet[(length+3) & ~3:]
                if kind == 3:
                    done = True
                    break
                if kind == 2:
                    raise OSError('Netlink query failed')
                if kind != 16 or len(payload) < 16:
                    continue
                _, _, _, index, flags, _ = struct.unpack_from('=BBHiII', payload)
                attrs = attributes(payload[16:])
                name = attrs.get(3, b'').rstrip(b'\0').decode()
                if name not in rows:
                    continue
                info = attributes(attrs.get(18, b''))
                if info.get(1, b'').rstrip(b'\0') != b'can':
                    continue
                can = attributes(info.get(2, b''))
                timing = struct.unpack('=8I', can[1]) if len(can.get(1, b'')) == 32 else None
                dtiming = struct.unpack('=8I', can[9]) if len(can.get(9, b'')) == 32 else None
                mode = struct.unpack('=II', can[5])[1] if len(can.get(5, b'')) == 8 else 0
                state = struct.unpack('=I', can[4])[0] if len(can.get(4, b'')) == 4 else None
                states = ['ERROR-ACTIVE', 'ERROR-WARNING', 'ERROR-PASSIVE', 'BUS-OFF', 'STOPPED', 'SLEEPING']
                rows[name] = {'exists': True, 'index': index, 'up': bool(flags & 1),
                    'modes': [label for bit, label in [(1, 'LOOPBACK'), (2, 'LISTEN-ONLY'), (32, 'FD')] if mode & bit],
                    'bus_state': states[state] if state is not None and state < len(states) else None,
                    'bitrate': timing[0] if timing else None, 'data_bitrate': dtiming[0] if dtiming else None,
                    'data_sjw': dtiming[6] if dtiming else None}
    return rows


# 注释：本模块的 CanObserver 定义；用途见文件说明与下方原实现。
class CanObserver:
    # 注释：建立共享缓存与互斥锁，启动接收线程及链路观察线程。
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}
        self.indices = {}
        self.error = ''
        self.links = {}
        threading.Thread(target=self.run, daemon=True).start()
        threading.Thread(target=self.inspect_links, daemon=True).start()

    # 注释：后台定期刷新链路信息，异常时保留错误并清除过时结果。
    def inspect_links(self):
        while True:
            try:
                rows = inspect_can_links()
                with self.lock:
                    self.links = rows
            except (OSError, ValueError, struct.error) as exc:
                with self.lock:
                    self.links = {}
                self.error = str(exc)
            time.sleep(2)

    # 注释：只接收两路 CAN-FD；接口重建时清空旧值并重新绑定，防止沿用上次设备反馈。
    def run(self):
        while True:
            sockets = {}
            try:
                indices = {name: socket.if_nametoindex(name) for name in ['can0', 'can1']}
                with self.lock:
                    if self.indices != indices:
                        self.values.clear()
                    self.indices = indices
                for name in indices:
                    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                    sockets[s] = name
                    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                    s.bind((name,))
                self.error = ''
                while True:
                    if any(socket.if_nametoindex(n) != i for n, i in indices.items()):
                        raise OSError('CAN 接口发生变化')
                    ready, _, _ = select.select(list(sockets), [], [], .3)
                    for s in ready:
                        frame = s.recv(72)
                        cid = struct.unpack_from('=I', frame)[0]
                        data = frame[8:8+frame[4]]
                        mid = cid-0x10
                        if cid & 0xE0000000 or not 1 <= mid <= 8 or len(data) < 8 or data[0] & 15 != mid:
                            continue
                        with self.lock:
                            self.values[sockets[s], mid] = {'state': data[0] >> 4, 'received': time.monotonic(),
                                'position_rad': int.from_bytes(data[1:3], 'big')/65535*25-12.5}
            except OSError as exc:
                self.error = str(exc)
                with self.lock:
                    self.values.clear()
            finally:
                for s in sockets:
                    s.close()
            time.sleep(.5)

    # 注释：按 can0=右、can1=左生成每侧 8 个电机状态；超过 800 ms 的状态和位置返回未知。
    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            motors = []
            for ch, side in [('can0', 'right'), ('can1', 'left')]:
                for mid in range(1, 9):
                    v = self.values.get((ch, mid))
                    fresh = v is not None and now-v['received'] < .8
                    motors.append({'channel': ch, 'side': side, 'id': mid,
                        'state': v['state'] if fresh else None,
                        'age_ms': (now-v['received'])*1000 if v else None,
                        'position_rad': v['position_rad'] if fresh else None})
            return {'receive_only': True, 'interfaces': self.links.copy(), 'error': self.error,
                    'online': sum(m['state'] is not None for m in motors),
                    'enabled': sum(m['state'] == 1 for m in motors),
                    'disabled': sum(m['state'] == 0 for m in motors), 'motors': motors}
