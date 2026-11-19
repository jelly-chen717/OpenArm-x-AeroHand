# 项目中文注释（2026-10-29）：ROS 与电机反馈只读聚合。
# 调用与职责：订阅生命周期、目标和按键，合并接收式 CAN 反馈；127.0.0.1:8767/status。
# 运行边界：只读观察，不提供使能、运动或生命周期修改接口。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Read-only ROS lifecycle and fresh physical feedback for original VR standby.

No publisher, motor query, lifecycle mutation, or enable service exists here.
Imports and the status reducer are usable offline without ROS or CAN sockets.
"""

from collections import deque
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


HARDWARE = ("openarm_right_hardware_interface", "openarm_left_hardware_interface")
METRICS = ("feedback", "ik", "left_command", "right_command", "head_pose",
           "left_pose", "right_pose", "left_joy", "right_joy")
READ_ONLY_MESSAGE = "原版 VR 待机状态接口只读；未开放电机初始化、使能或运动控制"


# 注释：判断输入是否为有限数值，避免 NaN/Inf 进入状态或控制判断。
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 注释：用服务端单调时钟计算数据年龄，单位毫秒；无有效时间则返回未知。
def age_ms(received, now):
    return (now - received) * 1000 if finite(received) and 0 <= now - received else None


# 注释：计算短窗口消息频率、最大间隔和最后一帧年龄。
def rate(samples, count, now):
    recent = [stamp for stamp in samples if 0 <= now - stamp < 2]
    gaps = [b - a for a, b in zip(recent, recent[1:])]
    duration = recent[-1] - recent[0] if len(recent) > 1 else 0
    return {"count": count, "hz": (len(recent) - 1) / duration if duration > 0 else 0.0,
            "max_gap_ms": max(gaps, default=0) * 1000,
            "age_ms": age_ms(samples[-1], now) if samples else None}


# 注释：合并 ROS 与 CAN 观察；过期数据保留未知，历史查询不冒充实时反馈。
def build_status(ros, can, now, last_check=None):
    """Combine observations without promoting stale or historical data to live."""
    hardware = copy.deepcopy(ros.get("hardware", {}))
    hardware_age = age_ms(ros.get("hardware_seen"), now)
    controllers_age = age_ms(ros.get("controllers_seen"), now)
    hardware_fresh = hardware_age is not None and hardware_age < 5000
    hardware_complete = set(hardware) == set(HARDWARE)
    active = hardware_fresh and any(row.get("state_id") == 3 for row in hardware.values())
    fresh_hw_inactive = hardware_fresh and hardware_complete and all(
        row.get("state_id") in (1, 2) for row in hardware.values())
    driver_quiescent = (hardware_fresh and hardware_complete
                        and all(row.get("state_id") == 1 for row in hardware.values())
                        and controllers_age is not None and controllers_age < 5000
                        and ros.get("controllers") == {})

    raw = {(motor.get("channel"), motor.get("id")): motor for motor in can.get("motors", [])}
    motors = []
    for channel, side in (("can0", "right"), ("can1", "left")):
        for motor_id in range(1, 9):
            item = raw.get((channel, motor_id), {})
            age = item.get("age_ms")
            fresh = finite(age) and 0 <= age < 800
            state = item.get("state") if fresh else None
            if not isinstance(state, int) or isinstance(state, bool):
                state = None
            position = item.get("position_rad") if state is not None else None
            motors.append({"channel": channel, "side": side, "id": motor_id,
                           "state": state, "age_ms": age if finite(age) and age >= 0 else None,
                           "position_rad": position if finite(position) else None})
    online = sum(motor["state"] is not None for motor in motors)
    enabled = sum(motor["state"] == 1 for motor in motors)
    disabled = sum(motor["state"] == 0 for motor in motors)
    faulted = sum(motor["state"] not in (None, 0, 1) for motor in motors)
    state = ("enabled" if enabled == 16 else "disabled" if disabled == 16 else
             "fault" if faulted else "unknown" if online != 16 else "mixed")
    output = True if active or enabled else False if fresh_hw_inactive and online == 16 else None
    current_can = {"receive_only": True, "interfaces": copy.deepcopy(can.get("interfaces", {})),
                   "error": can.get("error", ""), "online": online, "enabled": enabled,
                   "disabled": disabled, "faulted": faulted, "motors": motors}

    targets = ros.get("targets", {})
    joints = []
    for motor in motors:
        if motor["id"] > 7:
            continue
        name = f"openarm_{motor['side']}_joint{motor['id']}"
        position = motor["position_rad"]
        target = targets.get(name)
        target = target if finite(target) else None
        joints.append({"name": name, "side": motor["side"], "index": motor["id"],
                       "position_deg": math.degrees(position) if position is not None else None,
                       "target_deg": math.degrees(target) if target is not None else None,
                       "error_deg": math.degrees(target - position) if target is not None and position is not None else None})
    joy = {side: {"age_ms": age_ms(value.get("received"), now), "grip": value.get("grip"),
                  "buttons": value.get("buttons", [])} for side, value in ros.get("joy", {}).items()}
    return {"mode": "original_standby", "ros_domain_id": 94, "control_available": False,
            "ready": False, "read_only": True, "state": state, "online": online,
            "enabled": enabled, "disabled": disabled, "faulted": faulted, "motors": motors,
            "output_enabled": output, "hardware_connected": True if online == 16 else None,
            "motor_state_verified": online == 16, "hardware": hardware,
            "hardware_states": {name: row.get("state_id") for name, row in hardware.items()},
            "driver_quiescent": bool(driver_quiescent), "known_motor_enabled": bool(enabled),
            "hardware_age_ms": hardware_age, "controllers": copy.deepcopy(ros.get("controllers", {})),
            "controllers_age_ms": controllers_age,
            "metrics": copy.deepcopy(ros.get("metrics", {})), "joints": joints, "joy": joy,
            "can": current_can, "last_motor_check": copy.deepcopy(last_check),
            "busy": False, "phase": "原版 VR 待机（只读）", "initialized": None,
            "last_error": ros.get("last_error", ""), "bus_error": can.get("error", ""),
            "standby_policy": "unconfigured_no_automatic_activation",
            "control_message": READ_ONLY_MESSAGE}


# 注释：读取历史查询的摘要及原时间戳，只用于展示。
def read_last_check(path):
    try:
        value = json.loads(Path(path).read_text())
        if not isinstance(value, dict):
            return None
        json.dumps(value, allow_nan=False)
        # History is shown with its original timestamp; it never drives live counts.
        return {key: value.get(key) for key in ("time_utc", "query_only", "online", "disabled", "enabled", "state", "run_id")}
    except (OSError, ValueError):
        return None


# 注释：本模块的 StatusStore 定义；用途见文件说明与下方原实现。
class StatusStore:
    # 注释：初始化 StatusStore 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {"hardware": {}, "hardware_seen": None, "controllers": {},
                     "controllers_seen": None, "targets": {}, "joy": {}, "last_error": ""}
        self.times = {key: deque(maxlen=600) for key in METRICS}
        self.counts = {key: 0 for key in METRICS}

    # 注释：记录某类消息到达时间及累计次数；调用者负责持锁。
    def mark(self, key, now):
        # Called only while self.lock is held by a ROS callback.
        self.times[key].append(now)
        self.counts[key] += 1

    # 注释：在锁内复制状态并计算各消息通道的频率，返回独立快照。
    def snapshot(self, now):
        with self.lock:
            value = copy.deepcopy(self.data)
            value["metrics"] = {key: rate(list(samples), self.counts[key], now)
                                for key, samples in self.times.items()}
        return value


# 注释：创建只读订阅和生命周期查询客户端，收集状态与消息节奏。
def create_ros_node(store):
    from rclpy.node import Node
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory

    # 注释：本模块的 create_ros_node.StatusNode 定义；用途见文件说明与下方原实现。
    class StatusNode(Node):
        # 注释：初始化 create_ros_node.StatusNode 的状态与依赖；资源和服务创建以函数体为准。
        def __init__(self):
            super().__init__("original_vr_readonly_status")
            self.status_clients = {
                "hardware": (self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components"), ListHardwareComponents),
                "controllers": (self.create_client(ListControllers, "/controller_manager/list_controllers"), ListControllers),
            }
            self.pending = {}
            self.create_timer(1.0, self.refresh_lifecycle)
            self.create_subscription(JointState, "/joint_states", lambda message: self.observe("feedback"), 10)
            self.create_subscription(JointTrajectory, "/joint_trajectory", lambda message: self.observe("ik"), 10)
            self.create_subscription(PoseStamped, "/xr/head/pose", lambda message: self.observe("head_pose"), 10)
            for side in ("left", "right"):
                self.create_subscription(Float64MultiArray, f"/{side}_forward_position_controller/commands",
                                         lambda message, side=side: self.command(side, message), 10)
                self.create_subscription(PoseStamped, f"/xr/controller_{side}/pose",
                                         lambda message, side=side: self.observe(f"{side}_pose"), 10)
                self.create_subscription(Joy, f"/xr/controller_{side}/joy",
                                         lambda message, side=side: self.joy(side, message), 10)

        # 注释：记录消息到达，不生成新的运动目标。
        def observe(self, key):
            with store.lock:
                store.mark(key, time.monotonic())

        # 注释：观察控制器实际命令并更新目标角度，单位为弧度；该回调本身不发布命令。
        def command(self, side, message):
            with store.lock:
                store.mark(f"{side}_command", time.monotonic())
                store.data["targets"].update({f"openarm_{side}_joint{index}": float(value)
                                              for index, value in enumerate(message.data[:7], 1) if finite(value)})

        # 注释：记录按键及握持状态；此处只用于诊断和生命周期前提检查。
        def joy(self, side, message):
            now = time.monotonic()
            with store.lock:
                store.mark(f"{side}_joy", now)
                store.data["joy"][side] = {"received": now, "buttons": list(message.buttons),
                                           "grip": bool(len(message.buttons) > 1 and message.buttons[1])}

        # 注释：发送只读生命周期查询，并回收超时的在途查询。
        def refresh_lifecycle(self):
            now = time.monotonic()
            for key, (client, service_type) in self.status_clients.items():
                pending = self.pending.get(key)
                if pending and now - pending[1] > 3:
                    pending[0].cancel()
                    self.pending.pop(key, None)
                if key in self.pending or not client.service_is_ready():
                    continue
                future = client.call_async(service_type.Request())
                self.pending[key] = (future, now)
                future.add_done_callback(lambda done, key=key: self.lifecycle_result(key, done))

        # 注释：处理异步生命周期查询结果，丢弃已过期或被替换的请求。
        def lifecycle_result(self, key, future):
            current = self.pending.get(key)
            if current is None or current[0] is not future:
                return
            self.pending.pop(key, None)
            try:
                response = future.result()
                if key == "hardware":
                    value = {component.name: {"state_id": component.state.id, "state": component.state.label,
                                             "plugin": getattr(component, "plugin_name", None)}
                             for component in response.component}
                else:
                    value = {controller.name: controller.state for controller in response.controller}
                with store.lock:
                    store.data[key] = value
                    store.data[key + "_seen"] = time.monotonic()
                    store.data["last_error"] = ""
            except Exception as error:
                with store.lock:
                    store.data["last_error"] = str(error)

    return StatusNode()


# 注释：把只读快照函数封装成 HTTP 状态处理器。
def make_handler(snapshot):
    # 注释：本模块的 make_handler.Handler 定义；用途见文件说明与下方原实现。
    class Handler(BaseHTTPRequestHandler):
        # 注释：处理或抑制 HTTP 访问日志，不改变接口业务结果。
        def log_message(self, *_):
            pass

        # 注释：序列化 JSON 响应，设置状态码及缓存/长度等 HTTP 头。
        def reply(self, status, value):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        # 注释：处理 HTTP GET；按固定路由返回状态、拒绝请求或交给已有处理器。
        def do_GET(self):
            if urlsplit(self.path).path not in ("/state", "/status"):
                return self.reply(404, {"error": "unknown endpoint"})
            self.reply(200, snapshot())

        # 注释：处理 HTTP POST；按固定路由返回状态、拒绝请求或交给已有处理器。
        def do_POST(self):
            self.close_connection = True
            self.reply(409, {"success": False, "control_available": False, "error": READ_ONLY_MESSAGE})

    return Handler


# 注释：该文件的显式执行入口；订阅生命周期、目标和按键，合并接收式 CAN 反馈；127.0.0.1:8767/status。
def main():
    if os.environ.get("ROS_DOMAIN_ID") != "94" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Original standby requires ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1")
    import rclpy
    from can_observer import CanObserver
    rclpy.init()
    store = StatusStore()
    node = create_ros_node(store)
    observer = CanObserver()
    history = os.environ.get("ORIGINAL_LAST_CHECK_PATH", "/test/reports/can_feedback_preparation.json")

    # 注释：用同一个单调时间合并 ROS、CAN 和历史摘要，确保数据年龄可比较。
    def snapshot():
        now = time.monotonic()
        return build_status(store.snapshot(now), observer.snapshot(), now, read_last_check(history))

    server = ThreadingHTTPServer(("127.0.0.1", 8767), make_handler(snapshot))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
