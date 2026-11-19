# 项目中文注释（2026-10-29）：容器内硬件初始化和关闭。
# 调用与职责：封装 ROS 硬件/控制器生命周期、反馈新鲜度、当前姿态保持及故障回退。
# 运行边界：实际使能与关闭发生在这里；不应从目录索引自动执行。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Explicit original-driver lifecycle operations; never publishes position targets.

Run inside the existing container with its sourced ROS environment.  This helper
uses the existing controller manager and the existing receive-only HTTP monitor.
It creates no controller manager, CAN socket, position publisher or enable-true.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time
import urllib.request
import uuid


HARDWARE = ("openarm_right_hardware_interface", "openarm_left_hardware_interface")
JSB = "joint_state_broadcaster"
COMMAND_CONTROLLERS = (
    "left_forward_position_controller", "right_forward_position_controller",
    "left_gripper_controller", "right_gripper_controller",
)
CONTROLLERS = (JSB, *COMMAND_CONTROLLERS)
JOINTS = tuple(f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)) + (
    "openarm_left_finger_joint1", "openarm_right_finger_joint1")
COMMAND_TOPICS = tuple(f"/{side}_forward_position_controller/commands" for side in ("left", "right")) + tuple(
    f"/{side}_gripper_controller/joint_trajectory" for side in ("left", "right"))
STATUS_URL = "http://127.0.0.1:8767/status"
QUERY_RECORD = Path("/test/reports/can_feedback_preparation.json")


# 注释：条件不成立时立即拒绝本次操作，并保留可解释的错误原因。
def require(condition, message):
    if not condition:
        raise RuntimeError(message)


# 注释：等待并核对唯一根 controller_manager，避免双驱动。
def await_single_root_manager(node, spin, *, clock=time.monotonic):
    """Wait only for empty discovery; reject any observed wrong/duplicate owner."""
    deadline = clock() + 5.0
    while True:
        managers = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                    if name == "controller_manager"]
        if managers:
            require(managers == [("controller_manager", "/")],
                    "Expected exactly one root controller_manager: " + str(managers))
            return
        remaining = deadline - clock()
        require(remaining > 0,
                "Expected exactly one root controller_manager after 5-second discovery wait: []")
        spin(min(0.05, remaining))


# 注释：判断输入是否为有限数值，避免 NaN/Inf 进入状态或控制判断。
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 注释：核对 ROS 域及本机通信设置，避免接入错误控制图。
def validate_domain(environment):
    require(environment.get("ROS_DOMAIN_ID") == "94" and environment.get("ROS_LOCALHOST_ONLY") == "1",
            "Requires ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1")


# 注释：核对允许的控制器名称、类型和状态。
def validate_controllers(states, allowed):
    require(set(states) <= set(allowed), "Unexpected third-party controller: " + str(states))
    require(all(value in ("unconfigured", "inactive", "active") for value in states.values()),
            "Unexpected controller state: " + str(states))


# 注释：核对只查询记录的来源、新鲜度和电机集合。
def validate_query_record(record, now=None):
    stamp = datetime.fromisoformat(record.get("time_utc", ""))
    require(stamp.tzinfo is not None, "Disabled query lacks timezone")
    now = now or datetime.now(timezone.utc)
    age = (now - stamp).total_seconds()
    require(0 <= age <= 30, "Disabled query proof older than 30 seconds or future dated")
    require(record.get("query_only") is True and record.get("enable_frames_sent") == 0
            and record.get("motion_frames_sent") == 0 and record.get("zero_frames_sent") == 0,
            "Not an exclusively read/query-only preparation record")
    require(all(record.get(k) == n for k, n in (("online", 16), ("disabled", 16),
                                              ("enabled", 0), ("faulted", 0))), "Query did not prove 16 disabled")
    motors = record.get("motors", [])
    lookup = {(row.get("channel"), row.get("id")): row for row in motors}
    require(len(motors) == len(lookup) == 16, "Disabled query lacks 16 unique motors")
    for channel in ("can0", "can1"):
        for motor_id in range(1, 9):
            row = lookup.get((channel, motor_id), {})
            require(row.get("state") == 0 and finite(row.get("position_rad"))
                    and finite(row.get("age_ms")) and 0 <= row["age_ms"] < 800,
                    "Disabled query contains stale, faulty or invalid motor")
    return {"time_utc": record["time_utc"], "age_s": age, "historical_query_not_live": True,
            "motors": motors}


# 注释：本模块的 UnsafeFeedback 定义；用途见文件说明与下方原实现。
class UnsafeFeedback(RuntimeError):
    pass


# 注释：核对电机身份、反馈年龄、状态和有限关节位置。
def validate_can(status, enabled_sides, active_only=False):
    require(status.get("mode") == "original_standby" and status.get("ros_domain_id") == 94,
            "Unexpected feedback monitor/domain")
    motors = status.get("can", {}).get("motors", [])
    lookup = {(item.get("channel"), item.get("id")): item for item in motors}
    require(len(motors) == 16 and len(lookup) == 16, "Expected exactly 16 motor feedback entries")
    for channel, side in (("can0", "right"), ("can1", "left")):
        for motor_id in range(1, 9):
            row = lookup.get((channel, motor_id), {})
            if row.get("state") not in (None, 0, 1):
                raise UnsafeFeedback(f"Motor fault: {channel}/{motor_id}: {row.get('state')}")
            if active_only and side not in enabled_sides:
                if row.get("state") == 1:
                    raise UnsafeFeedback("Unrequested side became enabled: " + side)
                continue
            require(finite(row.get("age_ms")) and 0 <= row["age_ms"] <= 250,
                    f"Missing/stale CAN feedback: {channel}/{motor_id}")
            require(row.get("state") == (1 if side in enabled_sides else 0),
                    f"Unexpected motor state/fault: {channel}/{motor_id}: {row.get('state')}")
            require(finite(row.get("position_rad")), f"Nonfinite motor position: {channel}/{motor_id}")
    return [{key: row.get(key) for key in ("channel", "id", "state", "position_rad", "age_ms")}
            for row in motors]


# 注释：核对握持与复位按键状态是否符合当前操作前提。
def validate_joy(joy):
    """Reject a recent held grip or home; absent inputs are expected with web frozen."""
    for side, row in joy.items():
        age = row.get("age_ms")
        if finite(age) and 0 <= age <= 1000:
            buttons = row.get("buttons", [])
            require(not row.get("grip") and not any(len(buttons) > i and buttons[i] for i in (1, 4)),
                    "Fresh grip/home pressed: " + side)


# 注释：筛选或验证完整、有限的关节位置集合。
def finite_joints(message):
    require(len(message.name) == len(message.position), "Malformed joint state")
    require(len(set(message.name)) == len(message.name), "Duplicate joint state names")
    positions = dict(zip(message.name, message.position))
    require(all(name in positions and finite(positions[name]) for name in JOINTS),
            "JointState lacks 16 finite original hardware joints")
    return {name: float(positions[name]) for name in JOINTS}


# 注释：比较初始与当前关节位置，拒绝超过阈值的姿态漂移。
def validate_drift(baseline, current, motor_baseline, motors):
    # Finger JointState is linear metres; CAN id8 is the physical motor radians.
    for name in JOINTS[:14]:
        require(abs(current[name] - baseline[name]) <= 0.10, "Arm hold drift > 0.10 rad: " + name)
    old = {(row["channel"], row["id"]): row["position_rad"] for row in motor_baseline}
    for row in motors:
        require(abs(row["position_rad"] - old[(row["channel"], row["id"])]) <= 0.10,
                f"Motor hold drift > 0.10 rad: {row['channel']}/{row['id']}")


# 注释：原实现说明：Small orchestration layer; adapter is injectable for offline order tests.
class Lifecycle:
    """Small orchestration layer; adapter is injectable for offline order tests."""

    # 注释：初始化 Lifecycle 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self, adapter, receipt):
        self.io, self.receipt = adapter, receipt
        self.owned_controllers = set()
        self.owned_hardware = set()

    # 注释：把本事务发生的事件及观察结果记入回执。
    def event(self, event_name, **fields):
        self.receipt["events"].append({"event": event_name, "monotonic": time.monotonic(), **fields})

    # 注释：读取或校验原硬件组件与控制器清单。
    def inventory(self, allowed):
        hardware, controllers = self.io.inventory()
        require(set(hardware) == set(HARDWARE), "Unexpected hardware inventory")
        validate_controllers(controllers, allowed)
        return hardware, controllers

    # 注释：关闭输出并确认本次硬件事务的静止前提。
    def quiet(self):
        self.io.assert_quiet()
        status = self.io.status()
        validate_joy(status.get("joy", {}))
        return status

    # 注释：按已有流程初始化硬件，使用当前反馈位置使能保持。
    def initialize_enable(self):
        self.io.assert_single_manager()
        hardware, controllers = self.inventory(())
        require(all(state == 1 for state in hardware.values()) and not controllers,
                "Initialization requires both HW unconfigured and no loaded controllers")
        self.io.bridge_false()
        self.event("bridge_false_acknowledged")
        self.io.establish_quiet(0.6)
        self.quiet()
        self.receipt["pre_activation_disabled_query"] = self.io.initial_proof()
        self.receipt["command_counts_before"] = self.io.command_counts()
        for name in CONTROLLERS:
            hardware, _ = self.inventory(self.owned_controllers)
            require(all(state == 1 for state in hardware.values()), "HW changed before controller preparation")
            self.quiet()
            # Own the attempt as a timed-out ROS call may still complete server-side.
            self.owned_controllers.add(name)
            self.io.load(name)
            self.io.configure(name)
            self.event("controller_prepared_inactive", name=name)
        hardware, controllers = self.inventory(self.owned_controllers)
        require(set(controllers) == set(CONTROLLERS) and all(s == "inactive" for s in controllers.values()),
                "Five original controllers must be inactive before activation")
        enabled_sides = set()
        for name, side in zip(HARDWARE, ("right", "left")):
            self.quiet()
            hardware, controllers = self.inventory(self.owned_controllers)
            require(hardware[name] == 1, "Hardware no longer starts from unconfigured")
            require(set(controllers) == set(CONTROLLERS) and all(s == "inactive" for s in controllers.values()),
                    "Controller activity appeared before hardware activation")
            self.io.initial_proof()  # Never begin an enable from an expired disabled proof.
            self.owned_hardware.add(name)
            # One direct request. No externally visible INACTIVE/zero-command gap.
            self.io.hardware_state(name, 3)
            enabled_sides.add(side)
            self.io.wait_feedback(enabled_sides, self.quiet, timeout=1.0)
            self.event("hardware_active", name=name)
        self.io.switch((JSB,), ())
        self.event("joint_state_broadcaster_active")
        baseline = self.io.wait_joints(timeout=1.0)
        motors = validate_can(self.quiet(), {"right", "left"})
        self.receipt["hold_baseline"] = {"monotonic": time.monotonic(), "joints": baseline, "motors": motors}
        _, controllers = self.inventory(self.owned_controllers)
        require(controllers.get(JSB) == "active" and all(controllers.get(n) == "inactive" for n in COMMAND_CONTROLLERS),
                "Unexpected controller activity before command-controller activation")
        self.io.switch(COMMAND_CONTROLLERS, ())
        self.event("command_controllers_active_without_target_publication")
        self.io.observe_hold(2.0, lambda: self.check_hold(baseline, motors))
        hardware, controllers = self.inventory(self.owned_controllers)
        require(all(state == 3 for state in hardware.values()) and set(controllers) == set(CONTROLLERS)
                and all(state == "active" for state in controllers.values()), "Final active inventory mismatch")
        self.receipt["final"] = {"hardware": hardware, "controllers": controllers,
                                  "joints": self.io.wait_joints(1.0),
                                  "motors": validate_can(self.quiet(), {"right", "left"})}
        self.receipt["command_counts_after"] = self.io.command_counts()
        require(self.receipt["command_counts_before"] == self.receipt["command_counts_after"],
                "Command publication occurred during initialization")
        self.receipt["bridge_enable_requested"] = False

    # 注释：在使能保持阶段核对反馈和姿态漂移。
    def check_hold(self, baseline, motors):
        current_motors = validate_can(self.quiet(), {"right", "left"})
        current = self.io.current_joints(max_age=0.25)
        validate_drift(baseline, current, motors, current_motors)

    # 注释：关闭原控制器与硬件，并保存关闭结果。
    def shutdown(self, explicit=False):
        errors = []

        # 注释：执行一项收尾动作，收集异常以便仍可尝试其余关闭步骤。
        def attempt(label, operation):
            try:
                operation()
                self.event(label)
            except Exception as error:
                errors.append(label + ": " + str(error))

        attempt("shutdown_bridge_false_acknowledged", self.io.bridge_false)
        hardware, controllers = self.io.inventory()
        require(set(hardware) == set(HARDWARE), "Unexpected shutdown hardware")
        allowed = set(CONTROLLERS) if explicit else self.owned_controllers
        validate_controllers(controllers, allowed)
        owned = allowed.intersection(controllers)
        active = tuple(name for name in COMMAND_CONTROLLERS if name in owned and controllers[name] == "active")
        if active:
            attempt("command_controllers_deactivated", lambda: self.io.switch((), active))
        if JSB in owned and controllers.get(JSB) == "active":
            attempt("joint_state_broadcaster_deactivated", lambda: self.io.switch((), (JSB,)))
        # No target-state 2 request: RM deactivates+cleans up under its own lock.
        candidates = set(HARDWARE) if explicit else self.owned_hardware
        for name in HARDWARE:
            if name in candidates and hardware[name] in (2, 3):
                # Continue to the other owned arm even if an earlier RPC failed.
                attempt("hardware_unconfigured_" + name, lambda name=name: self.io.hardware_state(name, 1))
        hardware, controllers = self.io.inventory()
        validate_controllers(controllers, allowed)
        hardware_off = all(state == 1 for state in hardware.values())
        if not hardware_off:
            errors.append("Shutdown did not reach both HW1")
        # Only unload resources whose names were owned and are actually present.
        # No unload is needed for an attempted load that never reached the server.
        if hardware_off:
            for name in CONTROLLERS:
                if name in owned and name in controllers:
                    if controllers[name] == "active":
                        errors.append("Controller still active; not unloaded: " + name)
                    else:
                        attempt("controller_unloaded_" + name, lambda name=name: self.io.unload(name))
        hardware, controllers = self.io.inventory()
        self.receipt["shutdown"] = {"hardware": hardware, "controllers": controllers,
                                     "verified_hardware_unconfigured": all(s == 1 for s in hardware.values()),
                                     "owned_controllers_unloaded": not set(controllers).intersection(owned),
                                     "motor_disabled_feedback_verified": False}
        # HW1 emits no queries: lack of post-cleanup CAN traffic is not proof of disabled.
        try:
            self.receipt["shutdown"]["motors"] = validate_can(self.io.status(), set())
            self.receipt["shutdown"]["motor_disabled_feedback_verified"] = True
        except Exception as error:
            self.receipt["shutdown"]["feedback_limit"] = str(error)
        if set(controllers).intersection(owned):
            errors.append("Owned controllers remain loaded")
        if errors:
            self.receipt["shutdown"]["errors"] = errors
            raise RuntimeError("; ".join(errors))


# 注释：本模块的 make_ros_adapter 定义；用途见文件说明与下方原实现。
def make_ros_adapter():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents, LoadController, UnloadController,
                                             ConfigureController, SwitchController, SetHardwareComponentState)
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import SetBool
    from trajectory_msgs.msg import JointTrajectory

    # 注释：本模块的 make_ros_adapter.RosAdapter 定义；用途见文件说明与下方原实现。
    class RosAdapter:
        # 注释：初始化 make_ros_adapter.RosAdapter 的状态与依赖；资源和服务创建以函数体为准。
        def __init__(self):
            self.node = Node("original_hardware_lifecycle_20261020_" + uuid.uuid4().hex[:8])
            self.counts = {topic: 0 for topic in COMMAND_TOPICS}
            self.ik_count = 0
            self.quiet_counts = None
            self.joys = {}
            self.joints, self.joints_seen = None, None
            self.clients = {}
            for topic in COMMAND_TOPICS:
                message_type = Float64MultiArray if topic.endswith("/commands") else JointTrajectory
                self.node.create_subscription(message_type, topic, lambda msg, topic=topic: self.command(topic), 10)
            self.node.create_subscription(JointTrajectory, "/joint_trajectory", self.ik, 10)
            self.node.create_subscription(JointState, "/joint_states", self.joint_state, qos_profile_sensor_data)
            for side in ("left", "right"):
                self.node.create_subscription(Joy, f"/xr/controller_{side}/joy",
                                              lambda msg, side=side: self.joy(side, msg), 10)
            self.types = {"list_controllers": ListControllers, "list_hardware_components": ListHardwareComponents,
                          "load_controller": LoadController, "configure_controller": ConfigureController,
                          "unload_controller": UnloadController,
                          "switch_controller": SwitchController, "set_hardware_component_state": SetHardwareComponentState}

        # 注释：处理 command 消息并更新本适配器的观察缓存或计数。
        def command(self, topic):
            self.counts[topic] += 1

        # 注释：处理 ik 消息并更新本适配器的观察缓存或计数。
        def ik(self, _):
            self.ik_count += 1

        # 注释：处理 joy 消息并更新本适配器的观察缓存或计数。
        def joy(self, side, message):
            self.joys[side] = {"seen": time.monotonic(), "buttons": list(message.buttons)}

        # 注释：处理 joint_state 消息并更新本适配器的观察缓存或计数。
        def joint_state(self, message):
            try:
                self.joints = finite_joints(message)
                self.joints_seen = time.monotonic()
            except RuntimeError:
                self.joints, self.joints_seen = None, None

        # 注释：让 ROS 执行回调并等待指定时长。
        def spin(self, duration=0.01):
            rclpy.spin_once(self.node, timeout_sec=duration)

        # 注释：调用带结果校验的既有服务或固定命令；语义取决于所在适配器。
        def call(self, service, request, service_type=None, timeout=3.0):
            service_type = service_type or self.types[service]
            path = service if service.startswith("/") else "/controller_manager/" + service
            if path not in self.clients:
                self.clients[path] = self.node.create_client(service_type, path)
            client = self.clients[path]
            require(client.wait_for_service(timeout_sec=1.0), "Service unavailable: " + path)
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                self.spin()
            require(future.done(), "ROS call timed out; server may still finish: " + path)
            result = future.result()
            require(result is not None, "Empty service response: " + path)
            if hasattr(result, "ok"):
                require(result.ok, "Service refused: " + path)
            if hasattr(result, "success"):
                require(result.success, "Service refused: " + path)
            return result

        # 注释：本模块的 make_ros_adapter.RosAdapter.assert_single_manager 定义；用途见文件说明与下方原实现。
        def assert_single_manager(self):
            await_single_root_manager(self.node, self.spin)

        # 注释：读取或校验原硬件组件与控制器清单。
        def inventory(self):
            hw = self.call("list_hardware_components", ListHardwareComponents.Request())
            ct = self.call("list_controllers", ListControllers.Request())
            return ({row.name: row.state.id for row in hw.component}, {row.name: row.state for row in ct.controller})

        # 注释：通过原桥服务请求关闭输出。
        def bridge_false(self):
            request = SetBool.Request()
            request.data = False
            self.call("/enable", request, SetBool)

        # 注释：加载指定原控制器或校验过的辅助模块，具体对象由调用处固定。
        def load(self, name):
            request = LoadController.Request()
            request.name = name
            self.call("load_controller", request)

        # 注释：配置指定原控制器，仍需后续显式激活。
        def configure(self, name):
            request = ConfigureController.Request()
            request.name = name
            self.call("configure_controller", request)

        # 注释：卸载指定原控制器，不接受任意控制器名称。
        def unload(self, name):
            request = UnloadController.Request()
            request.name = name
            self.call("unload_controller", request)

        # 注释：请求原硬件组件切换 ROS 生命周期状态。
        def hardware_state(self, name, target):
            require(target in (1, 3), "Only direct HW1/HW3 lifecycle targets allowed")
            request = SetHardwareComponentState.Request()
            request.name = name
            request.target_state.id = target
            self.call("set_hardware_component_state", request)

        # 注释：调用控制器切换服务，使用本事务的固定控制器集合。
        def switch(self, activate, deactivate):
            request = SwitchController.Request()
            request.activate_controllers = list(activate)
            request.deactivate_controllers = list(deactivate)
            request.strictness = 2
            request.activate_asap = False
            request.timeout.sec = 2
            self.call("switch_controller", request)

        # 注释：本模块的 make_ros_adapter.RosAdapter.status 定义；用途见文件说明与下方原实现。
        def status(self):
            with urllib.request.urlopen(STATUS_URL, timeout=0.4) as response:
                return json.load(response)

        # 注释：取得本次查询记录的原始字节校验与初始状态证明。
        def initial_proof(self):
            raw = QUERY_RECORD.read_bytes()
            proof = validate_query_record(json.loads(raw))
            return {**proof, "path": str(QUERY_RECORD), "sha256": hashlib.sha256(raw).hexdigest()}

        # 注释：返回各命令话题计数，用于验证初始化期间没有附加运动目标。
        def command_counts(self):
            return {**self.counts, "/joint_trajectory": self.ik_count}

        # 注释：核对桥、IK、手柄及命令计数满足当前静止条件。
        def assert_quiet(self):
            # Drain queued callbacks; repeated spins never wait longer than 1 ms.
            for _ in range(4):
                self.spin(0.001)
            require(self.quiet_counts is not None and self.command_counts() == self.quiet_counts,
                    "Arm/gripper/IK command flow detected")
            now = time.monotonic()
            validate_joy({side: {"age_ms": (now - row["seen"]) * 1000, "buttons": row["buttons"]}
                          for side, row in self.joys.items()})

        # 注释：按已有关闭服务建立无运动输出状态，并核验回读。
        def establish_quiet(self, duration):
            self.quiet_counts = self.command_counts()
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.spin()
                self.assert_quiet()

        # 注释：等待新鲜、完整且状态符合要求的电机反馈。
        def wait_feedback(self, sides, quiet, timeout):
            deadline = time.monotonic() + timeout
            last = None
            while time.monotonic() < deadline:
                self.spin()
                status = quiet()
                try:
                    return validate_can(status, sides, active_only=True)
                except UnsafeFeedback:
                    raise
                except RuntimeError as error:
                    last = error
            raise RuntimeError("Fresh physical enable verification failed: " + str(last))

        # 注释：返回当前反馈中的关节位置，保持名称映射。
        def current_joints(self, max_age):
            require(self.joints is not None and self.joints_seen is not None
                    and 0 <= time.monotonic() - self.joints_seen <= max_age, "No fresh complete JointState")
            return dict(self.joints)

        # 注释：等待完整关节位置，拒绝缺失或过期反馈。
        def wait_joints(self, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.spin()
                self.assert_quiet()
                if self.joints is not None and time.monotonic() - self.joints_seen <= 0.25:
                    return dict(self.joints)
            raise RuntimeError("JointState pending/missing after 1-second discovery wait")

        # 注释：在有限观察窗口内确认电机保持和目标未意外变化。
        def observe_hold(self, duration, check):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.spin(0.02)
                check()

    return RosAdapter()


# 注释：根据显式动作调用已有生命周期事务，异常时保留事件并尝试按原流程关闭。
def execute(action, adapter, receipt):
    lifecycle = Lifecycle(adapter, receipt)
    try:
        if action == "initialize-enable":
            lifecycle.initialize_enable()
        else:
            adapter.assert_single_manager()
            lifecycle.shutdown(explicit=True)
        receipt["success"] = True
    except BaseException as error:
        receipt.update(success=False, error=f"{type(error).__name__}: {error}")
        if action == "initialize-enable" and (lifecycle.owned_hardware or lifecycle.owned_controllers):
            try:
                lifecycle.shutdown()
            except BaseException as rollback_error:
                receipt["rollback_error"] = f"{type(rollback_error).__name__}: {rollback_error}"
    return receipt


# 注释：该文件的显式执行入口；封装 ROS 硬件/控制器生命周期、反馈新鲜度、当前姿态保持及故障回退。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("initialize-enable", "disable"))
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args()
    validate_domain(os.environ)
    receipt = {"run_id": uuid.uuid4().hex, "action": args.action, "ros_domain_id": 94,
               "time_utc": datetime.now(timezone.utc).isoformat(), "events": [],
               "publishes_position_targets": False, "bridge_enable_requested": False}
    destination = args.receipt or Path("/logs") / ("activation_" + receipt["run_id"] + ".json")
    require(destination.is_absolute() and destination.parent == Path("/logs"), "Receipt must be directly under /logs")
    require(destination.parent.is_dir() and not destination.exists(), "Missing writable /logs or receipt already exists")
    # Prove persistence is possible before lifecycle mutation.
    with destination.open("x", encoding="utf-8") as stream:
        json.dump({**receipt, "success": None, "phase": "starting"}, stream)
    import rclpy
    adapter = None
    try:
        rclpy.init()
        adapter = make_ros_adapter()
        execute(args.action, adapter, receipt)
    except BaseException as error:
        receipt.update(success=False, error=f"{type(error).__name__}: {error}")
    finally:
        receipt["finished_utc"] = datetime.now(timezone.utc).isoformat()
        destination.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps(receipt, allow_nan=False), flush=True)
        if adapter is not None:
            adapter.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if receipt.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
