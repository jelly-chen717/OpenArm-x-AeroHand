# 项目中文注释（2026-10-29）：VR 输入与运动链路采样。
# 调用与职责：订阅输入、关节反馈和 IK 轨迹，结合 HTTP 状态计算频率、间隔和偏差。
# 运行边界：诊断记录，不是新的控制器；历史样本不当作实时状态。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Bounded, read-only original VR sampling. No publisher, service client or CAN.

Run in the existing domain-94 container with its clean ROS environment. Arm
comparisons use the canonical 14 rotary joints by name; finger joints (metres)
are validated as finite but excluded from every angular comparison.
"""

import argparse
from collections import Counter
import datetime
import json
import math
import os
from pathlib import Path
import re
import threading
import time
import urllib.request


ARM_NAMES = tuple(f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8))
CONTROLLERS = ("joint_state_broadcaster", "left_forward_position_controller",
               "right_forward_position_controller", "left_gripper_controller", "right_gripper_controller")


# 注释：生成 UTC 时间字符串，用于回执与日志关联。
def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# 注释：判断输入是否为有限数值，避免 NaN/Inf 进入状态或控制判断。
def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


# 注释：在已有采样值上计算分位数，供延迟和间隔摘要使用。
def quantile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


# 注释：按采样到达时间汇总频率、间隔与长空窗。
def arrival_summary(stamps, start, end):
    gaps = [(b - a) * 1000 for a, b in zip(stamps, stamps[1:])]
    span = stamps[-1] - stamps[0] if len(stamps) > 1 else 0
    return {"count": len(stamps), "hz": (len(stamps) - 1) / span if span > 0 else 0.0,
            "window_hz": len(stamps) / max(end - start, 1e-9),
            "first_s": stamps[0] - start if stamps else None,
            "last_s": stamps[-1] - start if stamps else None,
            "end_age_ms": (end - stamps[-1]) * 1000 if stamps else None,
            "gap_p50_ms": quantile(gaps, .5), "gap_p95_ms": quantile(gaps, .95),
            "max_gap_ms": max(gaps) if gaps else None}


# 注释：按关节名称将位置数组转换为可对比的字典。
def named_positions(names, positions):
    if len(names) != len(positions):
        raise ValueError("names_positions_length_mismatch")
    if any(not isinstance(name, str) for name in names) or len(set(names)) != len(names):
        raise ValueError("invalid_or_duplicate_names")
    if any(not finite(value) for value in positions):
        raise ValueError("nonfinite_position")
    lookup = dict(zip(names, positions))
    missing = [name for name in ARM_NAMES if name not in lookup]
    if missing:
        raise ValueError("missing_arm_joints:" + ",".join(missing))
    return {name: float(lookup[name]) for name in ARM_NAMES}


# 注释：本模块的 HttpSampler 定义；用途见文件说明与下方原实现。
class HttpSampler(threading.Thread):
    # 注释：初始化 HttpSampler 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self):
        super().__init__(daemon=True)
        self.finished = threading.Event()
        self.latest = None
        self.first = None
        self.samples = 0
        self.errors = []
        self.unhealthy = 0
        self.health_issues = []

    # 注释：执行该对象的既有工作循环；订阅输入、关节反馈和 IK 轨迹，结合 HTTP 状态计算频率、间隔和偏差。
    def run(self):
        while not self.finished.is_set():
            try:
                with urllib.request.urlopen("http://127.0.0.1:8767/status", timeout=.5) as response:
                    state = json.load(response)
                healthy = (state.get("online") == 16 and state.get("enabled") == 16
                           and state.get("faulted") == 0
                           and set(state.get("hardware", {})) == {"openarm_left_hardware_interface", "openarm_right_hardware_interface"}
                           and all(row.get("state_id") == 3 for row in state.get("hardware", {}).values())
                           and all(state.get("controllers", {}).get(name) == "active" for name in CONTROLLERS)
                           and finite(state.get("hardware_age_ms")) and 0 <= state["hardware_age_ms"] < 5000
                           and finite(state.get("controllers_age_ms")) and 0 <= state["controllers_age_ms"] < 5000)
                row = {"received_monotonic": time.monotonic(), "online": state.get("online"),
                       "enabled": state.get("enabled"), "disabled": state.get("disabled"),
                       "faulted": state.get("faulted"), "output_enabled": state.get("output_enabled"),
                       "hardware": state.get("hardware"), "hardware_age_ms": state.get("hardware_age_ms"),
                       "controllers": state.get("controllers"), "controllers_age_ms": state.get("controllers_age_ms"),
                       "metrics": state.get("metrics"), "joy": state.get("joy"),
                       "can_interfaces": state.get("can", {}).get("interfaces"),
                       "bus_error": state.get("bus_error"), "healthy_5controllers_16motors": healthy}
                self.samples += 1
                self.latest = row
                if self.first is None:
                    self.first = row
                if not healthy:
                    self.unhealthy += 1
                    if len(self.health_issues) < 6:
                        self.health_issues.append({k: row[k] for k in ("received_monotonic", "online", "enabled", "faulted", "controllers")})
            except Exception as error:
                if len(self.errors) < 8:
                    self.errors.append(type(error).__name__ + ": " + str(error))
            self.finished.wait(.5)


# 注释：本模块的 Recorder 定义；用途见文件说明与下方原实现。
class Recorder:
    # 注释：初始化 Recorder 的状态与依赖；资源和服务创建以函数体为准。
    def __init__(self, started, duration):
        self.started = started
        self.arrivals = {key: [] for key in ("trajectory", "joint_states", "head_pose", "left_pose", "right_pose", "left_joy", "right_joy")}
        self.invalid = Counter()
        self.invalid_examples = []
        self.names_seen = {}
        self.latest_q = None
        self.latest_q_time = None
        self.anchor = None
        self.max_drift = {name: 0.0 for name in ARM_NAMES}
        self.max_difference = {name: None for name in ARM_NAMES}
        self.max_step = {name: None for name in ARM_NAMES}
        self.previous_target = None
        self.first_target = None
        self.first_target_delta = None
        self.compared = 0
        self.valid_targets = 0
        self.target_without_fresh_feedback = 0
        self.windows = []
        self.trace = []
        self.trace_interval = duration / 60
        self.trace_last = -math.inf
        self.latest_pose = {}
        self.latest_joy = {}

    # 注释：记录不完整或不合法的输入样本，不补造可用姿态。
    def invalid_message(self, topic, error, now):
        self.invalid[topic + ":" + str(error).split(":")[0]] += 1
        if len(self.invalid_examples) < 8:
            self.invalid_examples.append({"at_s": now - self.started, "topic": topic, "error": str(error)[:300]})

    # 注释：采集真实关节反馈及到达时间。
    def joint_states(self, message):
        now = time.monotonic()
        self.arrivals["joint_states"].append(now)
        self.names_seen["joint_states"] = list(message.name)
        try:
            q = named_positions(message.name, message.position)
        except ValueError as error:
            self.invalid_message("joint_states", error, now)
            return
        self.latest_q, self.latest_q_time = q, now
        if self.anchor is None:
            self.anchor = q.copy()
        for name in ARM_NAMES:
            self.max_drift[name] = max(self.max_drift[name], abs(q[name] - self.anchor[name]))

    # 注释：采集 IK 发布的轨迹目标及到达时间。
    def trajectory(self, message):
        now = time.monotonic()
        self.arrivals["trajectory"].append(now)
        self.names_seen["trajectory"] = list(message.joint_names)
        try:
            if not message.points:
                raise ValueError("empty_points")
            # The original bridge uses points[-1], so compare that same target.
            q = named_positions(message.joint_names, message.points[-1].positions)
        except ValueError as error:
            self.invalid_message("trajectory", error, now)
            return
        self.valid_targets += 1
        fresh = self.latest_q is not None and 0 <= now - self.latest_q_time <= .2
        if self.first_target is None:
            self.first_target = q.copy()
            if fresh:
                self.first_target_delta = {name: q[name] - self.latest_q[name] for name in ARM_NAMES}
        if self.previous_target is not None:
            for name in ARM_NAMES:
                step = abs(q[name] - self.previous_target[name])
                self.max_step[name] = max(self.max_step[name] or 0.0, step)
        self.previous_target = q.copy()
        if fresh:
            self.compared += 1
            for name in ARM_NAMES:
                difference = abs(q[name] - self.latest_q[name])
                self.max_difference[name] = max(self.max_difference[name] or 0.0, difference)
        else:
            self.target_without_fresh_feedback += 1
        if not self.windows or now - self.windows[-1]["last_monotonic"] > .5:
            if len(self.windows) < 32:
                self.windows.append({"first_s": now - self.started, "last_s": now - self.started,
                                     "last_monotonic": now, "count": 0})
        if self.windows:
            self.windows[-1].update(last_s=now - self.started, last_monotonic=now)
            self.windows[-1]["count"] += 1
        if now - self.trace_last >= self.trace_interval and len(self.trace) < 60:
            self.trace.append({"at_s": now - self.started, "target_rad": [q[name] for name in ARM_NAMES],
                               "measured_rad": [self.latest_q[name] for name in ARM_NAMES] if fresh else None,
                               "feedback_age_ms": (now - self.latest_q_time) * 1000 if self.latest_q_time is not None else None})
            self.trace_last = now

    # 注释：采集对应设备的位姿及到达时间。
    def pose(self, key, message):
        now = time.monotonic()
        self.arrivals[key].append(now)
        p, q = message.pose.position, message.pose.orientation
        values = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        valid = all(finite(value) for value in values) and math.hypot(*values[3:]) > 0
        if not valid:
            self.invalid_message(key, "invalid_pose_or_quaternion", now)
        self.latest_pose[key] = {"received": now, "finite_nonzero_quaternion": valid}

    # 注释：处理 joy 消息并更新本适配器的观察缓存或计数。
    def joy(self, side, message):
        now = time.monotonic()
        self.arrivals[side + "_joy"].append(now)
        valid = len(message.buttons) > 1 and all(value in (0, 1) for value in message.buttons) and all(finite(value) for value in message.axes)
        if not valid:
            self.invalid_message(side + "_joy", "invalid_buttons_or_axes", now)
        self.latest_joy[side] = {"received": now, "valid": valid,
                                 "grip": bool(message.buttons[1]) if valid else None}

    # 注释：汇总本轮输入、目标和反馈数据及记录范围。
    def result(self, ended):
        quality = {}
        for key, row in {**self.latest_pose, **{side + "_joy": value for side, value in self.latest_joy.items()}}.items():
            quality[key] = {k: value for k, value in row.items() if k != "received"}
            quality[key].update(age_ms=(ended - row["received"]) * 1000,
                                fresh_within_300ms=0 <= ended - row["received"] <= .3)
        return {"sampling_result": "untested_no_trajectory" if self.valid_targets == 0 else "targets_observed_only",
                "physical_following_acceptance": "not_tested_by_this_readonly_sampler",
                "topic_arrivals": {key: arrival_summary(values, self.started, ended) for key, values in self.arrivals.items()},
                "valid_target_count": self.valid_targets, "targets_compared_with_fresh_feedback": self.compared,
                "targets_without_fresh_feedback": self.target_without_fresh_feedback,
                "feedback_max_age_for_comparison_ms": 200, "message_joint_names_seen": self.names_seen,
                "angular_joint_order": list(ARM_NAMES), "excluded_finger_joints_units": "metres; not compared with rotary joints",
                "invalid_message_counts": dict(self.invalid), "invalid_examples": self.invalid_examples,
                "first_target_delta_rad": self.first_target_delta,
                "per_joint_max_abs_target_minus_measured_rad": self.max_difference,
                "per_joint_max_step_between_valid_targets_rad": self.max_step,
                "actual_joint_drift_from_first_jointstate_rad": self.max_drift if self.anchor is not None else None,
                "actual_net_drift_rad": {name: self.latest_q[name] - self.anchor[name] for name in ARM_NAMES} if self.anchor is not None else None,
                "latest_input_quality": quality,
                "trajectory_windows_gap_threshold_s": .5,
                "trajectory_windows": [{k: v for k, v in window.items() if k != "last_monotonic"} for window in self.windows],
                "trace_max_points": 60, "sampled_target_trace": self.trace}


# 注释：记录开始采样时日志的位置，避免把旧日志算入本轮。
def log_start(path):
    try:
        info = path.stat()
        return {"inode": info.st_ino, "offset": info.st_size}
    except OSError as error:
        return {"error": str(error)}


# 注释：提取本轮新增日志片段。
def log_delta(path, before):
    if "error" in before:
        return {"available": False, **before}
    try:
        info = path.stat()
        rotated = info.st_ino != before["inode"] or info.st_size < before["offset"]
        offset = 0 if rotated else before["offset"]
        with path.open("rb") as source:
            source.seek(offset)
            content = source.read(2 * 1024 * 1024)
        text = content.decode("utf-8", "replace")
        lines = [line[:350] for line in text.splitlines() if re.search(r"bus=no-buffer|ENOBUFS|no buffer space", line, re.I)]
        return {"available": True, "start_offset": offset, "end_size": info.st_size, "rotated_or_truncated": rotated,
                "bytes_read": len(content), "read_capped_count_is_lower_bound": info.st_size - offset > len(content),
                "can_no_buffer_lines": len(lines), "examples": lines[:8]}
    except OSError as error:
        return {"available": False, "error": str(error)}


# 注释：该文件的显式执行入口；订阅输入、关节反馈和 IK 轨迹，结合 HTTP 状态计算频率、间隔和偏差。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--report", default="/logs/vr_original_sample_" + datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    parser.add_argument("--ros-log", default="/logs/ros.log")
    args = parser.parse_args()
    if not finite(args.duration) or not 0 < args.duration <= 600:
        parser.error("duration must be finite, >0 and <=600 seconds")
    destination = Path(args.report)
    if not destination.is_absolute() or destination.suffix != ".json" or not destination.resolve().is_relative_to(Path("/logs").resolve()):
        parser.error("report must be an absolute /logs/*.json path")
    if destination.exists():
        parser.error("report already exists; choose a new evidence filename")
    if os.environ.get("ROS_DOMAIN_ID") != "94" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("source the original clean ROS environment: domain 94, localhost only")
    if not os.environ.get("HOME"):
        os.environ["HOME"] = "/tmp/original-vr-home"
    if not os.environ.get("ROS_LOG_DIR"):
        os.environ["ROS_LOG_DIR"] = "/logs/ros"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState, Joy
    from trajectory_msgs.msg import JointTrajectory
    rclpy.init()
    node = Node("original_vr_readonly_sampler_" + str(os.getpid()),
                enable_rosout=False, start_parameter_services=False)
    # Humble can create an automatic parameter-events publisher even when
    # parameter services are disabled. Remove metadata publishers before the
    # sampling interval; this observer has no publishing endpoint or client.
    for publisher in tuple(node.publishers):
        node.destroy_publisher(publisher)
    if tuple(node.publishers) or tuple(node.clients):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("Readonly sampler unexpectedly owns a publisher/client")
    started = time.monotonic()
    started_utc = utc()
    recorder = Recorder(started, args.duration)
    node.create_subscription(JointTrajectory, "/joint_trajectory", recorder.trajectory, 10)
    node.create_subscription(JointState, "/joint_states", recorder.joint_states, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, "/xr/head/pose", lambda message: recorder.pose("head_pose", message), qos_profile_sensor_data)
    for side in ("left", "right"):
        node.create_subscription(PoseStamped, f"/xr/controller_{side}/pose", lambda message, side=side: recorder.pose(side + "_pose", message), qos_profile_sensor_data)
        node.create_subscription(Joy, f"/xr/controller_{side}/joy", lambda message, side=side: recorder.joy(side, message), qos_profile_sensor_data)
    poller = HttpSampler()
    before = log_start(Path(args.ros_log))
    poller.start()
    reason = "duration_completed"
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0, started + args.duration - time.monotonic())))
    except KeyboardInterrupt:
        reason = "operator_interrupted"
    finally:
        ended = time.monotonic()
        poller.finished.set()
        poller.join(timeout=1)
        result = recorder.result(ended)
        result.update(kind="original_vr_readonly_sample", started_utc=started_utc, finished_utc=utc(),
                      duration_requested_s=args.duration, duration_observed_s=ended - started, stop_reason=reason,
                      ros_domain_id=94, readonly=True, control_publishers_created=0,
                      active_publishers_during_sampling=0, service_clients_created=0,
                      can_sockets_created=0, enable_or_motion_commands_sent=0,
                      http_health={"samples": poller.samples, "errors": poller.errors,
                                   "unhealthy_samples": poller.unhealthy,
                                   "all_samples_healthy": poller.samples > 0 and poller.unhealthy == 0 and not poller.errors,
                                   "first": poller.first, "last": poller.latest, "issues": poller.health_issues},
                      ros_log_increment=log_delta(Path(args.ros_log), before))
        node.destroy_node()
        rclpy.shutdown()
        with destination.open("x", encoding="utf-8") as output:
            json.dump(result, output, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            output.write("\n")
        print(json.dumps({"report": str(destination), "sampling_result": result["sampling_result"],
                          "valid_targets": recorder.valid_targets, "duration_s": ended - started,
                          "invalid_messages": sum(recorder.invalid.values()), "read_only": True}), flush=True)


if __name__ == "__main__":
    main()
