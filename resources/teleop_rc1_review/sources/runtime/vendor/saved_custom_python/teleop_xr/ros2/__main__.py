from __future__ import annotations

import os
import threading
import json
import time
import sys
import asyncio
import types
from typing import Any, Optional, TYPE_CHECKING
from dataclasses import asdict
import cv2
import numpy as np
import tyro
from loguru import logger
from teleop_xr import Teleop
from teleop_xr.video_stream import ExternalVideoSource
from teleop_xr.config import TeleopSettings
from teleop_xr.ros2.cli import Ros2CLI
from teleop_xr.messages import XRState
from teleop_xr.events import EventProcessor, ButtonEvent
from teleop_xr.ik_utils import (
    ensure_ik_dependencies,
    list_available_robots as _default_list_available_robots,
)
import transforms3d as t3d

if TYPE_CHECKING:
    from teleop_xr.ik.robot import BaseRobot
    from teleop_xr.ik.controller import IKController


def list_available_robots() -> dict[str, str]:
    """Return the robots entry points exposed by the IK module."""
    return _default_list_available_robots()


def list_robots_or_exit() -> None:
    try:
        robots = list_available_robots()
        logger.info("Available robots (via entry points):")
        if not robots:
            logger.info("  None")
        for name, path in robots.items():
            logger.info(f"  {name}: {path}")
    except ImportError:
        logger.error(
            "IK dependencies not installed. Install with: pip install 'teleop-xr[ik]'"
        )
        sys.exit(1)
    return


def load_robot_class(robot_spec: str | None = None) -> type[BaseRobot]:
    from teleop_xr.ik.loader import load_robot_class as _load_robot_class

    return _load_robot_class(robot_spec)


ROS_AVAILABLE = False
ROS_IMPORT_ERROR: ImportError | None = None
ALLOW_MISSING_ROS = os.environ.get("TELEOP_XR_ALLOW_NO_ROS") == "1"

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Pose, PoseStamped, PoseArray, TransformStamped
    from sensor_msgs.msg import Joy, Image, CompressedImage, JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_msgs.msg import Float64, String
    from tf2_ros import TransformBroadcaster
    from builtin_interfaces.msg import Time, Duration

    try:
        from cv_bridge import CvBridge

        HAS_CV_BRIDGE = True
    except ImportError:
        HAS_CV_BRIDGE = False
except ImportError as exc:
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )
    ROS_IMPORT_ERROR.__cause__ = exc
    rclpy = types.ModuleType("rclpy")

    def _noop(*args, **kwargs):
        pass

    setattr(rclpy, "init", _noop)
    setattr(rclpy, "shutdown", _noop)
    setattr(rclpy, "spin", _noop)
    setattr(rclpy, "spin_once", _noop)
    setattr(rclpy, "ok", lambda: False)
    qos_ns = types.SimpleNamespace(
        QoSProfile=lambda *args, **kwargs: None,
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=0),
    )
    setattr(rclpy, "qos", qos_ns)
    node_module = types.ModuleType("rclpy.node")
    setattr(
        node_module,
        "Node",
        type(
            "FakeNode",
            (),
            {
                "get_logger": lambda self: types.SimpleNamespace(
                    info=_noop, error=_noop, warn=_noop
                )
            },
        ),
    )
    setattr(rclpy, "node", node_module)
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = node_module
    Node = Pose = PoseStamped = PoseArray = TransformStamped = Joy = Image = (
        CompressedImage
    ) = JointState = type("_Dummy", (), {"__init__": lambda self, **kwargs: None})
    JointTrajectory = JointTrajectoryPoint = type(
        "_DummyTraj", (), {"__init__": lambda self, **kwargs: None}
    )
    Float64 = String = type(
        "_DummyField", (), {"__init__": lambda self, **kwargs: None}
    )
    TransformBroadcaster = type(
        "_DummyBroadcaster", (), {"__init__": lambda self, **kwargs: None}
    )
    Time = Duration = type(
        "_DummyTime",
        (),
        {
            "__init__": lambda self, **kwargs: (
                setattr(self, "sec", kwargs.get("sec", 0))
                or setattr(self, "nanosec", kwargs.get("nanosec", 0))
            )
        },
    )
    HAS_CV_BRIDGE = False
    CvBridge = None
else:
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = None


def ensure_ros_available() -> None:
    if ROS_AVAILABLE or ALLOW_MISSING_ROS:
        return
    raise ROS_IMPORT_ERROR or ImportError(
        "ROS2 is not sourced. Please source ROS2 before running this script."
    )


class RosBridgeHandler:
    """
    Redirects Loguru messages to the ROS 2 logging system.
    """

    def __init__(self, node: Node):
        self.ros_logger = node.get_logger()

    def write(self, message):
        # Extract the log record information
        record = message.record
        level = record["level"].name
        msg = record["message"]

        # Map Loguru levels to ROS 2 logging methods
        if level == "DEBUG":
            self.ros_logger.debug(msg)
        elif level == "INFO":
            self.ros_logger.info(msg)
        elif level == "WARNING":
            self.ros_logger.warn(msg)
        elif level == "ERROR":
            self.ros_logger.error(msg)
        elif level == "CRITICAL":
            self.ros_logger.fatal(msg)


XR_HAND_JOINTS = [
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
]


def pose_dict_to_matrix(pose):
    if not pose or "position" not in pose or "orientation" not in pose:
        return None
    pos = pose["position"]
    quat = pose["orientation"]
    return t3d.affines.compose(
        [pos["x"], pos["y"], pos["z"]],
        t3d.quaternions.quat2mat([quat["w"], quat["x"], quat["y"], quat["z"]]),
        [1.0, 1.0, 1.0],
    )


def matrix_to_pose_msg(mat):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(mat[0, 3]),
        float(mat[1, 3]),
        float(mat[2, 3]),
    )
    quat = t3d.quaternions.mat2quat(mat[:3, :3])
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    )
    return pose


def pose_msg_to_dict(pose: Any) -> dict[str, dict[str, float]]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "w": float(pose.orientation.w),
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
        },
    }


def supported_absolute_frames(robot: "BaseRobot") -> list[str]:
    return [frame for frame in ("left", "right") if frame in robot.supported_frames]


def pose_array_to_absolute_targets(
    robot: "BaseRobot", msg: Any
) -> dict[str, dict[str, dict[str, float]]]:
    frames = supported_absolute_frames(robot)
    if not frames:
        raise ValueError("Robot does not support left or right end-effectors")
    if not msg.poses:
        raise ValueError("PoseArray command is empty")
    if len(frames) > 1 and len(msg.poses) < len(frames):
        raise ValueError(
            f"Expected {len(frames)} poses for frames {frames}, received {len(msg.poses)}"
        )

    targets: dict[str, dict[str, dict[str, float]]] = {}
    for frame, pose in zip(frames, msg.poses):
        targets[frame] = pose_msg_to_dict(pose)
    return targets


def ms_to_time(ms):
    sec = int(ms / 1000)
    nanosec = int((ms % 1000) * 1e6)
    return Time(sec=sec, nanosec=nanosec)


def get_urdf_from_topic(node, topic, timeout):
    ensure_ros_available()
    node.get_logger().info(f"Fetching URDF from topic {topic} (timeout={timeout}s)...")
    urdf_str = None
    event = threading.Event()

    def callback(msg: String):
        nonlocal urdf_str
        urdf_str = msg.data
        event.set()

    sub = node.create_subscription(
        String,
        topic,
        callback,
        rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        ),
    )

    # Wait for message
    start_time = time.time()
    while rclpy.ok() and not event.is_set() and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub)

    if urdf_str:
        node.get_logger().info(f"Successfully fetched URDF ({len(urdf_str)} bytes)")
    else:
        node.get_logger().warning("Failed to fetch URDF from topic")

    return urdf_str


class ROSImageToVideoSource:
    def __init__(self, node, topic, source: ExternalVideoSource):
        self.node = node
        self.source = source
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        if topic.endswith("/compressed"):
            self.sub = node.create_subscription(
                CompressedImage, topic, self.callback, 10
            )
        else:
            self.sub = node.create_subscription(Image, topic, self.callback, 10)

    def callback(self, msg):
        if isinstance(msg, CompressedImage):
            if not HAS_CV_BRIDGE:
                self.node.get_logger().error(
                    "cv_bridge not available, cannot decode CompressedImage"
                )
                return
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        elif isinstance(msg, Image):
            if HAS_CV_BRIDGE:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                # Basic ROS image to numpy conversion (OpenCV BGR compatible)
                # Assume rgb8 or bgr8 for now, but handle basic cases
                dtype = np.uint8
                if msg.encoding == "rgb8":
                    frame_rgb = np.frombuffer(msg.data, dtype=dtype).reshape(
                        msg.height, msg.width, 3
                    )
                    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                elif msg.encoding == "bgr8":
                    frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                        msg.height, msg.width, 3
                    )
                elif msg.encoding == "mono8":
                    frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                        msg.height, msg.width, 1
                    )
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    self.node.get_logger().warning(
                        f"Unsupported image encoding: {msg.encoding}"
                    )
                    return
        else:
            self.node.get_logger().error(f"Unknown message type: {type(msg)}")
            return

        self.source.put_frame(frame)


def build_joy(gamepad):
    if not gamepad:
        return None, None
    buttons_list = gamepad.get("buttons", [])
    buttons = [1 if b.get("pressed") else 0 for b in buttons_list]
    axes = [float(val) for val in gamepad.get("axes", [])] + [
        float(b.get("value", 0.0)) for b in buttons_list
    ]
    touched = [1 if b.get("touched") else 0 for b in buttons_list]
    return (buttons, axes), touched


class TeleopNode(Node):
    def __init__(self, cli: Ros2CLI):
        super().__init__("teleop")
        self.cli = cli


class IKWorker(threading.Thread):
    """The existing 1.3.5 IK worker, with one exclusive shared input admission."""

    FEEDBACK_MAX_AGE_S = 0.25
    UMI_MAX_JOINT_DELTA_RAD = 0.1
    UMI_MAX_POSITION_RESIDUAL_M = 0.002
    UMI_MAX_ORIENTATION_RESIDUAL_DEG = 2.0

    def __init__(self, controller: IKController, robot: BaseRobot,
                 publisher: "rclpy.publisher.Publisher", state_container: dict,
                 node: "rclpy.node.Node", teleop: Optional[Teleop] = None):
        super().__init__(daemon=True)
        self.controller = controller
        self.robot = robot
        self.publisher = publisher
        self.state_container = state_container
        self.node = node
        self.teleop = teleop
        self.teleop_loop = None
        self.latest_xr_state: Optional[XRState] = None
        self.new_state_event = threading.Event()
        self.running = True
        self._state_lock = threading.Lock()
        self._epoch = 0
        self._controller_epoch = -1
        self._xr_seq = 0
        self._consumed_seq = 0
        self._absolute_seq = 0
        # Restart never selects a motion source. The webpage selects explicitly.
        self._input_source = "none"
        self._umi_job = None
        self._umi_hold_q = None
        self._feedback_q = np.full(len(robot.actuated_joint_names), np.nan)
        self._feedback_times_ns = np.zeros(len(robot.actuated_joint_names), dtype=np.int64)
        self._feedback_seq = 0
        state_container["control_input_source"] = "none"
        state_container["deadman_active"] = False
        state_container["ee_absolute_targets"] = None
        state_container["motor_rearm"] = True
        self._timing_token = None
        from vr_output_runtime import VROutputRuntime
        self.vr_output = VROutputRuntime(self)

    def publish_smoothed_locked(self, q):
        """Existing publisher, called only under the input/source epoch lock."""
        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = list(self.robot.actuated_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start = Duration(sec=0, nanosec=int(1e7))
        msg.points = [point]
        self.publisher.publish(msg)
        self.state_container["q"] = np.asarray(q)

    def set_teleop_loop(self, loop):
        self.teleop_loop = loop

    def get_input_source(self):
        with self._state_lock:
            return self._input_source

    def _clear_ingresses_locked(self):
        self.vr_output.cancel_locked("ingresses_cleared")
        self._timing_token = None
        self.state_container["xr_state"] = None
        self.state_container["deadman_active"] = False
        self.state_container["ee_absolute_targets"] = None
        self.state_container["motor_rearm"] = True
        self.latest_xr_state = None
        self._consumed_seq = self._xr_seq
        self._absolute_seq += 1
        self._umi_job = None
        self._umi_hold_q = None
        self.state_container["active"] = False

    def select_input_source(self, source):
        if source not in ("none", "vr", "umi"):
            raise ValueError("Input source must be none, vr or umi")
        with self._state_lock:
            self._epoch += 1
            self._input_source = source
            self.state_container["control_input_source"] = source
            self._clear_ingresses_locked()
            self.state_container["umi_preview"] = None
            self.new_state_event.set()
            return self._epoch

    def record_joint_feedback(self, names, positions):
        """Keep measured positions separately from predicted IK state.

        Partial messages are accepted only for their named joints. A snapshot is
        usable only when every actuated joint has a recent finite measurement.
        """
        if len(names) != len(positions) or len(set(names)) != len(names):
            return False
        received = time.monotonic_ns()
        known = {name: i for i, name in enumerate(self.robot.actuated_joint_names)}
        values = []
        try:
            for name, value in zip(names, positions):
                if name in known:
                    value = float(value)
                    if not np.isfinite(value):
                        return False
                    values.append((known[name], value))
        except (ValueError, TypeError):
            return False
        if not values:
            return False
        with self._state_lock:
            current_q = np.array(self.state_container["q"], dtype=float, copy=True)
            for index, value in values:
                self._feedback_q[index] = value
                self._feedback_times_ns[index] = received
                current_q[index] = value
            self._feedback_seq += 1
            self.state_container["q"] = current_q
            self.vr_output.feedback_locked()
        return True

    def _feedback_snapshot_locked(self, max_age_s):
        if (not len(self._feedback_q) or not np.all(np.isfinite(self._feedback_q))
                or np.any(self._feedback_times_ns <= 0)):
            raise ValueError("Complete actual joint feedback is unavailable")
        received = int(np.min(self._feedback_times_ns))
        age = (time.monotonic_ns() - received) / 1e9
        if age < 0.0 or age > max_age_s:
            raise ValueError("Actual joint feedback is stale")
        return {"q": self._feedback_q.copy().tolist(),
                "joint_names": list(self.robot.actuated_joint_names),
                "received_monotonic_ns": received, "age_s": age,
                "feedback_seq": self._feedback_seq, "epoch": self._epoch,
                "frame_id": "robot_model_root"}

    @staticmethod
    def _fk_pose_dict(pose):
        # jaxlie.SE3 stores the rotation quaternion before translation.
        values = np.asarray(pose.wxyz_xyz, dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("Invalid end-effector FK")
        return {"position": dict(zip(("x", "y", "z"), map(float, values[4:]))),
                "orientation": dict(zip(("w", "x", "y", "z"), map(float, values[:4])))}

    def _fk_dict(self, q):
        return {side: self._fk_pose_dict(pose)
                for side, pose in self.robot.forward_kinematics(np.asarray(q)).items()
                if side in ("left", "right")}

    def get_feedback_snapshot(self, max_age_s=0.25):
        max_age_s = float(max_age_s)
        if not np.isfinite(max_age_s) or not 0 < max_age_s <= 1.0:
            raise ValueError("Feedback age must be in (0, 1] seconds")
        with self._state_lock:
            result = self._feedback_snapshot_locked(max_age_s)
        result["ee"] = self._fk_dict(result["q"])
        result["age_s"] = (time.monotonic_ns() - result["received_monotonic_ns"]) / 1e9
        if result["age_s"] > max_age_s:
            raise ValueError("Actual feedback expired while computing FK")
        return result

    def arm_umi_anchor(self, expected_epoch):
        """Create a fresh motion epoch after preview; never enable motors."""
        with self._state_lock:
            if self._input_source != "umi" or expected_epoch != self._epoch:
                raise ValueError("UMI source or anchor epoch changed")
            feedback = self._feedback_snapshot_locked(self.FEEDBACK_MAX_AGE_S)
            self._epoch += 1
            self._clear_ingresses_locked()
            self._umi_hold_q = np.array(feedback["q"], dtype=float)
            self.state_container["q"] = self._umi_hold_q.copy()
            self.state_container["motor_rearm"] = False
            self.state_container["umi_last_error"] = None
            self.state_container["umi_preview"] = None
            self.new_state_event.set()
            return self._epoch

    def _pause_umi_locked(self, reason):
        self._epoch += 1
        self._clear_ingresses_locked()
        self.state_container["umi_last_error"] = str(reason)
        self.new_state_event.set()
        return self._epoch

    def pause_umi(self, reason="UMI paused"):
        with self._state_lock:
            if self._input_source != "umi":
                return self._epoch
            return self._pause_umi_locked(reason)

    def _selected_arm_indices(self, side):
        names = list(self.robot.actuated_joint_names)
        required = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        if any(name not in names for name in required):
            raise ValueError("UMI single-arm admission requires seven named OpenArm joints")
        return [names.index(name) for name in required]

    def submit_umi_targets(self, targets, source_epoch, deadline_monotonic_ns,
                           side, preview=False, hold_q=None):
        if side not in ("left", "right") or set(targets) != {side}:
            raise ValueError("UMI must submit exactly one selected arm target")
        selected = self._selected_arm_indices(side)
        pose = targets[side]
        try:
            pos = [float(pose["position"][k]) for k in ("x", "y", "z")]
            quat = np.array([float(pose["orientation"][k]) for k in ("w", "x", "y", "z")])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Invalid UMI target pose") from exc
        if not np.all(np.isfinite(pos + quat.tolist())) or np.linalg.norm(quat) < 1e-8:
            raise ValueError("Non-finite UMI target pose")
        quat /= np.linalg.norm(quat)
        copied_targets = {side: {"position": dict(zip(("x", "y", "z"), pos)),
                                "orientation": dict(zip(("w", "x", "y", "z"), quat.tolist()))}}
        deadline = int(deadline_monotonic_ns)
        with self._state_lock:
            if self._input_source != "umi" or source_epoch != self._epoch:
                raise ValueError("UMI source epoch changed")
            if deadline <= time.monotonic_ns():
                raise ValueError("UMI target is expired")
            feedback = self._feedback_snapshot_locked(self.FEEDBACK_MAX_AGE_S)
            if not preview and (not self.state_container.get("motor_output_enabled", False)
                                or self.state_container.get("motor_rearm", True)
                                or self._umi_hold_q is None):
                raise ValueError("Manual IK permission and a fresh UMI anchor are required")
            anchor = self._umi_hold_q
            if anchor is None:
                anchor = np.array(feedback["q"], dtype=float)
            hold = np.asarray(anchor if hold_q is None else hold_q, dtype=float).copy()
            if hold.shape != self._feedback_q.shape or not np.all(np.isfinite(hold)):
                raise ValueError("Invalid UMI engagement hold positions")
            # A supplied snapshot may predate arm_umi_anchor slightly; reject a
            # substantial jump of either arm or gripper even in a trusted caller.
            frozen = np.ones(len(hold), dtype=bool)
            frozen[selected] = False
            if np.max(np.abs(hold[frozen] - np.asarray(feedback["q"])[frozen])) > self.UMI_MAX_JOINT_DELTA_RAD:
                raise ValueError("UMI hold positions differ from fresh actual feedback")
            self._absolute_seq += 1
            self._umi_job = {"targets": copied_targets, "epoch": source_epoch,
                             "deadline": deadline, "side": side, "selected": selected,
                             "preview": bool(preview), "hold_q": hold,
                             "sequence": self._absolute_seq}
            self.new_state_event.set()
            return self._absolute_seq

    def update_state(self, state: XRState, rebase_required=False, deadman_active=None, timing=None):
        if deadman_active is None:
            deadman_active = self.controller._check_deadman(state)
        deadman_active = bool(deadman_active)
        with self._state_lock:
            if self._input_source != "vr":
                return
            previous = bool(self.state_container.get("deadman_active", False))
            if deadman_active != previous or (rebase_required and (previous or deadman_active)):
                self._epoch += 1
                self.vr_output.cancel_locked("grip_or_pose_epoch_changed")
            if not deadman_active:
                self.vr_output.cancel_locked("grip_released")
            self._timing_token = timing
            self.state_container["xr_state"] = state
            self.state_container["deadman_active"] = deadman_active
            if deadman_active:
                self.state_container["ee_absolute_targets"] = None
            self.latest_xr_state = state
            self._xr_seq += 1
            self.new_state_event.set()

    def set_output_gate(self, enabled):
        # Same webpage service contract; never enable a motor here.
        with self._state_lock:
            self._epoch += 1
            self.state_container["motor_output_enabled"] = bool(enabled)
            self.state_container["motor_ik_reset"] = False
            self._clear_ingresses_locked()
            self.new_state_event.set()

    def update_absolute_targets(self, targets):
        with self._state_lock:
            if (self._input_source != "vr"
                    or not self.state_container.get("motor_output_enabled", False)
                    or self.state_container.get("motor_rearm", True)
                    or self.state_container.get("deadman_active", False)):
                return
            self._absolute_seq += 1
            self.state_container["ee_absolute_targets"] = targets
            self.new_state_event.set()

    @staticmethod
    def _pose_residual(actual, desired):
        p = np.array([actual["position"][k] - desired["position"][k] for k in ("x", "y", "z")])
        a = np.array([actual["orientation"][k] for k in ("w", "x", "y", "z")])
        b = np.array([desired["orientation"][k] for k in ("w", "x", "y", "z")])
        cosine = np.clip(abs(float(np.dot(a, b))) / (np.linalg.norm(a) * np.linalg.norm(b)), 0.0, 1.0)
        return float(np.linalg.norm(p)), float(np.degrees(2.0 * np.arccos(cosine)))

    def _report_umi_result_locked(self, job, accepted, reason, **details):
        self.state_container["umi_result"] = {
            "accepted": bool(accepted), "reason": str(reason),
            "source_epoch": job["epoch"], "sequence": job["sequence"],
            "preview": job["preview"], **details}

    def _run_umi_job(self, job):
        from teleop_xr.ik.control_mode import ControlMode
        with self._state_lock:
            if (self._input_source != "umi" or job["epoch"] != self._epoch
                    or job["sequence"] != self._absolute_seq):
                self._report_umi_result_locked(job, False, "source_epoch_or_sequence_changed")
                return
            if time.monotonic_ns() >= job["deadline"]:
                self._report_umi_result_locked(job, False, "target_expired_before_ik")
                self._pause_umi_locked("UMI target expired before IK")
                return
            feedback = self._feedback_snapshot_locked(self.FEEDBACK_MAX_AGE_S)
        current = np.asarray(feedback["q"], dtype=float)
        if self.controller.get_mode() != ControlMode.EE_ABSOLUTE:
            self.controller.set_mode(ControlMode.EE_ABSOLUTE)
        started = time.perf_counter()
        raw = np.asarray(self.controller.submit_ee_absolute_targets(job["targets"], current), dtype=float)
        if raw.shape != current.shape or not np.all(np.isfinite(raw)):
            raise ValueError("IK returned an invalid UMI configuration")
        # Standard bimanual IK may adjust unselected joints through secondary
        # costs. Freeze those joints and both grippers at the engagement anchor.
        output = job["hold_q"].copy()
        output[job["selected"]] = raw[job["selected"]]
        fk = self._fk_dict(output)
        position_error, orientation_error = self._pose_residual(fk[job["side"]], job["targets"][job["side"]])
        record = {"sequence": job["sequence"], "epoch": job["epoch"],
                  "side": job["side"], "preview": job["preview"],
                  "q": output.tolist(), "joint_names": list(self.robot.actuated_joint_names),
                  "ee": fk, "position_residual_m": position_error,
                  "orientation_residual_deg": orientation_error,
                  "solve_time_s": time.perf_counter() - started,
                  "completed_monotonic_ns": time.monotonic_ns(), "published": False}
        with self._state_lock:
            if (self._input_source != "umi" or job["epoch"] != self._epoch
                    or job["sequence"] != self._absolute_seq):
                self._report_umi_result_locked(job, False, "source_epoch_or_sequence_changed")
                return
            if time.monotonic_ns() >= job["deadline"]:
                self._report_umi_result_locked(job, False, "target_expired_during_ik")
                self._pause_umi_locked("UMI target expired during IK")
                return
            latest = self._feedback_snapshot_locked(self.FEEDBACK_MAX_AGE_S)
            delta = float(np.max(np.abs(output[job["selected"]] - np.asarray(latest["q"])[job["selected"]])))
            record["max_selected_joint_delta_rad"] = delta
            record["admissible"] = bool(delta <= self.UMI_MAX_JOINT_DELTA_RAD
                                         and position_error <= self.UMI_MAX_POSITION_RESIDUAL_M
                                         and orientation_error <= self.UMI_MAX_ORIENTATION_RESIDUAL_DEG)
            self.state_container["umi_preview"] = dict(record,
                accepted=record["admissible"], source_epoch=job["epoch"])
            self.state_container["solve_time"] = record["solve_time_s"]
            if job["preview"]:
                self._report_umi_result_locked(job, record["admissible"],
                    "preview_admissible" if record["admissible"] else "preview_exceeds_test_limits",
                    **record)
                return
            if not record["admissible"]:
                self._report_umi_result_locked(job, False, "ik_exceeds_test_limits", **record)
                self._pause_umi_locked("UMI IK result exceeds the initial single-arm test limits")
                return
            if (not self.state_container.get("motor_output_enabled", False)
                    or self.state_container.get("motor_rearm", True)):
                self._report_umi_result_locked(job, False, "manual_gate_or_anchor_closed", **record)
                return
            msg = JointTrajectory()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = list(self.robot.actuated_joint_names)
            point = JointTrajectoryPoint()
            point.positions = output.tolist()
            point.time_from_start = Duration(sec=0, nanosec=int(1e7))
            msg.points = [point]
            self.publisher.publish(msg)
            self.state_container["q"] = output
            record["published"] = True
            self.state_container["umi_last_admission"] = record
            self._report_umi_result_locked(job, True, "published", **record)

    def run(self):
        from teleop_xr.ik.control_mode import ControlMode

        while self.running:
            if not self.new_state_event.wait(timeout=0.1):
                continue
            with self._state_lock:
                self.new_state_event.clear()
                source = self._input_source
                epoch = self._epoch
                umi_job = self._umi_job if source == "umi" else None
                self._umi_job = None
                state = self.latest_xr_state
                fresh = self._xr_seq != self._consumed_seq
                skipped_states = max(0, self._xr_seq-self._consumed_seq-1)
                self._consumed_seq = self._xr_seq
                timing = dict(self._timing_token or {}, worker_sequence=self._xr_seq,
                              coalesced_input_count=skipped_states)
                deadman_active = bool(self.state_container.get("deadman_active", False))
                targets = self.state_container.get("ee_absolute_targets")
                absolute_seq = self._absolute_seq
                if not deadman_active:
                    self.state_container["ee_absolute_targets"] = None
                permitted = bool(self.state_container.get("motor_output_enabled", False))
                if source == "vr" and self.state_container.get("motor_rearm", True):
                    if fresh and not deadman_active:
                        self.state_container["motor_rearm"] = False
                    permitted = False
            if epoch != self._controller_epoch:
                self.controller.reset()
                self._controller_epoch = epoch
                self.state_container["active"] = False
            if source == "umi":
                if umi_job is not None:
                    try:
                        self._run_umi_job(umi_job)
                    except Exception as exc:
                        with self._state_lock:
                            self._report_umi_result_locked(umi_job, False, str(exc))
                            if self._input_source == "umi" and umi_job["epoch"] == self._epoch:
                                self._pause_umi_locked(str(exc))
                        self.node.get_logger().error(f"Error in UMI IK admission: {exc}")
                continue
            if source != "vr" or not permitted or (not fresh and targets is None):
                continue
            if state is None and targets is None:
                continue
            try:
                with self._state_lock:
                    current_config = np.array(self.state_container["q"], copy=True)
                was_active = self.controller.active
                t0 = time.perf_counter()
                solve_started_ns = time.monotonic_ns()
                self.vr_output.trace.emit("ik_start", **timing, epoch=epoch,
                    queue_wait_ms=(solve_started_ns-timing.get("callback_ns", solve_started_ns))/1e6)
                if deadman_active:
                    if self.controller.get_mode() != ControlMode.TELEOP:
                        self.controller.set_mode(ControlMode.TELEOP)
                    if state is None:
                        self.state_container["solve_time"] = 0.0
                        self.state_container["active"] = self.controller.active
                        continue
                    new_config = np.array(self.controller.step(state, current_config))
                else:
                    if self.controller.get_mode() != ControlMode.EE_ABSOLUTE:
                        self.controller.set_mode(ControlMode.EE_ABSOLUTE)
                    if targets is None:
                        self.state_container["solve_time"] = 0.0
                        self.state_container["active"] = self.controller.active
                        continue
                    new_config = np.array(self.controller.submit_ee_absolute_targets(targets, current_config))
                self.state_container["solve_time"] = time.perf_counter() - t0
                self.state_container["active"] = self.controller.active
                if not was_active and self.controller.active:
                    self.node.get_logger().info("IK engagement started")
                with self._state_lock:
                    if (self._input_source != "vr" or epoch != self._epoch
                            or not self.state_container.get("motor_output_enabled", False)
                            or self.state_container.get("motor_rearm", True)
                            or deadman_active != bool(self.state_container.get("deadman_active", False))
                            or (not deadman_active and absolute_seq != self._absolute_seq)):
                        self.vr_output.trace.emit("ik_discard", **timing, epoch=epoch,
                            reason="source_gate_or_epoch_changed_during_solve")
                        continue
                    self.vr_output.trace.emit("ik_candidate", **timing, epoch=epoch,
                        solve_ms=self.state_container["solve_time"]*1000,
                        active=bool(self.controller.active),
                        joint_names=list(self.robot.actuated_joint_names), q=new_config.tolist())
                    takeover = False
                    if deadman_active:
                        takeover = self.vr_output.offer_locked(new_config, current_config,
                            epoch, timing, bool(self.controller.active))
                    else:
                        self.vr_output.cancel_locked("absolute_or_reset_path")
                    if not takeover and not np.array_equal(new_config, current_config):
                        msg = JointTrajectory()
                        msg.header.stamp = self.node.get_clock().now().to_msg()
                        msg.joint_names = self.robot.actuated_joint_names
                        point = JointTrajectoryPoint()
                        point.positions = [float(val) for val in new_config]
                        point.time_from_start = Duration(sec=0, nanosec=int(1e7))
                        msg.points = [point]
                        self.state_container["q"] = new_config
                        self.publisher.publish(msg)
                        self.vr_output.trace.emit("raw_publish", **timing, epoch=epoch,
                            q=new_config.tolist(), mode=self.vr_output.mode)
            except Exception as exc:
                self.node.get_logger().error(f"Error in IK Worker: {exc}")

def main():
    cli = tyro.cli(Ros2CLI)

    # Configure JAX only if in IK mode
    if cli.mode == "ik":
        ensure_ik_dependencies()

    if cli.list_robots:
        list_robots_or_exit()
        return

    # 1. Initialize ROS2
    ensure_ros_available()
    rclpy.init(args=["--ros-args"] + cli.ros_args)
    node = TeleopNode(cli)

    # 2. Remove Loguru's default handler
    logger.remove()

    # 3. Add the ROS 2 bridge as a sink
    bridge = RosBridgeHandler(node)
    logger.add(bridge.write, format="{message}", level="INFO")

    # 4. Add back a styled console sink for local output
    logger.add(
        sys.stderr,
        colorize=True,
        format="<green>{time}</green> <level>{message}</level>",
        level="INFO",
    )

    # --- Mode Setup ---
    robot = None
    solver = None
    controller = None
    ik_worker = None
    from teleop_diagnostics import Diagnostics
    diagnostics = Diagnostics()
    state_container: dict[str, Any] = {
        "active": False,
        "solve_time": 0.0,
        "xr_state": None,
    }

    if node.cli.mode == "ik":
        # Import IK modules only when needed
        try:
            from teleop_xr.ik.solver import PyrokiSolver
            from teleop_xr.ik.controller import IKController
        except ImportError as e:
            node.get_logger().error(
                f"IK dependencies not installed: {e}. Install with: pip install 'teleop-xr[ik]'"
            )
            sys.exit(1)

        robot_cls = load_robot_class(node.cli.robot_class or None)
        robot_args = node.cli.robot_args_dict

        urdf_string = None
        if not node.cli.no_urdf_topic:
            urdf_string = get_urdf_from_topic(
                node, node.cli.urdf_topic, node.cli.urdf_timeout
            )

        if urdf_string:
            robot_args["urdf_string"] = urdf_string

        node.get_logger().info(
            f"Initializing {robot_cls.__name__} with args: {robot_args}"
        )
        robot = robot_cls(**robot_args)
        solver = PyrokiSolver(robot)
        controller = IKController(robot, solver)
        controller.set_mode("ee_absolute")
        state_container["q"] = np.array(robot.get_default_config())
        state_container["deadman_active"] = False
        state_container["ee_absolute_targets"] = None


        # ROS2 Pub/Sub for IK
        ik_pub = node.create_publisher(JointTrajectory, node.cli.output_topic, 10)

        def joint_state_callback(msg: JointState):
            diagnostics.record("feedback")
            if ik_worker is None or not ik_worker.record_joint_feedback(msg.name, msg.position):
                return
            current_q = state_container["q"].copy()
            actuated_names = robot.actuated_joint_names

            if ik_worker and ik_worker.teleop and ik_worker.teleop_loop:
                joint_dict = dict(
                    zip(actuated_names, [float(val) for val in current_q])
                )
                asyncio.run_coroutine_threadsafe(
                    ik_worker.teleop.publish_joint_state(joint_dict),
                    ik_worker.teleop_loop,
                )

        node.create_subscription(JointState, "/joint_states", joint_state_callback, 10)

        def ee_absolute_callback(msg: PoseArray):
            if ik_worker is None or ik_worker.get_input_source() != "vr":
                return
            try:
                if state_container.get("deadman_active", False):
                    return
                targets = pose_array_to_absolute_targets(robot, msg)
                if ik_worker is not None:
                    ik_worker.update_absolute_targets(targets)
            except ValueError as exc:
                node.get_logger().warning(f"ee_absolute command rejected: {exc}")

        node.create_subscription(
            PoseArray, node.cli.ee_absolute_topic, ee_absolute_callback, 10
        )
        state_container["diagnostics"] = diagnostics
        ik_worker = IKWorker(controller, robot, ik_pub, state_container, node)
        ik_worker.start()
    else:
        node.get_logger().info("ROS2 Node starting in Teleop mode")

    # Merge topics
    topics = {}
    if node.cli.head_topic:
        topics["head"] = node.cli.head_topic
    if node.cli.wrist_left_topic:
        topics["wrist_left"] = node.cli.wrist_left_topic
    if node.cli.wrist_right_topic:
        topics["wrist_right"] = node.cli.wrist_right_topic
    topics.update(node.cli.extra_streams)

    video_sources = {}
    for key, topic in topics.items():
        source = ExternalVideoSource()
        ROSImageToVideoSource(node, topic, source)
        video_sources[key] = source

    # Create config dict for camera views
    camera_views = {k: {"device": topic} for k, topic in topics.items()}

    robot_vis = None
    if node.cli.mode == "ik" and robot:
        robot_vis = robot.get_vis_config()

    settings = TeleopSettings(
        host=node.cli.host,
        port=node.cli.port,
        input_mode=node.cli.input_mode,
        camera_views=camera_views,
        robot_vis=robot_vis,
    )

    teleop = Teleop(
        settings=settings,
        video_sources=video_sources,
    )

    from std_srvs.srv import SetBool as MotorGateSetBool

    def motor_ik_gate(request, response):
        if ik_worker is not None:
            ik_worker.set_output_gate(bool(request.data))
        else:
            state_container["motor_output_enabled"] = bool(request.data)
            state_container["motor_ik_reset"] = False
            state_container["motor_rearm"] = True
            state_container["ee_absolute_targets"] = None
        response.success = True
        response.message = "IK gate updated; old target cleared"
        return response

    node.create_service(MotorGateSetBool, "/motor_toggle/ik_gate", motor_ik_gate)

    if ik_worker:
        ik_worker.teleop = teleop

        # Shared input arbitration reuses this worker, solver and ROS publisher.
        from umi_motion_runtime import UmiMotionRuntime
        from umi_motion_web import install as install_umi_motion
        umi_motion_runtime = UmiMotionRuntime(ik_worker, robot, state_container, node)
        install_umi_motion(teleop.app, umi_motion_runtime)
        from teleop_capture import install as install_teleop_capture
        capture_manager = install_teleop_capture(teleop.app, node, teleop, ik_worker, umi_motion_runtime)
        # Capture callbacks run in the trace writer, never in the 100Hz lock.
        ik_worker.vr_output.trace.set_observer(capture_manager.observe)
        from vr_output_runtime import install as install_vr_output
        install_vr_output(teleop.app, ik_worker.vr_output)
        ik_worker.vr_output.attach_command_observer(node)

    from fastapi.routing import APIRoute
    # Insert before StaticFiles' catch-all route; this endpoint is read-only.
    def teleop_diagnostics():
        result = diagnostics.snapshot(state_container)
        result["input_guard"] = teleop.get_input_diagnostics()
        result["control_epoch"] = ik_worker._epoch if ik_worker else None
        result["vr_output"] = ik_worker.vr_output.snapshot() if ik_worker else None
        result["ik_backend"] = {
            "package": "teleop-xr", "version": "1.3.5",
            "solver": "upstream", "controller": "upstream",
            "worker": "upstream_ik_with_manual_admission_and_optional_output_stage",
            "solve_time_s": state_container.get("solve_time"),
            "warmup_complete": bool(solver and solver.warmup_complete),
            "warmup_error": solver.warmup_error if solver else None,
            "warmed_target_patterns": solver.warmed_target_patterns if solver else (),
            "worker_alive": bool(ik_worker and ik_worker.is_alive()),
            "active": state_container.get("active"),
            "output_enabled": state_container.get("motor_output_enabled", False),
            "rearm_required": state_container.get("motor_rearm", True),
            "legacy_worker_timing_metrics_available": False,
        }
        return result
    teleop.app.router.routes.insert(0, APIRoute(
        "/api/teleop/diagnostics", teleop_diagnostics, methods=["GET"]))

    broadcaster = TransformBroadcaster(node)

    publishers = {}

    def get_publisher(msg_type, topic):
        if topic not in publishers:
            publishers[topic] = node.create_publisher(msg_type, topic, 1)
        return publishers[topic]

    def publish_pose(topic, pose_dict, stamp, child_frame_id, tf_stamp):
        if not pose_dict:
            return
        mat = pose_dict_to_matrix(pose_dict)
        if mat is None:
            return
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = node.cli.frame_id
        msg.pose = matrix_to_pose_msg(mat)
        get_publisher(PoseStamped, topic).publish(msg)

        tf = TransformStamped()
        tf.header.stamp = tf_stamp  # Use PC timestamp for TF
        tf.header.frame_id = node.cli.frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        broadcaster.sendTransform(tf)

    def publish_joy(topic, joy_data, stamp):
        buttons, axes = joy_data
        msg = Joy()
        msg.header.stamp = stamp
        msg.header.frame_id = node.cli.frame_id
        msg.buttons = buttons
        msg.axes = axes
        get_publisher(Joy, topic).publish(msg)

    def publish_hand(device, stamp):
        handed = device.get("handedness", "none")
        joints_dict = device.get("joints", {})
        if not joints_dict:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = node.cli.frame_id

        for joint_name in XR_HAND_JOINTS:
            joint_pose_dict = joints_dict.get(joint_name)
            mat = pose_dict_to_matrix(joint_pose_dict)
            if mat is None:
                continue
            pose_msg = matrix_to_pose_msg(mat)
            pose_array.poses.append(pose_msg)

            if node.cli.publish_hand_tf:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = node.cli.frame_id
                tf.child_frame_id = f"xr/hand_{handed}/{joint_name}"
                tf.transform.translation.x = pose_msg.position.x
                tf.transform.translation.y = pose_msg.position.y
                tf.transform.translation.z = pose_msg.position.z
                tf.transform.rotation = pose_msg.orientation
                broadcaster.sendTransform(tf)

        get_publisher(PoseArray, f"xr/hand_{handed}/joints").publish(pose_array)

    event_processor = EventProcessor(cli.event_settings())

    def publish_button_event(event: ButtonEvent):
        try:
            msg = String()
            msg.data = json.dumps(asdict(event))
            get_publisher(String, f"xr/events/{event.type.value}").publish(msg)
        except Exception as exc:
            node.get_logger().warning(f"publish_button_event error: {exc}")

    event_processor.on_button_down(callback=publish_button_event)
    event_processor.on_button_up(callback=publish_button_event)
    event_processor.on_double_press(callback=publish_button_event)
    event_processor.on_long_press(callback=publish_button_event)

    primary_button_previous = {"left": False, "right": False}

    def teleop_xr_state_callback(_pose, xr_state):
        # Observe valid and missing grips even when no motion source is selected.
        timing = ik_worker.vr_output.note_input(xr_state or {}) if ik_worker else None
        # In non-VR modes preserve only the original A/X Joy reset path.
        # Empty axes suppress the original trigger-to-gripper branch, and no
        # VR target, grip or other button reaches the shared robot bridge.
        if ik_worker is not None and ik_worker.get_input_source() != "vr":
            stamp = node.get_clock().now().to_msg()
            for device in (xr_state or {}).get("devices", []):
                side = device.get("handedness")
                if device.get("role") != "controller" or side not in ("left", "right"):
                    continue
                joy_payload, _ = build_joy(device.get("gamepad"))
                if not joy_payload:
                    continue
                buttons, _axes = joy_payload
                primary = bool(len(buttons) > 4 and buttons[4])
                if primary and not primary_button_previous[side]:
                    ik_worker.pause_umi(f"{side} A/X reset requested")
                primary_button_previous[side] = primary
                reset_buttons = [0] * max(6, len(buttons))
                reset_buttons[4] = int(primary)
                publish_joy(f"xr/controller_{side}/joy", (reset_buttons, []), stamp)
            return
        try:
            # Capture the current event loop for ik_worker if not already set
            if ik_worker and ik_worker.teleop_loop is None:
                try:
                    loop = asyncio.get_running_loop()
                    ik_worker.set_teleop_loop(loop)
                except RuntimeError:
                    pass

            # Process events
            event_processor.process(_pose, xr_state)

            # Parse state for IK and container
            xr_data = xr_state.get("data", xr_state)
            state = XRState.model_validate(xr_data)
            diagnostics.record("xr")
            deadman = controller._check_deadman(state) if controller is not None else False
            if ik_worker is not None:
                ik_worker.update_state(state,
                    rebase_required=bool(xr_data.get("pose_guard_rebase_required", False)),
                    deadman_active=deadman, timing=timing)
            else:
                state_container["xr_state"] = state
                state_container["deadman_active"] = deadman

            ms = xr_state.get("timestamp_unix_ms") if xr_state else None
            stamp = (
                ms_to_time(ms) if ms is not None else node.get_clock().now().to_msg()
            )
            # Use PC timestamp for TF to avoid lag from headset/PC time difference
            tf_stamp = node.get_clock().now().to_msg()

            # Publish fetch latency if available
            fetch_latency = xr_state.get("fetch_latency_ms") if xr_state else None
            if fetch_latency is not None:
                latency_msg = Float64()
                latency_msg.data = float(fetch_latency)
                get_publisher(Float64, "xr/fetch_latency_ms").publish(latency_msg)

            for device in xr_state.get("devices", []) if xr_state else []:
                role = device.get("role")
                handed = device.get("handedness", "none")

                if role == "head":
                    publish_pose(
                        "xr/head/pose",
                        device.get("pose"),
                        stamp,
                        "xr/head",
                        tf_stamp,
                    )

                if role == "controller":
                    publish_pose(
                        f"xr/controller_{handed}/pose",
                        device.get("gripPose"),
                        stamp,
                        f"xr/controller_{handed}/pose",
                        tf_stamp,
                    )

                    joy_payload, touched = build_joy(device.get("gamepad"))
                    if joy_payload:
                        publish_joy(f"xr/controller_{handed}/joy", joy_payload, stamp)
                    if touched:
                        publish_joy(
                            f"xr/controller_{handed}/joy_touched",
                            (touched, []),
                            stamp,
                        )

                if role == "hand":
                    publish_hand(device, stamp)
        except Exception as exc:
            node.get_logger().warning(f"xr_state callback error: {exc}")

    teleop.subscribe(teleop_xr_state_callback)

    # Start ROS spin thread
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    teleop.run()


if __name__ == "__main__":
    main()
