"""Bind each arm joint to the exact loaded URDF, after mirror/mount transforms.

These are command-envelope checks, not a measurement of mechanical stops or
certification of the motor's acceleration/jerk capability.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ARM_NAMES = tuple(f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8))


def named_limits(profile):
    joints = profile.get("joints")
    if not isinstance(joints, dict) or set(joints) != set(ARM_NAMES):
        raise ValueError("Exactly 14 fully named arm limits are required; shared mirrored limits are not accepted")
    for name, limits in joints.items():
        for key in ("min_position", "max_position", "max_velocity", "max_acceleration", "max_jerk"):
            value = limits.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid {name} {key}")
        if limits["min_position"] >= limits["max_position"]:
            raise ValueError("Invalid position range: "+name)
        if any(limits[k] <= 0 for k in ("max_velocity", "max_acceleration", "max_jerk")):
            raise ValueError("Positive dynamic limits required: "+name)
    return joints


def verify_loaded_model(robot, profile):
    joints = named_limits(profile)
    if any(robot.actuated_joint_names.count(name) != 1 for name in ARM_NAMES):
        raise ValueError("Worker does not have exactly one of each required arm joint")
    if not getattr(robot, "urdf_path", None):
        raise ValueError("Loaded robot URDF path is unavailable")
    path = Path(robot.urdf_path)
    raw = path.read_bytes()
    expected = profile["model_urdf_sha256"]
    if hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError("Loaded robot URDF differs from the reviewed limit profile")
    tree = ET.fromstring(raw)
    elements = {j.get("name"): j for j in tree.findall("joint")}
    rows = []
    for name in ARM_NAMES:
        element = elements[name]
        if element.get("type") != "revolute":
            raise ValueError("Expected revolute arm joint: "+name)
        limit = element.find("limit")
        if limit is None:
            raise ValueError("Missing URDF limit: "+name)
        lower, upper, velocity = (float(limit.get(k)) for k in ("lower", "upper", "velocity"))
        values = joints[name]
        if (abs(lower-values["min_position"]) > 1e-9
                or abs(upper-values["max_position"]) > 1e-9
                or not 0 < values["max_velocity"] <= velocity):
            raise ValueError("Named profile disagrees with loaded URDF: "+name)
        rows.append(dict(name=name, urdf_lower=lower, urdf_upper=upper,
                         urdf_velocity=velocity, **values))
    return rows
