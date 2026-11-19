"""Validate the 14-axis POS_VEL configuration and collect a read-only migration inventory."""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import struct

from openarm_driver import DriverError, LIMITS, OpenArmDriver, ROOT, motor_type
from openarm_joint_mapping import finite, load_mapping


DEFAULT_CONFIG = ROOT / "configs/openarm_position_mode.json"
MAPPING_PATH = ROOT / "configs/openarm_joint_mapping.json"
SIDES = ("left", "right")
ARM_IDS = set(range(1, 8))
REGISTERS = {
    "OC_Value": 3, "ACC": 4, "DEC": 5, "MAX_SPD": 6,
    "MST_ID": 7, "ESC_ID": 8, "TIMEOUT": 9, "CTRL_MODE": 10,
    "hw_ver": 13, "sw_ver": 14, "sub_ver": 36,
    "PMAX": 21, "VMAX": 22, "TMAX": 23,
    "I_BW": 24, "KP_ASR": 25, "KI_ASR": 26, "KP_APR": 27, "KI_APR": 28,
}


def load_config(path=DEFAULT_CONFIG):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    version, mappings = load_mapping(MAPPING_PATH, hardware=True)
    if (type(config["schema_version"]) is not int or config["schema_version"] != 1
            or config["mapping_version"] != version or config["target_control_mode"] != "POS_VEL"
            or type(config["ctrl_mode"]) is not int or config["ctrl_mode"] != 2):
        raise ValueError("Expected POS_VEL mode 2 with the current joint mapping")
    arms = config["arms"]
    if set(arms) != set(SIDES):
        raise ValueError("Expected left and right arm configurations")
    channels = []
    for side in SIDES:
        ids = arms[side]["motor_ids"]
        channel = arms[side]["channel"]
        if len(ids) != 7 or any(type(motor_id) is not int for motor_id in ids) or set(ids) != ARM_IDS:
            raise ValueError(f"{side}: expected J1–J7 exactly once")
        if type(channel) is not int or not 0 < channel <= 0xffff:
            raise ValueError(f"{side}: invalid PCAN channel")
        channels.append(channel)
    if len(set(channels)) != len(SIDES):
        raise ValueError("PCAN channels must be different")
    return config, mappings


def encode_target(mapping, position_rad, speed_rad_s):
    """Encode a POS_VEL target using a positive upper speed in model coordinates."""
    position_rad, speed_rad_s = finite(position_rad), finite(speed_rad_s)
    if not mapping.limits[0] <= position_rad <= mapping.limits[1]:
        raise ValueError("Position outside mapped joint range")
    position = mapping.to_motor(position_rad)
    speed = abs(mapping.velocity_to_motor(speed_rad_s))
    position_range, speed_range, _ = LIMITS[motor_type(mapping.motor_id)]
    if abs(position) > position_range or speed_rad_s <= 0 or speed > speed_range:
        raise ValueError("Position or speed outside POS_VEL encoding range")
    payload = struct.pack("<ff", position, speed)
    encoded_position, encoded_speed = struct.unpack("<ff", payload)
    if not mapping.limits[0] <= mapping.to_model(encoded_position) <= mapping.limits[1]:
        raise ValueError("Encoded position outside mapped joint range")
    if encoded_speed == 0:
        raise ValueError("Speed rounds to zero in POS_VEL encoding")
    return mapping.motor_id + 0x100, payload


def check_config(config, mappings):
    checked = {}
    for side in SIDES:
        arm = config["arms"][side]
        target_ids = []
        for motor_id in arm["motor_ids"]:
            mapping = mappings[side, motor_id]
            midpoint = sum(mapping.limits) / 2
            target_id, _ = encode_target(mapping, midpoint, .05)
            target_ids.append(hex(target_id))
        checked[side] = {"channel": hex(arm["channel"]), "target_can_ids": target_ids}
    return {"target_control_mode": config["target_control_mode"], "ctrl_mode": config["ctrl_mode"],
            "mapping_version": config["mapping_version"], "arms": checked}


def _fully_disabled(snapshot):
    motors = snapshot["motors"]
    return (snapshot["bus_status"] == "0x0" and len(motors) == 7
            and {m["motor_id"] for m in motors} == ARM_IDS
            and all(m.get("online") and m.get("state") == 0 for m in motors))


def read_inventory(config, *, driver_factory=OpenArmDriver):
    """Read both arms with the existing exclusive PCAN leases and no motor writes."""
    axes = []
    with ExitStack() as stack:
        drivers = {
            side: stack.enter_context(driver_factory(side, config["arms"][side]["channel"], .1))
            for side in SIDES
        }
        before = {side: driver.snapshot() for side, driver in drivers.items()}
        if all(map(_fully_disabled, before.values())):
            for side, driver in drivers.items():
                for motor_id in config["arms"][side]["motor_ids"]:
                    parameters, errors = {}, {}
                    for name, register in REGISTERS.items():
                        try:
                            value, raw = driver.read_parameter(motor_id, register, include_raw=True)
                            parameters[name] = {"register": register, "value": value, "raw_reply": raw}
                        except (DriverError, OSError) as exc:
                            errors[name] = str(exc)
                    axes.append({"side": side, "channel": config["arms"][side]["channel"],
                                 "motor_id": motor_id, "parameters": parameters, "read_errors": errors})
        after = {side: driver.snapshot() for side, driver in drivers.items()}
    complete = (len(axes) == 14 and all(not axis["read_errors"] for axis in axes)
                and all(map(_fully_disabled, after.values())))
    target_matches = sum(axis["parameters"].get("CTRL_MODE", {}).get("value") == config["ctrl_mode"]
                         for axis in axes)
    return {"kind": "position_mode_read_only_inventory", "timestamp": datetime.now(timezone.utc).isoformat(),
            "target_control_mode": config["target_control_mode"], "target_ctrl_mode": config["ctrl_mode"],
            "mapping_version": config["mapping_version"], "before": before, "after": after, "axes": axes,
            "complete": complete, "axes_matching_target": target_matches}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--inventory", type=Path, metavar="OUTPUT_JSON",
                        help="Read actual motor parameters over PCAN and save a new backup")
    args = parser.parse_args(argv)
    try:
        config, mappings = load_config(args.config)
        result = check_config(config, mappings)
        if args.inventory is None:
            print(json.dumps(result, indent=2))
            return 0
        if args.inventory.exists():
            parser.error(f"Backup already exists: {args.inventory}")
        inventory = read_inventory(config)
        args.inventory.parent.mkdir(parents=True, exist_ok=True)
        with args.inventory.open("x", encoding="utf-8") as backup:
            json.dump(inventory, backup, indent=2)
            backup.write("\n")
        print(json.dumps({"backup": str(args.inventory), "complete": inventory["complete"],
                          "axes_read": len(inventory["axes"]), "axes_matching_target": inventory["axes_matching_target"]}))
        return 0 if inventory["complete"] else 2
    except (DriverError, KeyError, OSError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
