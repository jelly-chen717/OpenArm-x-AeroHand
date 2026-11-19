"""Compose a piano scene from a sign-adapted V1 copy and pinned Menagerie sources.

Only the display model has hands replaced. Official IK continues to use the
original V1 scene; named arm joints map its results into this scene.
"""
import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from openarm_app.joint_convention import apply_v1_convention

PIANO = Path(__file__).resolve().parent
ROOT = PIANO.parents[1]
V1 = ROOT / "vendor/openarm_mujoco/v1"
HAND = PIANO / "third_party/tetheria_aero_hand_open"


def numbers(values):
    return " ".join(str(float(x)) for x in values)


def section(root, tag):
    found = root.find(tag)
    return found if found is not None else ET.SubElement(root, tag)


def scene_xml(config=None):
    cfg = json.loads((Path(config) if config else PIANO / "scene.json").read_text())
    width, depth, height = cfg["size_m"]
    pitch, gap = cfg["white_key_pitch_m"], cfg["white_key_gap_m"]
    midis = range(cfg["first_midi"], cfg["first_midi"] + cfg["key_count"])
    whites = [n for n in midis if n % 12 in (0, 2, 4, 5, 7, 9, 11)]
    if not 0 < gap < pitch or len(whites)*pitch > width - .008:
        raise ValueError("White-key pitch/gap does not fit the keyboard width")
    if min(width, depth, height) <= 0 or not 0 < cfg["key_travel_m"] < height/2:
        raise ValueError("Invalid piano dimensions or key travel")
    # Explicitly start from the project's V1 scene, resolving its model include.
    root = ET.parse(V1 / "scene.xml").getroot()
    root.remove(root.find("include"))
    robot = ET.parse(V1 / "openarm_bimanual.xml").getroot()
    apply_v1_convention(robot)
    for child in robot:
        section(root, child.tag).extend(copy.deepcopy(list(child))) if child.tag in (
            "worldbody", "asset") else root.append(copy.deepcopy(child))
    root.set("model", "OpenArm V1 / Aero hands / floating piano")
    root.find("compiler").attrib.pop("meshdir", None)
    assets = {}
    for mesh in root.findall("asset/mesh"):
        path = V1 / "meshes" / mesh.get("file")
        filename = "v1_" + mesh.get("file").replace("/", "_")
        assets[filename] = path.read_bytes()
        mesh.set("file", filename)
    # Remove complete gripper subtrees and every reference to removed joints.
    removed = set()
    for side in ("left", "right"):
        link = root.find(f".//body[@name='openarm_{side}_link7']")
        hand = link.find(f"body[@name='openarm_{side}_hand']")
        removed.update(e.get("name") for e in hand.iter() if e.get("name"))
        link.remove(hand)
    for tag in ("tendon", "equality", "actuator", "contact"):
        parent = root.find(tag)
        for child in list(parent):
            if any(v in removed for e in child.iter() for v in e.attrib.values()):
                parent.remove(child)
    root.remove(root.find("keyframe"))  # rebuilt by joint names after compilation
    for side in ("left", "right"):
        hand = ET.parse(HAND / f"{side}_hand.xml").getroot()
        if side == "left":
            # Both hands reuse base_link.STL, but the left palm frame reverses
            # its local y axis. Mirror only the shell, not the articulated hand.
            hand.find("asset/mesh[@name='left_frame_link']").set("scale", "1 -1 1")
        # Keep the articulated hand, but omit the upstream mounting bracket.
        # The source files remain unchanged; the palm attaches directly to link7.
        bracket = hand.find("worldbody/body[@name='tetheria_mount']")
        palm = bracket.find("body[@name='palm']")
        palm.set("childclass", bracket.get("childclass"))
        rotation = np.empty(4)
        mujoco.mju_mulQuat(rotation, np.asarray(cfg["palm_alignment_quat_wxyz"]),
                          np.fromstring(palm.get("quat"), sep=" "))
        rotation /= np.linalg.norm(rotation)
        palm.set("quat", numbers(rotation))
        palm.set("pos", numbers(cfg["palm_pos_m"]))
        hand.find("worldbody").remove(bracket)
        hand.find("worldbody").append(palm)
        hand.find("asset").remove(hand.find("asset/mesh[@name='tetheria_mount']"))
        for exclude in list(hand.find("contact")):
            if "tetheria_mount" in exclude.attrib.values():
                hand.find("contact").remove(exclude)
        prefix = f"aero_{side}_"
        names = {e.get("name") for e in hand.iter() if e.get("name")}
        names.update(e.get("class") for e in hand.iter("default") if e.get("class"))
        for e in hand.iter():
            for key, value in list(e.attrib.items()):
                if value in names and key != "file":
                    e.set(key, prefix + value)
        for mesh in hand.findall("asset/mesh"):
            filename = prefix + mesh.get("file")
            assets[filename] = (HAND / "assets" / mesh.get("file")).read_bytes()
            mesh.set("file", filename)
        for tag in ("asset", "default", "contact", "tendon", "actuator", "sensor", "equality"):
            child = hand.find(tag)
            if child is not None:
                section(root, tag).extend(list(child))
        link = root.find(f".//body[@name='openarm_{side}_link7']")
        for body in hand.findall("worldbody/body"):
            link.append(body)
    world = root.find("worldbody")
    piano = ET.SubElement(world, "body", name="piano", pos=numbers(cfg["center_world_m"]))
    def box(name, pos, size, color, parent=piano, **attrs):
        return ET.SubElement(parent, "geom", name=name, type="box", pos=numbers(pos),
                             size=numbers(size), rgba=color, **attrs)
    # +x is robot-forward. Keyboard bass is +y (the player's left).
    box("piano_base", [0, 0, -height/2+.005], [depth/2, width/2, .005], ".055 .062 .075 1")
    box("piano_back_panel", [depth/2-.021, 0, .005], [.021, width/2, height/2-.005], ".09 .10 .12 1")
    for sign in (-1, 1):
        box(f"piano_cheek_{sign}", [0, sign*(width/2-.004), 0], [depth/2, .004, height/2], ".08 .085 .10 1")
    # Named slide joints and top sites support contact-driven key travel.
    white_idx = 0
    for midi in midis:
        white = midi in whites
        if white:
            y = (len(whites)/2-white_idx-.5)*pitch
            white_idx += 1
        else:
            y = (len(whites)/2-white_idx)*pitch
        key = ET.SubElement(piano, "body", name=f"key_{midi}")
        ET.SubElement(key, "joint", name=f"key_{midi}_press", type="slide", axis="0 0 -1",
                      range=f"0 {cfg['key_travel_m']}", limited="true", stiffness="80", damping="1")
        if white:
            x, z, sx, sy, sz = -.066, .005, .023, (pitch-gap)/2, .005
            # Notch the back of each white key around neighboring black keys.
            # Black keys must not sit on overlapping white-key collision boxes.
            lower, upper = y-sy, y+sy
            if midi-1 in midis and (midi-1) % 12 not in (0,2,4,5,7,9,11):
                upper = y+pitch/2-pitch*.29-gap/2
            if midi+1 in midis and (midi+1) % 12 not in (0,2,4,5,7,9,11):
                lower = y-pitch/2+pitch*.29+gap/2
            box(f"key_{midi}_rear", [.001,(lower+upper)/2,z], [.044,(upper-lower)/2,sz],
                ".92 .91 .87 1", parent=key, mass="0.012")
        else:
            x, z, sx, sy, sz = .001, .01875, .043, pitch*.29, .00875
        box(f"key_{midi}_surface", [x,y,z], [sx,sy,sz],
            ".92 .91 .87 1" if white else ".018 .021 .026 1", parent=key, mass="0.025")
        ET.SubElement(key, "site", name=f"key_{midi}_top", pos=numbers([x if white else x-.025,y,z+sz]),
                      size="0.002", group="4", rgba="0.2 0.8 0.5 1")
    root.find("statistic").set("center", ".2 0 .40")
    root.find("statistic").set("extent", "1.25")
    root.find("visual/quality").set("shadowsize", "2048")
    root.find("visual/global").set("azimuth", "135")
    root.find("visual/global").set("elevation", "-25")
    root.find("visual").append(ET.Element("map", znear="0.01"))
    return root, assets, cfg


def build_scene(config=None):
    root, assets, cfg = scene_xml(config)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets)
    data = mujoco.MjData(model)
    for side in ("left", "right"):
        data.qpos[model.joint(f"openarm_{side}_joint4").qposadr[0]] = 1.570796
        for joint, angle in cfg.get("thumb_rest_q_rad", {}).items():
            thumb_joint = model.joint(f"aero_{side}_{side}_thumb_{joint}")
            if not np.isfinite(angle) or not thumb_joint.range[0] <= angle <= thumb_joint.range[1]:
                raise ValueError(f"Thumb rest position outside joint limits: {joint}")
            data.qpos[thumb_joint.qposadr[0]] = angle
    mujoco.mj_forward(model, data)
    return model, data, cfg


def main():
    parser = argparse.ArgumentParser(description="Build/check/render the piano scene")
    parser.add_argument("--render", type=Path)
    args = parser.parse_args()
    model, data, cfg = build_scene()
    print(json.dumps({"nq":model.nq,"nu":model.nu,"keys":cfg["key_count"],"assumptions":cfg["assumptions"]}, indent=2))
    if args.render:
        import struct
        import zlib
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [.2,0,.42]
        camera.distance, camera.azimuth, camera.elevation = 1.75, 135, -25
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        option.sitegroup[:] = 0
        option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = False
        model.vis.global_.offwidth, model.vis.global_.offheight = 1440, 1080
        with mujoco.Renderer(model, height=1080, width=1440) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=option)
            args.render.parent.mkdir(parents=True, exist_ok=True)
            pixels = renderer.render()
            def chunk(kind, payload):
                return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind+payload))
            raw = b''.join(b'\0' + row.tobytes() for row in pixels)
            args.render.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B',1440,1080,8,2,0,0,0))
                                    + chunk(b'IDAT',zlib.compress(raw)) + chunk(b'IEND',b''))


if __name__ == "__main__":
    main()
