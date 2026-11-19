"""Offline search for parallel, adjacent hands using the official arm IK.

Does not send commands to the running simulation or to hardware.
"""
import json
import argparse
from pathlib import Path
import struct
import zlib

import mujoco
import numpy as np

from openarm_app.kinematics import OpenArmIK
from openarm_app.joint_convention import SIGNS
from paino_skill.scene import PIANO, build_scene


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--metric", choices=("conservative", "visual"), default="conservative")
    args = parser.parse_args()
    model, data, config = build_scene()
    rest = data.qpos.copy()
    arms = {s: OpenArmIK(side=s) for s in ("right", "left")}
    indices = {s: [model.joint(f"openarm_{s}_joint{i}").qposadr[0]
                   for i in range(1, 8)] for s in arms}
    hands = {}
    for side in arms:
        # Include visual mesh hulls as well as the simplified contact geometry.
        body_ids = {i for i in range(model.nbody)
                    if model.body(i).name.startswith(f"aero_{side}_")}
        hands[side] = [i for i in range(model.ngeom) if model.geom_bodyid[i] in body_ids
                       and ((model.geom_group[i] == 2) if args.metric == "visual" else
                            (model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                             or model.geom_contype[i] or model.geom_conaffinity[i]))]

    def gap():
        best = (.1, None, None)
        for a in hands["right"]:
            for b in hands["left"]:
                distance = mujoco.mj_geomDistance(model, data, a, b, .1, None)
                if distance < best[0]:
                    best = (float(distance), a, b)
        return best

    def solve(half_spacing):
        data.qpos[:] = rest
        qs = {}
        for side, arm in arms.items():
            pose = arm.forward(arm.home)
            pose[:3] = [.32, -half_spacing if side == "right" else half_spacing, -.18]
            seed = arm.home.copy()
            seed[1] = SIGNS[side][1] * (.6 if side == "right" else -.6)
            q = arm.inverse(pose, seed, max_iterations=500,
                            position_tolerance=2e-7, orientation_tolerance=2e-6)
            qs[side] = q
            data.qpos[indices[side]] = q
        mujoco.mj_forward(model, data)
        return qs, gap()

    low, high = (.04, .075) if args.metric == "visual" else (.065, .11)
    _, inside = solve(low)
    _, outside = solve(high)
    if not inside[0] < 0 < outside[0]:
        raise RuntimeError(f"No contact bracket: {inside}, {outside}")
    for _ in range(17):
        middle = (low + high)/2
        _, nearest = solve(middle)
        if nearest[0] < 0:
            low = middle
        else:
            high = middle
    qs, nearest = solve(high)
    contacts = [{"geom1": model.geom(c.geom1).name, "geom2": model.geom(c.geom2).name,
                 "distance_m": float(c.dist)} for c in data.contact if c.dist < -1e-6]
    if contacts and args.metric == "conservative":
        raise RuntimeError(f"Endpoint has penetrating contacts: {contacts}")
    output = PIANO / f"logs/adjacent-hands-{args.metric}"
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "definition": ("Minimum distance between visible hand geometries (mesh convex hulls); collision proxies may overlap."
                       if args.metric == "visual" else "Minimum distance between hand visual mesh convex hulls and collision primitives; zero within numerical tolerance."),
        "reference_frame": "arm_origin", "mode": "offline ideal kinematics",
        "wrist_half_spacing_m": high,
        "minimum_hand_gap_m": nearest[0],
        "closest_geoms": [model.geom(i).name or f"geom_{i}" for i in nearest[1:]],
        "q_rad": {s: q.tolist() for s, q in qs.items()},
        "q_deg": {s: np.rad2deg(q).tolist() for s, q in qs.items()},
        "wrist_pose": {s: arms[s].forward(q).tolist() for s, q in qs.items()},
        "minimum_joint_limit_margin_rad": {s: float(np.min(np.minimum(q-arms[s].limits[:,0], arms[s].limits[:,1]-q))) for s,q in qs.items()},
        "penetrating_contacts": contacts,
        "scene_config": config,
        "limitations": "Static posture only. Meshes use convex hull distances. No continuous path collision certification or hardware validation.",
    }
    (output / "posture.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    np.save(output / "qpos.npy", data.qpos)
    print(json.dumps(report, indent=2), flush=True)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.27, 0, .43]
    camera.distance, camera.azimuth, camera.elevation = 1.65, 135, -32
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    option.sitegroup[:] = 0
    option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = False
    model.vis.global_.offwidth, model.vis.global_.offheight = 1440, 1080
    with mujoco.Renderer(model, height=1080, width=1440) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        pixels = renderer.render()
    def chunk(kind, payload):
        return struct.pack('!I', len(payload)) + kind + payload + struct.pack('!I', zlib.crc32(kind+payload))
    raw = b''.join(b'\0'+row.tobytes() for row in pixels)
    (output / "preview.png").write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',1440,1080,8,2,0,0,0))
                                        +chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


if __name__ == "__main__":
    main()


