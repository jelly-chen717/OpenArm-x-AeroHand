from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from openarm_control import ArmSetup
from openarm_mujoco.v2 import JointResolver
from openarm_app.joint_convention import apply_v1_convention


def load_v1(xml=None, *, side="right", model=None, unified=True):
    if side not in ("right", "left"):
        raise ValueError("side must be right or left")
    # The PyPI model distribution bundles V2 only; V1 comes from the pinned repo.
    path = (Path(xml) if xml else Path(__file__).resolve().parents[3] /
            "vendor/openarm_mujoco/v1/scene.xml")
    if not path.is_file():
        raise FileNotFoundError(f"V1 scene missing: {path}; restore the pinned model repository")
    # Native Windows filename handling fails on some non-ASCII paths. Python
    # reads official assets; a private XML copy adapts joint signs before compilation.
    if model is None:
        assets = {p.relative_to(path.parent).as_posix(): p.read_bytes()
                  for p in path.parent.rglob("*") if p.is_file()
                  and p.suffix.lower() in {".xml", ".stl", ".obj", ".png", ".jpg", ".jpeg"}}
        source = path.read_text(encoding="utf-8")
        if unified:
            if 'openarm_bimanual.xml' in assets:
                robot = ET.fromstring(assets['openarm_bimanual.xml'])
                assets['openarm_bimanual.xml'] = ET.tostring(apply_v1_convention(robot))
            else:
                source = ET.tostring(apply_v1_convention(ET.fromstring(source)), encoding='unicode')
        model = mujoco.MjModel.from_xml_string(source, assets)
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key < 0:
        raise ValueError("Required home keyframe missing")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_ee_control_point")
    origin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "arm_origin")
    if min(ee, origin) < 0:
        raise ValueError("Required V1 end-effector/origin site missing")
    setup = ArmSetup(model, data, JointResolver(model), [side],
                     {side: ee}, {side: "site"}, origin, "site")
    names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    if min(ids) < 0 or not np.all(model.jnt_limited[ids]):
        raise ValueError(f"Required bounded {side}-arm joints missing")
    indices = setup.joint_resolver.arm_qpos_indices(side)
    if not np.array_equal(indices, model.jnt_qposadr[ids]):
        raise ValueError("Resolver qpos mapping mismatch")
    return setup, model.jnt_range[ids].copy(), path.resolve()
