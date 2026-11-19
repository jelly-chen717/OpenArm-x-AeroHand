import base64
import numpy as np
import mujoco
from openarm_app.visualization import geometry_ids, visual_model, visual_state


def test_export_preserves_compiled_mesh_and_world_pose():
    model = mujoco.MjModel.from_xml_string('''<mujoco><asset><mesh name="tetra" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset><worldbody>
    <body pos=".2 .3 .4"><joint type="hinge" axis="0 0 1"/><geom type="mesh" mesh="tetra" rgba=".1 .2 .3 1"/>
    <geom type="sphere" size=".05" group="3"/></body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[0] = .7
    mujoco.mj_forward(model, data)
    ids = geometry_ids(model)
    assert ids == [0]
    visual = visual_model(model, ids)
    frame = visual_state(model, data, ids)
    mesh = visual["meshes"]["0"]
    vertices = np.frombuffer(base64.b64decode(mesh["vertices"]), dtype="<f4").reshape(-1, 3)
    faces = np.frombuffer(base64.b64decode(mesh["faces"]), dtype="<u4").reshape(-1, 3)
    np.testing.assert_array_equal(vertices, model.mesh_vert)
    np.testing.assert_array_equal(faces, model.mesh_face)
    np.testing.assert_array_equal(frame["positions"][0], data.geom_xpos[0])
    np.testing.assert_array_equal(frame["rotations"][0], data.geom_xmat[0])
    np.testing.assert_allclose(visual["geoms"][0]["rgba"], [.1,.2,.3,1])
