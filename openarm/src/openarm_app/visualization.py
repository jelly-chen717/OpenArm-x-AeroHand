"""Read-only export of the active MuJoCo display model for browser rendering."""
import base64
import numpy as np


def geometry_ids(model):
    return [i for i in range(model.ngeom) if model.geom_group[i] != 3 and model.geom_rgba[i, 3] > 0]


def visual_state(model, data, ids):
    return {"positions": data.geom_xpos[ids].tolist(), "rotations": data.geom_xmat[ids].tolist()}


def visual_model(model, ids):
    meshes = {}
    geoms = []
    for i in ids:
        mesh = int(model.geom_dataid[i]) if int(model.geom_type[i]) == 7 else -1
        material = int(model.geom_matid[i])
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[i]
        geoms.append({"type": int(model.geom_type[i]), "size": model.geom_size[i].tolist(),
                      "mesh": mesh, "rgba": rgba.tolist()})
        if mesh >= 0 and str(mesh) not in meshes:
            v, f = int(model.mesh_vertadr[mesh]), int(model.mesh_faceadr[mesh])
            vertices = np.asarray(model.mesh_vert[v:v+model.mesh_vertnum[mesh]], dtype="<f4")
            faces = np.asarray(model.mesh_face[f:f+model.mesh_facenum[mesh]], dtype="<u4")
            meshes[str(mesh)] = {"vertices": base64.b64encode(vertices.tobytes()).decode(),
                                 "faces": base64.b64encode(faces.tobytes()).decode()}
    return {"geoms": geoms, "meshes": meshes}
