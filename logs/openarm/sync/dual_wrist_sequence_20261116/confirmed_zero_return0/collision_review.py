"""Offline geometry audit only. Does not import hardware transports."""
from pathlib import Path
import json, sys, struct, zlib
import numpy as np
import mujoco
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'openarm/src'))
from openarm_app.kinematics.model_adapter import load_v1
OUT=Path(__file__).resolve().parent
request=json.loads((OUT/'request.draft.json').read_text(encoding='utf-8'))
setup,_,path=load_v1()
m,d=setup.model,setup.data
ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f"openarm_{j['side']}_joint{j['motor_id']}") for j in request['joints']]
qi=m.jnt_qposadr[ids]
points=request['points']
worst=None
stages=[]
for stage,(a,b) in enumerate(zip(points,points[1:]),1):
    hits=0
    first=None
    for u in np.linspace(0,1,401):
        blend=10*u**3-15*u**4+6*u**5
        q=np.array(a['q'])+(np.array(b['q'])-np.array(a['q']))*blend
        d.qpos[qi]=q
        mujoco.mj_forward(m,d)
        if d.ncon:
            hits+=1
            if first is None:first=float(u)
        for c in d.contact[:d.ncon]:
            if worst is None or c.dist<worst['distance_m']:
                worst=dict(stage=stage,fraction=float(u),distance_m=float(c.dist),
                           pair=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in [c.geom1,c.geom2]],
                           position=c.pos.tolist(),q=q.tolist(),qpos=d.qpos.tolist())
    stages.append(dict(stage=stage,contact_samples=hits,first_contact_fraction=first))

def obj_stats(filename):
    verts=[];faces=[]
    for line in filename.read_text().splitlines():
        if line.startswith('v '):verts.append([float(x)*.001 for x in line.split()[1:4]])
        elif line.startswith('f '):
            v=[int(x.split('/')[0])-1 for x in line.split()[1:]]
            faces.extend([[v[0],v[i],v[i+1]] for i in range(1,len(v)-1)])
    v=np.array(verts);f=np.array(faces)
    hull=ConvexHull(v)
    volume=abs(float(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6))
    return dict(file=str(filename.relative_to(ROOT)),vertices=len(v),triangles=len(f),
                bounds_m=[v.min(axis=0).tolist(),v.max(axis=0).tolist()],signed_mesh_volume_m3=volume,
                convex_hull_volume_m3=float(hull.volume),convex_hull_to_mesh_volume=float(hull.volume/volume) if volume else None)

body_stats=[obj_stats(path.parent/'meshes/visual/body'/f'body_link0_{i}.obj') for i in range(6)]
grip_stats=[obj_stats(path.parent/'meshes/visual/gripper'/f'{p}_{i}.obj') for p in ['hand','finger'] for i in range(2)]
collision_geoms=[]
for name in ['openarm_body_link0_collision','openarm_right_hand_collision','openarm_right_right_finger_collision']:
    g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name)
    mid=int(m.geom_dataid[g])
    collision_geoms.append(dict(name=name,mesh=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_MESH,mid),
                                margin=float(m.geom_margin[g]),gap=float(m.geom_gap[g])))
result=dict(worst=worst,stages=stages,body_visual_meshes=body_stats,gripper_visual_meshes=grip_stats,
            collision_geoms=collision_geoms)

def world_mesh(g):
    mid=m.geom_dataid[g]
    va=m.mesh_vertadr[mid];vn=m.mesh_vertnum[mid]
    fa=m.mesh_faceadr[mid];fn=m.mesh_facenum[mid]
    vertices=m.mesh_vert[va:va+vn]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    return vertices,m.mesh_face[fa:fa+fn].copy()

def edge_surface_hits(v,f,t):
    edges=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
    low=t.min(axis=(0,1));high=t.max(axis=(0,1))
    a,b=v[edges[:,0]],v[edges[:,1]]
    mask=np.all(np.maximum(a,b)>=low,axis=1)&np.all(np.minimum(a,b)<=high,axis=1)
    hits=[]
    e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0]
    for a,b in zip(a[mask],b[mask]):
        direction=b-a
        h=np.cross(np.broadcast_to(direction,e2.shape),e2)
        det=np.einsum('ij,ij->i',e1,h)
        keep=np.abs(det)>1e-12
        inv=np.zeros_like(det);inv[keep]=1/det[keep]
        ss=a-t[:,0]
        u=np.einsum('ij,ij->i',ss,h)*inv
        qq=np.cross(ss,e1)
        vv=(qq@direction)*inv
        tt=np.einsum('ij,ij->i',e2,qq)*inv
        good=keep&(u>=-1e-8)&(vv>=-1e-8)&(u+vv<=1+1e-8)&(tt>=-1e-8)&(tt<=1+1e-8)
        for value in tt[good]:
            hits.append((a+direction*value).tolist())
        if len(hits)>=30:break
    return hits

if worst:
    d.qpos[:]=worst['qpos'];mujoco.mj_forward(m,d)
    base=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'openarm_body_link0_visual_3')
    bv,bf=world_mesh(base)
    exact=[]
    for g in range(m.ngeom):
        name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) or ''
        if ('openarm_right_hand_visual' in name or 'openarm_right_right_finger_visual' in name) and m.geom_group[g]==2:
            gv,gf=world_mesh(g)
            hits=edge_surface_hits(gv,gf,bv[bf])
            if not hits:hits=edge_surface_hits(bv,bf,gv[gf])
            exact.append(dict(geom=name,visual_surface_intersection_count_capped=len(hits),intersection_points_m=hits,
                              bounds_m=[gv.min(axis=0).tolist(),gv.max(axis=0).tolist()]))
    result['exact_visual_surface_intersections']=exact

def save_png(file,arr):
    height,width,_=arr.shape
    raw=b''.join(b'\0'+row.tobytes() for row in arr)
    def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    file.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

if worst:
    d.qpos[:]=worst['qpos'];mujoco.mj_forward(m,d)
    m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    renderer=mujoco.Renderer(m,height=700,width=850)
    images=[]
    for azimuth,collision in [(135,False),(135,True),(-135,False),(-135,True)]:
        cam=mujoco.MjvCamera()
        cam.lookat[:]=np.array(worst['position'])+np.array([0,0,.10])
        cam.distance=.95;cam.azimuth=azimuth;cam.elevation=-12
        opt=mujoco.MjvOption()
        opt.geomgroup[:]=0
        opt.geomgroup[0]=1
        opt.geomgroup[3 if collision else 2]=1
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT]=1
        renderer.update_scene(d,camera=cam,scene_option=opt)
        images.append(renderer.render().copy())
    renderer.close()
    save_png(OUT/'collision_review.png',np.concatenate([np.concatenate(images[:2],axis=1),np.concatenate(images[2:],axis=1)],axis=0))
    result['image_layout']='Left column visual meshes; right column collision meshes. Top azimuth 135, bottom -135. Worst penetration pose.'
(OUT/'collision_geometry_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(worst=worst,stages=stages,base=body_stats[3],collision_geoms=collision_geoms),ensure_ascii=False))
