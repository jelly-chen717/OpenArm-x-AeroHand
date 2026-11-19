"""Render new MuJoCo frames for a model-space direction comparison only."""
from pathlib import Path
import json,sys,math,struct,zlib
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'openarm/src'))
from openarm_app.kinematics.model_adapter import load_v1
OUT=Path(__file__).resolve().parent
request=json.loads((OUT/'reconfirmed_original/request.draft.json').read_text(encoding='utf-8'))
setup,_,_=load_v1();m,d=setup.model,setup.data
keys=[(j['side'],j['motor_id']) for j in request['joints']]
ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_{s}_joint{j}') for s,j in keys]
qidx=m.jnt_qposadr[ids]
m.vis.global_.offwidth=900;m.vis.global_.offheight=850
renderer=mujoco.Renderer(m,height=850,width=900)
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.39]
cam.distance=1.45;cam.azimuth=180;cam.elevation=-3
opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[0]=1;opt.geomgroup[2]=1
images=[]
for j5 in [(-88,-88),(-88,88)]:
    q=np.array(request['initial_model_position'])
    for side,val in zip(['left','right'],j5):
        q[keys.index((side,5))]+=math.radians(val)
        q[keys.index((side,7))]+=math.radians(88)
    d.qpos[qidx]=q;mujoco.mj_forward(m,d)
    renderer.update_scene(d,camera=cam,scene_option=opt)
    images.append(renderer.render().copy())
renderer.close()
arr=np.concatenate(images,axis=1)
h,w,_=arr.shape
raw=b''.join(b'\0'+row.tobytes() for row in arr)
def chunk(k,b):return struct.pack('!I',len(b))+k+b+struct.pack('!I',zlib.crc32(k+b)&0xffffffff)
(OUT/'outward_direction_comparison_raw.png').write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
print('Rendered both stage-2 endpoints from latest reconfirmed baseline, fixed front camera.')
