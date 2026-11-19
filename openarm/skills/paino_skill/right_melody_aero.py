"""Reviewed melody in the existing OpenArm/Aero/88-key scene, simulation only."""
import argparse
import json
from pathlib import Path
import struct
import time
import zlib

import mujoco
import numpy as np

from openarm_app.sim_server import ArmServerState
from openarm_app.kinematics.openarm_ik import pose_error
from paino_skill.right_melody import ROOT, DEFAULT_PLAN, load_plan, state_at, time_grid

CONFIG=Path(__file__).with_name('scene_right_melody.json')
FINGERS=('index','middle','ring','pinky')


class ContactAudio:
    """Monophonic Windows playback using the same samples as the web frontend."""
    def __init__(self,midis):
        import winsound
        self.backend=winsound
        self.paths={m:Path(__file__).with_name('samples')/f'tone ({m-20}).wav' for m in midis}
        for path in self.paths.values():
            if not path.is_file():
                raise FileNotFoundError(path)
        self.on_count=0

    def event(self,event):
        if event['type']=='on':
            self.backend.PlaySound(str(self.paths[event['midi']]),
                self.backend.SND_FILENAME|self.backend.SND_ASYNC|self.backend.SND_NODEFAULT)
            self.on_count+=1
        else:
            self.stop()

    def stop(self):
        self.backend.PlaySound(None,0)


def prepare(plan):
    config=CONFIG.with_name('scene_right_melody_eop.json') if plan.get('format')=='eop' else CONFIG
    engine=ArmServerState(scene='piano',piano_config=config)
    arm=engine.arms['right']
    base_index=plan.get('base_index',19)
    q=engine.score.hand_position('right',base_index+2)['q'].copy()
    base=arm.forward(q)
    times=time_grid(plan)
    rows=[]
    errors=[]
    positions={}
    for t in times:
        y,_,_=state_at(plan,t)
        target=base.copy()
        target[1]+=y
        q=arm.inverse(target,q,position_tolerance=.00005,orientation_tolerance=.0005)
        rows.append(q.copy())
        errors.append(pose_error(arm.forward(q),target))
        if any(abs(y-e['y_mm']/1000)<1e-10 for e in plan['events']) and round(y,6) not in positions:
            y=round(y,6)
            index=base_index+2+round(-y/.026)
            positions[y]=engine.score._validate_position('right',index,q)
    qs=np.array(rows)
    speeds=np.diff(qs,axis=0)/np.diff(times)[:,None]
    limits=np.array([10.471976,10.471976,3.769911,3.769911,12.566371,12.566371,12.566371])
    if np.any(abs(speeds)>limits) or np.max(abs(np.diff(qs,axis=0)))>.1:
        raise ValueError('Joint speed or continuity check failed')
    return engine,times,qs,positions,dict(
        max_position_error_m=float(np.max(np.array(errors)[:,0])),
        max_orientation_error_rad=float(np.max(np.array(errors)[:,1])),
        peak_joint_speed_rad_s=np.max(abs(speeds),axis=0).tolist(),
        peak_joint_acceleration_rad_s2=np.max(abs(np.diff(speeds,axis=0)/
            ((np.diff(times)[:-1]+np.diff(times)[1:])/2)[:,None]),axis=0).tolist())


def frame(engine,plan,t,q,positions):
    _,amounts,event=state_at(plan,t)
    engine.joints['right']=q.copy()
    engine.piano.time=float(t)
    for f,amount in zip(FINGERS,amounts):
        engine.piano.values['right'][f]=float(amount*positions[event['y_mm']/1000]['strokes'][f]) if event else 0.
    engine.refresh_model()


def picture(engine,path):
    camera=mujoco.MjvCamera()
    camera.lookat[:]=[.4,-.08,.42]
    camera.distance,camera.azimuth,camera.elevation=1.55,135,-25
    option=mujoco.MjvOption()
    option.geomgroup[3]=0
    option.sitegroup[:]=0
    option.flags[mujoco.mjtVisFlag.mjVIS_TENDON]=False
    with mujoco.Renderer(engine.model,height=480,width=640) as renderer:
        renderer.update_scene(engine.data,camera=camera,scene_option=option)
        pixels=renderer.render()
        def chunk(kind,payload):
            return struct.pack('!I',len(payload))+kind+payload+struct.pack('!I',zlib.crc32(kind+payload))
        raw=b''.join(b'\0'+row.tobytes() for row in pixels)
        path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',640,480,8,2,0,0,0))+
                         chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def run(viewer=False, bpm=60.):
    if not np.isfinite(bpm) or bpm<=0:
        raise ValueError('BPM must be finite and positive')
    rate=bpm/10.
    from paino_skill.right_melody_eop import parse
    plan=parse()
    output=ROOT/f'logs/openarm/right_melody_eop_20261117_{bpm:g}bpm'
    output.mkdir(parents=True,exist_ok=True)
    (output/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2),encoding='utf-8')
    engine,times,qs,positions,metrics=prepare(plan)
    metrics['peak_joint_speed_rad_s']=(np.array(metrics['peak_joint_speed_rad_s'])*rate).tolist()
    metrics['peak_joint_acceleration_rad_s2']=(np.array(metrics['peak_joint_acceleration_rad_s2'])*rate**2).tolist()
    bad=[]
    collisions={}
    for t,q in zip(times,qs):
        frame(engine,plan,t,q,positions)
        _,_,event=state_at(plan,t)
        expected=set()
        if event:
            from paino_skill.score import WHITES
            expected={WHITES[event['key']+1]}
        if engine.piano.pressed-expected:
            bad.append(dict(time_s=float(t),actual=sorted(engine.piano.pressed),expected=sorted(expected)))
        # Exclude only intentional fingertip/key contacts, not hand/arm collisions.
        for c in engine.data.contact:
            a,b=int(c.geom1),int(c.geom2)
            if ((a in engine.piano.pads and b in engine.piano.key_geoms) or
                (b in engine.piano.pads and a in engine.piano.key_geoms)):
                continue
            if c.dist < -.0001:
                pair=' / '.join(engine.model.geom(g).name for g in (a,b))
                collisions[pair]=min(collisions.get(pair,0),float(c.dist))
    events=list(engine.piano.events)
    on=[e for e in events if e['type']=='on']
    off=[e for e in events if e['type']=='off']
    from paino_skill.score import WHITES
    expected=[WHITES[e['key']+1] for e in plan['events']]
    correct=(len(on)==len(expected) and len(off)==len(expected) and [e['midi'] for e in on]==expected
        and np.allclose([e['time_s'] for e in on],[e['press_start_s'] for e in plan['events']],atol=1e-7)
        and not bad and not collisions and not engine.piano.pressed)
    summary=dict(status='passed' if correct else 'failed',offline_only=True,hardware_executable=False,
        scene='existing Aero hands and 88-key piano, task-local config',
        config=str(CONFIG.with_name('scene_right_melody_eop.json')),servo='ideal position, contact-driven keys; not physical POS_VEL',
        bpm=bpm,score_s=plan['duration_s']/rate,pre_roll_s=.95/rate,
        transition_s=.75/rate,peak_cartesian_speed_mm_s=260*rate,
        simulation_automatic_motion_limits=True,
        exceeds_hardware_rated_reference=bool(np.any(np.array(metrics['peak_joint_speed_rad_s'])>
            np.array([10.471976,10.471976,3.769911,3.769911,12.566371,12.566371,12.566371]))),
        samples=len(times),note_on_count=len(on),note_off_count=len(off),
        wrong_contacts=bad,unexpected_collisions=collisions,
        events=[dict(e,time_s=e['time_s']/rate) for e in events],**metrics)
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    np.savez_compressed(output/'trajectory.npz',time_s=times/rate,q_rad=qs)
    print(json.dumps({k:v for k,v in summary.items() if k!='events'},ensure_ascii=False),flush=True)
    if not correct:
        raise RuntimeError('Existing-scene preflight failed; inspect summary.json')
    frame(engine,plan,0,qs[np.argmin(abs(times))],positions)
    picture(engine,output/'preview.png')
    if viewer:
        from mujoco import viewer as mj_viewer
        frame(engine,plan,times[0],qs[0],positions)
        sound=ContactAudio(set(expected))
        last_event=engine.piano.event_id
        with mj_viewer.launch_passive(engine.model,engine.data) as view:
            view.cam.lookat[:]=[.4,-.08,.42]
            view.cam.distance,view.cam.azimuth,view.cam.elevation=1.55,135,-25
            view.opt.geomgroup[3]=0
            view.opt.sitegroup[:]=0
            view.opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON]=False
            # Offline verification keeps every IK sample. Live playback uses a
            # 60 Hz clock plus exact finger boundaries, with a separate 30 Hz
            # display cadence; viewer.sync() must not pace 4106 solver samples.
            live_times=set(np.arange(times[0],times[-1],rate/60).tolist())
            live_times.update([float(times[0]),float(times[-1])])
            for e in plan['events']:
                live_times.update(e['press_start_s']+v for v in (0.,e['press_hold_s'],e['press_hold_s']+.2))
            live_times=np.array(sorted(live_times))
            live_q=np.column_stack([np.interp(live_times,times,qs[:,j]) for j in range(7)])
            # Finish the first graphics upload before starting the score clock.
            view.sync()
            time.sleep(.3)
            view.sync()
            start=time.perf_counter()
            last_display=start-1
            max_lateness=0.
            for t,q in zip(live_times,live_q):
                if not view.is_running():
                    break
                delay=start+(t+.95)/rate-time.perf_counter()
                if delay>0:
                    time.sleep(delay)
                max_lateness=max(max_lateness,time.perf_counter()-(start+(t+.95)/rate))
                with view.lock():
                    frame(engine,plan,t,q,positions)
                for event in engine.piano.events:
                    if event['id']>last_event:
                        sound.event(event)
                        last_event=event['id']
                if time.perf_counter()-last_display>=1/30 or t==times[-1]:
                    view.sync()
                    last_display=time.perf_counter()
            sound.stop()
            (output/'viewer_run.json').write_text(json.dumps(dict(completed=bool(t==times[-1]),
                bpm=bpm,last_score_time_s=float(t/rate),wall_time_s=time.perf_counter()-start,
                max_lateness_s=max_lateness,live_frames=len(live_times),
                audio_note_on_count=sound.on_count,audio_backend='Windows winsound; existing WAV samples')),
                encoding='utf-8')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true')
    parser.add_argument('--bpm',type=float,default=60.)
    args=parser.parse_args()
    run(args.viewer,args.bpm)
