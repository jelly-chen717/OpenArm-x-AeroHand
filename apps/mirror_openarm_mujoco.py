"""Independent feedback mirror. Replay by default; PCAN queries require explicit hardware mode."""
from contextlib import ExitStack
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'openarm/src'))
from openarm_joint_mapping import finite, load_mapping, CONVENTION_VERSION
from openarm_feedback import FeedbackCache, FeedbackWorker, PcanFeedbackSource, Sample


class Mirror:
    def __init__(self, mappings):
        import mujoco
        from openarm_app.kinematics.model_adapter import load_v1
        self.mj = mujoco
        conventions = {m.coordinate_convention for m in mappings.values()}
        if len(conventions) != 1 or not conventions <= {'official-v1', CONVENTION_VERSION}:
            raise ValueError('Mixed or unsupported mirror coordinate conventions')
        self.setup, _, _ = load_v1(unified=CONVENTION_VERSION in conventions)
        self.model, self.data = self.setup.model, self.setup.data
        self.mappings = mappings
        self.indices = {}
        for key, mapping in mappings.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, mapping.joint_name)
            if jid < 0:
                raise ValueError('Missing model joint: '+mapping.joint_name)
            if any(abs(a-b) > 1e-10 for a,b in zip(mapping.limits, self.model.jnt_range[jid])):
                raise ValueError('Mapping limits differ from mirror model: '+mapping.joint_name)
            self.indices[key] = (int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid]))

    def apply(self, report):
        if not report['valid']:
            return False
        updates = []
        try:
            for sample in report['samples']:
                key = report['side'], sample.motor_id
                mapping = self.mappings[key]
                q = mapping.to_model(sample.position)
                v = mapping.velocity_to_model(sample.velocity)
                if not mapping.limits[0] <= q <= mapping.limits[1]:
                    raise ValueError('mapped position outside limits')
                updates.append((*self.indices[key], q, v))
        except ValueError as exc:
            report.update(valid=False, reason=str(exc))
            return False
        # Atomic per-arm update: no partial pose on invalid samples.
        for qi, vi, q, v in updates:
            self.data.qpos[qi] = q
            self.data.qvel[vi] = v
        self.mj.mj_forward(self.model, self.data)
        return True


def replay_events(path):
    events = [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
    events = [e for e in events if e.get('type') not in ('session','exit')]
    previous = -1
    for event in events:
        t = finite(event['t'])
        if t < 0 or t < previous or event['side'] not in ('left', 'right'):
            raise ValueError('Invalid replay timestamp or side')
        previous = t
    return events


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=('replay', 'hardware'), default='replay')
    p.add_argument('--mapping', type=Path, default=ROOT/'configs/openarm_joint_mapping.json')
    p.add_argument('--replay', type=Path, default=ROOT/'configs/fixtures/openarm_feedback_demo.jsonl')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--duration', type=float, default=5)
    p.add_argument('--side', choices=('left','right','both'), default='both')
    p.add_argument('--right-channel', type=lambda s:int(s,0), default=0x51)
    p.add_argument('--left-channel', type=lambda s:int(s,0), default=0x52)
    p.add_argument('--period', type=float, default=.03)
    p.add_argument('--max-age', type=float, default=.3)
    p.add_argument('--max-skew', type=float, default=.2)
    p.add_argument('--confirm-bus-ownership', action='store_true')
    p.add_argument('--preview-candidate-mapping', action='store_true',
                   help='Display real feedback with unverified model candidates for visual calibration')
    p.add_argument('--log', type=Path)
    p.add_argument('--stop-file', type=Path, help='Exit cleanly when this file exists')
    args=p.parse_args()
    for v in (args.duration,args.period,args.max_age,args.max_skew):
        if finite(v)<=0:p.error('Timing values must be positive')
    if args.mode=='hardware' and not args.confirm_bus_ownership:
        p.error('Complete A02 and pass --confirm-bus-ownership before querying PCAN')
    if args.left_channel==args.right_channel:p.error('Channels must differ')
    sides=('left','right') if args.side=='both' else (args.side,)
    verified_mapping = args.mode == 'hardware' and not args.preview_candidate_mapping
    version,mappings=load_mapping(args.mapping,hardware=verified_mapping,sides=sides)
    mapping_status = 'VERIFIED MAPPING' if verified_mapping else 'UNVERIFIED MODEL CANDIDATE'
    mirror=Mirror(mappings)
    cache=FeedbackCache()
    events=replay_events(args.replay) if args.mode=='replay' else []
    logpath=args.log or ROOT/'logs/openarm/sync'/f'mirror_{time.time_ns()}.jsonl'
    logpath.parent.mkdir(parents=True,exist_ok=True)
    with ExitStack() as stack:
        log=stack.enter_context(logpath.open('x',encoding='utf-8'))
        log.write(json.dumps(dict(type='session',mode=args.mode,mapping_version=version,
            selected_sides=sides,
            mapping_status=mapping_status,
            channels={side:hex(getattr(args, side+'_channel')) for side in sides}
                if args.mode=='hardware' else {},
            feedback_kind='synthetic_fixture' if args.mode=='replay' else 'hardware',
            max_age_s=args.max_age,max_skew_s=args.max_skew))+'\n')
        viewer=None
        if not args.headless:
            import mujoco.viewer
            viewer=stack.enter_context(mujoco.viewer.launch_passive(mirror.model,mirror.data))
        for side in sides if args.mode=='hardware' else ():
            channel=getattr(args,side+'_channel')
            w=FeedbackWorker(lambda s=side,c=channel:PcanFeedbackSource(s,c),side,cache,args.period)
            w.start()
            stack.callback(w.stop)
        started=time.monotonic()
        cursor=0
        previous={}
        last_print=-1.
        while time.monotonic()-started<args.duration and (viewer is None or viewer.is_running()):
            if args.stop_file is not None and args.stop_file.exists():
                break
            now=time.monotonic()
            elapsed=now-started
            while cursor<len(events) and events[cursor]['t']<=elapsed:
                e=events[cursor]
                samples=[Sample(s['motor_id'],s['position'] if s['position'] is not None else float('nan'),s.get('velocity',0),
                    started+e['t']-s.get('age_s',0),s.get('state',0)) for s in e.get('samples',[])]
                cache.update(e['side'],samples,e.get('error'))
                cursor+=1
            status=[]
            changed=False
            for side in sides:
                r=cache.snapshot(side,now,args.max_age,args.max_skew)
                if r['valid']:mirror.apply(r)
                key=(r['sequence'],r['valid'],r['reason'])
                if previous.get(side)!=key:
                    changed=True
                    record={k:v for k,v in r.items() if k!='samples'}
                    record.update(t=elapsed,samples=[dict(asdict(s),age_s=max(0,now-s.received_at)) for s in r['samples']])
                    # Use null for nonfinite input in evidence; status preserves rejection reason.
                    def clean(v):
                        if isinstance(v,float) and not math.isfinite(v):return None
                        if isinstance(v,list):return [clean(x) for x in v]
                        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
                        return v
                    log.write(json.dumps(clean(record),allow_nan=False)+'\n');log.flush()
                    previous[side]=key
                age='--' if r['age_s'] is None else f"{r['age_s']:.3f}s"
                status.append(f'{side}: '+('OK' if r['valid'] else str(r['reason']))+' age='+age)
            if elapsed-last_print>=1 or (changed and any(not cache.snapshot(s,now,args.max_age,args.max_skew)['valid'] for s in sides)):
                print(args.mode+' | '+' | '.join(status),flush=True);last_print=elapsed
            if viewer:
                viewer.set_texts([(mirror.mj.mjtFontScale.mjFONTSCALE_100,
                    mirror.mj.mjtGridPos.mjGRID_TOPLEFT,
                    args.mode+' / '+mapping_status+' / '+','.join(sides),
                    '\n'.join(status))])
                viewer.sync()
            time.sleep(1/30)
        log.write(json.dumps(dict(type='exit',elapsed_s=time.monotonic()-started))+'\n')
    print('Log:',logpath)


if __name__=='__main__':
    main()
