import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from openarm_joint_mapping import JointMapping, load_mapping
from openarm_feedback import FeedbackCache, FeedbackWorker, Sample
from mirror_openarm_mujoco import Mirror, ROOT


def batch(now=10, position=.1):
    return [Sample(i, position, .02, now) for i in range(1,8)]


def test_reverse_sign_offset_roundtrip():
    m=JointMapping('right',1,'openarm_right_joint1',-1,2,.3,-.2,(-2,2))
    assert m.to_model(.5)==pytest.approx(-.6)
    assert m.to_motor(m.to_model(.5))==pytest.approx(.5)
    assert m.velocity_to_motor(m.velocity_to_model(.2))==pytest.approx(.2)
    with pytest.raises(ValueError):m.to_model(float('nan'))
    with pytest.raises(ValueError):JointMapping('right',1,'openarm_right_joint1',1,0,0,0,(-1,1))


def test_candidate_never_authorizes_hardware(tmp_path):
    path=ROOT/'configs/openarm_joint_mapping.json'
    assert len(load_mapping(path)[1])==14
    config = json.loads(path.read_text())
    for row in config['joints']:
        row['hardware_verified'] = False
    unverified = tmp_path/'unverified.json'
    unverified.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='not verified'):load_mapping(unverified,hardware=True)


def test_hardware_mapping_validates_selected_arm(tmp_path):
    config=json.loads((ROOT/'configs/openarm_joint_mapping.json').read_text())
    for row in config['joints']:
        row['hardware_verified'] = False
        if row['side']=='right':
            row.update(hardware_verified=True, evidence='test fixture',
                       hardware_mapping=row['model_candidate'])
    path=tmp_path/'mapping.json'
    path.write_text(json.dumps(config))
    assert len(load_mapping(path,hardware=True,sides=('right',))[1])==7
    with pytest.raises(ValueError,match='not verified'):
        load_mapping(path,hardware=True)
    config['joints'].pop()
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='7'):
        load_mapping(path,hardware=True,sides=('right',))


def test_duplicate_missing_and_nonfinite_config(tmp_path):
    config=json.loads((ROOT/'configs/openarm_joint_mapping.json').read_text())
    path=tmp_path/'mapping.json'
    config['joints'][-1]=config['joints'][0]
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='Duplicate'):load_mapping(path)
    config['joints']=config['joints'][:1]
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='14'):load_mapping(path)


def test_partial_stale_skew_nan_and_reconnect():
    c=FeedbackCache()
    c.update('left',batch())
    assert c.snapshot('left',10,.2,.1)['valid']
    assert not c.snapshot('left',10.3,.2,.1)['valid']
    c.update('left',batch()[:6])
    assert not c.snapshot('left',10,.2,.1)['valid']
    c.update('left',batch(position=float('nan')))
    assert not c.snapshot('left',10,.2,.1)['valid']
    c.update('left',[Sample(i,.1,0,10 if i==1 else 10.15) for i in range(1,8)])
    assert not c.snapshot('left',10.15,.3,.1)['valid']
    c.update('left',(), 'disconnected')
    assert not c.snapshot('left',10,.2,.1)['valid']
    c.update('left',batch(11))
    assert c.snapshot('left',11,.2,.1)['valid']


def test_worker_recovers_and_closes_on_same_thread():
    calls=[]
    ready=threading.Event()
    class Source:
        count=0
        def __enter__(self):calls.append(('open',threading.get_ident()));return self
        def read(self):
            Source.count+=1
            if Source.count==1:raise OSError('temporary timeout')
            ready.set();return batch(time.monotonic())
        def __exit__(self,*args):calls.append(('close',threading.get_ident()))
    cache=FeedbackCache();w=FeedbackWorker(Source,'right',cache,.005)
    w.start()
    try:
        assert ready.wait(1)
    finally:w.stop()
    assert calls[0][0]=='open' and calls[-1][0]=='close'
    assert calls[0][1]==calls[-1][1]!=threading.get_ident()
    assert cache.snapshot('right',time.monotonic(),1,1)['valid']


def test_worker_reopens_after_initialization_and_read_failure():
    calls = []
    ready = threading.Event()
    class Source:
        def __init__(self):
            self.number = len([c for c in calls if c[0] == 'open'])
        def __enter__(self):
            calls.append(('open', threading.get_ident()))
            if self.number == 0:
                raise OSError('adapter missing')
            return self
        def read(self):
            if self.number == 1:
                raise OSError('adapter unplugged')
            ready.set()
            return batch(time.monotonic())
        def __exit__(self, *args):
            calls.append(('close', threading.get_ident()))
    cache = FeedbackCache()
    worker = FeedbackWorker(Source, 'right', cache, .005, reconnect_delay=.005)
    worker.start()
    try:
        assert ready.wait(1)
    finally:
        worker.stop()
    assert sum(c[0] == 'open' for c in calls) == 3
    assert sum(c[0] == 'close' for c in calls) == 2
    assert len({c[1] for c in calls}) == 1
    assert worker.error is None
    assert cache.snapshot('right', time.monotonic(), 1, 1)['valid']


def test_model_atomic_update_freeze_and_independent_arm():
    _,maps=load_mapping(ROOT/'configs/openarm_joint_mapping.json')
    m=Mirror(maps);c=FeedbackCache()
    def legal(side,offset):
        return [Sample(i,maps[side,i].to_motor(sum(maps[side,i].limits)/2+offset),0,10) for i in range(1,8)]
    c.update('left',legal('left',0))
    assert m.apply(c.snapshot('left',10,1,1))
    left_indices=[m.indices['left',i][0] for i in range(1,8)]
    saved=m.data.qpos[left_indices].copy()
    broken=legal('left',0);broken[-1]=Sample(7,100,0,10)
    c.update('left',broken)
    report=c.snapshot('left',10,1,1)
    assert not m.apply(report) and not report['valid']
    np.testing.assert_array_equal(m.data.qpos[left_indices],saved)
    c.update('right',legal('right',.02))
    assert m.apply(c.snapshot('right',10,1,1))
    np.testing.assert_array_equal(m.data.qpos[left_indices],saved)
