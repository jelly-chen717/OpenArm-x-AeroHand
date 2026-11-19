import copy
import json
import numpy as np
import pytest
from paino_skill.right_melody import load_plan, DEFAULT_PLAN, state_at, time_grid


def test_full_score_timing_release_and_endpoints():
    plan=load_plan(DEFAULT_PLAN)
    grid=time_grid(plan)
    for e in plan['events']:
        t=e['press_start_s']
        y,fingers,active=state_at(plan,t)
        assert active['index']==e['index']
        assert y==pytest.approx(e['y_mm']/1000)
        assert np.sum(fingers)==1
        assert np.sum(state_at(plan,t+.5)[1])==0
        assert round(t+1.25,8) in grid
    assert state_at(plan,78)[2] is None
    assert state_at(plan,-.95)[0]==0
    assert state_at(plan,-.2)[0]==pytest.approx(-.078)


def test_repeated_notes_release_and_retrigger():
    plan=load_plan(DEFAULT_PLAN)
    a,b=plan['events'][21:23]
    assert a['key']==b['key']
    assert max(state_at(plan,a['press_start_s']+.5)[1])==0
    assert max(state_at(plan,b['press_start_s'])[1])==1


def test_existing_aero_scene_continuous_path_and_contacts():
    from paino_skill.right_melody_aero import prepare, frame
    from paino_skill.score import WHITES
    plan=load_plan(DEFAULT_PLAN)
    engine,times,qs,positions,metrics=prepare(plan)
    assert engine.model.body('aero_right_palm').id>0
    assert len(engine.piano.keys)==88
    assert engine.scene_config['white_key_pitch_m']==.026
    assert metrics['max_position_error_m']<.00005
    for e in plan['events']:
        t=e['press_start_s']
        q=qs[np.argmin(abs(times-t))]
        frame(engine,plan,t,q,positions)
        assert engine.piano.pressed=={WHITES[e['key']+1]}
        frame(engine,plan,t+.5,q,positions)
        assert not engine.piano.pressed


@pytest.mark.parametrize('field,value',[('key',1),('move_y_mm',0),('move_s',.2),('press_start_s',.1)])
def test_reject_corrupt_plan(tmp_path,field,value):
    plan=copy.deepcopy(load_plan(DEFAULT_PLAN))
    plan['events'][0][field]=value
    path=tmp_path/'invalid.json'
    path.write_text(json.dumps(plan),encoding='utf-8')
    with pytest.raises(ValueError):
        load_plan(path)
