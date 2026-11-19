import pytest
from paino_skill.right_melody_eop import parse,state


def test_user_score_lengths_and_rests():
    p=parse()
    assert len(p['events'])==51
    assert len(p['slots'])==53
    assert p['duration_s']/6==23.25
    assert [s['beats'] for s in p['slots'] if '.' in s['token']]==[.75,.75]
    assert [s['beats'] for s in p['slots'] if s['key'] is None]==[.25,1.]
    assert min(e['key'] for e in p['events'])==16
    for s in p['slots']:
        if s['key'] is None:
            assert state(p,s['press_start_s']+.01)[2] is None
    for e in p['events']:
        y,fingers,event=state(p,e['press_start_s'])
        assert event['index']==e['index']
        assert y==pytest.approx(e['y_mm']/1000)
        assert sum(fingers)==1


def test_explicit_eop_duration_override():
    p=parse("=5'. =4' 5 0")
    assert [s['beats'] for s in p['slots']]==[.75,.25,1.,1.]
    with pytest.raises(ValueError):
        parse('5 nonsense')
