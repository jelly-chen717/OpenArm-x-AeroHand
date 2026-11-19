from pathlib import Path

import pytest

from paino_skill.score import parse_score

EXAMPLE = """# ignored t = 999
left_arm_move(17,5)
right_arm_move(28,5)
sleep(5)
sleep(1)
sleep(2.692309)
hand(D,7)
sleep(.1)
hand(U,7)
sleep(.284615)
hand(D,1x)
sleep(.1)
hand(U,1x)
right_arm_move(32,1.053847)
sleep(1.053847)
hand(D,8)
sleep(.1)
hand(U,8)
sleep(.284615)
"""


def test_example_exact_times_and_finger_mapping():
    s = parse_score(EXAMPLE, finger_count=4)
    assert s.cursor_us == s.duration_us == 10_615_386
    assert [a['time_us'] for a in s.actions] == [0,0,8_692_309,8_792_309,9_076_924,9_176_924,9_176_924,10_230_771,10_330_771]
    assert [(f['id'],f['key'],f['finger']) for f in s.actions[4]['fingers']] == [('1',17,'pinky'),('x',31,'pinky')]
    assert s.actions[7]['fingers'][0]['key'] == 33
    assert parse_score('right_arm_move(28,1)\nsleep(1)\nhand(D,6789x)\nhand(U,6789x)',finger_count=5).actions[1]['fingers'][0]['key']==28
    assert parse_score('hand(U,1x)',finger_count=4).summary()['sample_midis']==[]


@pytest.mark.parametrize('text',[
    'import os', '__import__("os").system("x")', 'sleep(1);sleep(1)',
    'sleep(float("nan"))', 'sleep(NaN)', 'sleep(inf)', 'sleep(-1)',
    'sleep(.0000001)', 'sleep(1e-9999)', 'sleep(1.0000000000000000000000000001)',
    'right_arm_move(28,0)', 'left_arm_move(21,1)', 'right_arm_move(50,1)',
    'left_arm_move(1.0,1)', 'right_arm_move(23,1)', 'hand(D,5)', 'hand(D,6)',
    'hand(D,7)', 'right_arm_move(28,1)\nhand(D,7)',
    'right_arm_move(28,1)\nright_arm_move(29,1)',
    'right_arm_move(28,1)\nsleep(1)\nhand(D,7)\nright_arm_move(29,1)',
    'right_arm_move(28,1)\nsleep(1)\nhand(D,7)\nhand(D,7)',
    'right_arm_move(28,1)\nsleep(1)\nhand(D,7)', 'hand(U,11)',
])
def test_rejects_bad_text_before_execution(text):
    with pytest.raises(ValueError): parse_score(text,finger_count=4)


def test_explicit_configuration_and_independent_motion():
    for count in (None,True,3,'4',4.0):
        with pytest.raises(ValueError): parse_score(EXAMPLE,finger_count=count)
    with pytest.raises(TypeError): parse_score(EXAMPLE)
    s=parse_score('left_arm_move(17,5)\nright_arm_move(28,3)\nsleep(3)\nhand(D,x)\nsleep(.1)\nhand(U,x)\nsleep(1.9)',finger_count=4)
    assert s.duration_us==5_000_000
    assert s.actions[2]['time_us']==3_000_000


def test_user_song_full_file():
    s=parse_score((Path(__file__).parents[1]/'scores/moon.txt').read_text(encoding='utf-8'),finger_count=4)
    assert s.duration_us==107_391_256
    assert len(s.actions)==280 and s.summary()['note_count']==154
