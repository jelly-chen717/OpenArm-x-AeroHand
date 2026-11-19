import hashlib
import json

import mujoco
import numpy as np
import pytest

from paino_skill.scene import HAND, build_scene
from openarm_app.sim_server import ArmServerState


@pytest.fixture(scope="module")
def piano():
    return ArmServerState(scene="piano")


def test_pinned_hand_sources():
    manifest = json.loads((HAND / "SOURCE.json").read_text())
    assert len(manifest["commit"]) == 40
    for item in manifest["files"]:
        assert hashlib.sha256((HAND / item["path"]).read_bytes()).hexdigest() == item["sha256"]


def test_dimensions_and_complete_replacement(piano):
    m, d = piano.model, piano.data
    assert m.body("piano").jntnum == 0  # suspended, no falling base
    assert np.allclose(m.geom("piano_base").size[:2]*2, [.188, 1.24])
    assert np.allclose(m.body("piano").pos, [.67, -.02, .42])
    keys = [m.joint(i).name for i in range(m.njnt) if m.joint(i).name.startswith("key_")]
    assert len(keys) == 88
    assert sum(n % 12 in (0,2,4,5,7,9,11) for n in range(21,109)) == 52
    assert d.site("key_36_top").xpos[1] - d.site("key_38_top").xpos[1] == pytest.approx(.0235)
    for side in ("left", "right"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{side}_hand") == -1
        assert m.body(f"aero_{side}_palm").id > 0
        # At home the palm's longitudinal axis points forward, not upward.
        assert np.allclose(d.body(f"aero_{side}_palm").xmat.reshape(3, 3)[:, 2],
                           [1, 0, 0], atol=1e-3)
        assert m.body(f"aero_{side}_palm").parentid == m.body(f"openarm_{side}_link7").id
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"aero_{side}_tetheria_mount") == -1
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_adapter_visual") == -1
    assert np.isfinite(d.qpos).all()
    assert all(contact.dist > -1e-8 for contact in d.contact)


def test_ideal_motion_maps_arm_names_and_preserves_hands_keys(piano):
    before = piano.data.qpos.copy()
    target = {s: piano.joints[s] + [.05,0,0,.05,0,0,0] for s in piano.SIDES}
    piano.command({"action":"move_joints", "arm":"both", "q":target})
    piano.advance(piano.duration)
    mask = np.ones(piano.model.nq, dtype=bool)
    for side in piano.SIDES:
        indices = piano.scene_qindices[side]
        mask[indices] = False
        assert np.allclose(piano.data.qpos[indices], target[side])
        source = piano.arms[side].setup
        piano.arms[side].forward(target[side])
        site = f"{side}_ee_control_point"
        assert np.allclose(piano.data.site(site).xpos, source.data.site(site).xpos)
        assert np.allclose(piano.data.site(site).xmat, source.data.site(site).xmat)
    assert np.array_equal(piano.data.qpos[mask], before[mask])
    assert piano.snapshot()["scene"] == "piano"
    assert piano.state == "COMPLETED"


def reset_midpoint(piano):
    from paino_skill.scene import PIANO
    cfg = json.loads((PIANO / "calibration.json").read_text(encoding="utf-8"))
    piano.command({"action":"stop"})
    for side in piano.SIDES:
        piano.joints[side][:] = cfg["center_q_rad"][side]
        piano.velocities[side][:] = 0
    piano.refresh_model()
    return piano.piano.event_id


@pytest.mark.parametrize("side,midi,finger", [("right",60,"index"),("left",55,"index"),
                                            ("right",64,"middle"),("left",54,"index"),
                                            ("right",61,"index")])
def test_aero_real_contact_press_release(piano, side, midi, finger):
    after = reset_midpoint(piano)
    other = "left" if side == "right" else "right"
    held = piano.joints[other].copy()
    piano.command({"action":"piano_play","arm":side,"finger":finger,"midi":midi,
                   "interpolation_steps":180,"velocity_limits":[2]*7,"acceleration_limits":[20]*7})
    assert piano.piano.busy
    assert piano.piano.event_id == after  # accepting a note is not sounding it
    max_depth = 0
    for _ in range(215):
        piano.advance(1/60)
        max_depth = max(max_depth, piano.data.qpos[piano.piano.keys[midi]])
        np.testing.assert_array_equal(piano.joints[other], held)
    assert piano.piano.sequence["phase"] == "done"
    assert .0006 < max_depth <= .008
    events = [e for e in piano.piano.snapshot(after)["events"] if e["midi"] == midi]
    assert events and events[0]["type"] == "on" and events[-1]["type"] == "off"
    assert all(e["source"] == "aero_contact" for e in events)
    assert events[0]["finger"] == finger and events[0]["hand"] == side
    assert midi not in piano.piano.pressed
    assert piano.data.qpos[piano.piano.keys[midi]] == 0


def test_contact_hold_no_retrigger_pause_and_stop(piano):
    after = reset_midpoint(piano)
    piano.command({"action":"piano_play","arm":"right","midi":60,"interpolation_steps":180,
                   "velocity_limits":[2]*7,"acceleration_limits":[20]*7})
    for _ in range(190): piano.advance(1/60)
    assert 60 in piano.piano.pressed
    piano.command({"action":"pause"})
    fingers = dict(piano.piano.values["right"])
    for _ in range(10): piano.advance(1/60)
    assert piano.piano.values["right"] == fingers
    assert len([e for e in piano.piano.snapshot(after)["events"] if e["midi"]==60 and e["type"]=="on"]) == 1
    with pytest.raises(ValueError): piano.command({"action":"piano_play","midi":62})
    piano.command({"action":"stop"})
    assert not piano.piano.busy and not piano.piano.pressed
    assert piano.piano.snapshot(after)["events"][-1]["type"] == "off"
    # Manual finger movement at the hover position also produces real contacts.
    after = piano.piano.event_id
    piano.command({"action":"piano_finger","arm":"right","finger":"index","amount":.175})
    for _ in range(20): piano.advance(1/60)
    assert 60 in piano.piano.pressed
    assert len([e for e in piano.piano.snapshot(after)["events"] if e["type"]=="on"]) == 1
    piano.command({"action":"piano_release"})
    assert not piano.piano.pressed


def test_invalid_performance_commands_are_atomic(piano):
    reset_midpoint(piano)
    before = piano.snapshot()
    for command in ({"action":"piano_play","midi":20}, {"action":"piano_play","midi":True},
                    {"action":"piano_play","midi":109}, {"action":"piano_play","midi":60,"arm":"both"},
                    {"action":"piano_finger","arm":"right","finger":"index","amount":2},
                    {"action":"piano_finger","arm":"right","finger":"index","amount":float('nan')},
                    {"action":"piano_play","midi":60,"interpolation_steps":1}):
        with pytest.raises((ValueError, RuntimeError)): piano.command(command)
        assert piano.snapshot() == before


def test_score_independent_arms_atomic_chord_and_absolute_timing(piano):
    reset_midpoint(piano)
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = ('left_arm_move(17,2)\nright_arm_move(28,1)\nsleep(1)\n'
            'hand(D,7)\nsleep(.3)\nhand(U,7)\nsleep(.7)\n'
            'hand(D,1x)\nsleep(.2)\nhand(U,1x)\nsleep(.1)')
    before = piano.piano.event_id
    ready = piano.command({'action':'piano_score_prepare','text':text,'finger_count':4,
                           'velocity_limit':2.,'acceleration_limit':20.})
    piano.command({'action':'piano_score_start','token':ready['token']})
    assert set(piano.score.motions)=={'left','right'}
    for ms in range(10,2301,10):
        clock[0]=ms*1_000_000
        piano.advance(1/60)
        if ms==1000:
            assert 'left' in piano.score.motions and 'right' not in piano.score.motions
            assert piano.score.held=={'7'}
        if ms==1200:
            assert piano.score.held=={'7'}  # No implicit 0.1-second release.
        if ms==2000:
            assert not piano.score.motions and piano.score.held=={'1','x'}
    assert piano.score.status=='COMPLETED' and not piano.piano.pressed
    assert piano.elapsed==2.3
    events=piano.piano.snapshot(before)['events']
    chord=[e for e in events if e['type']=='on' and e['midi'] in (48,72)]
    assert {e['midi'] for e in chord}=={48,72}
    assert len({e['time_s'] for e in chord})==1


def test_score_timeout_stops_releases_and_never_catches_up(piano):
    reset_midpoint(piano)
    clock=[0];piano.score.clock=lambda:clock[0]
    text='right_arm_move(28,1)\nsleep(1)\nhand(D,7)\nsleep(1)\nhand(U,7)'
    ready=piano.command({'action':'piano_score_prepare','text':text,'finger_count':4})
    piano.command({'action':'piano_score_start','token':ready['token']})
    for ms in range(10,1011,10):
        clock[0]=ms*1_000_000;piano.advance(1/60)
    assert piano.score.held=={'7'} and piano.piano.pressed
    clock[0]+=200_000_000;piano.advance(1/60)
    assert piano.score.status=='ERROR' and piano.state=='STOPPED'
    assert not piano.piano.pressed and not piano.score.held
    events=piano.piano.event_id
    clock[0]+=5_000_000_000;piano.advance(1/60)
    assert piano.piano.event_id==events  # No stale note-ons dispatched later.


def test_score_preflight_rejects_limits_and_changed_start_pose(piano):
    reset_midpoint(piano)
    q={s:piano.joints[s].copy() for s in piano.SIDES}
    with pytest.raises(ValueError,match='未延长时间'):
        piano.command({'action':'piano_score_prepare','finger_count':4,
                       'text':'right_arm_move(35,.01)\nsleep(.01)'})
    for side in piano.SIDES:np.testing.assert_array_equal(q[side],piano.joints[side])
    ready=piano.command({'action':'piano_score_prepare','finger_count':4,
                         'text':'right_arm_move(28,2)\nsleep(2)'})
    piano.joints['right'][0]+=.001
    with pytest.raises(ValueError,match='姿态已改变'):
        piano.command({'action':'piano_score_start','token':ready['token']})
    reset_midpoint(piano)


def test_score_stop_cancels_future_actions(piano):
    reset_midpoint(piano)
    clock=[0];piano.score.clock=lambda:clock[0]
    ready=piano.command({'action':'piano_score_prepare','finger_count':4,
                         'text':'right_arm_move(28,1)\nsleep(1)\nhand(D,7)\nsleep(.1)\nhand(U,7)'})
    piano.command({'action':'piano_score_start','token':ready['token']})
    with pytest.raises(ValueError):piano.command({'action':'piano_finger','arm':'right','finger':'index','amount':.2})
    piano.command({'action':'stop'})
    with pytest.raises(ValueError):
        piano.command({'action':'piano_score_start','token':ready['token']})
    before=piano.joints['right'].copy()
    clock[0]=2_000_000_000;piano.advance(1/60)
    assert piano.score.status=='STOPPED' and not piano.piano.pressed
    np.testing.assert_array_equal(before,piano.joints['right'])


def test_score_right_24_retries_wrist_pose_and_produces_correct_chord(piano):
    reset_midpoint(piano)
    piano.score.position_cache.pop(('right', 24), None)
    piano.piano.pose_cache.pop(('right', 'index', 60), None)
    before_q = {s: piano.joints[s].copy() for s in piano.SIDES}
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = ('# regression: previously failed on line 5\n# four fingers\n\n\n'
            'right_arm_move(24,5)\nsleep(5)\n'
            'hand(D,789x)\nsleep(.1)\nhand(U,789x)')
    ready = piano.command({'action': 'piano_score_prepare', 'text': text, 'finger_count': 4})
    assert ready['duration_us'] == 5_100_000
    position = piano.score.position_cache['right', 24]
    assert abs(position['wrist_seed_offset_rad']) <= .18
    for side in piano.SIDES:
        np.testing.assert_array_equal(piano.joints[side], before_q[side])
    q, limits = position['q'], piano.arms['right'].limits
    assert np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1])
    before = piano.piano.event_id
    piano.command({'action': 'piano_score_start', 'token': ready['token']})
    for ms in range(10, 5101, 10):
        clock[0] = ms * 1_000_000
        piano.advance(1/60)
        if ms == 5000:
            assert piano.piano.pressed == {60, 62, 64, 65}
    assert piano.score.status == 'COMPLETED'
    assert not piano.piano.pressed and not piano.score.held
    on = [event for event in piano.piano.snapshot(before)['events'] if event['type'] == 'on']
    assert {event['midi'] for event in on} == {60, 62, 64, 65}
    assert len(on) == 4 and len({event['time_s'] for event in on}) == 1
    reset_midpoint(piano)


def test_score_automatic_motion_preserves_fast_score_timing(piano):
    reset_midpoint(piano)
    text = 'right_arm_move(29,.15)\nsleep(.15)\nhand(D,7)\nsleep(.1)\nhand(U,7)'
    original = {side: piano.joints[side].copy() for side in piano.SIDES}
    ready = piano.command({'action': 'piano_score_prepare', 'text': text,
                           'finger_count': 4, 'velocity_limit': .01,
                           'acceleration_limit': .01, 'automatic_motion': True})
    assert ready['automatic_motion'] is True
    assert ready['duration_us'] == 250_000
    assert ready['peak_velocity_rad_s'] > .01
    assert ready['peak_acceleration_rad_s2'] > .01
    assert piano.score.prepared['score'].actions[0]['duration_us'] == 150_000
    for side in piano.SIDES:
        np.testing.assert_array_equal(original[side], piano.joints[side])
    clock = [0]
    piano.score.clock = lambda: clock[0]
    piano.command({'action': 'piano_score_start', 'token': ready['token']})
    for ms in range(10, 251, 10):
        clock[0] = ms * 1_000_000
        piano.advance(1/60)
    assert piano.score.status == 'COMPLETED'
    assert not piano.piano.pressed
    reset_midpoint(piano)


@pytest.mark.parametrize('speed', [.5, 1., 2.])
def test_score_rate_scales_motion_and_contact_timestamps(piano, speed):
    reset_midpoint(piano)
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = 'right_arm_move(24,1)\nsleep(1)\nhand(D,7)\nsleep(.2)\nhand(U,7)'
    ready = piano.command({'action': 'piano_score_prepare', 'text': text,
                           'finger_count': 4, 'automatic_motion': True})
    before = piano.piano.event_id
    piano.command({'action': 'piano_score_start', 'token': ready['token'], 'rate': speed})
    for ms in range(5, round(1200/speed)+1, 5):
        clock[0] = ms*1_000_000
        piano.advance(1/60)
    assert piano.score.status == 'COMPLETED'
    events = [event for event in piano.piano.snapshot(before)['events'] if event['midi'] == 60]
    assert [event['type'] for event in events] == ['on', 'off']
    assert events[1]['time_s']-events[0]['time_s'] == pytest.approx(.2/speed)
    assert not piano.piano.pressed
    reset_midpoint(piano)


def test_score_pause_freezes_motion_and_chord_then_resumes_at_new_rates(piano):
    reset_midpoint(piano)
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = 'right_arm_move(24,1)\nsleep(1)\nhand(D,789x)\nsleep(.4)\nhand(U,789x)'
    ready = piano.command({'action': 'piano_score_prepare', 'text': text,
                           'finger_count': 4, 'automatic_motion': True})
    piano.command({'action': 'piano_score_start', 'token': ready['token']})
    for _ in range(20):
        clock[0] += 10_000_000
        piano.advance(1/60)
    piano.command({'action': 'pause'})
    frozen = piano.joints['right'].copy()
    frozen_time = piano.piano.time
    clock[0] += 5_000_000_000
    piano.advance(1/60)
    assert piano.score.status == 'PAUSED'
    assert piano.score.elapsed_us == 200_000
    assert piano.piano.time == frozen_time
    np.testing.assert_array_equal(frozen, piano.joints['right'])
    np.testing.assert_array_equal(piano.velocities['right'], np.zeros(7))
    with pytest.raises(ValueError):
        piano.command({'action': 'piano_score_prepare', 'text': text, 'finger_count': 4})
    piano.command({'action': 'piano_score_rate', 'rate': 2.})
    piano.command({'action': 'resume'})
    for _ in range(40):
        clock[0] += 10_000_000
        piano.advance(1/60)
    assert piano.score.elapsed_us == 1_000_000
    assert piano.score.held == set('789x')
    piano.command({'action': 'piano_score_pause'})
    events = piano.piano.event_id
    clock[0] += 8_000_000_000
    piano.advance(1/60)
    assert piano.piano.pressed == {60, 62, 64, 65}
    assert piano.piano.event_id == events
    piano.command({'action': 'piano_score_rate', 'rate': .5})
    piano.command({'action': 'piano_score_resume'})
    assert piano.piano.event_id == events
    for _ in range(80):
        clock[0] += 10_000_000
        piano.advance(1/60)
    assert piano.score.status == 'COMPLETED'
    assert not piano.piano.pressed
    reset_midpoint(piano)


def test_score_live_rate_change_is_continuous_and_pause_stop_releases(piano):
    reset_midpoint(piano)
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = 'right_arm_move(24,1)\nsleep(1)\nhand(D,7)\nsleep(1)\nhand(U,7)'
    ready = piano.command({'action': 'piano_score_prepare', 'text': text,
                           'finger_count': 4, 'automatic_motion': True})
    piano.command({'action': 'piano_score_start', 'token': ready['token']})
    for _ in range(20):
        clock[0] += 10_000_000
        piano.advance(1/60)
    before = piano.joints['right'].copy()
    piano.command({'action': 'piano_score_rate', 'rate': 2.})
    np.testing.assert_array_equal(before, piano.joints['right'])
    for invalid in [None, True, '2', 0, .49, 2.01, float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            piano.command({'action': 'piano_score_rate', 'rate': invalid})
    assert piano.score.rate == 2.
    for _ in range(40):
        clock[0] += 10_000_000
        piano.advance(1/60)
    assert piano.score.elapsed_us == 1_000_000
    piano.command({'action': 'pause'})
    piano.command({'action': 'stop'})
    assert piano.score.status == 'STOPPED'
    assert not piano.piano.pressed and not piano.score.held
    with pytest.raises(ValueError):
        piano.command({'action': 'piano_score_resume'})
    reset_midpoint(piano)


def test_score_fast_notes_crossed_in_one_tick_keep_order_and_duration(piano):
    reset_midpoint(piano)
    clock = [0]
    piano.score.clock = lambda: clock[0]
    text = 'right_arm_move(24,.2)\nsleep(.2)\nhand(D,7)\nsleep(.002)\nhand(U,7)'
    ready = piano.command({'action': 'piano_score_prepare', 'text': text,
                           'finger_count': 4, 'automatic_motion': True})
    piano.command({'action': 'piano_score_start', 'token': ready['token'], 'rate': 2.})
    for _ in range(10):
        clock[0] += 9_000_000
        piano.advance(1/60)
    before = piano.piano.event_id
    clock[0] += 15_000_000
    piano.advance(1/60)
    events = [event for event in piano.piano.snapshot(before)['events'] if event['midi'] == 60]
    assert [event['type'] for event in events] == ['on', 'off']
    assert events[1]['time_s']-events[0]['time_s'] == pytest.approx(.001)
    assert piano.score.status == 'COMPLETED'
    reset_midpoint(piano)
