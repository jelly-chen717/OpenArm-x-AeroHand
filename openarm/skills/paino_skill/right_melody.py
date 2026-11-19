"""Validated melody plan and shared absolute-time Cartesian/finger schedule."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PLAN = ROOT / 'docs/project/右手旋律逐音规划_20261117.json'
FINGERS = ['食指', '中指', '无名指', '小指']


def smooth(u):
    u = np.clip(u, 0., 1.)
    return u**3 * (10 - 15*u + 6*u*u)


def load_plan(path):
    p = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    events = p['events']
    if not p['offline_only'] or p['hardware_executable']:
        raise ValueError('Only offline plans are accepted')
    if len(events) != 52 or p['pitch_mm'] != 26 or p['note_interval_s'] != 1.5:
        raise ValueError('Expected the reviewed 52-note / 26 mm / 10 BPM plan')
    previous = 0
    for i, e in enumerate(events):
        f = FINGERS.index(e['finger'])
        if (e['index'] != i+1 or e['key'] != e['index_key']+f
                or e['y_mm'] != -(e['index_key']-19)*26
                or e['move_y_mm'] != e['y_mm']-previous
                or e['press_start_s'] != i*1.5):
            raise ValueError(f'Inconsistent event {i+1}')
        if e['move_y_mm'] and e['move_s'] != .75:
            raise ValueError('Move duration differs from reviewed plan')
        if i and e['move_y_mm'] and e['move_start_s'] != (i-1)*1.5+.5:
            raise ValueError('Move must begin after release')
        previous = e['y_mm']
    return p


def state_at(plan, t):
    """Score time starts at zero; -0.95 .. 0 is opening move + settling."""
    if plan.get('format')=='eop':
        from paino_skill.right_melody_eop import state
        return state(plan,t)
    y = 0.
    for i, e in enumerate(plan['events']):
        if not e['move_y_mm']:
            continue
        start = -.95 if i == 0 else e['move_start_s']
        if t < start:
            break
        y = (e['y_mm']-e['move_y_mm'] + e['move_y_mm']*smooth((t-start)/.75))/1000
        if t < start+.75:
            break
    fingers = np.zeros(4)
    active = None
    if 0 <= t < 78:
        e = plan['events'][min(int(t/1.5), 51)]
        phase = t-e['press_start_s']
        if phase < .5:
            # Ideal onset at the score boundary; release is smooth over .2 s.
            fingers[FINGERS.index(e['finger'])] = 1. if phase < .3 else 1-smooth((phase-.3)/.2)
            active = e
    return y, fingers, active


def time_grid(plan):
    if plan.get('format')=='eop':
        from paino_skill.right_melody_eop import grid
        return grid(plan)
    times = list(np.arange(-.95, 78, .02)) + [-.95, -.2, 0., 78.]
    for e in plan['events']:
        times.extend(e['press_start_s']+x for x in (0, .3, .5, 1.25, 1.45))
    return np.array(sorted(set(round(t, 8) for t in times if t <= 78)))

