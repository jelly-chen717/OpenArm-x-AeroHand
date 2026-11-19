"""Reproduce a piano review's touch events with a local MuJoCo virtual clock."""

import argparse
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

import numpy as np
from openarm_app.sim_server import ArmServerState


US_PER_S = 1_000_000


def check_sources(review):
    for name in ('score', 'piano_scene', 'joint_mapping'):
        source = review['sources'][name]
        if sha256(Path(source['path']).read_bytes()).hexdigest() != source['sha256']:
            raise ValueError(f'{name} has changed. Export a new review before replay.')


def retime(review, candidate, score_text):
    rate = review['trial_budget_unmeasured']['candidate_rate']
    source_lines = score_text.splitlines()
    commands = []
    for move in candidate['moves']:
        if move['trial_candidate_slack_s'] < 0:
            raise ValueError(f'Trial budget failed at score line {move["line"]}')
        start = move['trial_candidate_move_start_us']
        duration = move['trial_candidate_move_end_us'] - start
        commands.append((start, move['line'],
                         f'{move["side"]}_arm_move({move["index"]}, {duration / US_PER_S:.6f})'))
    for hand in review['finger_events']:
        line = hand['line']
        commands.append((round(hand['time_us'] / rate), line, source_lines[line - 1]))

    text, cursor = [], 0
    end = review['trial_duration_us']
    for time_us, _, line in sorted(commands + [(end, len(source_lines) + 1, '# End of score')]):
        if time_us > cursor:
            text.append(f'sleep({(time_us - cursor) / US_PER_S:.6f})')
        text.append(line)
        cursor = time_us
    return '\n'.join(text)


def note_ons(actions, rate):
    return Counter((round(action['time_us'] / rate), finger['midi']) for action in actions
                   if action['kind'] == 'hand' and action['direction'] == 'D'
                   for finger in action['fingers'])


def replay(review, candidate, score_text, *, trial):
    scene = review['sources']['piano_scene']['path']
    engine = ArmServerState(scene='piano', piano_config=scene)
    for position in candidate['hand_positions']:
        side, index = position['side'], position['index']
        q = np.array(position['q_model_rad'])
        engine.score.position_cache[side, index] = engine.score._validate_position(side, index, q)

    if trial:
        rate = review['trial_budget_unmeasured']['candidate_rate']
        text = retime(review, candidate, score_text)
        ready = engine.score.prepare(
            text, 4, velocity=max(review['optional_sdk_speed_cap_rad_s']),
            acceleration=review['trial_budget_unmeasured']['acceleration_rad_s2'],
        )
    else:
        rate = 1.
        ready = engine.score.prepare(score_text, 4, automatic_motion=True)
    wanted = note_ons(review['finger_events'], rate)
    if note_ons(ready['actions'], 1.) != wanted:
        raise ValueError('Prepared note timeline differs from the review')

    clock_ns = [0]
    engine.score.clock = lambda: clock_ns[0]
    last_event_id = engine.piano.event_id
    engine.score.start(ready['token'])
    actual, captured, now = Counter(), [], 0
    duration = ready['duration_us']
    moving_step = 10_000 if trial else 5_000
    while now < duration and engine.score.status == 'RUNNING':
        step = moving_step if engine.score.motions else 40_000
        now = min(duration, now + step)
        clock_ns[0] = now * 1_000
        engine.advance(step / US_PER_S)
        events = engine.piano.events
        if events and events[0]['id'] > last_event_id + 1:
            raise RuntimeError('Touch event buffer overran during replay')
        for event in reversed(events):
            if event['id'] <= last_event_id:
                break
            time_us = round(event['time_s'] * US_PER_S)
            captured.append({**event, 'time_us': time_us})
            if event['type'] == 'on':
                actual[time_us, event['midi']] += 1
        last_event_id = engine.piano.event_id

    missing, extra = wanted - actual, actual - wanted
    finished = engine.score.status == 'COMPLETED' and not engine.piano.pressed
    return {
        'rate': rate, 'duration_us': duration, 'status': engine.score.status, 'error': engine.score.error,
        'expected_note_ons': wanted.total(), 'actual_note_ons': actual.total(),
        'missing_count': missing.total(), 'extra_count': extra.total(),
        'first_missing': list(missing.items())[:10], 'first_extra': list(extra.items())[:10],
        'keys_released': not engine.piano.pressed,
        'passed': finished and not missing and not extra,
        'touch_events': sorted(captured, key=lambda event: event['id']),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('review', type=Path, help='JSON exported by plan_piano_hardware.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.review.read_bytes()
    review = json.loads(raw)
    if review.get('schema') != 'piano_posvel_offline_review_v1' or review.get('offline_only') is not True:
        parser.error('Input must be an offline piano review')
    check_sources(review)
    sources = [Path(review['sources'][name]['path']) for name in ('score', 'piano_scene', 'joint_mapping')]
    if args.output.resolve() in {args.review.resolve(), *(path.resolve() for path in sources)}:
        parser.error('Output would overwrite an input')
    score_text = sources[0].read_text(encoding='utf-8-sig')
    candidate = review.get('candidate', review)
    results = []
    for trial in (False, True):
        result = replay(review, candidate, score_text, trial=trial)
        results.append(result)
        print(f'{result["rate"]:g}x: {result["actual_note_ons"]}/{result["expected_note_ons"]} '
              f'note-ons, status={result["status"]}, passed={result["passed"]}', flush=True)
    proof = {'offline_only': True, 'review_sha256': sha256(raw).hexdigest(), 'replays': results}
    args.output.write_text(json.dumps(proof, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    if not all(result['passed'] for result in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
