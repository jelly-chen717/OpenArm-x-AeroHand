"""Verify actual console wrist and finger commands against every source note."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from build_openarm_piano_points import WristModel
from openarm_app.sim_server import ArmServerState
from openarm_joint_mapping import load_mapping
from openarm_monitor import PERIOD
from openarm_monitor_motion import MotionPlanner
from openarm_piano_air import MIDI_START, PREPARE_S
from openarm_piano_origin import load_origin
from openarm_piano_performance import ENTRY_WAIT_S, PERFORMANCE_SPEED, SCORES, US_PER_S, music_files
from paino_skill.performance import note_name
from paino_skill.score import WHITES, parse_score

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "logs/openarm/sync/piano_performance_latest.json"
# Servo channels 3–6 are index, middle, ring, pinky on both hands.
CODES = {"left": {3: "4", 4: "3", 5: "2", 6: "1"},
         "right": {3: "7", 4: "8", 5: "9", 6: "x"}}


def _source_events(path, info):
    text = path.read_text(encoding="utf-8-sig")
    source = parse_score(text, finger_count=4)
    first = min(a["time_us"] for a in source.actions if a.get("direction") == "D")
    header = MIDI_START.search(text)
    source_intro = (int(header[1]) * US_PER_S + int((header[2] or "").ljust(6, "0"))
                    if header else round(PREPARE_S * US_PER_S))
    parsed_skips = {(n["side"], n["line"], n["key"]): n for n in info["skipped_details"]}
    intro = round(info["prepare_s"] * US_PER_S)
    speed, pauses = info["speed"], info["pause_times_us"]
    expected, active, discarded, high = Counter(), {}, [], {}
    for action in source.actions:
        if action["kind"] != "hand":
            continue
        at = action["time_us"]
        for finger in action["fingers"]:
            code, side, key = finger["id"], finger["side"], finger["key"]
            if action["direction"] == "D":
                relative = at - first
                take = intro + round(relative / speed) + bisect_right(pauses, relative) * US_PER_S
                skip = parsed_skips.pop((side, action["line"], key), None)
                active[code] = (take, at, finger["midi"], skip is not None)
                if skip:
                    assert skip["on_us"] == at - source_intro
                    discarded.append(skip)
                else:
                    expected[take, "on", side, finger["midi"]] += 1
                    if side == "right" and key in (43, 44, 45):
                        high[take, key] = dict(source_line=action["line"], source_on_us=at - source_intro,
                                               source_finger_code=code)
            elif code in active:
                take, begin, midi, skipped = active.pop(code)
                if not skipped:
                    expected[take + round((at - begin) / speed), "off", side, midi] += 1
    assert not active and not parsed_skips
    return expected, discarded, high, {
        "arm_moves": dict(Counter(a["side"] for a in source.actions if a["kind"] == "move")),
        "finger_codes": sorted({f["id"] for a in source.actions if a.get("direction") == "D" for f in a["fingers"]}),
        "source_first_press_us": first,
        "source_leading_rest_us": first - source_intro,
        "score_duration_from_first_press_us": source.duration_us - first,
    }


def _moves(plan):
    return {arm.keys[0][0]: [(round(t0 * US_PER_S), round(t1 * US_PER_S), q1)
                            for (t0, q0), (t1, q1) in zip(arm.points, arm.points[1:]) if q0 != q1]
            for arm in plan["trajectory"].arms}


def _console_events(plan, wrists, anchors, pitch_mm, *, quantize):
    score = plan["trial_fingers"]
    trajectory = plan["trajectory"]
    curves = {arm.keys[0][0]: arm for arm in trajectory.arms}
    moves = _moves(plan)
    events, high, incidents = Counter(), [], []
    same_hand_moves = 0
    other_hand_moves = 0
    overlapping_notes = 0
    period_us = round(PERIOD * US_PER_S)
    for side in score.sides:
        held = {}
        fk_cache = {}
        other = "right" if side == "left" else "left"
        segments = moves[side]
        other_segments = moves[other]
        end_times = [end for _, end, _ in segments]
        other_ends = [end for _, end, _ in other_segments]
        for event in score.events[side][1:]:
            sample = event.time_us // period_us * period_us if quantize else event.time_us
            q = curves[side].sample(sample / US_PER_S)[0]
            if q not in fk_cache:
                y_mm = float(wrists[side].forward(q)[1]) * 1_000
                fk_cache[q] = anchors[side].key + math.floor((anchors[side].xyz[1] - y_mm) / pitch_mm + .5)
            wrist_key = fk_cache[q]
            for channel, down in event.changes:
                offset = channel - 3
                key = wrist_key + (offset if side == "right" else -offset)
                midi = WHITES[key - 1] if 1 <= key <= len(WHITES) else -1
                action = (event.time_us, "on" if down else "off", side, midi)
                events[action] += 1
                if down:
                    if channel in held:
                        incidents.append(dict(kind="repressed_while_held", side=side, time_us=event.time_us,
                                              channel=channel))
                    held[channel] = (event.time_us, midi)
                    overlapping_notes += len(held) - 1
                    if side == "right" and key in (43, 44, 45):
                        high.append(dict(time_us=event.time_us, note=note_name(midi), score_key=key,
                                         wrist_note=note_name(WHITES[wrist_key - 1]), wrist_key=wrist_key,
                                         finger_code=CODES[side][channel], channel=channel))
                    continue
                press = held.pop(channel, None)
                if press is None or press[1] != midi:
                    incidents.append(dict(kind="release_pitch", pressed=press, released=midi, side=side,
                                          time_us=event.time_us, channel=channel))
                if press is not None:
                    onset = press[0]
                    for begin, end, _ in segments[bisect_right(end_times, onset):]:
                        if begin >= event.time_us:
                            break
                        same_hand_moves += 1
                        incidents.append(dict(kind="same_hand_move_while_held", side=side, midi=press[1],
                                              on_us=onset, off_us=event.time_us, move_us=(begin, end)))
                    for begin, _, _ in other_segments[bisect_right(other_ends, onset):]:
                        if begin >= event.time_us:
                            break
                        other_hand_moves += 1
        if held:
            incidents.append(dict(kind="held_at_end", side=side, channels=list(held)))
    return events, high, dict(wrist_moves={side: len(segments) for side, segments in moves.items()},
                              overlapping_notes=overlapping_notes, same_hand_moves_during_hold=same_hand_moves,
                              opposite_hand_moves_during_hold=other_hand_moves, incidents=incidents)


def _differences(expected, actual):
    def by_time(events, direction):
        groups = defaultdict(list)
        for (time, kind, side, midi), count in sorted(events.items()):
            if kind == direction:
                groups[time, side].extend([midi] * count)
        return groups

    missing = by_time(expected - actual, "on")
    extra = by_time(actual - expected, "on")
    wrong = []
    for time, side in missing.keys() & extra.keys():
        matched = min(len(missing[time, side]), len(extra[time, side]))
        for written, played in zip(missing[time, side][:matched], extra[time, side][:matched], strict=True):
            wrong.append(dict(time_us=time, side=side, expected_midi=written, actual_midi=played))
        del missing[time, side][:matched]
        del extra[time, side][:matched]
    return dict(
        wrong_notes=sorted(wrong, key=lambda item: (item["time_us"], item["side"])),
        missing_notes=[dict(time_us=t, side=s, midi=m) for (t, s), notes in missing.items() for m in notes],
        extra_notes=[dict(time_us=t, side=s, midi=m) for (t, s), notes in extra.items() for m in notes],
        release_mismatches=[
            dict(classification=kind, time_us=t, side=s, midi=m)
            for kind, events in (("missing", expected-actual), ("extra", actual-expected))
            for (t, s), notes in by_time(events, "off").items() for m in notes],
    )


def _native_text(plan, points, anchors):
    """Carry the compiled wrist and finger sequence into the native 88-key model."""
    stamps = defaultdict(list)
    for side, segments in _moves(plan).items():
        offset = 3 if side == "left" else 0
        anchor = anchors[side].key - offset
        stamps[0].append(f"{side}_arm_move({anchor}, 0.25)")
        qkeys = {tuple(map(math.radians, point.deg)): key for key, point in points[side].items()}
        for begin, end, target in segments:
            index = qkeys[target] - offset
            stamps[begin].append(f"{side}_arm_move({index}, {(end-begin)/US_PER_S:.6f})")
    score = plan["trial_fingers"]
    for side in score.sides:
        for event in score.events[side][1:]:
            releases, presses = [], []
            for channel, down in event.changes:
                (presses if down else releases).append(CODES[side][channel])
            # Releases precede a same-stamp move. Any new press is after arrival.
            if releases:
                stamps[event.time_us].insert(0, f"hand(U,{''.join(releases)})")
            if presses:
                stamps[event.time_us].append(f"hand(D,{''.join(presses)})")
    stamps.setdefault(score.duration_us, [])
    last, records = 0, []
    for stamp, commands in sorted(stamps.items()):
        if stamp > last:
            records.append(f"sleep({(stamp-last)/US_PER_S:.6f})")
        records.extend(commands)
        last = stamp
    return "\n".join(records)


def _contact_replay(engine, text, expected, initial):
    engine.joints = {side: q.copy() for side, q in initial.items()}
    engine.refresh_model()
    prepared = engine.score.prepare(text, 4, automatic_motion=True)
    now = 0
    engine.score.clock = lambda: now * 1_000
    cursor = engine.piano.event_id
    start = engine.piano.time
    engine.score.start(prepared["token"])
    actual = Counter()
    while engine.score.status == "RUNNING" and now < prepared["duration_us"]:
        step = min(prepared["duration_us"] - now, 5_000 if engine.score.motions else 40_000)
        now += step
        engine.advance(step / US_PER_S)
        fresh = [event for event in reversed(engine.piano.events) if event["id"] > cursor]
        for event in reversed(fresh):
            assert event["id"] == cursor + 1
            cursor = event["id"]
            stamp = round((event["time_s"] - start) * US_PER_S)
            attrs = event["type"], event["hand"] if event["type"] == "on" else "", event["midi"]
            match = next(((stamp + i, *attrs) for i in (0, -1, 1)
                          if expected[stamp + i, *attrs] > actual[stamp + i, *attrs]), (stamp, *attrs))
            actual[match] += 1
    differences = _differences(expected, actual)
    passed = engine.score.status == "COMPLETED" and not engine.piano.pressed and actual == expected
    return dict(passed=passed, status=engine.score.status, error=engine.score.error,
                presses=sum(n for k, n in actual.items() if k[1] == "on"),
                releases=sum(n for k, n in actual.items() if k[1] == "off"),
                duration_us=prepared["duration_us"], keys_released_at_end=not engine.piano.pressed,
                **differences)


def _first_attack(events):
    first = min((t for t, action, _, _ in events if action == "on"), default=None)
    return dict(time_us=first, notes=[dict(side=s, midi=m, note=note_name(m)) for t, action, s, m in sorted(events)
                                     if t == first and action == "on" and m >= 0])


def verify(output, score_ids, *, simulate, piano_config=None):
    version, mappings = load_mapping(ROOT / "configs/openarm_joint_mapping.json", hardware=True)
    planner = MotionPlanner(version, mappings, virtual=False)
    wrists = {side: WristModel(arm) for side, arm in planner.arms.items()}
    points, anchors, digest = load_origin()
    start = {(side, j): math.radians(q) for side, point in anchors.items() for j, q in enumerate(point.deg, 1)}
    if simulate:
        engine = ArmServerState(scene="piano", piano_config=piano_config)
        native_start = {side: engine.score.hand_position(side, point.key - (3 if side == "left" else 0))["q"].copy()
                        for side, point in anchors.items()}
    results = {}
    failed = []
    for score_id in score_ids:
        file = SCORES[score_id]
        plan, _ = planner.prepare_piano_trial(dict(mode="piano-performance", score_id=score_id), start, None)
        info = plan["piano_trial"]
        assert info["wait_s"] == ENTRY_WAIT_S and info["speed"] == PERFORMANCE_SPEED.get(score_id, 1.)
        assert not info["moves_over_budget"]
        origin = next(target for label, _, target in plan["steps"] if label.startswith("钢琴原位"))
        assert origin == start
        expected, skipped, original_high, parsed = _source_events(file, info)
        exact, high, motion = _console_events(plan, wrists, anchors, 22.5, quantize=False)
        sampled, _, sampled_motion = _console_events(plan, wrists, anchors, 22.5, quantize=True)
        for note in high:
            note.update(original_high.get((note["time_us"], note["score_key"]), {}))
        first = _first_attack(exact)
        assert plan["trial_fingers"].duration_us - round(info["prepare_s"] * US_PER_S) == (
            round(parsed["score_duration_from_first_press_us"] / info["speed"])
            + len(info["pause_times_us"]) * US_PER_S)
        passed = sampled == exact == expected and not motion["incidents"] and not sampled_motion["incidents"]
        contacts = None
        if simulate:
            native_expected = Counter()
            for (stamp, action, side, midi), count in expected.items():
                native_expected[stamp, action, side if action == "on" else "", midi] += count
            try:
                contacts = _contact_replay(engine, _native_text(plan, points, anchors), native_expected, native_start)
            except (ValueError, RuntimeError, AssertionError) as exc:
                contacts = dict(passed=False, error=str(exc))
        if not passed or contacts is not None and not contacts["passed"]:
            failed.append(score_id)
        results[score_id] = {
            "score": file.name, "score_sha256": info["score_sha256"],
            "source": parsed, "source_notes": info["note_count"] + len(skipped),
            "played_notes": sum(n for k, n in exact.items() if k[1] == "on"),
            "speed": info["speed"], "first_wrist_targets": info["first_wrist_targets"],
            "wait_s": info["wait_s"], "prepare_s": info["prepare_s"],
            "pause_times_us": info["pause_times_us"], "first_attack": first,
            "expected_first_attack": _first_attack(expected), "controller_matches": passed,
            "notes_at_unreachable_index_keys": high, "skipped_notes": skipped,
            "scheduled_replay": dict(**motion, **_differences(expected, exact)),
            "controller_tick_replay": dict(**sampled_motion, **_differences(expected, sampled)),
            "mujoco_contacts": contacts,
        }
        print(score_id, "console", passed, "notes", info["note_count"], "skipped", len(skipped),
              "sync pauses", len(info["pause_times_us"]), "MuJoCo",
              contacts["passed"] if contacts is not None else "not run",
              contacts.get("error") if contacts and not contacts["passed"] else "", flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(points_sha256=digest, controller_tick_s=PERIOD,
                   simulation_config=engine.scene_config if simulate else None,
                   origin_notes={side: point.note for side, point in anchors.items()},
                   non_score_files=sorted(p.name for p in set(music_files()) - set(SCORES.values())),
                   controller_failures=[score for score, result in results.items() if not result["controller_matches"]],
                   simulation_failures=[score for score, result in results.items()
                                        if result["mujoco_contacts"] and not result["mujoco_contacts"]["passed"]],
                   scores=results, failed=failed)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if failed:
        raise SystemExit(f"Replay differences in {', '.join(failed)}. See {output}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--score", action="append", choices=SCORES, help="Verify only the named score (repeatable)")
    parser.add_argument("--controller-only", action="store_true", help="Skip the native MuJoCo contact replay")
    parser.add_argument("--piano-config", type=Path, help="Optional MuJoCo-only scene geometry for contact replay")
    args = parser.parse_args()
    verify(args.output, args.score or SCORES, simulate=not args.controller_only, piano_config=args.piano_config)
