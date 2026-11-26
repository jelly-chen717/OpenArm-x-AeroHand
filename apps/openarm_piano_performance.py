"""Compile a piano score into one arm and hand performance at score tempo."""
from bisect import bisect_right
from collections import defaultdict
from dataclasses import dataclass
from math import ceil, dist, radians

from openarm_control.config import ARM_JOINT_VELOCITY_LIMITS_RAD_S
from openarm_execution import rated_limits
from openarm_piano_air import CHORD_SCORE, CHORUS_SCORE, MELODY_SCORE, PIANO, PREVIEW_US, attacks, load_notes
from openarm_piano_origin import OFFSETS, SIDES, PianoFingerProjection, load_origin, origin_step
from openarm_piano_wrist import US_PER_S, WristPath, WristProfile, WristTransfer
from openarm_start_pose import PoseStep
from paino_skill.controller import music_files
from paino_skill.performance import note_name
from tools.aero_score.core import CHANNEL, Event, FingerScore

PRIORITY_SCORES = {"jin": MELODY_SCORE, "jin-chorus": CHORUS_SCORE, "jin-chords": CHORD_SCORE,
                   "jin-full": PIANO / "music_data/槿（60 BPM 完整钢琴版）.txt",
                   "little-apple": PIANO / "music_data/小苹果.txt"}
SCOPES = ("first", "prelude", "full")
PERFORMANCE_SPEED = {"jin": .8, "little-apple": .8}
RETIMED_SCORES = frozenset((*PERFORMANCE_SPEED, "jin-chords"))
APPROACH_S = 3.
ENTRY_WAIT_S = 2.
LIFT_S = .25
PASSIVE = (None,) * 7
WRIST_SETTLE_US = 40_000
WRIST_SPEED_RAD_S = 2.2
ADJACENT_SPEED_RAD_S = 4.4


def _score_sides(notes):
    """Hands the raw score writes notes for. Display metadata only: the hand
    channel still decides what actually runs from the compiled fingers."""
    return tuple(side for side in SIDES if any(note["side"] == side for note in notes))


def _score_library():
    scores, sides = dict(PRIORITY_SCORES), {}
    for path in music_files():
        if path in PRIORITY_SCORES.values():
            continue
        try:
            notes = load_notes(path)[0]
        except (OSError, ValueError):
            continue
        scores[path.stem] = path
        sides[path.stem] = _score_sides(notes)
    for score_id, path in PRIORITY_SCORES.items():
        try:
            notes = load_notes(path)[0]
        except (OSError, ValueError):
            continue
        sides[score_id] = _score_sides(notes)
    return scores, sides


SCORES, SCORE_SIDES = _score_library()


def performance_options():
    return {score_id: dict(title=path.stem, available=True, notice=None,
                           approach_s=APPROACH_S, wait_s=ENTRY_WAIT_S,
                           speed=PERFORMANCE_SPEED.get(score_id, 1.),
                           sides=list(SCORE_SIDES.get(score_id, SIDES)))
            for score_id, path in SCORES.items()}


@dataclass(frozen=True)
class Passage:
    start: int
    end: int
    notes: tuple


@dataclass(frozen=True)
class Arrangement:
    points: dict
    anchors: dict
    points_sha: str
    notes: list
    duration: int
    score_sha: str
    first_onset: int
    parts: dict
    travels: dict
    skipped: list


def _passages(notes, side, first_onset):
    """A wrist stays fixed through all notes with overlapping held durations."""
    groups = []
    end = -1
    for onset, chord in attacks(n for n in notes if n["side"] == side):
        if onset >= end:
            groups.append([])
        groups[-1].extend(chord)
        end = max(end, *(n["off_us"] for n in chord))
    return [Passage(group[0]["on_us"] - first_onset,
                    max(n["off_us"] for n in group) - first_onset, tuple(group))
            for group in groups]


def _speed_limits(mappings, side):
    return [min(limit, rated_limits(j)["max_speed"] * mappings[side, j].scale)
            for j, limit in enumerate(ARM_JOINT_VELOCITY_LIMITS_RAD_S, 1)]


def _move_us(start, finish, limits):
    # Rest-to-rest quintic peak velocity is 1.875 times displacement / time.
    return ceil(1.875 * US_PER_S * max(abs(x-y)/limit for x, y, limit in zip(start, finish, limits)))


def _travel_times(points, limits):
    positions = {key: tuple(map(radians, point.deg)) for key, point in points.items()}
    return {(a, b): _move_us(start, finish, limits)
            for a, start in positions.items() for b, finish in positions.items()}


def _select_wrists(passages, points, offsets, home, travel):
    """Prefer a chain of positions with fewer 1× timing deficits, then less travel.

    A passage no wrist can cover is skipped instead of aborting the score; the
    caller reports the skipped notes and keeps playing everything else.
    """
    costs, parents = {home: (0, 0)}, []
    previous, kept, skipped = None, [], []
    for group in passages:
        keys = {n["key"] for n in group.notes}
        choices = [key for key in points if all(note-key in offsets for note in keys)]
        if not choices:
            skipped.append(group)
            continue
        kept.append(group)
        step, updated = {}, {}
        for key in choices:
            def candidate(old):
                misses, distance = costs[old]
                needed = travel[old, key]
                delayed = previous is not None and old != key and needed + WRIST_SETTLE_US > group.start - previous.end
                return misses + delayed, distance + needed

            parent = min(costs, key=candidate)
            step[key], updated[key] = parent, candidate(parent)
        parents.append(step)
        previous, costs = group, updated
    keys = []
    key = min(costs, key=costs.get)
    for step in reversed(parents):
        keys.append(key)
        key = step[key]
    return list(zip(kept, reversed(keys))), skipped


def _scheduled_time(stamp, pauses, speed):
    return round(stamp / speed) + bisect_right(pauses, stamp) * US_PER_S


def _release_time(note, first_onset, pauses, speed):
    return (_scheduled_time(note["on_us"] - first_onset, pauses, speed)
            + round((note["off_us"] - note["on_us"]) / speed))


def _shared_pauses(parts, travels, first_onset, speed, *, lift_lead_us=0):
    """Insert common one-second rests where the wrist cannot arrive before the next press."""
    moves = [(after.start, before, travels[side][old, new])
             for side, strokes in parts.items()
             for (before, old), (after, new) in zip(strokes, strokes[1:]) if old != new]
    pauses = []
    for onset, before, needed in sorted(moves, key=lambda move: move[0]):
        release = max(_release_time(note, first_onset, pauses, speed) for note in before.notes)
        gap = _scheduled_time(onset, pauses, speed) - release
        while gap < needed + WRIST_SETTLE_US + lift_lead_us:
            pauses.append(onset)
            gap += US_PER_S
    return pauses


def _arm_and_finger_track(strokes, points, offsets, home, first_onset, pauses, travel, speed,
                          *, intro, profiles, synchronize):
    wrist = strokes[0][1] if strokes else home
    path, frames = [(0, points[wrist].deg)], defaultdict(dict)
    transfers = []
    last_release, over_budget = 0, 0
    for group, target in strokes:
        onset = _scheduled_time(group.start, pauses, speed)
        if wrist != target:
            gap = onset - last_release
            needed = travel[wrist, target]
            profile = profiles.get((wrist, target))
            # The arm controller samples every 40 ms. Reach the key one tick
            # before the serial hand schedule presses it.
            lift = min(WRIST_SETTLE_US, max(0, gap - needed - WRIST_SETTLE_US))
            begin, arrival = last_release + lift, onset - WRIST_SETTLE_US
            if synchronize:
                begin = max(begin, last_release + 1)
                arrival = begin + needed if profile is not None else max(arrival, begin + needed)
                transfers.append(WristTransfer(intro + begin, intro + arrival, intro + onset,
                                               points[target], profile))
            over_budget += needed > arrival - begin
            if begin > path[-1][0]:
                path.append((begin, points[wrist].deg))
            path.append((arrival, points[target].deg))
            wrist = target
        for note in group.notes:
            stamp = note["on_us"] - first_onset
            down = _scheduled_time(stamp, pauses, speed)
            up = _release_time(note, first_onset, pauses, speed)
            channel = CHANNEL[offsets[note["key"] - wrist]]
            frames[down][channel] = True
            frames[up][channel] = False
            last_release = max(last_release, up)
    return path, frames, over_budget, tuple(transfers)


def _transfer_profiles(points, parts, mappings, score_id):
    all_profiles, travels = {}, {}
    for side, strokes in parts.items():
        limits = tuple(min(WRIST_SPEED_RAD_S, limit) for limit in _speed_limits(mappings, side))
        adjacent_limits = tuple(min(ADJACENT_SPEED_RAD_S, rated_limits(j)["max_speed"] * mappings[side, j].scale)
                                for j in range(1, 8)) if score_id == "little-apple" else ()
        j7_max = mappings[side, 7].limits[1]
        profiles, needed = {}, {}
        for (_, old), (_, new) in zip(strokes, strokes[1:]):
            if old == new or (old, new) in profiles:
                continue
            boost = adjacent_limits if abs(new - old) == 1 else ()
            speed = boost or limits
            start, end = points[side][old].deg, points[side][new].deg
            profile = WristProfile.between(start, end, j7_max, boost)
            profiles[old, new] = profile
            needed[old, new] = profile.duration_us(start, end, speed)
        all_profiles[side], travels[side] = profiles, needed
    return all_profiles, travels


def arrange_score(score_id, mappings, *, sides=SIDES):
    if not isinstance(score_id, str) or score_id not in SCORES:
        raise ValueError("请选择控制台曲库中的琴谱。")
    points, anchors, points_sha = load_origin()
    score_keys = PianoFingerProjection.load(anchors).score_keys
    notes, duration, score_sha, _ = load_notes(SCORES[score_id])
    notes = [note for note in notes if note["side"] in sides]
    if not notes:
        return Arrangement(points, anchors, points_sha, notes, duration, score_sha, 0, {}, {}, [])
    first_onset = notes[0]["on_us"]
    parts, travels, skipped = {}, {}, []
    for side in sides:
        reachable = {wrist + offset for wrist in points[side] for offset in OFFSETS[side]}
        playable = []
        for note in (n for n in notes if n["side"] == side):
            key = note["key"]
            if key not in score_keys:
                skipped.append(dict(note, reason="outside_physical_keyboard"))
            elif key not in reachable:
                skipped.append(dict(note, reason="no_finger_at_solved_wrist"))
            else:
                playable.append(note)
        travels[side] = _travel_times(points[side], _speed_limits(mappings, side))
        parts[side], groups = _select_wrists(_passages(playable, side, first_onset), points[side], OFFSETS[side],
                                             anchors[side].key, travels[side])
        skipped.extend(dict(note, reason="no_common_wrist") for group in groups for note in group.notes)
    skipped.sort(key=lambda note: (note["on_us"], note["side"], note["key"]))
    return Arrangement(points, anchors, points_sha, notes, duration, score_sha, first_onset, parts, travels, skipped)


def _scope_end(arrangement, scope):
    if scope == "first":
        return next((n["on_us"] for n in arrangement.notes if n["on_us"] > arrangement.first_onset),
                    arrangement.duration)
    if scope == "prelude":
        return min(PREVIEW_US, arrangement.duration)
    if scope == "full":
        return arrangement.duration
    raise ValueError("请选择首和弦、前奏或全曲。")


def _point_details(point):
    return dict(note=point.note, physical_key=point.physical_key, xyz_mm=point.xyz, joints_deg=point.deg)


def fingering_preview(scope, score_id, mappings):
    arrangement = arrange_score(score_id, mappings)
    limit = _scope_end(arrangement, scope)
    points = arrangement.points
    assigned = {(n["side"], n["on_us"], n["key"]): key
                for strokes in arrangement.parts.values() for passage, key in strokes for n in passage.notes}
    skipped = {(n["side"], n["on_us"], n["key"]): n["reason"] for n in arrangement.skipped}
    rows, first_targets = [], {}
    for onset, chord in attacks(n for n in arrangement.notes if n["on_us"] < limit):
        fingers = {side: [] for side in SIDES}
        wrists = {}
        for note in chord:
            side, key = note["side"], note["key"]
            lookup = (side, onset, key)
            wrist = assigned.get(lookup)
            finger = None
            if wrist is not None:
                wrists[side] = wrist
                finger = OFFSETS[side][key - wrist]
                first_targets.setdefault(side, _point_details(points[side][wrist]))
            fingers[side].append(dict(key=key, physical_key=key - 9, midi=note["midi"], finger=finger,
                                      off_us=min(note["off_us"], limit), skip_reason=skipped.get(lookup)))
        pending = [side for side in SIDES if any(f["finger"] is None for f in fingers[side])]
        distance = (dist(points["left"][wrists["left"]].xyz, points["right"][wrists["right"]].xyz)
                    if len(wrists) == 2 else None)
        rows.append(dict(time_us=onset, notes=chord, wrists=wrists, fingers=fingers,
                         pending_wrists=pending, wrist_distance_mm=distance))
    groups = sum(bool(row["pending_wrists"]) for row in rows)
    return dict(scope=scope, score_id=score_id, score_sha256=arrangement.score_sha,
                points_sha256=arrangement.points_sha, duration_us=limit, first_wrist_targets=first_targets,
                note_count=sum(len(row["notes"]) for row in rows), unassigned=groups,
                pending_wrist_groups=groups, rows=rows)


def _excerpt(strokes, first_onset, limit):
    for passage, wrist in strokes:
        if passage.start + first_onset >= limit:
            break
        notes = tuple(dict(note, off_us=min(note["off_us"], limit))
                      for note in passage.notes if note["on_us"] < limit)
        yield Passage(passage.start, min(passage.end, limit - first_onset), notes), wrist


def compile_performance(score_id, held_plan, mappings, *, scope="full", speed=None, sides=SIDES, trial=False):
    arrangement = arrange_score(score_id, mappings, sides=sides)
    if speed is None:
        speed = 1. if trial else PERFORMANCE_SPEED.get(score_id, 1.)
    points, anchors = arrangement.points, arrangement.anchors
    first_onset = arrangement.first_onset
    limit = _scope_end(arrangement, scope)
    parts = {s: list(_excerpt(arrangement.parts.get(s, ()), first_onset, limit)) for s in sides}
    travels = arrangement.travels
    retimed = not trial and score_id in RETIMED_SCORES
    skipped = [dict(n, note=note_name(n["midi"])) for n in arrangement.skipped if n["on_us"] < limit]
    skipped_groups = len({(n["side"], n["on_us"]) for n in skipped})
    profiles, transfer_times = _transfer_profiles(points, parts, mappings, score_id) if retimed else ({}, travels)
    # Keep a release frame chronologically ahead of its lift readback, even
    # when the score leaves almost no time between adjacent phrases.
    pauses = _shared_pauses(parts, transfer_times, first_onset, speed, lift_lead_us=1 if retimed else 0)
    playing_us = max(0, _scheduled_time(limit - first_onset, pauses, speed))
    opening = {s: points[s][parts[s][0][1]] if parts[s] else anchors[s] for s in sides}
    held = (held_plan or {}).get("goal", {})
    at_first = trial and all(held.get((s, j)) == radians(q)
                             for s, point in opening.items() for j, q in enumerate(point.deg, 1))
    wait = (0. if at_first else WRIST_SETTLE_US / US_PER_S) if trial else ENTRY_WAIT_S
    approach = 0.
    steps = [PoseStep("抬指准备", PASSIVE, PASSIVE, LIFT_S)]
    right = opening["right"].deg if "right" in opening else PASSIVE
    left = opening["left"].deg if "left" in opening else PASSIVE
    if not at_first:
        origin = origin_step(anchors)
        steps.append(PoseStep(origin.name, origin.right_deg if "right" in sides else PASSIVE,
                              origin.left_deg if "left" in sides else PASSIVE, origin.duration_s))
        if any(opening[s].key != anchors[s].key for s in sides):
            needed = max(travels[s][anchors[s].key, opening[s].key] for s in sides)
            approach = max(APPROACH_S, needed / US_PER_S)
            steps.append(PoseStep("定位至首个手位", right, left, approach))
        steps.append(PoseStep(f"入位后等待 {wait:g} 秒", right, left, wait))
    intro = round(sum(step.duration_s for step in steps) * US_PER_S)

    paths, events = {}, {}
    used = {side: () for side in SIDES}
    over_budget = 0
    for side in sides:
        path, frames, short, transfers = _arm_and_finger_track(
            parts[side], points[side], OFFSETS[side], anchors[side].key, first_onset,
            pauses, transfer_times.get(side, {}), speed, intro=intro, profiles=profiles.get(side, {}),
            synchronize=not trial)
        if playing_us > path[-1][0]:
            path.append((playing_us, path[-1][1]))
        lift = None
        if retimed and approach and opening[side].key != anchors[side].key:
            lift = WristProfile.between(anchors[side].deg, opening[side].deg, mappings[side, 7].limits[1])
        paths[side] = WristPath(tuple((stamp / US_PER_S, q) for stamp, q in path), transfers, lift)
        over_budget += short
        used[side] = tuple(sorted({ch for frame in frames.values() for ch in frame}))
        if used[side]:
            home = used[side] if trial else sorted(CHANNEL.values())
            events[side] = (Event(0, (), tuple((ch, False) for ch in home)), *(
                Event(intro + stamp, (), tuple(sorted(frame.items()))) for stamp, frame in sorted(frames.items())))
    played = sum(len(group.notes) for strokes in parts.values() for group, _ in strokes)
    notice = (f"有 {len(skipped)} 个音超出实琴范围或无可用指法，已跳过。其余音符按谱演奏。") if skipped else None
    info = dict(performance=not trial, dual=len(sides) == 2, side=sides[0] if len(sides) == 1 else None,
                anchor_run=3, scope=scope, speed=speed, score_id=score_id,
                score_name=SCORES[score_id].stem, note_count=played, from_score_start=at_first,
                prepare_s=intro / US_PER_S, approach_s=approach, wait_s=wait, pause_times_us=pauses,
                moves_over_budget=over_budget,
                score_sha256=arrangement.score_sha, points_sha256=arrangement.points_sha, virtual_fingering=False,
                skipped_groups=skipped_groups, skipped_notes=len(skipped), skipped_details=skipped, skip_notice=notice,
                origin_targets={s: _point_details(anchors[s]) for s in sides},
                first_wrist_targets={s: _point_details(opening[s]) for s in sides})
    return steps, info, FingerScore(events, used, intro + playing_us, 0, played), paths
