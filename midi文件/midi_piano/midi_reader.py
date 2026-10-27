"""Read one explicit track/channel, retaining the global MIDI tempo map."""
from bisect import bisect_right
from dataclasses import asdict, dataclass
from pathlib import Path

import mido


@dataclass(frozen=True)
class Note:
    pitch: int
    start: float
    duration: float
    velocity: int


@dataclass
class Song:
    source: str
    track: int
    channel: int  # MIDI user numbering: 1..16
    notes: list[Note]
    end_seconds: float
    tempos: list[dict]
    warnings: list[str]

    def to_dict(self):
        return asdict(self)


def load(path):
    try:
        mid = mido.MidiFile(path)
    except (OSError, ValueError, EOFError, KeyError) as exc:
        raise ValueError(f"无法读取 MIDI：{exc}") from exc
    if mid.type not in (0, 1) or mid.ticks_per_beat <= 0:
        raise ValueError("仅支持 Type 0/1、PPQN 时间制 MIDI，不支持 Type 2 或 SMPTE")
    return mid


def streams(mid):
    result = []
    for i, track in enumerate(mid.tracks):
        counts = {}
        for msg in track:
            if msg.type == "note_on" and msg.velocity > 0:
                counts[msg.channel + 1] = counts.get(msg.channel + 1, 0) + 1
        for channel, count in sorted(counts.items()):
            result.append({"track": i, "name": track.name, "channel": channel, "notes": count})
    return result


def read_song(path, track=None, channel=None, transpose=0):
    mid = load(path)
    candidates = [s for s in streams(mid)
                  if (track is None or s["track"] == track)
                  and (channel is None or s["channel"] == channel)]
    if len(candidates) != 1:
        listing = "; ".join(f"track={s['track']} channel={s['channel']} ({s['notes']}音)" for s in streams(mid))
        raise ValueError("请选择唯一旋律音轨/通道，使用 --track 和 --channel。可选：" + (listing or "无音符"))
    selected = candidates[0]
    if selected["channel"] == 10:
        raise ValueError("通道 10 为打击乐通道，不能作为七音旋律")

    # All tracks contribute tempos, including an unselected conductor track.
    tempo_at = {0: 500000}
    explicit = {}
    for tr in mid.tracks:
        tick = 0
        for msg in tr:
            tick += msg.time
            if msg.type == "set_tempo":
                if msg.tempo <= 0:
                    raise ValueError("MIDI 速度必须大于零")
                if tick in explicit and explicit[tick] != msg.tempo:
                    raise ValueError(f"tick={tick} 存在冲突的速度事件")
                explicit[tick] = msg.tempo
                tempo_at[tick] = msg.tempo
    ticks = sorted(tempo_at)
    seconds = [0.0]
    for a, b in zip(ticks, ticks[1:]):
        seconds.append(seconds[-1] + mido.tick2second(b - a, mid.ticks_per_beat, tempo_at[a]))

    def at(tick):
        i = bisect_right(ticks, tick) - 1
        return seconds[i] + mido.tick2second(tick - ticks[i], mid.ticks_per_beat, tempo_at[ticks[i]])

    events = []
    tick = 0
    warnings = []
    for order, msg in enumerate(mid.tracks[selected["track"]]):
        tick += msg.time
        if getattr(msg, "channel", -1) != selected["channel"] - 1:
            continue
        if msg.type in ("note_on", "note_off"):
            on = msg.type == "note_on" and msg.velocity > 0
            events.append((tick, 1 if on else 0, order, msg))
        elif msg.type == "control_change" and msg.control == 64:
            warnings.append("延音踏板事件不参与演奏；时值由 Note On/Off 决定。")
        elif msg.type == "pitchwheel" and msg.pitch != 0:
            raise ValueError(f"tick={tick} 包含弯音，无法映射为固定七音")
    end_tick = tick
    active = None
    notes = []
    # Release before a new press at the same tick (valid repeated notes).
    for tick, kind, _, msg in sorted(events, key=lambda e: e[:3]):
        if kind:
            if active is not None:
                raise ValueError(f"{at(tick):.3f}s 存在和弦或音符重叠：{active[1]} 与 {msg.note}")
            active = (tick, msg.note, msg.velocity)
        else:
            if active is None or active[1] != msg.note:
                raise ValueError(f"{at(tick):.3f}s 音符 {msg.note} 的 Note Off 无对应 Note On")
            start, pitch, velocity = active
            if tick <= start:
                raise ValueError("音符时值必须大于零")
            target = pitch + transpose
            if not 0 <= target <= 127:
                raise ValueError("移调后的 MIDI 编号超出 0..127")
            notes.append(Note(target, at(start), at(tick) - at(start), velocity))
            active = None
    if active is not None:
        raise ValueError(f"音符 {active[1]} 缺少 Note Off")
    if not notes:
        raise ValueError("所选音轨/通道没有完整音符")
    return Song(str(Path(path).resolve()), selected["track"], selected["channel"], notes,
                at(end_tick), [{"at_seconds": at(t), "bpm": mido.tempo2bpm(tempo_at[t])} for t in ticks],
                list(dict.fromkeys(warnings)))
