"""Fixed-position four-finger monophonic MIDI player; hardware only with --run."""
import argparse
import json
import math
from pathlib import Path
import time
import piano_four_player as hand
from midi_piano.midi_reader import read_song

ROOT = Path(__file__).resolve().parent
MAP = {60: 'index', 62: 'middle', 64: 'ring', 65: 'pinky'}

def plan(song, timing, speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('速度必须为正数')
    if not song.notes:
        raise ValueError('曲目没有音符')
    scale = 1 / speed
    overhead = timing['down_seconds'] + timing['up_seconds'] + timing['gap_seconds']
    for n in song.notes:
        if n.pitch not in MAP:
            raise ValueError(f'不支持音高 {n.pitch}；仅支持 C4/D4/E4/F4 (60/62/64/65)，不会删除音符')
        if n.duration <= 0:
            raise ValueError('音符时值必须大于零')
        scale = max(scale, (overhead + 0.05) / n.duration)
    return [dict(finger=MAP[n.pitch], start=n.start * scale,
                 hold=max(0.05, min(n.duration * scale * 0.7, n.duration * scale - overhead))) for n in song.notes], scale

def perform(motion, targets, timing, events):
    origin = motion.clock()
    for i, event in enumerate(events, 1):
        deadline = origin + event['start']
        if motion.clock() > deadline + 0.2:
            raise RuntimeError('调度明显迟到，停止演奏；不会快速补发动作')
        motion.until(deadline)
        print(f"{i}/{len(events)}: {event['finger']}", flush=True)
        motion.move(targets[event['finger']], timing['down_seconds'])
        motion.sleep(event['hold'])
        motion.move(targets['rest'], timing['up_seconds'])
        motion.sleep(timing['gap_seconds'])

def main():
    ap = argparse.ArgumentParser(description='四指 MIDI：默认模拟；--run 实际驱动')
    ap.add_argument('midi', nargs='?', type=Path)
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--fast', action='store_true')
    ap.add_argument('--speed', type=float, default=0.5)
    ap.add_argument('--track', type=int)
    ap.add_argument('--channel', type=int)
    args = ap.parse_args()
    if args.run and args.fast:
        ap.error('--fast 仅用于模拟')
    if args.midi is None:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        selected = filedialog.askopenfilename(title='选择四音单旋律 MIDI', initialdir=str(ROOT), filetypes=[('MIDI', '*.mid *.midi')])
        root.destroy()
        if not selected:
            return 0
        args.midi = Path(selected)
    transport = motion = targets = None
    failed = False
    try:
        cfg = json.loads((ROOT / 'piano_four_config.json').read_text(encoding='utf-8'))
        hand.validate(cfg, hardware=True)
        song = read_song(args.midi, track=args.track, channel=args.channel)
        events, scale = plan(song, cfg['timing'], args.speed)
        targets, constants = hand.build_targets(cfg)
        print(f'曲目：{args.midi.name}；{len(events)} 音；原时长 {song.end_seconds:.2f}s；时间倍率 {scale:.2f}')
        for warning in song.warnings:
            print(warning)
        if args.run:
            print('即将连接机械手并进入准备姿态。Ctrl+C 停止。', flush=True)
            transport = hand.Transport(cfg, constants)
            current = transport.read_pose()
            motion = hand.Motion(transport.send, current, cfg['timing']['update_hz'])
            motion.move(targets['rest'], cfg['timing']['startup_seconds'])
        elif args.fast:
            clock = [0.0]
            def sleep(s): clock[0] += s
            motion = hand.Motion(lambda p: None, targets['rest'], cfg['timing']['update_hz'], lambda: clock[0], sleep)
            print('快速模拟：不打开串口')
        else:
            motion = hand.Motion(lambda p: None, targets['rest'], cfg['timing']['update_hz'])
            print('实时模拟：不打开串口')
        perform(motion, targets, cfg['timing'], events)
        print('演奏指令执行完成；实物发声与回弹请现场确认。' if args.run else '模拟完成。')
    except KeyboardInterrupt:
        print('已停止，尝试恢复准备姿态。')
    except Exception as exc:
        print(f'错误：{exc}')
        failed = True
    finally:
        if transport is not None:
            try:
                if motion is not None:
                    motion.move(targets['rest'], cfg['timing']['stop_seconds'])
                    print('准备姿态指令已发送。')
            except (Exception, KeyboardInterrupt) as exc:
                print(f'复位未完成，请人工检查：{exc}')
                failed = True
            finally:
                transport.close()
    return int(failed)

if __name__ == '__main__':
    raise SystemExit(main())
