import argparse
import json
from pathlib import Path
import sys

from .devices import Clock, SimArm, SimHand, VirtualClock, validate_hardware
from .executor import execute
from .midi_reader import load, read_song, streams
from .planner import ROOT, build_plan, read_config


def preview(plan):
    print(f"音符数：{len(plan['notes'])}；跨组换位：{plan['group_changes']} 次（另含首次定位）")
    print(f"原曲时长：{plan['source']['end_seconds']:.3f}s；计划总时长：{plan['total_seconds']:.3f}s")
    print(f"请求速度：{plan['requested_speed']:.3f}x；可执行速度：{plan['effective_speed']:.3f}x；时间倍率：{plan['time_scale']:.3f}")
    print("速度变化：" + "; ".join(f"{e['at_seconds']:.3f}s: {e['bpm']:.2f} BPM" for e in plan['tempos']))
    for warning in plan["warnings"]:
        print("提示：" + warning)
    print("序号 音名       MIDI 原始开始/时值(s) 计划触键/保持(s)  位置/手指  力度")
    for n in plan["notes"]:
        print(f"{n['index']:3} {n['name']:10} {n['pitch']:3}  {n['source_start']:7.3f}/{n['source_duration']:6.3f}  "
              f"{n['onset']:8.3f}/{n['hold']:6.3f}  {n['group']}/{n['finger']:6} {n['velocity']}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="七音单旋律 MIDI 演奏；默认模拟，不连接设备")
    parser.add_argument("command", nargs="?", choices=("inspect", "simulate", "run"), default="simulate")
    parser.add_argument("file", nargs="?", type=Path, default=ROOT / "examples" / "ode_to_joy.mid")
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--track", type=int, help="音轨编号，从 0 开始")
    parser.add_argument("--channel", type=int, choices=range(1, 17), help="MIDI 通道，1..16")
    parser.add_argument("--transpose", type=int, default=0, help="显式半音移调，如 -12")
    parser.add_argument("--speed", type=float, default=1, help="请求速度倍率；受动作速度约束自动降低")
    parser.add_argument("--list", action="store_true", help="仅列出音轨/通道")
    parser.add_argument("--fast", action="store_true", help="模拟使用虚拟时钟，立即运行完整计划")
    parser.add_argument("--output", type=Path, help="JSON 计划路径；默认在独立目录 output 下")
    args = parser.parse_args(argv)
    try:
        if args.command == "run" and (args.fast or args.list):
            raise ValueError("run 不接受 --fast 或 --list")
        if args.list:
            for stream in streams(load(args.file)):
                print(f"track={stream['track']} channel={stream['channel']} notes={stream['notes']} name={stream['name']}")
            return 0
        cfg = read_config(args.config)
        song = read_song(args.file, args.track, args.channel, args.transpose)
        plan = build_plan(song, cfg, args.speed)
        preview(plan)
        output = args.output or ROOT / "output" / f"{args.file.stem}.plan.json"
        # Prevent accidental overwrite of an input or a source/configuration file.
        if output.suffix.lower() != ".json" or not output.name.endswith(".plan.json"):
            raise ValueError("计划输出文件名必须以 .plan.json 结尾")
        if output.resolve() in (args.config.resolve(), args.file.resolve()):
            raise ValueError("不能覆盖输入文件")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"计划已保存：{output.resolve()}")
        if args.command == "inspect":
            return 0
        if args.command == "run":
            validate_hardware(cfg)
            raise RuntimeError("没有可用的真实机械臂执行器")
        print("模拟模式：不打开串口，不驱动机械臂；不会发出钢琴声音。" + (" 使用虚拟时钟。" if args.fast else " 按计划时间运行，Ctrl+C 停止。"))
        clock = VirtualClock() if args.fast else Clock()
        hand = SimHand(clock)
        execute(plan, cfg, hand, SimArm(clock, hand), clock)
        print("模拟完成：全部音符已执行，手指处于释放状态。")
        return 0
    except KeyboardInterrupt:
        print("已停止；执行器已尝试停止与释放。", file=sys.stderr)
        return 130
    except (Exception,) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
