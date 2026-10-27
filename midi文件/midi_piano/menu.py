"""Interactive console launcher; user paths are never evaluated by a shell."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def main():
    print("七音单旋律 MIDI 演奏\n1. 查看并保存演奏计划\n2. 按节奏模拟\n3. 快速模拟\n4. 列出音轨\n5. 实物运行（OpenArm 尚未接入，当前会阻止）\n0. 退出")
    choice = input("选择 [默认 3]：").strip() or "3"
    if choice == "0":
        return 0
    if choice not in ("1", "2", "3", "4", "5"):
        print("无效选项")
        return 1
    supplied = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else input("MIDI 路径（可拖入，留空使用欢乐颂）：").strip().strip('"')
    path = supplied or str(ROOT / "examples" / "ode_to_joy.mid")
    command = "inspect" if choice in ("1", "4") else "run" if choice == "5" else "simulate"
    args = [sys.executable, str(ROOT / "player.py"), command, path]
    if choice == "3":
        args.append("--fast")
    if choice == "4":
        args.append("--list")
    else:
        track = input("音轨编号（从 0 开始，留空自动选择唯一旋律）：").strip()
        channel = input("通道编号（1–16，留空自动选择）：").strip()
        transpose = input("移调半音数（如 -12，默认 0）：").strip()
        speed = input("速度倍率（默认 1，过快自动整曲降速）：").strip()
        for flag, value in (("--track", track), ("--channel", channel), ("--transpose", transpose), ("--speed", speed)):
            if value:
                args.extend([flag, value])
    return subprocess.run(args, check=False).returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("已退出")
        raise SystemExit(130)
