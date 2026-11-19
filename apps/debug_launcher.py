"""Workspace menu for the feedback mirror and simulation frontends."""

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / "openarm/.venv/Scripts/python.exe"
START_SIMULATION = ROOT / "openarm/tools/start_simulation.py"
MIRROR = ROOT / "apps/mirror_openarm_mujoco.py"


def commands():
    return {
        "1": [str(PYTHON), str(MIRROR), "--mode", "hardware", "--side", "both",
              "--left-channel", "0x51", "--right-channel", "0x52",
              "--confirm-bus-ownership", "--period", "0.03", "--max-age", "0.15",
              "--max-skew", "0.05", "--duration", "86400"],
        "2": [str(PYTHON), str(START_SIMULATION), "--scene", "default"],
        "3": [str(PYTHON), str(START_SIMULATION), "--scene", "piano"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Validate entry paths and exit")
    parser.add_argument("--run", choices=("1", "2", "3"), help=argparse.SUPPRESS)
    args = parser.parse_args()
    for path in (PYTHON, START_SIMULATION, MIRROR):
        if not path.is_file():
            print(f"缺少入口文件 {path}")
            return 1
    if args.check:
        print("三个入口路径检查通过。")
        return 0
    if args.run:
        try:
            code = subprocess.call(commands()[args.run], cwd=ROOT)
        except KeyboardInterrupt:
            return 0
        except OSError as exc:
            print(f"启动失败 {exc}", flush=True)
            code = 1
        if code:
            print(f"入口 {args.run} 启动或运行失败（退出码 {code}）。请查看上方错误及日志路径。", flush=True)
            try:
                input("按回车关闭此窗口……")
            except (EOFError, KeyboardInterrupt):
                pass
        return code

    processes = {}
    entries = commands()
    while True:
        print("\nOpenArm 统一调试入口\n"
              "1  双臂实机反馈镜像（CAN查询，左0x51 / 右0x52）\n"
              "2  网页关节调试 + MuJoCo（http://127.0.0.1:8088）\n"
              "3  钢琴演奏 + MuJoCo（http://127.0.0.1:8090）\n"
              "0  退出菜单\n\n"
              "镜像关闭MuJoCo窗口后释放CAN。网页调试与钢琴共用8769端口，切换前关闭前一个窗口。\n"
              "实机镜像运行期间，由它独占两路CAN。菜单退出后已打开的终端继续运行。")
        try:
            choice = input("请选择 > ").strip()
        except (EOFError, KeyboardInterrupt):
            return 0
        if choice == "0":
            return 0
        if choice not in entries:
            continue
        process = processes.get(choice)
        if process is not None and process.poll() is None:
            print("此入口已打开，请查看对应终端。")
            continue
        try:
            # A separate console keeps logs and Ctrl+C available for each service.
            processes[choice] = subprocess.Popen(
                [str(PYTHON), str(Path(__file__).resolve()), "--run", choice],
                cwd=ROOT, creationflags=subprocess.CREATE_NEW_CONSOLE,
            )
            print("已打开对应终端，启动进度及错误信息显示在该窗口。")
        except OSError as exc:
            print(f"启动失败 {exc}")


if __name__ == "__main__":
    sys.exit(main())
