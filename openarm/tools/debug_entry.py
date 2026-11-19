"""Local debug menu. No hardware access on startup or offline checks."""
from pathlib import Path
import argparse
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
PYTHON = ROOT / '.venv/Scripts/python.exe'


def run(args, cwd=ROOT):
    return subprocess.run([str(PYTHON), *map(str, args)], cwd=cwd).returncode


def check():
    logs = ROOT / 'logs/debug'
    logs.mkdir(parents=True, exist_ok=True)
    path = logs / time.strftime('offline-%Y%m%d-%H%M%S.txt')
    commands = [
        ['-m', 'openarm_app.cli', 'inspect'],
        ['-m', 'openarm_app.cli', '--arm', 'left', 'inspect'],
        ['-m', 'pip', 'check'],
        ['-m', 'unittest', 'discover', '-s', str(WORKSPACE / 'apps'), '-p', 'test_openarm_driver.py', '-v'],
    ]
    failed = False
    with path.open('w', encoding='utf-8') as log:
        log.write('OFFLINE ONLY: no adapter opened, no CAN requests.\n')
        for args in commands:
            result = subprocess.run([str(PYTHON), *args], cwd=ROOT, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, encoding='utf-8', errors='replace')
            text = f'\nCOMMAND: {args}\n{result.stdout}\nEXIT: {result.returncode}\n'
            log.write(text)
            print(text)
            failed |= result.returncode != 0
    print(f'检查记录：{path}')
    return int(failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        return check()
    while True:
        print('\nOpenArm 调试入口（默认离线，不自动连接硬件）\n'
              '1  打开控制资料与调试指南\n2  离线自检并保存日志\n'
              '3  启动双臂 MuJoCo 仿真（独立进程）\n'
              '4  启动网页仿真控制台（先启动 3）\n'
              '5  接机后查询状态（发送 CAN 状态请求，不使能）\n'
              '6  接机后监测 10 秒（发送 CAN 状态请求，不使能）\n'
              '7  打开实机日志目录\n0  退出')
        choice = input('请选择：').strip()
        if choice == '0':
            return 0
        if choice == '1':
            os.startfile(ROOT / '调试指南.md')
        elif choice == '2':
            check()
        elif choice in ('3', '4'):
            module = 'openarm_app.sim_server' if choice == '3' else 'openarm_app.web_controller'
            subprocess.Popen([str(PYTHON), '-m', module], cwd=ROOT)
            print('已启动。关闭 MuJoCo 窗口结束仿真；退出本菜单不会结束服务。')
        elif choice in ('5', '6'):
            side = input('查询哪侧？right / left / both（空白取消）：').strip()
            if side not in ('right', 'left', 'both'):
                continue
            print('默认右通道 0x51、左通道 0x52；物理归属需核对。')
            if input('确认已接机且通道未被其他程序占用，输入 CAN 执行：').strip() != 'CAN':
                continue
            run([WORKSPACE / 'apps/openarm_driver.py',
                 'status' if choice == '5' else 'monitor', '--side', side], WORKSPACE)
        elif choice == '7':
            path = WORKSPACE / 'logs/openarm'
            path.mkdir(parents=True, exist_ok=True)
            os.startfile(path)


if __name__ == '__main__':
    sys.exit(main())
