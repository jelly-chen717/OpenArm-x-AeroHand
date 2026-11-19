# 项目中文注释（2026-10-29）：容器内子进程监护。
# 调用与职责：按 CAN 是否齐全选择 ROS 待机或纯 Web，再启动标准 teleop-xr Web/IK。
# 运行边界：负责进程生命周期；硬件激活仍由用户网页操作。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Supervise the raw standby launch, read-only status, and stock teleop-xr CLI."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request


HARDWARE = {"openarm_right_hardware_interface", "openarm_left_hardware_interface"}


# 注释：确认两个硬件组件均未配置、控制器为空且状态新鲜。
def standby_ready(state):
    hardware = state.get("hardware", {})
    return (state.get("mode") == "original_standby" and state.get("control_available") is False
            and set(hardware) == HARDWARE
            and all(value.get("state_id") == 1 for value in hardware.values())
            and isinstance(state.get("hardware_age_ms"), (int, float))
            and 0 <= state["hardware_age_ms"] < 5000
            and state.get("controllers") == {}
            and isinstance(state.get("controllers_age_ms"), (int, float))
            and 0 <= state["controllers_age_ms"] < 5000
            and state.get("output_enabled") is not True)


# 注释：该文件的显式执行入口；按 CAN 是否齐全选择 ROS 待机或纯 Web，再启动标准 teleop-xr Web/IK。
def main():
    if os.environ.get("ROS_DOMAIN_ID") != "94" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Original standby requires domain 94 and localhost-only ROS")
    port_text = os.environ.get("ORIGINAL_VR_PORT", "4446")
    if not port_text.isdecimal() or not 1 <= int(port_text) <= 65535 or int(port_text) == 8767:
        raise RuntimeError("Invalid ORIGINAL_VR_PORT")
    if Path('/state/custom_chain_runtime.json').exists():
        raise RuntimeError('Archive source mode cannot run beside the extra control bridge')
    if not Path('/state/source_control.json').is_file():
        raise RuntimeError('Missing archive source profile')
    children = []
    stopped = False

    # 注释：把退出信号转换为监护循环停止标志，随后按既有顺序收尾子进程。
    def stop(*_):
        nonlocal stopped
        stopped = True

    # 注释：将指定子进程输出追加到 /logs，Web 进程降为 uid 1002，并登记以便统一退出。
    def start(name, command, env=None, uid=None):
        Path("/logs").mkdir(exist_ok=True)
        log = open("/logs/" + name + ".log", "ab", buffering=0)
        try:
            options={'env':env}
            if uid is not None:
                if os.geteuid()==0:options.update(user=uid,group=uid,extra_groups=[])
                elif os.geteuid()!=uid:raise RuntimeError('Unexpected web owner')
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, **options)
        except BaseException:
            log.close()
            raise
        children.append((name, child, log))
        print(json.dumps({"started": name, "pid": child.pid}), flush=True)

    # 注释：发现受管子进程退出时报告失败，由外层统一收尾。
    def check_children():
        for name, child, _ in children:
            if child.poll() is not None:
                raise RuntimeError(name + " exited with " + str(child.returncode))

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    exit_code = 0
    try:
        # With a missing CAN interface, expose Web only; do not launch any driver.
        # Reconnecting hardware never starts a driver automatically in this mode.
        web_only = not all((Path('/sys/class/net')/name).exists() for name in ('can0','can1'))
        if web_only:
            start("status", ["/usr/bin/python3", "-B", "/test/app/web_offline_status.py"])
            print(json.dumps({"mode":"original_web_offline","ros_control_started":False}),flush=True)
        else:
            start("ros", ["/usr/bin/python3", "-B", "/opt/ros/humble/bin/ros2", "launch", "/test/app/raw_standby.launch.py"])
            start("status", ["/usr/bin/python3", "-B", "/test/app/ros_status.py"])
            ready = False
            deadline = time.monotonic() + 40
            while not stopped and time.monotonic() < deadline:
                check_children()
                try:
                    with urllib.request.urlopen("http://127.0.0.1:8767/status", timeout=1) as response:
                        ready = standby_ready(json.load(response))
                except (OSError, ValueError):
                    ready = False
                if ready:
                    break
                time.sleep(0.2)
            if stopped:
                return 0
            if not ready:
                raise RuntimeError("Original ROS standby did not confirm both hardware components unconfigured")
        # The unified 4446 app owns all public workstation routes.
        web_env=os.environ.copy()
        web_env.update(PYTHONPATH='/test/app:/test/vendor/saved_custom_python:/opt/motor_toggle:'+web_env.get('PYTHONPATH',''),
                       HOME='/tmp/archive-source-web',XDG_CACHE_HOME='/tmp/archive-source-cache',
                       JAX_COMPILATION_CACHE_DIR='/logs/archive-source-jax-cache',
                       JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS='0',OPENARM_SCHEDULER_PROFILE='off')
        for key in ('HOME','XDG_CACHE_HOME','JAX_COMPILATION_CACHE_DIR'):
            directory=Path(web_env[key]);directory.mkdir(mode=0o700,parents=True,exist_ok=True)
            if directory.is_symlink():raise RuntimeError('Unexpected writable path symlink')
            if os.geteuid()==0:os.chown(directory,1002,1002)
            if directory.stat().st_uid!=1002:raise RuntimeError('Unexpected web directory owner')
        web_command = ["/opt/venv-teleop-xr/bin/python", "-B", "-m", "teleop_xr.ros2",
                       "--mode", "ik", "--robot-class", "openarm", "--host", "0.0.0.0", "--port", port_text]
        if web_only:
            # Expand the same installed model as raw_standby.launch.py locally.
            # Passing the stock CLI's urdf_string avoids a network clone without ROS.
            model_program = """import os,xacro
from ament_index_python.packages import get_package_share_directory
share=get_package_share_directory('openarm_description')
model=xacro.process_file(os.path.join(share,'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'),
 mappings={'arm_type':'openarm_v1.0','bimanual':'true','use_fake_hardware':'false',
 'ros2_control':'true','right_can_interface':'can0','left_can_interface':'can1',
 'return_to_zero':'false','gravity_scale':'1.0','teaching_mode':'false'}).toprettyxml(indent='  ')
print(model)
"""
            model = subprocess.check_output(['/usr/bin/python3','-B','-c',model_program],text=True,timeout=15)
            web_command += ['--no-urdf-topic','--robot-args',json.dumps({'urdf_string':model})]
        start("web",web_command,env=web_env,uid=1002)
        while not stopped:
            check_children()
            time.sleep(0.25)
    except Exception as error:
        print(str(error), file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        for _, child, _ in reversed(children):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 6
        for _, child, log in reversed(children):
            try:
                child.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=2)
            finally:
                log.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
