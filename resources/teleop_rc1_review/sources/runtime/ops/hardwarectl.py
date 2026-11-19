# 项目中文注释（2026-10-29）：宿主机硬件生命周期入口。
# 调用与职责：在所属容器中调用 hardware_lifecycle，保存回执并处理 Web 暂停/恢复。
# 运行边界：initialize-enable 和 disable 会改变真实电机与控制器状态。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Host entry for explicit original-controller enable/disable operations."""
import argparse
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import ssl
import subprocess
import time
import urllib.request
import uuid

ROOT=Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
CLIENT=ROOT/'ops/hardware_lifecycle_20261020.py'


# 注释：生成 UTC 时间字符串，用于回执与日志关联。
def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# 注释：在宿主机运行固定命令，保留输出和错误。
def host_command(command_line,timeout=25,controlled=False):
    from operation_lock import child_fds
    # A transport helper may have submitted a Docker action before a host
    # deadline expires. Keep the lease until it acknowledges completion.
    result=subprocess.run(command_line,capture_output=True,text=True,
                          timeout=None if controlled else timeout,pass_fds=child_fds())
    if result.returncode:
        raise RuntimeError('Command failed: '+' '.join(command_line[:4])+' '+result.stderr[-2200:]+result.stdout[-1000:])
    return result.stdout


# 注释：定位所属容器中原 teleop-xr Web 进程，避免选中其他程序。
def stock_web(container_id,command=host_command,required=True,proc=Path('/proc')):
    rows=command(['docker','top',container_id,'-eo','pid,args']).splitlines()
    rows=[line for line in rows if '-m teleop_xr.ros2 ' in line and '--port 4446' in line]
    if not rows and not required:
        return None
    assert len(rows)==1, 'Expected exactly one stock web process'
    pid=int(rows[0].split()[0])
    arguments=(proc/str(pid)/'cmdline').read_bytes()
    assert b'teleop_xr.ros2' in arguments and b'4446' in arguments
    return pid,arguments


# 注释：按现有流程临时暂停原 Web 工作者，为硬件生命周期操作建立静止条件。
def pause_web(process,proc=Path('/proc')):
    if process is None:
        return None
    pid,arguments=process
    assert (proc/str(pid)/'cmdline').read_bytes()==arguments,'Web PID identity changed'
    state=(proc/str(pid)/'stat').read_text().rsplit(')',1)[1].split()[0]
    if state in ('T','t'):
        return None  # Do not later resume a process paused by another action.
    os.kill(pid,signal.SIGSTOP)
    return process


# 注释：记录硬件操作结束时的反馈、控制器和输出状态。
def final_observation(action,read):
    try:
        return read(),None
    except (OSError,ValueError) as error:
        if action!='disable':
            raise
        return {'state':'unknown','output_enabled':None},str(error)


# 注释：在所属容器内执行已有生命周期客户端并读取结构化回执。
def lifecycle_call(line,container_id,receipt_name,uncertain,timeout=120):
    """Keep the host lease until a timed-out Docker exec's ROS helper is gone."""
    try:
        return subprocess.run(line,capture_output=True,text=True,timeout=timeout)
    except subprocess.TimeoutExpired:
        uncertain()
        while True:
            try:
                inspect=subprocess.run(['docker','inspect',container_id],capture_output=True,text=True,timeout=10)
                if inspect.returncode!=0:
                    raise RuntimeError('Container inspection unavailable')
                state=json.loads(inspect.stdout)[0]['State']
                if not state['Running']:
                    break
                top=subprocess.run(['docker','top',container_id,'-eo','pid,args'],capture_output=True,text=True,timeout=10)
                if top.returncode!=0:
                    raise RuntimeError('Helper inspection unavailable')
                if not any('hardware_lifecycle_20261020.py' in row and '/logs/'+receipt_name in row
                           for row in top.stdout.splitlines()):
                    break
            except (OSError,ValueError,KeyError,RuntimeError,subprocess.TimeoutExpired):
                # Unknown is not an exit acknowledgement. Keep the transaction
                # occupied instead of permitting a second hardware operation.
                pass
            time.sleep(1)
        return subprocess.CompletedProcess(line,1,'','Client timeout; helper exit subsequently verified')


# 注释：解析显式 initialize-enable/disable/status 动作，核对所属容器后运行原生命周期工具并保存完整回执。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['initialize-enable','disable','status'])
    parser.add_argument('--hardware-only',action='store_true',
                        help='Complete on verified motor hold; report VR readiness separately')
    parser.add_argument('--cancel-file',type=Path)
    args=parser.parse_args()
    if args.action=='status':
        with urllib.request.urlopen('http://127.0.0.1:8767/status',timeout=3) as response:
            print(json.dumps(json.load(response),ensure_ascii=False))
        return
    if args.cancel_file is None:
        args.cancel_file=ROOT/'state'/('hardware_cancel_'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()[:8]+'_'+str(os.getpid())+'.request')
    assert Path(__file__).resolve().parent.parent==ROOT
    spec=importlib.util.spec_from_file_location('original_manager',ROOT/'ops/manage.py')
    manage=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(manage)
    settings=manage.runtime()
    manage.backup_complete(settings)
    manage.check_owned_dropin(settings)
    assert manage.DROPIN.is_file()
    assert not manage.customized_container()['State']['Running']
    manage.require_tests_stopped()
    manage.require_no_other_managers(manage.original_container(settings))
    identity=uuid.uuid4().hex
    if args.cancel_file:
        assert args.cancel_file.resolve().parent == (ROOT/'state').resolve()
    directory=ROOT/'reports'/('hardware_'+args.action+'_'+identity)
    directory.mkdir(mode=0o700)
    progress={'action':args.action,'run_id':identity,'started_utc':utc(),'phase':'precheck',
              'report_directory':str(directory),'bridge_enable_requested':False,
              'lifecycle_client_sha256':hashlib.sha256(CLIENT.read_bytes()).hexdigest(),
              'motion_test_performed':False}
    paused=None
    client_receipt=None
    lifecycle_started=False

    # 注释：把当前事务进度写入回执，便于失败后核对和恢复。
    def save():
        tmp=directory/'progress.tmp'
        tmp.write_text(json.dumps(progress,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(directory/'progress.json')
        print(json.dumps({k:progress.get(k) for k in ['phase','error','finished_utc']}),flush=True)

    command=host_command

    # 注释：本模块的 main.status 定义；用途见文件说明与下方原实现。
    def status():
        with urllib.request.urlopen('http://127.0.0.1:8767/status',timeout=2) as response:
            return json.load(response)

    # 注释：本模块的 main.web_process 定义；用途见文件说明与下方原实现。
    def web_process(required=True):
        return stock_web(settings['container_id'],command,required)

    # 注释：仅在进程身份与原记录一致时恢复先前暂停的 Web 工作者。
    def resume_if_unchanged():
        nonlocal paused
        if paused:
            pid,arguments=paused
            assert (Path('/proc')/str(pid)/'cmdline').read_bytes()==arguments, 'Paused PID identity changed'
            os.kill(pid,signal.SIGCONT)
            progress['stock_web_resumed_utc']=utc()
            paused=None

    save()
    try:
        if args.cancel_file and args.cancel_file.exists():
            raise RuntimeError('Operation cancelled before hardware preparation')
        if args.action=='initialize-enable':
            # Repeat a finite query immediately before activation, with the sole
            # hardware driver fully stopped. The historical proof is <=30 s old.
            progress['phase']='fresh_disabled_query'
            save()
            progress['can_preparation']=json.loads(command(['python3','-B',str(ROOT/'ops/prepare_can_20261020.py')],controlled=True))
        if args.cancel_file and args.cancel_file.exists():
            raise RuntimeError('Operation cancelled before lifecycle activation')
        deadline=time.monotonic()+20
        while True:
            assert manage.original_container(settings)['State']['Running']
            try:
                if args.action=='initialize-enable':
                    manage.standby_status()
                process=web_process(required=args.action!='disable')
                break
            except (OSError,RuntimeError,AssertionError):
                if time.monotonic()>deadline:
                    raise
                time.sleep(.2)
        paused=pause_web(process)
        progress['stock_web_paused']=({'host_pid':paused[0],'time_utc':utc()} if paused else None)
        progress['phase']='lifecycle_client_running'
        save()
        receipt_name='activation_'+identity+'.json'
        shell='''set -eo pipefail
source /opt/ros/humble/setup.bash
source /test/ros2_ws/install/local_setup.bash
export PYTHONPATH="/test/app:/test/ops${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export HOME=/tmp/original-vr-home ROS_LOG_DIR=/logs/ros
exec /usr/bin/python3 -B /test/ops/hardware_lifecycle_20261020.py "$1" --receipt "$2"
'''
        line=['docker','exec',settings['container_id'],'/usr/bin/env','-i',
              'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin','LANG=C.UTF-8',
              '/bin/bash','-c',shell,'hardwarectl',args.action,'/logs/'+receipt_name]
        lifecycle_started=True
        # 注释：将无法确认的收尾结果记入回执，避免伪报成功。
        def uncertain():
            progress['phase']='lifecycle_result_unknown_waiting_for_helper'
            save()
        result=lifecycle_call(line,settings['container_id'],receipt_name,uncertain)
        (directory/'client_stdout.log').write_text(result.stdout)
        (directory/'client_stderr.log').write_text(result.stderr)
        path=ROOT/'logs'/receipt_name
        assert path.is_file(), 'Lifecycle helper did not persist its receipt'
        client_receipt=json.loads(path.read_text())
        progress['lifecycle_receipt']=client_receipt
        progress['lifecycle_returncode']=result.returncode
        assert result.returncode==0 and client_receipt.get('success') is True, 'Lifecycle helper failed; inspect its rollback receipt'
        assert client_receipt.get('bridge_enable_requested') is False
        if args.cancel_file and args.cancel_file.exists() and args.action=='initialize-enable':
            raise RuntimeError('Operation cancelled after lifecycle activation; closing hardware')
        if args.action=='initialize-enable':
            final=client_receipt['final']
            assert final['hardware']=={'openarm_right_hardware_interface':3,'openarm_left_hardware_interface':3}
            expected={'joint_state_broadcaster','left_forward_position_controller','right_forward_position_controller',
                      'left_gripper_controller','right_gripper_controller'}
            assert set(final['controllers'])==expected and set(final['controllers'].values())=={'active'}
            assert len(final['motors'])==16 and all(m['state']==1 and 0<=m['age_ms']<=250 for m in final['motors'])
            assert client_receipt['command_counts_before']==client_receipt['command_counts_after']
        resume_if_unchanged()
        if args.action=='initialize-enable' and not args.hardware_only:
            progress['phase']='enabled_hold_waiting_web'
            save()
            deadline=time.monotonic()+540
            context=ssl._create_unverified_context()
            while time.monotonic()<deadline:
                if args.cancel_file and args.cancel_file.exists():
                    raise RuntimeError('Operation cancelled while waiting for VR startup')
                state=status()
                assert state['hardware_states']=={'openarm_right_hardware_interface':3,'openarm_left_hardware_interface':3}
                assert state['enabled']==16 and state['faulted']==0 and state['online']==16, 'Enabled feedback was lost after activation'
                ready={}
                for port in [4446]:
                    try:
                        with urllib.request.urlopen('https://127.0.0.1:'+str(port)+'/',timeout=2,context=context) as response:
                            ready[str(port)]=response.status
                    except OSError:
                        pass
                if len(ready)==1 and all(code==200 for code in ready.values()):
                    progress['web']=ready
                    break
                time.sleep(3)
            else:
                raise RuntimeError('Web readiness timed out after successful hardware enable')
        progress['final_status'],observation_error=final_observation(args.action,status)
        if observation_error:
            progress['final_status_error']=observation_error
        progress['hardware_only']=args.hardware_only
        progress['phase']='complete'
        progress['finished_utc']=utc()
        save()
    except BaseException as error:
        progress['error']=repr(error)
        progress['phase']='failed'
        progress['finished_utc']=utc()
        # A later web/feedback failure after a successful enable must not leave
        # the transaction's motors active without an attempted orderly close.
        if args.action=='initialize-enable' and lifecycle_started:
            try:
                if not paused:
                    paused=pause_web(web_process(required=False))
                close_name='activation_'+identity+'_postcheck_close.json'
                close_line=[*line[:-2],'disable','/logs/'+close_name]
                close_result=lifecycle_call(close_line,settings['container_id'],close_name,uncertain,timeout=45)
                progress['postcheck_close_returncode']=close_result.returncode
                (directory/'postcheck_close_stdout.log').write_text(close_result.stdout)
                (directory/'postcheck_close_stderr.log').write_text(close_result.stderr)
                close_path=ROOT/'logs'/close_name
                if close_path.exists():
                    client_receipt=json.loads(close_path.read_text())
                    progress['postcheck_close_receipt']=client_receipt
            except BaseException as close_error:
                progress['postcheck_close_error']=repr(close_error)
        if paused:
            shutdown=(client_receipt or {}).get('shutdown',{})
            safe=bool(shutdown.get('verified_hardware_unconfigured') and shutdown.get('owned_controllers_unloaded'))
            try:
                current=status()
                safe=safe or (set(current['hardware_states'].values())=={1}
                              and not any(value=='active' for value in current['controllers'].values()))
            except (OSError,ValueError):
                pass
            if safe:
                try:
                    resume_if_unchanged()
                except Exception as resume_error:
                    progress['resume_error']=repr(resume_error)
            else:
                progress['stock_web_left_paused']=True
        save()
        raise


if __name__=='__main__':
    import sys
    if len(sys.argv)>1 and sys.argv[1]=='status':
        main()
    else:
        from operation_lock import exclusive
        with exclusive(ROOT):
            main()
