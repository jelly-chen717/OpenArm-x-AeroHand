# 项目中文注释（2026-10-29）：显式待机恢复辅助。
# 调用与职责：确认未配置待机后重启原所属服务，等待唯一 4446 页面与 IK 预热。
# 运行边界：会启停服务；不自动使能。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Preserve reboot evidence, then restart only the selected original standby."""
import datetime
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tarfile
import time
import urllib.error
import urllib.request
import ssl

ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')


# 注释：生成 UTC 时间字符串，用于回执与日志关联。
def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# 注释：该文件的显式执行入口；确认未配置待机后重启原所属服务，等待唯一 4446 页面与 IK 预热。
def main():
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    report = ROOT/'reports'/('recovery_'+stamp)
    report.mkdir(mode=0o700)
    progress = {'started_utc':utc(),'report_directory':str(report),'phase':'precheck',
                'automatic_enable':False,'can_configuration_changed':False,'control_code_changed':False}

    # 注释：把当前事务进度写入回执，便于失败后核对和恢复。
    def save():
        data = json.dumps(progress,ensure_ascii=False,indent=2)+'\n'
        temp = report/'progress.tmp'
        temp.write_text(data)
        temp.replace(report/'progress.json')
        print(json.dumps({k:progress.get(k) for k in ['phase','error','finished_utc']}),flush=True)

    # 注释：处理 command 消息并更新本适配器的观察缓存或计数。
    def command(args,timeout=25):
        p = subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        if p.returncode:
            raise RuntimeError(' '.join(args[:4])+': '+p.stderr[-2500:]+p.stdout[-1000:])
        return p.stdout

    save()
    try:
        spec = importlib.util.spec_from_file_location('original_manager',ROOT/'ops/manage.py')
        manage = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(manage)
        settings = manage.runtime()
        manage.backup_complete(settings)
        manage.check_owned_dropin(settings)
        assert manage.DROPIN.is_file(), 'Original-control service override is not selected'
        old = manage.customized_container()
        assert not old['State']['Running'], 'Customized container is running'
        manage.require_tests_stopped()
        container = manage.original_container(settings)
        manage.require_no_other_managers(container)
        before = manage.standby_status() if container['State']['Running'] else None
        progress['container_before'] = container['State']
        progress['standby_before'] = before
        progress['log_offsets_before'] = {p.name:p.stat().st_size for p in (ROOT/'logs').glob('*.log')}
        (report/'container_before.json').write_text(json.dumps(container,indent=2)+'\n')
        (report/'unit_before.txt').write_text(command(['systemctl','--user','cat','openarm-teleop-standby.service']))
        # Keep exact pre-restart bytes without clearing any running log file.
        archive = report/'logs_before_restart.tar.gz'
        files = [Path('/var/log/syslog'),Path('/var/log/kern.log'),
                 *(ROOT/'logs'/n for n in ['ros.log','status.log','web.log','console.log'])]
        with tarfile.open(archive,'w:gz') as output:
            for path in files:
                if not path.is_file():
                    continue
                content = path.read_bytes()
                info = tarfile.TarInfo(str(path).lstrip('/'))
                info.size = len(content)
                info.mode = 0o600
                info.mtime = int(path.stat().st_mtime)
                output.addfile(info,io.BytesIO(content))
        progress['evidence_archive'] = {'path':str(archive),'bytes':archive.stat().st_size,
            'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()}
        progress['phase'] = 'evidence_preserved'
        save()
        progress['stop_result'] = manage.stop()
        progress['phase'] = 'original_container_stopped'
        save()
        command(['systemctl','--user','restart','openarm-teleop-standby.service'])
        progress['restart_dispatched_utc'] = utc()
        progress['phase'] = 'waiting_standby'
        save()
        deadline = time.monotonic()+45
        state = None
        while time.monotonic()<deadline:
            current = manage.original_container(settings)
            if not current['State']['Running']:
                raise RuntimeError('Original container exited during restart: '+str(current['State']))
            try:
                state = manage.standby_status()
                break
            except (OSError,ValueError,RuntimeError):
                time.sleep(1)
        assert state is not None, 'Fresh unconfigured standby was not confirmed'
        progress['standby_after_restart'] = state
        progress['phase'] = 'waiting_original_ik_warmup'
        save()
        context = ssl._create_unverified_context()
        deadline = time.monotonic()+540
        while time.monotonic()<deadline:
            if not manage.original_container(settings)['State']['Running']:
                raise RuntimeError('Original container exited during IK warmup')
            ready = {}
            for port in [4446]:
                try:
                    with urllib.request.urlopen('https://127.0.0.1:'+str(port)+'/',
                                                timeout=3,context=context) as response:
                        ready[str(port)] = {'http':response.status,'bytes':len(response.read())}
                except OSError:
                    pass
            if len(ready)==1 and all(v['http']==200 for v in ready.values()):
                progress['web'] = ready
                break
            time.sleep(5)
        else:
            raise RuntimeError('Web readiness timeout after original IK warmup')
        progress['verification'] = json.loads(command(['python3','-B',str(ROOT/'ops/verify_original.py')],timeout=90))
        progress['container_after'] = manage.original_container(settings)['State']
        progress['finished_utc'] = utc()
        progress['phase'] = 'complete'
        save()
    except BaseException as error:
        progress['phase'] = 'failed'
        progress['error'] = repr(error)
        progress['finished_utc'] = utc()
        save()
        raise


if __name__ == '__main__':
    main()
