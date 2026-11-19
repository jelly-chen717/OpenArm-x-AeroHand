"""Create the separate original standby and select it in the existing user unit."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from urllib.request import urlopen

ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
BACKUP = Path('/home/robot/Teleop/VR_to_Openarm_custom_backup_20261019T070523Z')
IMAGE = 'sha256:42fd90e26269004a72cc175bfa02c74fcc609af6ab2bb9bc11470ef0114c7bfb'
NAME = 'openarm-vr-original-control-20261019'
DROPIN = Path.home()/'.config/systemd/user/openarm-teleop-standby.service.d/90-original-control-20261019.conf'
CONTENT = '[Service]\nExecStart=\nExecStart=/usr/bin/python3 -B '+str(ROOT/'ops/manage.py')+' start\n'


def run(args, timeout=30):
    return subprocess.check_output(args, text=True, timeout=timeout)


def main():
    os.umask(0o077)
    assert ROOT.is_dir() and not ROOT.is_symlink()
    resume = (ROOT/'runtime.json').exists()
    assert not DROPIN.exists() or (resume and DROPIN.read_text() == CONTENT)
    manifest = json.loads((BACKUP/'manifest.json').read_text())
    assert manifest['complete'] and manifest['phase'] == 'complete'
    staged = json.loads((ROOT/'reports/prepared_sources.json').read_text())
    assert staged['complete'] and staged['source_count'] == 303
    assert staged['wheel']['original_control_and_frontend_bytes_preserved']
    assert not (ROOT/'vendor/python/webxr/out').exists()
    for row in staged['compiled_libraries']:
        assert row.get('mapped_target_present', True), row
    prior = json.loads(run(['docker', 'inspect', manifest['container_id'],
        '34281d6ba939682e65360bee7ac80c91f6897b38291c855bae1f3905a001d3a6',
        '05dc367eb4d9f1923ae9788766b7b5e38b7e09cd520ca6ddd25a299ce1232df2']))
    assert all(not c['State']['Running'] for c in prior)
    if not resume:
        assert NAME not in run(['docker', 'ps', '-a', '--format', '{{.Names}}']).splitlines()
    args = ['docker','create','--name',NAME,'--label','openarm.vr.original-control=20261019',
            '--restart','no','--network','host','--user',str(os.getuid())+':'+str(os.getgid()),
            '--read-only','--cap-drop','ALL','--cap-add','NET_RAW',
            '--security-opt','no-new-privileges','--cpus','8','--memory','12g','--pids-limit','1024',
            '--ulimit','core=0','--tmpfs','/tmp:rw,size=3g,mode=1777','--tmpfs','/home/operator:rw,size=32m',
            '--shm-size','1g','--mount','type=bind,src='+str(ROOT)+',dst=/test,readonly',
            '--mount','type=bind,src='+str(ROOT/'state')+',dst=/state,readonly',
            '--mount','type=bind,src='+str(ROOT/'logs')+',dst=/logs',
            '--workdir','/test','--entrypoint','/usr/bin/env',IMAGE,
            '-i','PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'LANG=C.UTF-8','PYTHONFAULTHANDLER=1','ORIGINAL_VR_PORT=4446',
            '/bin/bash','/test/ops/run_original.sh']
    identity = json.loads((ROOT/'runtime.json').read_text())['container_id'] if resume else run(args).strip()
    runtime = {'schema_version':1,'container_id':identity,'container_name':NAME,'image':IMAGE,
               'labels':{'openarm.vr.original-control':'20261019'},
               'backup_manifest_sha256':hashlib.sha256((BACKUP/'manifest.json').read_bytes()).hexdigest(),
               'owned_dropin_sha':hashlib.sha256(CONTENT.encode()).hexdigest(),
               'ports':{'console':4443,'original_vr':4446},'automatic_enable':False}
    if resume:
        assert json.loads((ROOT/'runtime.json').read_text()) == runtime
    else:
        with (ROOT/'runtime.json').open('x') as f:
            json.dump(runtime,f,indent=2)
            f.write('\n')
    # Run all identity, source-ownership, backup and other-manager guards before
    # writing the unit override. start is the only operation that launches ROS.
    started = json.loads(run(['python3','-B',str(ROOT/'ops/manage.py'),'start']))
    ready = False
    deadline = time.monotonic()+45
    while time.monotonic() < deadline:
        try:
            with urlopen('http://127.0.0.1:8767/status',timeout=2) as response:
                state = json.load(response)
            ready = state.get('mode') == 'original_standby' and state.get('driver_quiescent') is True
        except (OSError,ValueError):
            ready = False
        if ready:
            break
        time.sleep(0.5)
    assert ready, 'New original standby did not become ready; no service override installed'
    DROPIN.parent.mkdir(exist_ok=True,parents=True)
    if not DROPIN.exists():
        with DROPIN.open('x') as f:
            f.write(CONTENT)
    run(['systemctl','--user','disable','openarm-vr-view-mirror.service'])
    run(['systemctl','--user','daemon-reload'])
    run(['systemctl','--user','start','openarm-teleop-standby.service'])
    result = {'runtime':runtime,'start':started,'dropin':str(DROPIN),'automatic_enable':False}
    (ROOT/'reports/deployment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
