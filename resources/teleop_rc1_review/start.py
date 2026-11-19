#!/usr/bin/env python3
"""Desktop entry: start missing services, never enable or move motors."""
import json,os,pwd,subprocess,sys,webbrowser
from pathlib import Path
ROOT=Path('/opt/robotcontrol')

def main():
    receipt=json.loads((ROOT/'install_receipt.json').read_text())
    uid=receipt['uid'];owner=receipt['owner']
    if os.geteuid()!=uid:
        args=['runuser','-u',owner,'--','env',f'XDG_RUNTIME_DIR=/run/user/{uid}',
              f'DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/{uid}/bus','python3',str(ROOT/'start.py'),'--no-browser']
        result=subprocess.run(args if os.geteuid()==0 else ['sudo',*args])
        if result.returncode:return result.returncode
        if '--no-browser' not in sys.argv:webbrowser.open('https://127.0.0.1:4448/')
        return 0
    env=os.environ.copy();env.update(XDG_RUNTIME_DIR=f'/run/user/{uid}',DBUS_SESSION_BUS_ADDRESS=f'unix:path=/run/user/{uid}/bus')
    def command(a):return subprocess.check_output(a,text=True,env=env)
    command(['systemctl','--user','start','robotcontrol-ui-4448.service','openarm-original-test-panel-api.service'])
    config=json.loads((ROOT/'runtime/device_config.json').read_text())
    if config.get('can_mapping_confirmed') is not True:
        print('Complete device configuration: sudo python3 /opt/robotcontrol/configure.py');return 2
    command(['python3',str(ROOT/'runtime/ops/manage.py'),'start'])
    state=json.loads(command(['docker','inspect',receipt['camera_container']]))[0]['State']
    if not state['Running']:command(['docker','start',receipt['camera_container']])
    print('RobotControl started; motors remain under manual webpage control.')
    print('Open https://<this computer LAN IP>:4448/ and wait for IK warmup.')
    return 0

if __name__=='__main__':raise SystemExit(main())
