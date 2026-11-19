"""Authorized fixed transport restoration, executed on the target host.

Uses the target's existing ip binary and matching loader/libraries in a short
NET_ADMIN-only container. Never opens AF_CAN or changes motor configuration.
"""
import json
from pathlib import Path
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parent.parent
IMAGE = 'sha256:20651f9228f3ca4f95246f133d2a964f492d3a686eecd99ed6214a3cfafac7eb'


def run(args, timeout=20):
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stderr)
    return result.stdout


def rows():
    return {v['ifname']: v for v in json.loads(run(['ip','-j','-details','-statistics','link','show']))
            if v['ifname'] in ['can0','can1']}


def identity(name):
    base = Path('/sys/class/net')/name
    device = (base/'device').resolve(strict=True)
    usb = next(p for p in [device, *device.parents] if (p/'idVendor').is_file())
    assert (device/'driver').resolve().name == 'peak_usb'
    assert (usb/'idVendor').read_text().strip() == '0c72' and (usb/'idProduct').read_text().strip() == '0011'
    port = {'can0':'0x0', 'can1':'0x1'}[name]
    assert (base/'dev_id').read_text().strip() == port
    return str(device), str(usb), socket.if_nametoindex(name), port


def ip(args):
    loader = str(Path('/lib64/ld-linux-x86-64.so.2').resolve())
    libs = str(Path('/lib/x86_64-linux-gnu').resolve())
    tool = str(Path('/usr/sbin/ip').resolve())
    return run(['docker','run','--rm','--network','host','--user','0','--cap-drop','ALL','--cap-add','NET_ADMIN',
        '--security-opt','no-new-privileges:true','--read-only','--label','openarm.vr.can-repair=20261018',
        '--mount','type=bind,src='+loader+',dst=/hostld,readonly',
        '--mount','type=bind,src='+libs+',dst=/hostlib,readonly',
        '--mount','type=bind,src='+tool+',dst=/hostip,readonly',
        '--entrypoint','/hostld',IMAGE,'--library-path','/hostlib','/hostip',*args])


def config(name, listen, dsjw):
    return ip(['link','set','dev',name,'type','can','listen-only','on' if listen else 'off',
        'fd','on','bitrate','1000000','sample-point','0.75','sjw','1',
        'dbitrate','5000000','dsample-point','0.75','dsjw',str(dsjw)])


def main():
    receipt = {'authorized':'Human explicitly allowed restoring LISTEN-ONLY off / data SJW=2 and state queries only',
        'time_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()), 'motor_frames_sent':0,
        'motor_enable_authorized':False, 'motion_authorized':False}
    before = rows()
    ids = {n:identity(n) for n in ['can0','can1']}
    assert len({v[1] for v in ids.values()}) == 1
    for v in before.values():
        d=v['linkinfo']['info_data']
        assert 'UP' in v['flags'] and d['state']=='ERROR-ACTIVE'
        assert set(d['ctrlmode']) in [{'FD'}, {'FD','LISTEN-ONLY'}]
        assert d['bittiming']['bitrate']==1000000 and d['data_bittiming']['bitrate']==5000000
        assert d['bittiming']['sample_point']==.75 and d['data_bittiming']['sample_point']==.75
        assert d['bittiming']['sjw']==1 and d['data_bittiming']['sjw'] in [1,2]
    receipt['before']=before
    receipt['standby_stop']=run(['python3',str(ROOT/'ops/handoffctl.py'),'stop'])
    for name in ['codex-openarm-motor-toggle-20261008','openarm-vr-package-test-20261018','openarm-vr-package-real-standby-20261018']:
        assert not json.loads(run(['docker','inspect',name]))[0]['State']['Running']
    assert not any('ros2_control_node' in l or 'motor_control.py' in l for l in run(['ps','-eo','args']).splitlines())
    changed=[]
    try:
        for name in ['can0','can1']:
            assert identity(name)==ids[name] and rows()[name]==before[name]
            ip(['link','set','dev',name,'down'])
            assert identity(name)==ids[name]
            config(name,False,2)
            ip(['link','set','dev',name,'up'])
            after=rows()[name]['linkinfo']['info_data']; old=before[name]['linkinfo']['info_data']
            assert after['bittiming']==old['bittiming']
            assert after['data_bittiming']==dict(old['data_bittiming'],sjw=2)
            assert set(after['ctrlmode'])=={'FD'} and after['state']=='ERROR-ACTIVE'
            assert after.get('restart_ms')==old.get('restart_ms')
            changed.append(name)
    except Exception as exc:
        receipt['error']=str(exc);receipt['rollback_errors']=[]
        for name in ['can0','can1']:
            try:
                assert identity(name)==ids[name]
                ip(['link','set','dev',name,'down'])
                old=before[name]['linkinfo']['info_data']
                config(name,'LISTEN-ONLY' in old['ctrlmode'],old['data_bittiming']['sjw'])
            except Exception as rollback:
                receipt['rollback_errors'].append(str(rollback))
            finally:
                if identity(name)==ids[name]:ip(['link','set','dev',name,'up'])
        receipt['after']=rows()
        (ROOT/'reports/can_restore_failed.json').write_text(json.dumps(receipt,indent=2))
        raise
    receipt.update(changed=changed,after=rows(),exact_changes_only_verified=True)
    (ROOT/'reports/can_restored_authorized.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(receipt))


if __name__=='__main__':main()
