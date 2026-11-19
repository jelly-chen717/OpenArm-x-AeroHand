"""Read-only final acceptance of source provenance, HTTP routes and standby."""
import hashlib
import json
from pathlib import Path
import re
import ssl
import subprocess
import time
import urllib.error
import urllib.request

ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
TLS = ssl._create_unverified_context()


def run(args):
    return subprocess.check_output(args,text=True,timeout=15)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch(url, **kwargs):
    with urllib.request.urlopen(urllib.request.Request(url,**kwargs),context=TLS,timeout=5) as response:
        return response.status,response.read()


def denied(url, **kwargs):
    try:
        fetch(url,**kwargs)
    except urllib.error.HTTPError as error:
        assert error.code == 409, (url,error.code)
        return error.code
    raise AssertionError('Control endpoint was not rejected: '+url)


def main():
    runtime = json.loads((ROOT/'runtime.json').read_text())
    prepared = json.loads((ROOT/'reports/prepared_sources.json').read_text())
    vr_code,vr_body = fetch('https://192.168.31.220:4446/')
    assert vr_code == 200
    assert hashlib.sha256(vr_body).hexdigest() == digest(ROOT/'vendor/python/teleop_xr/dist/index.html')
    code,body = fetch('https://192.168.31.220:4443/')
    html = body.decode()
    assert code == 200 and '/legacy-readonly.js' in html
    assert not re.search(r'<script[^>]+src=[^>]*(?:motor-toggle\.js|vr-operation-view\.|vr-start-guide\.)',html)
    assets = re.findall(r'(?:src|href)="(/[^"?#]+)(?:[^"#]*)"',html)
    checked = []
    for path in sorted(set(assets)):
        if path.endswith(('.js','.css','.woff2')):
            status,data = fetch('https://192.168.31.220:4443'+path)
            assert status == 200 and data
            checked.append(path)
    rejection = {
        'legacy_ws':denied('https://192.168.31.220:4443/ws'),
        'legacy_control_post':denied('https://192.168.31.220:4443/api/motor/set',data=b'{}',method='POST'),
        'monitor_control_post':denied('http://127.0.0.1:8767/set',data=b'{}',method='POST'),
    }
    from manage import standby_status
    state = standby_status()
    containers = json.loads(run(['docker','inspect',runtime['container_id'],
        '4d915ea7f02c43316e5b217158715fee5f2fc23ed6f3e84716861a0e77f55bf5',
        '34281d6ba939682e65360bee7ac80c91f6897b38291c855bae1f3905a001d3a6',
        '05dc367eb4d9f1923ae9788766b7b5e38b7e09cd520ca6ddd25a299ce1232df2']))
    assert containers[0]['State']['Running'] and all(not c['State']['Running'] for c in containers[1:])
    assert containers[0]['HostConfig']['ReadonlyRootfs']
    assert not any(m.get('Name') == 'codex-openarm-vr-mock-home-20261008' for m in containers[0]['Mounts'])
    top = run(['docker','top',runtime['container_id'],'-eo','pid,args'])
    manager = [line for line in top.splitlines() if '/controller_manager/ros2_control_node ' in line]
    assert len(manager) == 1
    pid = manager[0].split()[0]
    maps = [line for line in Path('/proc/'+pid+'/maps').read_text().splitlines() if 'libopenarm_hardware.so' in line]
    assert maps and all('/test/ros2_ws/' in line for line in maps)
    global_managers = [line for line in run(['ps','-eo','pid,args']).splitlines()
                       if re.search(r'(?:^|\s)(?:\S*/)?ros2_control_node(?:\s|$)',line)]
    assert len(global_managers) == 1 and global_managers[0].split()[0] == pid
    env = {}
    for line in top.splitlines():
        if any(token in line for token in ('-m teleop_xr.ros2','/openarm_teleop_bridge/jt_bridge')):
            process = line.split()[0]
            values = dict(item.split('=',1) for item in Path('/proc/'+process+'/environ').read_text().split('\0') if '=' in item)
            selected = {k:values.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','PYTHONPATH','AMENT_PREFIX_PATH','LD_LIBRARY_PATH')}
            assert selected['ROS_DOMAIN_ID'] == '94'
            assert '/test/vendor/python' in selected['PYTHONPATH']
            assert not any(old in ':'.join(v or '' for v in selected.values())
                           for old in ('/opt/motor_toggle','/opt/readme_ws','/opt/original_real_ws'))
            env[process] = selected
    assert len(env) == 2
    for row in prepared['source_comparison']:
        assert digest(ROOT/row['path']) == row['zip_sha256']
    for row in prepared['wheel']['entries']:
        if not row.get('deployment_tls_exception'):
            assert digest(ROOT/'vendor/python'/row['path']) == row['wheel_sha256']
    assert run(['systemctl','--user','is-active','openarm-teleop-standby.service']).strip() == 'active'
    assert 'openarm-camera-media-20261010' in run(['docker','ps','--format','{{.Names}}']).splitlines()
    result = {'complete':True,'time_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        'source_count':len(prepared['source_comparison']), 'wheel_member_count':prepared['wheel']['member_count'],
        'unmodified_control_and_stock_vr':True,'tls_configuration_exceptions':['cert.pem','key.pem'],
        'console_http':code,'stock_vr_http':vr_code,'stock_vr_html_matches_wheel':True,
        'legacy_assets_checked':checked,'control_routes_rejected':rejection,'standby':state,
        'container_id':runtime['container_id'],'preserved_containers_stopped':True,
        'actual_hardware_library_maps':maps,'actual_control_process_env':env,
        'global_controller_manager_count':1,'processes':top,
        'automatic_enable':False,'motion_test_performed':False,'headset_test_performed':False,
        'customized_restore_command':'python3 -B '+str(ROOT/'ops/manage.py')+' restore-custom'}
    (ROOT/'reports/final_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))


if __name__ == '__main__':
    main()
