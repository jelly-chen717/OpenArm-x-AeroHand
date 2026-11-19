"""Installer transformations are limited to deployment identity and device binding."""
import hashlib,json,re,shutil
from pathlib import Path

OLD='/home/robot/Teleop/VR_to_Openarm_original_4443_20261019'
CRITICAL=['app/staged_arm_motion.py','app/archive_bridge_adapter.py','app/vr_output_runtime.py',
          'app/vr_joint_limits.py','app/vr_smoothing_config.json',
          'vendor/saved_custom_python/teleop_xr/ik/controller.py',
          'vendor/saved_custom_python/teleop_xr/ik/solver.py',
          'ros2_ws/install/openarm_teleop_bridge/lib/python3.10/site-packages/openarm_teleop_bridge/jt_bridge.py']

def replace_checked(text,old,new):
    if old not in text:raise ValueError('Expected deployment fragment is missing: '+old[:100])
    return text.replace(old,new)

def adapt(root,package,uid,transport_source):
    root=Path(root);package=Path(package)
    before={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in CRITICAL}
    for p in (root/'ops').glob('*.py'):
        s=p.read_text(encoding='utf8');s=s.replace(OLD,root.as_posix()).replace("'1002:1002'",repr(f'{uid}:{uid}'))
        if p.name=='supervisor.py':
            s=s.replace('1002',str(uid)).replace('"--host", "0.0.0.0"','"--host", "127.0.0.1"')
            # stdout must preserve the exact ROS model bytes in device-free mode.
            s=replace_checked(s,'print(model)\n','print(model,end="")\n')
        if p.name in ('hardwarectl.py','prepare_can_20261020.py'):
            s=replace_checked(s,"        assert not manage.customized_container()['State']['Running']\n" if p.name=='prepare_can_20261020.py' else "    assert not manage.customized_container()['State']['Running']\n",'')
        if p.name=='panel_operations.py':
            s=replace_checked(s,"        if manage.customized_container()['State']['Running'] is not False:\n            raise RuntimeError('Preserved customized runtime is running')\n",'')
        if p.name=='restart_standby_20261020.py':
            s=replace_checked(s,"        old = manage.customized_container()\n        assert not old['State']['Running'], 'Customized container is running'\n",'')
        if p.name=='archive_output.py':
            s=replace_checked(s,"    require(not manager.customized_container()['State']['Running'], 'Customized backup is running')\n",'')
        p.write_text(s,encoding='utf8')
    shutil.copy2(package/'portable_manage.py',root/'ops/manage.py')
    t=transport_source
    t=re.sub(r"IMAGE = '[^']+'","IMAGE = 'sha256:514530039dcae6e04334d43e3157ded83382009bd339a16e33d711b66d08d7cb'",t)
    # Keep the existing PEAK VID/PID and physical channel identity checks.
    marker="    base = Path('/sys/class/net')/name"
    t=replace_checked(t,marker,"    config=json.loads((ROOT/'device_config.json').read_text())\n    assert config.get('can_mapping_confirmed') is True, 'Run configure.py to confirm can0 right / can1 left'\n"+marker)
    marker="    port = {'can0':'0x0', 'can1':'0x1'}[name]"
    t=replace_checked(t,marker,"    actual_serial=(usb/'serial').read_text().strip() if (usb/'serial').exists() else None\n    assert actual_serial and actual_serial==config.get('can_serial'), 'CAN adapter serial differs from configured device'\n"+marker)
    # Only the existing functions are used; omit its historical standalone main.
    t=t[:t.index('\ndef main():')]
    transport=root/'ops/restore_transport.py';transport.write_text(t,encoding='utf8')
    p=root/'ops/prepare_can_20261020.py';s=p.read_text(encoding='utf8')
    s=re.sub(r"OLD_TRANSPORT = Path\('[^']+'\)","OLD_TRANSPORT = ROOT/'ops/restore_transport.py'",s)
    s=re.sub(r"TRANSPORT_SHA = '[0-9a-f]+'","TRANSPORT_SHA = '"+hashlib.sha256(transport.read_bytes()).hexdigest()+"'",s)
    p.write_text(s,encoding='utf8')
    for n,h in before.items():
        if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h:raise ValueError('Motion source changed: '+n)
    return before
