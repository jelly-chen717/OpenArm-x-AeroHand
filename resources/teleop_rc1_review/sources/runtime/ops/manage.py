# 项目中文注释（2026-10-29）：所属容器的管理器。
# 调用与职责：验证容器身份、备份、唯一驱动及待机状态，处理 start/stop/restore-custom。
# 运行边界：可启停服务或恢复定制版本；start 不自动使能电机。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Manage the original-control standby without enabling or moving motors."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parent.parent
HOST_PROJECT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
BACKUP = Path('/home/robot/Teleop/VR_to_Openarm_custom_backup_20261019T070523Z')
DROPIN = Path.home() / '.config/systemd/user/openarm-teleop-standby.service.d/90-original-control-20261019.conf'
NAME = 'openarm-vr-original-control-20261019'
OLD = '4d915ea7f02c43316e5b217158715fee5f2fc23ed6f3e84716861a0e77f55bf5'
TESTS = ['34281d6ba939682e65360bee7ac80c91f6897b38291c855bae1f3905a001d3a6',
         '05dc367eb4d9f1923ae9788766b7b5e38b7e09cd520ca6ddd25a299ce1232df2']
BACKUP_IMAGE = 'sha256:42fd90e26269004a72cc175bfa02c74fcc609af6ab2bb9bc11470ef0114c7bfb'
UNITS = ['openarm-teleop-standby.service', 'openarm-vr-view-mirror.service']
HARDWARE = {'openarm_right_hardware_interface', 'openarm_left_hardware_interface'}
STATUS_URL = 'http://127.0.0.1:8767/status'
HEX = re.compile(r'^[0-9a-f]{64}$')
MANAGER = re.compile(r'(?:^|\s)(?:\S*/)?ros2_control_node(?:\s|$)')


# 注释：条件不成立时立即拒绝本次操作，并保留可解释的错误原因。
def require(condition, message):
    if not condition:
        raise RuntimeError(message)


# 注释：执行该对象的既有工作循环；验证容器身份、备份、唯一驱动及待机状态，处理 start/stop/restore-custom。
def run(arguments, timeout=8):
    from operation_lock import child_fds
    mutation=(arguments[:2] in (['docker','start'],['docker','stop'])
              or arguments[:3] in (['systemctl','--user','start'],['systemctl','--user','stop'],
                                   ['systemctl','--user','daemon-reload'],['systemctl','--user','enable'],
                                   ['systemctl','--user','disable']))
    options={'text':True,'timeout':None if mutation else timeout}
    if mutation:
        options['pass_fds']=child_fds()
    return subprocess.check_output(arguments,**options)


# 注释：计算文件 SHA-256，用于核对原件或部署身份。
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


# 注释：读取 runtime.json 并校验容器、镜像及备份校验字段。
def runtime():
    data = json.loads((ROOT / 'runtime.json').read_text())
    require(data.get('schema_version') == 1, 'Unsupported runtime manifest')
    require(isinstance(data.get('container_id'), str) and HEX.fullmatch(data['container_id'])
            and data['container_id'] not in [OLD, *TESTS], 'Invalid original-control container ID')
    require(data.get('container_name') == NAME, 'Unexpected original-control container name')
    require(isinstance(data.get('image'), str) and data['image'].startswith('sha256:')
            and HEX.fullmatch(data['image'][7:]), 'Invalid exact runtime image')
    labels = data.get('labels')
    require(isinstance(labels, dict) and labels.get('openarm.vr.original-control') == '20261019'
            and all(isinstance(k, str) and isinstance(v, str) for k, v in labels.items()),
            'Missing original-control ownership label')
    for key in ['backup_manifest_sha256', 'owned_dropin_sha']:
        require(isinstance(data.get(key), str) and HEX.fullmatch(data[key]), 'Missing checksum: ' + key)
    return data


# 注释：读取指定容器的 Docker 元数据并核对唯一身份。
def inspect(identity):
    data = json.loads(run(['docker', 'inspect', identity]))
    require(len(data) == 1 and data[0]['Id'] == identity, 'Docker returned a different container ID')
    return data[0]


# 注释：核对当前容器名称、镜像、标签、网络与重启策略。
def original_container(settings):
    container = inspect(settings['container_id'])
    require(container['Name'] == '/' + NAME and container['Image'] == settings['image'],
            'Original-control name/image does not match runtime.json')
    actual = container.get('Config', {}).get('Labels') or {}
    require(all(actual.get(k) == v for k, v in settings['labels'].items()),
            'Original-control container ownership labels do not match')
    require(container['HostConfig']['NetworkMode'] == 'host'
            and container['HostConfig']['RestartPolicy']['Name'] == 'no',
            'Unexpected original-control network/restart policy')
    return container


# 注释：确认定制版本完整备份及必需归档存在，保留可恢复依据。
def backup_complete(settings):
    manifest_path = BACKUP / 'manifest.json'
    require(digest(manifest_path) == settings['backup_manifest_sha256'], 'Backup manifest checksum differs')
    manifest = json.loads(manifest_path.read_text())
    require(manifest.get('complete') is True and manifest.get('phase') == 'complete'
            and manifest.get('kind') == 'customized_4443_full_backup'
            and manifest.get('folder') == str(BACKUP)
            and manifest.get('container_id') == OLD
            and manifest.get('image_id') == BACKUP_IMAGE
            and manifest.get('original_container_preserved') is True, 'Customized backup is not complete')
    required = {'host_configuration.tar.gz', 'home-operator.tar.gz', 'container-image.tar.gz',
                *(f'source-{i}.tar.gz' for i in range(7))}
    archives = manifest.get('archives', {})
    require(required <= set(archives), 'Backup manifest is missing required archives')
    for name in required:
        entry = archives[name]
        path = BACKUP / name
        require(path.is_file() and path.stat().st_size == entry.get('bytes')
                and isinstance(entry.get('sha256'), str) and HEX.fullmatch(entry['sha256']),
                'Backup archive missing or changed size: ' + name)
    for unit in UNITS:
        require((BACKUP / 'services' / (unit + '.effective')).is_file(), 'Missing backed-up service: ' + unit)
        require(manifest.get('unit_states', {}).get(unit, {}).get('is-enabled')
                in ['enabled', 'enabled-runtime', 'disabled', 'static', 'indirect'],
                'Unsupported backed-up service enabled state: ' + unit)
    return manifest


# 注释：核对定制版本所用主目录卷的唯一挂载。
def home_volume(container):
    matches = [mount for mount in container.get('Mounts', []) if mount.get('Destination') == '/home/operator']
    require(len(matches) == 1 and matches[0].get('Type') == 'volume', 'Customized home volume is missing')
    return {key: matches[0].get(key) for key in ['Type', 'Name', 'Source', 'Destination', 'RW']}


# 注释：核对保留的定制版本容器与原备份对应，避免覆盖错对象。
def customized_container():
    saved = json.loads((BACKUP / 'container.inspect.json').read_text())
    require(saved['Id'] == OLD and saved['Name'] == '/codex-openarm-motor-toggle-20261008',
            'Backup does not describe the preserved customized container')
    current = inspect(OLD)
    require(current['Name'] == saved['Name'] and current['Image'] == saved['Image'],
            'Preserved customized container name/image changed')
    require(home_volume(current) == home_volume(saved), 'Preserved customized home volume changed')
    return current


# 注释：确认旧测试容器没有运行，保持单一控制源。
def require_tests_stopped():
    for identity in TESTS:
        require(not inspect(identity)['State']['Running'], 'A 4445 test control container is running')


# 注释：将系统中的控制管理器与所属容器比较，拒绝额外驱动。
def require_no_other_managers(container):
    lines = [line for line in run(['ps', '-eo', 'pid,args']).splitlines() if MANAGER.search(line)]
    managers = {line.split(None, 1)[0] for line in lines}
    owned = set()
    if container['State']['Running']:
        owned = {line.split(None, 1)[0] for line in
                 run(['docker', 'top', container['Id'], '-eo', 'pid,args']).splitlines()
                 if MANAGER.search(line)}
    require(managers <= owned, 'Another ros2_control_node is running')


# 注释：读取并校验硬件未配置状态；纯 Web 模式需另外证明没有硬件管理器。
def standby_status():
    with urllib.request.urlopen(STATUS_URL, timeout=3) as response:
        state = json.load(response)
    if state.get('mode') == 'original_web_offline':
        # This mode starts no ROS hardware process and makes no motor-state claim.
        require(state.get('web_only') is True and state.get('ros_control_started') is False,
                'Invalid Web-only status')
        require(state.get('output_enabled') is False and state.get('hardware') == {}
                and state.get('controllers') == {} and state.get('driver_quiescent') is True
                and state.get('known_motor_enabled') is None,
                'Web-only runtime reports a driver or output')
        stamp=state.get('status_generated_monotonic_s')
        require(type(stamp) in (int,float) and 0 <= time.monotonic()-stamp < 5,
                'Web-only status is stale')
        c=original_container(runtime())
        require(c['State']['Running'], 'Web-only container is not running')
        require_no_other_managers(c)
        require(not any(MANAGER.search(line) for line in run(['docker','top',c['Id'],'-eo','pid,args']).splitlines()),
                'A hardware manager is running in Web-only mode')
        return state
    output = state.get('output_enabled')
    require(state.get('mode') == 'original_standby' and (output is False or output is None),
            'Original-control output is enabled or malformed')
    hardware = state.get('hardware', {})
    require(set(hardware) == HARDWARE and all(isinstance(v, dict) and v.get('state_id') == 1
                                            for v in hardware.values()),
            'Both hardware components must be unconfigured before container stop')
    age = state.get('hardware_age_ms')
    require(type(age) in [int, float] and 0 <= age < 5000, 'Hardware standby status is stale')
    require(state.get('controllers') == {}, 'Loaded controllers prohibit standby stop')
    require(state.get('driver_quiescent') is True, 'Driver/controller quiescence is not fresh and confirmed')
    known_enabled = state.get('known_motor_enabled')
    require(known_enabled is False or known_enabled is None, 'Motor feedback reports enabled motors')
    return state


# 注释：读取当前、定制备份及旧测试容器状态；运行中才读取待机证明。
def status():
    settings = runtime()
    containers = [original_container(settings), inspect(OLD), *(inspect(i) for i in TESTS)]
    result = {'containers': {c['Name']: {'id': c['Id'], 'image': c['Image'], 'state': c['State']}
                             for c in containers}}
    if containers[0]['State']['Running']:
        try:
            result['standby'] = standby_status()
        except Exception as error:
            result['standby_error'] = str(error)
    return result


# 注释：等待系统时钟同步后再启动 ROS 定时器。
def wait_clock_sync(timeout=60):
    """Do not create ROS timers against the PC's uncorrected boot-time clock."""
    deadline=time.monotonic()+timeout
    while True:
        try:
            if run(['timedatectl','show','-p','NTPSynchronized','--value'],timeout=3).strip()=='yes':
                return
        except (OSError,subprocess.SubprocessError):
            pass
        if time.monotonic()>=deadline:
            raise RuntimeError('System clock is not synchronized; original ROS standby was not started')
        time.sleep(1)


# 注释：在镜像、备份、卷和单驱动检查通过后启动原所属容器；自动使能标记始终为 false。
def start():
    require(ROOT == HOST_PROJECT, 'Run this manager only in the intended host project')
    settings = runtime()
    backup_complete(settings)
    old = customized_container()
    require(not old['State']['Running'], 'Customized control container is still running')
    require_tests_stopped()
    container = original_container(settings)
    old_home = home_volume(old)['Name']
    require(not any(m.get('Name') == old_home and m.get('RW') for m in container.get('Mounts', [])),
            'Original-control runtime must not write the preserved customized home volume')
    require_no_other_managers(container)
    if container['State']['Running']:
        standby_status()
        return {'action': 'start', 'already_running': True, 'container_id': container['Id']}
    wait_clock_sync()
    run(['docker', 'start', container['Id']], timeout=20)
    require(original_container(settings)['State']['Running'], 'Original-control container failed to start')
    return {'action': 'start', 'container_id': container['Id'], 'automatic_enable': False}


# 注释：仅在原所属容器已证明静止时停止它，避免误停其他控制栈。
def stop():
    require(ROOT == HOST_PROJECT, 'Run this manager only in the intended host project')
    settings = runtime()
    container = original_container(settings)
    if container['State']['Running']:
        standby_status()
        run(['docker', 'stop', '-t', '12', container['Id']], timeout=25)
        require(not original_container(settings)['State']['Running'], 'Original-control container is still running')
    return {'action': 'stop', 'container_id': container['Id'], 'stopped': True}


# 注释：核对 systemd 覆盖文件仍是本部署拥有的版本。
def check_owned_dropin(settings):
    require(not DROPIN.is_symlink(), 'Managed drop-in was replaced by a symlink')
    if DROPIN.exists():
        require(DROPIN.is_file() and digest(DROPIN) == settings['owned_dropin_sha'],
                'Drop-in is not this runtime\'s unchanged owned file; preserving it')


# 注释：按原备份与所有权检查恢复保留的定制版本。
def restore_custom():
    require(ROOT == HOST_PROJECT, 'Run this manager only in the intended host project')
    settings = runtime()
    manifest = backup_complete(settings)
    customized_container()
    require_tests_stopped()
    check_owned_dropin(settings)
    stop()
    # The original unit is oneshot+RemainAfterExit. Clear its active state so
    # the next start actually executes the restored Docker command.
    run(['systemctl', '--user', 'stop', *UNITS], timeout=15)
    require_no_other_managers(customized_container())
    check_owned_dropin(settings)
    if DROPIN.exists():
        DROPIN.unlink()
    run(['systemctl', '--user', 'daemon-reload'])
    for unit in UNITS:
        expected = (BACKUP / 'services' / (unit + '.effective')).read_text()
        require(run(['systemctl', '--user', 'cat', unit]) == expected,
                'Effective service differs from backup; other overrides were preserved: ' + unit)
        enabled = manifest['unit_states'][unit]['is-enabled']
        if enabled in ['enabled', 'enabled-runtime']:
            arguments = ['systemctl', '--user', 'enable']
            if enabled == 'enabled-runtime':
                arguments.append('--runtime')
            run([*arguments, unit])
        elif enabled == 'disabled':
            run(['systemctl', '--user', 'disable', unit])
    run(['systemctl', '--user', 'start', UNITS[0]], timeout=20)
    require(customized_container()['State']['Running'], 'Customized standby service did not start its container')
    run(['systemctl', '--user', 'start', UNITS[1]], timeout=20)
    require(not original_container(settings)['State']['Running'], 'Original-control runtime unexpectedly restarted')
    require_tests_stopped()
    require_no_other_managers(customized_container())
    return {'action': 'restore-custom', 'container_id': OLD, 'customized_running': True,
            'automatic_enable': False, 'web_readiness': 'pending_IK_warmup'}


# 注释：该文件的显式执行入口；验证容器身份、备份、唯一驱动及待机状态，处理 start/stop/restore-custom。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'start', 'stop', 'restore-custom'])
    action = parser.parse_args().action
    handlers = {'status': status, 'start': start, 'stop': stop, 'restore-custom': restore_custom}
    print(json.dumps(handlers[action](), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1 and sys.argv[1]=='status':
        main()
    else:
        from operation_lock import exclusive
        with exclusive(ROOT):
            main()
