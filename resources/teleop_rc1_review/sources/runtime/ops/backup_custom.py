"""Preserve the exact stopped customized stack; never enable or move motors."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
import urllib.request

IDENTITY = '4d915ea7f02c43316e5b217158715fee5f2fc23ed6f3e84716861a0e77f55bf5'
OTHERS = ['34281d6ba939682e65360bee7ac80c91f6897b38291c855bae1f3905a001d3a6',
          '05dc367eb4d9f1923ae9788766b7b5e38b7e09cd520ca6ddd25a299ce1232df2']
UNITS = ['openarm-teleop-standby.service', 'openarm-vr-view-mirror.service']
BASE = Path('/home/robot/Teleop')
os.umask(0o077)


def call(args, timeout=30):
    return subprocess.check_output(args, text=True, timeout=timeout)


def inspect(identity):
    return json.loads(call(['docker', 'inspect', identity]))[0]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temp.replace(path)


def stream_archive(args, path):
    with path.with_suffix(path.suffix + '.stderr').open('wb') as errors:
        process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=errors)
        try:
            with path.open('wb') as raw:
                with gzip.GzipFile(fileobj=raw, mode='wb', compresslevel=1, mtime=0) as output:
                    shutil.copyfileobj(process.stdout, output, 4 * 1024 * 1024)
            if process.wait(timeout=30):
                raise RuntimeError('Archive command failed: ' + str(args[:3]))
        except BaseException:
            process.kill()
            process.wait()
            raise
        finally:
            process.stdout.close()
    return {'bytes': path.stat().st_size, 'sha256': digest(path)}


def main():
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
    folder = BASE / ('VR_to_Openarm_custom_backup_' + stamp)
    folder.mkdir()
    state = {'kind': 'customized_4443_full_backup', 'folder': str(folder),
             'started_utc': stamp, 'container_id': IDENTITY, 'complete': False,
             'phase': 'checking_disabled_state', 'archives': {}}

    def progress(phase):
        state.update(phase=phase, updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
        atomic(folder / 'manifest.json', state)

    try:
        container = inspect(IDENTITY)
        assert container['Id'] == IDENTITY and container['State']['Running']
        assert container['Name'] == '/codex-openarm-motor-toggle-20261008'
        for identity in OTHERS:
            assert not inspect(identity)['State']['Running']
        running_ids = call(['docker', 'ps', '-q']).split()
        running = json.loads(call(['docker', 'inspect', *running_ids]))
        volume = next(m for m in container['Mounts'] if m['Destination'] == '/home/operator')
        assert volume['Type'] == 'volume'
        writers = [c['Id'] for c in running if any(
            m.get('Name') == volume['Name'] and m.get('RW') for m in c['Mounts'])]
        assert writers == [IDENTITY], 'Another running container shares the writable home volume'
        state['volume'] = volume
        atomic(folder / 'container.inspect.json', container)
        (folder / 'container.diff.txt').write_text(call(['docker', 'diff', IDENTITY]))
        (folder / 'services').mkdir()
        state['unit_states'] = {}
        for unit in UNITS:
            state['unit_states'][unit] = {}
            for query in ['is-enabled', 'is-active']:
                result = subprocess.run(['systemctl', '--user', query, unit], capture_output=True, text=True)
                state['unit_states'][unit][query] = result.stdout.strip()
            (folder / 'services' / (unit + '.effective')).write_text(
                call(['systemctl', '--user', 'cat', unit]))
        with tarfile.open(folder / 'host_configuration.tar.gz', 'w:gz') as archive:
            for name in UNITS:
                for path in [Path.home()/'.config/systemd/user'/name,
                             Path.home()/'.config/systemd/user'/(name+'.d')]:
                    if path.exists():
                        archive.add(path, arcname=str(path).lstrip('/'), recursive=True)
            for name in ['AGENTS.md', 'PROJECT.md', 'registry.json', 'integration/OWNERSHIP.json']:
                path = Path('/home/robot/openarm_station') / name
                archive.add(path, arcname=str(path).lstrip('/'))
        state['archives']['host_configuration.tar.gz'] = {
            'bytes': (folder/'host_configuration.tar.gz').stat().st_size,
            'sha256': digest(folder/'host_configuration.tar.gz')}
        with urllib.request.urlopen('http://127.0.0.1:8767/state', timeout=3) as response:
            motors = json.load(response)
        assert motors['online'] == motors['disabled'] == 16 and motors['enabled'] == 0
        assert not motors['output_enabled'] and not motors['busy'] and not motors['bus_error']
        atomic(folder / 'disabled_before_stop.json', motors)
        progress('stopping_customized_stack')
        call(['systemctl', '--user', 'stop', *UNITS])
        call(['docker', 'stop', '-t', '12', IDENTITY], timeout=25)
        assert not inspect(IDENTITY)['State']['Running']
        progress('committing_container')
        tag = 'openarm-customized:backup-' + stamp.lower()
        image_id = call(['docker', 'commit', '--message',
                         'Preserved 4443 customized runtime before original-control rollback; motors disabled',
                         IDENTITY, tag], timeout=120).strip()
        state.update(image_tag=tag, image_id=image_id, original_container_preserved=True)
        atomic(folder/'image.inspect.json', json.loads(call(['docker', 'image', 'inspect', image_id]))[0])
        progress('saving_home_volume')
        state['archives']['home-operator.tar.gz'] = stream_archive(
            ['docker', 'cp', '-a', IDENTITY+':/home/operator/.', '-'], folder/'home-operator.tar.gz')
        with tarfile.open(folder/'home-operator.tar.gz', 'r|gz') as archive:
            count = 0
            for member in archive:
                assert not member.name.startswith('/') and '..' not in Path(member.name).parts
                count += 1
            state['home_archive_members'] = count
        progress('saving_runtime_sources')
        paths = ['/opt/motor_toggle', '/opt/motor_toggle_ws/src', '/opt/readme_ws/src',
                 '/opt/original_real_ws/src', '/opt/venv-teleop-xr/lib/python3.10/site-packages/teleop_xr',
                 '/opt/vr_view_mirror', '/opt/deployment']
        for index, path in enumerate(paths):
            name = 'source-' + str(index) + '.tar.gz'
            state['archives'][name] = dict(stream_archive(
                ['docker', 'cp', '-a', IDENTITY+':'+path, '-'], folder/name), source_path=path)
            progress('saving_runtime_sources')
        progress('saving_portable_image')
        state['archives']['container-image.tar.gz'] = stream_archive(
            ['docker', 'save', image_id], folder/'container-image.tar.gz')
        progress('verifying_archives')
        for name, item in state['archives'].items():
            assert digest(folder/name) == item['sha256']
        state.update(complete=True, finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
        progress('complete')
    except BaseException as error:
        state['error'] = repr(error)
        progress('failed')
        raise


if __name__ == '__main__':
    main()
