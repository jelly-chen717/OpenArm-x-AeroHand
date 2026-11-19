"""Stage original sources on the target after the customized stack is backed up.

Run this file explicitly with target Python 3.  It never starts containers,
executes ROS/CAN code, changes an existing source tree, or removes partial work.
The original workspace's absolute /test links are intentionally preserved:
the new runtime must mount ROOT at /test and ROOT/state at /state.
"""

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import time
import uuid
import zipfile


ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
TEST = Path('/home/robot/Teleop/VR_to_Openarm_test_20261018')
ORIGINAL = Path('/home/robot/Teleop/VR_to_Openarm')
ARCHIVE = Path('/home/robot/Teleop/VR_to_Openarm.zip')
BACKUP = Path('/home/robot/Teleop/VR_to_Openarm_custom_backup_20261019T070523Z')
OLD_ID = '4d915ea7f02c43316e5b217158715fee5f2fc23ed6f3e84716861a0e77f55bf5'
OTHER_IDS = (
    '34281d6ba939682e65360bee7ac80c91f6897b38291c855bae1f3905a001d3a6',
    '05dc367eb4d9f1923ae9788766b7b5e38b7e09cd520ca6ddd25a299ce1232df2',
)
ARCHIVE_SHA = 'd74860675e9ec4df631835b74a2b5fbcd493a0e43fb86644634020c5ec4e05d3'
WHEEL_SHA = 'd799fa611b6d16e2cc6324ef164ee6e3eb00a1f7bbed1634e634b7de1b52e6be'
SELECTED = (
    'ros2_ws/src/openarm_can/',
    'ros2_ws/src/openarm_description/',
    'ros2_ws/src/openarm_ros2/openarm_hardware/',
    'ros2_ws/src/openarm_ros2/openarm_bringup/',
    'ros2_ws/src/openarm_ros2/openarm_teleop_bridge/',
)
IGNORED_PARTS = {'build', 'install', 'log', '__pycache__', '.git'}
SUPPORT = {'teleop_xr/cert.pem', 'teleop_xr/key.pem'}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def utc():
    return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda: source.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def command(args, timeout=60):
    result = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
    require(result.returncode == 0,
            'Command failed: ' + ' '.join(args[:3]) + ': ' + result.stderr.strip())
    return result.stdout


def inspect_stopped():
    records = json.loads(command(['docker', 'inspect', OLD_ID, *OTHER_IDS]))
    require(len(records) == 3, 'Unexpected container inventory')
    result = []
    for expected, item in zip((OLD_ID, *OTHER_IDS), records):
        require(item['Id'] == expected, 'Container identity mismatch')
        require(not item['State']['Running'], 'A preserved/control container is running')
        require(not item['State'].get('Paused', False), 'A container is paused')
        result.append({'id': item['Id'], 'name': item['Name'],
                       'status': item['State']['Status'], 'running': False})
    require(records[0]['Name'] == '/codex-openarm-motor-toggle-20261008',
            'Preserved container name mismatch')
    return result


def backup_preflight():
    manifest_path = BACKUP / 'manifest.json'
    require(manifest_path.is_file() and not manifest_path.is_symlink(),
            'Backup manifest is missing or is a symlink')
    manifest = json.loads(manifest_path.read_text())
    require(manifest.get('complete') is True and manifest.get('phase') == 'complete',
            'Customized backup is not complete; no staging is allowed')
    require(manifest.get('folder') == str(BACKUP)
            and manifest.get('container_id') == OLD_ID
            and manifest.get('original_container_preserved') is True,
            'Backup identity does not match the preserved customized stack')
    required = {'host_configuration.tar.gz', 'home-operator.tar.gz',
                'container-image.tar.gz', *(f'source-{i}.tar.gz' for i in range(7))}
    require(required <= manifest.get('archives', {}).keys(), 'Backup archive set is incomplete')
    for name, row in manifest['archives'].items():
        require(PurePosixPath(name).name == name, 'Unsafe backup archive name')
        path = BACKUP / name
        require(path.is_file() and not path.is_symlink()
                and path.stat().st_size == row['bytes'], 'Backup archive is absent/truncated: ' + name)
        require(len(row.get('sha256', '')) == 64, 'Backup archive hash is missing: ' + name)
    image = json.loads(command(['docker', 'image', 'inspect', manifest['image_id']]))
    require(len(image) == 1 and image[0]['Id'] == manifest['image_id'],
            'Committed rollback image is not available')
    return {'path': str(manifest_path), 'sha256': digest(manifest_path),
            'complete': True, 'image_id': manifest['image_id'],
            'archive_checks': 'manifest completed after its own full SHA verification; sizes rechecked',
            'archive_count': len(manifest['archives'])}


def safe_zip_name(name):
    parts = PurePosixPath(name).parts
    require(name and '\\' not in name and not name.startswith('/')
            and '..' not in parts and not any(':' in part for part in parts),
            'Unsafe ZIP member path')
    return parts


def source_preflight():
    require(digest(ARCHIVE) == ARCHIVE_SHA, 'Original ZIP hash mismatch')
    baseline_path = TEST / 'baseline/archive_source_manifest.json'
    baseline = json.loads(baseline_path.read_text())
    require(isinstance(baseline, list) and len(baseline) == 303,
            'Expected exactly 303 baseline source records')
    rows = {}
    for row in baseline:
        rel = row['path']
        safe_zip_name(rel)
        require(row.get('type') == 'file' and rel not in rows
                and any(rel.startswith(prefix) for prefix in SELECTED),
                'Invalid/duplicate baseline source record')
        rows[rel] = row
    canonical = {}
    with zipfile.ZipFile(ARCHIVE) as archive:
        for member in archive.infolist():
            safe_zip_name(member.filename)
            if member.is_dir() or not member.filename.startswith('VR_to_Openarm/'):
                continue
            rel = member.filename[len('VR_to_Openarm/'):]
            if not any(rel.startswith(prefix) for prefix in SELECTED):
                continue
            if IGNORED_PARTS.intersection(PurePosixPath(rel).parts):
                continue
            require(rel not in canonical, 'Duplicate source ZIP member: ' + rel)
            require(not stat.S_ISLNK(member.external_attr >> 16), 'Source ZIP contains symlink')
            data = archive.read(member)
            canonical[rel] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
    require(set(canonical) == set(rows) and len(canonical) == 303,
            'ZIP selected source inventory differs from the 303-file baseline')
    actual = set()
    for current, dirs, files in os.walk(TEST / 'ros2_ws/src', followlinks=False):
        for name in dirs + files:
            path = Path(current) / name
            require(not path.is_symlink(), 'Test source contains unexpected symlink: ' + str(path))
        for name in files:
            path = Path(current) / name
            require(path.is_file(), 'Test source contains a non-regular file')
            actual.add(path.relative_to(TEST).as_posix())
    require(actual == set(canonical), 'Test source has missing/additional files')
    comparison = []
    mismatches = []
    for rel in sorted(canonical):
        expected = canonical[rel]
        original, test = ORIGINAL / rel, TEST / rel
        require(original.is_file() and test.is_file()
                and not original.is_symlink() and not test.is_symlink(),
                'Source missing/non-regular: ' + rel)
        item = {'path': rel, 'bytes': expected['bytes'], 'zip_sha256': expected['sha256'],
                'baseline_sha256': rows[rel]['sha256'],
                'original_sha256': digest(original), 'test_sha256': digest(test)}
        item['match'] = all(item[key] == expected['sha256'] for key in
                            ('baseline_sha256', 'original_sha256', 'test_sha256'))
        if not item['match'] or rows[rel]['bytes'] != expected['bytes']:
            mismatches.append(rel)
        comparison.append(item)
    return comparison, mismatches, {'path': str(baseline_path), 'sha256': digest(baseline_path)}


def tree_manifest(base):
    """Hash only regular files; record link text without following outside links."""
    result = []
    for current, dirs, files in os.walk(base, followlinks=False):
        for name in sorted(dirs + files):
            path = Path(current) / name
            rel = path.relative_to(base).as_posix()
            if path.is_symlink():
                result.append({'path': rel, 'type': 'symlink', 'target': os.readlink(path)})
            elif path.is_file():
                result.append({'path': rel, 'type': 'file', 'bytes': path.stat().st_size,
                               'sha256': digest(path)})
            else:
                require(path.is_dir(), 'Unexpected special file: ' + str(path))
    return sorted(result, key=lambda row: row['path'])


def assert_new_destination(path):
    require(path == ROOT or ROOT in path.parents, 'Destination is outside the new project')
    for parent in (path, *path.parents):
        if parent == ROOT.parent:
            break
        require(not parent.is_symlink(), 'Destination path traverses a symlink')
    require(not path.exists() and not path.is_symlink(), 'Destination already exists: ' + str(path))


def write_new_json(path, value):
    with path.open('x', encoding='utf-8') as output:
        json.dump(value, output, ensure_ascii=False, indent=2)
        output.write('\n')


def wheel_preflight():
    wheels = sorted((TEST / 'vendor').glob('teleop_xr-1.3.5-*.whl'))
    require(len(wheels) == 1 and not wheels[0].is_symlink(), 'Expected one original wheel')
    wheel = wheels[0]
    require(digest(wheel) == WHEEL_SHA, 'Original teleop_xr wheel hash mismatch')
    with zipfile.ZipFile(wheel) as archive:
        names = set()
        for member in archive.infolist():
            safe_zip_name(member.filename)
            require(member.filename not in names, 'Duplicate wheel entry')
            names.add(member.filename)
            mode = member.external_attr >> 16
            require(not stat.S_ISLNK(mode), 'Wheel contains a symlink')
            require(not mode or stat.S_IFMT(mode) in (0, stat.S_IFREG, stat.S_IFDIR),
                    'Wheel contains a special entry')
        require(SUPPORT <= names, 'Pinned wheel TLS members are missing')
        require(len(names) == 92, 'Pinned wheel must contain exactly 92 original members')
        require('teleop_xr/dist/index.html' in names, 'Stock VR frontend is absent from wheel')
    return wheel


def compiled_libraries(base):
    result = []
    for current, dirs, files in os.walk(base, followlinks=False):
        for name in sorted(dirs + files):
            if not (name.endswith('.so') or '.so.' in name):
                continue
            path = Path(current) / name
            item = {'path': str(path), 'runtime_path': '/test/' + path.relative_to(ROOT).as_posix(),
                    'symlink': path.is_symlink()}
            if path.is_symlink():
                target = os.readlink(path)
                item['target'] = target
                # Do not dereference an absolute /test link through the host filesystem.
                if target.startswith('/test/'):
                    mapped = ROOT / target[len('/test/'):]
                    item['mapped_staged_target'] = str(mapped)
                    item['mapped_target_present'] = mapped.is_file()
            elif path.is_file():
                item.update(bytes=path.stat().st_size, sha256=digest(path))
            result.append(item)
    return result


def main():
    require(os.name == 'posix', 'This staging script is for explicit target-side execution only')
    os.umask(0o077)
    report = {'kind': 'original_4443_source_staging', 'started_utc': utc(),
              'root': str(ROOT), 'complete': False, 'phase': 'preflight',
              'original_code_modified': False, 'containers_started': False,
              'ros_can_commands_sent': False,
              'planned_ports': {'chinese_read_only_console': 4443, 'stock_vr': 4446},
              'runtime_mount_contract': {'project': '/test', 'state': '/state',
                                         'old_home_volume_must_not_be_mounted': True},
              'created_destinations': []}
    report_path = None
    try:
        # No target writes, including reports, before backup/identity preconditions pass.
        report['backup'] = backup_preflight()
        report['containers_before'] = inspect_stopped()
        for path in (ROOT / 'ros2_ws', ROOT / 'vendor', ROOT / 'legacy_ui',
                     ROOT / 'state/tls', ROOT / 'baseline/wheel_tls'):
            assert_new_destination(path)
        require(not (ROOT / 'reports/prepared_sources.json').exists(),
                'Completed source-staging report already exists')
        for path in (ROOT, ROOT / 'reports', ROOT / 'state', ROOT / 'baseline'):
            require(not path.is_symlink(), 'Project/report/state parent is a symlink')
            path.mkdir(mode=0o700, parents=True, exist_ok=True)
        report_path = ROOT / 'reports' / ('prepare_sources_attempt_' +
                        time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '_' + uuid.uuid4().hex + '.json')
        comparison, mismatches, baseline = source_preflight()
        report.update(archive={'path': str(ARCHIVE), 'sha256': ARCHIVE_SHA},
                      source_baseline=baseline, source_comparison=comparison,
                      source_count=len(comparison), source_mismatches=mismatches)
        require(not mismatches, '303-file source hash comparison failed; staging refused')
        wheel = wheel_preflight()
        report['wheel'] = {'source': str(wheel), 'sha256': WHEEL_SHA, 'version': '1.3.5'}

        report['phase'] = 'copying_complete_ros_workspace'
        # Preserve every absolute /test symlink in build/install, without dereferencing it.
        source_workspace = tree_manifest(TEST / 'ros2_ws')
        shutil.copytree(TEST / 'ros2_ws', ROOT / 'ros2_ws', symlinks=True)
        report['created_destinations'].append(str(ROOT / 'ros2_ws'))
        staged_workspace = tree_manifest(ROOT / 'ros2_ws')
        require(source_workspace == staged_workspace, 'Full ROS workspace copy verification failed')
        report['workspace_manifest'] = staged_workspace
        report['workspace_manifest_count'] = len(staged_workspace)
        report['absolute_test_symlinks'] = [r for r in staged_workspace
                    if r['type'] == 'symlink' and r['target'].startswith('/test/')]
        for row in comparison:
            row['staged_sha256'] = digest(ROOT / row['path'])
            require(row['staged_sha256'] == row['zip_sha256'], 'Staged source mismatch: ' + row['path'])
        report['compiled_libraries'] = compiled_libraries(ROOT / 'ros2_ws')

        report['phase'] = 'extracting_original_wheel'
        (ROOT / 'vendor').mkdir(mode=0o700)
        report['created_destinations'].append(str(ROOT / 'vendor'))
        staged_wheel = ROOT / 'vendor' / wheel.name
        shutil.copy2(wheel, staged_wheel)
        require(digest(staged_wheel) == WHEEL_SHA, 'Copied wheel hash mismatch')
        python_root = ROOT / 'vendor/python'
        python_root.mkdir(mode=0o700)
        wheel_entries = []
        with zipfile.ZipFile(staged_wheel) as archive:
            archive.extractall(python_root)
            for member in archive.infolist():
                if member.is_dir():
                    continue
                expected = hashlib.sha256(archive.read(member)).hexdigest()
                path = python_root / member.filename
                actual = digest(path)
                require(not path.is_symlink() and actual == expected,
                        'Extracted wheel member mismatch: ' + member.filename)
                wheel_entries.append({'path': member.filename, 'bytes': member.file_size,
                                      'wheel_sha256': expected, 'staged_sha256': actual, 'match': True})
        report['wheel'].update(staged_path=str(staged_wheel),
                               member_count=len(wheel_entries), entries=wheel_entries,
                               initial_extraction_all_original_bytes_verified=True,
                               compatibility_patches_applied=False)

        report['phase'] = 'copying_preserved_chinese_template'
        inspect_stopped()
        command(['docker', 'cp', OLD_ID +
                 ':/opt/venv-teleop-xr/lib/python3.10/site-packages/teleop_xr/dist',
                 str(ROOT / 'legacy_ui')], timeout=120)
        report['created_destinations'].append(str(ROOT / 'legacy_ui'))
        require((ROOT / 'legacy_ui/index.html').is_file(), 'Legacy UI index is missing after copy')
        report['legacy_ui'] = {'source_container': OLD_ID,
                    'source_path': '/opt/venv-teleop-xr/lib/python3.10/site-packages/teleop_xr/dist',
                    'destination': str(ROOT / 'legacy_ui'), 'modified': False,
                    'not_authorized_as_a_control_frontend': True,
                    'manifest': tree_manifest(ROOT / 'legacy_ui')}

        report['phase'] = 'copying_tls_without_old_home_mount'
        tls = ROOT / 'state/tls'
        tls.mkdir(mode=0o700)
        report['created_destinations'].append(str(tls))
        archived_tls = ROOT / 'baseline/wheel_tls'
        archived_tls.mkdir(mode=0o700)
        report['created_destinations'].append(str(archived_tls))
        report['tls'] = {'directory': str(tls), 'files': [],
                         'private_content_logged': False, 'old_home_volume_modified': False,
                         'deployment_configuration_exception': True,
                         'exception_scope': 'only the two packaged certificate/key files; no control/frontend code'}
        for name in ('cert.pem', 'key.pem'):
            support_path = python_root / 'teleop_xr' / name
            wheel_row, = [row for row in wheel_entries if row['path'] == 'teleop_xr/' + name]
            require(support_path.is_file() and not support_path.is_symlink()
                    and digest(support_path) == wheel_row['wheel_sha256'],
                    'Packaged TLS original bytes are not intact')
            original_tls = archived_tls / name
            # Preserve the original wheel TLS bytes in a new exclusive destination.
            with support_path.open('rb') as source, original_tls.open('xb') as output:
                shutil.copyfileobj(source, output)
            original_tls.chmod(0o600)
            require(digest(original_tls) == wheel_row['wheel_sha256'], 'TLS baseline copy mismatch')
            path = tls / name
            command(['docker', 'cp', '-L', OLD_ID +
                     ':/home/operator/.local/share/openarm/tls/' + name, str(path)])
            require(path.is_file() and not path.is_symlink() and path.stat().st_size > 0,
                    'TLS copy did not produce a regular nonempty file')
            path.chmod(0o600)
            # This is the sole explicit byte-preservation exception.  Only a file
            # just extracted and hash-verified above may be replaced with a link.
            require(digest(support_path) == wheel_row['wheel_sha256'], 'TLS exception source changed')
            support_path.unlink()
            support_path.symlink_to('/state/tls/' + name)
            wheel_row.update(deployment_tls_exception=True,
                             original_preserved_at=str(original_tls), final_type='symlink')
            report['tls']['files'].append({'name': name, 'mode': '0600',
                         'support_path': str(support_path), 'support_target': '/state/tls/' + name,
                         'original_wheel_sha256': wheel_row['wheel_sha256'],
                         'original_preserved_at': str(original_tls)})
        # Every original non-TLS member, including all control and frontend code,
        # remains byte-for-byte identical.  Both TLS originals remain in baseline.
        for row in wheel_entries:
            if row['path'] in SUPPORT:
                continue
            require(digest(python_root / row['path']) == row['wheel_sha256'],
                    'Wheel bytes changed after extraction: ' + row['path'])
        final_files = tree_manifest(python_root)
        actual_files = {row['path'] for row in final_files}
        expected_files = {row['path'] for row in wheel_entries} | SUPPORT
        require(actual_files == expected_files, 'Unexpected files were added to the original wheel tree')
        report['wheel'].update(original_control_and_frontend_bytes_preserved=True,
                               deployment_exception_files=sorted(SUPPORT))
        report['support_files'] = sorted(SUPPORT)
        report['containers_after'] = inspect_stopped()
        report.update(complete=True, phase='complete', finished_utc=utc())
        write_new_json(ROOT / 'reports/prepared_sources.json', report)
        write_new_json(report_path, report)
        print(json.dumps({'complete': True, 'root': str(ROOT),
                          'report': str(ROOT / 'reports/prepared_sources.json'),
                          'attempt_report': str(report_path), 'source_count': len(comparison),
                          'wheel_sha256': WHEEL_SHA,
                          'wheel_member_count': len(wheel_entries),
                          'legacy_file_count': len(report['legacy_ui']['manifest']),
                          'containers_started': False, 'ros_can_commands_sent': False}))
    except BaseException as error:
        report.update(complete=False, phase='failed', failed_utc=utc(), error=repr(error),
                      partial_work_preserved=True)
        if report_path is not None and not report_path.exists():
            write_new_json(report_path, report)
        print(json.dumps({'complete': False, 'phase': 'failed', 'error': repr(error),
                          'attempt_report': str(report_path) if report_path else None,
                          'partial_work_preserved': True}))
        raise


if __name__ == '__main__':
    main()
