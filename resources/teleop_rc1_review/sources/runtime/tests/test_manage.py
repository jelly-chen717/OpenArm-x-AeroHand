"""Management contract tests; subprocess and HTTP calls are always mocked."""
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'ops'))


def load_manager():
    spec = importlib.util.spec_from_file_location('original_manager_under_test', PROJECT / 'ops/manage.py')
    module = importlib.util.module_from_spec(spec)
    with mock.patch('subprocess.check_output', side_effect=AssertionError('Unmocked subprocess')), \
            mock.patch('subprocess.run', side_effect=AssertionError('Unmocked subprocess')), \
            mock.patch('urllib.request.urlopen', side_effect=AssertionError('Unmocked HTTP')):
        spec.loader.exec_module(module)
    return module


manager = load_manager()
NEW = 'a' * 64


class ManageTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='manage-unittest-', dir=PROJECT)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.backup = self.root / 'backup'
        self.backup.mkdir()
        (self.backup / 'services').mkdir()
        self.dropin = self.root / 'services/standby.d/90-original-control-20261019.conf'
        self.dropin.parent.mkdir(parents=True)
        self.dropin.write_text('[Service]\nExecStart=\nExecStart=python3 manage.py start\n')
        self.other_override = self.dropin.parent / '20-user-setting.conf'
        self.other_override.write_text('# Preserved user override\n')
        self.settings = {'schema_version': 1, 'container_id': NEW,
                         'container_name': manager.NAME, 'image': manager.BACKUP_IMAGE,
                         'labels': {'openarm.vr.original-control': '20261019'},
                         'owned_dropin_sha': hashlib.sha256(self.dropin.read_bytes()).hexdigest()}
        old_home = {'Type': 'volume', 'Name': 'preserved-home', 'Source': '/volumes/preserved-home',
                    'Destination': '/home/operator', 'RW': True}
        self.rows = {identity: {'Id': identity, 'Name': '/test-' + identity[:8],
                     'Image': 'sha256:' + 'b' * 64, 'State': {'Running': False}, 'Mounts': []}
                     for identity in [manager.OLD, *manager.TESTS]}
        self.rows[manager.OLD].update(Name='/codex-openarm-motor-toggle-20261008', Mounts=[old_home])
        self.rows[NEW] = {'Id': NEW, 'Name': '/' + manager.NAME, 'Image': self.settings['image'],
                          'Config': {'Labels': dict(self.settings['labels'])},
                          'HostConfig': {'NetworkMode': 'host', 'RestartPolicy': {'Name': 'no'}},
                          'State': {'Running': False}, 'Mounts': []}
        (self.backup / 'container.inspect.json').write_text(json.dumps(self.rows[manager.OLD]))
        self.manifest = {'complete': True, 'phase': 'complete', 'kind': 'customized_4443_full_backup',
                         'folder': str(self.backup), 'container_id': manager.OLD,
                         'image_id': manager.BACKUP_IMAGE, 'original_container_preserved': True,
                         'archives': {}, 'unit_states': {unit: {'is-enabled': 'enabled'} for unit in manager.UNITS}}
        for name in ['host_configuration.tar.gz', 'home-operator.tar.gz', 'container-image.tar.gz',
                     *(f'source-{i}.tar.gz' for i in range(7))]:
            data = ('mock archive ' + name).encode()
            (self.backup / name).write_bytes(data)
            self.manifest['archives'][name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        self.unit_text = {unit: '# original ' + unit + '\n' for unit in manager.UNITS}
        for unit, text in self.unit_text.items():
            (self.backup / 'services' / (unit + '.effective')).write_text(text)
        self.write_manifest()
        self.status = {'mode': 'original_standby', 'output_enabled': False, 'controllers': {},
                       'hardware': {key: {'state_id': 1} for key in manager.HARDWARE}, 'hardware_age_ms': 20,
                       'driver_quiescent': True, 'known_motor_enabled': False}
        self.extra_manager = False
        self.different_unit = None
        self.calls = []
        for key, value in [('ROOT', self.root), ('HOST_PROJECT', self.root), ('BACKUP', self.backup),
                           ('DROPIN', self.dropin)]:
            patcher = mock.patch.object(manager, key, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = mock.patch.object(manager.subprocess, 'check_output', side_effect=self.fake_command)
        self.process = patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch.object(manager.subprocess, 'run', side_effect=AssertionError('Unexpected subprocess.run'))
        self.unused_run = patcher.start()
        self.addCleanup(patcher.stop)
        patcher = mock.patch.object(manager.urllib.request, 'urlopen', side_effect=self.fake_http)
        self.http = patcher.start()
        self.addCleanup(patcher.stop)

    def write_manifest(self):
        path = self.backup / 'manifest.json'
        path.write_text(json.dumps(self.manifest))
        self.settings['backup_manifest_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        (self.root / 'runtime.json').write_text(json.dumps(self.settings))

    def fake_http(self, url, timeout):
        self.assertEqual(url, manager.STATUS_URL)
        self.calls.append(('http', url))
        return io.BytesIO(json.dumps(self.status).encode())

    def managers(self):
        rows = []
        for identity, pid in [(NEW, '201'), (manager.OLD, '301')]:
            if self.rows[identity]['State']['Running']:
                rows.append(pid + ' /opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args')
        if self.extra_manager:
            rows.append('999 /elsewhere/ros2_control_node --ros-args')
        return rows

    def fake_command(self, arguments, text, timeout, **options):
        args = list(arguments)
        self.calls.append(tuple(args))
        self.assertTrue(text)
        if args[:2] == ['docker', 'inspect']:
            return json.dumps([self.rows[args[2]]])
        if args == ['timedatectl','show','-p','NTPSynchronized','--value']:
            return 'yes\n'
        if args[:2] == ['docker', 'start']:
            self.rows[args[2]]['State']['Running'] = True
            return args[2]
        if args[:2] == ['docker', 'stop']:
            self.rows[args[-1]]['State']['Running'] = False
            return args[-1]
        if args[:2] == ['docker', 'top']:
            pid = '201' if args[2] == NEW else '301'
            return 'PID COMMAND\n' + next((line for line in self.managers() if line.startswith(pid)), '')
        if args[:3] == ['ps', '-eo', 'pid,args']:
            return '\n'.join(self.managers())
        if args[:2] == ['systemctl', '--user']:
            if args[2] == 'cat':
                return self.unit_text[args[3]] + ('# new unrelated change\n' if args[3] == self.different_unit else '')
            if args[2] == 'start' and args[3] == manager.UNITS[0]:
                self.assertFalse(self.dropin.exists(), 'Custom service started before owned drop-in removal')
                self.rows[manager.OLD]['State']['Running'] = True
            return ''
        raise AssertionError('Unexpected mocked command: ' + repr(args))

    def mutations(self):
        return [call for call in self.calls if call[:2] in [('docker', 'start'), ('docker', 'stop')]
                or (call[:2] == ('systemctl', '--user') and call[2] != 'cat')]

    def test_import_is_inert(self):
        self.assertEqual(load_manager().NAME, manager.NAME)

    def test_start_requires_complete_checksum_pinned_backup(self):
        self.manifest['complete'] = False
        self.write_manifest()
        with self.assertRaises(RuntimeError):
            manager.start()
        self.assertEqual(self.mutations(), [])
        self.manifest['complete'] = True
        self.write_manifest()
        (self.backup / 'manifest.json').write_text('{}')
        with self.assertRaises(RuntimeError):
            manager.start()
        self.assertEqual(self.mutations(), [])

    def test_start_rejects_missing_or_changed_size_backup_archives(self):
        (self.backup / 'home-operator.tar.gz').write_bytes(b'changed')
        with self.assertRaises(RuntimeError):
            manager.start()
        self.assertEqual(self.mutations(), [])

    def test_start_refuses_wrong_id_name_image_or_labels(self):
        original = copy.deepcopy(self.rows[NEW])
        variants = [dict(original, Id='c' * 64), dict(original, Name='/wrong-name'),
                    dict(original, Image='sha256:' + 'd' * 64), dict(original, Config={'Labels': {}})]
        for row in variants:
            with self.subTest(row=row):
                self.rows[NEW] = row
                with self.assertRaises(RuntimeError):
                    manager.start()
                self.assertEqual(self.mutations(), [])

    def test_start_refuses_any_preserved_or_test_control_stack_running(self):
        for identity in [manager.OLD, *manager.TESTS]:
            with self.subTest(identity=identity):
                self.rows[identity]['State']['Running'] = True
                with self.assertRaises(RuntimeError):
                    manager.start()
                self.rows[identity]['State']['Running'] = False
                self.assertEqual(self.mutations(), [])

    def test_start_refuses_another_manager_and_shared_writable_custom_home(self):
        self.extra_manager = True
        with self.assertRaises(RuntimeError):
            manager.start()
        self.extra_manager = False
        self.rows[NEW]['Mounts'] = copy.deepcopy(self.rows[manager.OLD]['Mounts'])
        with self.assertRaises(RuntimeError):
            manager.start()
        self.assertEqual(self.mutations(), [])

    def test_start_only_starts_the_exact_original_container(self):
        result = manager.start()
        self.assertEqual(self.mutations(), [('docker', 'start', NEW)])
        self.assertFalse(result['automatic_enable'])
        self.http.assert_not_called()

    def test_unsynchronized_clock_prevents_any_ros_container_start(self):
        with mock.patch.object(manager,'wait_clock_sync',side_effect=RuntimeError('clock not synchronized')):
            with self.assertRaisesRegex(RuntimeError,'clock not synchronized'):
                manager.start()
        self.assertEqual(self.mutations(),[])

    def test_clock_sync_wait_rechecks_and_is_bounded(self):
        with mock.patch.object(manager,'run',side_effect=['no\n','yes\n']) as run, \
             mock.patch.object(manager.time,'sleep') as sleep:
            manager.wait_clock_sync()
        self.assertEqual(run.call_count,2)
        sleep.assert_called_once_with(1)
        with mock.patch.object(manager,'run',return_value='no\n'), \
             mock.patch.object(manager.time,'monotonic',side_effect=[0,60]), \
             mock.patch.object(manager.time,'sleep') as sleep:
            with self.assertRaisesRegex(RuntimeError,'not synchronized'):
                manager.wait_clock_sync()
        sleep.assert_not_called()

    def test_unknown_clock_sync_does_not_count_as_synchronized(self):
        with mock.patch.object(manager,'run',side_effect=OSError('unavailable')), \
             mock.patch.object(manager.time,'monotonic',side_effect=[0,60]):
            with self.assertRaisesRegex(RuntimeError,'not synchronized'):
                manager.wait_clock_sync()

    def test_stop_refuses_active_hardware_loaded_controllers_stale_or_enabled_state(self):
        self.rows[NEW]['State']['Running'] = True
        original = copy.deepcopy(self.status)
        variants = [dict(original, output_enabled=True), dict(original, controllers={'arm': 'inactive'}),
                    dict(original, hardware_age_ms=5000), dict(original, hardware_age_ms=None),
                    dict(original, hardware_age_ms=float('nan')), dict(original, hardware_age_ms=True),
                    dict(original, driver_quiescent=False), dict(original, driver_quiescent=None),
                    dict(original, known_motor_enabled=True)]
        for state in [2, 3]:
            row = copy.deepcopy(original)
            row['hardware'][next(iter(manager.HARDWARE))]['state_id'] = state
            variants.append(row)
        for value in variants:
            with self.subTest(value=value):
                self.status = value
                with self.assertRaises(RuntimeError):
                    manager.stop()
                self.assertEqual(self.mutations(), [])

    def test_stop_checks_fresh_hardware_before_exact_container_stop(self):
        self.rows[NEW]['State']['Running'] = True
        # Unconfigured hardware intentionally does not poll CAN. Unknown raw
        # motor state must not deadlock stopping a freshly quiescent driver.
        self.status.update(output_enabled=None, known_motor_enabled=None)
        self.assertTrue(manager.stop()['stopped'])
        self.assertEqual(self.mutations(), [('docker', 'stop', '-t', '12', NEW)])
        self.assertLess(self.calls.index(('http', manager.STATUS_URL)), self.calls.index(self.mutations()[0]))

    def test_stop_is_inert_if_already_stopped(self):
        self.assertTrue(manager.stop()['stopped'])
        self.assertEqual(self.mutations(), [])
        self.http.assert_not_called()

    def test_restore_preserves_a_dropin_that_is_not_owned(self):
        self.rows[NEW]['State']['Running'] = True
        self.dropin.write_text('# Changed by another owner\n')
        with self.assertRaises(RuntimeError):
            manager.restore_custom()
        self.assertEqual(self.dropin.read_text(), '# Changed by another owner\n')
        self.assertEqual(self.mutations(), [])

    def test_restore_rejects_changed_custom_image_or_home_volume(self):
        original = copy.deepcopy(self.rows[manager.OLD])
        for change in ['image', 'volume']:
            self.rows[manager.OLD] = copy.deepcopy(original)
            if change == 'image':
                self.rows[manager.OLD]['Image'] = 'sha256:' + 'e' * 64
            else:
                self.rows[manager.OLD]['Mounts'][0]['Name'] = 'replacement-volume'
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                manager.restore_custom()
            self.assertEqual(self.mutations(), [])

    def test_restore_order_and_enabled_states_preserve_other_overrides(self):
        self.rows[NEW]['State']['Running'] = True
        self.manifest['unit_states'][manager.UNITS[1]]['is-enabled'] = 'disabled'
        self.write_manifest()
        result = manager.restore_custom()
        changes = self.mutations()
        self.assertEqual(changes[0], ('docker', 'stop', '-t', '12', NEW))
        self.assertLess(changes.index(('systemctl', '--user', 'stop', *manager.UNITS)),
                        changes.index(('systemctl', '--user', 'daemon-reload')))
        self.assertLess(changes.index(('systemctl', '--user', 'daemon-reload')),
                        changes.index(('systemctl', '--user', 'start', manager.UNITS[0])))
        self.assertIn(('systemctl', '--user', 'enable', manager.UNITS[0]), changes)
        self.assertIn(('systemctl', '--user', 'disable', manager.UNITS[1]), changes)
        self.assertFalse(self.dropin.exists())
        self.assertEqual(self.other_override.read_text(), '# Preserved user override\n')
        self.assertTrue(self.rows[manager.OLD]['State']['Running'])
        self.assertFalse(self.rows[NEW]['State']['Running'])
        self.assertFalse(result['automatic_enable'])

    def test_restore_refuses_changed_effective_unit_without_overwriting_it(self):
        self.different_unit = manager.UNITS[0]
        with self.assertRaises(RuntimeError):
            manager.restore_custom()
        self.assertFalse(self.rows[manager.OLD]['State']['Running'])
        self.assertTrue(self.other_override.exists())
        self.assertNotIn(('systemctl', '--user', 'start', manager.UNITS[0]), self.mutations())

    def test_readonly_status_does_not_mutate_any_container_or_service(self):
        self.assertIn('/' + manager.NAME, manager.status()['containers'])
        self.assertEqual(self.mutations(), [])
        self.http.assert_not_called()


if __name__ == '__main__':
    unittest.main()
