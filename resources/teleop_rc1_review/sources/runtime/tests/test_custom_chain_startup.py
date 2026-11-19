"""Configuration and read-only UI checks; never starts processes or hardware."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


supervisor = load('custom_startup_under_test', ROOT / 'ops/supervisor.py')
WEB_SOURCE = ROOT / 'app/custom_chain/custom_chain_web.py'
if not WEB_SOURCE.exists():
    WEB_SOURCE = ROOT.parent / 'vr_motion_channel_20261021/custom_chain/custom_chain_web.py'
web = load('custom_web_under_test', WEB_SOURCE)


class CustomStartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name) / 'runtime'
        self.base.mkdir()
        self.config = Path(self.temp.name) / 'config.json'
        (self.base / 'worker.py').write_text('# real deployment source\n')
        files = {'worker.py': hashlib.sha256((self.base / 'worker.py').read_bytes()).hexdigest()}
        self.data = {'kind': 'vr_custom_chain_runtime_v1', 'schema_version': 1, 'enabled': True,
                     'scope': 'right_arm_test', 'run_id': 'a' * 32, 'files_sha256': files,
                     'runtime_id': hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()}

    def save(self):
        self.config.write_text(json.dumps(self.data))

    def test_no_configuration_preserves_original_startup(self):
        self.assertIsNone(supervisor.custom_chain_config(self.config, self.base))

    def test_manifest_selects_software_only_without_enabling_any_gate(self):
        self.save()
        with patch.object(supervisor.subprocess, 'Popen') as process:
            self.assertEqual(supervisor.custom_chain_config(self.config, self.base), self.data)
        process.assert_not_called()

    def test_explicit_bimanual_scope_is_preserved_without_starting_or_enabling(self):
        self.data['scope'] = 'bimanual_test'
        self.save()
        with patch.object(supervisor.subprocess, 'Popen') as process:
            self.assertEqual(supervisor.custom_chain_config(self.config, self.base)['scope'], 'bimanual_test')
        process.assert_not_called()

    def test_modified_file_or_manifest_is_refused(self):
        self.save()
        (self.base / 'worker.py').write_text('different code')
        with self.assertRaises(RuntimeError):
            supervisor.custom_chain_config(self.config, self.base)
        self.data['runtime_id'] = '0' * 64
        self.save()
        with self.assertRaises(RuntimeError):
            supervisor.custom_chain_config(self.config, self.base)

    def test_wrong_scope_or_run_cannot_expand_motion(self):
        for key, value in [('scope', 'both_arms'), ('run_id', '../escape'), ('enabled', 'true')]:
            original = self.data[key]
            self.data[key] = value
            self.save()
            with self.assertRaises(RuntimeError):
                supervisor.custom_chain_config(self.config, self.base)
            self.data[key] = original

    def test_path_escape_is_rejected_even_if_digest_matches(self):
        self.data['files_sha256'] = {'../config.json': '0' * 64}
        self.data['runtime_id'] = hashlib.sha256(json.dumps(self.data['files_sha256'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        self.save()
        with self.assertRaises(RuntimeError):
            supervisor.custom_chain_config(self.config, self.base)

    def test_vr_feedback_route_is_get_only(self):
        app = type('App', (), {'add_api_route': lambda _, *a, **k: calls.append((a, k))})()
        calls = []
        web.install(app)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0][0], '/api/motors')
        self.assertEqual(calls[0][1]['methods'], ['GET'])

    def test_missing_observer_does_not_fabricate_motor_feedback(self):
        with patch.object(web.urllib.request, 'urlopen', side_effect=OSError('offline')):
            state = web.feedback()
        self.assertEqual(state['state'], 'unknown')
        self.assertEqual(state['motors'], [])


if __name__ == '__main__':
    unittest.main()
