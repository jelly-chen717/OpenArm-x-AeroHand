"""Only fake operations and a temporary local HTTP server; no ROS/Docker/CAN."""
import importlib.util
from email.message import Message
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock
import urllib.error
import urllib.request


SPEC = importlib.util.spec_from_file_location('panel_backend', Path(__file__).resolve().parents[1] / 'ops/panel_backend.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)
TOKEN = 'test-token-' + 'a' * 40


class FakeExecutor:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.blocked = set()
        self.failure = None
        self.running = self.max_running = 0
        self.physical = {'can_initialize': True, 'can_enable': True, 'can_disable': True,
                         'busy_external': False, 'initialized': False, 'bridge_enabled': False,
                         'online': 16, 'enabled': 0, 'output_enabled': False}

    def status(self):
        return dict(self.physical)

    def run(self, action, cancel_event):
        with self.lock:
            self.calls.append(action)
            self.running += 1
            self.max_running = max(self.max_running, self.running)
        try:
            self.entered.set()
            if action in self.blocked:
                while not self.release.wait(.005):
                    if cancel_event.is_set():
                        return {'success': False, 'cancelled': True, 'error': 'Cooperative cancellation'}
            if self.failure:
                raise self.failure
            return {'success': True, 'action': action, 'bridge_enabled': False}
        finally:
            with self.lock:
                self.running -= 1


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('Timed out waiting for fake worker')


class QueueTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.executor = FakeExecutor()
        self.queue = m.JobQueue(self.executor, self.directory.name)

    def tearDown(self):
        self.executor.release.set()
        self.queue.close()
        self.directory.cleanup()

    def test_get_state_is_read_only_and_job_success_does_not_invent_physical_state(self):
        for _ in range(3):
            self.assertFalse(self.queue.state()['initialized'])
        self.assertEqual(self.executor.calls, [])
        self.queue.submit('prepare', 'prepare-1')
        wait_for(lambda: not self.queue.state()['busy'])
        state = self.queue.state()
        self.assertEqual(state['last_operation']['phase'], 'complete')
        self.assertFalse(state['initialized'])
        self.assertFalse(state['bridge_enabled'])

    def test_concurrent_enables_dispatch_only_once_and_never_queue_a_second_enable(self):
        self.executor.blocked.add('enable')
        barrier = threading.Barrier(3)
        outcomes = []
        def submit(identity):
            barrier.wait()
            try:
                outcomes.append(self.queue.submit('enable', identity)[0])
            except m.RequestError as error:
                outcomes.append(error.status)
        threads = [threading.Thread(target=submit, args=(str(i),)) for i in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()
        self.assertTrue(self.executor.entered.wait(1))
        self.assertCountEqual(outcomes, [202, 409])
        self.assertEqual(self.executor.calls, ['enable'])
        self.assertEqual(self.executor.max_running, 1)

    def test_request_id_is_idempotent_before_and_after_completion(self):
        first = self.queue.submit('enable', 'same')[1]
        second = self.queue.submit('enable', 'same')[1]
        self.assertEqual(first['id'], second['id'])
        wait_for(lambda: not self.queue.state()['busy'])
        self.assertEqual(self.queue.submit('enable', 'same')[0], 200)
        self.assertEqual(self.executor.calls, ['enable'])
        with self.assertRaises(m.RequestError) as error:
            self.queue.submit('disable', 'same')
        self.assertEqual(error.exception.status, 409)

    def test_priority_disable_cancels_enable_and_all_stop_requests_coalesce(self):
        self.executor.blocked.update(('enable', 'disable'))
        self.queue.submit('enable', 'start')
        self.assertTrue(self.executor.entered.wait(1))
        first = self.queue.submit('disable', 'stop-1')[1]
        second = self.queue.submit('disable', 'stop-2')[1]
        self.assertEqual(first['id'], second['id'])
        wait_for(lambda: self.executor.calls == ['enable', 'disable'])
        third = self.queue.submit('disable', 'stop-3')[1]
        self.assertEqual(first['id'], third['id'])
        self.assertEqual(self.executor.max_running, 1)
        self.assertEqual(self.queue.jobs[0]['phase'], 'cancelled')

    def test_external_busy_blocks_prepare_and_enable_but_allows_explicit_stop(self):
        self.executor.physical.update(busy_external=True, can_disable=False)
        self.assertTrue(self.queue.state()['busy'])
        for action in ('prepare', 'enable'):
            with self.assertRaises(m.RequestError) as error:
                self.queue.submit(action)
            self.assertEqual(error.exception.status, 409)
        self.assertEqual(self.queue.submit('disable')[0], 202)
        wait_for(lambda: self.executor.calls == ['disable'])

    def test_capability_false_and_unknown_action_do_not_start_operations(self):
        self.executor.physical.update(can_enable=False)
        for action in ('enable', 'not-a-command'):
            with self.assertRaises(m.RequestError):
                self.queue.submit(action)
        self.assertEqual(self.executor.calls, [])

    def test_executor_failure_is_persisted_without_automatic_retry(self):
        self.executor.failure = RuntimeError('Fake control failure')
        self.queue.submit('enable', 'failed')
        wait_for(lambda: not self.queue.state()['busy'])
        state = self.queue.state()
        self.assertEqual(state['last_operation']['phase'], 'failed')
        self.assertEqual(state['error'], 'Fake control failure')
        journal = json.loads(self.queue.path.read_text())
        self.assertEqual(journal['jobs'][-1]['phase'], 'failed')
        self.assertEqual(self.executor.calls, ['enable'])

    def test_restart_marks_unfinished_job_interrupted_without_replay(self):
        self.queue.close()
        self.queue.path.write_text(json.dumps({'schema_version': 1, 'jobs': [
            {'id': 'old', 'action': 'enable', 'request_ids': ['old-request'], 'phase': 'running'}]}))
        self.queue = m.JobQueue(self.executor, self.directory.name)
        self.assertEqual(self.queue.state()['last_operation']['phase'], 'interrupted')
        self.assertEqual(self.queue.submit('enable', 'old-request')[1]['phase'], 'interrupted')
        self.assertEqual(self.executor.calls, [])

    def test_missing_success_verification_is_failed_not_complete(self):
        self.executor.run = lambda action, cancel_event: {'initialized': True}
        self.queue.submit('prepare')
        wait_for(lambda: not self.queue.state()['busy'])
        self.assertEqual(self.queue.state()['last_operation']['phase'], 'failed')

    def test_unjournaled_enable_is_not_dispatched(self):
        with mock.patch.object(self.queue, '_persist', side_effect=OSError('Fake disk failure')):
            with self.assertRaises(m.RequestError) as error:
                self.queue.submit('enable', 'disk-failed')
        self.assertEqual(error.exception.status, 503)
        self.assertEqual(self.executor.calls, [])

    def test_idempotency_survives_backend_restart_without_replaying_success(self):
        first = self.queue.submit('enable', 'persisted')[1]
        wait_for(lambda: not self.queue.state()['busy'])
        self.queue.close()
        self.queue = m.JobQueue(self.executor, self.directory.name)
        code, replay = self.queue.submit('enable', 'persisted')
        self.assertEqual((code, replay['id']), (200, first['id']))
        self.assertEqual(self.executor.calls, ['enable'])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.executor = FakeExecutor()
        self.queue = m.JobQueue(self.executor, self.directory.name)
        self.server = m.ThreadingHTTPServer(('127.0.0.1', 0), m.handler_type(self.queue, TOKEN))
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()

    def tearDown(self):
        self.executor.release.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        self.queue.close()
        self.directory.cleanup()

    def request(self, path, body=None, token=TOKEN, method=None, content_type='application/json'):
        data = json.dumps(body).encode() if body is not None else None
        request = urllib.request.Request('http://127.0.0.1:' + str(self.server.server_port) + path,
                                         data=data, method=method)
        if token is not None:
            request.add_header('X-OpenArm-Panel-Token', token)
        request.add_header('Content-Type', content_type)
        try:
            response = urllib.request.urlopen(request, timeout=2)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.status, json.load(response)

    def test_authentication_and_get_never_dispatch_control(self):
        self.assertEqual(self.request('/state', token=None)[0], 403)
        self.assertEqual(self.request('/set', {'enabled': True}, token='wrong')[0], 403)
        self.assertEqual(self.request('/state')[0], 200)
        self.assertEqual(self.request('/initialize')[0], 404)
        self.assertEqual(self.executor.calls, [])

    def test_initialize_only_dispatches_prepare_not_enable(self):
        self.assertEqual(self.request('/initialize', {})[0], 202)
        wait_for(lambda: self.executor.calls == ['prepare'])

    def test_exact_boolean_and_unknown_fields_are_rejected(self):
        for payload in ({'enabled': 1}, {'enabled': 'false'}, {'enabled': None}, {},
                        {'enabled': True, 'side': 'right'}, {'enabled': True, 'shell': 'echo bad'}):
            self.assertEqual(self.request('/set', payload)[0], 400, payload)
        self.assertEqual(self.request('/initialize', {'request_id': 'not-allowed'})[0], 400)
        self.assertEqual(self.executor.calls, [])

    def test_enable_and_disable_map_only_to_fixed_actions(self):
        self.assertEqual(self.request('/set', {'enabled': True, 'request_id': 'http-start'})[0], 202)
        wait_for(lambda: not self.queue.state()['busy'])
        self.assertEqual(self.request('/set', {'enabled': False, 'request_id': 'http-stop'})[0], 202)
        wait_for(lambda: self.executor.calls == ['enable', 'disable'])

    def test_body_size_type_and_unknown_endpoint_are_rejected(self):
        self.assertEqual(self.request('/set', {'enabled': True, 'request_id': 'x' * 600})[0], 413)
        self.assertEqual(self.request('/set', [True])[0], 400)
        self.assertEqual(self.request('/set', {'enabled': True}, content_type='text/plain')[0], 415)
        self.assertEqual(self.request('/arbitrary-command', {})[0], 404)
        self.assertEqual(self.executor.calls, [])

    def test_journal_does_not_contain_api_credential(self):
        self.request('/initialize', {})
        wait_for(lambda: not self.queue.state()['busy'])
        self.assertNotIn(TOKEN, self.queue.path.read_text())

    def test_ambiguous_duplicate_enabled_fields_do_not_start_control(self):
        request = urllib.request.Request(
            'http://127.0.0.1:' + str(self.server.server_port) + '/set',
            data=b'{"enabled":true,"enabled":false}',
            headers={'Content-Type': 'application/json', 'X-OpenArm-Panel-Token': TOKEN})
        with self.assertRaises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request, timeout=2)
        self.assertEqual(error.exception.code, 400)
        error.exception.close()
        self.assertEqual(self.executor.calls, [])


class RejectionDrainTests(unittest.TestCase):
    def handler(self, length, body_read=False):
        handler_class = m.handler_type(None, TOKEN)
        handler = handler_class.__new__(handler_class)
        handler.command = 'POST'
        handler.headers = Message()
        handler.headers['Content-Length'] = str(length)
        handler.connection = mock.Mock()
        handler.rfile = mock.Mock()
        handler.wfile = io.BytesIO()
        handler.body_read = body_read
        handler.send_response = mock.Mock()
        handler.send_header = mock.Mock()
        handler.end_headers = mock.Mock()
        return handler

    def test_rejected_small_body_is_drained_once_with_short_timeout(self):
        handler = self.handler(600)
        handler.reply(413, {'error': 'too large'})
        handler.connection.settimeout.assert_called_once_with(.2)
        handler.rfile.read.assert_called_once_with(600)
        self.assertTrue(handler.close_connection)
        handler.reply(400, {'error': 'already consumed'})
        handler.rfile.read.assert_called_once_with(600)

    def test_processed_body_or_unbounded_rejection_never_reads_again(self):
        for length, consumed in ((40, True), (4097, False)):
            with self.subTest(length=length, consumed=consumed):
                handler = self.handler(length, consumed)
                handler.reply(400, {'error': 'invalid'})
                handler.rfile.read.assert_not_called()
                handler.connection.settimeout.assert_not_called()


if __name__ == '__main__':
    unittest.main()
