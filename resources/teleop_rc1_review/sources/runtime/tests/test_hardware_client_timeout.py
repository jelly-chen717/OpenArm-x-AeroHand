import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ops'))

spec=importlib.util.spec_from_file_location('hardwarectl_test',Path(__file__).resolve().parents[1]/'ops/hardwarectl.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def done(output='',code=0):
    return subprocess.CompletedProcess([],code,output,'')


class TimeoutTests(unittest.TestCase):
    def test_shutdown_may_continue_without_web_but_ambiguous_web_is_rejected(self):
        self.assertIsNone(m.stock_web('owned',lambda args:'PID ARGS',required=False))
        with self.assertRaises(AssertionError):
            m.stock_web('owned',lambda args:'PID ARGS',required=True)
        rows='1 python -m teleop_xr.ros2 --port 4446\n2 python -m teleop_xr.ros2 --port 4446'
        with self.assertRaises(AssertionError):
            m.stock_web('owned',lambda args:rows,required=False)

    def test_previously_paused_web_is_not_owned_for_resume(self):
        with tempfile.TemporaryDirectory() as directory:
            proc=Path(directory);(proc/'91').mkdir()
            args=b'python\0-m\0teleop_xr.ros2\0--port\0'+b'4446\0'
            (proc/'91/cmdline').write_bytes(args)
            (proc/'91/stat').write_text('91 (python) T 1')
            with mock.patch.object(m.os,'kill') as kill:
                self.assertIsNone(m.pause_web((91,args),proc=proc))
                kill.assert_not_called()
            (proc/'91/stat').write_text('91 (python) S 1')
            with mock.patch.object(m.os,'kill') as kill, mock.patch.object(m.signal,'SIGSTOP',19,create=True):
                self.assertEqual(m.pause_web((91,args),proc=proc),(91,args))
                kill.assert_called_once_with(91,m.signal.SIGSTOP)

    def test_missing_observer_after_shutdown_is_unknown_not_verified_off(self):
        read=mock.Mock(side_effect=OSError('monitor unavailable'))
        state,error=m.final_observation('disable',read)
        self.assertEqual(state,{'state':'unknown','output_enabled':None})
        self.assertIn('monitor unavailable',error)
        with self.assertRaises(OSError):
            m.final_observation('initialize-enable',read)

    def test_controlled_prepare_keeps_lease_without_forced_timeout(self):
        with mock.patch.object(m.subprocess,'run',return_value=done('ok')) as run, \
             mock.patch('operation_lock.child_fds',return_value=(41,)):
            self.assertEqual(m.host_command(['python3','prepare.py'],controlled=True),'ok')
        self.assertIsNone(run.call_args.kwargs['timeout'])
        self.assertEqual(run.call_args.kwargs['pass_fds'],(41,))

    def test_readonly_host_command_retains_bounded_timeout(self):
        with mock.patch.object(m.subprocess,'run',return_value=done('rows')) as run:
            m.host_command(['docker','top','owned'],timeout=7)
        self.assertEqual(run.call_args.kwargs['timeout'],7)

    def test_timeout_does_not_allow_next_step_until_helper_exit(self):
        states=[subprocess.TimeoutExpired(['exec'],120),
                done(json.dumps([{'State':{'Running':True}}])),
                done('15 python hardware_lifecycle_20261020.py initialize-enable --receipt /logs/a.json'),
                done(json.dumps([{'State':{'Running':True}}])),done('PID ARGS')]
        with mock.patch.object(m.subprocess,'run',side_effect=states) as run, \
             mock.patch.object(m.time,'sleep') as sleep:
            uncertain=mock.Mock()
            result=m.lifecycle_call(['exec'],'owned-container','a.json',uncertain)
        self.assertNotEqual(result.returncode,0)
        uncertain.assert_called_once()
        sleep.assert_called_once_with(1)
        self.assertFalse(any('kill' in str(v) or 'stop' in str(v) for v in run.call_args_list))

    def test_unknown_daemon_state_does_not_count_as_helper_exit(self):
        states=[subprocess.TimeoutExpired(['exec'],120),done('daemon unavailable',1),
                done(json.dumps([{'State':{'Running':False}}]))]
        with mock.patch.object(m.subprocess,'run',side_effect=states),mock.patch.object(m.time,'sleep') as sleep:
            result=m.lifecycle_call(['exec'],'owned-container','a.json',lambda:None)
        self.assertEqual(result.returncode,1)
        sleep.assert_called_once()

    def test_successful_exec_is_returned_without_any_poll(self):
        expected=done('ok')
        with mock.patch.object(m.subprocess,'run',return_value=expected) as run:
            self.assertIs(m.lifecycle_call(['exec'],'owned','a.json',lambda:None),expected)
        self.assertEqual(run.call_count,1)


if __name__=='__main__':unittest.main()
