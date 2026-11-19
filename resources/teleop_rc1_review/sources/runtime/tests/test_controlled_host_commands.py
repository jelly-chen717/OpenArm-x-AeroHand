"""A host deadline must not orphan a submitted transport mutation."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

OPS=Path(__file__).resolve().parents[1]/'ops'
sys.path.insert(0,str(OPS))


def load(name):
    spec=importlib.util.spec_from_file_location('controlled_'+name,OPS/(name+'.py'))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ControlledCommands(unittest.TestCase):
    def test_docker_start_stop_wait_for_ack_and_inherit_lease(self):
        manager=load('manage')
        with mock.patch.object(manager.subprocess,'check_output',return_value='ok') as call, \
             mock.patch('operation_lock.child_fds',return_value=(42,)):
            for action in ('start','stop'):
                manager.run(['docker',action,'owned'],timeout=1)
                self.assertIsNone(call.call_args.kwargs['timeout'])
                self.assertEqual(call.call_args.kwargs['pass_fds'],(42,))
            manager.run(['docker','inspect','owned'],timeout=7)
            self.assertEqual(call.call_args.kwargs['timeout'],7)
            self.assertNotIn('pass_fds',call.call_args.kwargs)

    def test_network_container_waits_for_ack_but_reads_keep_timeout(self):
        prepare=load('prepare_can_20261020')
        with mock.patch.object(prepare.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'ok','')) as call, \
             mock.patch('operation_lock.child_fds',return_value=(43,)):
            prepare.transport_command(['docker','run','fixed-network-helper'],timeout=1)
            self.assertIsNone(call.call_args.kwargs['timeout'])
            self.assertEqual(call.call_args.kwargs['pass_fds'],(43,))
            prepare.transport_command(['ip','-j','link'],timeout=9)
            self.assertEqual(call.call_args.kwargs['timeout'],9)
            self.assertNotIn('pass_fds',call.call_args.kwargs)


if __name__=='__main__':unittest.main()
