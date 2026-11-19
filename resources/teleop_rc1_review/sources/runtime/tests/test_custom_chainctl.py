"""Host coordination tests; all subprocess/ROS/hardware effects are fake."""
from contextlib import nullcontext
from copy import deepcopy
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import tempfile
import sys
import subprocess
import unittest
from unittest import mock

sys.path.insert(0,str(Path(__file__).parents[1]/'ops'))

spec=importlib.util.spec_from_file_location('chainctl',Path(__file__).parents[1]/'ops/custom_chainctl.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
CONFIG=dict(boot_id='boot',run_id='a'*32,runtime_id='b'*64,scope='right_arm_test')
core_spec=importlib.util.spec_from_file_location('host_test_runtime_checks',Path(__file__).resolve().parents[2]/'vr_motion_channel_20261021/custom_chain/control_core.py')
checks=importlib.util.module_from_spec(core_spec);core_spec.loader.exec_module(checks)


def standby():
    return dict(mode='original_standby',hardware_states={name:1 for name in checks.HARDWARE},
                controllers={},hardware_age_ms=5.,controllers_age_ms=5.)


def held():
    return dict(standby(),online=16,enabled=16,faulted=0,
                hardware_states={name:3 for name in checks.HARDWARE},
                controllers={name:'active' for name in checks.CONTROLLERS},
                motors=[dict(channel=channel,id=i,state=1,age_ms=5.) for channel in ('can0','can1') for i in range(1,9)])


class IO:
    def __init__(self,root):self.root=root;self.calls=[];self.generation=0;self.begin_hook=lambda:None;self.state=standby();self.status_hook=lambda:None
    def guard(self,config):self.calls.append('guard')
    def hardware(self,action,cancel=None):self.calls.append(('hardware',action));return {'success':True}
    def hardware_status(self):self.calls.append('hardware_status');self.status_hook();return deepcopy(self.state)
    def hardware_plan(self,config):return m.hardware_plan(self.hardware_status(),checks)
    def gates(self,action,config,cancel=None,**kwargs):self.calls.append(('gates',action));return {'success':True}
    def stop_generation(self):return self.generation
    def cancel(self,config):self.calls.append('cancel');self.generation+=1
    def begin(self,config,cancel_file):
        self.calls.append('begin');self.begin_hook()
        return self.root/'state/custom_chain_cancel_eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee.request'


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);(self.root/'state').mkdir()
        self.io=IO(self.root)
        self.lease=mock.patch.object(m,'exclusive',side_effect=lambda root:nullcontext())
        self.lease.start();self.addCleanup(self.lease.stop)

    def test_enable_verifies_closed_before_hardware_then_opens_and_rechecks_runtime(self):
        result=m.execute('enable',self.io,CONFIG)
        self.assertTrue(result['success'])
        self.assertEqual(self.io.calls,['guard','guard','begin',('gates','close'),'hardware_status',('hardware','initialize-enable'),'guard',('gates','open')])
        self.assertEqual(result['hardware_plan'],'initialize')

    def test_fresh_exact_hold_reuses_hardware_but_keeps_full_real_gate_sequence(self):
        self.io.state=held()
        result=m.execute('enable',self.io,CONFIG)
        self.assertTrue(result['success'],result)
        self.assertEqual(result['hardware_plan'],'reuse_hold')
        self.assertEqual(self.io.calls,['guard','guard','begin',('gates','close'),'hardware_status','guard',('gates','open')])
        self.assertNotIn(('hardware','initialize-enable'),self.io.calls)

    def test_bad_hold_or_mixed_hardware_never_opens_or_initializes_and_rolls_back(self):
        states=[]
        for change in ({'online':15},{'faulted':1},{'controllers':{}},
                       {'hardware_age_ms':5000},{'controllers_age_ms':float('nan')},
                       {'hardware_states':{name:2 for name in checks.HARDWARE}}):
            states.append(dict(held(),**change))
        state=held();state['motors'][0]['age_ms']=250.01;states.append(state)
        state=held();state['motors'][0]['id']=2;states.append(state)
        states.extend([{},dict(standby(),hardware_age_ms=5000),
                       dict(standby(),hardware_states={'other_left':1,'other_right':1})])
        for state in states:
            with self.subTest(state=state):
                self.io.calls=[];self.io.state=state
                result=m.execute('enable',self.io,CONFIG)
                self.assertFalse(result['success'])
                self.assertNotIn(('gates','open'),self.io.calls)
                self.assertNotIn(('hardware','initialize-enable'),self.io.calls)
                self.assertEqual(self.io.calls[-2:],[('gates','close'),('hardware','disable')])

    def test_concurrent_off_during_hold_status_cannot_open_a_gate(self):
        self.io.state=held();self.io.status_hook=lambda:setattr(self.io,'generation',1)
        result=m.execute('enable',self.io,CONFIG)
        self.assertFalse(result['success'])
        self.assertNotIn(('gates','open'),self.io.calls)
        self.assertNotIn(('hardware','initialize-enable'),self.io.calls)

    def test_status_reader_is_bounded_read_only_and_injectable(self):
        response=mock.MagicMock()
        response.__enter__.return_value=__import__('io').StringIO(json.dumps(held()))
        with mock.patch.object(m.urllib.request,'urlopen',return_value=response) as read:
            self.assertEqual(m.HostControl(self.root).hardware_status(),held())
        read.assert_called_once_with('http://127.0.0.1:8767/status',timeout=3)

    def test_runtime_checks_are_exact_manifest_source_and_not_a_second_ros_client(self):
        base=self.root/'app/custom_chain';base.mkdir(parents=True)
        path=base/'control_core.py';path.write_text('HARDWARE = set()\ndef hardware_hold(state):\n    pass\n')
        config={'files_sha256':{'control_core.py':hashlib.sha256(path.read_bytes()).hexdigest()}}
        self.assertTrue(callable(m.runtime_checks(self.root,config).hardware_hold))
        path.write_text('changed')
        with self.assertRaisesRegex(RuntimeError,'configured SHA'):m.runtime_checks(self.root,config)

    def test_disable_requests_cancel_and_both_false_before_waiting_for_lease(self):
        def lease(root):
            self.assertEqual(self.io.calls,['guard','cancel',('gates','close')])
            return nullcontext()
        with mock.patch.object(m,'exclusive',side_effect=lease):result=m.execute('disable',self.io,CONFIG)
        self.assertTrue(result['success'])
        self.assertEqual(self.io.calls[-2:],[('gates','close'),('hardware','disable')])

    def test_gate_open_failure_attempts_both_gate_and_hardware_rollback(self):
        gate=self.io.gates
        def fail(action,config,cancel=None,**kwargs):
            if action=='open':return {'success':False,'error':'real service rejected'}
            return gate(action,config,cancel)
        self.io.gates=fail;result=m.execute('enable',self.io,CONFIG)
        self.assertFalse(result['success'])
        self.assertEqual(self.io.calls[-2:],[('gates','close'),('hardware','disable')])

    def test_close_unknown_never_prevents_hardware_disable(self):
        self.io.cancel=mock.Mock(side_effect=RuntimeError('unknown operation record'))
        self.io.gates=mock.Mock(side_effect=RuntimeError('worker not reachable'))
        result=m.execute('disable',self.io,CONFIG)
        self.assertFalse(result['success'])
        self.assertIn(('hardware','disable'),self.io.calls)

    def test_concurrent_off_before_request_registration_prevents_hardware_on(self):
        self.io.begin_hook=lambda:setattr(self.io,'generation',1)
        result=m.execute('enable',self.io,CONFIG)
        self.assertFalse(result['success'])
        self.assertNotIn(('hardware','initialize-enable'),self.io.calls)
        self.assertNotIn(('gates','open'),self.io.calls)

    def test_off_after_registration_uses_unique_cancel_and_never_deletes_it(self):
        path=self.root/'state/custom_chain_cancel_eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee.request'
        gate=self.io.gates
        def close_then_cancel(action,config,cancel=None):
            result=gate(action,config,cancel)
            if action=='close':path.touch()
            return result
        self.io.gates=close_then_cancel
        self.assertFalse(m.execute('enable',self.io,CONFIG)['success'])
        self.assertTrue(path.exists())
        self.assertNotIn(('hardware','initialize-enable'),self.io.calls)

    def test_actual_host_cancel_mapping_is_fixed_and_request_files_are_never_reused(self):
        host=m.HostControl(self.root)
        path=host.begin(CONFIG)
        self.assertFalse(path.exists())
        host.cancel(CONFIG)
        self.assertTrue(path.is_file())
        with self.assertRaisesRegex(RuntimeError,'unique'):
            host.begin(CONFIG,path)
        self.assertTrue(path.exists())
        self.assertIsNotNone(host.stop_generation())

    def test_status_does_not_take_lease_or_mutate_a_gate(self):
        with mock.patch.object(m,'exclusive',side_effect=AssertionError('status must not lock hardware')):
            self.assertTrue(m.execute('status',self.io,CONFIG)['success'])
        self.assertEqual(self.io.calls,['guard',('gates','status')])

    def test_runtime_fingerprint_checks_actual_files_and_rejects_extra_code(self):
        base=self.root/'app/custom_chain';base.mkdir(parents=True)
        path=base/'control_core.py';path.write_text('example')
        files={'control_core.py':hashlib.sha256(path.read_bytes()).hexdigest()}
        config={'files_sha256':files,'runtime_id':hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()}
        m.verify_files(self.root,config)
        path.write_text('changed')
        with self.assertRaisesRegex(RuntimeError,'SHA differs'):m.verify_files(self.root,config)
        path.write_text('example');(base/'unregistered.py').write_text('extra')
        with self.assertRaisesRegex(RuntimeError,'unmanifested'):m.verify_files(self.root,config)

    def test_off_during_hardware_hold_prevents_later_true_sequence(self):
        def hardware(action,cancel=None):
            self.io.calls.append(('hardware',action))
            if action=='initialize-enable':self.io.generation+=1
            return {'success':True}
        self.io.hardware=hardware
        result=m.execute('enable',self.io,CONFIG)
        self.assertFalse(result['success'])
        self.assertNotIn(('gates','open'),self.io.calls)
        self.assertIn(('hardware','disable'),self.io.calls)

    def test_ros_wrapper_prepends_app_without_overwriting_sourced_python_path(self):
        result=subprocess.CompletedProcess([],0,'CUSTOM_CHAIN_RECEIPT={"success":true}\n','')
        with mock.patch.object(m.subprocess,'run',return_value=result) as run:
            self.assertTrue(m.HostControl(self.root).gates('status',dict(CONFIG,container_id='f'*64))['success'])
        command=run.call_args.args[0]
        shell=command[command.index('-c')+1]
        self.assertIn('source /opt/ros/humble/setup.bash',shell)
        self.assertIn('source /test/ros2_ws/install/local_setup.bash',shell)
        self.assertIn('export PYTHONPATH="/test/app${PYTHONPATH:+:$PYTHONPATH}"',shell)
        self.assertNotIn('export PYTHONPATH=/test/app\n',shell)

    @unittest.skipUnless(os.name=='posix','Bash environment evaluation runs on the target Linux host')
    def test_actual_bash_wrapper_retains_both_ros_setup_paths(self):
        result=subprocess.CompletedProcess([],0,'CUSTOM_CHAIN_RECEIPT={"success":true}\n','')
        with mock.patch.object(m.subprocess,'run',return_value=result) as run:
            m.HostControl(self.root).gates('status',dict(CONFIG,container_id='f'*64))
        command=run.call_args.args[0];shell=command[command.index('-c')+1]
        # Substitute only the two setup fixtures and final diagnostic command;
        # execute the exact wrapper export in Bash, without ROS or Docker.
        shell=shell.replace('source /opt/ros/humble/setup.bash',
                            'export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages')
        shell=shell.replace('source /test/ros2_ws/install/local_setup.bash',
                            'export PYTHONPATH="/test/ros2_ws/install/lib/python3.10/site-packages:$PYTHONPATH"')
        shell=shell.replace('exec /usr/bin/python3 -B /test/app/custom_chain/control_core.py "$@"',
                            'printf "%s" "$PYTHONPATH"')
        actual=subprocess.run(['/bin/bash','-c',shell],env={'PATH':'/usr/bin:/bin'},capture_output=True,text=True,check=True)
        self.assertEqual(actual.stdout,'/test/app:/test/ros2_ws/install/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages')

    def test_scope_comes_from_validated_configuration_and_is_passed_to_the_ros_helper(self):
        config=dict(CONFIG,schema_version=1,kind='vr_custom_chain_runtime_v1',enabled=True,container_id='f'*64,
                    original_inputs_isolated=True,python_path='/test/app/custom_chain/python',support_path='/test/app/custom_chain/support')
        path=self.root/'state/custom_chain_runtime.json'
        for scope in ('right_arm_test','bimanual_test'):
            config['scope']=scope;path.write_text(json.dumps(config))
            with mock.patch.object(Path,'read_text',return_value='boot'):
                self.assertEqual(m.configuration(self.root)['scope'],scope)
            result=subprocess.CompletedProcess([],0,'CUSTOM_CHAIN_RECEIPT={"success":true}\n','')
            with mock.patch.object(m.subprocess,'run',return_value=result) as run:
                m.HostControl(self.root).gates('status',config)
            args=run.call_args.args[0]
            self.assertEqual(args[args.index('--scope')+1],scope)
        config['scope']='home_all';path.write_text(json.dumps(config))
        with self.assertRaisesRegex(RuntimeError,'contract'):
            m.configuration(self.root)


if __name__=='__main__':unittest.main()
