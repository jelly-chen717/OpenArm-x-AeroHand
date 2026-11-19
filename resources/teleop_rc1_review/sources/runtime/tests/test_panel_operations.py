"""Executor contract tests; no real network, subprocess, ROS or CAN."""
import copy
from contextlib import nullcontext
import json
import hashlib
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest import mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ops'))
import panel_operations as m


def quiet():
    return {'mode':'original_standby','state':'unknown','online':0,'enabled':0,'disabled':0,
            'faulted':0,'hardware_states':{'left':1,'right':1},'controllers':{},
            'hardware_age_ms':5,'controllers_age_ms':5,'motors':[],'output_enabled':None}


def query():
    return {'time_utc':'2026-10-20T09:00:00+00:00','query_only':True,'enable_frames_sent':0,
            'motion_frames_sent':0,'online':16,'disabled':16,'enabled':0,'faulted':0,
            'motors':[{'channel':c,'id':i,'state':0,'age_ms':5,'position_rad':0.01}
                      for c in ('can0','can1') for i in range(1,9)]}


def enabled():
    s=quiet()
    s.update(state='enabled',online=16,enabled=16,hardware_states={'left':3,'right':3},
             controllers={str(i):'active' for i in range(5)},
             motors=[dict(v,state=1) for v in query()['motors']],output_enabled=True)
    return s


def healthy_hold():
    state=enabled()
    state['hardware_states']={name:3 for name in ('openarm_left_hardware_interface','openarm_right_hardware_interface')}
    state['controllers']={name:'active' for name in ('joint_state_broadcaster','left_forward_position_controller',
                         'right_forward_position_controller','left_gripper_controller','right_gripper_controller')}
    return state


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        for name in ('reports','state','logs'):(self.root/name).mkdir()
        self.e=m.OperationExecutor(self.root)
        self.state=quiet()
        self.e.raw_status=lambda:copy.deepcopy(self.state)
        self.e.external=lambda:[]
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(m,'boot_id',return_value='current-boot').start()
        mock.patch.object(m,'exclusive',side_effect=lambda root:nullcontext(10)).start()
        self.calls=[]

        def run(argv,event,cancel_path=None):
            self.calls.append(argv)
            if 'disable' in argv:self.state=quiet()
            elif 'initialize-enable' in argv:self.state=enabled()
            else:
                (self.root/'reports/can_feedback_preparation.json').write_text(json.dumps(query()))
            return 'mocked.log'
        self.e._run=run

    def prepare(self):
        self.assertTrue(self.e.run('prepare',threading.Event())['success'])

    def motion(self,phase='following',**changes):
        run='a'*32
        active=phase=='following'
        record={'kind':'right_vr_motion_bridge_v1','mode':'right_motion_test',
                'boot_id':'current-boot','run_id':run,'phase':phase,'closed':phase=='closed',
                'active':active,'bridge_enabled':active,'heartbeat_monotonic_s':100.,
                'published':4,'mapping_mode':'strict_absolute','reason':None,
                'state_path':str((self.root/'logs/right_motion_bridge.json').absolute()),
                'cancel_file':str((self.root/'state'/('right_motion_cancel_'+run+'.request')).absolute())}
        record.update(changes)
        (self.root/'logs/right_motion_bridge.json').write_text(json.dumps(record),encoding='utf-8')
        return record

    def chain(self,worker_enabled=False,bridge_enabled=False,following=False,scope='right_arm_test',**worker_changes):
        config=dict(boot_id='current-boot',run_id='b'*32,runtime_id='c'*64,scope=scope)
        core=self.root/'app/custom_chain/control_core.py';core.parent.mkdir(parents=True,exist_ok=True)
        original=Path(__file__).resolve().parents[2]/'vr_motion_channel_20261021/custom_chain/control_core.py'
        core.write_bytes(original.read_bytes())
        config['files_sha256']={'control_core.py':hashlib.sha256(core.read_bytes()).hexdigest()}
        (self.root/'state/custom_chain_runtime.json').write_text('{}')
        self.e.chain_configuration=lambda:config
        identity=dict(config,heartbeat_monotonic_s=100.)
        worker=dict(identity,kind='vr_custom_ik_state_v1',instance_id='d'*32,enabled=worker_enabled,
                    rearm_required=True,warmup_ready=True,xr_valid=True,xr_age_ms=1,deadman_active=False,
                    buttons={side:[False]*6 for side in ('left','right')})
        worker.update(worker_changes)
        bridge=dict(identity,kind='vr_custom_bridge_state_v1',instance_id='e'*32,enabled=bridge_enabled,
                    following=following,scope=scope,home_active=False,pending_count=0,
                    worker_runtime_id=config['runtime_id'],worker_instance_id=worker['instance_id'],targets_published=12)
        if scope=='bimanual_test':
            bridge['capabilities']={'arm_sides':['left','right'],'gripper_sides':['left','right'],'home':False}
        for which,row in (('worker',worker),('bridge',bridge)):
            (self.root/'logs'/('custom_chain_'+which+'.json')).write_text(json.dumps(row))
        return worker,bridge

    def test_custom_chain_state_is_separate_from_finite_bridge_and_requires_real_fresh_identity(self):
        self.motion('following')
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.chain()
            state=self.e.status()
            self.assertEqual(state['custom_chain']['phase'],'closed')
            self.assertFalse(state['bridge_enabled'])
            self.assertEqual(state['bridge_policy'],'custom_chain_right_arm')
            self.chain(True,True)
            state=self.e.status()
            self.assertEqual(state['custom_chain']['phase'],'armed')
            self.assertTrue(state['bridge_enabled'])
            self.assertFalse(state['custom_chain']['following'])
            self.assertEqual(state['custom_chain']['published'],12)
        with mock.patch.object(m.time,'monotonic',return_value=100.6):
            state=self.e.status()
            self.assertIsNone(state['bridge_enabled'])
            self.assertEqual(state['custom_chain']['phase'],'stale')

    def test_custom_chain_unknown_never_falls_back_to_finite_following_or_old_enable(self):
        self.chain()
        self.motion()
        (self.root/'logs/custom_chain_worker.json').unlink()
        state=self.e.status()
        self.assertIsNone(state['bridge_enabled'])
        self.assertFalse(state['custom_chain']['verified'])
        self.assertFalse(state['can_enable'])

    def test_custom_enable_calls_coordinator_instead_of_hardware_only_and_off_closes_gates_first(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.chain();self.prepare();self.calls.clear()
            original=self.e._run
            def run(argv,event,cancel_path=None):
                if str(self.root/'ops/custom_chainctl.py') in argv:
                    self.calls.append(argv)
                    if 'enable' in argv:
                        self.state=enabled();self.chain(True,True)
                    elif 'disable' in argv:
                        self.state=quiet();self.chain()
                    return 'mocked-chain.log'
                return original(argv,event,cancel_path)
            self.e._run=run
            result=self.e.run('enable',threading.Event())
            self.assertTrue(result['success'],result)
            self.assertIn(str(self.root/'ops/custom_chainctl.py'),self.calls[0])
            self.assertIn('enable',self.calls[0])
            self.assertNotIn('--hardware-only',self.calls[0])
            self.assertTrue(result['bridge_enabled'])
            self.calls.clear()
            def lease(root):
                self.assertIn('close-gates',self.calls[0])
                return nullcontext(10)
            with mock.patch.object(m,'exclusive',side_effect=lease):
                result=self.e.run('disable',threading.Event())
            self.assertTrue(result['success'],result)
            self.assertIn('disable',self.calls[1])

    def test_bad_chain_configuration_does_not_suppress_existing_hardware_off(self):
        self.chain()
        self.e.chain_configuration=mock.Mock(side_effect=RuntimeError('bad chain configuration'))
        result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertTrue(any('disable' in row and '--hardware-only' in row for row in self.calls))

    def test_bimanual_status_validates_both_actual_scopes_without_requiring_hw1_joint_feedback(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.chain(scope='bimanual_test');self.prepare()
            state=self.e.status()
            self.assertEqual(state['custom_chain']['scope'],'bimanual_test')
            self.assertEqual(state['custom_chain']['phase'],'closed')
            self.assertEqual(state['bridge_policy'],'custom_chain_bimanual')
            self.assertTrue(state['can_enable'])
            worker=json.loads((self.root/'logs/custom_chain_worker.json').read_text())
            worker['scope']='right_arm_test'
            (self.root/'logs/custom_chain_worker.json').write_text(json.dumps(worker))
            self.assertFalse(self.e.status()['custom_chain']['verified'])
            self.assertFalse(self.e.status()['can_enable'])

    def test_missing_motion_heartbeat_is_unknown_and_never_a_closed_claim(self):
        state=self.e.status()
        self.assertIsNone(state['bridge_enabled'])
        self.assertIsNone(state['vr_motion']['active'])
        self.assertFalse(state['vr_motion']['verified'])
        self.prepare()
        self.assertTrue(self.e.status()['can_enable'])

    def test_fresh_motion_states_report_only_actual_following_as_active(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.25):
            for phase in m.VR_MOTION_PHASES:
                self.motion(phase)
                state=self.e.status()
                self.assertTrue(state['vr_motion']['verified'],phase)
                self.assertEqual(state['bridge_enabled'],phase=='following')
                self.assertEqual(state['vr_motion']['state'],phase)
                self.assertEqual(state['vr_motion']['published'],4)

    def test_stale_future_and_prior_boot_motion_never_appear_active(self):
        for now in (100.501,99.9):
            self.motion()
            with mock.patch.object(m.time,'monotonic',return_value=now):
                state=self.e.status()
            self.assertEqual(state['vr_motion']['state'],'stale')
            self.assertIsNone(state['bridge_enabled'])
            self.assertTrue(state['vr_motion']['cancel_available'])
            self.assertTrue(state['can_disable'])
        self.motion(boot_id='old-boot')
        state=self.e.status()
        self.assertFalse(state['vr_motion']['cancel_available'])
        self.assertIsNone(state['bridge_enabled'])

    def test_invalid_mode_run_paths_and_safety_flags_do_not_manufacture_motion_status(self):
        cases=[{'kind':'other'},{'mode':'unbounded'},{'run_id':'../outside'},
               {'cancel_file':'/tmp/right_motion_cancel_'+('a'*32)+'.request'},
               {'cancel_file':'/state/right_motion_cancel_'+('b'*32)+'.request'},
               {'state_path':'/tmp/right_motion_bridge.json'}, {'active':'true'},
               {'bridge_enabled':False},{'closed':True},{'phase':'moving'},
               {'mapping_mode':'unknown'},{'published':True},{'published':-1},
               {'heartbeat_monotonic_s':float('nan')},{'fault_reason':'unexpected_fault'}]
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            for fields in cases:
                self.motion(**fields)
                state=self.e.status()
                self.assertIsNone(state['bridge_enabled'],fields)
                self.assertFalse(state['vr_motion']['verified'],fields)

    def test_readonly_status_never_writes_a_cancel_or_dispatches_hardware(self):
        self.motion()
        self.e._run=mock.Mock(side_effect=AssertionError('read-only status dispatched'))
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.assertTrue(self.e.status()['bridge_enabled'])
        self.assertEqual(list((self.root/'state').glob('right_motion_cancel*')),[])
        self.e._run.assert_not_called()

    def test_off_maps_container_cancel_path_to_fixed_host_state_before_lease(self):
        self.motion(cancel_file='/state/right_motion_cancel_'+('a'*32)+'.request',
                    state_path='/logs/right_motion_bridge.json')
        path=self.root/'state'/('right_motion_cancel_'+('a'*32)+'.request')
        def lease(root):
            self.assertTrue(path.is_file(),'cancel must precede lease acquisition')
            request=json.loads(path.read_text())
            self.assertEqual(request['run_id'],'a'*32)
            self.assertEqual(request['boot_id'],'current-boot')
            return nullcontext(10)
        with mock.patch.object(m,'exclusive',side_effect=lease):
            result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertTrue(result['vr_motion_cancel']['requested'])
        self.assertIn('disable',self.calls[0])

    def test_stale_heartbeat_does_not_refuse_cancel_or_old_hardware_off(self):
        self.motion(heartbeat_monotonic_s=1.)
        result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertTrue(result['vr_motion_cancel']['requested'])
        self.assertIn('disable',self.calls[0])

    def test_off_rejects_arbitrary_cancel_or_run_mismatch_but_still_attempts_hardware_close(self):
        outside=self.root/'outside.request'
        for fields in ({'cancel_file':str(outside)},
                       {'cancel_file':'/state/right_motion_cancel_'+('b'*32)+'.request'},
                       {'boot_id':'old-boot'}):
            self.motion(**fields);self.calls.clear()
            result=self.e.run('disable',threading.Event())
            self.assertTrue(result['success'],result)
            self.assertFalse(result['vr_motion_cancel']['requested'])
            self.assertFalse(outside.exists())
            self.assertEqual(list((self.root/'state').glob('right_motion_cancel*')),[])
            self.assertIn('disable',self.calls[0])

    def test_enable_and_prepare_never_create_motion_cancel_or_arm_requests(self):
        self.motion('closed')
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.prepare()
            self.assertTrue(self.e.run('enable',threading.Event())['success'])
        self.assertEqual(list((self.root/'state').glob('right_motion*')),[])

    def test_duplicate_and_oversized_state_file_are_rejected(self):
        path=self.root/'logs/right_motion_bridge.json'
        for raw in ('{"run_id":"one","run_id":"two"}', ' '*65537):
            path.write_text(raw)
            self.assertFalse(self.e.status()['vr_motion']['verified'])

    def test_cancel_is_idempotent_and_never_follows_existing_symlink(self):
        self.motion()
        self.assertTrue(self.e.cancel_vr_motion()['requested'])
        self.assertTrue(self.e.cancel_vr_motion()['already_requested'])
        path=self.root/'state'/('right_motion_cancel_'+('a'*32)+'.request')
        path.unlink()
        outside=self.root/'untouched.txt';outside.write_text('untouched')
        try:
            path.symlink_to(outside)
        except OSError:
            return
        self.assertFalse(self.e.cancel_vr_motion()['requested'])
        self.assertEqual(outside.read_text(),'untouched')

    def test_off_retries_only_cooperative_bridge_lease_and_does_not_fake_shutdown(self):
        self.motion()
        leases=[RuntimeError('Another hardware operation holds the project lease'),nullcontext(10)]
        with mock.patch.object(m,'exclusive',side_effect=leases),mock.patch.object(m.time,'sleep'):
            result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertTrue(result['vr_motion_cancel']['requested'])
        self.motion()
        with mock.patch.object(m,'exclusive',side_effect=RuntimeError('foreign identity refused')):
            result=self.e.run('disable',threading.Event())
        self.assertFalse(result['success'])
        self.assertIn('foreign identity',result['error'])

    def test_unexpected_cancel_observer_failure_still_attempts_existing_hardware_off(self):
        self.e.cancel_vr_motion=mock.Mock(side_effect=RuntimeError('observer failed'))
        result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertFalse(result['vr_motion_cancel']['requested'])
        self.assertIn('observer failed',result['vr_motion_cancel']['error'])
        self.assertIn('disable',self.calls[0])

    def test_query_requires_exact_sixteen_finite_disabled_motors(self):
        self.assertTrue(m.valid_query(query()))
        for key,value in [('id',9),('position_rad',float('nan')),('state',1),('age_ms',900)]:
            q=query();q['motors'][0][key]=value
            self.assertFalse(m.valid_query(q),key)

    def test_hw2_and_stale_lifecycle_are_not_prepared_standby(self):
        s=quiet();s['hardware_states']['left']=2
        self.assertFalse(m.fresh_quiet(s))
        s=quiet();s['hardware_age_ms']=6000
        self.assertFalse(m.fresh_quiet(s))

    def test_prepare_only_queries_and_does_not_enable(self):
        self.prepare()
        self.assertEqual(len(self.calls),1)
        self.assertTrue(self.calls[0][-1].endswith('prepare_can_20261020.py'))
        s=self.e.status()
        self.assertTrue(s['initialized'])
        self.assertTrue(s['can_enable'])
        self.assertTrue(s['enable_readiness_verified'])
        self.assertFalse(s['feedback_fresh'])
        self.assertEqual(s['state'],'unknown')
        self.assertEqual(s['online'],0)

    def test_old_boot_or_changed_query_cannot_unlock_enable(self):
        self.prepare()
        with mock.patch.object(m,'boot_id',return_value='different-boot'):
            self.assertFalse(self.e.status()['can_enable'])
        (self.root/'reports/can_feedback_preparation.json').write_text('{}')
        self.assertFalse(self.e.status()['can_enable'])

    def test_enable_without_preparation_never_dispatches(self):
        self.assertFalse(self.e.run('enable',threading.Event())['success'])
        self.assertEqual(self.calls,[])

    def test_enable_uses_fresh_query_lifecycle_mode_without_web_wait(self):
        self.prepare();self.calls.clear()
        out=self.e.run('enable',threading.Event())
        self.assertTrue(out['success'])
        self.assertEqual(len(self.calls),1)
        self.assertIn('initialize-enable',self.calls[0])
        self.assertIn('--hardware-only',self.calls[0])
        self.assertIn('--cancel-file',self.calls[0])
        self.assertFalse(out['bridge_enabled'])

    def test_stop_closes_lifecycle_then_verifies_disabled_with_finite_query(self):
        self.state=enabled()
        out=self.e.run('disable',threading.Event())
        self.assertTrue(out['success'])
        self.assertIn('disable',self.calls[0])
        self.assertTrue(self.calls[1][-1].endswith('prepare_can_20261020.py'))
        self.assertNotIn('initialize-enable',str(self.calls))
        self.assertEqual(out['state']['state'],'unknown')
        self.assertTrue(self.e.status()['can_enable'])

    def test_failed_query_does_not_manufacture_disabled_success(self):
        self.e._run=mock.Mock(side_effect=RuntimeError('query failed'))
        out=self.e.run('prepare',threading.Event())
        self.assertFalse(out['success'])
        self.assertFalse(self.e.status()['initialized'])

    def test_cancelled_enable_does_not_start_lifecycle(self):
        self.prepare();self.calls.clear()
        stop=threading.Event();stop.set()
        out=self.e.run('enable',stop)
        self.assertFalse(out['success'])
        self.assertTrue(out['cancelled'])
        self.assertEqual(self.calls,[])

    def test_get_status_never_dispatches_and_freshness_is_factual(self):
        self.e._run=mock.Mock(side_effect=AssertionError('read must be inert'))
        self.state=enabled()
        self.assertTrue(self.e.status()['feedback_fresh'])
        self.state['motors'][0]['age_ms']=500
        self.assertFalse(self.e.status()['feedback_fresh'])
        self.e._run.assert_not_called()

    def test_missing_monitor_allows_explicit_stop_but_never_enable_or_prepare(self):
        self.prepare();self.calls.clear()
        self.e.raw_status=mock.Mock(side_effect=OSError('observer unavailable'))
        state=self.e.status()
        self.assertFalse(state['telemetry_available'])
        self.assertTrue(state['can_disable'])
        self.assertFalse(state['can_enable'])
        self.assertFalse(state['can_initialize'])
        self.assertEqual(state['state'],'unknown')
        self.assertIsNone(state['output_enabled'])
        self.assertIsNone(state['known_motor_enabled'])
        self.assertFalse(state['motor_state_verified'])
        for action in ('prepare','enable'):
            self.assertFalse(self.e.run(action,threading.Event())['success'])
        self.assertEqual(self.calls,[])

    def test_closed_healthy_hold_can_reopen_without_forcing_hw1_or_losing_off(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.chain(scope='bimanual_test');self.prepare();self.state=healthy_hold()
            self.e._save_ready('enabled')
            state=self.e.status()
            self.assertTrue(state['can_enable'])
            self.assertTrue(state['can_disable'])
            self.assertFalse(state['can_initialize'])
            self.calls.clear()
            def run(argv,event,cancel_path=None):
                self.calls.append(argv)
                self.chain(True,True,scope='bimanual_test')
                return 'mocked-reopen.log'
            self.e._run=run
            result=self.e.run('enable',threading.Event())
            self.assertTrue(result['success'],result)
            self.assertIn(str(self.root/'ops/custom_chainctl.py'),self.calls[0])
            self.assertIn('enable',self.calls[0])
            self.assertTrue(result['bridge_enabled'])

    def test_hold_reopen_requires_full_actual_motor_feedback_and_closed_fresh_chain(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.chain(scope='bimanual_test');self.prepare();self.state=healthy_hold()
            self.assertTrue(self.e.status()['can_enable'])
            self.state['motors'][0]['age_ms']=251
            self.assertFalse(self.e.status()['can_enable'])
            self.state=healthy_hold();self.state['motors'][0]['id']=2
            self.assertFalse(self.e.status()['can_enable'])
            self.state=healthy_hold();self.chain(False,True,scope='bimanual_test')
            self.assertFalse(self.e.status()['can_enable'])
        with mock.patch.object(m.time,'monotonic',return_value=100.6):
            self.assertFalse(self.e.status()['can_enable'])

    def test_actual_home_capability_pending_and_active_are_distinct_from_teleop(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            for mode in ('pending','homing'):
                worker,bridge=self.chain(False,True,scope='bimanual_test')
                bridge.update(home_output_enabled=True,home_pending=mode=='pending',home_active=mode=='homing',
                              home_side='left',home_mode=mode,following_window_limit_s=60,
                              last_home_result={'reason':'completed','side':'right','published_count':300,'ignored':{'nested':'not displayed'}},
                              last_enabled_close={'reason':'following_window_expired','worker':{'nested':'not displayed'}})
                bridge['capabilities']['home']=True
                (self.root/'logs/custom_chain_bridge.json').write_text(json.dumps(bridge))
                chain=self.e.status()['custom_chain']
                self.assertTrue(chain['verified'],chain)
                self.assertTrue(chain['home_supported'])
                self.assertTrue(chain['enabled'])
                self.assertFalse(chain['worker_enabled'])
                self.assertFalse(chain['following'])
                self.assertEqual(chain['phase'],'home_pending' if mode=='pending' else 'homing')
                self.assertEqual(chain['last_home_result'],{'reason':'completed','side':'right','published_count':300})
                self.assertEqual(chain['last_enabled_close'],{'reason':'following_window_expired'})
                self.assertEqual(chain['following_window_limit_s'],60)
                self.assertFalse(self.e.status()['can_enable'])

    def test_home_support_is_never_inferred_from_a_flag_or_stale_snapshot(self):
        for changes in ({'home_active':True},{'home_pending':True,'home_mode':'pending','home_side':'left'},
                        {'capabilities':{'arm_sides':['left','right'],'gripper_sides':['left','right'],'home':True}}):
            with mock.patch.object(m.time,'monotonic',return_value=100.1):
                _,bridge=self.chain(False,True,scope='bimanual_test')
                bridge.update(changes)
                (self.root/'logs/custom_chain_bridge.json').write_text(json.dumps(bridge))
                self.assertFalse(self.e.status()['custom_chain']['verified'])
        with mock.patch.object(m.time,'monotonic',return_value=100.6):
            self.assertIsNone(self.e.status()['custom_chain']['home_supported'])

    def test_paused_and_resuming_keep_original_following_deadline_and_never_offer_enable(self):
        with mock.patch.object(m.time,'monotonic',return_value=100.1):
            self.state=healthy_hold()
            for paused,awaiting,following,phase in ((True,False,False,'paused'),
                    (True,True,False,'paused'),(False,True,False,'resuming'),(False,False,True,'following')):
                _,bridge=self.chain(True,True,following,scope='bimanual_test')
                bridge.update(paused=paused,awaiting_fresh_target=awaiting,
                              following_at_monotonic_s=80.,following_remaining_s=40.,following_window_limit_s=60.)
                (self.root/'logs/custom_chain_bridge.json').write_text(json.dumps(bridge))
                state=self.e.status();chain=state['custom_chain']
                self.assertTrue(chain['verified'],chain)
                self.assertEqual(chain['phase'],phase)
                self.assertEqual(chain['following_at_monotonic_s'],80.)
                self.assertAlmostEqual(chain['following_remaining_s'],39.9)
                self.assertTrue(chain['enabled'])
                self.assertEqual(chain['following'],following)
                self.assertFalse(state['can_enable'])
                self.assertTrue(state['can_disable'])
                self.assertEqual(self.calls,[])

    def test_unknown_or_inconsistent_pause_cannot_manufacture_a_valid_continuation(self):
        for changes in ({'paused':'true'},{'awaiting_fresh_target':1},
                        {'paused':True,'following':True},{'paused':True,'enabled':False},
                        {'paused':True,'following_at_monotonic_s':90.,'following_remaining_s':None},
                        {'paused':True,'following_at_monotonic_s':80.,'following_remaining_s':61.,'following_window_limit_s':60.}):
            with mock.patch.object(m.time,'monotonic',return_value=100.1):
                _,bridge=self.chain(True,True,scope='bimanual_test')
                bridge.update(changes)
                (self.root/'logs/custom_chain_bridge.json').write_text(json.dumps(bridge))
                state=self.e.status()
                self.assertFalse(state['custom_chain']['verified'])
                self.assertFalse(state['can_enable'])
        with mock.patch.object(m.time,'monotonic',return_value=100.6):
            self.assertIsNone(self.e.status()['custom_chain']['paused'])

    def test_monitor_recovery_after_shutdown_can_finish_stop_and_query_verification(self):
        available=False
        observations=[]
        original_run=self.e._run

        def observe():
            observations.append(available)
            if not available:
                raise OSError('monitor absent until shutdown')
            return copy.deepcopy(self.state)

        def run(argv,event,cancel_path=None):
            nonlocal available
            if 'disable' in argv:
                self.assertEqual(observations,[])
                available=True
            return original_run(argv,event,cancel_path)

        self.e.raw_status=observe
        self.e._run=run
        result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertIn('disable',self.calls[0])
        self.assertTrue(self.calls[1][-1].endswith('prepare_can_20261020.py'))
        self.assertEqual(result['state']['state'],'unknown')
        self.assertEqual(result['state']['online'],0)
        self.assertNotIn('initialize-enable',str(self.calls))

    def test_monitor_remaining_unknown_does_not_manufacture_verified_stop(self):
        self.e.raw_status=mock.Mock(side_effect=OSError('monitor still absent'))
        calls=[]

        def run(argv,event,cancel_path=None):
            calls.append(argv)
            if 'disable' not in argv:
                raise RuntimeError('finite query could not verify driver shutdown')
            return 'shutdown-attempt.log'

        self.e._run=run
        result=self.e.run('disable',threading.Event())
        self.assertFalse(result['success'])
        self.assertEqual(len(calls),2)
        self.assertIn('disable',calls[0])
        self.assertFalse(self.e.prepared.exists())
        state=self.e.status()
        self.assertEqual(state['state'],'unknown')
        self.assertFalse(state['feedback_fresh'])
        self.assertIsNone(state['output_enabled'])

    def test_inactive_controller_residue_keeps_explicit_cleanup_available(self):
        self.state['controllers']={'right_gripper_controller':'inactive'}
        state=self.e.status()
        self.assertTrue(state['can_disable'])
        self.assertFalse(state['can_initialize'])
        self.assertFalse(state['can_enable'])
        result=self.e.run('disable',threading.Event())
        self.assertTrue(result['success'],result)
        self.assertIn('disable',self.calls[0])
        self.assertEqual(result['state']['controllers'],{})

    def test_unknown_monitor_does_not_bypass_shutdown_identity_or_ownership_refusal(self):
        self.e.raw_status=mock.Mock(side_effect=OSError('observer unavailable'))
        self.e._run=mock.Mock(side_effect=RuntimeError('foreign controller ownership refused'))
        result=self.e.run('disable',threading.Event())
        self.assertFalse(result['success'])
        self.assertEqual(self.e._run.call_count,1)
        self.assertIn('disable',self.e._run.call_args.args[0])
        self.assertIn('ownership',result['error'])
        self.assertFalse(self.e.prepared.exists())

    def test_stale_lifecycle_and_malformed_monitor_still_offer_only_stop(self):
        self.state['hardware_age_ms']=6000
        state=self.e.status()
        self.assertTrue(state['can_disable'])
        self.assertFalse(state['can_enable'])
        self.assertFalse(state['can_initialize'])
        self.e.raw_status=lambda:None
        state=self.e.status()
        self.assertTrue(state['can_disable'])
        self.assertFalse(state['telemetry_available'])
        self.assertIsNone(state['output_enabled'])
        self.e.raw_status=lambda:{'hardware_states':None,'controllers':None,'motors':None}
        state=self.e.status()
        self.assertTrue(state['can_disable'])
        self.assertFalse(state['telemetry_available'])
        self.assertFalse(state['can_initialize'])
        self.assertFalse(state['can_enable'])


if __name__=='__main__':
    unittest.main()
