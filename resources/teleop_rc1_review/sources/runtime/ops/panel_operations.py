# 项目中文注释（2026-10-29）：网页操作执行器。
# 调用与职责：协调 prepare/enable/disable、CAN 准备、硬件生命周期与 IK/原桥许可。
# 运行边界：持有跨进程操作锁执行；enable 包含实际使能及后续许可操作。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Fixed, serialized host operations behind the test motor panel."""
import datetime
import hashlib
import json
import os
import math
from pathlib import Path
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid

from operation_lock import exclusive, child_fds

SOURCE_SHA256='1e5d17c0dcaecd2ce46e1005d878d750978f9930180b1c348719f739b2612ccb'


# 注释：只把真正的连接错误判为不可达，HTTP 错误不当作服务已停止。
def status_unreachable(error):
    # An HTTP response or malformed JSON is not proof that the observer stopped.
    return isinstance(error,OSError) and not isinstance(error,urllib.error.HTTPError)


# 注释：读取本次系统启动标识，使准备记录不能跨重启误用。
def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


# 注释：判断唯一驱动是否处于新鲜的未配置待机状态。
def fresh_quiet(s):
    return (s.get('mode')=='original_standby' and s.get('controllers')=={}
            and set(s.get('hardware_states',{}).values())=={1}
            and len(s.get('hardware_states',{}))==2
            and isinstance(s.get('hardware_age_ms'),(int,float)) and 0<=s['hardware_age_ms']<5000
            and isinstance(s.get('controllers_age_ms'),(int,float)) and 0<=s['controllers_age_ms']<5000)


# 注释：核对本次查询确实只读、16 个电机均关闭且反馈有效。
def valid_query(q):
    motors=q.get('motors',[])
    return (q.get('query_only') is True and q.get('enable_frames_sent')==0
            and q.get('motion_frames_sent')==0 and q.get('online')==16
            and q.get('disabled')==16 and q.get('enabled')==0 and q.get('faulted')==0
            and len(motors)==16 and {(m.get('channel'),m.get('id')) for m in motors}
                =={(channel,i) for channel in ('can0','can1') for i in range(1,9)}
            and all(m.get('state')==0 and isinstance(m.get('age_ms'),(int,float))
                    and 0<=m['age_ms']<800 and isinstance(m.get('position_rad'),(int,float))
                    and math.isfinite(m['position_rad']) for m in motors))


# 注释：本模块的 OperationExecutor 定义；用途见文件说明与下方原实现。
class OperationExecutor:
    # 注释：保存项目根、当前启动准备记录和子进程集合，初始化操作占用状态。
    def __init__(self,root):
        self.root=Path(root)
        self.prepared=self.root/'state/panel_prepared.json'
        self.running=False
        self.children=set()
        self._recovery_probe=None

    # 注释：从本机只读观察器读取原始生命周期与反馈快照。
    def raw_status(self):
        with urllib.request.urlopen('http://127.0.0.1:8767/status',timeout=2) as response:
            return json.load(response)

    # 注释：扫描已知硬件命令的在途进程，识别不属于本执行器的并发操作。
    def external(self):
        names={str(self.root/'ops'/n) for n in ('hardwarectl.py','prepare_can_20261020.py','manage.py','archive_output.py')}
        rows=[]
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                argv=[v.decode() for v in path.read_bytes().split(b'\0') if v]
                if not any(v in names for v in argv) or 'status' in argv:
                    continue
                pid=int(path.parent.name)
                ancestor=pid
                while ancestor>1 and ancestor not in self.children:
                    fields=(Path('/proc')/str(ancestor)/'stat').read_text().rsplit(')',1)[1].split()
                    ancestor=int(fields[1])
                if ancestor in self.children:
                    continue
                rows.append({'pid':pid,'argv':argv})
            except (OSError,ValueError,UnicodeError,IndexError):
                continue
        return rows

    # 注释：核对本次开机的准备记录与原始查询文件哈希。
    def readiness(self,s):
        try:
            record=json.loads(self.prepared.read_text())
            query=self.root/'reports/can_feedback_preparation.json'
            q=json.loads(query.read_text())
            return (record.get('boot_id')==boot_id() and record.get('stage') in ('ready','enabled')
                    and record.get('query_sha256')==hashlib.sha256(query.read_bytes()).hexdigest()
                    and valid_query(q)),record
        except (OSError,ValueError,KeyError):
            return False,{}

    # 注释：只读核对已停止运行时的身份与单驱动条件，不自动恢复服务。
    def _stopped_source_runtime(self):
        """Read-only ownership proof; callers that act must obtain it afresh."""
        import manage
        if manage.ROOT.resolve()!=self.root.resolve():
            raise RuntimeError('Runtime manager belongs to another project')
        profile_path=self.root/'state/source_control.json'
        if profile_path.is_symlink() or profile_path.resolve()!=profile_path:
            raise RuntimeError('Source profile path differs')
        profile=json.loads(profile_path.read_text())
        if (not isinstance(profile,dict) or profile.get('kind')!='archive_source_control_v1'
                or profile.get('boot_id')!=boot_id() or profile.get('source_sha256')!=SOURCE_SHA256
                or (self.root/'state/custom_chain_runtime.json').exists()):
            raise RuntimeError('Current archive source profile is not verified')
        settings=manage.runtime()
        manage.backup_complete(settings)
        manage.check_owned_dropin(settings)
        if not manage.DROPIN.is_file():
            raise RuntimeError('Owned standby service drop-in is missing')
        container=manage.original_container(settings)
        if container['State']['Running'] is not False:
            raise RuntimeError('Owned runtime is not stopped')
        if manage.customized_container()['State']['Running'] is not False:
            raise RuntimeError('Preserved customized runtime is running')
        manage.require_tests_stopped()
        manage.require_no_other_managers(container)
        return manage,settings

    # 注释：短暂缓存可恢复性提示；真正操作时仍需重新校验。
    def _can_recover_stopped(self):
        # Polling never starts a service. Only this short-lived UI hint is cached;
        # run('prepare') repeats every proof while holding the existing lease.
        now=time.monotonic()
        if self._recovery_probe is None or not 0<=now-self._recovery_probe[0]<2:
            try:
                self._stopped_source_runtime()
                allowed=True
            except Exception:
                allowed=False
            self._recovery_probe=(time.monotonic(),allowed)
        return self._recovery_probe[1]

    # 注释：只读合并反馈、队列占用和生命周期，计算原网页按钮可用性；不因为轮询而使能。
    def status(self):
        available=True
        unreachable=False
        try:
            s=self.raw_status()
            if not isinstance(s,dict):
                raise ValueError('Invalid status response')
            if (not isinstance(s.get('hardware_states',{}),dict)
                    or not isinstance(s.get('controllers',{}),dict)
                    or not isinstance(s.get('motors',[]),list)):
                raise ValueError('Invalid lifecycle or feedback response')
        except (OSError,ValueError) as error:
            available=False
            unreachable=status_unreachable(error)
            s={'state':'unknown','online':0,'enabled':0,'disabled':0,'faulted':0,
               'motors':[],'hardware_states':{},'controllers':{},
               'output_enabled':None,'known_motor_enabled':None,'motor_state_verified':False,
               'error':'状态服务暂不可达：'+str(error)}
        motors=s.get('motors',[])
        fresh=(len(motors)==16 and all(isinstance(m.get('age_ms'),(int,float))
               and 0<=m['age_ms']<=250 and m.get('state') in (0,1) for m in motors))
        ext=self.external()
        recover=(unreachable and not ext and not self.running and self._can_recover_stopped())
        if recover:s['error']='控制服务已停止，请点击恢复并初始化'
        ready,record=self.readiness(s)
        quiet=fresh_quiet(s)
        lifecycle_known=(available and len(s.get('hardware_states',{}))==2
                         and all(v in (1,2,3) for v in s['hardware_states'].values())
                         and isinstance(s.get('hardware_age_ms'),(int,float))
                         and 0<=s['hardware_age_ms']<5000
                         and isinstance(s.get('controllers_age_ms'),(int,float))
                         and 0<=s['controllers_age_ms']<5000)
        can_enable=bool(quiet and not ext and not self.running and not s.get('faulted') and not s.get('known_motor_enabled'))
        can_park = False
        if not ext and not self.running:
            try:
                from archive_output import hardware_hold
                hardware_hold(s)
                can_park = True
            except Exception:
                pass
        s.update(telemetry_available=available,feedback_fresh=fresh,initialized=ready,enable_readiness_verified=can_enable,
                 initialization_report={'time':record.get('time'),'stage':record.get('stage')},
                 can_initialize=bool(recover or quiet and not ext and not self.running
                                     and not s.get('known_motor_enabled') and not s.get('faulted')),
                 can_enable=can_enable,
                 can_park=can_park,
                 # A stop request may enter the existing identity/ownership and
                 # ROS shutdown checks even when the HTTP observer is missing.
                 # Loaded inactive controllers also require explicit cleanup.
                 can_disable=bool(not recover and (not lifecycle_known or self.running or ext or s.get('controllers')
                                  or s.get('output_enabled') or s.get('known_motor_enabled')
                                  or any(v in (2,3) for v in s.get('hardware_states',{}).values()))),
                 busy_external=bool(ext),external_operations=ext,
                 bridge_enabled=None,bridge_policy='archive_source_control',
                 control_mode='archive_source_control_v1',
                 control_source='VR_to_Openarm.zip original bridge; saved custom frontend and IK')
        return s

    # 注释：在有效查询基础上保存当前启动周期的准备状态。
    def _save_ready(self,stage):
        path=self.root/'reports/can_feedback_preparation.json'
        query=json.loads(path.read_text())
        if not valid_query(query):
            raise RuntimeError('Finite query did not verify 16 disabled motors')
        record={'boot_id':boot_id(),'stage':stage,'time':query['time_utc'],
                'query_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        self.prepared.write_text(json.dumps(record)+'\n')

    # 注释：限时等待新鲜的未配置待机状态。
    def _wait_quiet(self):
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            try:
                if fresh_quiet(self.raw_status()):
                    return
            except (OSError,ValueError):
                pass
            time.sleep(.2)
        raise RuntimeError('查询后未确认唯一驱动的未配置待机状态')

    # 注释：运行既有硬件命令并保留日志与操作锁；取消通过文件传递。
    def _run(self,argv,cancel_event,cancel_path=None):
        log=self.root/'logs'/('panel_operation_'+uuid.uuid4().hex+'.log')
        with log.open('wb') as output:
            child=subprocess.Popen(argv,stdout=output,stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL,pass_fds=child_fds())
            self.children.add(child.pid)
            try:
                # Never kill a docker-exec client on a timer: the ROS helper may
                # still be live. Its operation retains the inherited lease.
                while child.poll() is None:
                    if cancel_path and cancel_event.is_set():
                        cancel_path.touch(exist_ok=True)
                    time.sleep(.1)
                if child.returncode:
                    if str(self.root/'ops/archive_output.py') in argv and 'park' in argv:
                        try:
                            failed=json.loads(log.read_text(encoding='utf-8'))
                        except (OSError,ValueError):
                            failed={}
                        raise RuntimeError('收臂未完成，未执行自动关闭使能：'+str(failed.get('error') or log))
                    raise RuntimeError('硬件操作失败，回执：'+str(log))
            finally:
                self.children.discard(child.pid)
        return str(log)

    # 注释：调用原 IK/桥许可事务，复用当前队列的取消机制。
    def _source_output(self,action,event,cancel_path=None):
        command=['python3','-B',str(self.root/'ops/archive_output.py'),action]
        if cancel_path is not None:command.extend(['--cancel-file',str(cancel_path)])
        return self._run(command,event,cancel_path)

    # 注释：串行协调 prepare/enable/disable；使能后才开放既有 IK/原桥许可，失败时执行已有关闭回退。
    def run(self,action,cancel_event):
        if action not in ('prepare','enable','disable','park'):
            raise ValueError('Unsupported action')
        hardware_activated=False
        close_error=None
        standby_recovery=None
        try:
            # Existing CLI operations are never interrupted by an OS signal.
            while self.external():
                if action!='disable':
                    raise RuntimeError('Another hardware operation is active')
                for row in self.external():
                    if str(self.root/'ops/hardwarectl.py') in row['argv']:
                        args=row['argv']
                        file=(Path(args[args.index('--cancel-file')+1]) if '--cancel-file' in args else
                              self.root/'state'/('hardware_cancel_'+boot_id()[:8]+'_'+str(row['pid'])+'.request'))
                        if file.resolve().parent==(self.root/'state').resolve():
                            file.touch(exist_ok=True)
                time.sleep(.2)
            with exclusive(self.root):
                # The lifecycle helper validates runtime identity, the unique
                # manager, and controller ownership itself. Observer failure
                # must not prevent an explicit attempt to close that hardware.
                # Preparation/enable retain the strict live-status prerequisite.
                try:
                    state=None if action=='disable' else self.raw_status()
                except OSError as error:
                    if action!='prepare' or not status_unreachable(error):raise
                    if cancel_event.is_set():
                        return {'success':False,'cancelled':True,'error':'初始化已取消'}
                    # A missing observer alone grants nothing. Revalidate the
                    # exact stopped source runtime under this transaction lock.
                    manager,settings=self._stopped_source_runtime()
                    self.running=True
                    try:
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'初始化已取消'}
                        standby_recovery=manager.start()
                        if (not isinstance(standby_recovery,dict) or standby_recovery.get('action')!='start'
                                or standby_recovery.get('automatic_enable') is not False
                                or standby_recovery.get('container_id')!=settings['container_id']):
                            raise RuntimeError('Standby restart did not confirm the exact non-enabling runtime')
                        self._recovery_probe=None
                        self._wait_quiet()
                        state=self.raw_status()
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'初始化已取消',
                                    'standby_recovery':standby_recovery}
                    finally:self.running=False
                if action=='enable' and state.get('enabled')==16 and state.get('faulted')==0:
                    if not self.status()['feedback_fresh']:
                        raise RuntimeError('Enabled feedback is stale')
                    if cancel_event.is_set():
                        return {'success':False,'cancelled':True,'error':'使能已取消'}
                    hardware_activated=True
                    cancel_path=self.root/'state'/('panel_cancel_'+uuid.uuid4().hex+'.request')
                    self.running=True
                    try:self._source_output('on',cancel_event,cancel_path)
                    finally:self.running=False
                    return {'success':True,'already_enabled':True,'bridge_enabled':True}
                if action in ('prepare','enable') and not fresh_quiet(state):
                    raise RuntimeError('初始化/使能前双臂必须未配置且无活动控制器')
                if action=='enable' and not self.status()['can_enable']:
                    raise RuntimeError('请先完成本次开机后的初始化检查')
                if action == 'park' and not self.status().get('can_park'):
                    raise RuntimeError('反馈或使能状态不完整，不能自动收臂；必要时使用立即关闭')
                self.running=True
                try:
                    if action in ('prepare','disable','park'):
                        self.prepared.unlink(missing_ok=True)
                    park_receipt = None
                    if action == 'park':
                        cancel_path=self.root/'state'/('panel_cancel_'+uuid.uuid4().hex+'.request')
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'收臂已取消'}
                        log=self._source_output('park',cancel_event,cancel_path)
                        park_receipt=json.loads(Path(log).read_text())
                        if (park_receipt.get('action')!='park' or park_receipt.get('success') is not True
                                or park_receipt.get('parked') is not True
                                or park_receipt.get('feedback_arrival_verified') is not True
                                or park_receipt.get('final_stage')!='park_lower_j2'):
                            raise RuntimeError('未确认垂下到位，未关闭使能')
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'收臂事务已取消，交由立即关闭处理'}
                        from archive_output import hardware_hold
                        hardware_hold(self.raw_status())
                        # 只有本次收臂的到位证明成立，才调用原硬件 OFF。
                        self._run(['python3','-B',str(self.root/'ops/hardwarectl.py'),'disable','--hardware-only'],threading.Event())
                        self._wait_quiet()
                    if action=='disable':
                        try:self._source_output('off',threading.Event())
                        except Exception as error:close_error=str(error)
                        self._run(['python3','-B',str(self.root/'ops/hardwarectl.py'),'disable','--hardware-only'],threading.Event())
                        self._wait_quiet()
                    if action=='prepare':
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'初始化已取消'}
                        self._run(['python3','-B',str(self.root/'ops/prepare_can_20261020.py')],cancel_event)
                        self._wait_quiet()
                        self._save_ready('ready')
                    elif action=='enable':
                        cancel_path=self.root/'state'/('panel_cancel_'+uuid.uuid4().hex+'.request')
                        if cancel_event.is_set():
                            return {'success':False,'cancelled':True,'error':'使能已取消'}
                        self._run(['python3','-B',str(self.root/'ops/hardwarectl.py'),'initialize-enable',
                                   '--hardware-only','--cancel-file',str(cancel_path)],cancel_event,cancel_path)
                        hardware_activated=True
                        result=self.raw_status()
                        if (result.get('online')!=16 or result.get('enabled')!=16 or result.get('faulted')!=0
                                or not self.status()['feedback_fresh']
                                or set(result.get('hardware_states',{}).values())!={3}
                                or len(result.get('controllers',{}))!=5
                                or set(result['controllers'].values())!={'active'}):
                            raise RuntimeError('使能回执后没有16电机正常反馈')
                        if cancel_event.is_set():cancel_path.touch(exist_ok=True)
                        self._source_output('on',cancel_event,cancel_path)
                        self._save_ready('enabled')
                    return {'success':True,'action':action,'bridge_enabled':(True if action=='enable' else
                            False if action in ('disable','park') and close_error is None else None),
                            'park_receipt':park_receipt,
                            'output_close_error':close_error,
                            'standby_recovery':standby_recovery,
                            'cancelled':cancel_event.is_set(),'state':self.raw_status()}
                finally:
                    self.running=False
        except Exception as error:
            rollback={}
            if hardware_activated:
                try:self._source_output('off',threading.Event())
                except Exception as stop_error:rollback['output_error']=str(stop_error)
                # A worker starting up must not undo a manual motor-enable
                # request. Keep verified healthy hold after closing IK/output;
                # hardware/feedback errors still use the standard OFF path.
                try:
                    held=self.status()
                    healthy=(held.get('online')==16 and held.get('enabled')==16
                             and held.get('faulted')==0 and held.get('feedback_fresh') is True
                             and len(held.get('hardware_states',{}))==2
                             and set(held['hardware_states'].values())=={3}
                             and len(held.get('controllers',{}))==5
                             and set(held['controllers'].values())=={'active'})
                except Exception:healthy=False
                if healthy and 'output_error' not in rollback:rollback['hardware_hold_retained']=True
                else:
                    try:self._run(['python3','-B',str(self.root/'ops/hardwarectl.py'),'disable','--hardware-only'],threading.Event())
                    except Exception as stop_error:rollback['hardware_error']=str(stop_error)
            return {'success':False,'action':action,'error':str(error),'bridge_enabled':None,'rollback':rollback}
