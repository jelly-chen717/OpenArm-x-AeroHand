# 项目中文注释（2026-10-29）：受控 CAN 通信恢复与状态查询。
# 调用与职责：验证物理 CAN 身份，调用校验过的旧 transport helper，查询 16 个电机并保存记录。
# 运行边界：会修改 CAN 链路并发送查询帧；不发送使能、运动或零位帧。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""Restore the approved CAN timing and query all motors with drivers stopped."""
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time

ROOT = Path('/home/robot/Teleop/VR_to_Openarm_original_4443_20261019')
OLD_TRANSPORT = Path('/home/robot/Teleop/VR_to_Openarm_test_20261018/ops/restore_transport.py')
TRANSPORT_SHA = '3b5b4e3e20328b068bed39e7f43ec93c0b7a6b4d95f298d97b5ee15ef89556aa'
CHANNELS = ('can0', 'can1')


# 注释：加载指定原控制器或校验过的辅助模块，具体对象由调用处固定。
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# 注释：执行该对象的既有工作循环；验证物理 CAN 身份，调用校验过的旧 transport helper，查询 16 个电机并保存记录。
def run(args):
    return subprocess.check_output(args,text=True,timeout=25)


# 注释：生成 UTC 时间字符串，用于回执与日志关联。
def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# 注释：运行既有链路辅助命令，并向变更子进程继承操作锁。
def transport_command(args,timeout=20):
    from operation_lock import child_fds
    mutation=args[:2]==['docker','run']
    options={'capture_output':True,'text':True,'timeout':None if mutation else timeout}
    if mutation:
        options['pass_fds']=child_fds()
    result=subprocess.run(args,**options)
    if result.returncode:
        raise RuntimeError(result.stderr)
    return result.stdout


# 注释：在链路修改或主动查询前确认没有另一个电机控制驱动。
def assert_no_drivers():
    arguments=run(['ps','-eo','args']).splitlines()
    assert not any('ros2_control_node' in line or 'motor_control.py' in line for line in arguments), 'Another CAN driver is running'


# 注释：只接受既有 CAN 配置或已知 USB 重插后的空链路状态。
def initial_profile(name,row):
    """Accept known timing for repair; a degraded bus is not enable-ready."""
    assert row.get('ifname')==name and row.get('linkinfo',{}).get('info_kind')=='can', 'Not the intended CAN link'
    info=row['linkinfo']['info_data']
    if 'UP' not in row.get('flags',[]):
        # Same empty-link profile as the already reviewed restore_can_replug.
        assert info.get('state')=='STOPPED' and info.get('ctrlmode',[])==[], 'Initial CAN is not DOWN/STOPPED with empty mode'
        assert info.get('bittiming') in (None,{}) and info.get('data_bittiming') in (None,{}), 'Initial CAN is already configured'
        assert row.get('mtu') in (16,72), 'Unknown CAN frame layout'
        return 'fresh'
    # A disconnected/reconnected arm can leave the adapter degraded.
    # Admit it only to transport repair; verification below stays strict.
    assert info.get('state') in {'ERROR-ACTIVE','ERROR-WARNING','ERROR-PASSIVE','BUS-OFF'}, 'Unsupported initial CAN state: '+name+' '+str(info.get('state'))
    assert set(info['ctrlmode']) in [{'FD'},{'FD','LISTEN-ONLY'}]
    assert info['bittiming']['bitrate']==1000000 and info['data_bittiming']['bitrate']==5000000
    assert info['bittiming']['sample_point']==.75 and info['data_bittiming']['sample_point']==.75
    assert info['bittiming']['sjw']==1 and info['data_bittiming']['sjw'] in [1,2]
    return 'configured'


def requires_repair(profile, info):
    """Only a healthy bus with the approved timing may skip link recovery."""
    return not (profile == 'configured' and info.get('state') == 'ERROR-ACTIVE'
                and set(info.get('ctrlmode', [])) == {'FD'}
                and info.get('data_bittiming', {}).get('sjw') == 2)


# 注释：再次核对物理设备身份与原时序，防止恢复过程中换错接口。
def same_links(transport,before,identities,profiles):
    """Recheck identity and configuration without comparing traffic counters."""
    assert {name:transport.identity(name) for name in CHANNELS}==identities, 'CAN device identity changed'
    current=transport.rows()
    assert set(current)==set(CHANNELS), 'CAN link pair changed'
    for name in CHANNELS:
        assert initial_profile(name,current[name])==profiles[name], 'CAN profile changed'
        old=before[name]['linkinfo']['info_data'];now=current[name]['linkinfo']['info_data']
        for key in ('ctrlmode','bittiming','data_bittiming','restart_ms'):
            assert now.get(key)==old.get(key), 'CAN configuration changed before repair'


# 注释：核对恢复后的 CAN 速率、采样点、SJW 和模式与批准配置一致。
def verified_configured(name,row,old,profile):
    info=row['linkinfo']['info_data']
    assert 'UP' in row['flags'] and info['state']=='ERROR-ACTIVE' and set(info['ctrlmode'])=={'FD'}
    if profile=='configured':
        # Preserve the original UP-link comparison, including all timing fields.
        assert info['bittiming']==old['bittiming']
        assert info['data_bittiming']==dict(old['data_bittiming'],sjw=2)
    else:
        assert row.get('ifname')==name and row['linkinfo'].get('info_kind')=='can' and row.get('mtu')==72
        for key,bitrate,sjw in [('bittiming',1000000,1),('data_bittiming',5000000,2)]:
            timing=info.get(key,{})
            assert timing.get('bitrate')==bitrate and timing.get('sample_point')==.75 and timing.get('sjw')==sjw, 'CAN timing differs'
        assert info.get('berr_counter') in (None,{'tx':0,'rx':0}), 'CAN bus errors observed'
    assert info.get('restart_ms')==old.get('restart_ms'), 'CAN restart policy changed'


# 注释：仅对本次触及的链路按原身份恢复配置，记录失败。
def rollback_links(transport,before,identities,profiles,changed):
    errors=[]
    for name in changed:
        try:
            assert_no_drivers()
            assert transport.identity(name)==identities[name], 'CAN device identity changed'
            transport.ip(['link','set','dev',name,'down'])
            assert transport.identity(name)==identities[name], 'CAN device identity changed'
            if profiles[name]=='configured':
                old=before[name]['linkinfo']['info_data']
                transport.config(name,'LISTEN-ONLY' in old['ctrlmode'],old['data_bittiming']['sjw'])
                assert transport.identity(name)==identities[name], 'CAN device identity changed'
                transport.ip(['link','set','dev',name,'up'])
                assert transport.identity(name)==identities[name], 'CAN device identity changed'
                row=transport.rows()[name];info=row['linkinfo']['info_data']
                assert 'UP' in row['flags'] and info['state']=='ERROR-ACTIVE'
                assert info['bittiming']==old['bittiming'] and info['data_bittiming']==old['data_bittiming']
                assert set(info['ctrlmode'])==set(old['ctrlmode'])
            else:
                # An empty pre-replug timing did not exist: leave our touched
                # device DOWN rather than invent a rollback configuration.
                assert 'UP' not in transport.rows()[name]['flags']
        except BaseException as error:
            errors.append(name+': '+repr(error))
    return errors


# 注释：恢复所属待机服务并等待只读状态确认。
def restart_standby(manage,settings,receipt):
    current=manage.original_container(settings)
    if not current['State']['Running']:
        assert_no_drivers()
        manage.require_no_other_managers(current)
    receipt['standby_restart']=manage.start()
    deadline=time.monotonic()+45
    while True:
        try:
            receipt['standby_after']=manage.standby_status()
            return
        except (OSError,ValueError,RuntimeError,KeyError):
            if time.monotonic()>=deadline:
                raise
            time.sleep(.2)


# 注释：发送有限批次状态查询并解析 16 个电机反馈；不发送运动或使能帧。
def query():
    assert_no_drivers()
    result={'time_utc':utc(),'query_only':True,'enable_frames_sent':0,'motion_frames_sent':0,
            'zero_frames_sent':0,'tx':[]}
    sockets={}
    values={}
    try:
        for channel in ['can0','can1']:
            sock=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FD_FRAMES,1)
            sock.bind((channel,))
            sockets[sock]=channel
        for batch in range(3):
            assert_no_drivers()
            for sock,channel in sockets.items():
                for motor_id in range(1,9):
                    payload=bytes([motor_id,0,0xcc,0,0,0,0,0])
                    sock.send(struct.pack('=IBB2x64s',0x7ff,8,1,payload))
                    result['tx'].append({'channel':channel,'id':0x7ff,'data':payload.hex()})
                    time.sleep(.002)
            deadline=time.monotonic()+.16
            while time.monotonic()<deadline:
                ready,_,_=select.select(list(sockets),[],[],max(0,deadline-time.monotonic()))
                for sock in ready:
                    frame=sock.recv(72)
                    can_id=struct.unpack_from('=I',frame)[0]
                    data=frame[8:8+frame[4]]
                    motor_id=can_id-0x10
                    if not can_id&0xe0000000 and 1<=motor_id<=8 and len(data)>=8 and data[0]&15==motor_id:
                        values[sockets[sock],motor_id]={'channel':sockets[sock],'id':motor_id,
                            'state':data[0]>>4,'position_rad':int.from_bytes(data[1:3],'big')/65535*25-12.5,
                            'received':time.monotonic()}
        now=time.monotonic()
        result['motors']=[dict(v,age_ms=(now-v['received'])*1000) for v in values.values()]
        for motor in result['motors']:
            motor.pop('received')
        fresh=[m for m in result['motors'] if m['age_ms']<800]
        result.update(online=len(fresh),disabled=sum(m['state']==0 for m in fresh),
                      enabled=sum(m['state']==1 for m in fresh),faulted=sum(m['state'] not in [0,1] for m in fresh))
        result['state']='disabled' if result['disabled']==16 else 'unknown'
        return result
    finally:
        for sock in sockets:
            sock.close()


# 注释：在操作锁内核对物理接口与辅助脚本 SHA，恢复批准的链路配置，完成有限状态查询后返回待机。
def main():
    assert Path(__file__).resolve().parent.parent==ROOT
    report=ROOT/'reports'/('can_prepare_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'.json')
    receipt={'started_utc':utc(),'phase':'precheck','motor_enable_authorized':False,
             'motion_test_authorized':False,'enable_frames_sent':0,'motion_frames_sent':0}
    before=None
    changed=[]
    transport=None
    manage=settings=None
    stop_attempted=False
    identities=profiles=None
    try:
        manage=load('original_manager',ROOT/'ops/manage.py')
        settings=manage.runtime()
        manage.backup_complete(settings)
        manage.check_owned_dropin(settings)
        assert manage.DROPIN.is_file()
        assert str(ROOT/'ops/manage.py')+' start' in run(['systemctl','--user','show','openarm-teleop-standby.service','-p','ExecStart'])
        assert not manage.customized_container()['State']['Running']
        manage.require_tests_stopped()
        current=manage.original_container(settings)
        manage.require_no_other_managers(current)
        if current['State']['Running']:
            receipt['standby_before']=manage.standby_status()
        assert not OLD_TRANSPORT.is_symlink() and OLD_TRANSPORT.is_file(), 'Unexpected transport helper path'
        receipt['transport_helper_sha256']=hashlib.sha256(OLD_TRANSPORT.read_bytes()).hexdigest()
        assert receipt['transport_helper_sha256']==TRANSPORT_SHA, 'Transport helper changed'
        transport=load('validated_transport',OLD_TRANSPORT)
        # Preserve its fixed CAN configuration, but do not time out a Docker
        # client while its network helper could still be changing the link.
        transport.run=transport_command
        before=transport.rows()
        identities={name:transport.identity(name) for name in CHANNELS}
        assert set(before)=={'can0','can1'} and len({i[1] for i in identities.values()})==1
        profiles={name:initial_profile(name,before[name]) for name in CHANNELS}
        assert len(set(profiles.values()))==1, 'Mixed CAN profiles require separate inspection'
        receipt.update(before=before,identities_before=identities,profiles=profiles)
        same_links(transport,before,identities,profiles)
        # Loading, checksum, physical identity and both profiles must all pass
        # before a precheck failure may interrupt the existing HW1 service.
        stop_attempted=True
        receipt['stopped']=manage.stop()
        assert not manage.original_container(settings)['State']['Running'], 'Owned CAN driver did not stop'
        assert_no_drivers()
        same_links(transport,before,identities,profiles)
        for name in CHANNELS:
            assert_no_drivers()
            assert transport.identity(name)==identities[name]
            old=before[name]['linkinfo']['info_data']
            current_info=transport.rows()[name]['linkinfo']['info_data']
            if not requires_repair(profiles[name],current_info):
                continue
            changed.append(name)  # Include partially completed mutations in rollback.
            transport.ip(['link','set','dev',name,'down'])
            assert transport.identity(name)==identities[name]
            transport.config(name,False,2)
            assert transport.identity(name)==identities[name]
            transport.ip(['link','set','dev',name,'up'])
            assert transport.identity(name)==identities[name]
            verified_configured(name,transport.rows()[name],old,profiles[name])
        receipt.update(changed=changed,after=transport.rows(),phase='transport_restored')
        feedback=query()
        receipt['feedback']=feedback
        # Save factual query results even if an individual motor is missing.
        (ROOT/'reports/can_feedback_preparation.json').write_text(json.dumps(feedback,indent=2)+'\n')
        assert feedback['online']==16 and feedback['disabled']==16 and feedback['enabled']==0 and feedback['faulted']==0, 'Motor feedback did not confirm all 16 disabled and fault-free'
        # A successful reply alone must not hide a bus that degraded again.
        receipt['after_query']=transport.rows()
        assert {name:transport.identity(name) for name in CHANNELS}==identities, 'CAN device identity changed after query'
        for name in CHANNELS:
            verified_configured(name,receipt['after_query'][name],before[name]['linkinfo']['info_data'],profiles[name])
        receipt['feedback_verified_utc']=utc()
        # The oneshot standby unit remains active while its Docker container is
        # stopped for the finite query. Restart the owned container directly;
        # a systemd reentry would not inherit the host's exclusive lease.
        restart_standby(manage,settings,receipt)
        receipt['phase']='feedback_verified_standby_restarted'
        receipt['finished_utc']=utc()
        report.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'complete':True,'report':str(report),'phase':receipt['phase'],
                          'online':feedback['online'],'disabled':feedback['disabled'],'enabled':feedback['enabled'],
                          'motor_enable_commands_sent':0,'motion_commands_sent':0}))
    except BaseException as error:
        if stop_attempted:
            try:
                if not manage.original_container(settings)['State']['Running']:
                    receipt['rollback_errors']=rollback_links(transport,before,identities,profiles,changed)
                else:
                    receipt['rollback_skipped']='Owned service is running; no CAN mutation is permitted'
            except BaseException as rollback:
                receipt['rollback_errors']=[repr(rollback)]
            try:
                restart_standby(manage,settings,receipt)
            except BaseException as restore:
                receipt['restore_error']=repr(restore)
        receipt.update(phase='failed',changed=changed,error=repr(error),finished_utc=utc())
        report.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'complete':False,'report':str(report),'error':repr(error)}))
        raise


if __name__=='__main__':
    from operation_lock import exclusive
    with exclusive(ROOT):
        main()
