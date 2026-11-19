"""Portable deployment identity and lifecycle; original motion logic is untouched."""
import argparse,hashlib,json,os,re,subprocess,time,urllib.request,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
DROPIN=Path.home()/'.config/systemd/user/openarm-teleop-standby.service.d/90-original-control-20261019.conf'
STATUS_URL='http://127.0.0.1:8767/status'
HARDWARE={'openarm_right_hardware_interface','openarm_left_hardware_interface'}
MANAGER=re.compile(r'(?:^|\s)(?:\S*/)?ros2_control_node(?:\s|$)')

def require(value,message):
    if not value:raise RuntimeError(message)

def run(args,timeout=15):
    from operation_lock import child_fds
    return subprocess.check_output(args,text=True,timeout=timeout,pass_fds=child_fds())

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def runtime():
    d=json.loads((ROOT/'runtime.json').read_text())
    require(d.get('schema_version')==2 and d.get('distribution')=='RobotControl','Not a portable installation')
    require(re.fullmatch('[0-9a-f]{64}',d.get('container_id','')),'Invalid container identity')
    require(d.get('automatic_enable') is False,'Automatic enable is forbidden')
    return d

def original_container(settings):
    c=json.loads(run(['docker','inspect',settings['container_id']]))[0]
    require(c['Id']==settings['container_id'] and c['Image']==settings['image'],'Container identity differs')
    require(c['Config']['Labels'].get('robotcontrol.owner')==settings['install_id'],'Container ownership differs')
    require(c['HostConfig']['NetworkMode']=='host' and c['HostConfig']['RestartPolicy']['Name']=='no','Unexpected container lifecycle')
    require(any(m['Source']==str(ROOT) and m['Destination']=='/test' and not m['RW'] for m in c['Mounts']),'Runtime mount differs')
    return c

def backup_complete(settings):
    """Compatibility entry: verify installed release, not old-machine backups."""
    p=ROOT/'install_manifest.json'
    require(digest(p)==settings['install_manifest_sha256'],'Installation manifest differs')
    manifest=json.loads(p.read_text())
    require(manifest['install_id']==settings['install_id'],'Installation identity differs')
    for n,h in manifest['critical_files'].items():
        require(not (ROOT/n).is_symlink() and digest(ROOT/n)==h,'Installed file changed: '+n)
    return manifest

def check_owned_dropin(settings):
    require(DROPIN.is_file() and not DROPIN.is_symlink() and digest(DROPIN)==settings['owned_dropin_sha'],'Owned service changed')

def require_no_other_managers(container):
    rows=run(['ps','-eo','pid,args']).splitlines()
    pids={r.split()[0] for r in rows if MANAGER.search(r)}
    owned=set()
    if container['State']['Running']:
        owned={r.split()[0] for r in run(['docker','top',container['Id'],'-eo','pid,args']).splitlines() if MANAGER.search(r)}
    require(pids<=owned,'Another hardware controller is running')

def require_tests_stopped():
    # Fresh machines have no historical container IDs. Inspect actual processes.
    require_no_other_managers(original_container(runtime()))

def standby_status():
    op=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with op.open(STATUS_URL,timeout=3) as f:s=json.load(f)
    if s.get('mode')=='original_web_offline':
        require(s.get('web_only') is True and s.get('ros_control_started') is False
                and s.get('output_enabled') is False and s.get('hardware')=={}
                and s.get('controllers')=={} and s.get('driver_quiescent') is True,
                'Invalid Web-only standby')
        stamp=s.get('status_generated_monotonic_s')
        require(type(stamp) in (float,int) and 0<=time.monotonic()-stamp<5,'Stale Web-only status')
        c=original_container(runtime());require_no_other_managers(c)
        require(not any(MANAGER.search(row) for row in run(['docker','top',c['Id'],'-eo','pid,args']).splitlines()),'Unexpected hardware process')
        return s
    require(s.get('mode')=='original_standby' and s.get('output_enabled') in (False,None),'Motion output is not closed')
    h=s.get('hardware',{})
    require(set(h)==HARDWARE and all(v.get('state_id')==1 for v in h.values()),'Hardware must be unconfigured')
    require(type(s.get('hardware_age_ms')) in (float,int) and 0<=s['hardware_age_ms']<5000,'Stale hardware status')
    require(s.get('controllers')=={} and s.get('driver_quiescent') is True and s.get('known_motor_enabled') in (False,None),'Driver is not quiescent')
    return s

def start():
    d=runtime();backup_complete(d);check_owned_dropin(d)
    c=original_container(d);require_no_other_managers(c)
    if c['State']['Running']:return {'action':'start','container_id':c['Id'],'already_running':True,'automatic_enable':False}
    # Update boot identity only while the owned driver is stopped.
    profile=ROOT/'state/source_control.json';p=json.loads(profile.read_text())
    p.update(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),run_id=uuid.uuid4().hex)
    profile.write_text(json.dumps(p,indent=2))
    require(time.time()>1735689600,'System clock is not set')
    run(['docker','start',d['container_id']],timeout=40)
    require(original_container(d)['State']['Running'],'Controller failed to start')
    return {'action':'start','container_id':c['Id'],'started':True,'automatic_enable':False}

def stop():
    d=runtime();c=original_container(d)
    if c['State']['Running']:
        standby_status();run(['docker','stop','-t','12',c['Id']],timeout=30)
        require(not original_container(d)['State']['Running'],'Controller did not stop')
    return {'stopped':True}

def status():
    d=runtime();c=original_container(d)
    return {'container':c['Name'],'running':c['State']['Running'],'automatic_enable':False}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','stop','status']);a=parser.parse_args()
    from operation_lock import exclusive
    with exclusive(ROOT):print(json.dumps(globals()[a.action]()))
