"""Read-only left/right J6 comparison and J2 context. No enable or MIT frames."""
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path
import json
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0,str(ROOT/'apps'))
from openarm_driver import OpenArmDriver

registers = {0:'UV_Value',1:'KT_Value',2:'OT_Value',3:'OC_Value',6:'MAX_SPD',9:'TIMEOUT',
             10:'CTRL_MODE',11:'Damp',12:'Inertia',13:'hw_ver',14:'sw_ver',20:'Gr',
             21:'PMAX',22:'VMAX',23:'TMAX',24:'I_BW',25:'KP_ASR',26:'KI_ASR',27:'KP_APR',28:'KI_APR'}
rows = []
with ExitStack() as stack:
    arms = {s:stack.enter_context(OpenArmDriver(s,c,.2)) for s,c in [('left',0x51),('right',0x52)]}
    for side in ('left','right'):
        for j in (6,2):
            arm=arms[side]
            state=arm.query_motor(j)
            if state.state != 0:
                raise RuntimeError('Expected disabled state; no state-changing commands sent')
            row=dict(side=side,motor_id=j,before=asdict(state),parameters={})
            for reg,name in registers.items():
                try: row['parameters'][name]=dict(register=reg,value=arm.read_parameter(j,reg))
                except Exception as exc: row['parameters'][name]=dict(register=reg,error=str(exc))
            row['after']=asdict(arm.query_motor(j))
            rows.append(row)
    final=[arms[s].snapshot() for s in ('left','right')]
result=dict(time_unix=time.time(),read_only=True,no_enable_or_MIT_frames=True,rows=rows,final=final,
            register_source='resources/openarm_control_extract/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp',
            note='Parameter equality is not proof of identical physical payload or motor capability. ASR/APR values do not replace explicit MIT Kp/Kd.')
(HERE/'hardware_parameters.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
left,right=rows[0],rows[2]
print(json.dumps(dict(j6_left=left['parameters'],j6_right=right['parameters'],
                     final=[dict(side=r['side'],online=r['online'],bus_status=r['bus_status'],states=[m.get('state') for m in r['motors']]) for r in final]),ensure_ascii=False))
