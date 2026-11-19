"""Read registers and feedback only; never enable, disable, zero, or command MIT."""
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path
import json
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'apps'))
from openarm_driver import OpenArmDriver, LIMITS, motor_type

out = Path(__file__).with_name('readonly_probe.json')
keys = [(side, j) for j in range(7, 0, -1) for side in ('left', 'right')]
with ExitStack() as stack:
    arms = {s: stack.enter_context(OpenArmDriver(s, c, .1)) for s,c in [('left',0x51),('right',0x52)]}
    before = [a.snapshot() for a in arms.values()]
    if any(r['online'] != 7 or r['bus_status'] != '0x0' or any(m['state'] != 0 for m in r['motors']) for r in before):
        raise RuntimeError('Read-only probe requires 14 online disabled joints')
    params = []
    for s,j in keys:
        values = [arms[s].read_parameter(j, r) for r in (10,21,22,23)]
        params.append(dict(side=s,motor_id=j,registers=[10,21,22,23],values=values,expected=[1,*LIMITS[motor_type(j)]]))
    cycles, skew = [], []
    latest = {}
    for n in range(30):
        begin = time.perf_counter()
        stamps = {}
        for s,j in keys:
            if arms[s].read_parameter(j,10) != 1:
                raise RuntimeError('Not MIT mode')
            state = arms[s].query_motor(j)
            if state.state != 0:
                raise RuntimeError('External state changed; no motion was sent by this probe')
            latest[f'{s}:{j}'] = asdict(state)
            stamps[(s,j)] = time.perf_counter()
        cycles.append(time.perf_counter()-begin)
        skew.extend(abs(stamps[('left',j)]-stamps[('right',j)]) for j in range(1,8))
        time.sleep(.02)
    after = [a.snapshot() for a in arms.values()]
result = dict(time_unix=time.time(),kind='read_only_no_MIT_command_no_enable',before=before,after=after,
              parameters=params,latest=latest,cycle_seconds=cycles,
              cycle_median_s=statistics.median(cycles),cycle_max_s=max(cycles),
              cycle_p95_s=sorted(cycles)[int(.95*(len(cycles)-1))],paired_reply_skew_max_s=max(skew),
              note='Register+feedback reads approximate command-loop transaction count; no real MIT dispatch timing measured.')
out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:result[k] for k in ('kind','cycle_median_s','cycle_p95_s','cycle_max_s','paired_reply_skew_max_s')}))
