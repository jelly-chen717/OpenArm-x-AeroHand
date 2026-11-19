"""User-corrected EOP score; timing stored at 10 BPM for shared playback."""
import re
import numpy as np
from paino_skill.right_melody import smooth

SOURCE="""(=3' =2' =1' =6 =5' =4' | =3' =2' =1' =5 =5'. =4' | =3' =2' =1' =5 =4' =3' | =2' =1' =7 =5 =5 =1' =7 =1' |
=3' =2' =1' =6 =5' =4' | =3' =2' =1' =5 =5'. =4' | 5 7 5 2 7 =0 =7 =1' | 7 1' 5) 0 6 7 1'"""


def parse(source=SOURCE):
    tokens=source.replace('(',' ').replace(')',' ').replace('|',' ').split()
    slots=[]
    time=0.
    for token in tokens:
        match=re.fullmatch(r'(=?)([0-7])(\'?) (\.?)'.replace(' ',''),token)
        if not match:
            raise ValueError(f'Invalid EOP token: {token}')
        short,digit,octave,dot=match.groups()
        # Explicit user exception: =5'. means dotted eighth, not dotted sixteenth.
        beats=.75 if dot else (.25 if short else 1.)
        key=None if digit=='0' else 14+int(digit)+(7 if octave else 0)
        slots.append(dict(token=token,key=key,press_start_s=time,duration_s=beats*6,beats=beats))
        time+=beats*6
    notes=[s for s in slots if s['key'] is not None]
    # Optimize changes first, distance second across four-finger spans.
    paths={}
    for e in notes:
        updated={}
        for anchor in range(max(1,e['key']-3),min(e['key'],33)+1):
            choices=[(cost[0]+int(old!=anchor),cost[1]+abs(old-anchor),path+[anchor])
                for old,(cost,path) in paths.items()]
            best=min(choices) if choices else (0,0,[anchor])
            updated[anchor]=((best[0],best[1]),best[2])
        paths=updated
    _,anchors=min(paths.values())
    base=min(anchors)
    previous=base
    moves=[]
    for i,(e,anchor) in enumerate(zip(notes,anchors)):
        e.update(index=i+1,index_key=anchor,finger_index=e['key']-anchor,
                 y_mm=-(anchor-base)*26,press_hold_s=e['duration_s']*.8,release_wait_s=.2)
        if anchor!=previous:
            end=e['press_start_s']-.2
            moves.append(dict(start=end-.75,end=end,y0=-(previous-base)*.026,y1=-(anchor-base)*.026))
            if i:
                prev=notes[i-1]
                prev['press_hold_s']=min(prev['press_hold_s'],end-.75-.2-prev['press_start_s'])
                if prev['press_hold_s']<=0:
                    raise ValueError('No time to release before move')
        previous=anchor
    return dict(format='eop',source=source,events=notes,slots=slots,moves=moves,base_index=base,
                duration_s=time,offline_only=True,hardware_executable=False)


def state(plan,t):
    y=0.
    for move in plan['moves']:
        if t<move['start']:
            break
        y=move['y0']+(move['y1']-move['y0'])*smooth((t-move['start'])/.75)
        if t<move['end']:
            break
        y=move['y1']
    fingers=np.zeros(4)
    for e in plan['events']:
        phase=t-e['press_start_s']
        if 0<=phase<e['press_hold_s']+.2:
            fingers[e['finger_index']]=1. if phase<e['press_hold_s'] else 1-smooth((phase-e['press_hold_s'])/.2)
            return y,fingers,e
    return y,fingers,None


def grid(plan):
    times=list(np.arange(-.95,plan['duration_s'],.02))+[-.95,plan['duration_s']]
    for e in plan['events']:
        times.extend(e['press_start_s']+x for x in (0,e['press_hold_s'],e['press_hold_s']+.2))
    for move in plan['moves']:
        times.extend([move['start'],move['end']])
    return np.array(sorted(set(round(t,8) for t in times)))
