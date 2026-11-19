import {RobotView} from '/viewer.js';
import {PianoAudio} from '/audio.js';
const $=id=>document.getElementById(id),names={left:'左手',right:'右手'},fingerNames={index:'食指',middle:'中指',ring:'无名指',pinky:'小指',thumb:'拇指'};
const labels={IDLE:'等待目标',EXECUTING:'移位中',PAUSED:'暂停',STOPPED:'停止',COMPLETED:'已到位'};
const audio=new PianoAudio();window.__pianoAudio=audio;
let table=null,timing=null,layout=[],state=null,piano=null,online=false,pending=false,lastSeen=0,view=null,activeMotion=null,eventSession=null,cursor=0,activeSequence=null,demo=false;
try{view=new RobotView($('viewer'),$('viewerStatus'));}catch(e){$('viewerStatus').textContent='3D 不可用：'+e.message;}
$('resetView').onclick=()=>view?.reset();
function notice(text,error=false){$('notice').textContent=text;$('notice').classList.toggle('error',error);}
function log(text){const p=document.createElement('p');p.textContent=new Date().toLocaleTimeString()+' '+text;$('log').prepend(p);while($('log').children.length>60)$('log').lastChild.remove();}
async function request(url,body,timeout=6000){const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),timeout);try{const r=await fetch(url,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined,signal:ctrl.signal,cache:'no-store'});const value=await r.json();if(!r.ok)throw Error(value.error||r.status);return value;}catch(e){if(e.name==='AbortError')throw Error('请求超时，结果未知；请观察状态，不会自动重发。');throw e;}finally{clearTimeout(timer);}}
function owner(key){return table&&Object.keys(table.calibration.regions).find(s=>key>=table.calibration.regions[s][0]&&key<=table.calibration.regions[s][1]);}
function rowFor(key){return table?.rows.find(r=>r.arm===owner(key)&&r.key===key);}
function available(){return online&&Date.now()-lastSeen<4500&&state?.scene==='piano'&&state?.mode==='ideal';}
function busy(){return ['EXECUTING','PAUSED'].includes(state?.state)||piano?.busy;}
function valid(){const n=Number($('steps').value);return timing&&Number.isInteger(n)&&n>=1&&n<=timing.max_steps&&['velocity','acceleration'].every(id=>Number.isFinite(Number($(id).value))&&Number($(id).value)>0);}
function parameters(){return {interpolation_steps:Number($('steps').value),velocity_limit:Number($('velocity').value),acceleration_limit:Number($('acceleration').value)};}
function timingPreview(){$('timingPreview').textContent=valid()?`移位周期 ${(timing.period_s*1000).toFixed(3)} ms · ${$('steps').value} 步 = ${(Number($('steps').value)*timing.period_s).toFixed(3)} s；触键另约 0.45 s`:'请输入有效步数和正数上限';}
for(const id of ['steps','velocity','acceleration'])$(id).oninput=()=>{timingPreview();buttons();};
function preset(n,v,a){$('steps').value=n;$('velocity').value=v;$('acceleration').value=a;timingPreview();buttons();}
$('fastPreset').onclick=()=>preset(timing.default_steps,timing.default_velocity,timing.default_acceleration);
$('slowPreset').onclick=()=>preset(300,.2,.4);
function buttons(){
 const ready=available(),locked=busy()||pending||demo,mode=$('keyMode').value;
 document.querySelectorAll('[data-midi]').forEach(b=>{b.disabled=mode==='preview'?false:!ready||!valid()||locked||(mode==='position'&&(!b.dataset.key||rowFor(Number(b.dataset.key))?.status!=='ready'));});
 $('center').disabled=!ready||locked||!valid()||!table;
 for(const id of ['steps','velocity','acceleration','fastPreset','slowPreset','playHand','playFinger','keyMode'])$(id).disabled=!timing||locked;
 $('demo').disabled=!ready||locked||!valid();$('release').disabled=!ready||locked;
 document.querySelectorAll('[data-finger]').forEach(b=>b.disabled=!ready||locked);
 $('stop').disabled=!ready;$('pause').disabled=!ready||pending||(!busy()||piano?.paused||state?.state==='PAUSED');
 $('resume').disabled=!ready||pending||!(piano?.paused||state?.state==='PAUSED');
}
async function enableSound(){await audio.enable();$('sound').textContent='采样声音已开启';$('sound').classList.add('online');}
window.addEventListener('piano-audio-error',e=>notice('采样播放失败：'+e.detail,true));
$('sound').onclick=()=>enableSound().catch(e=>notice(e.message,true));
$('volume').oninput=()=>audio.setVolume(Number($('volume').value));
$('keyMode').onchange=()=>{audio.silence();buttons();notice({play:'点击琴键：机械臂移位，Aero 手指接触后发声。远端琴键可能无法到达。',position:'CSV 定位模式：只数白键，保持原中点与左右分区。',preview:'触屏试音：按住发声、松开衰减，不移动机器人。'}[$('keyMode').value]);};
function draw(){
 $('keyboard').replaceChildren();
 for(const k of layout){
  const b=document.createElement('button');b.dataset.midi=k.midi;b.className=k.white?'key '+(owner(k.white_key)||''):'black-key';
  if(k.white){b.dataset.key=k.white_key;b.style.gridColumn=k.white_key;b.innerHTML=`<span class="marker">${k.note}</span><b>${k.white_key}</b>`;}
  else{b.style.left=`${(k.after_white-.29)*100/52}%`;b.style.width=`${.58*100/52}%`;b.textContent=k.note;}
  b.title=`${k.note} · MIDI ${k.midi}${k.white?' · 白键 '+k.white_key:''}`;b.setAttribute('aria-label',b.title);if(k.midi===60)b.classList.add('anchor');
  b.onclick=()=>{if($('keyMode').value==='preview')return;void enableSound().then(()=>move($('keyMode').value==='play'?'play':'key',k)).catch(e=>notice(e.message,true));};
  b.onpointerdown=async e=>{if($('keyMode').value!=='preview')return;e.preventDefault();b.setPointerCapture(e.pointerId);b.classList.add('preview-down');try{await enableSound();if(b.classList.contains('preview-down'))audio.on(k.midi,.7,`preview:${k.midi}`);}catch(ex){notice(ex.message,true);}};
  const up=()=>{b.classList.remove('preview-down');audio.off(`preview:${k.midi}`);};b.onpointerup=up;b.onpointercancel=up;b.onlostpointercapture=up;$('keyboard').append(b);
 }
 if(table){$('rows').replaceChildren();for(const r of table.rows){const tr=document.createElement('tr');for(const v of [names[r.arm],r.key,r.offset_steps,r.delta_y_m.toFixed(4),r.status==='ready'?'可用':'未通过',r.q?r.q.map(x=>x.toFixed(6)).join(' / '):r.error]){const td=document.createElement('td');td.textContent=v;tr.append(td);}$('rows').append(tr);}
  $('count').textContent=`${table.rows.filter(r=>r.status==='ready').length} / ${table.rows.length}`;$('countNote').textContent='CSV 定位；触键按指尖位置解算';$('tableSummary').textContent=`展开 ${table.rows.length} 行中点定位`;}
 buttons();
}
function apply(s){
 state=s;online=true;lastSeen=Date.now();if(s.piano)piano=s.piano;
 const correct=s.api_version===2&&s.scene==='piano'&&s.mode==='ideal';$('connection').textContent=correct?'● 钢琴仿真已连接':'场景不匹配';$('connection').classList.toggle('online',correct);
 $('motionState').textContent=labels[s.state]||s.state;$('progress').value=s.progress||0;$('percent').textContent=Math.round((s.progress||0)*100)+'%';$('elapsed').textContent=`${(s.elapsed_s||0).toFixed(2)} / ${(s.duration_s||0).toFixed(2)} s`;
 $('stepProgress').textContent=s.interpolation_steps?`移位 ${s.completed_steps} / ${s.interpolation_steps} 步`:'等待移位';
 $('jointFeedback').replaceChildren();for(const side of ['left','right']){const p=document.createElement('div');p.textContent=`${names[side]} rad　`+(s.arms?.[side]?.q||[]).map(x=>x.toFixed(3)).join(' · ');$('jointFeedback').append(p);}view?.update(s.visual);
 if(activeMotion&&s.motion_id===activeMotion&&s.state==='COMPLETED'&&!activeSequence){notice('动作已完成，保持当前位置。');activeMotion=null;}buttons();
}
async function move(action,key){
 pending=true;if(['play','key','center'].includes(action)){activeMotion=null;activeSequence=null;notice('正在解算并发送目标…');}buttons();
 try{
  if(action==='play'){await enableSound();await audio.prepare(key.midi);}
  const body={action,...parameters()};if(action==='key')body.key=key.white_key;if(action==='play')Object.assign(body,{midi:key.midi,arm:$('playHand').value,finger:$('playFinger').value});
  const s=await request('/api/move',body,30000);apply(s);
  if(['play','key','center'].includes(action)){
   activeMotion=s.motion_id;activeSequence=action==='play'?s.piano?.sequence?.id:null;
   $('target').textContent=action==='play'?`${key.note} · ${names[s.piano?.sequence?.hand]||''}${fingerNames[$('playFinger').value]}`:action==='key'?`${names[owner(key.white_key)]} · 第 ${key.white_key} 号白键`:'双手中点';
   $('targetDetail').textContent=action==='play'?'移位 → 手指按下 → 接触发声 → 抬起':action==='key'?'按 CSV 移位':'恢复原中点';
   notice(action==='play'?'正在移位，声音由实际指尖触键事件触发。':'目标已接受，正在移动。');
  }else{if(action==='stop'){demo=false;activeSequence=null;activeMotion=null;audio.silence();}notice({stop:'已停止，手指松开。',pause:'已暂停。',resume:'继续执行。',release:'手指已松开。'}[action]);}
  log(action==='play'?`Aero ${key.note}`:action);return s;
 }catch(e){notice(e.message,true);log(e.message);throw e;}finally{pending=false;buttons();}
}
for(const id of ['center','pause','resume','stop','release'])$(id).onclick=()=>move(id).catch(()=>{});
let fingerQueue=Promise.resolve();
function fingerCommand(side,finger,amount){fingerQueue=fingerQueue.catch(()=>{}).then(()=>request('/api/move',{action:'finger',arm:side,finger,amount},10000)).then(apply).catch(e=>notice(e.message,true));}
for(const side of ['left','right'])for(const [finger,label] of Object.entries(fingerNames)){
 const b=document.createElement('button');b.dataset.finger=finger;b.dataset.hand=side;b.textContent=names[side]+label;
 b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);b.classList.add('held');void enableSound().catch(e=>notice(e.message,true));fingerCommand(side,finger,.35);};
 const release=()=>{if(b.classList.contains('held')){b.classList.remove('held');fingerCommand(side,finger,0);}};b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=release;$('fingers').append(b);
}
async function pianoPoll(){
 try{
  const p=await request('/api/piano?after='+cursor,undefined,3000);piano=p;
  if(eventSession!==p.session){eventSession=p.session;cursor=p.last_event_id;audio.silenceContact();}
  else{for(const e of p.events){if(e.id<=cursor)continue;cursor=e.id;if(p.score?.status!=='RUNNING'){if(e.type==='on')audio.on(e.midi,e.velocity);else audio.off(`contact:${e.midi}`);}}cursor=Math.max(cursor,p.last_event_id);}
  document.querySelectorAll('[data-midi]').forEach(b=>b.classList.toggle('sounding',p.pressed.includes(Number(b.dataset.midi))));
  $('contactStatus').textContent=p.pressed.length?'正在按下：'+p.pressed.map(n=>layout.find(k=>k.midi===n)?.note||n).join(' · '):'指尖接触监听中 · '+(audio.enabled?'声音已开启':'请开启声音');
  const seq=p.sequence;if(activeSequence===seq?.id){$('motionState').textContent=p.paused?'暂停':({moving:'移位中',pressing:'手指按下',releasing:'抬指',done:'弹奏完成',failed:'未触键',stopped:'已停止'}[seq.phase]||seq.phase);if(!p.busy){notice(seq.hit?'动作已完成，指尖触键与发声事件已确认。':'动作结束，但未检测到目标琴键下压。',!seq.hit);activeSequence=null;activeMotion=null;}}buttons();
 }catch(e){audio.silenceContact();$('contactStatus').textContent='触键反馈断开，接触声音已释放';}finally{setTimeout(pianoPoll,40);}
}
$('demo').onclick=async()=>{
 try{await enableSound();demo=true;buttons();for(const midi of [60,62,64,65,67,65,64,62,60]){if(!demo)break;await move('play',layout.find(k=>k.midi===midi));const id=activeSequence,start=Date.now();while(demo&&(piano?.sequence?.id!==id||piano?.busy)){if(Date.now()-start>60000)throw Error('演奏等待超时，请停止后检查');await new Promise(r=>setTimeout(r,60));}if(piano?.sequence?.phase!=='done')break;}}catch(e){notice(e.message,true);}finally{demo=false;buttons();}
};
window.addEventListener('blur',()=>{audio.silence();document.querySelectorAll('[data-finger].held').forEach(b=>{b.classList.remove('held');fingerCommand(b.dataset.hand,b.dataset.finger,0);});document.querySelectorAll('.preview-down').forEach(b=>b.classList.remove('preview-down'));});
async function poll(){try{apply(await request('/api/state',undefined,4000));}catch(e){online=false;$('connection').textContent='● 仿真未连接';$('connection').classList.remove('online');buttons();}finally{setTimeout(poll,180);}}
try{const values=await Promise.all([request('/api/table').catch(e=>{log(e.message);return null;}),request('/api/timing'),request('/api/keyboard')]);[table,timing]=values;layout=values[2].keys;preset(timing.default_steps,timing.default_velocity,timing.default_acceleration);$('steps').max=timing.max_steps;draw();notice('88 键已加载。开启声音后，点击琴键让 Aero Hand 触键弹奏。');}catch(e){notice('初始化失败：'+e.message,true);}
setInterval(buttons,500);poll();pianoPoll();
