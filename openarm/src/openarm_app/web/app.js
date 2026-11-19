import {RobotView} from './viewer.js';
import {quaternionToRPY,rpyToQuaternion} from './pose_math.js';
let robotView;try{robotView=new RobotView(document.getElementById('viewer'),document.getElementById('viewerStatus'));}catch(e){document.getElementById('viewerStatus').textContent='3D 初始化失败：'+e.message;}
document.getElementById('resetView').onclick=()=>robotView?.reset();
const $ = id => document.getElementById(id), sides = ['right','left'], names = {right:'右臂',left:'左臂'};
const labels = {IDLE:'等待目标',EXECUTING:'执行中',PAUSED:'已暂停',STOPPED:'已停止',COMPLETED:'已完成'};
let state=null, online=false, pending=false, stopRequested=false, initialized=false, lastSeen=0, commandEpoch=0;

const rad = x=>x*Math.PI/180, deg=x=>x*180/Math.PI;
for(const side of ['left','right']){
  const card=document.createElement('div');card.className='arm-card';
  card.innerHTML=`<h3>${names[side]} <span>实际角度 / 目标角度 °</span></h3>`;
  for(let i=0;i<7;i++)card.insertAdjacentHTML('beforeend',`<div class="joint-row"><span>J${i+1}</span><output id="actual-${side}-${i}">—</output><button data-side="${side}" data-index="${i}" data-sign="-1" aria-label="${names[side]} J${i+1} 减少" disabled>−</button><input id="q-${side}-${i}" type="number" step="0.1" aria-label="${names[side]} J${i+1} 目标角度" disabled><button data-side="${side}" data-index="${i}" data-sign="1" aria-label="${names[side]} J${i+1} 增加" disabled>+</button></div>`);
  $('jointPanels').append(card);
  const pose=document.createElement('div');pose.className='arm-card';pose.innerHTML=`<h3>${names[side]} <span>XYZ + Rx / Ry / Rz</span></h3>`;
  ['X','Y','Z','Rx','Ry','Rz'].forEach((label,i)=>pose.insertAdjacentHTML('beforeend',`<div class="pose-field"><label for="pose-${side}-${i}">${label}</label><input id="pose-${side}-${i}" type="number" step="0.001" disabled></div>`));
  $('posePanels').append(pose);
  const fk=document.createElement('div');fk.innerHTML=`<b>${names[side]}</b>`;["X","Y","Z","Rx","Ry","Rz"].forEach((name,i)=>fk.insertAdjacentHTML("beforeend",`<div class="feedback-value"><span>${name} · ${i<3?"m":"°"}</span><output id="fk-${side}-${i}">—</output></div>`));$('fk').append(fk);
}

function selected(){return $('arm').value==='both'?sides:[$('arm').value];}
function log(message){const p=document.createElement('p');p.textContent=new Date().toLocaleTimeString()+'  '+message;$('log').prepend(p);while($('log').children.length>100)$('log').lastChild.remove();}
function notice(message,error=false){$('notice').textContent=message;$('notice').classList.toggle('error',error);}
function available(){return online && Date.now()-lastSeen<4000;}
function updateButtons(){
  const connected=available(), busy=state && ['EXECUTING','PAUSED'].includes(state.state), movable=connected&&!pending&&!busy;
  for(const id of ['moveJoints','movePose','preview','home'])$(id).disabled=!movable;
  $('fill').disabled=!connected||pending;
  $('pause').disabled=!connected||pending||state?.state!=='EXECUTING';
  $('resume').disabled=!connected||pending||state?.state!=='PAUSED';
  $('stop').disabled=!connected; // Always available, including during a pending command.
  document.querySelectorAll('[data-sign]').forEach(b=>b.disabled=!movable||!selected().includes(b.dataset.side));
  for(const side of sides)for(const kind of ['q','pose'])for(let i=0;i<(kind==='q'?7:6);i++)$(`${kind}-${side}-${i}`).disabled=!connected||pending||!selected().includes(side);
}
async function request(path,body,timeout=30000){
  const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeout);
  try{const response=await fetch(path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined,signal:controller.signal,cache:'no-store'});const data=await response.json();if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);return data;}
  catch(e){if(e.name==='AbortError')throw Error('请求超时，结果未知；请先查看状态，不要重复发送。');throw e;}finally{clearTimeout(timer);}
}
function applyState(value){
  if(value.api_version!==2 || !value.arms?.right || !value.arms?.left)throw Error('请重启新版双臂仿真服务');
  state=value;online=true;lastSeen=Date.now();$('connection').textContent='● 本机仿真已连接';$('connection').classList.add('online');
  $('state').textContent=labels[value.state]||value.state;$('scene').textContent=value.scene==='piano'?'钢琴 · Aero Hand Open':'标准 V1 双臂场景';
  $('percent').textContent=(value.progress*100).toFixed(0)+'%';$('progress').value=value.progress;
  $('time').textContent=`${value.elapsed_s.toFixed(1)} / ${value.duration_s.toFixed(1)} s`;
  for(const side of sides){const arm=value.arms[side];for(let i=0;i<7;i++){
    $(`actual-${side}-${i}`).textContent=deg(arm.q[i]).toFixed(2)+'°';
    const input=$(`q-${side}-${i}`);input.min=deg(arm.limits[i][0]);input.max=deg(arm.limits[i][1]);
  }const six=[...arm.pose.slice(0,3),...quaternionToRPY(arm.pose.slice(3))];six.forEach((v,i)=>$(`fk-${side}-${i}`).textContent=v.toFixed(i<3?4:2));}
  robotView?.update(value.visual);
  if(!initialized){fill();initialized=true;notice('已加载双臂当前值。输入目标或使用 ± 点动；IK 预览不会运动。');log('已连接 '+(value.scene==='piano'?'钢琴场景':'双臂仿真'));}
  updateButtons();
}
function fill(){if(!state)return;for(const side of sides){for(let i=0;i<7;i++)$(`q-${side}-${i}`).value=deg(state.arms[side].q[i]).toFixed(6);const pose=state.arms[side].pose,six=[...pose.slice(0,3),...quaternionToRPY(pose.slice(3))];six.forEach((v,i)=>$(`pose-${side}-${i}`).value=v.toFixed(9));}}

function number(id){const el=$(id);if(!el.value.trim()||!Number.isFinite(el.valueAsNumber))throw Error('请填写有效数字：'+(el.getAttribute('aria-label')||id));return el.valueAsNumber;}
function targets(kind){const values={};for(const side of selected()){values[side]=Array.from({length:kind==='q'?7:6},(_,i)=>{const x=number(`${kind}-${side}-${i}`);return kind==='q'?rad(x):x;});if(kind==='pose')values[side]=[...values[side].slice(0,3),...rpyToQuaternion(values[side].slice(3))];if(kind==='q')values[side].forEach((q,i)=>{const [lo,hi]=state.arms[side].limits[i];if(q<lo||q>hi)throw Error(`${names[side]} J${i+1} 超出模型限位`);});}return $('arm').value==='both'?values:values[$('arm').value];}
async function command(action,values={}){
  if(!available()){notice('仿真反馈已断开，请等待重新连接。',true);return;}
  if(pending){if(action==='stop'){stopRequested=true;notice('停止已排队，将在当前请求返回后发送。');}return;}
  const epoch=++commandEpoch;pending=true;updateButtons();
  const arm=values.arm||$('arm').value;
  if(['move_joints','move_pose','home'].includes(action)){const count=arm==='both'?14:7;values={velocity_limits:Array(count).fill(.2*Number($('speed').value)/100),acceleration_limits:Array(count).fill(.4*Number($('speed').value)/100),...values};}
  const body={action,...(['move_joints','move_pose','preview_pose','home'].includes(action)?{arm}:{}),...values};
  try{const result=await request('/api/command',body);if(epoch!==commandEpoch)return;
    if(action==='preview_pose'){const goals=result.arm==='both'?result.q_goal:{[result.arm]:result.q_goal};for(const [side,q] of Object.entries(goals))q.forEach((v,i)=>$(`q-${side}-${i}`).value=deg(v).toFixed(6));notice('IK 预览成功：解已填入对应关节目标，机械臂未运动。');}
    else{applyState(result);notice(action==='stop'?'已停止，保持当前位置。':'指令已接受；完成状态以仿真反馈为准。');}
    log(action+' · '+(body.arm||'整个动作'));
  }catch(e){notice(e.message,true);log('指令失败 / 未确认：'+e.message);}finally{if(epoch===commandEpoch){pending=false;updateButtons();if(stopRequested){stopRequested=false;command('stop');}}}
}
function guarded(fn){return ()=>{try{fn();}catch(e){notice(e.message,true);log(e.message);}};}
$('moveJoints').onclick=guarded(()=>command('move_joints',{q:targets('q')}));
$('movePose').onclick=guarded(()=>command('move_pose',{pose:targets('pose')}));
$('preview').onclick=guarded(()=>command('preview_pose',{pose:targets('pose')}));
for(const action of ['pause','resume','stop','home'])$(action).onclick=()=>command(action);
document.querySelectorAll('[data-sign]').forEach(button=>button.onclick=guarded(()=>{const side=button.dataset.side,index=Number(button.dataset.index),q=state.arms[side].q.slice();q[index]+=rad(Number($('step').value)*Number(button.dataset.sign));command('move_joints',{arm:side,q});}));
$('fill').onclick=()=>{fill();notice('已填入当前值。');};$('clear').onclick=()=>$('log').replaceChildren();$('arm').onchange=updateButtons;
$('speed').oninput=()=>$('speedLabel').textContent=$('speed').value+'%';
async function poll(){const epoch=commandEpoch;try{const value=await request('/api/state',undefined,4000);if(epoch===commandEpoch)applyState(value);}catch(e){online=false;$('connection').textContent='● 仿真未连接';$('connection').classList.remove('online');$('state').textContent='反馈断开';$('percent').textContent='—';if(initialized)notice('反馈已断开，显示值为最后记录。请检查 MuJoCo 窗口。',true);updateButtons();}finally{setTimeout(poll,400);}}
setInterval(updateButtons,500);log('控制台已就绪，等待本机仿真。');poll();
