import {PianoAudio} from '/audio.js';
const $=id=>document.getElementById(id),audio=new PianoAudio();
window.__pianoAudio=audio;
const labels={left:'左手',right:'右手'},chars={left:'1234',right:'789x'};
const fingerIds={left:{pinky:'1',ring:'2',middle:'3',index:'4'},right:{index:'7',middle:'8',ring:'9',pinky:'x'}};
let plan=null,prepared=null,notes=[],moves=[],whiteKeys=[],allKeys=[],state=null,online=false,lastSeen=0;
let pending=false,generation=0,owned=false,finished=false,audioOrigin=null,eventSession=null,cursor=0;
let step=1,selected=null,songs=[],stopping=false,listVersion=0;
const sources=new Map(),keyElements=new Map();
function notice(message,error=false){for(const id of ['scoreStatus','configStatus']){$(id).textContent=message;$(id).classList.toggle('error',error);}}
async function request(url,body,timeout=6000){
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeout);
 try{const r=await fetch(url,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined,signal:controller.signal,cache:'no-store'});const result=await r.json();if(!r.ok)throw Error(result.error||r.status);return result;}
 catch(e){if(e.name==='AbortError')throw Error('请求超时，结果未知；请查看状态，不会自动重发。');throw e;}finally{clearTimeout(timer);}
}
function connected(){return online&&performance.now()-lastSeen<4000;}
function running(){return state?.score?.status==='RUNNING';}
function robotBusy(){return running()||state?.busy||['EXECUTING','PAUSED'].includes(state?.arm_state);}
function showStep(value){
 step=value;['selectScreen','playScreen','completeScreen'].forEach((id,i)=>$(id).hidden=i+1!==value);
 buttons();
}
function endScreen(status,error=''){
 $('settings').close();$('completeTitle').textContent=status==='COMPLETED'?'演奏完成':status==='STOPPED'?'演奏已结束':'演奏已中止';
 $('completeMessage').textContent=status==='COMPLETED'?'最后一个音符落下，感谢聆听。':status==='STOPPED'?'旋律暂歇，下一首等你开启。':'本次演奏未完成，请检查后重新开始。';
 $('completeSong').textContent=selected?.name||'本次演奏';notice(error,!!error);showStep(3);
}
function buttons(){
 const locked=pending||stopping||robotBusy(),hasText=!!$('scoreText').value.trim();
 for(const id of ['scoreFile','scoreText','automaticMotion','refreshSongs'])$(id).disabled=locked;
 for(const id of ['scoreVelocity','scoreAcceleration'])$(id).disabled=locked||$('automaticMotion').checked;
 $('songSelect').disabled=locked||!songs.length;
 $('scoreParse').disabled=locked||!hasText;$('scorePrepare').disabled=locked||!hasText||!connected();
 $('scoreStart').disabled=locked||!hasText||!selected||!connected();
 $('scoreStop').disabled=stopping||!connected()||step!==2;
 $('finishButton').disabled=locked;
 for(const b of keyElements.values())b.disabled=locked;
}
function clearPlan(){generation++;prepared=null;plan=null;notes=[];moves=[];owned=false;finished=false;audioOrigin=null;audio.silence();audio.releaseSamples();$('summary').textContent='';buttons();}
function buildVisual(result){
 plan=result;notes=[];moves=[];const down=new Map(),positions={left:null,right:null};
 for(const a of result.actions){
  if(a.kind==='move'){moves.push({side:a.side,from:positions[a.side]??a.index,to:a.index,start:a.time_us,end:a.time_us+a.duration_us});positions[a.side]=a.index;}
  else for(const f of a.fingers){if(a.direction==='D')down.set(f.id,{...f,start:a.time_us});else if(down.has(f.id)){notes.push({...down.get(f.id),end:a.time_us});down.delete(f.id);}}
 }
 $('summary').textContent=`${result.action_count} 条动作 · ${result.note_count} 次按指 · ${(result.duration_us/1e6).toFixed(6)} 秒`+(result.automatic_motion?` · 原速仿真峰值 ${result.peak_velocity_rad_s.toFixed(3)} rad/s、${result.peak_acceleration_rad_s2.toFixed(3)} rad/s²`:'');
}
function scoreRequest(action){return request('/api/score',{action,text:$('scoreText').value,finger_count:4,automatic_motion:$('automaticMotion').checked,velocity_limit:Number($('scoreVelocity').value),acceleration_limit:Number($('scoreAcceleration').value)},125000);}
async function check(action){
 const version=generation;pending=true;prepared=null;buttons();notice(action==='prepare'?'正在预检整份曲目…':'正在解析文本…');
 try{const r=await scoreRequest(action);if(version!==generation)return;buildVisual(r);owned=false;finished=false;if(action==='prepare')prepared=r;notice(`${action==='prepare'?'预检通过，可以开始':'已解析'} · ${r.note_count} 次按指 · ${(r.duration_us/1e6).toFixed(6)} 秒`);}
 catch(e){if(version===generation){plan=null;notes=[];moves=[];notice(e.message,true);}}finally{if(version===generation){pending=false;buttons();}}
}
function markSelection(){
 $('songSelect').value=selected?.id||'';
 $('selectionLabel').textContent=selected?`已选择 · ${selected.name}`:'选择一首喜欢的歌';$('filename').textContent=selected?.name||'尚未选择歌曲';
}
async function scanSongs(){
 const version=++listVersion;$('refreshSongs').disabled=true;
 try{const r=await request('/api/music');if(version!==listVersion)return;songs=r.songs;$('songCount').textContent=songs.length;$('songSelect').replaceChildren();
  const placeholder=new Option(songs.length?'请选择歌曲':'暂无歌曲，请添加 TXT 后刷新','');placeholder.disabled=true;$('songSelect').append(placeholder);
  for(const song of songs)$('songSelect').append(new Option(song.name,song.id));
  if(selected?.id&&!songs.some(s=>s.id===selected.id)){clearPlan();selected=null;$('scoreText').value='';}
  markSelection();
 }catch(e){notice('歌单读取失败：'+e.message,true);}finally{buttons();}
}
async function selectSong(song){
 if(pending||robotBusy())return;clearPlan();selected=null;$('scoreText').value='';markSelection();const version=generation;pending=true;buttons();notice('正在载入歌曲…');
 try{const r=await request('/api/music/text?id='+encodeURIComponent(song.id));if(version!==generation)return;selected={id:song.id,name:r.name};$('scoreText').value=r.text;markSelection();notice('');}
 catch(e){if(version===generation)notice(e.message,true);}finally{if(version===generation){pending=false;buttons();}}
}
async function loadFile(file){
 if(!file||pending||robotBusy())return;clearPlan();const version=generation;pending=true;buttons();
 try{if(file.size>512000)throw Error('TXT 文件超过 512 KB');const text=await file.text();if(version!==generation)return;selected={id:null,name:file.name.replace(/\.txt$/i,'')};$('scoreText').value=text;markSelection();await check('parse');}
 catch(e){if(version===generation)notice(e.message,true);}finally{if(version===generation){pending=false;buttons();}}
}
$('scoreFile').onchange=()=>loadFile($('scoreFile').files[0]);$('refreshSongs').onclick=scanSongs;
$('songSelect').onchange=()=>{const song=songs.find(s=>s.id===$('songSelect').value);if(song)selectSong(song);};
for(const id of ['scoreText','scoreVelocity','scoreAcceleration','automaticMotion'])$(id).oninput=()=>{
 clearPlan();if(id==='scoreText'){selected={id:null,name:selected?.name||'自定义曲目'};markSelection();}
 notice('内容或参数已改变，下次开始前将重新预检。');buttons();
};
$('scoreParse').onclick=()=>check('parse');$('scorePrepare').onclick=()=>check('prepare');
$('scoreStart').onclick=async()=>{
 if(pending||robotBusy()||!selected)return;
 const version=++generation;pending=true;owned=false;finished=false;prepared=null;showStep(2);notice('正在检查曲目与双手动作…');
 try{
  // Resume audio directly from the button gesture before asynchronous preflight.
  audio.silence();await audio.enable();if(version!==generation)return;
  const ready=await scoreRequest('prepare');if(version!==generation)return;buildVisual(ready);prepared=ready;
  notice('正在准备钢琴音色…');await audio.pinSamples(ready.sample_midis);if(version!==generation){audio.releaseSamples();return;}
  const result=await request('/api/score',{action:'start',token:ready.token},15000);if(version!==generation)return;
  owned=true;finished=false;prepared=null;state=result.piano;lastSeen=performance.now();audioOrigin=audio.context.currentTime+.15-result.piano.time_s;
  notice('');
 }catch(e){if(version===generation){audio.releaseSamples();notice(e.message,true);if(!robotBusy())showStep(1);}}
 finally{if(version===generation){pending=false;buttons();}}
};
$('scoreStop').onclick=async()=>{
 if(stopping)return;generation++;prepared=null;pending=false;stopping=true;finished=true;audioOrigin=null;audio.silence();audio.releaseSamples();buttons();notice('正在结束演奏…');
 try{const result=await request('/api/move',{action:'stop'});state=result.piano;lastSeen=performance.now();endScreen('STOPPED');}
 catch(e){notice(e.message,true);}finally{stopping=false;buttons();}
};
$('finishButton').onclick=()=>{if(pending||robotBusy())return;clearPlan();selected=null;$('scoreText').value='';markSelection();notice('');showStep(1);scanSongs();};
$('volume').oninput=()=>audio.setVolume(Number($('volume').value));
window.addEventListener('piano-audio-error',e=>notice('采样播放失败：'+e.detail,true));
let secret='',lastKey=0;
window.addEventListener('keydown',e=>{
 if(e.ctrlKey||e.metaKey||e.altKey||e.repeat||e.target.closest('input,textarea,[contenteditable="true"]')||$('settings').open)return;
 const now=performance.now();if(now-lastKey>3000)secret='';lastKey=now;secret=(secret+e.key.toLowerCase()).slice(-6);
 if(secret==='config'){secret='';$('settings').showModal();}
});
$('closeSettings').onclick=()=>$('settings').close();
function drawKeys(){
 $('keys').replaceChildren();
 let whiteIndex=0;
 for(const k of allKeys){
  if(k.white)whiteIndex++;
  const b=document.createElement('button');b.className='key'+(k.white_key===24?' boundary':'');if(!k.white){b.classList.add('black');b.style.left=(whiteIndex/52*100-.605)+'%';}if(k.white)b.dataset.key=k.white_key;b.dataset.midi=k.midi;b.title=k.white?`白键 ${k.white_key} · ${k.note}`:k.note;b.setAttribute('aria-label',b.title);
  b.innerHTML=`<span class="finger"></span><span class="number">${k.white?k.white_key:""}</span><span class="note">${k.note}</span>`;
  b.onpointerdown=async e=>{if(pending||robotBusy())return;e.preventDefault();b.setPointerCapture(e.pointerId);b.classList.add('preview');try{await audio.enable();if(b.classList.contains('preview'))audio.on(k.midi,.7,`preview:${k.midi}`);}catch(ex){notice(ex.message,true);}};
  const up=()=>{b.classList.remove('preview');audio.off(`preview:${k.midi}`);};b.onpointerup=up;b.onpointercancel=up;b.onlostpointercapture=up;
  keyElements.set(k.midi,b);$('keys').append(b);
 }
}
function updateKeys(){
 const pressed=new Set(state?.pressed||[]);
 for(const k of whiteKeys){const b=keyElements.get(k.midi),source=sources.get(k.midi),side=source?.hand||(k.white_key<=23?'left':'right'),down=pressed.has(k.midi);b.classList.toggle('pressed-left',down&&side==='left');b.classList.toggle('pressed-right',down&&side==='right');b.querySelector('.finger').textContent=down?(fingerIds[side]?.[source?.finger]||'●'):'';}
}
function displayTime(){
 if(!owned)return 0;
 const s=state?.score;if(!s)return 0;
 const extra=s.status==='RUNNING'?Math.min(100000,(performance.now()-lastSeen)*1000):0;
 return Math.min(s.duration_us,s.elapsed_us+extra);
}
function handPosition(side,t){
 const s=state?.score;
 if(plan&&!owned&&!running()){const first=moves.find(m=>m.side===side);return {pos:first?.to??null,target:first?.to??null,moving:false,preview:true};}
 if(owned&&plan){let pos=null,target=null,moving=false;for(const m of moves.filter(m=>m.side===side)){if(t<m.start)break;target=m.to;if(t>=m.end)pos=m.to;else{const u=(t-m.start)/(m.end-m.start),q=u*u*u*(10+u*(-15+6*u));pos=m.from+(m.to-m.from)*q;moving=true;break;}}if(s?.status!=='RUNNING')moving=false;return {pos,target,moving};}
 return {pos:running()?(s?.indices?.[side]??null):null,target:s?.indices?.[side]??null,moving:s?.moving?.includes(side)||false};
}
function draw(){
 const t=displayTime();
 for(const side of ['left','right']){const h=handPosition(side,t),el=$(side+'Hand');el.hidden=h.pos===null;if(h.pos!==null)el.style.left=((h.pos-1)/52*100)+'%';el.classList.toggle('moving',h.moving);const held=h.preview?[]:state?.score?.held||[];[...el.children].forEach((span,i)=>span.classList.toggle('down',held.includes(chars[side][i])));$(side+'State').textContent=h.pos===null?`${labels[side]}：未定位`:h.preview?`${labels[side]}：待定位至 ${h.target}–${h.target+3}`:h.moving?`${labels[side]}：移动至 ${h.target}–${h.target+3}`:`${labels[side]}：${Math.round(h.pos)}–${Math.round(h.pos)+3} · ${held.filter(c=>chars[side].includes(c)).join('')||'手指抬起'}`;}
 if(step!==2){requestAnimationFrame(draw);return;}
 const canvas=$('roll'),dpr=Math.min(window.devicePixelRatio||1,2),w=canvas.clientWidth,h=185;
 if(canvas.width!==Math.round(w*dpr)||canvas.height!==h*dpr){canvas.width=Math.round(w*dpr);canvas.height=h*dpr;}
 const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.strokeStyle='#23404f';ctx.lineWidth=1;
 for(let i=0;i<=52;i++){ctx.beginPath();ctx.moveTo(i*w/52,0);ctx.lineTo(i*w/52,h);ctx.stroke();}
 for(let i=0;i<3;i++){ctx.beginPath();ctx.moveTo(0,i*h/3);ctx.lineTo(w,i*h/3);ctx.stroke();}
 if(!running()||owned)for(const n of notes){if(n.start>t+3000000||n.end<t)continue;const bottom=Math.min(h,h-(n.start-t)/3000000*h),top=Math.max(0,h-(n.end-t)/3000000*h);if(bottom<0||top>h)continue;ctx.fillStyle=n.side==='left'?'#3aafd9':'#5dbfa5';ctx.fillRect((n.key-1)*w/52+2,top,w/52-4,Math.max(3,bottom-top));}
 ctx.fillStyle='#406d84';ctx.fillRect(0,h-2,w,2);requestAnimationFrame(draw);
}
async function poll(){
 try{const p=await request('/api/piano?after='+cursor,undefined,3000);state=p;online=true;lastSeen=performance.now();$('connection').textContent='● 仿真已连接';$('connection').classList.add('online');
  if(eventSession!==p.session){if(eventSession!==null){prepared=null;owned=false;notice('仿真已重启，请重新预检。');}eventSession=p.session;cursor=p.last_event_id;sources.clear();audio.silenceContact();for(const e of p.events)if(e.type==='on'&&p.pressed.includes(e.midi))sources.set(e.midi,e);}
  else for(const e of p.events){if(e.id<=cursor)continue;cursor=e.id;if(e.type==='on')sources.set(e.midi,e);else sources.delete(e.midi);if(owned&&!finished){const at=audioOrigin===null?undefined:audioOrigin+e.time_s;if(e.type==='on')audio.on(e.midi,e.velocity,`contact:${e.midi}`,at);else audio.off(`contact:${e.midi}`,at);}}
  cursor=Math.max(cursor,p.last_event_id);updateKeys();
  const s=p.score;if(s.status==='RUNNING'&&!owned&&!pending&&step!==2){showStep(2);}
  if(step===2&&!pending&&!stopping&&['COMPLETED','STOPPED','ERROR'].includes(s.status)&&!finished){finished=true;audioOrigin=null;audio.releaseSamples();if(s.status!=='COMPLETED')audio.silence();endScreen(s.status,s.error||'');} 
 }catch(e){online=false;$('connection').textContent='● 仿真未连接';$('connection').classList.remove('online');audio.silenceContact();}finally{buttons();setTimeout(poll,40);}
}
window.addEventListener('blur',()=>{for(const b of keyElements.values()){b.classList.remove('preview');audio.off(`preview:${b.dataset.midi}`);}});
try{const r=await request('/api/keyboard');allKeys=r.keys;whiteKeys=r.keys.filter(k=>k.white);drawKeys();}catch(e){notice(e.message,true);}
buttons();scanSongs();poll();draw();
