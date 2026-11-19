
'use strict';
const $=id=>document.getElementById(id),assets=JSON.parse($('assets').textContent),library=assets.songs;
const whites=Array.from({length:88},(_,i)=>i+21).filter(m=>[0,2,4,5,7,9,11].includes(m%12));
const keyboardRange = { minMidi: 21, maxMidi: 108, firstWhite: 1, lastWhite: 52, whiteCount: 52 };
const seq={left:'1234',right:'789x'},MAX=86400000000;
const number='[+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?';
const moveRE=new RegExp('^(left|right)_arm_move\\(\\s*([+-]?\\d+)\\s*,\\s*('+number+')\\s*\\)$'),sleepRE=new RegExp('^sleep\\(\\s*('+number+')\\s*\\)$');
function micros(s,positive=false){
 if(s.length>64||!Number.isFinite(Number(s))||Number(s)<0||Number(s)>86400)throw Error('时间必须在 0–86400 秒内');
 const parts=s.toLowerCase().replace(/^\+/,'').split('e'),p=parts[0].split('.'),digits=(p[0]+(p[1]||'')).replace(/^-/,'');
 let n=BigInt(digits),power=6+Number(parts[1]||0)-(p[1]||'').length;
 if(n===0n){if(positive)throw Error('移动时间必须大于 0');return 0;}
 if(Math.abs(power)>100)throw Error('时间必须精确到整数微秒');
 if(power>=0)n*=10n**BigInt(power);else{const d=10n**BigInt(-power);if(n%d)throw Error('时间必须精确到整数微秒');n/=d;}
 if(positive&&n===0n)throw Error('移动时间必须大于 0');return Number(n);
}
function parseScore(text){
 if(text.length>512000)throw Error('文本超过 512000 字符');
 let t=0;const index={left:null,right:null},end={left:0,right:0},held=new Map(),actions=[],notes=[];
 text.replace(/^\uFEFF/,'').split(/\r?\n/).forEach((raw,i)=>{const s=raw.split('#')[0].trim();if(!s)return;
 try{let m;if((m=s.match(sleepRE))){t+=micros(m[1]);if(t>MAX)throw Error('总时间超过 24 小时');return;}
 if((m=s.match(moveRE))){const side=m[1],k=Number(m[2]),dt=micros(m[3],true),lo=side==='left'?keyboardRange.firstWhite:24,hi=side==='left'?20:keyboardRange.lastWhite-3;
 if(!Number.isSafeInteger(k)||k<lo||k>hi)throw Error(side+' 四指覆盖超出 A0–C8 音域，index 应为 '+lo+'–'+hi);
 if(end[side]>t)throw Error('该侧移动尚未完成');if([...held.keys()].some(f=>seq[side].includes(f)))throw Error('移动前必须抬起该侧全部手指');
 if(t+dt>MAX)throw Error('移动结束超过 24 小时');index[side]=k;end[side]=t+dt;actions.push({kind:'move',side,index:k,time:t,duration:dt});
 }else if((m=s.match(/^hand\(\s*([DU])\s*,\s*([0-9x]+)\s*\)$/))){const direction=m[1],chars=[...m[2]],fingers=[];
 if(new Set(chars).size!==chars.length)throw Error('同一指令重复手指');
 for(const f of chars){const side=seq.left.includes(f)?'left':seq.right.includes(f)?'right':null;if(!side)throw Error('四指模式不允许手指 '+f);
 const key=index[side]===null?null:index[side]+seq[side].indexOf(f);
 if(direction==='D'){if(key===null)throw Error('该侧尚未指定 index');if(end[side]>t)throw Error('移动未完成，不能按键');if(held.has(f))throw Error('手指尚未抬起');const note={start:t,end:null,key,midi:whites[key-1],side,f};notes.push(note);held.set(f,note);}
 else if(held.has(f)){held.get(f).end=t;held.delete(f);}fingers.push({f,side,key});}
 actions.push({kind:'hand',direction,fingers,time:t});
 }else throw Error('未知指令，只接受移臂、hand、sleep');
 if(actions.length>20000)throw Error('动作超过 20000 条');
 }catch(e){throw Error(`第 ${i+1} 行（${(t/1e6).toFixed(6)} s）：${e.message}`);}});
 if(held.size)throw Error('结束时仍有手指按下：'+[...held.keys()].join(''));if(!actions.length)throw Error('文件没有动作');
 return {actions,notes,duration:Math.max(t,end.left,end.right)};
}
async function audio(){if(!ctx){ctx=new AudioContext();master=ctx.createGain();master.gain.value=Number($('volume').value);analyser=ctx.createAnalyser();master.connect(analyser);analyser.connect(ctx.destination);}await ctx.resume();}
async function prepare(midi){if(buffers.has(midi))return buffers.get(midi);const encoded=assets.samples[midi];if(!encoded)throw Error('缺少琴键采样：MIDI '+midi);const bytes=Uint8Array.from(atob(encoded),c=>c.charCodeAt(0));const b=await ctx.decodeAudioData(bytes.buffer);buffers.set(midi,b);return b;}
function sound(note,at,duration){const source=ctx.createBufferSource(),gain=ctx.createGain();source.buffer=buffers.get(note.midi);source.connect(gain);gain.connect(master);gain.gain.setValueAtTime(.8,at);gain.gain.setValueAtTime(.8,at+duration);gain.gain.linearRampToValueAtTime(0,at+duration+.04);source.start(at);source.stop(at+duration+.05);const v={source,gain};voices.push(v);source.onended=()=>{source.disconnect();gain.disconnect();voices=voices.filter(x=>x!==v);};}
function silence(){for(const v of voices){try{v.source.stop();}catch{}v.source.disconnect();v.gain.disconnect();}voices=[];}
function draw(){if(!run)return;const t=position(),state={left:null,right:null};
 for(const a of run.score.actions){if(a.time>t)break;if(a.kind==='move'){const old=state[a.side],from=old?old.index:a.index;state[a.side]={index:a.index,position:from+(a.index-from)*Math.min(1,(t-a.time)/a.duration),moving:t<a.time+a.duration};}}
 const active=run.score.notes.filter(n=>n.start<=t&&n.end>t);document.querySelectorAll('.key.active').forEach(e=>e.classList.remove('active','right'));
 for(const n of active){const k=$('k'+n.key);k.classList.add('active');if(n.side==='right')k.classList.add('right');}
 for(const side of ['left','right']){const el=$(side),s=state[side];el.hidden=!s;if(s){el.style.left=((s.position-keyboardRange.firstWhite)/keyboardRange.whiteCount*100)+'%';el.classList.toggle('moving',s.moving);}Array.from(el.children).forEach((c,i)=>c.classList.toggle('down',active.some(n=>n.f===seq[side][i])));}
 const elapsed=Math.min(t,run.score.duration);$('progress').value=run.score.duration?elapsed/run.score.duration:1;$('progressPercent').textContent=Math.round($('progress').value*100)+'%';$('elapsed').textContent=formatTime(elapsed)+' / '+formatTime(run.score.duration);
 $('positions').innerHTML=['left','right'].map(s=>'<span>'+(s==='left'?'左手':'右手')+'：'+(state[s]?`${state[s].index}–${state[s].index+3}${state[s].moving?' 移动中':''}`:'未定位')+'</span>').join('');
 if(!pianoStage){const c=$('roll'),w=c.clientWidth,dpr=devicePixelRatio||1;if(c.width!==Math.round(w*dpr)){c.width=Math.round(w*dpr);c.height=210*dpr;}const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,210);g.strokeStyle='#23434f';for(let i=0;i<=keyboardRange.whiteCount;i++){g.beginPath();g.moveTo(i*w/keyboardRange.whiteCount,0);g.lineTo(i*w/keyboardRange.whiteCount,210);g.stroke();}
 for(const n of run.score.notes){if(n.end<t||n.start>t+3000000)continue;const y=210-(n.start-t)/3000000*210,h=Math.max(3,(n.end-n.start)/3000000*210);g.fillStyle=n.side==='left'?'#79C7FF':'#EDBA78';g.fillRect((n.key-keyboardRange.firstWhite)*w/keyboardRange.whiteCount+1,y-h,w/keyboardRange.whiteCount-2,h);}}$('remaining').textContent='预计剩余 '+formatTime((run.score.duration-t)/rate);if(status==='播放中')frame=requestAnimationFrame(draw);
}
function formatTime(us){const seconds=Math.floor(us/1e6);return Math.floor(seconds/60)+':'+String(seconds%60).padStart(2,'0');}
let count=keyboardRange.firstWhite-1;for(let midi=keyboardRange.minMidi;midi<=keyboardRange.maxMidi;midi++){const white=whites.includes(midi),k=document.createElement('button');k.className='key'+(white?'':' black');if(white){count++;k.id='k'+count;k.style.left=((count-keyboardRange.firstWhite)*100/keyboardRange.whiteCount)+'%';k.innerHTML='<span>'+count+'<br>'+['C','C♯','D','D♯','E','F','F♯','G','G♯','A','A♯','B'][midi%12]+(Math.floor(midi/12)-1)+'</span>';k.title='试听白键 '+count;k.onclick=async()=>{if(run||loading)return;try{const token=loadToken;await audio();await prepare(midi);if(token===loadToken&&!document.hidden&&!run&&!loading)sound({midi},ctx.currentTime,.4);}catch(e){$('notice').textContent=e.message;}};}else{k.style.left=((count-keyboardRange.firstWhite+1-.3)*100/keyboardRange.whiteCount)+'%';k.disabled=true;k.setAttribute('aria-label','黑键（TXT 仅使用白键）');}$('keys').append(k);}
for(const side of ['left','right'])$(side).innerHTML=[...seq[side]].map(f=>'<span><b>'+f+'</b></span>').join('');
let ctx,master,analyser,run=null,loading=false,timer=0,frame=0,voices=[],buffers=new Map();
let selected=-1,status='待选曲',rate=1,loadToken=0,importing=false,query='',composing=false,searchTimer,scrollFrame=0;
const reduced=matchMedia('(prefers-reduced-motion:reduce)'),colors=['#91cfff','#8ed9ca','#c5b1e7','#edc38e','#a8b4ef'];
for(const song of library){try{song.score=parseScore(song.text);}catch(e){song.error=e.message;}}
function position(){return !run?0:Math.max(0,Math.min(run.score.duration,run.offset+(status==='播放中'?(ctx.currentTime-run.anchor)*1e6*rate:0)));}
function notice(text=''){$('notice').textContent=text;}
function updateRows() {
  document.querySelectorAll('.song').forEach(button => {
    const song = library[Number(button.dataset.index)];
    const current = run?.song === song && ['播放中', '已暂停'].includes(status);
    button.setAttribute('aria-pressed', String(Number(button.dataset.index) === selected));
    button.classList.toggle('is-current', current);
    button.classList.toggle('is-playing', current && status === '播放中');
    button.querySelector('.mark').innerHTML = song.error ? icons.error :
      current ? icons.bars : Number(button.dataset.index) === selected ? icons.check : '';
  });
}
const icons={check:'<svg viewBox="0 0 24 24" aria-label="已选中"><path d="m5 12 4 4L19 6"/></svg>',bars:'<svg viewBox="0 0 24 24" aria-label="当前演奏"><path d="M5 9v6m5-10v14m5-12v10m5-7v4"/></svg>',error:'<svg viewBox="0 0 24 24" aria-label="曲谱错误"><path d="m12 3 10 18H2zM12 9v5m0 3v1"/></svg>'};
function renderLibrary() {
  const rows = [];
  library.forEach((song, index) => {
    if (!song.name.toLocaleLowerCase().includes(query)) return;
    const button = document.createElement('button');
    button.className = 'song';
    button.dataset.index = index;
    button.title = song.name;
    button.setAttribute('aria-label', song.name);
    button.innerHTML = '<span class="song-index" aria-hidden="true"></span>' +
      '<span class="song-copy"><span class="name"></span><span class="song-duration"></span></span>' +
      '<span class="mark"></span>';
    button.querySelector('.song-index').textContent = String(index + 1).padStart(2, '0');
    button.querySelector('.name').textContent = song.name;
    button.querySelector('.song-duration').textContent = song.score ? formatTime(song.score.duration) : '曲谱待修正';
    button.onclick = () => {
      if (suppressClick || loading) return;
      selectSong(index);
      if (modalDrawer()) drawer(false);
    };
    button.ondblclick = () => {
      if (!suppressClick && !loading) { selectSong(index); start(); }
    };
    button.onfocus = () => { $('focusTitle').textContent = song.name; reveal(button); };
    rows.push(button);
  });
  $('rows').replaceChildren(...rows);
  $('empty').hidden = rows.length !== 0;
  $('libraryCount').textContent = query ? rows.length + ' / ' + library.length + ' 首' : library.length + ' 首';
  updateRows();
}
function reveal(b){const list=$('songList'),r=b.getBoundingClientRect(),v=list.getBoundingClientRect();cancelAnimationFrame(scrollFrame);if(r.top<v.top+12)list.scrollTop-=v.top+12-r.top;else if(r.bottom>v.bottom-12)list.scrollTop+=r.bottom-v.bottom+12;}
function selectSong(i){selected=i;const song=library[i];if(!run)status=song.error?'出错':'已选曲';notice(song.error?'曲谱无效：'+song.error+'。请点击“编辑曲谱”修正。':'');updateUI();}
function updateUI() {
  document.body.classList.toggle('is-playing', status === '播放中');
  wakeChrome();
  $('stageHint').hidden = !!run;
  const song = library[selected];
  const active = !!run;
  const ended = ['已完成', '已结束', '出错'].includes(status);
  const paused = status === '已暂停';
  $('state').textContent = status;
  $('playingName').textContent = run ? run.name : song?.name || '选一首歌，让音乐响起。';
  $('playingName').title = $('playingName').textContent;
  $('selectionInfo').textContent = run ? '当前演奏 · ' + formatTime(run.score.duration) :
    song?.score ? '曲谱时长 ' + formatTime(song.score.duration) :
    song?.error ? '修正曲谱后即可演奏' : '从曲库选择歌曲，或导入本地 TXT。';
  const pending = active && song && song !== run.song;
  $('pending').hidden = !pending;
  $('pendingName').textContent = pending ? '待播放：' + song.name : '';
  $('playPending').disabled = loading || !!song?.error;
  $('start').disabled = loading || (active && !ended && !paused) || !(ended && run ? run.song.score : song?.score);
  $('start').textContent = paused ? '继续' : '开始';
  $('pause').disabled = loading || status !== '播放中';
  $('restart').disabled = loading || !active;
  $('stop').disabled = !loading && (!active || ended);
  $('cancel').hidden = !loading;
  $('choose').hidden = !ended;
  $('settings').disabled = loading || !song;
  $('speed').disabled = loading;
  if (!run) {
    const duration = song?.score?.duration || 0;
    $('progress').value = 0;
    $('progressPercent').textContent = '0%';
    $('elapsed').textContent = '0:00 / ' + formatTime(duration);
    $('remaining').textContent = duration ? '预计剩余 ' + formatTime(duration / rate) : '预计剩余 —';
  }
  updateRows();
}
function cleanup(){clearTimeout(timer);timer=0;cancelAnimationFrame(frame);frame=0;silence();}
function finish(message='演奏已结束'){if(loading){cancelLoading();return;}if(!run)return;run.offset=position();cleanup();status=message==='演奏完成'?'已完成':message==='出错'?'出错':'已结束';if(status==='已完成')run.offset=run.score.duration;draw();updateUI();}
function schedule(){if(!run||status!=='播放中')return;try{if(ctx.state!=='running')throw Error('音频设备中断，请重新开始');const t=position();while(run.next<run.score.notes.length){const n=run.score.notes[run.next];if(n.start>t+120000*rate)break;run.next++;if(n.end<=t)continue;const at=ctx.currentTime+Math.max(0,n.start-t)/1e6/rate;if(t-n.start>run.tolerance*1e6*rate&&n.start>=run.offset)throw Error('前台调度超时，请重新开始并保持页面在前台');sound(n,at,(n.end-Math.max(t,n.start))/1e6/rate);}if(t>=run.score.duration){finish('演奏完成');return;}timer=setTimeout(schedule,20);}catch(e){finish('出错');notice(e.message);}}
function align(offset){cleanup();run.offset=offset;run.anchor=ctx.currentTime;run.next=run.score.notes.findIndex(n=>n.end>offset);if(run.next<0)run.next=run.score.notes.length;status='播放中';// 恢复跨越暂停点的持续音符，后续音符仍由同一音频时钟调度。
while(run.next<run.score.notes.length&&run.score.notes[run.next].start<offset){const n=run.score.notes[run.next++];if(n.end>offset)sound(n,ctx.currentTime,(n.end-offset)/1e6/rate);}schedule();if(status==='播放中')draw();updateUI();}
function cancelLoading(){loadToken++;loading=false;cleanup();status=run?'已结束':selected<0?'待选曲':'已选曲';notice('音色准备已取消，可重新开始。');updateUI();}
async function start(replay=false){if(loading)return;const song=replay&&run?run.song:library[selected];if(!song)return;if(song.error){notice('曲谱无效：'+song.error+'。请编辑曲谱。');return;}cleanup();run=null;loading=true;status='准备音色';const token=++loadToken;updateUI();notice('正在准备本曲所需的全部钢琴采样…');try{await audio();const midis=[...new Set(song.score.notes.map(n=>n.midi))];for(let i=0;i<midis.length;i++){if(token!==loadToken)return;await prepare(midis[i]);notice('正在准备音色 '+(i+1)+' / '+midis.length);}if(token!==loadToken)return;const tolerance=Number($('tolerance').value);if(!Number.isFinite(tolerance)||tolerance<50||tolerance>1000)throw Error('调度容差必须为 50–1000 毫秒，请在编辑曲谱中修正');if(document.hidden)throw Error('页面不在前台，请返回后重新开始');run={song,name:song.name,score:song.score,offset:0,anchor:ctx.currentTime,next:0,tolerance:tolerance/1000};loading=false;notice();align(0);}catch(e){if(token!==loadToken)return;cleanup();loading=false;status='出错';notice('无法开始：'+e.message+'。可重试或编辑曲谱。');updateUI();}}
async function pause(){if(!run||loading)return;if(status==='播放中'){run.offset=position();cleanup();status='已暂停';draw();updateUI();}else if(status==='已暂停'){try{await audio();if(status==='已暂停'&&!document.hidden)align(run.offset);}catch(e){finish('出错');notice(e.message);}}}
const hoverPointer = matchMedia('(hover: hover) and (pointer: fine)');
const libraryPanel = $('library');
const libraryEdge = $('libraryEdge');
let drawerOpenTimer = 0;
let drawerCloseTimer = 0;
let drawerBlocked = false;
let libraryPinned = false;
let keyboardLibrary = false;
let filePickerOpen = false;

function modalDrawer() {
  return !hoverPointer.matches && innerWidth < 1024;
}
function clearDrawerTimers() {
  clearTimeout(drawerOpenTimer);
  clearTimeout(drawerCloseTimer);
}
function drawer(open, focus = false) {
  clearDrawerTimers();
  const active = document.activeElement;
  if (!open && libraryPanel.contains(active)) {
    const target = $('drawerOpen');
    target.focus({preventScroll:true});
  }
  libraryPanel.inert = !open;
  libraryPanel.setAttribute('aria-hidden', String(!open));
  libraryPanel.classList.toggle('open', open);
  document.body.classList.toggle('library-open', open);
  for (const button of [$('drawerOpen'), libraryEdge]) {
    button.setAttribute('aria-expanded', String(open));
  }
  const modal = open && modalDrawer();
  $('transport').inert = modal;
  $('mixControls').inert = modal;
  $('overlay').hidden = !modal;
  $('stagePanel').inert = modal;
  if (modal) {
    libraryPanel.setAttribute('role', 'dialog');
    libraryPanel.setAttribute('aria-modal', 'true');
  } else {
    libraryPanel.removeAttribute('role');
    libraryPanel.removeAttribute('aria-modal');
  }
  if (open && (focus || modal)) $('search').focus({preventScroll:true});
  if (!open) { keyboardLibrary = false; libraryPinned = false; }
  wakeChrome();
}
function scheduleDrawerClose() {
  clearTimeout(drawerOpenTimer);
  clearTimeout(drawerCloseTimer);
  if (!hoverPointer.matches || !libraryPanel.classList.contains('open')) return;
  drawerCloseTimer = setTimeout(() => {
    const active = document.activeElement;
    const editing = libraryPanel.contains(active) && active.matches('input, textarea');
    if (libraryPanel.matches(':hover') || libraryEdge.matches(':hover') ||
        editing || libraryPinned || keyboardLibrary || filePickerOpen || importing || $('config').open || drag) return;
    drawer(false);
  }, 350);
}
function enterLibrary() {
  clearTimeout(drawerCloseTimer);
  if (!hoverPointer.matches || drawerBlocked || $('config').open) return;
  clearTimeout(drawerOpenTimer);
  drawerOpenTimer = setTimeout(() => drawer(true), 300);
}
function closeLibrary() {
  drawerBlocked = libraryPanel.matches(':hover') || libraryEdge.matches(':hover');
  drawer(false);
}
for (const region of [libraryEdge, libraryPanel]) {
  region.addEventListener('pointerenter', enterLibrary);
  region.addEventListener('pointerleave', event => {
    if (event.relatedTarget instanceof Node &&
        (libraryPanel.contains(event.relatedTarget) || libraryEdge.contains(event.relatedTarget))) return;
    drawerBlocked = false;
    scheduleDrawerClose();
  });
}
for (const button of [$('drawerOpen'), libraryEdge]) {
  button.onclick = () => {
    drawerBlocked = false;
    const open = !libraryPanel.classList.contains('open');
    libraryPinned = open;
    keyboardLibrary = button.matches(':focus-visible');
    drawer(open, true);
  };
}
$('drawerClose').onclick = closeLibrary;
$('overlay').onclick = closeLibrary;
libraryPanel.addEventListener('pointerdown', () => { keyboardLibrary = false; });
document.addEventListener('pointerup', scheduleDrawerClose);
document.addEventListener('pointercancel', scheduleDrawerClose);
libraryPanel.addEventListener('keydown', () => { keyboardLibrary = true; });
libraryPanel.addEventListener('focusin', () => {
  if (document.activeElement.matches(':focus-visible')) keyboardLibrary = true;
  clearTimeout(drawerCloseTimer);
});
libraryPanel.addEventListener('focusout', () => {
  setTimeout(() => {
    if (!libraryPanel.contains(document.activeElement)) keyboardLibrary = false;
    scheduleDrawerClose();
  }, 0);
});
document.addEventListener('keydown', event => {
  if (!libraryPanel.classList.contains('open') || $('config').open) return;
  if (event.key === 'Escape') {
    event.preventDefault();
    closeLibrary();
    return;
  }
  if (event.key !== 'Tab' || !modalDrawer()) return;
  const items = [...libraryPanel.querySelectorAll('button,input')]
    .filter(item => !item.disabled && item.getClientRects().length);
  const first = items[0], last = items.at(-1);
  if (event.shiftKey && document.activeElement === first) {
    event.preventDefault(); last.focus();
  } else if (!event.shiftKey && document.activeElement === last) {
    event.preventDefault(); first.focus();
  }
});
function syncDrawerMode() {
  drawerBlocked = false;
  drawer(false);
  if (run && status !== '播放中') draw();
}
addEventListener('resize', syncDrawerMode);
hoverPointer.addEventListener('change', syncDrawerMode);
$('config').addEventListener('close', scheduleDrawerClose);
function finishFilePicker() {
  filePickerOpen = false;
  scheduleDrawerClose();
}
$('files').addEventListener('cancel', finishFilePicker);
$('files').addEventListener('change', finishFilePicker);
addEventListener('focus', () => {
  if (filePickerOpen) setTimeout(finishFilePicker, 300);
});
$('songList').onkeydown=e=>{const buttons=[...document.querySelectorAll('.song')],i=buttons.indexOf(document.activeElement);let next;if(e.key==='ArrowDown')next=Math.min(buttons.length-1,i+1);if(e.key==='ArrowUp')next=Math.max(0,i-1);if(e.key==='Home')next=0;if(e.key==='End')next=buttons.length-1;if(next!==undefined){e.preventDefault();buttons[next]?.focus({preventScroll:true});}};
function search(){clearTimeout(searchTimer);if(!composing)searchTimer=setTimeout(()=>{query=$('search').value.toLocaleLowerCase();renderLibrary();$('songList').scrollTop=0;},120);}
$('search').oninput=search;$('search').addEventListener('compositionstart',()=>{composing=true;clearTimeout(searchTimer);});$('search').addEventListener('compositionend',()=>{composing=false;search();});$('clearSearch').onclick=()=>{clearTimeout(searchTimer);$('search').value='';query='';renderLibrary();$('search').focus();};
let suppressClick=false,drag=null,scrollTarget=0,lastWheel=0;
const list=$('songList');list.addEventListener('wheel',e=>{cancelAnimationFrame(scrollFrame);const wheel=e.deltaMode!==0||(Number.isInteger(e.deltaY)&&Math.abs(e.deltaY)>=80&&e.deltaX===0);if(reduced.matches||!wheel){lastWheel=0;return;}e.preventDefault();const now=performance.now();if(now-lastWheel>260)scrollTarget=list.scrollTop;lastWheel=now;scrollTarget=Math.max(0,Math.min(list.scrollHeight-list.clientHeight,scrollTarget+e.deltaY*(e.deltaMode===1?16:e.deltaMode===2?list.clientHeight:1)));const from=list.scrollTop,target=scrollTarget;function step(t){const p=Math.min(1,(t-now)/240);list.scrollTop=from+(target-from)*(1-Math.pow(1-p,3));if(p<1)scrollFrame=requestAnimationFrame(step);}scrollFrame=requestAnimationFrame(step);},{passive:false});
list.addEventListener('pointerdown',e=>{cancelAnimationFrame(scrollFrame);if(e.pointerType!=='mouse'||e.button!==0)return;drag={y:e.clientY,top:list.scrollTop,id:e.pointerId,started:false};suppressClick=false;});list.addEventListener('pointermove',e=>{if(!drag)return;const d=e.clientY-drag.y;if(!drag.started&&Math.abs(d)>6){drag.started=true;suppressClick=true;list.setPointerCapture(e.pointerId);}if(drag.started){list.scrollTop=drag.top-d;e.preventDefault();}});function endDrag(){if(!drag)return;const started=drag.started;drag=null;if(started)setTimeout(()=>suppressClick=false,400);}list.addEventListener('pointerup',endDrag);list.addEventListener('pointercancel',endDrag);list.addEventListener('touchstart',()=>cancelAnimationFrame(scrollFrame),{passive:true});
$('start').onclick=()=>status==='已暂停'?pause():start(['已完成','已结束','出错'].includes(status));$('playPending').onclick=()=>start();$('pause').onclick=pause;$('restart').onclick=()=>start(true);$('stop').onclick=()=>finish();$('cancel').onclick=cancelLoading;
$('speed').oninput = () => {
  const offset = position();
  const playing = status === '播放中';
  rate = Number($('speed').value);
  $('speedValue').value = rate.toFixed(1) + '×';
  $('speed').setAttribute('aria-valuetext', rate.toFixed(1) + '倍');
  if (run) {
    if (playing) align(offset);
    else { run.offset = offset; draw(); }
  } else updateUI();
};
$('volume').oninput = () => {
  const volume = Number($('volume').value);
  $('volumeValue').value = Math.round(volume * 100) + '%';
  $('volume').setAttribute('aria-valuetext', Math.round(volume * 100) + '%');
  if (master) master.gain.setTargetAtTime(volume, ctx.currentTime, .02);
};
$('choose').onclick=()=>{cleanup();run=null;status=selected<0?'待选曲':library[selected].error?'出错':'已选曲';updateUI();keyboardLibrary=true;drawer(true,true);document.querySelector('.song[aria-pressed=true]')?.focus();};
$('import').onclick=()=>{filePickerOpen=true;clearDrawerTimers();$('files').click();};$('files').onchange=async e=>{if(importing)return;const files=[...e.target.files];if(!files.length)return;importing=true;$('import').disabled=true;$('import').textContent='正在校验…';const added=[],fail=[];try{for(const f of files){try{if(!/\.txt$/i.test(f.name))throw Error('仅支持 TXT 文件');if(f.size>2000000)throw Error('文件超过 2 MB');const text=await f.text(),score=parseScore(text);added.push({name:f.name.replace(/\.txt$/i,''),text,score});}catch(err){fail.push(f.name+'：'+err.message);}}if(fail.length){notice('本批次未导入：\n'+fail.join('\n'));}else{const first=library.length;library.push(...added);clearTimeout(searchTimer);query='';$('search').value='';renderLibrary();selectSong(first);document.querySelector('.song[data-index="'+first+'"]')?.focus();notice('已导入 '+added.length+' 首，仅在本次会话保留。');}}finally{importing=false;$('import').disabled=false;$('import').textContent='导入 TXT';e.target.value='';scheduleDrawerClose();}};
$('settings').onclick=()=>{const song=library[selected];if(!song||loading)return;$('text').value=song.text;$('configError').textContent='';$('config').showModal();};$('close').onclick=()=>$('config').close();$('apply').onclick=()=>{try{const text=$('text').value,score=parseScore(text);library.push({name:library[selected].name+'（编辑）',text,score});query='';$('search').value='';renderLibrary();selectSong(library.length-1);$('config').close();}catch(e){$('configError').textContent=e.message;}};
document.addEventListener('visibilitychange',()=>{if(document.hidden){if(loading)cancelLoading();else if(run&&['播放中','已暂停'].includes(status))finish();silence();notice('页面进入后台，演奏已结束。返回后请点击“再听一次”重新启动。');}});window.addEventListener('pagehide',()=>{loadToken++;loading=false;cleanup();if(run){run.offset=position();status='已结束';}});

let pianoStage = null;
let wasFullscreen = false;
function wakeChrome() {
  document.body.classList.remove('chrome-hidden');
}
function setImmersive(enabled) {
  drawer(false);
  document.body.classList.toggle('immersive', enabled);
  $('immerse').textContent = enabled ? '退出沉浸' : '沉浸模式';
  $('immerse').setAttribute('aria-pressed', String(enabled));
  wakeChrome();
}
$('immerse').onclick = () => {
  const enabled = !document.body.classList.contains('immersive');
  setImmersive(enabled);
  if (enabled) $('scroll').focus({preventScroll:true});
};
$('fullscreen').onclick = async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen();
    else {
      setImmersive(true);
      await document.documentElement.requestFullscreen();
      $('scroll').focus({preventScroll:true});
    }
  } catch (error) { notice('浏览器未允许全屏，可使用沉浸模式。'); }
};
document.addEventListener('fullscreenchange', () => {
  const full = !!document.fullscreenElement;
  $('fullscreen').textContent = full ? '退出全屏' : '全屏';
  $('fullscreen').setAttribute('aria-label', full ? '退出全屏' : '进入全屏');
  if (wasFullscreen && !full) setImmersive(false);
  wasFullscreen = full;
});
document.addEventListener('keydown', event => {
  if (event.defaultPrevented || event.target.matches('input, textarea, select') || $('config').open) return;
  if (event.key === 'Escape' && !document.fullscreenElement) setImmersive(false);
  if (event.code === 'Space' && !event.target.matches('button')) {
    event.preventDefault();
    if (run && ['播放中','已暂停'].includes(status)) pause();
    else if (!loading) start();
  }
  wakeChrome();
});
document.addEventListener('pointermove', wakeChrome, {passive:true});
document.addEventListener('pointerdown', wakeChrome, {passive:true});
document.addEventListener('focusin', wakeChrome);
function initStage() {
  try {
    pianoStage = createPianoStage($('sceneHost'), () => ({
      score: run?.score || library[selected]?.score || null,
      time: run ? position() : 0,
      playing: status === '播放中', sounding: ['播放中', '已暂停'].includes(status), hasRun: !!run, loading,
    }), number => $('k' + number).click());
    $('scroll').classList.add('has-webgl');
  } catch (error) {
    $('sceneHost').hidden = true;
    $('stageHint').hidden = true;
    notice('三维舞台初始化失败，已启用原键盘视图。' + error.message);
  }
}

renderLibrary();updateUI();initStage();window.pianoStandalone={parseScore,micros,whites,get stage(){return pianoStage;},get running(){return status==='播放中';},get analyser(){return analyser;},get context(){return ctx;},get state(){return status;},get position(){return position();},get voices(){return voices.length;},get tasks(){return {timer,frame,loading};}};

