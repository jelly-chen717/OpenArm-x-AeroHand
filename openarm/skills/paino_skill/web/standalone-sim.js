// Appended to the standalone module only when served by the simulation controller.
document.title = '钢琴时光 · 仿真演奏';
const offlineUpdateUI = updateUI;
const offlineFinish = finish;
const offlineCancelLoading = cancelLoading;
let simOwned = false;
let simScoreToken = null;
let simStopping = false;
let simCursor = 0;
let simSession = null;
let simElapsed = 0;
let simOnline = false;
let simPollVersion = 0;
let simAudioOrigin = 0;
let simControlPending = false;
let rateUpdateRunning = false;
let requestedRate = 1;
const contactVoices = new Map();
const allContactVoices = new Set();
const heldAudio = new Map();
const pausedOffsets = new Map();

prepare = async function (midi) {
  if (buffers.has(midi)) return buffers.get(midi);
  const response = await fetch('/samples/' + midi + '.wav');
  if (!response.ok) throw Error('无法读取琴键采样 MIDI ' + midi);
  const buffer = await ctx.decodeAudioData(await response.arrayBuffer());
  buffers.set(midi, buffer);
  return buffer;
};

async function simRequest(path, body, timeout = 6000) {
  const controller = new AbortController();
  const deadline = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(path, {
      method: body ? 'POST' : 'GET',
      headers: body ? {'Content-Type': 'application/json'} : {},
      body: body ? JSON.stringify(body) : undefined,
      signal: controller.signal,
      cache: 'no-store',
    });
    const result = await response.json();
    if (!response.ok) throw Error(result.error || '仿真请求失败');
    return result;
  } finally {
    clearTimeout(deadline);
  }
}

function releaseContact(midi, when) {
  heldAudio.delete(midi);
  const voice = contactVoices.get(midi);
  if (!voice) return;
  contactVoices.delete(midi);
  const at = when === undefined ? ctx.currentTime :
    Math.max(ctx.currentTime, when + voice.started - voice.requested);
  voice.gain.gain.setTargetAtTime(0, at, .02);
  voice.source.stop(at + .12);
}

function silenceContacts() {
  contactVoices.clear();
  for (const voice of allContactVoices) {
    voice.gain.gain.cancelScheduledValues(ctx.currentTime);
    voice.gain.gain.setTargetAtTime(0, ctx.currentTime, .01);
    voice.source.stop(ctx.currentTime + .06);
  }
}

function playContact(event) {
  releaseContact(event.midi);
  const buffer = buffers.get(event.midi);
  if (!buffer || ctx?.state !== 'running') return;
  const source = ctx.createBufferSource();
  const gain = ctx.createGain();
  source.buffer = buffer;
  gain.gain.value = Math.max(.05, Math.min(1, event.velocity || .7));
  source.connect(gain);
  gain.connect(master);
  const requested = simAudioOrigin + event.time_s;
  const started = Math.max(ctx.currentTime, requested);
  const offset = Math.min(event.offset || 0, buffer.duration);
  const voice = {source, gain, requested, started};
  heldAudio.set(event.midi, {started, offset});
  contactVoices.set(event.midi, voice);
  allContactVoices.add(voice);
  source.onended = () => {
    source.disconnect();
    gain.disconnect();
    allContactVoices.delete(voice);
    if (contactVoices.get(event.midi) === voice) contactVoices.delete(event.midi);
  };
  source.start(started, offset);
}

position = function () {
  return run ? Math.min(run.score.duration, simElapsed) : 0;
};

updateUI = function () {
  offlineUpdateUI();
  $('pause').disabled = !simOwned || status !== '播放中' || simControlPending || rateUpdateRunning || simStopping;
  $('pause').title = '暂停机器人、进度与声音';
  $('speed').disabled = loading || simStopping || simControlPending || !simOnline;
  $('speed').title = '实时调整机器人与谱面播放速度';
  $('start').disabled ||= simStopping || simControlPending || rateUpdateRunning || !simOnline;
  $('restart').disabled ||= simStopping || loading || !simOnline;
  $('playPending').disabled ||= simStopping || loading || !simOnline;
  $('choose').disabled = simStopping || loading;
  $('stop').disabled ||= simStopping;
  $('state').textContent = simStopping ? '正在停止' : loading ? status :
    !simOnline ? '仿真未连接' : status === '待选曲' ? '仿真已连接' : status;
};

async function stopPreparedScore(token, reason) {
  if (token) await simRequest('/api/score', {action: 'stop', token, reason}, 15000);
}

async function stopSimulation(reason = 'user_stop') {
  if (simStopping) throw Error('正在停止仿真，请稍后');
  simStopping = true;
  simPollVersion++;
  updateUI();
  try {
    await stopPreparedScore(simScoreToken, reason);
    simScoreToken = null;
    simOwned = false;
  } finally {
    simStopping = false;
    silenceContacts();
    heldAudio.clear();
    pausedOffsets.clear();
    updateUI();
  }
}

finish = function (message = '演奏已结束') {
  if (loading) { cancelLoading(); return; }
  offlineFinish(message);
  silenceContacts();
  heldAudio.clear();
  pausedOffsets.clear();
  if (simOwned && message === '演奏已结束') {
    stopSimulation().then(() => notice('已结束演奏。'),
      error => notice('停止结果未知：' + error.message));
  }
};

cancelLoading = function () {
  offlineCancelLoading();
  stopSimulation('cancel_start').catch(error => notice('取消结果未知：' + error.message));
};

function rememberContactOffsets() {
  if (status !== '播放中') return;
  pausedOffsets.clear();
  for (const [midi, voice] of heldAudio) {
    pausedOffsets.set(midi, voice.offset + Math.max(0, ctx.currentTime - voice.started));
  }
}

function applyTransport(snapshot) {
  simPollVersion++;
  simElapsed = snapshot.score.elapsed_us;
  simCursor = snapshot.last_event_id;
  rate = snapshot.score.rate;
  simAudioOrigin = ctx.currentTime + .08 - snapshot.time_s;
  rememberContactOffsets();
  silenceContacts();
  heldAudio.clear();
  cancelAnimationFrame(frame);
  if (snapshot.score.status === 'PAUSED') {
    status = '已暂停';
  } else if (snapshot.score.status === 'RUNNING') {
    status = '播放中';
    for (const midi of snapshot.pressed) {
      playContact({midi, time_s: snapshot.time_s, offset: pausedOffsets.get(midi) || 0});
    }
  } else {
    simOwned = false;
    simScoreToken = null;
    finish(snapshot.score.status === 'COMPLETED' ? '演奏完成' : snapshot.score.status === 'ERROR' ? '出错' : '演奏已结束');
  }
  draw();
  updateUI();
}

pause = async function () {
  if (!simOwned || simControlPending || rateUpdateRunning || simStopping) return;
  const action = status === '已暂停' ? 'resume' : 'pause';
  const token = loadToken;
  simControlPending = true;
  simPollVersion++;
  updateUI();
  try {
    if (action === 'resume') await audio();
    const result = await simRequest('/api/score', {action}, 15000);
    if (token !== loadToken || !simOwned || simStopping) return;
    applyTransport(result.piano);
    notice(action === 'pause' ? '仿真已暂停，点击“继续”恢复。' : '仿真已继续');
  } catch (error) {
    notice('暂停或继续结果未知：' + error.message);
  } finally {
    simControlPending = false;
    updateUI();
  }
};

start = async function (replay = false) {
  if (loading || simStopping) return;
  const song = replay && run ? run.song : library[selected];
  if (!song || song.error) {
    notice(song?.error || '请先选择歌曲');
    return;
  }
  const token = ++loadToken;
  simPollVersion++;
  loading = true;
  status = '正在预检';
  updateUI();
  try {
    // Resume audio in the user gesture before any network request.
    await audio();
    if (simOwned) await stopSimulation();
    if (token !== loadToken) return;
    cleanup();
    run = null;
    simElapsed = 0;
    notice('正在检查整曲手位与触键…');
    const ready = await simRequest('/api/score', {
      action: 'prepare', text: song.text, finger_count: 4, automatic_motion: true,
    }, 125000);
    if (token !== loadToken) {
      await stopPreparedScore(ready.token, 'cancel_start');
      return;
    }
    simScoreToken = ready.token;
    status = '准备音色';
    updateUI();
    // Any key may sound from contact while the hand moves.
    for (let i = 0; i < 88; i += 4) {
      await Promise.all(Array.from({length: Math.min(4, 88 - i)}, (_, j) => prepare(21 + i + j)));
      if (token !== loadToken) return;
    }
    if (document.hidden) throw Error('请返回页面后重新开始');
    const started = await simRequest('/api/score', {action: 'start', token: ready.token, rate}, 15000);
    if (token !== loadToken) return;
    simPollVersion++;
    simOwned = true;
    simOnline = true;
    simSession = started.piano.session;
    simCursor = started.piano.last_event_id;
    simAudioOrigin = ctx.currentTime + .15 - started.piano.time_s;
    run = {song, name: song.name, score: song.score, offset: 0, anchor: ctx.currentTime, next: 0};
    rate = started.piano.score.rate;
    requestedRate = rate;
    $('speed').value = String(rate);
    $('speedValue').value = rate.toFixed(1) + '×';
    status = '播放中';
    loading = false;
    notice('仿真已开始 · 声音由模型触键事件触发');
    draw();
    updateUI();
  } catch (error) {
    if (token !== loadToken) return;
    loading = false;
    status = '出错';
    cleanup();
    silenceContacts();
    // A timed-out start may have reached the server.
    try { await stopSimulation('cancel_start'); } catch { /* Display the initiating failure below. */ }
    notice('无法开始：' + error.message);
    updateUI();
  }
};

// The original handler stored the offline function itself.
$('pause').onclick = pause;
$('cancel').onclick = cancelLoading;
$('speed').oninput = async () => {
  requestedRate = Number($('speed').value);
  $('speedValue').value = requestedRate.toFixed(1) + '×';
  $('speed').setAttribute('aria-valuetext', requestedRate.toFixed(1) + '倍');
  if (!simOwned) { rate = requestedRate; updateUI(); return; }
  if (rateUpdateRunning || simControlPending || simStopping) return;
  const token = loadToken;
  rateUpdateRunning = true;
  simPollVersion++;
  updateUI();
  try {
    while (simOwned && !simStopping && token === loadToken) {
      const target = requestedRate;
      const result = await simRequest('/api/score', {action: 'rate', rate: target}, 15000);
      if (token !== loadToken || !simOwned || simStopping) break;
      applyTransport(result.piano);
      notice('演奏倍速 ' + rate.toFixed(1) + '×');
      if (target === requestedRate) break;
    }
  } catch (error) {
    $('speed').value = String(rate);
    $('speedValue').value = rate.toFixed(1) + '×';
    notice('倍速调整失败：' + error.message);
  } finally {
    rateUpdateRunning = false;
    updateUI();
  }
};

async function pollSimulation() {
  const version = simPollVersion;
  try {
    const snapshot = await simRequest('/api/piano?after=' + simCursor);
    if (version !== simPollVersion || simControlPending || rateUpdateRunning || simStopping) return;
    const reconnected = !simOnline;
    simOnline = true;
    if ($('notice').textContent === '正在连接 MuJoCo 仿真…') notice();
    if (simSession !== snapshot.session) {
      const restarted = simSession !== null;
      simSession = snapshot.session;
      simCursor = snapshot.last_event_id;
      if (restarted && simOwned) {
        simOwned = false;
        simScoreToken = null;
        finish('出错');
        notice('仿真已重启，请重新开始');
      }
    } else if (reconnected && simOwned && snapshot.score.status === 'RUNNING' &&
      snapshot.events.length && (snapshot.events[0].id > simCursor + 1 ||
        simAudioOrigin + snapshot.events[0].time_s < ctx.currentTime)) {
      // Restore the current keys when the recovered events can no longer play on time.
      applyTransport(snapshot);
    } else {
      for (const event of snapshot.events) {
        if (event.id <= simCursor) continue;
        simCursor = event.id;
        if (simOwned) {
          if (event.type === 'on') playContact(event);
          else releaseContact(event.midi, simAudioOrigin + event.time_s);
        }
      }
      simCursor = Math.max(simCursor, snapshot.last_event_id);
    }
    if (simOwned && !loading) {
      simElapsed = snapshot.score.elapsed_us;
      rate = snapshot.score.rate;
      if (snapshot.score.status === 'PAUSED' && status !== '已暂停' ||
          snapshot.score.status === 'RUNNING' && status === '已暂停') applyTransport(snapshot);
      if (['COMPLETED', 'STOPPED', 'ERROR'].includes(snapshot.score.status)) {
        simOwned = false;
        simScoreToken = null;
        finish(snapshot.score.status === 'COMPLETED' ? '演奏完成' :
          snapshot.score.status === 'ERROR' ? '出错' : '演奏已结束');
        notice(snapshot.score.error || (snapshot.score.status === 'COMPLETED' ? '仿真演奏完成' : '仿真演奏已结束'));
      } else if (reconnected) {
        notice('已恢复仿真连接。');
      }
    }
  } catch (error) {
    if (version !== simPollVersion || simControlPending || rateUpdateRunning || simStopping) return;
    simOnline = false;
    if (simOwned) {
      notice('暂时读不到仿真状态，正在重新连接…');
    }
  } finally {
    updateUI();
    setTimeout(pollSimulation, simOnline || simOwned ? 40 : 250);
  }
}

window.addEventListener('pagehide', () => {
  if (simScoreToken) {
    const stop = {action: 'stop', token: simScoreToken, reason: 'page_closed'};
    navigator.sendBeacon('/api/score', new Blob([JSON.stringify(stop)], {type: 'application/json'}));
  }
  silenceContacts();
}, {capture: true});
document.addEventListener('visibilitychange', event => {
  event.stopImmediatePropagation();
  if (!document.hidden) return;
  if (loading) cancelLoading();
  else if (simOwned && status === '播放中') pause();
}, {capture: true});
notice('正在连接 MuJoCo 仿真…');
updateUI();
pollSimulation();
