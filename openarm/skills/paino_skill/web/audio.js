// User WAV samples: tone (1).wav = A0 (MIDI 21), tone (88).wav = C8.
export class PianoAudio {
  constructor(){this.context=null;this.voices=new Map();this.allVoices=new Set();this.cache=new Map();this.loading=new Map();this.pinned=new Set();this.enabled=false;this.played=0;this.volume=.5;}
  async enable(){
    if(!this.context){
      const Audio=window.AudioContext||window.webkitAudioContext;
      if(!Audio)throw Error('当前浏览器不支持音频，请使用 Chrome 或 Edge');
      this.context=new Audio({latencyHint:'interactive'});this.master=this.context.createGain();this.master.gain.value=this.volume*.35;
      const limiter=this.context.createDynamicsCompressor();limiter.threshold.value=-12;limiter.ratio.value=8;
      this.analyser=this.context.createAnalyser();this.master.connect(limiter);limiter.connect(this.analyser);this.analyser.connect(this.context.destination);
    }
    await this.context.resume();this.enabled=this.context.state==='running';
    if(!this.enabled)throw Error('请点击“开启声音”允许浏览器播放');
  }
  setVolume(value){this.volume=value;if(this.master)this.master.gain.setTargetAtTime(value*.35,this.context.currentTime,.02);}
  async prepare(midi){
    if(!Number.isInteger(midi)||midi<21||midi>108)throw Error('音符必须为 A0–C8');
    if(this.cache.has(midi)){const b=this.cache.get(midi);this.cache.delete(midi);this.cache.set(midi,b);return b;}
    if(this.loading.has(midi))return this.loading.get(midi);
    const task=(async()=>{
      const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),15000);
      try{
        const response=await fetch(`/samples/${midi}.wav`,{signal:controller.signal});
        if(!response.ok)throw Error(`无法读取 tone (${midi-20}).wav`);
        const buffer=await this.context.decodeAudioData(await response.arrayBuffer());
        this.cache.set(midi,buffer);
        this.prune();
        return buffer;
      }finally{clearTimeout(timer);}
    })();
    this.loading.set(midi,task);
    try{return await task;}finally{this.loading.delete(midi);}
  }
  prune(){for(const midi of this.cache.keys()){if(this.cache.size<=24)break;if(!this.pinned.has(midi))this.cache.delete(midi);}}
  async pinSamples(midis){
    this.pinned=new Set(midis);
    try{for(let i=0;i<midis.length;i+=4)await Promise.all(midis.slice(i,i+4).map(m=>this.prepare(m)));}
    catch(e){this.releaseSamples();throw e;}
  }
  releaseSamples(){this.pinned.clear();this.prune();}
  async on(midi,velocity=0.7,id=`contact:${midi}`,when){
    if(!this.enabled||this.context.state!=='running')return;
    this.off(id);if(this.voices.size>=32)this.off(this.voices.keys().next().value);
    // Register before loading so release/cancel cannot produce a delayed note.
    const voice={id,source:null,gain:null};this.voices.set(id,voice);this.allVoices.add(voice);
    try{
      let buffer=this.cache.get(midi);if(!buffer)buffer=await this.prepare(midi);
      if(this.voices.get(id)!==voice)return;
      const source=this.context.createBufferSource(),gain=this.context.createGain();
      source.buffer=buffer;gain.gain.value=Math.max(.05,Math.min(1,velocity));source.connect(gain);gain.connect(this.master);
      const requested=when??this.context.currentTime,started=Math.max(this.context.currentTime,requested);
      Object.assign(voice,{source,gain,requested,started});this.played++;
      source.onended=()=>{source.disconnect();gain.disconnect();this.allVoices.delete(voice);if(this.voices.get(id)===voice)this.voices.delete(id);};source.start(started);
    }catch(error){
      if(this.voices.get(id)!==voice)return;
      this.voices.delete(id);this.allVoices.delete(voice);window.dispatchEvent(new CustomEvent('piano-audio-error',{detail:error.message}));
    }
  }
  off(id,when){const v=this.voices.get(id);if(!v)return;this.voices.delete(id);if(!v.source){this.allVoices.delete(v);return;}const now=this.context.currentTime,t=when===undefined?now:Math.max(now,when+v.started-v.requested);v.gain.gain.setTargetAtTime(0,t,.04);v.source.stop(t+.25);}
  silence(){for(const id of [...this.voices.keys()])this.off(id);for(const v of this.allVoices){if(v.source){const t=this.context.currentTime;v.gain.gain.cancelScheduledValues(t);v.gain.gain.setTargetAtTime(0,t,.01);v.source.stop(t+.06);}}}
  silenceContact(){for(const id of [...this.voices.keys()])if(id.startsWith('contact:'))this.off(id);for(const v of this.allVoices){if(v.id.startsWith('contact:')&&v.source){const t=this.context.currentTime;v.gain.gain.cancelScheduledValues(t);v.gain.gain.setTargetAtTime(0,t,.01);v.source.stop(t+.06);}}}
}
