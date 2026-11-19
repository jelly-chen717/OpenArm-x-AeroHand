const {chromium}=require('C:/Users/maker/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert=require('assert'),fs=require('fs'),path=require('path');
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
 try{
  const page=await browser.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('http://127.0.0.1:8089');
  await page.locator('#sound').click();await page.waitForFunction(()=>window.__pianoAudio.enabled);
  const samples=await page.evaluate(async()=>{
   const a=window.__pianoAudio,result=[];
   for(const midi of [21,60,69,108]){
    const b=await a.prepare(midi),r=await fetch(`/samples/${midi}.wav`),v=new DataView(await r.arrayBuffer());
    let offset=12;while(offset<v.byteLength&&String.fromCharCode(...[0,1,2,3].map(i=>v.getUint8(offset+i)))!=='data')offset+=8+v.getUint32(offset+4,true)+(v.getUint32(offset+4,true)%2);
    offset+=8;let error=0;
    // Verify original PCM independently of output device resampling.
    const native=new OfflineAudioContext(2,1,44100),source=await native.decodeAudioData(await(await fetch(`/samples/${midi}.wav`)).arrayBuffer());
    for(let c=0;c<2;c++)for(let i=4410;i<4510;i++)error=Math.max(error,Math.abs(source.getChannelData(c)[i]-v.getInt16(offset+(i*2+c)*2,true)/32768));
    result.push({midi,channels:b.numberOfChannels,duration:b.duration,pcm_error:error});
   }
   return result;
  });
  console.log(JSON.stringify(samples));
  // Chrome's PCM normalization may differ by less than one 16-bit quantum.
  for(const s of samples){assert.equal(s.channels,2);assert(s.duration>=6);assert(s.pcm_error<1/32768);}
  await page.locator('#keyMode').selectOption('preview');
  const key=page.locator('[data-midi="69"]');await key.scrollIntoViewIfNeeded();const box=await key.boundingBox();
  await page.mouse.move(box.x+box.width/2,box.y+box.height-20);await page.mouse.down();
  await page.waitForFunction(()=>{const a=window.__pianoAudio,v=new Float32Array(a.analyser.fftSize);a.analyser.getFloatTimeDomainData(v);return v.some(x=>Math.abs(x)>.001);});
  const rms=await page.evaluate(()=>{const a=window.__pianoAudio,v=new Float32Array(a.analyser.fftSize);a.analyser.getFloatTimeDomainData(v);return Math.sqrt(v.reduce((s,x)=>s+x*x,0)/v.length);});
  await page.mouse.up();await page.waitForFunction(()=>!window.__pianoAudio.voices.has('preview:69'));
  // Releasing while loading must not produce a delayed note.
  await page.route('**/samples/22.wav',async r=>{await new Promise(resolve=>setTimeout(resolve,350));await r.continue();});
  const cancelled=await page.evaluate(async()=>{const a=window.__pianoAudio,n=a.played,task=a.on(22,.7,'cancel-test');a.off('cancel-test');await task;return a.played===n&&!a.voices.has('cancel-test');});assert(cancelled);
  await page.route('**/samples/23.wav',r=>r.fulfill({status:404,body:'missing'}));
  const failed=await page.evaluate(async()=>{const a=window.__pianoAudio,n=a.played;await a.on(23,.7,'missing-test');return a.played===n&&!a.voices.has('missing-test');});assert(failed);
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('采样播放失败'));
  await page.locator('#keyMode').selectOption('play');
  await page.locator('#connection.online').waitFor({timeout:60000});
  const before=await page.evaluate(()=>window.__pianoAudio.played);
  await page.locator('[data-midi="60"]').click({position:{x:10,y:160}});
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('指尖触键与发声事件已确认'),{},{timeout:60000});
  assert((await page.evaluate(()=>window.__pianoAudio.played))>before);
  await page.locator('#center').click();await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('动作已完成'),{},{timeout:60000});
  assert.deepEqual(errors,[]);
  fs.writeFileSync(path.resolve(__dirname,'../logs/user-samples-browser.json'),JSON.stringify({samples,rms,cancelled_load_silent:cancelled,missing_sample_reported:failed,aero_C4_sample:true,page_errors:errors},null,2));
  console.log('USER_SAMPLES_BROWSER_PASSED',JSON.stringify({rms,samples}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
