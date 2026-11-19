// Real browser + local MuJoCo integration; pure software, no hardware.
const {chromium}=require('C:/Users/maker/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const fs=require('fs'),path=require('path'),assert=require('assert');
const output=path.resolve(__dirname,'../logs/performance-browser');
fs.mkdirSync(output,{recursive:true});
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1520,height:1120}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const piano=async()=>await(await page.request.get('http://127.0.0.1:8089/api/piano')).json();
  await page.goto('http://127.0.0.1:8089',{waitUntil:'domcontentloaded'});
  await page.locator('#connection.online').waitFor({timeout:60000});
  await page.waitForFunction(()=>document.querySelectorAll('[data-midi]').length===88);
  assert.equal(await page.locator('.key').count(),52);assert.equal(await page.locator('.black-key').count(),36);
  assert.equal(await page.locator('#steps').inputValue(),'120');
  await page.waitForFunction(()=>document.querySelector('#contactStatus').textContent.includes('监听中'));
  await page.locator('#sound').click();
  await page.waitForFunction(()=>window.__pianoAudio.enabled);
  await page.locator('#keyMode').selectOption('preview');
  const a4=page.locator('[data-midi="69"]');await a4.scrollIntoViewIfNeeded();
  const box=await a4.boundingBox();await page.mouse.move(box.x+box.width/2,box.y+box.height-20);await page.mouse.down();
  await page.waitForFunction(()=>window.__pianoAudio.voices.has('preview:69'));
  await page.waitForFunction(()=>{const a=window.__pianoAudio.analyser,v=new Float32Array(a.fftSize);a.getFloatTimeDomainData(v);return v.some(x=>Math.abs(x)>.001);},{},{timeout:10000});
  const rms=await page.evaluate(()=>{const a=window.__pianoAudio.analyser,v=new Float32Array(a.fftSize);a.getFloatTimeDomainData(v);return Math.sqrt(v.reduce((s,x)=>s+x*x,0)/v.length);});
  assert(rms>.001,`No audible signal: ${rms}`);await page.mouse.up();
  await page.waitForFunction(()=>!window.__pianoAudio.voices.has('preview:69'));
  await page.locator('#keyMode').selectOption('play');
  const outcomes=[];
  for(const [midi,side] of [[60,'right'],[55,'left'],[61,'right']]){
   const before=await piano(),played=await page.evaluate(()=>window.__pianoAudio.played);
   await page.locator(`[data-midi="${midi}"]`).click({position:{x:10,y:midi===61?50:160}});
   await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('指尖触键与发声事件已确认'),{},{timeout:60000});
   const after=await piano();assert.equal(after.sequence.midi,midi);assert.equal(after.sequence.phase,'done');
   const events=after.events.filter(e=>e.id>before.last_event_id&&e.midi===midi);
   assert(events.some(e=>e.type==='on'&&e.source==='aero_contact'&&e.hand===side));assert(events.some(e=>e.type==='off'));
   assert((await page.evaluate(()=>window.__pianoAudio.played))>played);
   outcomes.push({midi,side,events});console.log(JSON.stringify({midi,side,result:'contact and audio confirmed'}));
  }
  // The demonstration must complete all nine physical finger strikes.
  const beforeDemo=await piano();await page.locator('#demo').click();
  await page.waitForFunction(()=>document.querySelector('#demo').disabled);
  await page.waitForFunction(()=>!document.querySelector('#demo').disabled,{},{timeout:180000});
  const afterDemo=await piano(),notes=afterDemo.events.filter(e=>e.id>beforeDemo.last_event_id&&e.type==='on').map(e=>e.midi);
  assert.deepEqual(notes.slice(-9),[60,62,64,65,67,65,64,62,60]);assert.equal(afterDemo.sequence.phase,'done');
  const finger=page.locator('[data-finger="index"][data-hand="right"]');
  await finger.scrollIntoViewIfNeeded();const fbox=await finger.boundingBox();
  await page.mouse.move(fbox.x+fbox.width/2,fbox.y+fbox.height/2);await page.mouse.down();
  await page.waitForFunction(()=>document.querySelector('[data-midi="60"]').classList.contains('sounding'),{},{timeout:10000});
  await page.mouse.up();await page.waitForFunction(()=>!document.querySelector('[data-midi="60"]').classList.contains('sounding'));
  await page.locator('#keyMode').selectOption('position');
  assert.equal(await page.locator('.key:not(:disabled)').count(),30);
  await page.locator('#center').click();
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('动作已完成'),{},{timeout:60000});
  await page.locator('#keyMode').selectOption('play');
  await page.waitForFunction(()=>document.querySelector('#viewerStatus').textContent.includes('实时模型'),{},{timeout:60000});
  await page.evaluate(()=>window.scrollTo(0,0));await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
  await page.setViewportSize({width:900,height:1000});await page.screenshot({path:path.join(output,'narrow.png'),fullPage:true});
  assert.deepEqual(errors,[]);
  fs.writeFileSync(path.join(output,'report.json'),JSON.stringify({white:52,black:36,preview_rms:rms,outcomes,demo_notes:notes,incidental_transition_notes:notes.slice(0,-9),manual_press_release:true,csv_enabled:30,center_restored:true,page_errors:errors},null,2));
  console.log('PERFORMANCE_BROWSER_PASSED');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
