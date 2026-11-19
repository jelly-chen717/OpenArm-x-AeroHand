const {chromium}=require('C:/Users/maker/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert=require('assert'),fs=require('fs'),path=require('path');
const out=path.resolve(__dirname,'../logs/music-flow-browser');fs.mkdirSync(out,{recursive:true});
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
 const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[],requests=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('request',r=>requests.push(r.url()));
 const waitStatus=text=>page.waitForFunction(t=>document.querySelector('#scoreStatus').textContent.includes(t),text,{timeout:125000});
 const config=async()=>{await page.keyboard.type('config');await page.locator('#settings').waitFor({state:'visible'});};
 try{
  await page.goto('http://127.0.0.1:8090/');await page.locator('#connection.online').waitFor();await page.waitForFunction(()=>document.querySelector('#songSelect').options.length>1);
  assert.equal(await page.locator('#settings').isVisible(),false);assert(await page.locator('#scoreStart').isDisabled());
  assert.equal(await page.locator('.key:not(.black)').count(),52);assert.equal(await page.locator('.key.black').count(),36);
  const names=await page.locator('#songSelect option:not([value=""])').allTextContents();assert(names.includes('小苹果')&&names.includes('月亮代表我的心'));
  await page.locator('#songSelect').selectOption({label:'小苹果'});await page.waitForFunction(()=>!document.querySelector('#scoreStart').disabled);
  await page.screenshot({path:path.join(out,'selection.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(out,'mobile-selection.png'),fullPage:true});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));await page.setViewportSize({width:1440,height:1000});
  await config();await page.locator('#scoreVelocity').fill('2');await page.locator('#scoreAcceleration').fill('20');await page.locator('#scorePrepare').click();await waitStatus('第 408 行');
  await page.locator('#scoreVelocity').fill('2.5');await page.locator('#scoreAcceleration').fill('50');await page.locator('#scorePrepare').click();await waitStatus('预检通过');
  await page.locator('#closeSettings').click();await page.locator('#songSelect').selectOption({label:'月亮代表我的心'});await page.waitForFunction(()=>!document.querySelector('#scoreStart').disabled);
  await config();await page.locator('#scorePrepare').click();await waitStatus('预检通过');
  const short='left_arm_move(17,2)\nright_arm_move(28,1)\nsleep(1)\nhand(D,7)\nsleep(.3)\nhand(U,7)\nsleep(.7)\nhand(D,1x)\nsleep(.3)\nhand(U,1x)\nsleep(.7)';
  await page.locator('#scoreFile').setInputFiles({name:'三步演奏测试.txt',mimeType:'text/plain',buffer:Buffer.from(short)});await waitStatus('已解析');await page.locator('#closeSettings').click();
  await page.locator('#scoreStart').click();await page.locator('#playScreen').waitFor({state:'visible'});
  await page.waitForFunction(()=>document.querySelectorAll('.key.pressed-left,.key.pressed-right').length===2,{},{timeout:20000});
  const chord=await page.locator('.key.pressed-left,.key.pressed-right').evaluateAll(es=>es.map(e=>Number(e.dataset.key)));assert.deepEqual(chord,[17,31]);
  await page.screenshot({path:path.join(out,'playing.png'),fullPage:true});await page.locator('#completeScreen').waitFor({state:'visible',timeout:20000});
  assert.equal(await page.locator('#completeTitle').innerText(),'演奏完成');assert.equal(await page.evaluate(()=>window.__pianoAudio.played),3);
  await page.screenshot({path:path.join(out,'complete.png'),fullPage:true});await page.locator('#finishButton').click();await page.locator('#selectScreen').waitFor({state:'visible'});assert(await page.locator('#scoreStart').isDisabled());
  // A cancelled preflight may finish later, but it must never start motion.
  await page.locator('#songSelect').selectOption({label:'小苹果'});await page.waitForFunction(()=>!document.querySelector('#scoreStart').disabled);
  await page.route('**/api/score',async route=>{if(route.request().postDataJSON()?.action==='prepare')await new Promise(r=>setTimeout(r,400));await route.continue();});
  await page.locator('#scoreStart').click();await page.locator('#scoreStop').click();await page.locator('#completeScreen').waitFor({state:'visible'});assert.equal(await page.locator('#completeTitle').innerText(),'演奏已结束');
  await page.waitForTimeout(1000);const stopped=await(await page.request.get('http://127.0.0.1:8090/api/piano')).json();assert.notEqual(stopped.score.status,'RUNNING');assert.deepEqual(stopped.pressed,[]);await page.unroute('**/api/score');
  await page.locator('#finishButton').click();await page.locator('#songSelect').selectOption({label:'小苹果'});await page.waitForFunction(()=>!document.querySelector('#scoreStart').disabled);
  await page.locator('#scoreStart').click();await page.waitForFunction(async()=>(await(await fetch('/api/piano')).json()).score.status==='RUNNING',{},{timeout:125000});
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(out,'mobile-playing.png'),fullPage:true});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.locator('#scoreStop').click();await page.locator('#completeScreen').waitFor({state:'visible'});assert.equal(await page.locator('#completeTitle').innerText(),'演奏已结束');await page.locator('#finishButton').click();
  assert(!requests.some(u=>u.startsWith('http://127.0.0.1:8089/')));assert.deepEqual(errors,[]);
  fs.writeFileSync(path.join(out,'report.json'),JSON.stringify({songs:names,white:52,black:36,hidden_config:true,preflight_both:true,chord,audio_notes:3,completed_returns_to_selection:true,cancel_pending:true,stop_running:true,mobile_fits:true,page_errors:errors},null,2));console.log('MUSIC_FLOW_PASSED');
 }catch(e){await page.screenshot({path:path.join(out,'failure.png'),fullPage:true});console.error('UI:',await page.locator('#scoreStatus').innerText());throw e;}finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
