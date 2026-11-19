// Integration check against the user's local ideal simulator; no hardware.
const {chromium}=require('C:/Users/maker/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const fs=require('fs');
const path=require('path');
const assert=require('assert');
const output=path.resolve(__dirname,'../logs/debugger-browser');
fs.mkdirSync(output,{recursive:true});
(async()=>{
 const browser=await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1120},deviceScaleFactor:1});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('http://127.0.0.1:8089',{waitUntil:'domcontentloaded'});
  await page.locator('#connection.online').waitFor({timeout:45000});
  await page.locator('#keyMode').selectOption('position');
  await page.waitForFunction(()=>document.querySelectorAll('.key:not(:disabled)').length===30,{},{timeout:45000});
  assert.equal(await page.locator('.key').count(),52);
  for(const key of [1,4,20,21,22,23])assert(await page.locator(`[data-key="${key}"]`).isDisabled());
  const table=await (await page.request.get('http://127.0.0.1:8089/api/table')).json();
  const state=async()=>await (await page.request.get('http://127.0.0.1:8089/api/state')).json();
  const initial=await state();
  const outcomes=[];
  assert.equal(await page.locator('#steps').inputValue(),'120');
  assert.equal(await page.locator('#velocity').inputValue(),'2');
  assert.equal(await page.locator('#acceleration').inputValue(),'20');
  await page.locator('#slowPreset').click();
  assert.equal(await page.locator('#velocity').inputValue(),'0.2');
  await page.locator('#fastPreset').click();
  await page.locator('#acceleration').fill('0');
  assert(await page.locator('[data-key="25"]').isDisabled());
  await page.locator('#fastPreset').click();
  // An insufficient count must fail without moving either arm.
  await page.locator('#steps').fill('1');
  await page.locator('[data-key="25"]').click();
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('至少需要'),{},{timeout:30000});
  assert.deepEqual((await state()).q14, initial.q14);
  for(const [key,side,steps] of [[25,'right',8],[18,'left',8]]){
   const before=await state();
   await page.locator('#steps').fill(String(steps));
   assert((await page.locator('#timingPreview').innerText()).includes((steps/60).toFixed(3)+' s'));
   await page.locator(`[data-key="${key}"]`).click();
   await page.waitForFunction(k=>document.querySelector('#target').textContent.includes(`第 ${k} 号白键`),key,{timeout:30000});
   await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('动作已完成'),{},{timeout:120000});
   const after=await state(),other=side==='right'?'left':'right';
   assert.equal(after.interpolation_steps,steps);
   assert.equal(after.completed_steps,steps);
   assert.equal(after.duration_s,steps/60);
   const goal=table.rows.find(r=>r.arm===side&&r.key===key).q;
   assert(after.arms[side].q.every((v,i)=>Math.abs(v-goal[i])<1e-9));
   assert(after.arms[other].q.every((v,i)=>Math.abs(v-before.arms[other].q[i])<1e-9));
   outcomes.push({key,side,steps,duration_s:after.duration_s,state:after.state,other_arm_held:true});
   console.log(JSON.stringify(outcomes.at(-1)));
  }
  await page.locator('#steps').fill('60');
  await page.locator('#center').click();
  await page.waitForFunction(()=>document.querySelector('#target').textContent==='双手中点',{},{timeout:30000});
  await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('动作已完成'),{},{timeout:120000});
  const final=await state();
  assert.equal(final.completed_steps,60);
  assert.equal(final.duration_s,1);
  for(const side of ['left','right'])assert(final.arms[side].q.every((v,i)=>Math.abs(v-table.calibration.center_q_rad[side][i])<1e-9));
  await page.waitForFunction(()=>document.querySelector('#viewerStatus').textContent.includes('实时模型'),{},{timeout:45000});
  await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
  const download=await page.request.get('http://127.0.0.1:8089/points.csv');assert(download.ok());assert((await download.body()).length>1000);
  await page.setViewportSize({width:900,height:1000});
  await page.screenshot({path:path.join(output,'narrow.png'),fullPage:true});
  assert.deepEqual(errors,[]);
  const report={outcomes,center_restored:true,enabled_keys:30,page_errors:errors,initial_q14:initial.q14,final_q14:final.q14};
  fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
  console.log('BROWSER_CHECK_PASSED');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
