// Run against an isolated headless simulation and web service, never hardware.
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({headless:true,channel:process.env.OPENARM_BROWSER_CHANNEL||'msedge'});
 const page=await browser.newPage({viewport:{width:1536,height:1000}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 try{
 await page.goto(process.env.OPENARM_WEB_URL);
 await page.waitForFunction(()=>document.querySelector('#connection').textContent.includes('已连接'));
 await page.waitForFunction(()=>document.querySelector('#viewerStatus').textContent.includes('实时模型'),null,{timeout:60000});
 const mathOK=await page.evaluate(async()=>{
   const {rpyToQuaternion:q,quaternionToRPY:r}=await import('/pose_math.js');
   const yaw=q([0,0,90]);if(Math.abs(yaw[0]-Math.SQRT1_2)>1e-10||Math.abs(yaw[3]-Math.SQRT1_2)>1e-10)return false;
   for(const x of [-170,-45,0,30,170])for(const y of [-110,-90,-89.9999,0,60,90,110])for(const z of [-120,0,55]){
    const a=q([x,y,z]),b=q(r(a));if(1-Math.abs(a.reduce((s,v,i)=>s+v*b[i],0))>1e-9)return false;
   }return true;
 });assert(mathOK,'RPY/quaternion roundtrip including singularities');
 await page.screenshot({path:'logs/web-controller-desktop.png',fullPage:true});
 const boxes=await page.evaluate(()=>Object.fromEntries(['visualPanel','ikPanel','feedbackPanel','jointPanel'].map(id=>{const r=document.getElementById(id).getBoundingClientRect();return [id,{x:r.x,y:r.y}]})));
 assert(boxes.visualPanel.x<boxes.ikPanel.x && boxes.visualPanel.y<boxes.feedbackPanel.y);assert(boxes.feedbackPanel.x<boxes.jointPanel.x && boxes.ikPanel.y<boxes.jointPanel.y);
 assert.equal(await page.locator('#fk-left-5').count(),1);
 await page.locator('#q-right-0').fill('5');await page.locator('#q-left-0').fill('-5');
 await page.locator('#moveJoints').click();
 await page.waitForFunction(()=>document.querySelector('#state').textContent==='执行中');
 await page.locator('#pause').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='已暂停');
 await page.locator('#resume').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='已完成');
 assert.match(await page.locator('#actual-right-0').textContent(),/5.00/);
 assert.match(await page.locator('#actual-left-0').textContent(),/-5.00/);
 await page.locator('#fill').click();
 await page.locator('#preview').click();await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('IK 预览成功'));
 await page.locator('#movePose').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='已完成');
 await page.locator('#home').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='执行中');
 await page.locator('#stop').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='已停止');
 await page.locator('#arm').selectOption('left');
 assert(await page.locator('#q-right-0').isDisabled());
 await page.locator('[aria-label="左臂 J1 增加"]').click();await page.waitForFunction(()=>document.querySelector('#state').textContent==='已完成');
 await page.locator('#q-left-0').fill('999');await page.locator('#moveJoints').click();
 await page.waitForFunction(()=>document.querySelector('#notice').textContent.includes('超出模型限位'));
 assert.equal(await page.locator('#scenes').count(),0);
 await page.setViewportSize({width:390,height:844});await page.screenshot({path:'logs/web-controller-mobile.png',fullPage:true});
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 await page.route('**/api/state',route=>route.abort());await page.waitForFunction(()=>document.querySelector('#state').textContent==='反馈断开');
 assert(await page.locator('#moveJoints').isDisabled());
 assert.deepEqual(errors,[]);console.log('PASS: dual move, pause/resume, IK, pose move, home/stop, left jog, limits, 3D model, four quadrants, RPY IK, disconnect, responsive layout');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
