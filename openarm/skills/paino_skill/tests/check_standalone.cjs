const { chromium } = require('C:/Users/maker/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert = require('node:assert/strict');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
(async () => {
  const browser = await chromium.launch({executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe',headless:true});
  try {
    const page = await browser.newPage({ viewport: {width:1440,height:900} });
    const errors=[],network=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('request',r=>{if(/^https?:/.test(r.url()))network.push(r.url());});
    await page.goto(pathToFileURL(path.resolve(__dirname,'../piano_standalone.html')).href, {timeout:90000});
    const results=await page.evaluate(()=>{
      const p=window.pianoStandalone, good='left_arm_move(17, .2)\nright_arm_move(24, .2)\nsleep(.2)\nhand(D,1x)\nsleep(.1)\nhand(U,1x)';
      const score=p.parseScore(good);
      const bad=['hand(D,5)','sleep(NaN)','sleep(-1)','sleep(.0000001)','alert(1)',
        'right_arm_move(50,1)', 'right_arm_move(24,1)\nhand(D,7)',
        'right_arm_move(24,1)\nright_arm_move(25,1)',
        'right_arm_move(24,1)\nsleep(1)\nhand(D,7)',
        'right_arm_move(24,1)\nsleep(1)\nhand(D,77)',
        'right_arm_move(24,1)\nsleep(1)\nhand(D,7)\nright_arm_move(25,1)',
        'right_arm_move(24,1)\nsleep(1)\nhand(D,7)\nhand(D,7)'];
      return {duration:score.duration,starts:score.actions.slice(0,2).map(a=>a.time),keys:score.notes.map(n=>n.key),
        exact:p.micros('2.692309'),bad:bad.map(s=>{try{p.parseScore(s);return false;}catch{return true;}})};
    });
    assert.equal(results.duration,300000);assert.deepEqual(results.starts,[0,0]);assert.deepEqual(results.keys,[17,27]);assert.equal(results.exact,2692309);assert(results.bad.every(Boolean));
    await page.locator('#start').click();
    await page.waitForFunction(()=>document.querySelectorAll('.key.active').length===2);
    await page.waitForFunction(()=>{const a=window.pianoStandalone.analyser,b=new Float32Array(a.fftSize);a.getFloatTimeDomainData(b);return b.some(x=>Math.abs(x)>.001);});
    assert.equal(await page.locator('#left').evaluate(e=>e.offsetTop),await page.locator('#right').evaluate(e=>e.offsetTop));
    await page.screenshot({path:path.resolve(__dirname,'../logs/standalone-playing.png')});
    await page.locator('#doneScreen').waitFor({state:'visible'});
    assert.equal(await page.locator('#result').textContent(),'演奏完成');
    await page.locator('#done').click();
    await page.locator('#start').click();await page.locator('#playScreen').waitFor({state:'visible'});
    await page.locator('#stop').click();assert.equal(await page.evaluate(()=>pianoStandalone.running),false);
    await page.locator('#done').click();await page.locator('#start').click();await page.locator('#playScreen').waitFor({state:'visible'});
    await page.evaluate(()=>{Object.defineProperty(document,'hidden',{configurable:true,value:true});document.dispatchEvent(new Event('visibilitychange'));});
    assert.equal(await page.evaluate(()=>pianoStandalone.running),false);
    await page.evaluate(()=>{delete document.hidden;});await page.locator('#done').click();
    await page.locator('body').click({position:{x:3,y:3}});await page.keyboard.type('config');await page.locator('#config').waitFor({state:'visible'});
    await page.locator('#text').fill('hand(D,5)');await page.locator('#apply').click();assert.match(await page.locator('#configError').textContent(),/第 1 行/);await page.locator('#close').click();
    await page.locator('#files').setInputFiles({name:'导入测试.txt',mimeType:'text/plain',buffer:Buffer.from('right_arm_move(24,.1)\nsleep(.1)\nhand(D,7)\nsleep(.1)\nhand(U,7)')});
    await page.waitForFunction(()=>document.querySelector('#songs').selectedOptions[0].text==='导入测试');
    await page.locator('#start').click();await page.locator('#doneScreen').waitFor({state:'visible'});
    assert.deepEqual(network,[]);assert.deepEqual(errors,[]);
    console.log(JSON.stringify({parser:results,offline:true,sampleAudio:true,alignedHands:true,replay:true,stop:true,backgroundStop:true,config:true,import:true,pageErrors:errors}));
  } finally {await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});
