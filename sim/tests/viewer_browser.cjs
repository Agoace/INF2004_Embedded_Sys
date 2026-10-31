/* Optional regression/evidence tool. Runtime viewer has no Playwright dependency. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {pathToFileURL} = require('node:url');
const {chromium} = require(process.env.SIM_PLAYWRIGHT_MODULE || 'playwright');
const [html,tracePath,out,escapedHtml,gapHtml] = process.argv.slice(2);
(async () => {
  fs.mkdirSync(out,{recursive:true});
  const trace = JSON.parse(fs.readFileSync(tracePath,'utf8'));
  const browser = await chromium.launch({headless:true});
  try {
    const context = await browser.newContext({offline:true,viewport:{width:1440,height:1000}});
    const page = await context.newPage(), errors=[], network=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
    page.on('request',r=>{if(!r.url().startsWith('file:'))network.push(r.url());});
    await context.route('**/*',route=>route.request().url().startsWith('file:')?route.continue():route.abort());
    const standalone=path.join(out,'standalone.html');if(path.resolve(html)!==path.resolve(standalone))fs.copyFileSync(html,standalone);
    const openedUrl=pathToFileURL(path.resolve(standalone)).href;
    await page.goto(openedUrl);
    await page.waitForFunction(()=>window.simViewer && document.querySelector('canvas').width>300);
    const seek = t => page.evaluate(t=>window.simViewer.seek(t),t);
    const current = () => page.evaluate(()=>window.simViewer.getState());
    const steps = trace.firmware_observation.detour_timeline;
    assert.deepEqual(await page.evaluate(()=>simViewer.data.steps),steps);
    assert.deepEqual(await page.evaluate(()=>simViewer.data.console),trace.events.filter(e=>e.type==='text').map(e=>[e.t_us,e.text]));
    for(const step of steps){
      await seek(step.t_us-1);assert.notEqual((await current()).dt?.step,step.step);
      await seek(step.t_us);assert.equal((await current()).dt.step,step.step);
    }
    await seek(12013600);assert(!(await current()).console.includes('[detour] STRAIGHT at'));
    await seek(12020200);assert((await current()).console.includes('[detour] STRAIGHT at'));
    assert.equal(await page.locator('#assumptions li').count(),trace.header.assumed_values.length);
    await page.locator('#scrub').evaluate(el=>{el.value='7800000';el.dispatchEvent(new Event('input',{bubbles:true}));});
    assert.equal((await current()).t_us,7800000);
    await seek(7681750);
    let state=await current();
    const samples=trace.events.filter(e=>e.type==='world'),i=samples.findIndex(e=>e.t_us>7681750)-1;
    const a=samples[i],b=samples[i+1],f=(7681750-a.t_us)/(b.t_us-a.t_us);
    for(const [j,key] of ['x_mm','y_mm','heading_deg'].entries())assert(Math.abs(state.pose[j]-(a[key]+(b[key]-a[key])*f))<1e-8);
    assert.equal(state.sample[0],a.t_us);assert.deepEqual(state.sample[4],a.encoder_counts);
    assert.deepEqual(state.duties,[0,0]); // STOP command already applies between world samples.
    await seek(0);await page.locator('#next').click();assert.equal((await current()).t_us,steps[0].t_us);
    await page.locator('#next').click();assert.equal((await current()).t_us,steps[1].t_us);
    await page.locator('#prev').click();assert.equal((await current()).t_us,steps[0].t_us);
    for(const value of ['0.25','1','4','16'])await page.selectOption('#speed',value);
    await page.locator('#play').click();await page.waitForTimeout(100);await page.keyboard.press('Space');
    const paused=(await current()).t_us;assert(paused>steps[0].t_us);
    await page.waitForTimeout(50);assert.equal((await current()).t_us,paused);
    await page.keyboard.press('Space');await page.waitForTimeout(50);await page.keyboard.press('Space');
    assert((await current()).t_us>paused);
    const rect=await page.locator('#map').boundingBox();
    await page.mouse.move(rect.x+rect.width/2,rect.y+rect.height/2);
    const before=await page.evaluate(()=>simViewer.getCamera());await page.mouse.wheel(0,-200);await page.waitForTimeout(30);
    assert((await page.evaluate(()=>simViewer.getCamera())).scale>before.scale);
    await page.mouse.down();await page.mouse.move(rect.x+rect.width/2+60,rect.y+rect.height/2+30);await page.mouse.up();
    assert.notEqual((await page.evaluate(()=>simViewer.getCamera())).x,before.x);
    await page.locator('#fit').click();
    const screenshots=[];
    for(const [name,t] of [['stop',7800000],['scan',8600000],['first_straight',12020200],['heading_gap_21_55s',21550000],['failed',34810200]]){
      await seek(t);await page.locator('#focus').click();
      const target=path.join(out,name+'.png');await page.screenshot({path:target,fullPage:true});screenshots.push(target);
      if(name==='heading_gap_21_55s'){state=await current();assert(state.ghost);assert(Math.abs(state.pose[2]-state.ghost[2])>7);}
    }
    await seek(40000000);assert.equal((await current()).phase,'NOT_RUNNING');
    assert.equal((await current()).ghost,null);
    assert.equal(await page.locator('#console').textContent(),trace.events.filter(e=>e.type==='text').map(e=>e.text).join(''));
    if(escapedHtml){
      await page.goto(pathToFileURL(path.resolve(escapedHtml)).href);await page.waitForFunction(()=>window.simViewer);
      await page.evaluate(()=>simViewer.seek(simViewer.data.meta.end_us));
      assert((await page.locator('#console').textContent()).includes('</script><b>literal</b>'));
      assert.equal(await page.locator('#console b').count(),0);
      assert.equal(await page.evaluate(()=>window.viewerInjected),undefined);
      await page.screenshot({path:path.join(out,'escaped_text.png'),fullPage:true});
    }
    if(gapHtml){
      await page.goto(pathToFileURL(path.resolve(gapHtml)).href);await page.waitForFunction(()=>window.simViewer);
      await page.evaluate(()=>simViewer.seek(21550000));assert.equal((await current()).pose,null);
      assert((await page.locator('#gap').textContent()).includes('gap'));
    }
    assert.deepEqual(errors,[]);assert.deepEqual(network,[]);
    const result={browser:browser.version(),opened_url:openedUrl,file_url:true,standalone_copy_without_trace:!fs.existsSync(path.join(out,'trace.json')),offline:true,errors,network_requests:network,
      step_times:steps.map(e=>e.t_us),screenshots,checks:['data fidelity','event-time boundaries','pose-only interpolation','commands between snapshots','step controls','speed controls','space/play/pause','pan/zoom/fit','idle ghost hidden','console fidelity',...(escapedHtml?['escaping']:[]),...(gapHtml?['missing world gap']:[])]};
    fs.writeFileSync(path.join(out,'browser_report.json'),JSON.stringify(result,null,2)+'\n');
    console.log(JSON.stringify(result));
  } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exit(1);});
