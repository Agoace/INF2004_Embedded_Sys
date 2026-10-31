/* Optional real-browser integration evidence. No runtime Playwright dependency. */
const fs=require('fs'),path=require('path'),assert=require('assert/strict'),http=require('http'),crypto=require('crypto');
const {spawn,execFileSync}=require('child_process'),{pathToFileURL}=require('url');
const {chromium}=require(process.env.SIM_PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'../..'),build=path.resolve(process.argv[2]||path.join(root,'sim/build'));
const out=path.join(root,'sim/output/web_check');fs.mkdirSync(out,{recursive:true});
const saveName='web_check_saved_'+crypto.randomBytes(4).toString('hex'),savedPath=path.join(root,'sim/scenarios',saveName+'.json');
let server,browser,created=false,log='';
const sha=p=>crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
async function start(build){const proc=spawn('python3',[path.join(root,'sim/viewer/serve.py'),'--build',build,'--port','0'],{cwd:root});proc.stderr.on('data',b=>log+=b);const url=await new Promise((resolve,reject)=>{let text='';proc.stdout.on('data',b=>{text+=b;const m=text.match(/http:\/\/127\.0\.0\.1:\d+/);if(m)resolve(m[0]);});proc.on('error',reject);proc.on('exit',code=>reject(Error('server exited '+code+' '+log)));});return {proc,url};}
const field=p=>'[data-field="'+p+'"]';
(async()=>{
 assert(!fs.existsSync(savedPath));server=await start(build);browser=await chromium.launch();const errors=[],external=[];
 const c=await browser.newContext({viewport:{width:1800,height:1200}});
 await c.route('**/*',r=>{const u=r.request().url();if(u.startsWith(server.url+'/')||u.startsWith('file:'))return r.continue();external.push(u);return r.abort();});
 const p=await c.newPage();const watch=page=>{page.on('pageerror',e=>errors.push(e.message));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});};watch(p);
 await p.goto(server.url,{waitUntil:'networkidle'});await p.waitForFunction(()=>window.envEditor?.getScenario()?.name==='box_straight'&&document.querySelector('canvas').width>300);
 const baselineSha=sha(path.join(root,'sim/scenarios/box_straight.json'));
 async function run(){const response=p.waitForResponse(r=>r.url()===server.url+'/api/run'&&r.request().method()==='POST');await p.click('#run');const r=await response;assert.equal(r.status(),200);const data=await r.json();await p.waitForFunction(id=>envEditor.getRun()?.id===id&&envEditor.lastPlaybackMs!==null,data.run.id);return {data,ms:await p.evaluate(()=>envEditor.lastPlaybackMs)};}
 const baseline=await run();assert.equal(baseline.data.run.result,'FAILED TIME');
 await p.click('#edit-floor');await p.locator(field('obstacles[0].size_mm[1]')).fill('240');await p.locator(field('obstacles[0].size_mm[0]')).fill('480');await p.locator(field('obstacles[0].pose.x_mm')).fill('980');
 await p.screenshot({path:path.join(out,'editor.png'),fullPage:true});
 const first=await run();assert.equal(first.data.run.result,'FAILED NO_SIDE');
 assert.deepEqual(first.data.trace.header.scenario.obstacles[0].size_mm.value,[480,240]);
 const firstFolder=path.join(root,'sim/output/web',first.data.run.id);
 const cli=path.join(out,'cli_equivalent');execFileSync(path.join(build,'robot_sim'),['--scenario',path.join(firstFolder,'scenario.json'),'--output',cli],{cwd:root,timeout:20000});
 assert(fs.readFileSync(path.join(firstFolder,'trace.json')).equals(fs.readFileSync(path.join(cli,'trace.json'))));
 async function endView(data){await p.locator('#viewer').evaluate((f,t)=>{const b=f.contentDocument.getElementById('play');if(b.textContent==='Pause')b.click();f.contentWindow.simViewer.seek(t);f.contentWindow.simViewer.focus();},data.trace.firmware_observation.detour_timeline.at(-1)?.t_us||data.trace.header.scenario.end_us);}
 await endView(first.data);await p.screenshot({path:path.join(out,'after_first_run.png'),fullPage:true});
 await p.click('#edit-floor');const rect=await p.locator('#floor').boundingBox();const coords=await p.evaluate(()=>[envEditor.screen(980,0),envEditor.screen(980,100)]);
 await p.mouse.move(rect.x+coords[0][0],rect.y+coords[0][1]);await p.mouse.down();await p.mouse.move(rect.x+coords[1][0],rect.y+coords[1][1],{steps:10});await p.mouse.up();
 assert.equal((await p.evaluate(()=>envEditor.getScenario())).obstacles[0].pose.value.y_mm,100);
 const dragged=await run();await endView(dragged.data);await p.screenshot({path:path.join(out,'after_drag_run.png'),fullPage:true});
 // Both saved results are one-click accessible, without changing their data.
 await p.locator('[data-run="'+first.data.run.id+'"]').click();await p.waitForFunction(id=>envEditor.getRun()?.id===id,first.data.run.id);
 await p.locator('[data-run="'+dragged.data.run.id+'"]').click();await p.waitForFunction(id=>envEditor.getRun()?.id===id,dragged.data.run.id);
 await p.click('#edit-floor');await p.locator(field('obstacles[0].size_mm[1]')).fill('-1');
 const before=await (await c.request.get(server.url+'/api/status')).json();await p.click('#run');await p.waitForFunction(()=>document.querySelector('.field-error'));
 assert((await p.locator('#message').textContent()).includes('Width'));await p.screenshot({path:path.join(out,'invalid_input.png'),fullPage:true});
 const bad=await p.evaluate(()=>envEditor.getScenario());assert.equal((await c.request.post(server.url+'/api/run',{data:{base:'box_straight',scenario:bad}})).status(),400);
 assert.equal((await (await c.request.get(server.url+'/api/status')).json()).started_runs,before.started_runs);
 await p.locator(field('obstacles[0].size_mm[1]')).fill('240');
 const locked=await p.evaluate(()=>envEditor.getScenario());locked.robot.geometry.width_mm=999;
 const rejected=await c.request.post(server.url+'/api/run',{data:{base:'box_straight',scenario:locked}});assert.equal(rejected.status(),400);assert((await rejected.json()).error.includes('read-only'));
 // Exercise adding/removing both obstacle shapes and editing a tile.
 for(const kind of ['bottle','box']){await p.click('#add-'+kind);const draft=await p.evaluate(()=>envEditor.getScenario());assert.equal(draft.obstacles.at(-1).type,kind);if(kind==='bottle')await p.locator(field('obstacles[1].diameter_mm')).fill('80');await p.getByRole('button',{name:'Remove '+draft.obstacles.at(-1).id,exact:true}).click();}
 await p.check('#grid-mode');const xy=await p.evaluate(()=>envEditor.screen(1600,315));await p.mouse.click(rect.x+xy[0],rect.y+xy[1]);await p.selectOption('#tile-type','curve');await p.click('#rotate');let tile=(await p.evaluate(()=>envEditor.getScenario())).track.tiles.find(t=>t.col===6&&t.row===1);assert.equal(tile.type,'curve');assert.equal(tile.rotation_deg,90);await p.selectOption('#tile-type','cross');await p.selectOption('#tile-type','blank');await p.uncheck('#grid-mode');
 await p.locator('#duration').fill('41');await p.locator('#save-name').fill(saveName);await p.click('#save');await p.waitForFunction(n=>envEditor.getScenario().name===n,saveName);created=true;assert(fs.existsSync(savedPath));
 await p.selectOption('#base','box_straight');await p.waitForFunction(()=>envEditor.getScenario().name==='box_straight');await p.selectOption('#base',saveName);await p.waitForFunction(n=>envEditor.getScenario().name===n,saveName);
 const saved=await run();assert.equal(saved.data.trace.header.scenario.name,saveName);assert.equal(saved.data.trace.header.scenario.end_us,41000000);
 await p.locator('#save-name').fill('box_straight');await p.click('#save');assert((await p.locator('#message').textContent()).includes('already exists'));
 assert.equal((await c.request.post(server.url+'/api/save',{data:{base:saveName,scenario:await p.evaluate(()=>envEditor.getScenario()),name:'box_straight'}})).status(),409);assert.equal(sha(path.join(root,'sim/scenarios/box_straight.json')),baselineSha);
 const paths={};for(const url of ['/../src/detour_test.c','/%2e%2e/src/detour_test.c','/etc/passwd','/runs/../../etc/passwd']){const response=await c.request.get(server.url+url);paths[url]=response.status();assert([400,404].includes(response.status()));}
 assert.equal((await c.request.get(server.url+'/api/status',{headers:{Origin:'http://evil.example'}})).status(),403);
 const peerRejected=await new Promise(resolve=>{const r=http.get(server.url+'/api/status',{localAddress:'127.0.0.2'},()=>resolve(false));r.on('error',()=>resolve(true));r.setTimeout(2000,()=>{r.destroy();resolve(false);});});assert(peerRejected);
 // The produced HTML also works without any server connection or trace file.
 const isolated=path.join(out,'offline');fs.mkdirSync(isolated,{recursive:true});fs.copyFileSync(path.join(firstFolder,'run.html'),path.join(isolated,'run.html'));
 await c.setOffline(true);const offline=await c.newPage();watch(offline);await offline.goto(pathToFileURL(path.join(isolated,'run.html')).href);await offline.waitForFunction(()=>window.simViewer);assert.equal(await offline.evaluate(()=>simViewer.data.scenario.obstacles[0].size_mm[0]),480);
 assert.deepEqual(errors,[]);assert.deepEqual(external,[]);
 const report={browser:browser.version(),url:server.url,baseline_click_to_playback_ms:baseline.ms,first_click_to_playback_ms:first.ms,drag_click_to_playback_ms:dragged.ms,first_run:first.data.run,drag_run:dragged.data.run,saved_run:saved.data.run,cli_trace_byte_identical:true,trace_sha256:sha(path.join(firstFolder,'trace.json')),invalid_input_no_run:true,readonly_model_rejected:true,saved_reload_and_run:true,existing_name_refused:true,path_rejections:paths,non_127_peer_rejected:peerRejected,offline_html_without_trace:true,console_errors:errors,external_requests:external,screenshots:['editor.png','after_first_run.png','after_drag_run.png','invalid_input.png'].map(f=>path.join(out,f)),saved_fixture_removed_after_check:true};
 fs.writeFileSync(path.join(out,'browser_report.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
})().catch(e=>{console.error(e);process.exitCode=1;}).finally(async()=>{if(browser)await browser.close();if(server)server.proc.kill('SIGTERM');if(created)fs.unlinkSync(savedPath);fs.writeFileSync(path.join(out,'server.log'),log);});
