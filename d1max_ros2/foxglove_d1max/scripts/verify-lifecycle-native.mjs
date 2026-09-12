// Actual desktop lifecycle QA only. NEVER invoke software emergency stop.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {writeFile} from 'node:fs/promises';
import {execFileSync} from 'node:child_process';
import assert from 'node:assert/strict';
assert.equal(process.env.D1MAX_LIFECYCLE_QA,'1','Explicit lifecycle QA opt-in required; Web must be idle and robot cable unplugged');
assert.equal((await (await import('node:fs/promises')).readFile('/sys/class/net/enx6c1ff7bc241e/operstate','utf8')).trim(),'down','This test must not start a live robot session');
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const page=browser.contexts().flatMap(c=>c.pages()).find(p=>p.url().includes('lay_0ebT8DAcov9bFrkG'));
 assert(page);page.setDefaultTimeout(15000);
 const row=name=>page.locator('[data-service="'+name+'"]');
 const button=name=>row(name).locator('button');
 const report={time:new Date().toISOString(),cycles:[],postPaths:[],errors:[]};
 page.on('request',r=>{if(r.method()==='POST'&&r.url().includes(':8771'))report.postPaths.push(new URL(r.url()).pathname);});
 page.on('pageerror',e=>report.errors.push(e.message));
 await page.reload({waitUntil:'domcontentloaded'});
 await row('monitor').waitFor();
 await page.waitForFunction(()=>['stopped','failed'].includes(document.querySelector('[data-service="monitor"]')?.getAttribute('data-phase')));
 // Manager being available with the ROS bridge off is the central Connect requirement.
 assert.equal(await button('monitor').isEnabled(),true);
 await button('monitor').click();
 await page.waitForFunction(()=>document.querySelector('[data-service="monitor"]')?.getAttribute('data-phase')==='failed');
 assert.match(await button('monitor').getAttribute('title'),/网络不可达/);
 report.offlineConnect='failed honestly; no worker processes started';
 const show=unit=>Object.fromEntries(execFileSync('systemctl',['--user','show',unit,'--property=ActiveState,MainPID,ControlGroup,KillMode,TimeoutStopUSec'],{encoding:'utf8'}).trim().split('\n').map(line=>line.split(/=(.*)/s).slice(0,2)));
 assert.equal(show('d1max-monitor-managed.service').MainPID,'0');
 for(let cycle=0;cycle<2;cycle++){
  await page.waitForFunction(()=>document.querySelector('[data-service="web"]')?.getAttribute('data-phase')==='stopped');
  // Two synchronous native click() events exercise the in-flight lock.
  await button('web').evaluate(e=>{e.click();e.click();});
  await page.waitForFunction(()=>document.querySelector('[data-service="web"]')?.getAttribute('data-phase')==='running');
  const started=show('d1max-web-managed.service');assert.notEqual(started.MainPID,'0');
  assert.equal(await row('web').getByRole('link').count(),1);
  const health=await fetch('http://127.0.0.1:8766/api/health').then(r=>r.json());assert.equal(health.ok,true);
  const overview=await fetch('http://127.0.0.1:8766/api/overview').then(r=>r.json());
  assert.equal(overview.processing_job.running,false,'Do not interrupt user point-cloud work');
  assert.ok(['idle','stopped'].includes(overview.runtime.status),'Do not interrupt user mapping');
  await button('web').click();
  await page.waitForFunction(()=>document.querySelector('[data-service="web"]')?.getAttribute('data-phase')==='stopped');
  const stopped=show('d1max-web-managed.service');assert.equal(stopped.MainPID,'0');assert.equal(stopped.ControlGroup,'');
  report.cycles.push({cycle,started,stopped});
 }
 // Restore Web availability because it was running before migration.
 await button('web').click();
 await page.waitForFunction(()=>document.querySelector('[data-service="web"]')?.getAttribute('data-phase')==='running');
 const before=show('d1max-web-managed.service').MainPID,posts=report.postPaths.length;
 await page.reload({waitUntil:'domcontentloaded'});
 await page.waitForFunction(()=>document.querySelector('[data-service="web"]')?.getAttribute('data-phase')==='running');
 assert.equal(show('d1max-web-managed.service').MainPID,before);
 assert.equal(report.postPaths.length,posts);
 assert.deepEqual(report.postPaths,['/v1/monitor/start','/v1/web/start','/v1/web/stop','/v1/web/start','/v1/web/stop','/v1/web/start']);
 assert.deepEqual(report.errors,[]);
 report.reload='no start, stop or duplicate process';report.robotCommands=0;
 report.services=await page.locator('.d1-service').evaluateAll(es=>es.map(e=>({name:e.dataset.service,phase:e.dataset.phase})));
 await writeFile('artifacts/lifecycle-native-checks.json',JSON.stringify(report,null,2));
 const cdp=await page.context().newCDPSession(page),shot=await cdp.send('Page.captureScreenshot',{format:'png',fromSurface:false,captureBeyondViewport:false});
 await writeFile('artifacts/lifecycle-native.png',Buffer.from(shot.data,'base64'));await cdp.detach();
 console.log(JSON.stringify(report,null,2));
}finally{await browser.close();}
