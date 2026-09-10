// Upgrade through native Foxglove import/save only. Never edits app caches or calls robot services.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
import {monitorLayout} from './monitor-layout.mjs';
const title='D1 Max · 实机工作台',retired=title+'（双电池合并前备份）';
const dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
async function records(){
 const result=[];for(const file of await readdir(dir)){try{result.push(JSON.parse(await readFile(dir+'/'+file,'utf8')));}catch{/* In-flight app writes are not ours to repair. */}}
 return result;
}
const all=await records(),active=all.filter(r=>r.name===title),backups=all.filter(r=>r.name===retired);
if(active.length>1||backups.length>1)throw Error('Ambiguous layout; inspect before migrating');
const current=active[0],backup=backups[0];
const currentData=current&&(current.working?.data??current.baseline.data);
if(currentData?.configById?.['Tab!d1batteries']&&currentData.configById['Gauge!d1b1']?.style==='bar'&&currentData.layout.direction==='column'){
 console.log('Compact dual-battery layout is already installed; no changes.');process.exit(0);
}
if(current&&backup)throw Error('An earlier backup exists; inspect before replacing it');
const record=current??backup;if(!record)throw Error('Open the live workbench first');
const old=record.working?.data??record.baseline.data;
await writeFile('artifacts/layout-before-battery-group.json',JSON.stringify(old,null,2)+'\n',{flag:'wx'}).catch(e=>{if(e.code!=='EEXIST')throw e;});
const generated=JSON.parse(await readFile('layouts/D1Max-Monitor.json','utf8'));
const data=monitorLayout(old,JSON.parse(await readFile('artifacts/monitor-map-metadata.json','utf8')),JSON.parse(await readFile('config/panel.json','utf8')));
for(const id of ['d1-calibration','d1-camera-front','d1-camera-rear','d1-telemetry'])data.userNodes[id]=generated.userNodes[id];
if(/foxglove.Urdf|8770|d1max_model\//.test(JSON.stringify(data)))throw Error('Model asset reference survived migration');
await writeFile('artifacts/monitor-layout-installed.json',JSON.stringify(data,null,2)+'\n');
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const page=browser.contexts().flatMap(c=>c.pages()).find(p=>decodeURIComponent(p.url()).includes('127.0.0.1:8769'));
 if(!page)throw Error('Open live workbench on port 8769 first');
 page.setDefaultTimeout(10000);
 async function closeMenus(){
  await page.locator('#layout-action-menu .MuiBackdrop-root').evaluateAll(bs=>bs.forEach(b=>b.click()));
  await page.locator('#layout-menu .MuiBackdrop-root').evaluateAll(bs=>bs.forEach(b=>b.click()));
 }
 await page.evaluate(async()=>window.desktopBridge.activateTab(await window.desktopBridge.getTabId()));
 await closeMenus();
 if(current){
  await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
  await page.getByRole('menuitem',{name:title,exact:true}).last().getByTestId('layout-actions').evaluate(b=>b.click());
  await page.getByRole('menuitem',{name:'重命名',exact:true}).evaluate(b=>b.click());
  const input=page.locator('#layout-menu input[type="text"]');await input.fill(retired);await input.press('Enter');
  await closeMenus();
 }
 await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
 await page.evaluate(({text,title})=>{window.__d1Picker=window.showOpenFilePicker;window.showOpenFilePicker=async()=>[{kind:'file',name:title+'.json',getFile:async()=>new File([text],title+'.json',{type:'application/json'})}];},{text:JSON.stringify(data),title});
 try{await page.getByTestId('import-layout').evaluate(b=>b.click());await page.locator('.d1-signals').waitFor();}
 finally{await page.evaluate(()=>{window.showOpenFilePicker=window.__d1Picker;delete window.__d1Picker;});}
 await closeMenus();await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
 await page.getByRole('menuitem',{name:title,exact:true}).last().getByTestId('layout-actions').evaluate(b=>b.click());
 const save=page.getByRole('menuitem',{name:'保存更改',exact:true});if(await save.isEnabled())await save.evaluate(b=>b.click());
 await closeMenus();
 const id=new URLSearchParams(page.url().split('?')[1]).get('layoutId');
 let persisted=false;for(let n=0;n<40;n++){
  const r=(await records()).find(r=>r.id===id&&r.name===title);
  if(r?.baseline?.data?.configById?.['Tab!d1batteries']&&r.baseline.data.configById['Gauge!d1b1']?.style==='bar'){persisted=true;break;}
  await page.waitForTimeout(250);
 }
 if(!persisted)throw Error('Layout not durably saved; left open, do not reload yet');
 await page.reload({waitUntil:'domcontentloaded'});await page.locator('.d1-signals').waitFor();
 await page.getByText('LiDAR + RGB · LIVE',{exact:true}).waitFor();
 console.log('Saved single-screen layout:',page.url());
}finally{await browser.close();}
