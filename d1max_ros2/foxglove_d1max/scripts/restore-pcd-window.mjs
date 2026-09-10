// Display-only, bounded restoration via native import/save. Never writes Foxglove caches.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import {restoreMapWindow,selectConfiguredMap} from './monitor-layout.mjs';
const loadMap=process.argv.includes('--load-configured-map');
const title='D1 Max · 实机工作台',retired=title+(loadMap?'（加载单层 PCD 前备份）':'（恢复 PCD 窗口前备份）');
const beforeFile=loadMap?'artifacts/layout-before-single-floor-load.json':'artifacts/layout-before-pcd-window-restore.json';
const installedFile=loadMap?'artifacts/single-floor-loaded-layout.json':'artifacts/pcd-window-restored-layout.json';
const dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
async function records(){
 const result=[];for(const file of await readdir(dir)){try{result.push(JSON.parse(await readFile(dir+'/'+file,'utf8')));}catch{/* Ignore in-flight app writes. */}}
 return result;
}
const all=await records(),active=all.filter(r=>r.name===title);
assert.equal(active.length,1,'Exactly one current workbench required');
const record=active[0],old=record.working?.data??record.baseline.data;
const previous=JSON.parse(await readFile('artifacts/layout-before-single-floor.json','utf8'));
const data=loadMap?selectConfiguredMap(old,JSON.parse(await readFile('artifacts/monitor-map-metadata.json','utf8')),JSON.parse(await readFile('config/panel.json','utf8'))):restoreMapWindow(old,previous);
if(JSON.stringify(data)===JSON.stringify(old)){console.log('PCD window already restored and unloaded.');process.exit(0);}
assert.ok(!all.some(r=>r.name===retired),'Inspect existing restoration backup before retrying');
assert.doesNotMatch(JSON.stringify(data.configById),/\/d1max\/maps\/building\/points|foxglove.Urdf/);
await writeFile(beforeFile,JSON.stringify(old,null,2)+'\n',{flag:'wx'});
await writeFile(installedFile,JSON.stringify(data,null,2)+'\n');
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const page=browser.contexts().flatMap(c=>c.pages()).find(p=>p.url().includes('layoutId='+record.id));
 assert(page,'Current workbench must be open');page.setDefaultTimeout(12000);
 async function closeMenus(){
  await page.locator('#layout-action-menu .MuiBackdrop-root').evaluateAll(bs=>bs.forEach(b=>b.click()));
  await page.locator('#layout-menu .MuiBackdrop-root').evaluateAll(bs=>bs.forEach(b=>b.click()));
 }
 await page.evaluate(async()=>window.desktopBridge.activateTab(await window.desktopBridge.getTabId()));
 await closeMenus();await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
 await page.getByRole('menuitem',{name:title,exact:true}).last().getByTestId('layout-actions').evaluate(b=>b.click());
 await page.getByRole('menuitem',{name:'重命名',exact:true}).evaluate(b=>b.click());
 const input=page.locator('#layout-menu input[type="text"]');await input.fill(retired);await input.press('Enter');
 await closeMenus();await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
 await page.evaluate(({text,title})=>{window.__d1Picker=window.showOpenFilePicker;window.showOpenFilePicker=async()=>[{kind:'file',name:title+'.json',getFile:async()=>new File([text],title+'.json',{type:'application/json'})}];},{text:JSON.stringify(data),title});
 try{
  await page.getByTestId('import-layout').evaluate(b=>b.click());
  await page.waitForURL(url=>!url.href.includes('layoutId='+record.id));
  await page.getByText('PCD · MAP',{exact:true}).waitFor();
  await page.locator('.d1-signals').waitFor();
 }finally{await page.evaluate(()=>{window.showOpenFilePicker=window.__d1Picker;delete window.__d1Picker;});}
 await closeMenus();await page.getByTestId('layout-menu-button').evaluate(b=>b.click());
 await page.getByRole('menuitem',{name:title,exact:true}).last().getByTestId('layout-actions').evaluate(b=>b.click());
 const save=page.getByRole('menuitem',{name:'保存更改',exact:true});if(await save.isEnabled())await save.evaluate(b=>b.click());
 await closeMenus();
 const id=new URLSearchParams(page.url().split('?')[1]).get('layoutId');
 let persisted=false;for(let n=0;n<40;n++){
  const r=(await records()).find(r=>r.id===id&&r.name===title);
  if(r?.baseline?.data?.configById?.['3D!d1map']){
   assert.deepEqual(r.baseline.data.layout,data.layout);
   assert.deepEqual(r.baseline.data.configById['3D!d1map'].topics,data.configById['3D!d1map'].topics);
   persisted=true;break;
  }
  await page.waitForTimeout(250);
 }
 assert(persisted,'Not durably saved; leave open and inspect before reload');
 console.log(loadMap?'Single-floor PCD selected and saved:':'PCD window restored and saved:',page.url());
}finally{await browser.close();}
