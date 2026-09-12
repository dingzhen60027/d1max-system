// Explicit, bounded cleanup through Foxglove native UI, with local JSON recovery.
// No app-cache writes, robot calls, data deletion, or layout geometry changes.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,mkdir,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const keepId='lay_0ebVCw8kOlp4hxiD',keepName='D1 Max · 实机工作台';
const allowed=new Map([
 ['lay_0eU4tSVGweOxOuOv','默认'],
 ['lay_0ebMVcde9g8WqXDk','D1 Max · 感知工作台'],
 ['lay_0ebSyr2k5GHPue21','D1 Max · 实机工作台（恢复 PCD 窗口前备份）'],
 ['lay_0ebT78XgKUL77d1G','D1 Max · 实机工作台（加载单层 PCD 前备份）'],
 ['lay_0ebT8DAcov9bFrkG','D1 Max · 实机工作台（定位接入前备份）'],
]);
const directory='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
async function records(){const all=[];for(const file of await readdir(directory)){try{const r=JSON.parse(await readFile(directory+'/'+file,'utf8'));if(r.id&&r.syncInfo?.status!=='locally-deleted')all.push(r)}catch{}}return all;}
const all=await records(),keep=all.find(r=>r.id===keepId);
assert.equal(keep?.name,keepName,'Final layout must exist with its verified ID');
const targets=all.filter(r=>r.id!==keepId);
for(const r of targets)assert.equal(allowed.get(r.id),r.name,'Unexpected layout; refuse broad deletion');
const stamp=new Date().toISOString().replaceAll(':','-'),backup='artifacts/layout-cleanup-'+stamp;
await mkdir(backup,{recursive:true});
for(const r of all)await writeFile(backup+'/'+r.id+'.json',JSON.stringify(r.working?.data??r.baseline.data,null,2)+'\n',{flag:'wx'});
const manifest={createdAt:new Date().toISOString(),keep:{id:keepId,name:keepName},targets:targets.map(r=>({id:r.id,name:r.name})),deleted:[],backup};
await writeFile(backup+'/manifest.json',JSON.stringify(manifest,null,2)+'\n');
console.log(JSON.stringify({backup,keep:manifest.keep,targets:manifest.targets}));
if(!process.argv.includes('--apply'))process.exit(0);
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const page=browser.contexts().flatMap(c=>c.pages()).find(p=>p.url().includes('layoutId='));assert(page);page.setDefaultTimeout(8000);
 const syncResponses=[];
 page.on('response',response=>{const url=new URL(response.url());if(response.request().method()==='DELETE')syncResponses.push({path:url.pathname,status:response.status()});});
 async function closeMenus(){for(const id of ['layout-action-menu','layout-menu'])await page.locator('#'+id+' .MuiBackdrop-root').evaluateAll(es=>es.forEach(e=>e.click()));}
 async function menu(){await closeMenus();await page.getByTestId('layout-menu-button').evaluate(e=>e.click());}
 await menu();await page.getByRole('menuitem',{name:keepName,exact:true}).last().evaluate(e=>e.click());
 await page.waitForURL(url=>url.href.includes('layoutId='+keepId));
 await menu();await page.getByRole('menuitem',{name:keepName,exact:true}).last().getByTestId('layout-actions').evaluate(e=>e.click());
 await page.locator('#layout-action-menu').waitFor();
 const save=page.getByRole('menuitem',{name:'保存更改',exact:true});if(await save.count()&&await save.isEnabled())await save.evaluate(e=>e.click());await closeMenus();
 for(const target of targets){
  const current=(await records()).find(r=>r.id===target.id);assert.equal(current?.name,target.name);
  const saved=JSON.parse(await readFile(backup+'/'+target.id+'.json','utf8'));
  assert.deepEqual(current.working?.data??current.baseline.data,saved,'Layout changed since backup; refuse deletion');
  await menu();await page.getByRole('menuitem',{name:target.name,exact:true}).last().getByTestId('layout-actions').evaluate(e=>e.click());
  await page.getByRole('menuitem',{name:'删除',exact:true}).evaluate(e=>e.click());
  const dialog=page.getByRole('dialog');await dialog.waitFor();
  const text=await dialog.innerText();assert(text.includes(target.name),text);
  const confirm=dialog.getByRole('button',{name:'删除',exact:true});await confirm.click();
  await dialog.waitFor({state:'hidden'});
  let removed=false;
  for(let count=0;count<40;count++){if(!(await records()).some(r=>r.id===target.id)){removed=true;break}await page.waitForTimeout(200);}
  assert(removed,'Deletion not confirmed for '+target.name);
  manifest.deleted.push(target.id);await writeFile(backup+'/manifest.json',JSON.stringify(manifest,null,2)+'\n');
  console.log('Deleted layout: '+target.name);
 }
 await closeMenus();await page.reload({waitUntil:'domcontentloaded'});await page.locator('.d1-lifecycle').waitFor();
 await page.getByText('PCD · LOCALIZATION',{exact:true}).waitFor();
 assert(page.url().includes('layoutId='+keepId));
 const final=await records();assert.deepEqual(final.map(r=>r.id),[keepId]);
 const currentData=final[0].working?.data??final[0].baseline.data;
 assert.deepEqual(currentData.layout,(keep.working?.data??keep.baseline.data).layout);
 await menu();await page.screenshot({path:backup+'/only-final-layout.png'});
 console.log(JSON.stringify({remaining:final.map(r=>({id:r.id,name:r.name})),backup,deleted:manifest.deleted.length,syncResponses}));
 await closeMenus();
}finally{await browser.close()}
