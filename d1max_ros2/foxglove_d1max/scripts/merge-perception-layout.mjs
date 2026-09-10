// Merge camera surfaces into the original perception view using native import.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
const title='D1 Max · 实机工作台',retired=title+'（合并前备份）';
const dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
const generated=JSON.parse(await readFile('layouts/D1Max-Live.json','utf8'));
let record;
for(const f of await readdir(dir)){
 const x=JSON.parse(await readFile(dir+'/'+f,'utf8'));
 if(x.name===retired)throw Error('Previous upgrade backup exists; inspect it before another import.');
 if(x.name===title){if(record)throw Error('Ambiguous live layout');record=x;}
}
if(!record)throw Error('Existing live layout not found');
const data=structuredClone(record.working?.data??record.baseline.data);
await writeFile('artifacts/live-layout-before-merge.json',JSON.stringify(data,null,2));
const main=data.configById['3D!d1scene'],previous=data.configById['3D!d1surround'];
if(!main)throw Error('Original perception scene missing');
main.foxglovePanelTitle=generated.configById['3D!d1scene'].foxglovePanelTitle;
for(const side of ['front','rear']){
 const imageTopic=`/${side}_camera/image_compressed`;
 const defaults=generated.configById['3D!d1scene'].topics[imageTopic];
 main.topics[imageTopic]={...defaults,...previous?.topics[imageTopic],visible:true,color:defaults.color};
 const info=main.topics[imageTopic].cameraInfoTopic;
 main.topics[info]={...generated.configById['3D!d1scene'].topics[info],...previous?.topics[info],visible:false};
 main.topics[`/${side}_lidar`]={...main.topics[`/${side}_lidar`],visible:true};
}
const tabs=data.configById['Tab!d1workspace'];
tabs.tabs=tabs.tabs.filter(t=>t.title!=='环绕感知');
tabs.activeTabIdx=tabs.tabs.findIndex(t=>t.title==='感知总览');
if(tabs.activeTabIdx<0)throw Error('Original perception tab missing');
delete data.configById['3D!d1surround'];
delete data.configById['Markdown!d1projectioninfo'];
await writeFile('artifacts/merged-layout-installed.json',JSON.stringify(data,null,2));
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 let page;
 for(const p of browser.contexts().flatMap(c=>c.pages()))if(await p.getByTestId('layout-menu-button').count()&&(await p.getByTestId('layout-menu-button').innerText()).trim()===title&&decodeURIComponent(p.url()).includes('127.0.0.1:8769'))page=p;
 if(!page)throw Error('Open live workbench on port 8769 first');
 page.setDefaultTimeout(8000);await page.evaluate(async()=>desktopBridge.activateTab(await desktopBridge.getTabId()));
 await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
 const row=page.getByRole('menuitem',{name:title,exact:true}).last();await row.hover();await row.getByTestId('layout-actions').click();
 await page.getByRole('menuitem',{name:'重命名',exact:true}).click();
 const input=page.locator('#layout-menu input[type="text"]');await input.fill(retired);await input.press('Enter');
 await page.keyboard.press('Escape');await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
 await page.evaluate(({text,title})=>{
  window.__d1Picker=window.showOpenFilePicker;
  window.showOpenFilePicker=async()=>[{kind:'file',name:title+'.json',getFile:async()=>new File([text],title+'.json',{type:'application/json'})}];
 },{text:JSON.stringify(data),title});
 try{
  await page.getByTestId('import-layout').click();
  await page.waitForFunction(title=>document.querySelector('[data-testid="layout-menu-button"]')?.textContent?.trim()===title,title);
  await page.getByText('感知总览',{exact:true}).waitFor();
 }
 finally{await page.evaluate(()=>{window.showOpenFilePicker=window.__d1Picker;delete window.__d1Picker;});}
 await page.reload({waitUntil:'domcontentloaded'});
 await page.locator('.d1-panel').waitFor();
 if(await page.getByText('环绕感知',{exact:true}).count())throw Error('Separate surround tab still present');
 console.log('Merged perception imported; original camera view, panel sizes and SDK configuration retained.');
}finally{await browser.close();}
