// Update the task-owned live layout through the official file-import UI.
// Preserve panel sizes/camera edits; save the pre-update layout as a local export.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
const title='D1 Max · 实机工作台', retired=title+'（资源迁移前）';
const dir='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
let record;for(const f of await readdir(dir)){const x=JSON.parse(await readFile(dir+'/'+f,'utf8'));if(x.name===title){if(record)throw Error('Ambiguous layout');record=x;}}
if(!record)throw Error('Existing live layout not found');
const data=record.working?.data??record.baseline.data;
await writeFile('artifacts/live-layout-before-http.json',JSON.stringify(data,null,2));
const model=JSON.parse(await readFile('config/live-model.json','utf8'));
for(const p of Object.values(data.configById)){
 if(p.layers?.['d1-urdf']){p.layers['d1-urdf'].url=model.urdfUrl;p.scene.transforms={...p.scene.transforms,showLabel:false,axisSize:.12};}
}
data.configById['Tab!d1workspace'].activeTabIdx=0;
await writeFile('artifacts/live-layout-installed.json',JSON.stringify(data,null,2));
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 let page;for(const p of browser.contexts().flatMap(c=>c.pages()))if(await p.getByTestId('layout-menu-button').count()&&(await p.getByTestId('layout-menu-button').innerText()).trim()===title)page=p;
 if(!page)throw Error('Live layout must be open');
 page.setDefaultTimeout(8000);await page.evaluate(async()=>window.desktopBridge.activateTab(await window.desktopBridge.getTabId()));
 await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
 const row=page.getByRole('menuitem',{name:title,exact:true}).last();await row.hover();await row.getByTestId('layout-actions').click();
 await page.getByRole('menuitem',{name:'重命名',exact:true}).click();const input=page.locator('#layout-menu input[type="text"]');await input.fill(retired);await input.press('Enter');
 await page.keyboard.press('Escape');await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
 await page.evaluate(({text,title})=>{window.__d1Picker=window.showOpenFilePicker;window.showOpenFilePicker=async()=>[{kind:'file',name:title+'.json',getFile:async()=>new File([text],title+'.json',{type:'application/json'})}];},{text:JSON.stringify(data),title});
 try{await page.getByTestId('import-layout').click();await page.getByText('机器人模型',{exact:true}).waitFor();}
 finally{await page.evaluate(()=>{window.showOpenFilePicker=window.__d1Picker;delete window.__d1Picker;});}
 console.log('Imported live HTTP-assets layout. Pre-update layout preserved as '+retired);
}finally{await browser.close();}
