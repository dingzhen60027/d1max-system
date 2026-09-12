// Read-only native layout QA. Never clicks Connect, Stop, safety or ROS publish.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile,readdir,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try{
 const page=browser.contexts().flatMap(c=>c.pages()).find(p=>p.url().includes('layoutId='));assert(page);
 const errors=[];page.on('pageerror',e=>errors.push(e.message));page.setDefaultTimeout(10000);
 await page.reload({waitUntil:'domcontentloaded'});await page.locator('.d1-lifecycle').waitFor();await page.getByText('PCD · LOCALIZATION',{exact:true}).waitFor();
 await page.getByLabel('定位未运行或无新鲜状态',{exact:true}).waitFor();
 const directory='/home/dndx/.config/Foxglove/studio-datastores/layouts-remote-om_0eNs22bzvmtldd6y';
 const id=new URLSearchParams(page.url().split('?')[1]).get('layoutId');let actual;
 for(const file of await readdir(directory)){try{const r=JSON.parse(await readFile(directory+'/'+file,'utf8'));if(r.id===id)actual=r.working?.data??r.baseline.data}catch{}}
 assert(actual);const before=JSON.parse(await readFile('artifacts/layout-before-localization.json','utf8'));
 assert.deepEqual(actual.layout,before.layout);
 assert.deepEqual(actual.configById['3D!d1scene'],before.configById['3D!d1scene']);
 assert.deepEqual(actual.configById['3D!d1map'].cameraState,before.configById['3D!d1map'].cameraState);
 assert.equal(actual.configById['3D!d1map'].followTf,'d1max_loc_map');
 assert.equal(await page.locator('.d1-panel button').count(),3);
 assert(await page.locator('.d1-estop').isDisabled());
 const overflow=await page.evaluate(()=>[...document.querySelectorAll('.d1-panel,.d1-lifecycle,.d1-service,.d1-estop')].filter(e=>e.scrollWidth>e.clientWidth+2||e.scrollHeight>e.clientHeight+2).map(e=>e.className));assert.deepEqual(overflow,[]);
 assert.deepEqual(errors,[]);await page.screenshot({path:'artifacts/localization-native.png'});
 const report={passed:true,layoutId:id,layoutUnchanged:true,liveCameraUnchanged:true,localizationIndicator:true,serviceCalls:0,errors};
 await writeFile('artifacts/localization-native-qa.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
}finally{await browser.close()}
