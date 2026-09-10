// Scoped layout navigation/inspection only. Never clicks robot control actions.
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs';
import {readFile} from 'node:fs/promises';
const browser=await chromium.connectOverCDP('http://127.0.0.1:9224');
try {
 const pages=browser.contexts().flatMap(c=>c.pages());
 let page;for(const p of pages)if(await p.getByTestId('layout-menu-button').count()) {
  if((await p.getByTestId('layout-menu-button').innerText()).trim()==='D1 Max · 感知工作台'){page=p;break;}
 }
 if(!page)throw Error('Open D1 Max · 感知工作台 first.');
 page.setDefaultTimeout(6000);
 await page.evaluate(async()=>window.desktopBridge.activateTab(await window.desktopBridge.getTabId()));
 const action=process.argv[2]??'inspect';
 if(action==='inspect')console.log((await page.locator('body').innerText()).slice(0,7000));
 else if(action==='screenshot')await page.screenshot({path:'artifacts/native-overview.png'});
 else if(action==='save') {
  await page.keyboard.press('Escape');await page.getByTestId('layout-menu-button').click();
  const row=page.getByRole('menuitem',{name:'D1 Max · 感知工作台',exact:true}).last();
  await row.hover();await row.getByTestId('layout-actions').click();
  const save=page.getByRole('menuitem',{name:'保存更改',exact:true});if(await save.isEnabled())await save.click();
  await page.keyboard.press('Escape');await page.keyboard.press('Escape');console.log('Saved native layout.');
 } else if(action==='verify') {
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.locator('.d1-panel').waitFor();
  if(await page.locator('.d1-console').count())throw Error('Old full-page UI still mounted.');
  if(!await page.getByRole('button',{name:'软件急停',exact:true}).isDisabled())throw Error('Expected read-only gateway.');
  console.log('Compact extension mounted; emergency control disabled; old full-page UI absent.');
  console.log('Page errors:',errors);
 } else throw Error('Supported: inspect, screenshot, save, verify');
} finally {await browser.close();}
