import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'
import { writeFile } from 'node:fs/promises'
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--use-gl=swiftshader']})
try{
 const page=await browser.newPage({viewport:{width:1440,height:960}})
 const errors=[];page.on('pageerror',e=>errors.push(e.message));page.on('console',e=>{if(e.type()==='error')errors.push(e.text())})
 for(const route of ['/','/3d/maps','/2d/build','/2d/versions']){
  await page.goto('http://127.0.0.1:8767/#'+route);await page.waitForTimeout(1300)
  await page.screenshot({path:'/tmp/d1max-'+route.replaceAll('/','_')+'.png'})
  console.log(JSON.stringify({route,errors,text:(await page.locator('body').innerText()).slice(0,1000),overflow:await page.evaluate(()=>({w:document.documentElement.scrollWidth,v:innerWidth}))}))
 }
 await writeFile('/tmp/d1max-smoke-errors.json',JSON.stringify(errors))
}finally{await browser.close()}
