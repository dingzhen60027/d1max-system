// Read-only live UI regression. Mutations are mocked or cancelled, never sent to the robot.
import assert from 'node:assert/strict'
import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'
const base=process.env.D1MAX_MAP_MANAGER_URL||'http://127.0.0.1:8766'
const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']})
const report={errors:[],checks:[]}
try{
 const page=await browser.newPage({viewport:{width:1440,height:1000}})
 page.on('pageerror',e=>report.errors.push(e.message))
 // Fail closed for every write, even if a selector accidentally hits a real action.
 await page.route('**/api/**',async route=>{
  if(['GET','HEAD'].includes(route.request().method()))return route.continue()
  return route.fulfill({status:409,contentType:'application/json',body:JSON.stringify({detail:'Read-only UI verification: write intercepted'})})
 })
 await page.goto(base+'/#/3d/maps');await page.locator('.cloud-metrics').waitFor()
 assert(await page.locator('.cloud-host canvas').count())
 await page.getByRole('button',{name:'配置参数并处理 PCD',exact:true}).click()
 const dialog=page.getByRole('dialog')
 await dialog.waitFor()
 assert.equal(await dialog.locator('.parameter-block').count(),4)
 assert.equal(await dialog.locator('.module-order button').count(),8)
 assert(await dialog.locator('a[download]').count())
 const toggle=dialog.getByRole('switch').first(),before=await toggle.getAttribute('aria-checked')
 await toggle.click();assert.notEqual(await toggle.getAttribute('aria-checked'),before);await toggle.click()
 await dialog.locator('[data-slot="select-trigger"]').click()
 await page.getByRole('option').first().waitFor({state:'visible'})
 assert((await page.getByRole('option').allTextContents()).some(v=>v.includes('Go2')))
 await page.keyboard.press('Escape')
 assert.equal(await page.getByRole('button',{name:'开始处理',exact:true}).getAttribute('type'),'submit')
 await dialog.getByRole('button',{name:'取消',exact:true}).click()
 report.checks.push('3D PCD preview and modular YAML form, switches, reordering controls and profiles preserved')
 await page.getByRole('button',{name:'更多建图操作'}).click()
 await page.getByRole('menuitem',{name:'清理 Web 建图进程'}).click()
 await page.getByRole('alertdialog').waitFor()
 await page.getByRole('button',{name:'取消',exact:true}).click()
 report.checks.push('Process cleanup is behind dropdown and explicit confirmation; not executed')
 for(const route of ['/3d/processed','/3d/planning','/3d/archived','/3d/failures','/2d/versions','/2d/build','/2d/navigation','/2d/archived','/']){
  await page.goto(base+'/#'+route);await page.waitForTimeout(600)
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth),1440)
  assert(!(await page.locator('body').innerText()).includes('后端连接异常'))
  assert(!(await page.locator('body').innerText()).includes('操作未完成'))
 }
 await page.screenshot({path:'/tmp/d1max-production-home.png'})
 report.checks.push('All homepage / 2D / 3D routes render with live data and no horizontal overflow')
 assert.deepEqual(report.errors,[])
 console.log(JSON.stringify(report,null,2))
}finally{await browser.close()}
