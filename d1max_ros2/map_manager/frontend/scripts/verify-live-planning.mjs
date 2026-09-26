// Private static fixture. ALL lifecycle writes are intercepted; never contacts ROS/robot/Web backend.
import assert from 'node:assert/strict'
import {createServer} from 'node:http'
import {readFile,mkdir,writeFile} from 'node:fs/promises'
import {extname,resolve,sep} from 'node:path'
import {fileURLToPath} from 'node:url'
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'

const dist=fileURLToPath(new URL('../dist/',import.meta.url))
const output=fileURLToPath(new URL('../artifacts/live-planning/',import.meta.url))
const types={'.html':'text/html','.js':'text/javascript','.css':'text/css','.woff2':'font/woff2'}
const forbidden=[],errors=[],writes=[],checks=[]
const data={phase:'stopped',busy:false,installed:true,mode:'LIVE_VISUALIZATION_NO_MOTION',motion_enabled:false,session_id:null,map_name:'09-23 跨楼层 SC-PGO',current_floor:'floor1',health:{},global_status:{},scan_status:{},stages:{},components:{localization:{name:'Faster-LIO + PCD'},global_planner:{name:'PCT'},local_planner:{name:'SCAN'}},connection:{phase:'stopped',active:false,health:{sdk_fresh:false,lidar_fresh:false,replay:false}}}
const snapshot=()=>{
 const result=structuredClone(data),now=Date.now()/1000
 result.snapshot_at_unix=now
 // A continuously reporting fixture renews source expiry; explicit expired stages stay expired.
 for(const stage of Object.values(result.stages))if(stage.tone==='ready'&&stage.expires_at_unix===undefined)stage.expires_at_unix=now+2
 return result
}
const server=createServer(async(req,res)=>{
 try {
  if(req.method!=='GET'||req.url.startsWith('/api/')){forbidden.push(req.method+' '+req.url);res.writeHead(403);res.end();return}
  const target=resolve(dist,'.'+new URL(req.url,'http://fixture').pathname.replace(/\/$/,'/index.html'))
  if(!target.startsWith(resolve(dist)+sep)){res.writeHead(403);res.end();return}
  res.writeHead(200,{'Content-Type':types[extname(target)]||'application/octet-stream'});res.end(await readFile(target))
 }catch{res.writeHead(404);res.end()}
})
let browser
try{
 await mkdir(output,{recursive:true})
 await new Promise(done=>server.listen(0,'127.0.0.1',done))
 const base='http://127.0.0.1:'+server.address().port
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true})
 const page=await browser.newPage({viewport:{width:1440,height:980}});page.setDefaultTimeout(6500)
 const waitEnabled=name=>page.waitForFunction(label=>[...document.querySelectorAll('button')].some(button=>button.textContent.trim()===label&&!button.disabled),name)
 page.on('pageerror',e=>errors.push(e.message))
 await page.route('**/*',async route=>{
  const req=route.request(),url=new URL(req.url()),path=url.pathname
  if(url.origin!==base){forbidden.push(req.method()+' '+req.url());return route.abort()}
  if(req.method()==='POST'){
   if(!['/api/localization/connect','/api/live-planning/start','/api/live-planning/stop'].includes(path)){forbidden.push('POST '+path);return route.abort()}
   assert.deepEqual(req.postDataJSON(),{})
   writes.push(path)
   if(path.endsWith('/connect'))data.connection={phase:'running',active:true,health:{sdk_fresh:true,lidar_fresh:true,replay:false}}
   if(path.endsWith('/start')){data.phase='running';data.busy=true;data.session_id='fixture-owned';data.health={state:'waiting_initial_pose',localized:false};data.stages={localization:{label:'待初值',tone:'waiting'},global_planner:{label:'待定位',tone:'waiting'},local_planner:{label:'待全局路径',tone:'waiting'}}}
   if(path.endsWith('/stop')){data.phase='stopped';data.busy=false;data.session_id=null;data.health={};data.global_status={};data.scan_status={};data.stages={}}
   return route.fulfill({json:snapshot()})
  }
  if(req.method()!=='GET'){forbidden.push(req.method()+' '+path);return route.abort()}
  if(path==='/api/live-planning/overview')return route.fulfill({json:snapshot()})
  if(path==='/api/overview')return route.fulfill({json:{items:[],summary:{map_count:0},processing_job:{running:false}}})
  if(path==='/api/2d/overview')return route.fulfill({json:{versions:[],profiles:[],job:{running:false,status:'idle'},invalid:[]}})
  if(path==='/api/navigation/overview')return route.fulfill({json:{phase:'stopped',busy:false,installed:true,health:{}}})
  if(path.startsWith('/api/')){forbidden.push('Unmocked '+path);return route.abort()}
  return route.continue()
 })
 await page.goto(base+'/#/');await page.locator('.home-planning-entry a[href="#/planning"]').click()
 await page.getByRole('heading',{name:'定位与规划',exact:true}).waitFor()
 assert.equal(await page.locator('.live-planning-page input,.live-planning-page canvas,.live-planning-page form').count(),0)
 assert(await page.getByRole('button',{name:'启动',exact:true}).isDisabled())
 assert(await page.getByRole('button',{name:'停止',exact:true}).isDisabled())
 assert.equal(await page.getByRole('button',{name:/解锁|速度控制|提交定位初值/}).count(),0)
 const configuration=page.locator('.live-configuration')
 assert.equal(await configuration.getAttribute('open'),null)
 assert(await configuration.locator('summary').getByText('运行配置',{exact:true}).isVisible())
 for(const component of Object.values(data.components))assert.equal(await configuration.getByText(component.name,{exact:true}).isVisible(),false)
 await page.screenshot({path:output+'/disconnected.png',fullPage:true})
 checks.push('Home leads to localization and planning; algorithm details collapsed; no Web coordinates, canvas, targets or motion controls')
 await page.getByRole('button',{name:'连接',exact:true}).click()
 await page.getByText('已连接',{exact:true}).waitFor()
 await waitEnabled('启动')
 assert(await page.getByRole('button',{name:'启动',exact:true}).isEnabled())
 await page.getByRole('button',{name:'启动',exact:true}).dblclick()
 await page.getByText('待初值',{exact:true}).waitFor()
 assert.equal(writes.filter(v=>v.endsWith('/start')).length,1)
 assert(await page.getByRole('button',{name:'启动',exact:true}).isDisabled())
 assert(await page.getByRole('button',{name:'停止',exact:true}).isEnabled())
 checks.push('Mocked connect/start works once, duplicate start disabled, RViz initial-pose handoff')
 data.health={state:'tracking',localized:true,navigation:{valid:true}}
 data.global_status={state:'shadow_path_published',active_reference:true}
 data.scan_status={ready:true,active_reference:true,last_spline_id:7,last_spline_stamp:Date.now()/1000}
 data.stages={localization:{label:'已定位',tone:'ready'},global_planner:{label:'已生成',tone:'ready'},local_planner:{label:'已生成',tone:'ready'}}
 await page.getByRole('button',{name:'刷新运行状态'}).click()
 await page.locator('.live-stage').filter({hasText:'局部规划'}).getByText('已生成',{exact:true}).waitFor()
 assert.equal(await page.locator('.live-stage.ready').count(),3)
 assert.equal(await page.locator('.live-stage[data-slot="card"]').count(),0)
 data.components={localization:{name:'替代定位器'},global_planner:{name:'替代全局规划器'},local_planner:{name:'替代局部规划器'}}
 await page.getByRole('button',{name:'刷新运行状态'}).click()
 await configuration.locator('summary').click()
 for(const component of Object.values(data.components))await configuration.getByText(component.name,{exact:true}).waitFor()
 assert(await configuration.getByText('RViz',{exact:true}).isVisible())
 assert.doesNotMatch(await page.locator('.live-planning-page').innerText(),/Faster-LIO|PCT|SCAN/)
 await configuration.locator('summary').click()
 checks.push('Renamed backend components render in configuration with no hardcoded algorithms in the workflow')
 await page.screenshot({path:output+'/running.png',fullPage:true})
 for(const width of [1920,1440,1100,800,390]){
  await page.setViewportSize({width,height:1000})
  assert.deepEqual(await page.locator('.live-planning-page,.live-planning-launch,.live-stage').evaluateAll(nodes=>nodes.filter(n=>n.scrollWidth>n.clientWidth+2).map(n=>n.className)),[],String(width))
 }
 await page.screenshot({path:output+'/mobile.png',fullPage:true})
 checks.push('Three live stages render; no horizontal stage overflow at five viewport sizes')
 data.stages.local_planner.expires_at_unix=Date.now()/1000-10
 await page.getByRole('button',{name:'刷新运行状态'}).click()
 await page.locator('.live-stage').filter({hasText:'局部规划'}).getByText('状态待更新',{exact:true}).waitFor()
 assert.equal(await page.locator('.live-stage.ready').count(),2)
 checks.push('A fresh response cannot renew an explicitly expired local-planner stage')
 data.health={};data.global_status={};data.scan_status={};data.stages={}
 await page.getByRole('button',{name:'刷新运行状态'}).click()
 await page.locator('.live-stage').filter({hasText:'定位'}).getByText('状态待更新',{exact:true}).waitFor()
 assert.equal(await page.locator('.live-stage.ready').count(),0)
 await page.getByRole('button',{name:'停止',exact:true}).click()
 await waitEnabled('启动')
 assert(await page.getByRole('button',{name:'启动',exact:true}).isEnabled())
 assert.equal(writes.filter(v=>v.endsWith('/stop')).length,1)
 checks.push('Missing health cannot remain green; stop is one owned lifecycle request')
 await page.setViewportSize({width:1440,height:980})
 await page.goto(base+'/#/2d/navigation')
 await page.getByRole('heading',{name:'Nav2（独立）',exact:true}).waitFor()
 assert.equal(await page.getByRole('button',{name:'提交定位初值',exact:true}).count(),0)
 await page.locator('.navigation-tabs').getByRole('button',{name:'返回定位与规划',exact:true}).click()
 await page.getByRole('heading',{name:'定位与规划',exact:true}).waitFor()
 checks.push('Legacy Nav2 URL stays independent; old Web initial-pose UI removed')
 assert.deepEqual(errors,[]);assert.deepEqual(forbidden,[])
 const report={checks,errors,forbidden,intercepted_writes:writes,screenshots:output,real_backend_posts:0}
 await writeFile(output+'/report.json',JSON.stringify(report,null,2))
 console.log(JSON.stringify(report,null,2))
}finally{
 if(browser)await browser.close()
 if(server.listening)await new Promise(done=>server.close(done))
}
