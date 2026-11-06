// Built UI only. Every API request is intercepted; no ROS, SDK or production backend.
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
let expired=false,failView=false
const data={phase:'stopped',busy:false,owned:false,release_configured:false,
 can_start:false,can_stop:false,can_view:false,can_connect:true,worker_error:'',startup_error:'',worker_operation:null,view_error:'',
 quarantined:false,session_id:null,purpose:'planning_only',motion_capable:false,execution_available:false,
 mainline:{entry_module:'d1max_pct_scan.navigation_session',task_owner:'BehaviorTree.CPP',entrypoint:'/fixture/tools/navigation_entry.sh'},
 release:{selected_id:'fixture-mainline',configured_id:null,running_id:null,readiness:'missing_activation',reason:'未配置主线发布版本',scope:{floors:['floor1'],stairs_enabled:false}},
 start_blocker:'未配置主线发布版本',
 connection:{phase:'stopped',active:false,health:{sdk_fresh:false,lidar_fresh:false,replay:false}}}
const snapshot=()=>({...structuredClone(data),snapshot_at_unix:Date.now()/1000-(expired?10:0)})
const server=createServer(async(req,res)=>{
 try {
  if(req.method!=='GET'||req.url.startsWith('/api/')){forbidden.push(req.method+' '+req.url);res.writeHead(403);res.end();return}
  const target=resolve(dist,'.'+new URL(req.url,'http://fixture').pathname.replace(/\/$/,'/index.html'))
  if(!target.startsWith(resolve(dist)+sep)){res.writeHead(403);res.end();return}
  res.writeHead(200,{'Content-Type':types[extname(target)]||'application/octet-stream'});res.end(await readFile(target))
 }catch{res.writeHead(404);res.end()}
})
let browser
try {
 await mkdir(output,{recursive:true})
 await new Promise(done=>server.listen(0,'127.0.0.1',done))
 const base='http://127.0.0.1:'+server.address().port
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true})
 const page=await browser.newPage({viewport:{width:1440,height:980}});page.setDefaultTimeout(6500)
 page.on('pageerror',e=>errors.push(e.message))
 await page.route('**/*',async route=>{
  const req=route.request(),url=new URL(req.url()),path=url.pathname
  if(url.origin!==base){forbidden.push(req.method()+' '+req.url());return route.abort()}
  if(req.method()==='POST'){
   if(!['/api/navigation-session/connect','/api/navigation-session/start','/api/navigation-session/stop',
    '/api/navigation-session/view/global'].includes(path)){
    forbidden.push('POST '+path);return route.abort()
   }
   assert.deepEqual(req.postDataJSON(),{});writes.push(path)
   if(path.endsWith('/view/global')&&failView){
    data.view_error='fixture technical SDK/session detail'
    return route.fulfill({status:409,json:{detail:data.view_error}})
   }
   if(path.endsWith('/view/global'))data.view_error=''
   if(path.endsWith('/connect')){
    data.connection={phase:'running',active:true,health:{sdk_fresh:true,lidar_fresh:true,replay:false}}
    data.can_connect=false
   }
   if(path.endsWith('/start')){
    Object.assign(data,{phase:'preparing',busy:true,owned:false,can_start:false,can_stop:true,can_view:false,
     can_connect:false,worker_operation:'start',worker_error:'',startup_error:'',view_error:''})
    data.session_id='fixture-owned'
   }
   if(path.endsWith('/stop')){
    Object.assign(data,{phase:'stopping',busy:true,can_start:false,can_stop:false,can_view:false,
     can_connect:false,worker_operation:'stop',worker_error:'',startup_error:'',view_error:''})
   }
   return route.fulfill({status:202,json:snapshot()})
  }
  if(req.method()!=='GET'){forbidden.push(req.method()+' '+path);return route.abort()}
  if(path==='/api/navigation-session/overview')return route.fulfill({json:snapshot()})
  if(path==='/api/overview')return route.fulfill({json:{items:[],summary:{map_count:0},processing_job:{running:false}}})
  if(path==='/api/2d/overview')return route.fulfill({json:{versions:[],profiles:[],job:{running:false,status:'idle'},invalid:[]}})
  if(path.startsWith('/api/')){forbidden.push('Unmocked '+path);return route.abort()}
  return route.continue()
 })
 await page.goto(base+'/#/');await page.locator('.home-planning-entry a[href="#/planning"]').click()
 await page.getByRole('heading',{name:'导航',exact:true}).waitFor()
 const start=page.getByRole('button',{name:'启动',exact:true})
 const stop=page.getByRole('button',{name:'退出',exact:true})
 const openRviz=page.getByRole('button',{name:'打开 RViz',exact:true})
 assert.equal(await openRviz.count(),1)
 assert.equal(await page.getByRole('button',{name:/^(全局视图|局部视图)$/}).count(),0)
 assert(await start.isDisabled());assert(await stop.isDisabled());assert(await openRviz.isDisabled())
 assert.equal(await page.locator('.navigation-profile-selector,.live-planning-page input,.live-planning-page canvas,.live-planning-page form').count(),0)
 assert.equal(await page.getByRole('button',{name:/解锁|速度控制|提交定位初值|确认执行/}).count(),0)
 await page.getByText('未启动',{exact:true}).waitFor()
 assert.doesNotMatch(await page.locator('.live-planning-page').innerText(),/floor1|仅规划|版本|BehaviorTree|未配置主线发布版本|启动不执行运动|目标与执行确认/)
 assert.equal(await page.locator('.live-planning-heading [data-slot=badge]').count(),1)
 assert.deepEqual(writes,[])
 await page.screenshot({path:output+'/mainline-unconfigured.png',fullPage:true})
 checks.push('Built entry is the unique mainline; unconfigured release cannot launch or fall back')

 await page.getByRole('button',{name:'连接机器狗',exact:true}).click()
 await page.getByText('机器狗已连接',{exact:true}).waitFor()
 assert(await start.isDisabled());assert.equal(writes.length,1)
 data.release_configured=true;data.can_start=true;data.release.readiness='ready'
 data.release.configured_id='fixture-mainline';data.release.reason='发布版本校验通过';data.start_blocker=''
 await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(b=>b.textContent.trim()==='启动'&&!b.disabled))
 await start.click();await page.getByText('准备中',{exact:true}).waitFor()
 assert.equal(data.owned,false);assert(await stop.isEnabled());assert(await openRviz.isDisabled())
 await stop.click();await page.getByText('退出中',{exact:true}).waitFor()
 assert(await stop.isDisabled());assert(await openRviz.isDisabled())
 Object.assign(data,{phase:'stopped',busy:false,owned:false,can_start:true,can_stop:false,can_view:false,session_id:null})
 await page.getByText('可启动',{exact:true}).waitFor()
 checks.push('202 acceptance releases local pending; preparing can be cancelled before a unit is owned')

 await start.click();await page.getByText('准备中',{exact:true}).waitFor()
 Object.assign(data,{phase:'failed',busy:false,owned:false,can_stop:true,
  worker_error:'fixture worker start stack detail '.repeat(20),startup_error:'fixture legacy startup detail'})
 await page.getByText('启动失败',{exact:true}).waitFor()
 assert(!(await page.locator('.live-planning-page').innerText()).includes('fixture worker start stack detail'))
 assert(!(await page.locator('.live-planning-page').innerText()).includes('fixture legacy startup detail'))
 assert(await stop.isEnabled());assert(await start.isDisabled())
 await stop.click();await page.getByText('退出中',{exact:true}).waitFor()
 Object.assign(data,{phase:'needs_review',busy:false,can_stop:true,worker_error:'fixture worker stop detail '.repeat(20)})
 await page.getByText('退出未完成',{exact:true}).waitFor()
 assert(!(await page.locator('.live-planning-page').innerText()).includes('fixture worker stop detail'))
 await stop.click();await page.getByText('退出中',{exact:true}).waitFor()
 Object.assign(data,{phase:'stopped',busy:false,owned:false,can_start:true,can_stop:false,can_view:false,session_id:null})
 await page.getByText('可启动',{exact:true}).waitFor()
 checks.push('Worker start and stop failures stay concise and permit backend-authorized recovery')

 await start.click();await page.getByText('准备中',{exact:true}).waitFor()
 data.phase='starting';data.owned=true
 await page.getByText('启动中',{exact:true}).waitFor();assert(await openRviz.isDisabled())
 data.phase='running';data.can_view=true;data.startup_report={phase:'started'};data.release.running_id='fixture-mainline'
 await page.getByText('运行中',{exact:true}).waitFor()
 assert(await start.isDisabled());assert(await stop.isEnabled())
 assert(await openRviz.isEnabled());assert.equal(writes.filter(v=>v.endsWith('/start')).length,3)
 assert.deepEqual(writes.filter(v=>v.includes('/view/')),[])
 Object.assign(data,{can_start:'true',can_stop:'true',can_view:'true',can_connect:'true'})
 await page.waitForResponse(async response=>new URL(response.url()).pathname==='/api/navigation-session/overview'&&
  (await response.json()).can_stop==='true')
 await page.waitForFunction(()=>[...document.querySelectorAll('.live-planning-page button')].every(button=>button.disabled))
 Object.assign(data,{can_start:false,can_stop:true,can_view:true,can_connect:false})
 await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(button=>button.textContent.trim()==='打开 RViz'&&!button.disabled))
 checks.push('Only strict backend can_* booleans enable buttons; RViz is available after the core ACK')
 await Promise.all([
  page.waitForResponse(response=>new URL(response.url()).pathname==='/api/navigation-session/view/global'&&response.request().method()==='POST'),
  openRviz.click(),
 ])
 assert.deepEqual(writes.filter(v=>v.includes('/view/')),['/api/navigation-session/view/global'])
 assert.equal(data.phase,'running')
 await page.getByText('运行中',{exact:true}).waitFor();assert(await stop.isEnabled())
 await page.screenshot({path:output+'/mainline-running.png',fullPage:true})
 checks.push('Explicit mocked connection/start and one Open RViz entry with one global view write')

 for(const width of [1920,1440,1100,800,390]){
  await page.setViewportSize({width,height:1000})
  const overflow=await page.locator('.live-planning-page,.live-planning-heading,.navigation-launch-actions,.live-planning-launch').evaluateAll(
   nodes=>nodes.filter(n=>n.scrollWidth>n.clientWidth+2).map(n=>n.className))
  assert.deepEqual(overflow,[],String(width))
 }
 await page.screenshot({path:output+'/mainline-mobile.png',fullPage:true})
 checks.push('No horizontal overflow at five viewport sizes')
 failView=true;await openRviz.click();await page.getByText('RViz未打开',{exact:true}).waitFor()
 assert(!(await page.locator('.live-planning-page').innerText()).includes('fixture technical SDK/session detail'))
 assert.equal(data.phase,'running');failView=false;data.view_error=''
 checks.push('Backend view failure stays truthful and concise; technical details are not printed in the UI')
 expired=true;await page.getByText('状态未知',{exact:true}).waitFor()
 assert(await stop.isDisabled());assert(await openRviz.isDisabled())
 expired=false;await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(b=>b.textContent.trim()==='退出'&&!b.disabled))
 await stop.click();await page.getByText('退出中',{exact:true}).waitFor()
 Object.assign(data,{phase:'stopped',busy:false,owned:false,can_start:true,can_stop:false,can_view:false,session_id:null})
 data.release.running_id=null
 await page.getByText('可启动',{exact:true}).waitFor()
 assert.equal(writes.filter(v=>v.endsWith('/stop')).length,4);assert(await openRviz.isDisabled())
 data.release.readiness='invalid';data.release.reason='发布文件已变化';data.can_start=false;data.release_configured=false
 await page.getByText('未启动',{exact:true}).waitFor();assert(await start.isDisabled())
 assert(!(await page.locator('.live-planning-page').innerText()).includes('发布文件已变化'))
 checks.push('Expired reads and backend admission deny controls; retirement completes through polling')
 assert.deepEqual(errors,[]);assert.deepEqual(forbidden,[])
 const report={checks,errors,forbidden,intercepted_writes:writes,screenshots:output,
  fixture_only:true,real_backend_posts:0,ros_initialized:false,sdk_connected:false}
 await writeFile(output+'/report.json',JSON.stringify(report,null,2))
 console.log(JSON.stringify(report,null,2))
} finally {
 if(browser)await browser.close()
 if(server.listening)await new Promise(done=>server.close(done))
}
