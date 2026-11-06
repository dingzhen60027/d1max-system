// Private static fixture only. Every API request is intercepted in the browser.
import assert from 'node:assert/strict'
import {createServer} from 'node:http'
import {readFile,mkdir} from 'node:fs/promises'
import {resolve,extname,sep,dirname} from 'node:path'
import {fileURLToPath} from 'node:url'
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'
const dist=resolve(dirname(fileURLToPath(import.meta.url)),'../dist')
const output=resolve(dirname(fileURLToPath(import.meta.url)),'../artifacts/navigation-launch')
const errors=[],forbidden=[],writes=[]
const state={phase:'stopped',busy:false,owned:false,release_configured:false,quarantined:false,
 can_start:false,can_stop:false,can_view:false,can_connect:false,worker_error:'',startup_error:'',view_error:'',
 purpose:'execution',can_select_purpose:true,
 purpose_capabilities:{planning_only:{can_start:false,can_connect:false},execution:{can_start:false,can_connect:false}},
 connection:{phase:'running',active:true,health:{sdk_fresh:true,lidar_fresh:true,replay:false}}}
const capabilities=(can_start,can_connect)=>{
 state.can_start=can_start;state.can_connect=can_connect
 for(const purpose of ['planning_only','execution'])state.purpose_capabilities[purpose]={can_start,can_connect}
}
const snapshot=()=>({...structuredClone(state),snapshot_at_unix:Date.now()/1000})
const server=createServer(async(req,res)=>{try{
 if(req.method!=='GET'||req.url.startsWith('/api/')){forbidden.push(req.url);res.writeHead(403);return res.end()}
 const path=resolve(dist,'.'+new URL(req.url,'http://fixture').pathname.replace(/\/$/,'/index.html'))
 if(!path.startsWith(dist+sep)){res.writeHead(403);return res.end()}
 res.writeHead(200,{'Content-Type':({'.html':'text/html','.js':'text/javascript','.css':'text/css','.woff2':'font/woff2'})[extname(path)]||'application/octet-stream'})
 res.end(await readFile(path))
}catch{res.writeHead(404);res.end()}})
let browser
try {
 await mkdir(output,{recursive:true});await new Promise(done=>server.listen(0,'127.0.0.1',done))
 const origin='http://127.0.0.1:'+server.address().port
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true})
 const page=await browser.newPage({viewport:{width:1440,height:980}});page.setDefaultTimeout(5000)
 page.on('pageerror',e=>errors.push(e.message))
 await page.route('**/*',async route=>{
  const r=route.request(),url=new URL(r.url())
  if(url.origin!==origin){forbidden.push(r.url());return route.abort()}
  if(url.pathname.startsWith('/api/')){
   if(r.method()==='POST'){
    assert(['connect','start','stop'].some(x=>url.pathname==='/api/navigation-session/'+x))
    const body=r.postDataJSON()
    if(url.pathname.endsWith('/stop'))assert.deepEqual(body,{})
    else {assert.deepEqual(Object.keys(body),['purpose']);assert(['planning_only','execution'].includes(body.purpose))}
    writes.push({path:url.pathname,body})
    if(url.pathname.endsWith('/start'))Object.assign(state,{phase:'preparing',busy:true,owned:false,
     purpose:body.purpose,can_start:false,can_stop:true,can_view:false,worker_operation:'start'})
    else if(url.pathname.endsWith('/connect'))Object.assign(state,{phase:'connecting',busy:true,owned:false,
     purpose:body.purpose,can_start:false,can_connect:false,can_stop:true,worker_operation:'connect'})
    else Object.assign(state,{phase:'stopping',busy:true,can_start:false,can_stop:false,can_view:false,worker_operation:'stop'})
    return route.fulfill({status:202,json:snapshot()})
   }
   if(url.pathname==='/api/navigation-session/overview')return route.fulfill({json:snapshot()})
   if(url.pathname==='/api/overview')return route.fulfill({json:{items:[],summary:{map_count:0},processing_job:{running:false}}})
   if(url.pathname==='/api/2d/overview')return route.fulfill({json:{versions:[],profiles:[],job:{running:false},invalid:[]}})
   forbidden.push(url.pathname);return route.abort()
  }
  return route.continue()
 })
 await page.goto(origin+'/#/planning')
 await page.getByRole('heading',{name:'导航',exact:true}).waitFor()
 const start=page.getByRole('button',{name:'启动',exact:true})
 const stop=page.getByRole('button',{name:'退出',exact:true})
 const openRviz=page.getByRole('button',{name:'打开 RViz',exact:true})
 const mode=page.getByRole('combobox',{name:'启动模式',exact:true})
 const connect=page.getByRole('button',{name:'连接机器狗',exact:true})
 await page.getByText('未启动',{exact:true}).waitFor()
 assert(await start.isDisabled());assert(await stop.isDisabled());assert(await openRviz.isDisabled())
 assert.equal(await mode.innerText(),'仅预览');assert(await mode.isEnabled())
 assert.equal(await page.locator('.live-planning-page form,.live-planning-page canvas').count(),0)
 assert.equal(await page.getByRole('button',{name:'连接机器狗',exact:true}).count(),1)
 assert.equal(await page.locator('.live-planning-page [data-slot="button"]').count(),4)
 assert.doesNotMatch(await page.locator('.live-planning-page').innerText(),/floor1|版本未激活|启动服务|停止服务/)
 await page.screenshot({path:output+'/unactivated.png',fullPage:true})
 state.release_configured=true;capabilities(false,true)
 state.connection={phase:'stopped',active:false,health:{sdk_fresh:false,lidar_fresh:false}}
 await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(x=>x.textContent.trim()==='连接机器狗'&&!x.disabled))
 await connect.click();await page.getByText('连接中',{exact:true}).first().waitFor()
 assert(await mode.isDisabled());assert.equal(await mode.innerText(),'仅预览')
 Object.assign(state,{phase:'stopped',busy:false,can_stop:false,
  connection:{phase:'running',active:true,health:{sdk_fresh:true,lidar_fresh:true}}})
 capabilities(true,false)
 await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(x=>x.textContent.trim()==='启动'&&!x.disabled))
 await start.click();await page.getByText('准备中',{exact:true}).waitFor()
 assert(await mode.isDisabled());assert.equal(await mode.innerText(),'仅预览')
 assert.equal(state.owned,false);assert(await stop.isEnabled());assert(await openRviz.isDisabled())
 state.phase='starting';state.owned=true
 await page.getByText('启动中',{exact:true}).waitFor();assert(await openRviz.isDisabled())
 state.phase='running';state.can_view=true;state.startup_report={phase:'started'}
 await page.getByText('运行中',{exact:true}).waitFor()
 assert(await start.isDisabled());assert(await stop.isEnabled());assert(await openRviz.isEnabled())
 await stop.click();await page.getByText('退出中',{exact:true}).waitFor()
 Object.assign(state,{phase:'stopped',busy:false,owned:false,can_start:true,can_stop:false,can_view:false})
 await page.getByText('可启动',{exact:true}).waitFor();assert(await openRviz.isDisabled())
 assert(await mode.isEnabled());await mode.click();await page.getByRole('option',{name:'运动导航',exact:true}).click()
 assert.equal(await mode.innerText(),'运动导航')
 await start.click();await page.getByText('准备中',{exact:true}).waitFor()
 assert(await mode.isDisabled());assert.equal(await mode.innerText(),'运动导航')
 Object.assign(state,{phase:'running',busy:true,owned:true,can_view:true})
 await page.getByText('运行中',{exact:true}).waitFor()
 // Reload defaults local state back to preview, but must display the actual
 // already-running execution session without another write or hot switch.
 await page.reload();await page.getByText('运行中',{exact:true}).waitFor()
 assert(await mode.isDisabled());assert.equal(await mode.innerText(),'运动导航')
 await page.screenshot({path:output+'/running-execution.png',fullPage:true})
 state.purpose=undefined
 await page.getByText('模式待核对',{exact:true}).waitFor();assert(await mode.isDisabled())
 state.purpose='execution'
 await page.getByText('运动导航',{exact:true}).waitFor()
 assert.deepEqual(writes,[
  {path:'/api/navigation-session/connect',body:{purpose:'planning_only'}},
  {path:'/api/navigation-session/start',body:{purpose:'planning_only'}},
  {path:'/api/navigation-session/stop',body:{}},
  {path:'/api/navigation-session/start',body:{purpose:'execution'}},
 ])
 assert.deepEqual(errors,[]);assert.deepEqual(forbidden,[])
 console.log('Navigation launch private browser fixture passed: preview default, explicit purpose on connect/start, stopped selection, locked actual mode across acceptance/reload, retirement and missing-mode display; every API intercepted.')
} finally {if(browser)await browser.close();await new Promise(done=>server.close(done))}
