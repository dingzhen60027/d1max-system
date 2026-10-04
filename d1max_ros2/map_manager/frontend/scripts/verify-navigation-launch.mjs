// Private static fixture only. Every API request is intercepted in the browser.
import assert from 'node:assert/strict'
import {createServer} from 'node:http'
import {readFile,mkdir} from 'node:fs/promises'
import {resolve,extname,sep,dirname} from 'node:path'
import {fileURLToPath} from 'node:url'
import {chromium} from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'
const root='/home/dndx/d1max_nav_ws/experiments/single_floor_execution_20260927'
const dist=resolve(dirname(fileURLToPath(import.meta.url)),'../dist'),output=resolve(root,'web_ui')
const errors=[],forbidden=[],writes=[]
const state={phase:'stopped',busy:false,release_configured:false,quarantined:false,scope:{floors:['floor1'],stairs_enabled:false}}
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
    assert(['start','stop'].some(x=>url.pathname==='/api/single-floor/'+x));assert.deepEqual(r.postDataJSON(),{})
    writes.push(url.pathname);state.busy=url.pathname.endsWith('/start');state.phase=state.busy?'running':'stopped'
    return route.fulfill({json:state})
   }
   if(url.pathname==='/api/single-floor/overview')return route.fulfill({json:state})
   if(url.pathname==='/api/live-planning/overview')return route.fulfill({json:{phase:'stopped',busy:false,installed:false,mode:'LIVE_VISUALIZATION_NO_MOTION',motion_enabled:false,connection:{}}})
   if(url.pathname==='/api/overview')return route.fulfill({json:{items:[],summary:{map_count:0},processing_job:{running:false}}})
   if(url.pathname==='/api/2d/overview')return route.fulfill({json:{versions:[],profiles:[],job:{running:false},invalid:[]}})
   if(url.pathname==='/api/navigation/overview')return route.fulfill({json:{phase:'stopped',busy:false,installed:true,health:{}}})
   forbidden.push(url.pathname);return route.abort()
  }
  return route.continue()
 })
 await page.goto(origin+'/#/planning')
 await page.getByRole('heading',{name:'定位与规划',exact:true}).waitFor()
 await page.getByRole('button',{name:'导航',exact:true}).click()
 await page.getByRole('heading',{name:'导航',exact:true}).waitFor()
 await page.getByText('版本未激活',{exact:true}).waitFor()
 assert(await page.getByRole('button',{name:'启动服务',exact:true}).isDisabled())
 assert.equal(await page.locator('.live-planning-page input,.live-planning-page form,.live-planning-page canvas').count(),0)
 assert.equal(await page.getByRole('button',{name:'连接',exact:true}).count(),0)
 await page.getByText('floor1',{exact:true}).waitFor()
 await page.screenshot({path:output+'/unactivated.png',fullPage:true})
 state.release_configured=true;await page.getByRole('button',{name:'刷新导航服务状态'}).click()
 await page.waitForFunction(()=>[...document.querySelectorAll('button')].some(x=>x.textContent==='启动服务'&&!x.disabled))
 await page.getByRole('button',{name:'启动服务',exact:true}).click();await page.getByText('运行中',{exact:true}).waitFor()
 assert(await page.getByRole('button',{name:'启动服务',exact:true}).isDisabled())
 await page.getByRole('button',{name:'停止服务',exact:true}).click();await page.getByText('未启动',{exact:true}).waitFor()
 assert.deepEqual(writes,['/api/single-floor/start','/api/single-floor/stop'])
 assert.deepEqual(errors,[]);assert.deepEqual(forbidden,[])
 console.log('Navigation launch private browser fixture passed: default preview, disabled unactivated profile, mocked start/stop only; no external requests.')
} finally {if(browser)await browser.close();await new Promise(done=>server.close(done))}
