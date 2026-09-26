// All API calls intercepted. No robot, production server, or user maps.
import assert from 'node:assert/strict'
import { createServer } from 'node:http'
import { readFile, mkdir } from 'node:fs/promises'
import { resolve, extname, sep } from 'node:path'
import { fileURLToPath } from 'node:url'
import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'

const dist=fileURLToPath(new URL('../dist/',import.meta.url))
const output=fileURLToPath(new URL('../artifacts/mola/',import.meta.url))
const yaml=await readFile(new URL('../../config/mapping/mola_lio_lc.yaml',import.meta.url),'utf8')
const algorithms=[{id:'faster_lio_pgo',name:'Faster-LIO + SC-PGO',input_mode:'live',available:true},
  {id:'mola_lio_lc',name:'MOLA-LIO + 离线回环',input_mode:'rosbag',manual_save:false,available:true}]
const data={items:[],summary:{map_count:0,failure_count:0},comparisons:{},failures:[],processing_configs:[],
  processing_job:{running:false,status:'idle',logs:[]},runtime:{status:'idle',logs:[]},algorithms}
const errors=[],requests=[],unsafe=[]
const server=createServer(async(req,res)=>{
  try{
    if(req.method!=='GET'||req.url.startsWith('/api/')){unsafe.push(req.method+' '+req.url);res.writeHead(403);res.end();return}
    const path=resolve(dist,'.'+new URL(req.url,'http://fixture').pathname.replace(/\/$/,'/index.html'))
    if(!path.startsWith(resolve(dist)+sep)){res.writeHead(403);res.end();return}
    res.writeHead(200,{'Content-Type':{'.html':'text/html','.js':'text/javascript','.css':'text/css','.woff2':'font/woff2'}[extname(path)]||'application/octet-stream'})
    res.end(await readFile(path))
  }catch{res.writeHead(404);res.end()}
})
let browser
try{
  await mkdir(output,{recursive:true})
  await new Promise((done,reject)=>{server.once('error',reject);server.listen(0,'127.0.0.1',done)})
  const base='http://127.0.0.1:'+server.address().port
  browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true})
  const page=await browser.newPage({viewport:{width:1440,height:1000}})
  page.setDefaultTimeout(7000)
  page.on('pageerror',error=>errors.push(error.message))
  await page.route('**/*',async route=>{
    const request=route.request(),url=new URL(request.url()),path=url.pathname
    if(url.origin!==base){unsafe.push(request.url());return route.abort()}
    if(path==='/api/overview')return route.fulfill({json:data})
    if(path==='/api/mapping/profiles/mola_lio_lc')return route.fulfill({json:{yaml,availability:{available:true}}})
    if(path==='/api/mapping/profiles/mola_lio_lc/validate'){
      requests.push({path,...request.postDataJSON()})
      return route.fulfill({json:{yaml,config:{input:{bag_path:'/tmp/fixture-bag'}}}})
    }
    if(path==='/api/runtime/start'){
      requests.push({path,...request.postDataJSON()})
      data.runtime={status:'running',algorithm:'mola_lio_lc',id:'a'.repeat(32),stage:'lio',progress:5,
        run_directory:'/tmp/fixture-mola-run',logs:['[lio] 正在建图']}
      return route.fulfill({json:data.runtime})
    }
    if(path==='/api/runtime/stop'){
      requests.push({path})
      data.runtime={...data.runtime,status:'cancelled',stage:'cancelled',logs:['任务已取消']}
      return route.fulfill({json:data.runtime})
    }
    if(path.startsWith('/api/')||request.method()!=='GET'){unsafe.push(path);return route.abort()}
    return route.continue()
  })
  await page.goto(base+'/#/3d/maps')
  await page.getByRole('button',{name:'建图管理',exact:true}).click()
  const dialog=page.getByRole('dialog')
  assert.equal(await dialog.getByRole('combobox').innerText(),'Faster-LIO + SC-PGO')
  await dialog.getByRole('combobox').click()
  await page.getByRole('option',{name:'MOLA-LIO + 离线回环',exact:true}).click()
  await dialog.getByRole('textbox',{name:'MOLA rosbag 目录'}).fill('/tmp/fixture-bag')
  await dialog.getByText('参数配置 · YAML',{exact:true}).click()
  await dialog.getByRole('textbox',{name:'MOLA 参数配置'}).waitFor()
  assert((await dialog.getByRole('textbox',{name:'MOLA 参数配置'}).inputValue()).includes('assume_planar_world: false'))
  await dialog.getByRole('button',{name:'校验配置',exact:true}).click()
  await page.waitForTimeout(150)
  assert.equal(requests.at(-1).bag_path,'/tmp/fixture-bag')
  for(const width of [1440,800,390]){
    await page.setViewportSize({width,height:1000})
    await page.waitForTimeout(150)
    assert.equal(await dialog.evaluate(n=>n.scrollWidth>n.clientWidth+2),false,`dialog overflow ${width}`)
    await page.screenshot({path:output+`/config-${width}.png`,fullPage:true,animations:'disabled'})
  }
  await page.setViewportSize({width:1440,height:1000})
  await dialog.getByRole('button',{name:'开始离线建图',exact:true}).click()
  await dialog.getByRole('button',{name:'取消任务',exact:true}).waitFor()
  assert.equal(requests.filter(r=>r.path==='/api/runtime/start').length,1)
  assert.equal(requests.at(-1).algorithm,'mola_lio_lc')
  assert.equal(requests.at(-1).options.bag_path,'/tmp/fixture-bag')
  assert(await dialog.getByRole('combobox').isDisabled())
  assert(await dialog.getByRole('button',{name:'阶段完成后自动保存',exact:true}).isDisabled())
  await page.screenshot({path:output+'/running.png',fullPage:true,animations:'disabled'})
  await dialog.getByRole('button',{name:'取消任务',exact:true}).click()
  await dialog.getByText('离线任务已取消',{exact:true}).waitFor()
  assert(await dialog.getByRole('combobox').isEnabled())
  algorithms[1].available=false;algorithms[1].unavailable_reason='测试：依赖不可用'
  await dialog.getByText('测试：依赖不可用',{exact:true}).waitFor()
  assert(await dialog.getByRole('button',{name:'开始离线建图',exact:true}).isDisabled())
  assert.deepEqual(errors,[]);assert.deepEqual(unsafe,[])
  console.log(JSON.stringify({checks:['default preserved','YAML validation','3 responsive widths','start payload','busy switch guard','automatic save','cancel','dependency gate'],requests:requests.map(r=>r.path),errors,unsafe,screenshots:output},null,2))
}finally{
  if(browser)await browser.close()
  if(server.listening)await new Promise(done=>server.close(done))
}
