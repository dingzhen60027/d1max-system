// Isolated UI fixtures only: no production backend, ROS, robot or disk-map writes.
import assert from 'node:assert/strict'
import { createServer } from 'node:http'
import { readFile, mkdir } from 'node:fs/promises'
import { extname, resolve, sep } from 'node:path'
import { fileURLToPath } from 'node:url'
import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'

const dist=fileURLToPath(new URL('../dist/',import.meta.url))
const output=fileURLToPath(new URL('../artifacts/copy-cleanup/',import.meta.url))
const svg='<svg xmlns="http://www.w3.org/2000/svg" width="640" height="400"><path fill="#ddd" d="M0 0h640v400H0z"/><path fill="white" stroke="black" stroke-width="10" d="M60 40h520v320H60z"/><path stroke="black" stroke-width="8" d="M240 40v200m180 120V180"/></svg>'
const params={filter_enabled:false,statistical_mean_k:20,statistical_std_dev_mul:.5,radius:.3,radius_min_points:4,z_min:.4,z_max:1.5,resolution:.05,padding:.5,min_points_per_cell:1,background:'unknown'}
const version={id:'grid-'+'1'.repeat(24),name:'测试单层地图',source_name:'测试原始点云',selected:true,archived:false,complete:true,width:640,height:400,metadata:{resolution:.05,origin:[0,0,0]},parameters:params,map_preview_url:'/fixture/map.svg',created_at:'2026-09-11T08:00:00',origin:'pcd',occupied_cells:2000,unknown_cells:1000,warning:'静态 PCD 无观测射线；自由栅格不等于已验证可通行，请检查楼层、墙体和机器人尺寸',download_url:'/api/2d/versions/fixture/download'}
const source={id:'fixture-raw',name:'测试原始点云',category:'maps',role:'optimized',family:'Faster-LIO + SC-PGO',archived:false,extension:'pcd',relative_path:'fixture/map.pcd',path:'/fixture/map.pcd',points:1000,size:16000,size_human:'16 KB',storage:'binary',modified_at:'2026-09-11T08:00:00',issues:['SC-PGO 未接受回环约束'],pair_key:'fixture',accepted_loops:0,rejected_loops:8,cloud_url:'/fixture/cloud.ply',fields:['x','y','z','intensity']}
const config={id:'fixture',name:'建筑结构保留',description:'流水线说明不应常驻',recommended:true,modules:[{id:'voxel',type:'voxel_downsample',enabled:true,parameters:{voxel_size:.1}}]}
const raw={items:[source,{...source,id:'fixture-archive',name:'测试归档点云',archived:true}],summary:{map_count:1,failure_count:0},comparisons:{},failures:[],processing_configs:[config],processing_job:{running:false,status:'idle',logs:[]},runtime:{status:'idle',logs:[]},algorithms:[{id:'faster_lio_pgo',name:'Faster-LIO + SC-PGO'}]}
const grid={versions:[version,{...version,id:'grid-'+'2'.repeat(24),selected:false,archived:true}],profiles:[{id:'conservative',name:'保守方案',parameters:params}],job:{running:false,status:'idle'},invalid:[]}
const localization={phase:'stopped',backend:'lio_pcd',installed:true,connection:{active:false,health:{}},health:{},logs:[]}
const unsafe=[],errors=[],checks=[]
const types={'.html':'text/html','.js':'text/javascript','.css':'text/css','.woff2':'font/woff2'}
const server=createServer(async(req,res)=>{
 try{
  if(req.method!=='GET'||req.url.startsWith('/api/')){unsafe.push(req.method+' '+req.url);res.writeHead(403);res.end();return}
  const path=resolve(dist,'.'+new URL(req.url,'http://fixture').pathname.replace(/\/$/,'/index.html'))
  if(!path.startsWith(resolve(dist)+sep)){res.writeHead(403);res.end();return}
  const body=await readFile(path);res.writeHead(200,{'Content-Type':types[extname(path)]||'application/octet-stream'});res.end(body)
 }catch{res.writeHead(404);res.end()}
})
let browser
try{
 await mkdir(output,{recursive:true})
 await new Promise((done,reject)=>{server.once('error',reject);server.listen(0,'127.0.0.1',done)})
 const base='http://127.0.0.1:'+server.address().port
 browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader']})
 const page=await browser.newPage({viewport:{width:1600,height:1000}});page.setDefaultTimeout(6000)
 page.on('pageerror',error=>errors.push(error.message))
 await page.route('**/*',async route=>{
  const request=route.request(),url=new URL(request.url()),path=url.pathname
  if(url.origin!==base||request.method()!=='GET'){unsafe.push(request.method()+' '+request.url());return route.abort()}
  if(path==='/fixture/map.svg')return route.fulfill({contentType:'image/svg+xml',body:svg})
  if(path==='/fixture/cloud.ply')return route.fulfill({body:'ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n5 0 0\n5 5 2\n0 5 1\n'})
  if(path==='/api/overview')return route.fulfill({json:raw})
  if(path==='/api/2d/overview')return route.fulfill({json:grid})
  if(path==='/api/localization/overview')return route.fulfill({json:localization})
  if(path.startsWith('/api/')){unsafe.push('Unmocked '+path);return route.abort()}
  return route.continue()
 })
 const go=async(path,title)=>{await page.goto(base+'/#'+path);await page.getByRole('heading',{name:title,exact:true}).waitFor();await page.waitForTimeout(100)}
 const shot=async name=>{await page.waitForTimeout(200);return page.screenshot({path:output+'/'+name+'.png',fullPage:true,animations:'disabled'})}
 await go('/','地图工作台');assert.equal(await page.locator('.workspace-choice').count(),2);await shot('home')
 assert(!(await page.locator('body').innerText()).includes('选择你的工作空间'))
 for(const [path,title] of [['/2d/versions','2D 地图版本'],['/2d/archived','2D 归档'],['/3d/maps','原始建图点云'],['/3d/processed','点云处理结果'],['/3d/planning','规划派生数据'],['/3d/archived','3D 归档'],['/3d/failures','运行异常记录']]){
  await go(path,title);assert.equal(await page.locator('.workspace-heading p,.workspace-heading .eyebrow').count(),0)
 }
 checks.push('2D/3D headings have no explanatory subtitles')
 await go('/2d/build','从 PCD 生成 2D 地图')
 await page.getByRole('combobox',{name:'源 PCD',exact:true}).waitFor()
 assert.equal(await page.locator('.grid-build-form [data-slot=card-description]').count(),0)
 const files=page.locator('.output-receipt');assert.equal(await files.getAttribute('open'),null)
 assert.equal(await files.getByText('localization.pcd · 定位点云',{exact:true}).isVisible(),false)
 await files.locator('summary').click();assert(await files.getByText('localization.pcd · 定位点云',{exact:true}).isVisible());await files.locator('summary').click()
 assert(await page.getByText('Z 使用 PCD 坐标，非离地高度。',{exact:true}).isVisible())
 assert(await page.getByRole('button',{name:'生成并保存候选版',exact:true}).isEnabled());await shot('build')
 checks.push('Build controls and coordinate units retained; output files expand on demand')
 await go('/2d/versions','2D 地图版本');await page.getByRole('button',{name:'修整 2D 地图',exact:true}).waitFor();await shot('versions')
 await page.getByText('通行性待核对',{exact:true}).click();assert(await page.getByText(version.warning,{exact:true}).isVisible())
 await page.getByRole('button',{name:'修整 2D 地图',exact:true}).click();await page.locator('.map-editor canvas.ready').waitFor()
 assert(await page.getByRole('button',{name:'擦除杂点 标记为可通行',exact:true}).isVisible())
 assert(await page.getByRole('button',{name:'画墙',exact:true}).isVisible());await shot('editor');await page.getByRole('button',{name:'关闭编辑器',exact:true}).click()
 checks.push('Editor tools and PCD preservation label retained; traversability warning expands on demand')
 await go('/2d/archived','2D 归档');await page.getByRole('button',{name:'全部删除',exact:true}).click()
 assert(await page.getByText(/操作无法恢复/).isVisible());assert(await page.getByRole('button',{name:'确认永久删除',exact:true}).isVisible());await page.getByRole('button',{name:'取消',exact:true}).click()
 await go('/3d/archived','3D 归档');await page.getByRole('button',{name:'全部删除',exact:true}).click()
 assert(await page.getByText(/无法恢复；处理结果的/).isVisible());await page.getByRole('button',{name:'取消',exact:true}).click()
 checks.push('2D/3D irreversible deletion confirmations retained; no deletion submitted')
 await go('/3d/maps','原始建图点云');await page.getByRole('button',{name:'配置参数并处理 PCD',exact:true}).click()
 await page.getByRole('heading',{name:'生成点云处理结果',exact:true}).waitFor()
 assert.equal(await page.locator('.processing-note').count(),0);assert.equal(await page.locator('.parameter-switch small').count(),0)
 assert(await page.getByRole('button',{name:'开始处理',exact:true}).isEnabled());await shot('processing');await page.getByRole('button',{name:'取消',exact:true}).click()
 checks.push('Processing parameters and actions retained; module IDs and implementation prose removed')
 await go('/2d/navigation','单楼层定位调试');await page.getByRole('button',{name:'启动定位',exact:true}).waitFor()
 assert.equal(await page.locator('.localization-top [data-slot=card-description]').count(),0)
 assert(await page.getByRole('button',{name:'启动定位',exact:true}).isDisabled());assert(await page.getByRole('button',{name:'提交定位初值',exact:true}).isDisabled())
 assert(await page.getByText('ws://127.0.0.1:8769',{exact:true}).isVisible());await shot('localization')
 localization.connection.error='测试：机器人链路断开'
 localization.health={state:'waiting_sensors',last_error:'测试：速度流未到达',warnings:['测试：机身外参未标定'],input_clock:{ready:'false',message:'测试：时间戳异常'}}
 await page.getByText('测试：机器人链路断开',{exact:true}).waitFor()
 assert(await page.getByText('测试：速度流未到达',{exact:true}).isVisible());assert(await page.getByText('测试：时间戳异常',{exact:true}).isVisible())
 const limits=page.getByText('定位限制 · 1',{exact:true});await limits.click();assert(await page.getByText('测试：机身外参未标定',{exact:true}).isVisible());await limits.click()
 checks.push('Connection, initial-pose gating, failure reasons and expandable limitations retained')
 for(const width of [1920,1440,1100,800,390]){
  await page.setViewportSize({width,height:1000});await page.waitForTimeout(100)
  assert.deepEqual(await page.locator('.localization-page,.localization-top [data-slot=card],.localization-seed').evaluateAll(nodes=>nodes.filter(n=>n.scrollWidth>n.clientWidth+3).map(n=>n.className)),[],String(width))
 }
 checks.push('Localization layout has no horizontal overflow at five viewport widths')
 assert(await page.getByText('启动 LIO + PCD 定位',{exact:false}).isVisible())
 localization.phase='running';localization.health={state:'relocalizing',local_backend:'faster_lio',initial_pose_ready:false,sensors:{imu:true,cloud:true,sdk:false},local_ekf_fresh:true,global_ekf_fresh:false}
 await page.getByText('受限重定位中',{exact:true}).waitFor()
 assert(await page.getByText('局部 LIO',{exact:false}).isVisible())
 assert(await page.getByRole('button',{name:'提交定位初值',exact:true}).isDisabled())
 localization.health={...localization.health,state:'fault',local_fault:'local_pose_jump',local_ekf_fresh:false}
 await page.getByRole('status').getByText('局部里程计已停止',{exact:true}).waitFor()
 assert(await page.getByRole('status').getByText('原因：local_pose_jump。请核对定位日志；未自动绕过检查。',{exact:true}).isVisible())
 assert(await page.getByRole('button',{name:'提交定位初值',exact:true}).isDisabled())
 checks.push('LIO backend, relocalizing and fault states render without false ready controls')
 assert.deepEqual(errors,[]);assert.deepEqual(unsafe,[])
 console.log(JSON.stringify({checks,errors,unsafe,screenshots:output},null,2))
}finally{
 if(browser)await browser.close()
 if(server.listening)await new Promise(done=>server.close(done))
}
