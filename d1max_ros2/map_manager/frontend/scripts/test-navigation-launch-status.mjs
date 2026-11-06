import test from 'node:test'
import assert from 'node:assert/strict'
import {readFileSync} from 'node:fs'
import {parse} from '@babel/parser'
import {navigationLaunchStatus} from '../src/navigation-launch-status.mjs'
import {createNavigationOverviewCoordinator} from '../src/navigation-overview-coordinator.mjs'
const NOW=1791015000
const view=(data,options={})=>navigationLaunchStatus(data,{nowUnix:NOW,...options})
const ready={phase:'stopped',busy:false,owned:false,snapshot_at_unix:NOW,
 can_start:true,can_stop:false,can_view:false,can_connect:false,
 connection:{active:true,phase:'connected',health:{sdk_fresh:true,lidar_fresh:true},replay:false}}
const active={...ready,phase:'running',busy:true,owned:true,can_start:false,can_stop:true,can_view:true}
const settle=()=>new Promise(done=>setImmediate(done))
const panel=readFileSync(new URL('../src/NavigationLaunch.jsx',import.meta.url),'utf8')
const panelAst=parse(panel,{sourceType:'module',plugins:['jsx']})
const component=panelAst.program.body.find(node=>node.type==='ExportDefaultDeclaration').declaration
const actionNode=component.body.body.flatMap(node=>node.declarations||[]).find(node=>node.id.name==='act').init
const actionCode=panel.slice(actionNode.start,actionNode.end)
function overviewFixture() {
 const requests=[],values=[],errors=[]
 const coordinator=createNavigationOverviewCoordinator({
  read:()=>new Promise((resolve,reject)=>requests.push({resolve,reject})),
  onValue:value=>values.push(value),onError:error=>errors.push(error),
 })
 return {coordinator,requests,values,errors}
}
function actionFixture() {
 const requests=[],writes=[],writing={current:false},alive={current:true}
 const state={data:{...ready,snapshot_at_unix:Date.now()/1000},pending:'',error:'',readError:''}
 const coordinator=createNavigationOverviewCoordinator({
  read:()=>new Promise((resolve,reject)=>requests.push({resolve,reject})),
  onValue:value=>{state.data=value;state.readError='';state.error=''},
  onError:()=>{state.readError='状态读取失败'},
 })
 // Execute the actual component handler against deferred, entirely local I/O.
 const act=Function('navigationLaunchStatus','data','pending','readError','writing','overview',
  'setPending','setError','setData','setReadError','setNowUnix','request','alive','console','return ('+actionCode+')')(
  navigationLaunchStatus,state.data,'','',writing,{current:coordinator},
  value=>{state.pending=value},value=>{state.error=value},value=>{state.data=value},
  value=>{state.readError=value},()=>{},
  (action,write)=>new Promise((resolve,reject)=>writes.push({action,write,resolve,reject})),alive,{error:()=>{}},
 )
 return {act,coordinator,requests,writes,writing,alive,state}
}
test('overview polls coalesce into one bounded read',async()=>{
 const {coordinator,requests,values}=overviewFixture()
 const first=coordinator.refresh()
 for(let i=0;i<20;i++)assert.equal(coordinator.refresh(),first)
 await settle();assert.equal(requests.length,1)
 requests[0].resolve(ready);assert.equal(await first,true)
 assert.deepEqual(values,[ready])
})
test('old overview cannot overwrite an accepted snapshot',async()=>{
 const {coordinator,requests,values}=overviewFixture()
 const old=coordinator.refresh();await settle();coordinator.beginWrite()
 assert.equal(await coordinator.refresh(),false)
 const accepted={...ready,phase:'preparing',can_start:false,can_stop:true}
 values.push(accepted)
 const fresh=coordinator.finishWrite()
 for(let i=0;i<20;i++)coordinator.refresh()
 await settle();assert.equal(requests.length,1)
 requests[0].resolve(ready);assert.equal(await old,false)
 await settle();assert.equal(requests.length,2);assert.deepEqual(values,[accepted])
 requests[1].resolve(accepted);assert.equal(await fresh,true)
 assert.deepEqual(values,[accepted,accepted])
})
test('HTTP acceptance releases the actual handler before either overview completes',async()=>{
 const fixture=actionFixture(),old=fixture.coordinator.refresh();await settle()
 let completed=false
 const operation=fixture.act('start').then(()=>{completed=true})
 assert.equal(fixture.writing.current,true);assert.equal(fixture.state.pending,'start')
 assert.equal(fixture.writes.length,1)
 const accepted={...fixture.state.data,phase:'preparing',can_start:false,can_stop:true}
 fixture.writes[0].resolve(accepted);await settle()
 assert.equal(completed,true);await operation
 assert.equal(fixture.writing.current,false);assert.equal(fixture.state.pending,'')
 assert.equal(fixture.requests.length,1);assert.equal(fixture.state.data,accepted)
 assert.equal(navigationLaunchStatus(fixture.state.data).canStop,true)
 fixture.requests[0].resolve({...ready,snapshot_at_unix:Date.now()/1000})
 assert.equal(await old,false);await settle()
 assert.equal(fixture.requests.length,2);assert.equal(fixture.state.data,accepted)
 const starting={...accepted,phase:'starting'}
 fixture.requests[1].resolve(starting);await settle();assert.equal(fixture.state.data,starting)
 const running=fixture.coordinator.refresh();await settle()
 fixture.requests[2].resolve({...starting,phase:'running',can_view:true})
 await running;assert.equal(navigationLaunchStatus(fixture.state.data).canView,true)
})
test('a rejected request releases the actual handler while authoritative reconciliation is pending',async()=>{
 const fixture=actionFixture()
 const operation=fixture.act('start')
 fixture.writes[0].reject(Error('HTTP timeout'));await operation
 assert.equal(fixture.writing.current,false);assert.equal(fixture.state.pending,'')
 assert.equal(fixture.state.error,'启动请求失败')
 await settle();assert.equal(fixture.requests.length,1)
 const failed={...fixture.state.data,phase:'failed',can_start:false,can_stop:true,worker_error:'准备失败'}
 fixture.requests[0].resolve(failed);await settle()
 assert.equal(fixture.state.error,'');assert.equal(navigationLaunchStatus(fixture.state.data).workerError,'启动失败')
})
test('an obsolete failed GET does not restore a read error after acceptance',async()=>{
 const {coordinator,requests,values,errors}=overviewFixture()
 const old=coordinator.refresh();await settle();coordinator.beginWrite()
 const fresh=coordinator.finishWrite()
 requests[0].reject(Error('old timeout'));assert.equal(await old,false)
 await settle();assert.equal(requests.length,2);assert.deepEqual(errors,[])
 requests[1].resolve(ready);assert.equal(await fresh,true);assert.deepEqual(values,[ready])
})
test('a failed current overview recovers on the next bounded poll',async()=>{
 const {coordinator,requests,values,errors}=overviewFixture()
 coordinator.beginWrite()
 const fresh=coordinator.finishWrite();await settle()
 requests[0].reject(Error('current timeout'));assert.equal(await fresh,false)
 assert.equal(errors.length,1);assert.deepEqual(values,[])
 const recovered=coordinator.refresh();await settle()
 requests[1].resolve(ready);assert.equal(await recovered,true);assert.deepEqual(values,[ready])
})
test('cleanup discards old reads and queued follow-up without creating another request',async()=>{
 const {coordinator,requests,values}=overviewFixture()
 const old=coordinator.refresh();await settle();coordinator.beginWrite()
 const queued=coordinator.finishWrite();coordinator.invalidate()
 assert.equal(await queued,false)
 requests[0].resolve(ready);assert.equal(await old,false)
 await settle();assert.equal(requests.length,1);assert.deepEqual(values,[])
 const resumed=coordinator.refresh();await settle()
 requests[1].resolve(ready);assert.equal(await resumed,true);assert.deepEqual(values,[ready])
})
test('unmounted acceptance does not send a follow-up read',async()=>{
 const fixture=actionFixture(),operation=fixture.act('start')
 fixture.alive.current=false;fixture.coordinator.invalidate()
 fixture.writes[0].resolve({...fixture.state.data,phase:'preparing'});await operation
 await settle();assert.equal(fixture.requests.length,0);assert.equal(fixture.writing.current,false)
})
test('only strict backend capability flags authorize lifecycle operations',()=>{
 const fields={can_start:'canStart',can_stop:'canStop',can_view:'canView',can_connect:'canConnect'}
 for(const [server,client] of Object.entries(fields)){
  assert.equal(view({...ready,[server]:true})[client],true)
  for(const value of [false,undefined,null,0,1,'true','false'])
   assert.equal(view({...ready,[server]:value})[client],false)
 }
 const legacy={...active,can_start:undefined,can_stop:undefined,can_view:undefined,can_connect:undefined,
  release_configured:true,quarantined:false,release:{readiness:'ready'},mainline:{entry_module:'legacy'}}
 for(const client of Object.values(fields))assert.equal(view(legacy)[client],false)
})
test('backend capability admission does not depend on duplicate release metadata',()=>{
 assert.equal(view(ready).canStart,true)
 assert.equal(view({...ready,mainline:undefined,release:undefined,release_configured:undefined}).canStart,true)
 assert.equal(view({...ready,can_start:false,release_configured:true,release:{readiness:'ready'}}).canStart,false)
})
test('preparing can be cancelled before there is an owned unit',()=>{
 const preparing={...ready,phase:'preparing',busy:true,owned:false,can_start:false,can_stop:true}
 assert.equal(view(preparing).label,'准备中')
 assert.equal(view(preparing).canStop,true);assert.equal(view(preparing).canStart,false)
 assert.equal(view(preparing).canView,false)
 assert.equal(view({...preparing,can_stop:false}).canStop,false)
})
test('recovery and retirement follow the backend stop gate',()=>{
 for(const phase of ['starting','running','failed','needs_review','conflict'])
  assert.equal(view({...ready,phase,owned:false,can_start:false,can_stop:true}).canStop,true)
 assert.equal(view({...active,can_stop:false}).canStop,false)
 assert.equal(view({...active,release:{readiness:'invalid'}}).canStop,true)
})
test('all pending HTTP writes prevent duplicate lifecycle operations',()=>{
 const permitted={...ready,can_stop:true,can_view:true,can_connect:true}
 for(const pending of ['start','stop','connect','view/global']){
  const result=view(permitted,{pending})
  for(const field of ['canStart','canStop','canConnect','canView'])assert.equal(result[field],false)
 }
})
test('snapshot freshness uses the original server timestamp',()=>{
 assert.equal(view({...ready,snapshot_at_unix:NOW-5}).canStart,true)
 assert.equal(view({...ready,snapshot_at_unix:NOW-5.001}).canStart,false)
 assert.equal(view(ready,{nowUnix:NOW+5.001}).canStart,false)
 assert.equal(view({...ready,snapshot_at_unix:NOW+.05}).canStart,true)
 assert.equal(view({...ready,snapshot_at_unix:NOW+.101}).canStart,false)
 for(const snapshot_at_unix of [undefined,0,'1791015000',NaN,Infinity,NOW+1])
  assert.equal(view({...ready,snapshot_at_unix}).fresh,false)
 for(const nowUnix of [NaN,Infinity])assert.equal(view(ready,{nowUnix}).fresh,false)
})
test('response receipt advances the UI clock without extending snapshot expiry',()=>{
 const response={...ready,snapshot_at_unix:NOW+.4}
 assert.equal(view(response).fresh,false)
 assert.equal(view(response,{nowUnix:NOW+.4}).fresh,true)
 assert.equal(view({...response,snapshot_at_unix:NOW-6},{nowUnix:NOW+.4}).fresh,false)
 assert.equal((panel.match(/setData\(value\);setReadError\(''\);setNowUnix\(Date\.now\(\)\/1000\)/g)||[]).length,2)
})
test('unknown expired and failed reads disable every capability and hide cached errors',()=>{
 const permitted={...active,can_start:true,can_connect:true,worker_error:'旧错误',view_error:'旧视图错误'}
 for(const [data,options] of [[null,{}],[permitted,{readError:'timeout'}],[permitted,{nowUnix:NOW+6}]]){
  const result=view(data,options)
  for(const field of ['canStart','canStop','canConnect','canView'])assert.equal(result[field],false)
  assert.equal(result.connectionLabel,'连接状态未知');assert.equal(result.workerError,'');assert.equal(result.viewError,'')
 }
})
test('worker failures use concise operation labels while details remain in the console',()=>{
 assert.equal(view({...ready,worker_error:' 准备失败 ',startup_error:'旧启动失败'}).workerError,'启动失败')
 assert.equal(view({...ready,worker_error:'异常堆栈'.repeat(500),worker_operation:'stop'}).workerError,'退出未完成')
 for(const worker_error of ['',null,{},17])
  assert.equal(view({...ready,worker_error,startup_error:' 核心启动失败 '}).workerError,'启动失败')
 assert.equal(view({...ready,worker_error:{error:'不可直接显示'}}).workerError,'')
 assert.match(panel,/const message=view\.workerError\|\|error\|\|/)
 assert.match(panel,/console\.error\('导航生命周期失败',\{/)
 assert.match(panel,/operation:data\.worker_operation,error:data\.worker_error\|\|data\.startup_error/)
 assert.match(panel,/onValue:value=>[^\n]*setError\(''\)/)
})
test('every backend lifecycle phase has a concise label',()=>{
 for(const [phase,label] of Object.entries({preparing:'准备中',starting:'启动中',stopping:'退出中',running:'运行中',
  conflict:'待核对',needs_review:'待核对',failed:'待核对'}))
  assert.equal(view({...ready,phase,can_start:false}).label,label)
 assert.equal(view(ready).label,'可启动')
 assert.equal(view({...ready,can_start:false}).label,'未启动')
})
test('an active monitor does not disguise an explicitly disconnected SDK',()=>{
 const data={...ready,can_start:false,connection:{active:true,phase:'running',health:{sdk_fresh:false,lidar_fresh:false,
  speed_report:{state:'disconnected'}}}}
 assert.equal(view(data).connectionLabel,'机器狗未连接')
 assert.equal(view(data).canConnect,false);assert.equal(view(data).canStart,false)
 for(const health of [{sdk_fresh:false},{sdk_fresh:'false',speed_report:{state:'disconnected'}},
  {sdk_fresh:false,speed_report:{state:'waiting_ready'}}])
  assert.equal(view({...data,connection:{...data.connection,health}}).connectionLabel,'数据未就绪')
 assert.equal(view({...data,connection:{...data.connection,health:{...data.connection.health,replay:true}}}).connectionLabel,'回放数据')
})
test('connection transitions take precedence over an older disconnected report',()=>{
 const disconnected={...ready,connection:{active:false,phase:'stopped',health:{sdk_fresh:false,lidar_fresh:false,
  speed_report:{state:'disconnected'}}}}
 assert.equal(view(disconnected,{pending:'connect'}).connectionLabel,'连接中')
 for(const phase of ['checking','connecting','disconnecting','starting','stopping'])
  assert.equal(view({...disconnected,connection:{...disconnected.connection,phase}}).connectionLabel,'连接中')
 assert.equal(view(ready).connectionLabel,'机器狗已连接')
 assert.equal(view({...ready,connection:{active:true,health:{sdk_fresh:'true',lidar_fresh:true}}}).connectionLabel,'数据未就绪')
 assert.equal(view({...disconnected,can_connect:true}).canConnect,true)
})
test('RViz availability follows the explicit backend startup ACK gate and soft failures remain retryable',()=>{
 assert.equal(view({...active,can_view:false,startup_report:{phase:'started'}}).canView,false)
 assert.equal(view({...ready,phase:'starting',can_view:false}).canView,false)
 const softFailure={...active,view_error:'RViz 未打开'}
 assert.equal(view(softFailure).canView,true);assert.equal(view(softFailure).canStop,true)
 assert.equal(view(softFailure).viewError,'RViz 未打开');assert.equal(view(softFailure).workerError,'')
})
test('planning page keeps one lifecycle API and no client motion or automatic view action',()=>{
 const source=name=>readFileSync(new URL('../src/'+name,import.meta.url),'utf8')
 assert.match(source('LivePlanningPanel.jsx'),/export \{default\} from '\.\/NavigationLaunch\.jsx'/)
 assert.doesNotMatch(source('LivePlanningPanel.jsx'),/PlanningPreview|useState|\/api\/live-planning|\/api\/localization/)
 assert.match(source('App.jsx'),/route\.mode==='planning'\?<LivePlanningPanel\//)
 assert.match(panel,/fetch\('\/api\/navigation-session\/\s*'\+action/)
 assert.doesNotMatch(panel,/\/api\/single-floor|\/api\/live-planning|\/api\/localization|act\('execute'\)|act\('confirm'\)|act\('goal'\)/)
 assert.doesNotMatch(panel,/useEffect\([^\n]*act\('(connect|view\/global)'\)/)
 assert.match(panel,/<h1>导航<\/h1>/)
})
test('JSX preserves explicit controls clock expiry and old read protection without waiting for polling',()=>{
 assert.match(panel,/onClick=\{\(\)=>act\('connect'\)\}/)
 assert.match(panel,/onClick=\{\(\)=>act\('view\/global'\)\}/)
 assert.equal((panel.match(/onClick=\{\(\)=>act\('view\//g)||[]).length,1)
 assert.doesNotMatch(panel,/act\('view\/local'\)|'view\/local':|>全局视图<|>局部视图</)
 assert.match(panel,/nowUnix:Date\.now\(\)\/1000/)
 assert.match(panel,/setInterval\(\(\)=>setNowUnix\(Date\.now\(\)\/1000\),500\)/)
 assert.match(panel,/clearInterval\(clock\)/)
 assert.match(panel,/writing\.current\|\|/)
 assert.match(panel,/createNavigationOverviewCoordinator/)
 assert.match(panel,/overview\.current\.beginWrite\(\)/)
 assert.match(panel,/void overview\.current\.finishWrite\(\{refresh:alive\.current\}\)/)
 assert.doesNotMatch(panel,/await overview\.current\.finishWrite/)
 assert.match(panel,/overview\.current\.invalidate\(\)/)
})
test('launch page keeps four concise controls and distinguishes request rejection from worker failure',()=>{
 assert.equal((panel.match(/<Badge\b/g)||[]).length,1)
 assert.equal((panel.match(/<Button\b/g)||[]).length,4)
 assert.doesNotMatch(panel,/view\.(scope|capabilityLabel|mainlineLabel|versionLabel|blocker)|CardFooter|AlertTitle|RefreshCw|floor1|navigation_runtime|BehaviorTree\.CPP/)
 assert.match(panel,/start:'启动请求失败',stop:'退出请求失败',connect:'连接失败','view\/global':'RViz未打开'/)
 assert.match(panel,/console\.error\('导航操作失败',\{action,error:e,status:e\.status,response:e\.response\}\)/)
 assert.match(panel,/failure\.status=response\.status;failure\.response=value/)
 for(const field of ['canStart','canStop','canConnect','canView'])
  assert.match(panel,new RegExp('disabled=\\{!view\\.'+field+'\\}'))
})
