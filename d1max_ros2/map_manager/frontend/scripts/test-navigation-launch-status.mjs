import test from 'node:test'
import assert from 'node:assert/strict'
import {readFileSync} from 'node:fs'
import {parse} from '@babel/parser'
import {navigationLaunchStatus,scopeLabel} from '../src/navigation-launch-status.mjs'
const NOW=1791015000
const view=(data,options={})=>navigationLaunchStatus(data,{nowUnix:NOW,...options})
const ready={phase:'stopped',busy:false,owned:false,release_configured:true,can_start:true,quarantined:false,
 snapshot_at_unix:NOW,mainline:{entry_module:'d1max_pct_scan.navigation_session',task_owner:'BehaviorTree.CPP',entrypoint:'tools/navigation_entry.sh'},
 release:{selected_id:'r3',configured_id:'r3',running_id:'',readiness:'ready',manifest_sha256:'a'.repeat(64),scope:{floors:['floor1'],stairs_enabled:false}},
 connection:{active:true,phase:'connected',health:{sdk_fresh:true,lidar_fresh:true},replay:false}}
test('unactivated profile cannot launch',()=>assert.equal(view({...ready,release_configured:false}).canStart,false))
test('only configured stopped profile may start',()=>assert.equal(view(ready).canStart,true))
test('unknown and stale reads cannot launch',()=>{assert.equal(view(null).canStart,false);assert.equal(view(ready,{readError:'timeout'}).canStart,false)})
test('duplicate start is disabled while pending or running',()=>{assert.equal(view(ready,{pending:'start'}).canStart,false);assert.equal(view({...ready,phase:'running',busy:true}).canStart,false)})
test('foreign conflict is not stopped by UI',()=>assert.equal(view({...ready,phase:'conflict',busy:true}).canStop,false))
test('quarantined state cannot be relaunched',()=>assert.equal(view({...ready,quarantined:true}).canStart,false))
test('owned running service can stop',()=>assert.equal(view({...ready,phase:'running',busy:true,owned:true}).canStop,true))
test('pending writes disable duplicate stop',()=>assert.equal(view({...ready,phase:'running',busy:true},{pending:'stop'}).canStop,false))
test('scope label follows release coverage, not a fixed profile',()=>{
 assert.equal(scopeLabel({floors:['floor1'],stairs_enabled:false}),'floor1')
 assert.equal(scopeLabel({floors:['floor1','floor2'],stairs_enabled:true}),'floor1 · floor2 · 含楼梯')
 assert.equal(scopeLabel(null),'');assert.equal(view(ready).scope,'floor1')
 assert.equal(view({...ready,scope:{floors:['floor1','floor2'],stairs_enabled:true}}).scope,'floor1')
 assert.equal(view({...ready,release:{...ready.release,scope:undefined},scope:{floors:['floor1']}}).scope,'')
 assert.equal(view({...ready,release:{...ready.release,scope:{floors:['floor1','floor2'],stairs_enabled:true}}}).scope,'floor1 · floor2 · 含楼梯')
})
test('planning page has one release-backed lifecycle entry, with no legacy fallback',()=>{
 const source=name=>readFileSync(new URL('../src/'+name,import.meta.url),'utf8')
 assert.match(source('LivePlanningPanel.jsx'),/export \{default\} from '\.\/NavigationLaunch\.jsx'/)
 assert.doesNotMatch(source('LivePlanningPanel.jsx'),/PlanningPreview|useState|\/api\/live-planning|\/api\/localization/)
 assert.match(source('LocalizationPanel.jsx'),/LivePlanningPanel\.jsx/)
 assert.match(source('App.jsx'),/route\.mode==='planning'\?<LivePlanningPanel\//)
 const panel=source('NavigationLaunch.jsx')
 assert.match(panel,/fetch\('\/api\/navigation-session\/\s*'\+action/)
 assert.doesNotMatch(panel,/\/api\/single-floor|\/api\/live-planning|\/api\/localization|act\('execute'\)|act\('confirm'\)|act\('goal'\)/)
 assert.match(panel,/<h1>导航<\/h1>/)
 assert.match(panel,/启动导航服务/)
 assert.match(panel,/启动不执行运动/)
 assert.match(panel,/目标与执行确认 · RViz/)
})
test('missing purpose or capability fields never imply physical execution',()=>{
 for(const fields of [{},{motion_capable:true,execution_available:true},
   {purpose:'execution',execution_available:true},
   {purpose:'execution',motion_capable:true},
   {purpose:'unknown',motion_capable:true,execution_available:true}]){
  const result=view({...ready,...fields})
  assert.equal(result.executionAvailable,false)
  assert.equal(result.capabilityLabel,'运动未就绪')
 }
})
test('planning-only never grants motion regardless of contradictory capability flags',()=>{
 const result=view({...ready,purpose:'planning_only',motion_capable:true,execution_available:true})
 assert.equal(result.canStart,true)
 assert.equal(result.executionAvailable,false)
 assert.equal(result.purpose,'planning_only')
})
test('only explicit execution availability is displayed and is not an execute action',()=>{
 const data={...ready,purpose:'execution',motion_capable:true,execution_available:true}
 assert.equal(view(data).executionAvailable,true)
 assert.equal(view(data).capabilityLabel,'执行需 RViz 确认')
 assert.equal(view(data,{readError:'timeout'}).executionAvailable,false)
 assert.equal(view({...data,motion_capable:'true'}).executionAvailable,false)
 assert.equal(view({...data,execution_available:'true'}).executionAvailable,false)
})

test('legacy flags or string booleans cannot replace authoritative can_start',()=>{
 assert.equal(view({phase:'stopped',busy:false,release_configured:true}).canStart,false)
 for(const fields of [{can_start:undefined},{can_start:'true'},{release_configured:'true'},{busy:'false'}])
  assert.equal(view({...ready,...fields}).canStart,false)
 const result=view({...ready,can_start:false,start_blocker:'机器狗数据尚未就绪'})
 assert.equal(result.canStart,false);assert.equal(result.blocker,'机器狗数据尚未就绪')
})
test('missing invalid or checking release cannot launch despite contradictory flags',()=>{
 for(const readiness of ['missing_activation','invalid','checking',undefined,'READY',true]){
  const result=view({...ready,release:{...ready.release,readiness,reason:'版本尚未验收'},
   purpose:'execution',motion_capable:true,execution_available:true})
  assert.equal(result.canStart,false);assert.equal(result.executionAvailable,false)
  assert.equal(result.blocker,'版本尚未验收')
 }
 assert.equal(view({...ready,release:undefined}).canStart,false)
})
test('wrong or missing module owner or entrypoint cannot claim the mainline ready',()=>{
 for(const mainline of [undefined,{}, {...ready.mainline,entry_module:'old_preview'},
  {...ready.mainline,task_owner:'motion_coordinator'}, {...ready.mainline,entrypoint:''}])
  assert.equal(view({...ready,mainline}).canStart,false)
})
test('legacy session metadata remains an explicit compatible mainline',()=>{
 const mainline={...ready.mainline,entry_module:'d1max_pct_scan.single_floor_session',entrypoint:'tools/single_floor_entry.sh'}
 assert.equal(view({...ready,mainline}).canStart,true)
 assert.equal(view({...ready,mainline:{...mainline,task_owner:'motion_coordinator'}}).canStart,false)
})
test('snapshot freshness uses original server timestamp not response receipt',()=>{
 assert.equal(view({...ready,snapshot_at_unix:NOW-5}).canStart,true)
 assert.equal(view({...ready,snapshot_at_unix:NOW-5.001}).canStart,false)
 assert.equal(view(ready,{nowUnix:NOW+5.001}).canStart,false)
 assert.equal(view(ready,{nowUnix:NOW+5.001}).blocker,'状态已过期，请刷新')
 assert.equal(view({...ready,snapshot_at_unix:NOW+.05}).canStart,true)
 assert.equal(view({...ready,snapshot_at_unix:NOW+.101}).canStart,false)
 for(const snapshot_at_unix of [undefined,0,'1791015000',NaN,Infinity,NOW+1])
  assert.equal(view({...ready,snapshot_at_unix}).canStart,false)
 for(const nowUnix of [NaN,Infinity])assert.equal(view(ready,{nowUnix}).canStart,false)
})
test('failed reads do not display cached execution or connection readiness',()=>{
 const result=view({...ready,purpose:'execution',motion_capable:true,execution_available:true},{readError:'状态读取超时'})
 assert.equal(result.blocker,'状态读取超时');assert.equal(result.connectionLabel,'连接状态未知')
 assert.equal(result.executionAvailable,false)
})
test('all pending writes prevent duplicate start stop connect and view operations',()=>{
 for(const pending of ['start','stop','connect','view/global','view/local']){
  assert.equal(view(ready,{pending}).canStart,false)
  const active=view({...ready,phase:'running',busy:true,owned:true},{pending})
  assert.equal(active.canStop,false);assert.equal(active.canView,false)
  assert.equal(view({...ready,connection:{active:false}},{pending}).canConnect,false)
 }
})
test('stop requires typed ownership and cannot repeat while already stopping',()=>{
 for(const owned of [false,undefined,'true'])assert.equal(view({...ready,phase:'running',busy:true,owned}).canStop,false)
 assert.equal(view({...ready,phase:'starting',busy:true,owned:true}).canStop,true)
 assert.equal(view({...ready,phase:'stopping',busy:true,owned:true}).canStop,false)
 assert.equal(view({...ready,phase:'running',busy:true,owned:true},{nowUnix:NOW+6}).canStop,false)
 // A bad newly selected version does not prevent retiring a current owned session.
 assert.equal(view({...ready,phase:'running',busy:true,owned:true,release:{readiness:'invalid'}}).canStop,true)
})
test('connect is explicit and cannot reconnect active busy replay or unreadable sessions',()=>{
 const disconnected={...ready,connection:{active:false,phase:'stopped',health:{sdk_fresh:false,lidar_fresh:false},replay:false}}
 assert.equal(view(disconnected).canConnect,true)
 for(const fields of [{busy:true},{phase:'running'},{connection:{active:true}},
  {connection:{active:false,replay:true}}, {connection:{active:false,health:{replay:true}}},
  {connection:{active:'false'}}, {connection:undefined}])
  assert.equal(view({...disconnected,...fields}).canConnect,false)
 assert.equal(view(disconnected,{readError:'timeout'}).canConnect,false)
 assert.equal(view(disconnected,{nowUnix:NOW+6}).canConnect,false)
 for(const phase of ['checking','connecting','disconnecting','starting','stopping'])
  assert.equal(view({...disconnected,connection:{...disconnected.connection,phase}}).canConnect,false)
 assert.equal(view({...ready,connection:{active:true,health:{sdk_fresh:'true',lidar_fresh:true}}}).connectionLabel,'连接中 · 等待数据')
 assert.equal(view(ready).connectionLabel,'机器狗已连接')
})
test('version line distinguishes running version from selection and never uses directory age',()=>{
 assert.equal(view(ready).versionLabel,'版本 r3')
 assert.equal(view({...ready,release:{...ready.release,running_id:'r2'}}).versionLabel,'运行 r2 · 已选 r3')
 assert.equal(view({...ready,release:{...ready.release,running_id:'r3'}}).versionLabel,'版本 r3')
 assert.equal(view({...ready,release:{...ready.release,selected_id:'',configured_id:''}}).versionLabel,'未选择版本')
 assert.equal(scopeLabel({floors:['floor1','floor2'],stairs_enabled:'true'}),'floor1 · floor2 · 无楼梯')
})
test('RViz is display-only and requires a fresh owned running core',()=>{
 const active={...ready,phase:'running',busy:true,owned:true}
 assert.equal(view(active).canView,true)
 for(const fields of [{phase:'starting'},{phase:'stopping'},{busy:false},{owned:false},{owned:'true'},{quarantined:true}])
  assert.equal(view({...active,...fields}).canView,false)
 assert.equal(view(active,{readError:'timeout'}).canView,false)
 assert.equal(view(active,{nowUnix:NOW+6}).canView,false)
 assert.equal(view({...active,view_error:'RViz 未打开'}).viewError,'RViz 未打开')
 assert.equal(view({...active,view_error:'旧错误'},{nowUnix:NOW+6}).viewError,'')
})
test('JSX keeps shadcn and lifecycle-only explicit clicks with expiring local clock',()=>{
 const panel=readFileSync(new URL('../src/NavigationLaunch.jsx',import.meta.url),'utf8')
 assert.doesNotThrow(()=>parse(panel,{sourceType:'module',plugins:['jsx']}))
 assert.match(panel,/onClick=\{\(\)=>act\('connect'\)\}/)
 assert.match(panel,/onClick=\{\(\)=>act\('view\/global'\)\}/)
 assert.equal((panel.match(/onClick=\{\(\)=>act\('view\//g)||[]).length,1)
 assert.match(panel,/>\{pending==='view\/global'&&<Spinner\/>\}打开 RViz<\/Button>/)
 assert.match(panel,/title="在同一窗口切换全局 \/ 局部视图"/)
 assert.doesNotMatch(panel,/act\('view\/local'\)|'view\/local':|>全局视图<|>局部视图</)
 assert.doesNotMatch(panel,/useEffect\([^\n]*act\('connect'\)/)
 assert.match(panel,/nowUnix:Date\.now\(\)\/1000/)
 assert.match(panel,/setInterval\(\(\)=>setNowUnix\(Date\.now\(\)\/1000\),500\)/)
 assert.match(panel,/clearInterval\(clock\)/)
 assert.match(panel,/writing\.current\|\|/)
 assert.match(panel,/components\/ui\/button/)
 assert.doesNotMatch(panel,/PCT|SCAN|单楼层|单层|live-planning-stages/)
})
