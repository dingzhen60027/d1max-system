import assert from 'node:assert/strict'
import test from 'node:test'
import {componentName,livePlanningStatus,livePlanningFailure} from '../src/live-planning-status.mjs'
import {readFileSync} from 'node:fs'

function fixture(){return {snapshot_at_unix:Date.now()/1000,phase:'stopped',busy:false,installed:true,mode:'LIVE_VISUALIZATION_NO_MOTION',motion_enabled:false,session_id:null,connection:{active:true,health:{sdk_fresh:true,lidar_fresh:true,replay:false}}}}
test('only real fresh connected stopped no-motion session may start',()=>{
 assert(livePlanningStatus(fixture()).canStart)
 for(const alter of [d=>d.connection.active=false,d=>d.connection.health.replay=true,d=>d.connection.health.sdk_fresh=false,d=>d.busy=true,d=>d.phase='conflict',d=>d.mode='live_motion',d=>d.motion_enabled=true,d=>d.installed=false]){
  const data=fixture();alter(data);assert.equal(livePlanningStatus(data).canStart,false)
 }
 assert.equal(livePlanningStatus(fixture(),{pending:'start'}).canStart,false)
 assert.equal(livePlanningStatus(fixture(),{readable:false}).canStart,false)
})
test('stop only targets known owned non-conflict session',()=>{
 const data={...fixture(),phase:'running',busy:true,session_id:'owned'}
 assert(livePlanningStatus(data).canStop)
 for(const edit of [{session_id:null},{phase:'conflict'},{phase:'stopping'}])assert.equal(livePlanningStatus({...data,...edit}).canStop,false)
})
test('a fully stopped failed session permits manual restart, not overlap',()=>{
 const data={...fixture(),phase:'failed'}
 assert(livePlanningStatus(data).canStart)
 assert.equal(livePlanningStatus({...data,busy:true}).canStart,false)
 assert.equal(livePlanningStatus(data,{pending:'start'}).canStart,false)
})
test('generic stages need current backend admission, not raw algorithm packets',()=>{
 const stages=Object.fromEntries(['localization','global_planner','local_planner'].map(key=>[key,{label:'已就绪',tone:'ready',expires_at_unix:102}]))
 const data={...fixture(),snapshot_at_unix:100,phase:'running',busy:true,session_id:'owned',stages}
 assert.deepEqual(livePlanningStatus(data,{now:100.2}).stages.map(s=>s.tone),['ready','ready','ready'])
 for(const edit of [{expires_at_unix:99},{expires_at_unix:null},{tone:'unknown'},{label:{}}]){
  assert.notEqual(livePlanningStatus({...data,stages:{...stages,local_planner:{...stages.local_planner,...edit}}},{now:100.2}).stages[2].tone,'ready')
 }
 assert(livePlanningStatus({...data,stages:{},scan_status:{ready:true,last_spline_id:9,counters:{marker_published:20}}},{now:100.2}).stages.every(stage=>stage.tone!=='ready'))
 assert.equal(livePlanningStatus({...data,phase:'stopped',busy:false},{now:100.2}).stages[0].label,'未启动')
})
test('expired or unreadable snapshot disables starting and clears ready indicators',()=>{
 for(const stamp of [96,101,NaN,Infinity,null,undefined,'100']){
  const data={...fixture(),snapshot_at_unix:stamp}
  assert.equal(livePlanningStatus(data,{now:100}).canStart,false)
  assert.equal(livePlanningStatus(data,{now:100}).connected,false)
 }
 const data={...fixture(),snapshot_at_unix:96,phase:'running',busy:true,session_id:'owned'}
 assert(livePlanningStatus(data,{now:100}).canStop,'an owned session remains stoppable while status is stale')
})
test('component display is configuration-driven without algorithm defaults',()=>{
 const components={global_planner:{id:'alternative_global',name:'其他全局算法'},local_planner:{id:'alternative_local',name:'其他局部算法'}}
 assert.equal(componentName(components,'global_planner'),'其他全局算法')
 assert.equal(componentName(components,'local_planner'),'其他局部算法')
 assert.equal(componentName(components,'localization'),'未配置')
 assert.equal(componentName({global_planner:{name:{}}},'global_planner'),'未配置')
 assert.equal(componentName({global_planner:{name:'x'.repeat(97)}},'global_planner'),'未配置')
})
test('known runtime failure is not hidden by polling timeout or generic launch error',()=>{
 const data={...fixture(),phase:'failed',error:'定位程序安装不完整：缺少模块 d1max_localization.estimation.pose_status'}
 assert.deepEqual(livePlanningFailure(data,{readError:'状态读取超时',error:'启动失败，请核对日志'}),{message:data.error,title:'运行异常'})
 assert.deepEqual(livePlanningFailure(null,{readError:'状态读取超时'}),{message:'状态读取超时',title:'状态不可用'})
 assert.deepEqual(livePlanningFailure(fixture(),{error:'启动请求失败',readError:'状态读取超时'}),{message:'启动请求失败',title:'运行异常'})
 const panel=readFileSync(new URL('../src/NavigationLaunch.jsx',import.meta.url),'utf8')
 assert.match(panel,/setData\(value\);setReadError\(''\)/)
})
test('shell and planning page do not contain algorithm branding or numeric workflow decoration',()=>{
 for(const file of ['App.jsx','LivePlanningPanel.jsx','NavigationLaunch.jsx','live-planning-status.mjs']){
  const source=readFileSync(new URL('../src/'+file,import.meta.url),'utf8')
  assert.doesNotMatch(source,/PCT|SCAN|pct_global|scan_local|planning_native_pct/)
 }
 const panel=readFileSync(new URL('../src/NavigationLaunch.jsx',import.meta.url),'utf8')
 assert.doesNotMatch(panel,/live-stage-index|启动后自动打开 RViz|RViz 工作流/)
 assert.match(panel,/启动不执行运动/)
 assert.match(panel,/退役当前任务并停止本次服务；不是急停/)
})
