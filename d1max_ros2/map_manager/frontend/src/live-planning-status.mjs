// The runtime adapter supplies algorithm-neutral display stages.
// Lifecycle and sensor safety remain authoritative on the backend.
const phaseNames={stopped:'未启动',starting:'启动中',running:'运行中',stopping:'停止中',failed:'运行异常',conflict:'状态异常'}
const stageKeys=['localization','global_planner','local_planner']
const tones=new Set(['ready','waiting','warning','idle'])

export function livePlanningFailure(data,{error='',readError=''}={}){
 const runtime=data?.error||error||data?.connection?.error
 return {message:runtime||readError,title:runtime?'运行异常':'状态不可用'}
}

export function componentName(components,key){
 const name=components?.[key]?.name
 return typeof name==='string'&&name.trim()&&name.length<=96?name:'未配置'
}

export function livePlanningStatus(data,{pending='',readable=true,now=Date.now()/1000}={}){
 const snapshot=data?.snapshot_at_unix
 const fresh=readable&&Number.isFinite(snapshot)&&now-snapshot>=-.1&&now-snapshot<=3.5
 const connection=data?.connection||{},health=connection.health||{}
 const connected=fresh&&connection.active===true&&health.sdk_fresh===true&&health.lidar_fresh===true&&health.replay===false
 const running=data?.phase==='running',busy=data?.busy===true||['starting','running','stopping'].includes(data?.phase)
 const owned=typeof data?.session_id==='string'&&!!data.session_id
 const noMotion=data?.mode==='LIVE_VISUALIZATION_NO_MOTION'&&data?.motion_enabled===false
 const safeToStart=!!data&&fresh&&noMotion&&data.installed===true&&connected&&!busy&&['stopped','failed'].includes(data.phase)
 const stages=stageKeys.map(key=>{
  if(!fresh)return {label:data?'状态待更新':'读取中',tone:data?'warning':'waiting'}
  if(!running)return {label:'未启动',tone:'idle'}
  const stage=data?.stages?.[key]
  if(!stage||typeof stage.label!=='string'||!stage.label.trim()||stage.label.length>64||!tones.has(stage.tone))return {label:'状态待更新',tone:'waiting'}
  if(stage.tone==='ready'&&(!Number.isFinite(stage.expires_at_unix)||stage.expires_at_unix<now))return {label:'状态待更新',tone:'waiting'}
  return {label:stage.label,tone:stage.tone}
 })
 return {connected,running,busy,readable:fresh,noMotion,stages,
  phaseLabel:!data?'读取中':!fresh?'状态待更新':phaseNames[data.phase]||'状态异常',
  canStart:!pending&&safeToStart,
  canStop:!pending&&busy&&owned&&data?.phase!=='conflict'&&data?.phase!=='stopping',
  canConnect:!pending&&fresh&&!!data&&!connection.active&&!['starting','stopping','connecting','disconnecting'].includes(connection.phase),
  connectionLabel:!fresh?(data?'连接状态待更新':'读取连接状态'):connected?'已连接':health.replay?'回放中':connection.active?'等待数据':'未连接',
  connectionTone:connected?'ready':fresh&&connection.active?'waiting':'offline'}
}
