// Coverage comes from the activated release, never from a fixed profile name.
export function scopeLabel(scope) {
 const floors=Array.isArray(scope?.floors)?scope.floors.filter(f=>typeof f==='string'&&f):[]
 if(!floors.length)return ''
 const stairs=scope?.stairs_enabled===true
 return floors.join(' · ')+(floors.length>1||stairs?(stairs?' · 含楼梯':' · 无楼梯'):'')
}

const text=value=>typeof value==='string'?value.trim():''
const MAINLINE='d1max_pct_scan.navigation_session'
// Existing activated releases may still report the legacy module name.
const LEGACY_MAINLINE='d1max_pct_scan.single_floor_session'

export function navigationLaunchStatus(data,{pending='',readError='',nowUnix=Date.now()/1000}={}) {
 const readable=!!data&&typeof data==='object'&&!readError
 const timestamp=data?.snapshot_at_unix
 const age=typeof timestamp==='number'&&Number.isFinite(timestamp)?nowUnix-timestamp:Infinity
 // UI clock rounding tolerance only; the original five-second expiry is not extended.
 const fresh=readable&&Number.isFinite(nowUnix)&&timestamp>0&&age>=-.1&&age<=5
 const phase=fresh?data.phase:'unknown'
 const quarantined=data?.quarantined===true||['conflict','failed','needs_review'].includes(phase)
 const release=data?.release||{}
 const mainlineVerified=[MAINLINE,LEGACY_MAINLINE].includes(data?.mainline?.entry_module)&&data?.mainline?.task_owner==='BehaviorTree.CPP'&&
  !!text(data?.mainline?.entrypoint)
 const releaseReady=fresh&&mainlineVerified&&data.release_configured===true&&release.readiness==='ready'
 const idle=fresh&&!pending&&data.busy===false&&phase==='stopped'&&!quarantined
 // can_start is server readiness, never execution permission. Legacy flags
 // alone cannot activate a release or let this page acquire motion authority.
 const canStart=idle&&releaseReady&&data.can_start===true
 const canStop=fresh&&!pending&&data.owned===true&&data.busy===true&&['running','starting'].includes(phase)
 const canView=fresh&&!pending&&data.owned===true&&data.busy===true&&phase==='running'&&!quarantined
 const connection=data?.connection||{}
 const replay=connection.replay===true||connection.health?.replay===true
 const connecting=['checking','connecting','disconnecting','starting','stopping'].includes(connection.phase)
 const canConnect=idle&&connection.active===false&&!replay&&!connecting
 const purpose=fresh?text(data.purpose):''
 const motionCapable=fresh&&data.motion_capable===true
 const executionAvailable=releaseReady&&motionCapable&&data.execution_available===true&&purpose==='execution'
 const configured=text(release.configured_id)||text(release.selected_id)
 const running=text(release.running_id)
 const versionLabel=running?(configured&&running!==configured?`运行 ${running} · 已选 ${configured}`:`版本 ${running}`):
  configured?`版本 ${configured}`:'未选择版本'
 let blocker=''
 if(readError)blocker=readError
 else if(!readable)blocker='正在读取状态'
 else if(!fresh)blocker='状态已过期，请刷新'
 else if(!mainlineVerified)blocker='启动入口未核对'
 else if(quarantined)blocker=text(data.start_blocker)||text(release.reason)||'当前服务需要核对'
 else if(release.readiness!=='ready'||data.release_configured!==true)
  blocker=text(data.start_blocker)||text(release.reason)||({missing_activation:'版本未激活',invalid:'版本校验失败',checking:'正在核对版本'})[release.readiness]||'版本尚未就绪'
 else if(data.busy===false&&!canStart&&!pending)blocker=text(data.start_blocker)||'当前不能启动'
 return {
  label:pending==='connect'?'连接中':pending==='start'?'启动中':pending==='stop'?'停止中':!fresh?'状态未知':
   quarantined?'待核对':release.readiness==='checking'?'核对中':!releaseReady&&!data.busy?'未就绪':
   ({running:'运行中',stopping:'停止中',starting:'启动中',stopped:canStart?'可启动':'未启动'})[phase]||'待核对',
  fresh,releaseReady,canStart,canStop,canConnect,canView,blocker,viewError:fresh?text(data.view_error):'',
  unavailable:fresh&&!releaseReady&&data.busy===false,
  scope:fresh?scopeLabel(release.scope):'',
  purpose,motionCapable,executionAvailable,
  capabilityLabel:executionAvailable?'执行需 RViz 确认':purpose==='planning_only'?'仅规划':'运动未就绪',
  mainlineLabel:mainlineVerified?'BehaviorTree.CPP':'入口待核对',versionLabel,
  connectionLabel:!fresh?'连接状态未知':replay?'回放数据':connection.active===true?
   (connection.health?.sdk_fresh===true&&connection.health?.lidar_fresh===true?'机器狗已连接':'连接中 · 等待数据'):
   connection.active===false?'机器狗未连接':'连接状态未知',
 }
}
