const text=value=>typeof value==='string'?value.trim():''

export function navigationLaunchStatus(data,{pending='',readError='',nowUnix=Date.now()/1000}={}) {
 const readable=!!data&&typeof data==='object'&&!readError
 const timestamp=data?.snapshot_at_unix
 const age=typeof timestamp==='number'&&Number.isFinite(timestamp)?nowUnix-timestamp:Infinity
 // UI clock rounding tolerance only; the original five-second expiry is not extended.
 const fresh=readable&&Number.isFinite(nowUnix)&&timestamp>0&&age>=-.1&&age<=5
 const phase=fresh?data.phase:'unknown'
 const workerError=fresh&&(text(data.worker_error)||text(data.startup_error))
 // The backend owns admission, cancellation, startup ACK, and recovery.
 const available=fresh&&!pending
 const canStart=available&&data.can_start===true
 const canStop=available&&data.can_stop===true
 const canView=available&&data.can_view===true
 const canConnect=available&&data.can_connect===true
 const connection=data?.connection||{}
 const replay=connection.replay===true||connection.health?.replay===true
 const connecting=['checking','connecting','disconnecting','starting','stopping'].includes(connection.phase)
 const sdkDisconnected=connection.health?.sdk_fresh===false&&connection.health?.speed_report?.state==='disconnected'
 const connectionLabel=!fresh?'连接状态未知':replay?'回放数据':
  connecting||pending==='connect'?'连接中':sdkDisconnected?'机器狗未连接':connection.active===true?
   (connection.health?.sdk_fresh===true&&connection.health?.lidar_fresh===true?'机器狗已连接':'数据未就绪'):
   connection.active===false?'机器狗未连接':'连接状态未知'
 return {
  label:pending==='connect'?'连接中':pending==='start'?'启动中':pending==='stop'?'退出中':!fresh?'状态未知':
   ({preparing:'准备中',starting:'启动中',running:'运行中',stopping:'退出中',
    stopped:canStart?'可启动':'未启动',conflict:'待核对',needs_review:'待核对',failed:'待核对'})[phase]||'状态未知',
  fresh,canStart,canStop,canConnect,canView,connectionLabel,
  workerError:workerError?(data.worker_operation==='stop'?'退出未完成':'启动失败'):'',
  viewError:fresh?text(data.view_error):'',
 }
}
