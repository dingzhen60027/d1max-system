import { useCallback, useEffect, useRef, useState } from 'react'
import { ArrowUpRight, Check, Crosshair, Download, FileWarning, Layers3, Link2, MapPin, Play, Radio, Square } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Alert, AlertTitle, AlertDescription } from '@/components/ui/alert'
import { Progress } from '@/components/ui/progress'
import { Spinner } from '@/components/ui/spinner'
import PoseEstimateMap from './PoseEstimateMap'
import { mcStatus } from './mc-status.mjs'
import { localizationIssue } from './localization-status.mjs'
import './localization.css'

const labels={stopped:'未启动',failed:'启动 / 运行失败',running:'进程运行中',starting:'启动中',stopping:'停止中',conflict:'进程冲突',waiting_sensors:'等待传感器',calibrating:'静止初始化',recovering_local:'重新初始化 · 请静止',waiting_initial_pose:'等待初始位姿',acquiring:'匹配确认中',filter_initializing:'导航滤波就绪中',navigation_fault:'导航输出已暂停',degraded:'定位降级',tracking:'定位已锁定',lost:'定位已失锁',relocalizing:'受限重定位中',fault:'局部里程计已停止'}
async function api(path,body){
 const response=await fetch('/api/localization/'+path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
 const value=await response.json()
 if(!response.ok)throw Error(typeof value.detail==='string'?value.detail:JSON.stringify(value.detail||'定位请求失败'))
 return value
}
export default function LocalizationPanel({version,navigate,otherBusy}){
 const [data,setData]=useState(null),[pending,setPending]=useState(''),[error,setError]=useState(''),[notice,setNotice]=useState(''),[requestId,setRequestId]=useState('')
 const [pose,setPose]=useState({x:'0',y:'0',z:'0',yaw:'0'}),[picked,setPicked]=useState(false),[previewError,setPreviewError]=useState(false)
 const lock=useRef(false),poll=useRef(false),alive=useRef(true),queuedAt=useRef(0)
 const refresh=useCallback(async()=>{if(poll.current)return;poll.current=true;try{const value=await api('overview');if(alive.current){setData(value);setError(e=>e.startsWith('状态读取失败')?'':e)}}catch(e){if(alive.current){setData(null);setError('状态读取失败：'+e.message)}}finally{poll.current=false}},[])
 useEffect(()=>{alive.current=true;refresh();const timer=setInterval(refresh,1500);return()=>{alive.current=false;clearInterval(timer)}},[refresh])
 useEffect(()=>{setPicked(false);setPreviewError(false);setRequestId('');setNotice('')},[version?.id])
 const health=data?.health||{},connection=data?.connection||{},sensor=health.sensors||{}
 const running=['running','starting','stopping','detached'].includes(data?.phase)
 const ready=connection.active&&connection.health?.sdk_fresh&&connection.health?.lidar_fresh&&connection.health?.replay===false
 const lioBackend=health.local_backend==='faster_lio'||data?.backend==='lio_pcd'
 const busy=!!pending,seedReady=data?.phase==='running'&&(lioBackend?health.initial_pose_ready===true:sensor.sdk&&sensor.imu&&sensor.cloud&&health.gyro_bias!==null&&health.gyro_bias!==undefined&&health.local_ekf_fresh)
 const acknowledged=requestId&&health.command_result?.id===requestId?health.command_result:null
 useEffect(()=>{if(requestId&&!acknowledged&&Date.now()-queuedAt.current>8000){setRequestId('');setNotice('');setError('初值回执超时，请核对节点状态与日志；没有自动重发。')}},[data,requestId,acknowledged])
 const mapMatches=!running||data?.version_id===version?.id
 const run=async(action,body,message)=>{if(lock.current)return;lock.current=true;setPending(action);setError('');setNotice('');try{const result=await api(action,body);if(alive.current){setNotice(message);if(action==='initial-pose'){setRequestId(result.request_id);queuedAt.current=Date.now()}}await refresh()}catch(e){if(alive.current)setError(e.message)}finally{lock.current=false;if(alive.current)setPending('')}}
 const gestureDisabled=busy||!mapMatches||previewError||!!requestId&&!acknowledged
 function submitPose(next){
  return run('initial-pose',{x:Number(next.x),y:Number(next.y),z:Number(next.z),yaw:Number(next.yaw)*Math.PI/180,reference:'body'},'初值已提交，等待确认。')
 }
 function commitArrow(next,complete){
  const value={x:Number(next.x).toFixed(3),y:Number(next.y).toFixed(3),z:String(next.z),yaw:Number(next.yaw).toFixed(3)}
  setPose(value);setPicked(complete)
  if(!complete){setNotice('请拖出箭头指定机头方向。');return}
  if(seedReady&&!gestureDisabled)void submitPose(value)
  else setNotice('初值已记录，定位就绪后可提交。')
 }
 const age=health.correction_age
 const dedicatedSpeed=health.sdk_source==='mc_stream'
 const calibrationPending=health.calibration&&(!health.calibration.extrinsics_verified||!health.calibration.time_alignment_verified)
 const observedPoseHz=health.output_observed_hz
 const speedReport=connection.health?.speed_report?.source==='sdk_mc'?connection.health.speed_report:health.speed_report||{}
 const speed=mcStatus(speedReport,dedicatedSpeed)
 const speedLabel=speed.label
 const issue=localizationIssue(health)
 return <div className="localization-page">
  {error&&<Alert variant="destructive" role="alert"><FileWarning/><AlertTitle>操作尚未完成</AlertTitle><AlertDescription>{error}</AlertDescription></Alert>}
  {(notice||acknowledged)&&<Alert role="status"><Radio/><AlertDescription>{acknowledged?(acknowledged.accepted?'初值已接收，等待匹配确认。':'初值未采用：'+acknowledged.message):notice}</AlertDescription></Alert>}
  {issue&&<Alert variant={issue.destructive?'destructive':'default'} role="status"><FileWarning/><AlertTitle>{issue.title}</AlertTitle><AlertDescription>{issue.detail}</AlertDescription></Alert>}
  <div className="localization-top">
   <Card><CardHeader><CardTitle><span className="step-number">1</span>连接实机数据</CardTitle></CardHeader><CardContent><div className="localization-chips"><Badge variant={connection.health?.sdk_fresh?'default':'outline'}>SDK {connection.health?.sdk_fresh?'在线':'待连接'}</Badge><Badge variant={connection.health?.lidar_fresh?'default':'outline'}>雷达 {connection.health?.lidar_fresh?'在线':'待连接'}</Badge>{connection.health?.replay&&<Badge variant="destructive">正在回放</Badge>}</div><Button variant="outline" disabled={busy||!data||connection.active||['starting','stopping'].includes(connection.phase)} onClick={()=>run('connect',{},'连接请求已提交，等待数据。')}>{pending==='connect'?<Spinner/>:<Link2/>}{connection.active?'数据后台已启动':'连接机器狗'}</Button>{connection.error&&<small>{connection.error}</small>}</CardContent></Card>
   <Card><CardHeader><CardTitle><span className="step-number">2</span>{lioBackend?'启动 LIO + PCD 定位':'启动双 EKF 定位'}</CardTitle></CardHeader><CardContent><div className="localization-map-name"><Layers3/><strong>{version?.name||'请先选用单层地图'}</strong></div><div className="localization-buttons"><Button disabled={busy||!data?.installed||!ready||running||data?.phase==='conflict'||!version||otherBusy} onClick={()=>run('start',{version_id:version.id},'定位启动中，请保持静止完成 IMU 标定。')}>{pending==='start'?<Spinner/>:<Play/>}启动定位</Button><Button variant="outline" disabled={busy||!running} onClick={()=>run('stop',{},'定位已停止，数据连接保留。')}>{pending==='stop'?<Spinner/>:<Square/>}停止</Button></div></CardContent></Card>
   <Card className="localization-live-card"><CardHeader><CardTitle><span className={'localization-dot '+(health.localized?'is-locked':'')}/>{labels[health.state]||labels[data?.phase]||'读取状态…'}</CardTitle></CardHeader><CardContent><div className="localization-metrics"><div><strong>{health.fusion?.accepted??'—'}</strong><span>接受匹配</span></div><div><strong>{Number.isFinite(age)?age.toFixed(2)+' s':'—'}</strong><span>校正数据龄</span></div><div><strong title={speed.detail}>{speedLabel}</strong><span>SDK 实收频率</span></div></div><div className="localization-chips">{calibrationPending&&<Badge variant="outline">调试 · 未标定</Badge>}{Number.isFinite(observedPoseHz)&&<Badge variant="outline">位姿 {observedPoseHz.toFixed(1)} Hz</Badge>}{[[lioBackend?'局部 LIO':'局部 EKF',health.local_ekf_fresh],[lioBackend?'地图校正':'全局 EKF',health.global_ekf_fresh],['IMU',sensor.imu]].map(([name,fresh])=><Badge key={name} variant={fresh?'secondary':'outline'}>{fresh&&<Check/>}{name}</Badge>)}</div>{health.state==='calibrating'&&<><small>请保持静止</small><Progress value={Math.min(100,100*(health.gyro_bias_samples||0)/(health.gyro_bias_required||100))}/></>}</CardContent></Card>
  </div>
  <div className="localization-workspace">
   <Card className="localization-map"><CardHeader><div><CardTitle><Crosshair/>拖箭头设置初始位姿</CardTitle><CardDescription>拖动定朝向 · 松开提交 · Esc 取消</CardDescription></div><Button variant="outline" onClick={()=>navigate('/2d/versions')}>地图版本<ArrowUpRight/></Button></CardHeader><CardContent>{version?<div className="localization-map-stage"><PoseEstimateMap key={version.id} version={version} pose={pose} picked={picked} disabled={gestureDisabled} onCommit={commitArrow} onError={()=>setPreviewError(true)}/>{previewError&&<p>地图预览加载失败，请重新扫描。</p>}</div>:<div className="localization-no-map"><MapPin/><p>未选用地图</p><Button onClick={()=>navigate('/2d/versions')}>选择地图</Button></div>}</CardContent></Card>
   <Card className="localization-seed"><CardHeader><CardTitle><span className="step-number">3</span>初始箭头</CardTitle></CardHeader><CardContent><form onSubmit={e=>{e.preventDefault();submitPose(pose)}}><div className="pose-estimate-summary"><span>机身位置</span><strong>{Number(pose.x).toFixed(2)}, {Number(pose.y).toFixed(2)} m</strong><span>机头朝向</span><strong>{Number(pose.yaw).toFixed(1)}°</strong></div><details className="pose-estimate-advanced"><summary>精确坐标 / 高级设置</summary><div className="localization-pose-fields">{[['x','X · m',-100000,100000],['y','Y · m',-100000,100000],['z','Z · m',-100,100],['yaw','机头朝向 · °',-180,180]].map(([key,label,min,max])=><label key={key}><span>{label}</span><Input aria-label={label} type="number" step="any" min={min} max={max} required value={pose[key]} onChange={e=>{setPose(v=>({...v,[key]:e.target.value}));setPicked(true)}}/></label>)}</div><p>+X = 0°，+Y = 90°。Z 为地图坐标中的机身高度。</p></details><Button type="submit" disabled={busy||!seedReady||!mapMatches||!picked||!!requestId&&!acknowledged}>{pending==='initial-pose'?<Spinner/>:<Crosshair/>}提交定位初值</Button></form><div className="localization-view-handoff"><strong>Foxglove</strong><code>ws://127.0.0.1:8769</code><small>橙：初值预览 · 黄：已验证位姿</small></div><Button nativeButton={false} variant="outline" render={<a href="/api/localization/config" download/>}><Download/>下载定位参数 YAML</Button></CardContent></Card>
  </div>
  {!!health.warnings?.length&&<details className="localization-logs"><summary>定位限制 · {health.warnings.length}</summary><ul>{health.warnings.map((warning,index)=><li key={index}>{warning}</li>)}</ul></details>}
  {data?.error&&<Alert variant="destructive"><AlertDescription>{data.error}</AlertDescription></Alert>}
  {health.state==='waiting_sensors'&&health.last_error&&<Alert><FileWarning/><AlertTitle>最近的数据校验记录</AlertTitle><AlertDescription>{health.last_error}</AlertDescription></Alert>}
  {health.input_clock?.ready==='false'&&<Alert variant="destructive"><FileWarning/><AlertTitle>传感器时间轴未就绪</AlertTitle><AlertDescription>{health.input_clock.message}</AlertDescription></Alert>}
  {health.matcher&&!health.localized&&<Alert><FileWarning/><AlertTitle>匹配器状态 · {health.matcher.message}</AlertTitle><AlertDescription>尝试 {health.matcher.attempts||0} 次 · 未收敛 {health.matcher.non_converged||0} 次 · 耗时 {health.matcher.elapsed_ms||'—'} ms</AlertDescription></Alert>}
  {!!data?.logs?.length&&<details className="localization-logs"><summary>定位日志 · 最近 50 行</summary><pre>{data.logs.join('\n')}</pre></details>}
 </div>
}
