import {useCallback,useEffect,useRef,useState} from 'react'
import {Check,ChevronDown,Crosshair,FileWarning,Link2,Map,Play,RefreshCw,Route,ScanLine,ShieldCheck,Square} from 'lucide-react'
import {Button} from '@/components/ui/button'
import {Badge} from '@/components/ui/badge'
import {Card,CardContent,CardFooter} from '@/components/ui/card'
import {Alert,AlertTitle,AlertDescription} from '@/components/ui/alert'
import {Spinner} from '@/components/ui/spinner'
import {componentName,livePlanningStatus,livePlanningFailure} from './live-planning-status.mjs'
import './live-planning.css'

async function request(path, {write=false,signal}={}) {
 const response=await fetch(path,write?{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal}:{signal})
 const value=await response.json().catch(()=>({}))
 if(!response.ok)throw Error(typeof value.detail==='string'?value.detail:'请求失败')
 return value
}
const stageDefinitions=[{key:'localization',name:'定位',icon:Crosshair},{key:'global_planner',name:'全局规划',icon:Route},{key:'local_planner',name:'局部规划',icon:ScanLine}]

export default function LivePlanningPanel() {
 const [data,setData]=useState(null),[pending,setPending]=useState(''),[error,setError]=useState(''),[readError,setReadError]=useState('')
 const [observedAt,setObservedAt]=useState(()=>Date.now()/1000)
 const alive=useRef(true),reading=useRef(false),writing=useRef(false)
 const refresh=useCallback(async()=>{
  if(reading.current)return
  reading.current=true
  try {const value=await request('/api/live-planning/overview',{signal:AbortSignal.timeout(6500)});if(alive.current){setData(value);setReadError('');setObservedAt(Date.now()/1000)}}
  catch(e){if(alive.current)setReadError(e.name==='TimeoutError'?'状态读取超时':e.message)}
  finally{reading.current=false}
 },[])
 useEffect(()=>{alive.current=true;void refresh();const timer=setInterval(()=>{setObservedAt(Date.now()/1000);void refresh()},1500);return()=>{alive.current=false;clearInterval(timer)}},[refresh])
 const view=livePlanningStatus(data,{pending,readable:!readError,now:observedAt})
 const act=async action=>{
  if(writing.current)return
  if(action==='start'&&!view.canStart||action==='stop'&&!view.canStop||action==='connect'&&!view.canConnect)return
  writing.current=true;setPending(action);setError('')
  try {
   const path=action==='connect'?'/api/localization/connect':'/api/live-planning/'+action
   const value=await request(path,{write:true,signal:AbortSignal.timeout(50000)})
   if(alive.current&&action!=='connect'){setData(value);setReadError('');setObservedAt(Date.now()/1000)}
  } catch(e) {if(alive.current)setError(e.name==='TimeoutError'?'请求超时，结果待确认':e.message)}
  finally {await refresh();writing.current=false;if(alive.current)setPending('')}
 }
 const failure=livePlanningFailure(data,{error,readError})
 return <main className="live-planning-page">
  <header className="live-planning-heading"><h1>定位与规划</h1><div><Badge variant="outline" className="live-phase">{pending==='start'?'启动中':pending==='stop'?'停止中':view.phaseLabel}</Badge><Button variant="ghost" size="icon" aria-label="刷新运行状态" title="刷新状态" disabled={!!pending} onClick={refresh}><RefreshCw/></Button></div></header>
  {failure.message&&<Alert variant="destructive" role="alert"><FileWarning/><AlertTitle>{failure.title}</AlertTitle><AlertDescription>{failure.message}</AlertDescription></Alert>}
  <Card className="live-planning-launch">
   <CardContent>
    <div className="live-connection-row"><div className="live-connection-copy"><span className={'live-status-dot '+view.connectionTone}/><div><strong>{view.connectionLabel}</strong><div className="live-sensor-tags"><span className={view.readable&&data?.connection?.health?.lidar_fresh?'is-fresh':''}>雷达</span><span className={view.readable&&data?.connection?.health?.sdk_fresh?'is-fresh':''}>运动数据</span></div></div></div><Button variant="outline" disabled={!view.canConnect} onClick={()=>act('connect')}>{pending==='connect'?<Spinner/>:<Link2/>}连接</Button></div>
    <div className="live-launch-actions"><div><Button size="lg" title="启动服务并打开 RViz" disabled={!view.canStart} onClick={()=>act('start')}>{pending==='start'?<Spinner/>:<Play/>}启动</Button><Button variant="outline" size="lg" title="停止并清理本次进程；不是机器人急停" disabled={!view.canStop} onClick={()=>act('stop')}>{pending==='stop'?<Spinner/>:<Square/>}停止</Button></div><span className="live-motion-mode"><ShieldCheck/>{view.noMotion?'仅预览 · 运动关闭':'模式待确认'}</span></div>
   </CardContent>
   <CardFooter><span><Map/>{data?.map_name||'未配置地图'}</span>{data?.current_floor&&<span className="live-floor">{({floor1:'一楼',floor2:'二楼'})[data.current_floor]||data.current_floor}</span>}</CardFooter>
  </Card>
  <section className="live-planning-stages" aria-label="运行状态" aria-live="polite">{stageDefinitions.map((step,index)=>{
   const stage=view.stages[index]
   return <div key={step.key} className={'live-stage '+stage.tone}><span className="live-stage-icon"><step.icon aria-hidden="true"/></span><div><h2>{step.name}</h2><span className="live-stage-state">{stage.tone==='ready'&&<Check/>}{stage.label}</span></div></div>
  })}</section>
  <details className="live-configuration"><summary>运行配置<ChevronDown aria-hidden="true"/></summary><dl>{stageDefinitions.map(step=><div key={step.key}><dt>{step.name}</dt><dd>{componentName(data?.components,step.key)}</dd></div>)}<div><dt>操作端</dt><dd>RViz</dd></div></dl></details>
 </main>
}
