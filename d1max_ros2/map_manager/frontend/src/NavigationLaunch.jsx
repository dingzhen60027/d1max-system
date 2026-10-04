import {useCallback,useEffect,useRef,useState} from 'react'
import {FileWarning,Play,Plug,RefreshCw,Square} from 'lucide-react'
import {Button} from '@/components/ui/button'
import {Badge} from '@/components/ui/badge'
import {Card,CardContent,CardFooter} from '@/components/ui/card'
import {Alert,AlertTitle,AlertDescription} from '@/components/ui/alert'
import {Spinner} from '@/components/ui/spinner'
import {navigationLaunchStatus} from './navigation-launch-status.mjs'

// Service lifecycle only. Goals and execution confirmation remain in RViz/BT.
export default function NavigationLaunch() {
 const [data,setData]=useState(null),[pending,setPending]=useState(''),[error,setError]=useState(''),[readError,setReadError]=useState('')
 const [nowUnix,setNowUnix]=useState(()=>Date.now()/1000)
 const alive=useRef(false),reading=useRef(false),writing=useRef(false)
 const request=async(action,write=false)=>{
  const response=await fetch('/api/navigation-session/'+action,write?
   {method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(100000)}:
   {signal:AbortSignal.timeout(6500)})
  const value=await response.json().catch(()=>({}))
  if(!response.ok)throw Error(typeof value.detail==='string'?value.detail:'请求失败')
  return value
 }
 const refresh=useCallback(async()=>{
  if(reading.current)return
  reading.current=true
  try {const value=await request('overview');if(alive.current){setData(value);setReadError('')}}
  catch(e){if(alive.current)setReadError(e.name==='TimeoutError'?'状态读取超时':e.message)}
  finally{reading.current=false}
 },[])
 useEffect(()=>{alive.current=true;void refresh();const timer=setInterval(refresh,1500)
  const clock=setInterval(()=>setNowUnix(Date.now()/1000),500)
  return()=>{alive.current=false;clearInterval(timer);clearInterval(clock)}},[refresh])
 const view=navigationLaunchStatus(data,{pending,readError,nowUnix})
 const act=async action=>{
  const current=navigationLaunchStatus(data,{pending,readError,nowUnix:Date.now()/1000})
  if(writing.current||!({start:current.canStart,stop:current.canStop,connect:current.canConnect,
   'view/global':current.canView})[action])return
  writing.current=true;setPending(action);setError('')
  try{const value=await request(action,true);if(alive.current)setData(value)}
  catch(e){if(alive.current)setError(e.name==='TimeoutError'?'请求超时，结果待核对':e.message)}
  finally{await refresh();writing.current=false;if(alive.current)setPending('')}
 }
 return <main className="live-planning-page">
  <header className="live-planning-heading"><h1>导航</h1><div>{view.scope&&<Badge variant="secondary">{view.scope}</Badge>}<Badge variant="secondary">{view.capabilityLabel}</Badge><Badge variant="outline">{view.label}</Badge>
   <Button variant="ghost" size="icon" aria-label="刷新导航服务状态" disabled={!!pending} onClick={refresh}><RefreshCw/></Button></div></header>
  {(error||view.viewError)&&<Alert variant="destructive"><FileWarning/><AlertTitle>{error?'操作未确认':'视图未打开'}</AlertTitle><AlertDescription>{error||view.viewError}</AlertDescription></Alert>}
  <Card className="live-planning-launch"><CardContent>
   <div className="live-mainline-row"><span>{view.mainlineLabel}</span><span>{view.versionLabel}</span></div>
   <div className="live-connection-row"><div className="live-connection-copy"><strong>{view.connectionLabel}</strong></div>
    <Button variant="outline" disabled={!view.canConnect} onClick={()=>act('connect')}>{pending==='connect'?<Spinner/>:<Plug/>}连接机器狗</Button>
   </div>
   {view.blocker&&<p className="live-launch-blocker" role="status">{view.blocker}</p>}
   <div className="live-launch-actions"><div>
    <Button size="lg" disabled={!view.canStart} onClick={()=>act('start')}>{pending==='start'?<Spinner/>:<Play/>}启动导航服务</Button>
    <Button size="lg" variant="outline" disabled={!view.canStop} onClick={()=>act('stop')} title="退役当前任务并停止本次服务；不是急停">{pending==='stop'?<Spinner/>:<Square/>}停止服务</Button>
   </div><span className="live-motion-mode">目标与执行确认 · RViz</span></div>
   <div className="live-view-actions"><Button variant="outline" disabled={!view.canView} onClick={()=>act('view/global')} title="在同一窗口切换全局 / 局部视图">{pending==='view/global'&&<Spinner/>}打开 RViz</Button></div>
  </CardContent><CardFooter><span>启动不执行运动</span></CardFooter></Card>
 </main>
}
