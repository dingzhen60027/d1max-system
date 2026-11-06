import {useCallback,useEffect,useRef,useState} from 'react'
import {FileWarning,Play,Plug,Square} from 'lucide-react'
import {Button} from '@/components/ui/button'
import {Badge} from '@/components/ui/badge'
import {Card,CardContent} from '@/components/ui/card'
import {Alert,AlertDescription} from '@/components/ui/alert'
import {Spinner} from '@/components/ui/spinner'
import {navigationLaunchStatus} from './navigation-launch-status.mjs'
import {createNavigationOverviewCoordinator} from './navigation-overview-coordinator.mjs'

// Service lifecycle only. Goals and execution confirmation remain in RViz/BT.
export default function NavigationLaunch() {
 const [data,setData]=useState(null),[pending,setPending]=useState(''),[error,setError]=useState(''),[readError,setReadError]=useState('')
 const [nowUnix,setNowUnix]=useState(()=>Date.now()/1000)
 const alive=useRef(false),writing=useRef(false),overview=useRef(null)
 const request=async(action,write=false)=>{
  const response=await fetch('/api/navigation-session/'+action,write?
   {method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(['start','stop'].includes(action)?6500:100000)}:
   {signal:AbortSignal.timeout(6500)})
  const value=await response.json().catch(()=>({}))
  if(!response.ok){
   const failure=Error(typeof value.detail==='string'?value.detail:'请求失败')
   failure.status=response.status;failure.response=value
   throw failure
  }
  return value
 }
 if(!overview.current)overview.current=createNavigationOverviewCoordinator({
  read:()=>request('overview'),
  onValue:value=>{if(alive.current){setData(value);setReadError('');setNowUnix(Date.now()/1000);setError('')}},
  onError:e=>{console.error('导航状态读取失败',e);if(alive.current)setReadError('状态读取失败')},
 })
 const refresh=useCallback(()=>overview.current.refresh(),[])
 useEffect(()=>{alive.current=true;void refresh();const timer=setInterval(refresh,1500)
  const clock=setInterval(()=>setNowUnix(Date.now()/1000),500)
  return()=>{alive.current=false;overview.current.invalidate();clearInterval(timer);clearInterval(clock)}},[refresh])
 const view=navigationLaunchStatus(data,{pending,readError,nowUnix})
 useEffect(()=>{
  if(data?.worker_error||data?.startup_error)console.error('导航生命周期失败',{
   operation:data.worker_operation,error:data.worker_error||data.startup_error})
  if(data?.view_error)console.error('导航视图未打开',data.view_error)
 },[data?.worker_error,data?.startup_error,data?.worker_operation,data?.view_error])
 const message=view.workerError||error||(view.viewError?'RViz未打开':'')
 const act=async action=>{
  const current=navigationLaunchStatus(data,{pending,readError,nowUnix:Date.now()/1000})
  if(writing.current||!({start:current.canStart,stop:current.canStop,connect:current.canConnect,
   'view/global':current.canView})[action])return
  writing.current=true;overview.current.beginWrite();setPending(action);setError('')
  try{const value=await request(action,true);if(alive.current){setData(value);setReadError('');setNowUnix(Date.now()/1000)}}
  catch(e){
   console.error('导航操作失败',{action,error:e,status:e.status,response:e.response})
   if(alive.current)setError(({start:'启动请求失败',stop:'退出请求失败',connect:'连接失败','view/global':'RViz未打开'})[action]||'操作失败')
  }
  finally{writing.current=false;if(alive.current)setPending('');void overview.current.finishWrite({refresh:alive.current})}
 }
 return <main className="live-planning-page navigation-launch-page">
  <header className="live-planning-heading"><h1>导航</h1><Badge variant="outline" role="status">{view.label}</Badge></header>
  {message&&<Alert variant="destructive"><FileWarning/><AlertDescription>{message}</AlertDescription></Alert>}
  <Card className="live-planning-launch"><CardContent>
   <div className="live-connection-row"><div className="live-connection-copy"><strong>{view.connectionLabel}</strong></div>
    <Button variant="outline" disabled={!view.canConnect} onClick={()=>act('connect')}>{pending==='connect'?<Spinner/>:<Plug/>}连接机器狗</Button>
   </div>
   <div className="navigation-launch-actions">
    <Button size="lg" disabled={!view.canStart} onClick={()=>act('start')}>{pending==='start'?<Spinner/>:<Play/>}启动</Button>
    <Button size="lg" variant="outline" disabled={!view.canStop} onClick={()=>act('stop')}>{pending==='stop'?<Spinner/>:<Square/>}退出</Button>
    <Button size="lg" variant="outline" disabled={!view.canView} onClick={()=>act('view/global')}>{pending==='view/global'&&<Spinner/>}打开 RViz</Button>
   </div>
  </CardContent></Card>
 </main>
}
