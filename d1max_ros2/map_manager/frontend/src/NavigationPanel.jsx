import {useCallback,useEffect,useRef,useState} from 'react'
import {Crosshair,FileWarning,LockKeyhole,Play,Route,ShieldCheck,Square,Unlock} from 'lucide-react'
import {Button} from '@/components/ui/button'
import {Badge} from '@/components/ui/badge'
import {Card,CardHeader,CardTitle,CardContent} from '@/components/ui/card'
import {Alert,AlertDescription} from '@/components/ui/alert'
import {Checkbox} from '@/components/ui/checkbox'
import {Select,SelectContent,SelectItem,SelectTrigger,SelectValue} from '@/components/ui/select'
import {AlertDialog,AlertDialogContent,AlertDialogHeader,AlertDialogTitle,AlertDialogDescription,AlertDialogFooter,AlertDialogCancel,AlertDialogAction} from '@/components/ui/alert-dialog'
import {Spinner} from '@/components/ui/spinner'
import {navigationControls,navigationReason} from './navigation-status.mjs'
import './navigation.css'

async function api(path,body){
 const response=await fetch('/api/navigation/'+path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
 const value=await response.json()
 if(!response.ok)throw Error(typeof value.detail==='string'?value.detail:JSON.stringify(value.detail||'导航请求失败'))
 return value
}
const phaseNames={running:'Nav2 运行中',starting:'Nav2 启动中',stopping:'Nav2 停止中',stopped:'Nav2 未启动',unavailable:'状态待核对',conflict:'会话冲突'}

export default function NavigationPanel({version,navigate,otherBusy}){
 const [data,setData]=useState(null),[mode,setMode]=useState('sim'),[motion,setMotion]=useState(false)
 const [showRviz,setShowRviz]=useState(true)
 const [pending,setPending]=useState(''),[error,setError]=useState(''),[notice,setNotice]=useState(''),[confirm,setConfirm]=useState(false)
 const alive=useRef(true),fetching=useRef(false),lock=useRef(false)
 const refresh=useCallback(async()=>{if(fetching.current)return;fetching.current=true;try{const value=await api('overview');if(alive.current)setData(value)}catch(e){if(alive.current){setData(null);setError(e.message)}}finally{fetching.current=false}},[])
 useEffect(()=>{alive.current=true;refresh();const timer=setInterval(refresh,2000);return()=>{alive.current=false;clearInterval(timer)}},[refresh])
 useEffect(()=>setConfirm(false),[version?.id,data?.session_id])
 useEffect(()=>{if(data?.busy&&['sim','live'].includes(data.mode)){setMode(data.mode);setMotion(data.enable_motion===true);setShowRviz(data.show_rviz===true)}},[data?.busy,data?.mode,data?.enable_motion,data?.show_rviz])
 const controls=navigationControls(data,version?.id),busy=!!pending,health=data?.health||{}
 const gate={...controls.gate,reason:controls.gate.armed?controls.gate.reason:(controls.gate.arm_block_reason||controls.gate.reason)}
 const invoke=async(path,body,message)=>{if(lock.current)return;lock.current=true;setPending(path==='command'?body.operation:path);setError('');setNotice('');try{const result=await api(path,path==='start'?{...body,show_rviz:showRviz}:body);if(result.accepted===false)throw Error(result.message||result.reason||'请求未被接受');if(alive.current){setNotice(message);setConfirm(false)}await refresh()}catch(e){if(alive.current)setError(e.message)}finally{lock.current=false;if(alive.current)setPending('')}}
 const command=operation=>invoke('command',{operation,session_id:data.session_id,version_id:data.version_id},
  operation==='arm'?'解锁请求已提交，等待 SDK 确认。':'运动已锁定；重新导航需手动解锁。')
 return <div className="navigation-workbench">
  <nav className="navigation-tabs" aria-label="Nav2 独立方案"><Badge variant="outline">Nav2（独立）</Badge><Button variant="outline" onClick={()=>navigate('/planning')}><Crosshair/>返回定位与规划</Button><Badge variant="outline">速度上限 1.5 m/s</Badge></nav>
  <div className="navigation-content">
   {(error||data?.error||health.error)&&<Alert variant="destructive" role="alert"><FileWarning/><AlertDescription>{error||data?.error||health.error}</AlertDescription></Alert>}
   {notice&&<Alert role="status"><AlertDescription>{notice}</AlertDescription></Alert>}
   <div className="navigation-actions"><label className="navigation-capability"><Checkbox checked={showRviz} disabled={busy||data?.busy} onCheckedChange={v=>setShowRviz(v===true)}/>同时打开 RViz2</label>{Number.isFinite(gate.limits?.max_forward)&&<Badge variant="secondary">当前前向限速 {gate.limits.max_forward.toFixed(2)} m/s</Badge>}</div>
   <div className="navigation-top">
    <Card><CardHeader><CardTitle><Play/>启动单层 Nav2</CardTitle></CardHeader><CardContent className="navigation-card-content">
     <strong>{version?.name||'请先选用单层地图'}</strong>
     <div className="navigation-actions"><Select value={mode} onValueChange={v=>{setMode(v);setMotion(false)}} disabled={busy||data?.busy}><SelectTrigger aria-label="导航运行模式"><SelectValue/></SelectTrigger><SelectContent><SelectItem value="sim">离线仿真 · 无实机运动</SelectItem><SelectItem value="live">实机导航 · 默认锁定</SelectItem></SelectContent></Select>
      <Button disabled={busy||otherBusy||!version||!data||data.busy||!data.installed} onClick={()=>invoke('start',{version_id:version.id,mode,enable_motion:mode==='live'&&motion},'Nav2 已启动，请在 RViz2 中发布目标。')}>{pending==='start'?<Spinner/>:<Play/>}启动 Nav2</Button>
      <Button variant="outline" disabled={busy||!data?.busy||data?.phase==='unavailable'} onClick={()=>invoke('stop',{},'Nav2 已停止并清理其进程组。')}><Square/>停止 Nav2</Button>
     </div>
     {mode==='live'&&<label className="navigation-capability"><Checkbox checked={motion} disabled={busy||data?.busy} onCheckedChange={v=>setMotion(v===true)}/>启用本次 SDK 运动能力（启动后仍需解锁）</label>}
     {mode==='live'&&!data?.busy&&<p className="navigation-reason">请先停止其他定位与规划会话。</p>}
     <p className="navigation-reason">目标点：RViz2「Nav2 Goal」 · 取消任务：RViz2「Navigation 2」面板</p>
    </CardContent></Card>
    <Card><CardHeader><CardTitle><ShieldCheck/>{phaseNames[data?.phase]||'读取状态…'}</CardTitle></CardHeader><CardContent className="navigation-card-content">
     <div className="navigation-actions"><Badge variant={controls.live?'outline':'secondary'}>{data?.busy?(controls.live?'实机':'离线仿真'):'未运行'}</Badge><Badge variant={gate.armed&&controls.live?'destructive':'outline'}>{controls.live?(gate.armed?'SDK 已解锁':'SDK 已锁定'):'SDK 不连接'}</Badge></div>
     <p className="navigation-reason">{navigationReason(gate.reason)}</p>
     <div className="navigation-actions">{controls.live&&data?.enable_motion&&<Button variant="outline" disabled={busy||!controls.canArm} onClick={()=>setConfirm(true)}><Unlock/>解锁 SDK 运动</Button>}{controls.live&&<Button variant="destructive" disabled={busy||!controls.canDisarm} onClick={()=>command('disarm')}><LockKeyhole/>锁定 SDK 运动</Button>}</div>
    </CardContent></Card>
   </div>
  </div>
  {controls.live&&controls.current&&<div className="navigation-safety-bar" role="group" aria-label="实机导航安全操作"><Button variant="destructive" disabled={busy} onClick={()=>command('disarm')}><LockKeyhole/>锁定运动</Button></div>}
  <AlertDialog open={confirm} onOpenChange={value=>!value&&!busy&&setConfirm(false)}><AlertDialogContent><AlertDialogHeader><AlertDialogTitle>解锁本次 SDK 导航运动？</AlertDialogTitle><AlertDialogDescription>确认机器人周围安全、有人监护且急停可用。不会自动起立、解除急停或切换运动状态；检查不通过则保持锁定。</AlertDialogDescription></AlertDialogHeader><AlertDialogFooter><AlertDialogCancel disabled={busy}>取消</AlertDialogCancel><AlertDialogAction disabled={busy||!controls.canArm} onClick={()=>command('arm')}>确认解锁</AlertDialogAction></AlertDialogFooter></AlertDialogContent></AlertDialog>
 </div>
}
