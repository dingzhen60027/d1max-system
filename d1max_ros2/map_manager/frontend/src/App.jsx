import { lazy, Suspense, useEffect, useState } from 'react'
import { ArrowRight, Archive, Box, ChevronRight, Database, FileWarning, Grid2X2, Home, Layers3, Map, Route, ScanLine, WandSparkles } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Card, CardHeader, CardTitle, CardContent, CardFooter } from '@/components/ui/card'
import { Breadcrumb, BreadcrumbList, BreadcrumbItem, BreadcrumbLink, BreadcrumbSeparator, BreadcrumbPage } from '@/components/ui/breadcrumb'
import { SidebarProvider, Sidebar, SidebarHeader, SidebarContent, SidebarFooter, SidebarGroup, SidebarGroupLabel, SidebarMenu, SidebarMenuItem, SidebarMenuButton, SidebarInset, SidebarTrigger, useSidebar } from '@/components/ui/sidebar'
import { Skeleton } from '@/components/ui/skeleton'
import { Alert, AlertTitle, AlertDescription } from '@/components/ui/alert'
const Workspace3D = lazy(()=>import('./Workspace3D.jsx'))
const Workspace2D = lazy(()=>import('./Workspace2D.jsx'))
export const GROUPS = {
 '3d':[{id:'maps',name:'原始点云',icon:Database},{id:'processed',name:'处理结果',icon:WandSparkles},{id:'planning',name:'规划产物',icon:Route},{id:'archived',name:'归档管理',icon:Archive},{id:'failures',name:'异常记录',icon:FileWarning}],
 '2d':[{id:'versions',name:'地图版本',icon:Map},{id:'build',name:'生成 2D 地图',icon:Grid2X2},{id:'navigation',name:'定位调试',icon:Route},{id:'archived',name:'归档管理',icon:Archive}],
}
export function navigate(path){window.location.hash=path}
function WorkspaceLink(props){
 const {setOpenMobile}=useSidebar()
 return <SidebarMenuButton {...props} onClick={()=>setOpenMobile(false)}/>
}
function readRoute(){
 const [path,query='']=(location.hash.slice(1)||'/').split('?')
 const [,mode,section]=path.split('/')
 if(!GROUPS[mode])return {mode:'home',section:'',query:new URLSearchParams()}
 return {mode,section:GROUPS[mode].some(v=>v.id===section)?section:GROUPS[mode][0].id,query:new URLSearchParams(query)}
}
function PreviewArtwork({mode}){
 return <svg viewBox="0 0 520 210" aria-hidden="true" className={'portal-art '+mode}>
  <defs><pattern id={'dots-'+mode} width="16" height="16" patternUnits="userSpaceOnUse"><circle cx="1" cy="1" r="1" fill="currentColor" opacity=".15"/></pattern></defs>
  <rect width="520" height="210" fill={'url(#dots-'+mode+')'}/>
  {mode==='2d'?<g transform="translate(125 25)">
   <path d="M0 0H264V158H0Z M84 0V58 M84 89V158 M182 0V70 M182 105V158 M0 72H52 M114 72H214 M242 72H264" fill="white" stroke="currentColor" strokeWidth="5" strokeLinejoin="round"/>
   <path d="M38 130V73H148V118H227" fill="none" stroke="#059669" strokeWidth="3" strokeDasharray="6 5"/>
   <circle cx="38" cy="130" r="8" fill="#059669"/><circle cx="227" cy="118" r="8" fill="white" stroke="#059669" strokeWidth="3"/>
  </g>:<g transform="translate(255 90)" fill="none" stroke="currentColor" strokeWidth="1.5">
   <path d="M-150 25L-10 -55L150 25L10 110Z" opacity=".22"/>
   {[0,1,2,3].map(i=><path key={i} d={`M-115 ${22-i*15}L-3 ${-42-i*15}L112 ${20-i*15}L1 ${84-i*15}Z`} opacity={.25+i*.18} strokeDasharray={i===3?'':'2 4'}/>)}
   <path d="M-115 -23V22 M-3 -87V-42 M112 -25V20 M1 39V84 M-57 -55V-10 M59 5V50"/>
   <path d="M-84 10L-27 -21L33 11L-22 43Z" stroke="#8b5cf6" strokeDasharray="2 4" strokeWidth="3"/>
  </g>}
 </svg>
}
function HomePage(){
 const [data,setData]=useState(null),[grid,setGrid]=useState(null),[error,setError]=useState('')
 useEffect(()=>{let live=true;const abort=new AbortController();Promise.all(['/api/overview','/api/2d/overview'].map(path=>fetch(path,{signal:abort.signal}).then(async r=>{if(!r.ok)throw Error('地图服务暂不可用');return r.json()}))).then(([d,g])=>{if(live){setData(d);setGrid(g)}}).catch(e=>{if(live)setError(e.message)});return()=>{live=false;abort.abort()}},[])
 return <div className="portal-home">
  <header className="portal-home-top"><a href="#/" className="portal-brand"><span><Layers3/></span><strong>D1 Max<span>MAP WORKSPACE</span></strong></a><Badge variant="outline"><span className="status-dot"/>本地工作台</Badge></header>
  <main className="home-content">
   <div className="home-intro"><h1>地图工作台</h1></div>
   {error&&<Alert variant="destructive"><FileWarning/><AlertTitle>后端连接异常</AlertTitle><AlertDescription>{error}，请在 Foxglove 检查 Web 状态后刷新。</AlertDescription></Alert>}
   <div className="workspace-choices">
    {[{mode:'2d',to:'/2d/versions',title:'2D 导航',icon:Map,steps:['生成栅格','地图修整','版本选用'],count:grid?.versions.filter(v=>!v.archived).length,unit:'个地图版本'},
      {mode:'3d',to:'/3d/maps',title:'3D 地图',icon:Box,steps:['原始建图','点云处理','结果对比'],count:data?.summary?.map_count,unit:'份原始点云'}].map(v=>
     <Card className={'workspace-choice '+v.mode} key={v.mode}><a className="choice-cover-link" href={'#'+v.to} aria-label={'进入 '+v.title}><PreviewArtwork mode={v.mode}/></a>
      <CardHeader><CardTitle><v.icon/>{v.title}</CardTitle></CardHeader>
      <CardContent><div className="choice-steps">{v.steps.map((step,index)=><span key={step}>{index>0&&<ChevronRight/>}{step}</span>)}</div></CardContent>
      <CardFooter><span className="choice-count">{v.count??'—'} {v.unit}</span><Button nativeButton={false} render={<a href={'#'+v.to}/>}>进入工作空间<ArrowRight/></Button></CardFooter>
     </Card>)}
   </div>
  </main>
  <footer className="home-footer"><span>D1 MAX</span></footer>
 </div>
}
export default function App(){
 const [route,setRoute]=useState(readRoute)
 useEffect(()=>{const change=()=>setRoute(readRoute());window.addEventListener('hashchange',change);return()=>window.removeEventListener('hashchange',change)},[])
 if(route.mode==='home')return <HomePage/>
 const modeName=route.mode==='2d'?'2D 导航':'3D 地图',sectionName=GROUPS[route.mode].find(v=>v.id===route.section)?.name
 return <SidebarProvider className="portal-shell" style={{'--sidebar-width':'204px'}}>
  <Sidebar className="portal-sidebar" collapsible="offcanvas">
   <SidebarHeader><a href="#/" className="portal-brand"><span><Layers3/></span><strong>D1 Max<small>地图工作台</small></strong></a></SidebarHeader>
   <SidebarContent><SidebarGroup><SidebarGroupLabel>工作空间</SidebarGroupLabel><SidebarMenu>
    <SidebarMenuItem><WorkspaceLink render={<a href="#/"/>}><Home/><span>工作台首页</span></WorkspaceLink></SidebarMenuItem>
    <SidebarMenuItem><WorkspaceLink isActive={route.mode==='2d'} render={<a href="#/2d/versions"/>}><Map/><span>2D 导航</span></WorkspaceLink></SidebarMenuItem>
    <SidebarMenuItem><WorkspaceLink isActive={route.mode==='3d'} render={<a href="#/3d/maps"/>}><Box/><span>3D 地图</span></WorkspaceLink></SidebarMenuItem>
   </SidebarMenu></SidebarGroup>
   <SidebarGroup><SidebarGroupLabel>{modeName} / 功能</SidebarGroupLabel><SidebarMenu>{GROUPS[route.mode].map(v=><SidebarMenuItem key={v.id}><WorkspaceLink isActive={route.section===v.id} render={<a href={'#/'+route.mode+'/'+v.id}/> }><v.icon/><span>{v.name}</span></WorkspaceLink></SidebarMenuItem>)}</SidebarMenu></SidebarGroup></SidebarContent>
   <SidebarFooter><div className="portal-local" title="ROS 2 · Zenoh · Domain 24"><ScanLine/><span>本地工作区</span></div></SidebarFooter>
  </Sidebar>
  <SidebarInset className="portal-inset"><header className="portal-breadcrumb"><SidebarTrigger/><Breadcrumb><BreadcrumbList><BreadcrumbItem><BreadcrumbLink href="#/">工作台</BreadcrumbLink></BreadcrumbItem><BreadcrumbSeparator/><BreadcrumbItem><BreadcrumbLink href={'#/'+route.mode+'/'+GROUPS[route.mode][0].id}>{modeName}</BreadcrumbLink></BreadcrumbItem><BreadcrumbSeparator/><BreadcrumbItem><BreadcrumbPage>{sectionName}</BreadcrumbPage></BreadcrumbItem></BreadcrumbList></Breadcrumb>{route.mode==='2d'&&<Badge variant="outline">未开放导航行走</Badge>}</header>
   <div className="portal-page"><Suspense fallback={<div className="page-loading"><Skeleton/><Skeleton/><span>正在打开工作空间…</span></div>}>
    {route.mode==='3d'?<Workspace3D section={route.section} setSection={section=>navigate('/3d/'+section)} navigate={navigate}/>:<Workspace2D section={route.section} query={route.query} navigate={navigate}/>}
   </Suspense></div>
  </SidebarInset>
 </SidebarProvider>
}
