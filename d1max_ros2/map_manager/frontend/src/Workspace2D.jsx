import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react'
import { Archive, ArrowRight, Check, CircleHelp, FileDown, FileWarning, Grid2X2, Layers3, Map, MapPin, Pencil, Plus, RefreshCw, RotateCcw, Route, Search, Settings2, ShieldCheck, Trash2, X, ZoomIn, ZoomOut, Maximize, GitCompareArrows } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Textarea } from '@/components/ui/textarea'
import { Switch } from '@/components/ui/switch'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Alert, AlertTitle, AlertDescription } from '@/components/ui/alert'
import { Empty, EmptyHeader, EmptyMedia, EmptyTitle, EmptyDescription } from '@/components/ui/empty'
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter } from '@/components/ui/dialog'
import { AlertDialog, AlertDialogContent, AlertDialogHeader, AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction } from '@/components/ui/alert-dialog'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Checkbox } from '@/components/ui/checkbox'
import { Progress } from '@/components/ui/progress'
import { Spinner } from '@/components/ui/spinner'
const MapEditor = lazy(()=>import('./MapEditor.jsx'))
const LocalizationPanel = lazy(()=>import('./LocalizationPanel.jsx'))
const defaults={filter_enabled:false,statistical_mean_k:20,statistical_std_dev_mul:.5,radius:.3,radius_min_points:4,z_min:.4,z_max:1.5,resolution:.05,padding:.5,min_points_per_cell:1,background:'unknown'}
async function api(path,options={}){
 const r=await fetch('/api/2d'+path,{headers:{'Content-Type':'application/json'},...options})
 const data=await r.json().catch(()=>({}))
 if(!r.ok)throw Error(Array.isArray(data.detail)?data.detail.map(v=>v.msg).join('；'):data.detail||'地图服务请求失败')
 return data
}
const post=(body)=>({method:'POST',body:JSON.stringify(body)})
const number=(n)=>Number(n||0).toLocaleString()
function Setting({label,name,value,set,min,max,step='any',hint}){
 return <label className="grid-setting"><span>{label}</span><Input aria-label={label} name={name} type="number" min={min} max={max} step={step} required value={value} onChange={e=>set(Number(e.target.value))}/>{hint&&<small>{hint}</small>}</label>
}
function GridBuildForm({sources,profiles,sourceQuery,busy,onBuild,onSaveProfile}){
 const [sourceId,setSourceId]=useState(sourceQuery||sources[0]?.id||'')
 const [name,setName]=useState('单层导航地图'),[note,setNote]=useState('')
 const [profile,setProfile]=useState('conservative'),[parameters,setParameters]=useState(()=>({...profiles.find(v=>v.id==='conservative')?.parameters||defaults}))
 const [profileName,setProfileName]=useState(''),[profileOpen,setProfileOpen]=useState(false)
 useEffect(()=>{if(sourceQuery)setSourceId(sourceQuery)},[sourceQuery])
 const change=(key,value)=>setParameters(v=>({...v,[key]:value}))
 const selected=sources.find(v=>v.id===sourceId)
 return <form className="grid-build-form" onSubmit={e=>{e.preventDefault();if(!busy&&selected)onBuild({source_id:sourceId,name:name.trim(),note,parameters})}}>
  <div className="grid-build-fields">
   <Card><CardHeader><CardTitle><span className="step-number">1</span>选择源点云</CardTitle></CardHeader><CardContent>
    <label className="grid-setting"><span>源 PCD</span><Select value={sourceId} onValueChange={setSourceId}><SelectTrigger aria-label="源 PCD"><SelectValue>{selected?.name||'选择一份 PCD'}</SelectValue></SelectTrigger><SelectContent>{sources.map(v=><SelectItem key={v.id} value={v.id}>{v.name} · {v.category==='processed'?'已处理':'原始'}</SelectItem>)}</SelectContent></Select></label>
    {selected?<div className="source-receipt"><Layers3/><span><strong>{number(selected.points)} 点</strong><small>{selected.size_human} · {selected.category==='processed'?'处理后点云':'建图输出'}</small></span><Badge variant="outline">只读</Badge></div>:<Alert variant="destructive"><FileWarning/><AlertDescription>{sourceQuery?'指定点云不存在或已归档，请重新选择。':'没有可用 PCD，请先在 3D 工作区准备点云。'}</AlertDescription></Alert>}
   </CardContent></Card>
   <Card><CardHeader><CardTitle><span className="step-number">2</span>配置生成参数</CardTitle></CardHeader><CardContent className="grid-config-content">
    <div className="grid-profile-line"><Select value={profile} onValueChange={id=>{setProfile(id);setParameters({...profiles.find(v=>v.id===id).parameters})}}><SelectTrigger aria-label="2D 参数方案"><SelectValue>{profiles.find(v=>v.id===profile)?.name||'自定义参数'}</SelectValue></SelectTrigger><SelectContent>{profiles.map(v=><SelectItem value={v.id} key={v.id}>{v.name}</SelectItem>)}</SelectContent></Select><Button type="button" variant="outline" disabled={busy} onClick={()=>setProfileOpen(true)}>另存方案</Button></div>
    <div className="grid-fieldset"><h3>导航高度切片</h3><p>Z 使用 PCD 坐标，非离地高度。</p><div className="grid-fields">
     <Setting label="Z 下限（m）" value={parameters.z_min} min={-100} max={100} set={v=>change('z_min',v)}/>
     <Setting label="Z 上限（m）" value={parameters.z_max} min={-100} max={100} set={v=>change('z_max',v)}/>
     <Setting label="栅格分辨率（m/格）" value={parameters.resolution} min={.02} max={.5} step={.01} set={v=>change('resolution',v)}/>
     <Setting label="边界留白（m）" value={parameters.padding} min={0} max={5} step={.1} set={v=>change('padding',v)}/>
     <Setting label="每格最少点数" value={parameters.min_points_per_cell} min={1} max={100} step={1} set={v=>change('min_points_per_cell',v)}/>
     <label className="grid-setting"><span>没有观测点的栅格</span><Select value={parameters.background} onValueChange={v=>change('background',v)}><SelectTrigger aria-label="没有观测点的栅格"><SelectValue>{parameters.background==='unknown'?'未知 · 保守':'自由 · Go2 兼容'}</SelectValue></SelectTrigger><SelectContent><SelectItem value="unknown">未知 · 保守（推荐）</SelectItem><SelectItem value="free">自由 · Go2 兼容，需核对</SelectItem></SelectContent></Select></label>
    </div></div>
    <div className="grid-fieldset"><div className="grid-filter-toggle"><span><h3>定位点云预处理</h3></span><Switch checked={parameters.filter_enabled} onCheckedChange={v=>change('filter_enabled',v)} aria-label="启用定位点云预处理"/></div>
     {parameters.filter_enabled&&<div className="grid-fields">
      <Setting label="统计邻居数" value={parameters.statistical_mean_k} min={2} max={500} step={1} set={v=>change('statistical_mean_k',v)}/>
      <Setting label="标准差倍率" value={parameters.statistical_std_dev_mul} min={.05} max={10} set={v=>change('statistical_std_dev_mul',v)}/>
      <Setting label="邻域半径（m）" value={parameters.radius} min={.01} max={5} set={v=>change('radius',v)}/>
      <Setting label="半径内最少点数" value={parameters.radius_min_points} min={1} max={500} step={1} set={v=>change('radius_min_points',v)}/>
     </div>}
    </div>
   </CardContent></Card>
  </div>
  <div className="grid-build-summary"><Card><CardHeader><CardTitle><span className="step-number">3</span>保存候选版本</CardTitle></CardHeader><CardContent>
   <label className="grid-setting"><span>版本名称</span><Input required maxLength={80} value={name} onChange={e=>setName(e.target.value)}/></label>
   <label className="grid-setting"><span>备注</span><Textarea rows={3} maxLength={500} value={note} onChange={e=>setNote(e.target.value)} placeholder="场地、楼层和本次参数调整"/></label>
   <details className="output-receipt"><summary>输出文件</summary>{['map.pgm · 栅格地图','map.yaml · 坐标与分辨率','localization.pcd · 定位点云','pipeline.yaml · 生成参数','manifest.json · 版本记录'].map(v=><span key={v}><Check/>{v}</span>)}</details>
   {parameters.background==='free'&&<Alert className="grid-caution"><CircleHelp/><AlertDescription>空白将标为自由空间，通行性需现场核对。</AlertDescription></Alert>}
   <Button className="build-submit" type="submit" disabled={busy||!selected||!name.trim()||parameters.z_min>=parameters.z_max}>{busy?<Spinner/>:<Grid2X2/>}{busy?'已有任务运行中':'生成并保存候选版'}</Button>
  </CardContent></Card></div>
  <Dialog open={profileOpen} onOpenChange={setProfileOpen}><DialogContent><DialogHeader><DialogTitle>保存 2D 参数方案</DialogTitle><DialogDescription className="sr-only">另存为 YAML 方案</DialogDescription></DialogHeader><Input aria-label="方案名称" value={profileName} onChange={e=>setProfileName(e.target.value)} placeholder="方案名称"/><DialogFooter><Button type="button" variant="outline" onClick={()=>setProfileOpen(false)}>取消</Button><Button type="button" disabled={busy||!profileName.trim()} onClick={async()=>{if(await onSaveProfile({name:profileName,parameters}))setProfileOpen(false)}}>保存方案</Button></DialogFooter></DialogContent></Dialog>
 </form>
}
function GridPreview({version,parent}){
 const host=useRef(null),[zoom,setZoom]=useState(1),[error,setError]=useState('')
 const fit=useCallback(()=>{if(host.current)setZoom(Math.max(.03,Math.min((host.current.clientWidth-64)/(version.width*(parent?2:1)),(host.current.clientHeight-64)/version.height,3)))},[version.width,version.height,parent?.id])
 useEffect(()=>{setError('');const observer=new ResizeObserver(fit);if(host.current)observer.observe(host.current);fit();return()=>observer.disconnect()},[fit,version.id])
 return <><div className={'grid-canvas '+(parent?'is-comparing':'')} ref={host}>{(parent?[parent,version]:[version]).map(v=><div className="grid-image-panel" key={v.id}>{parent&&<span>{v.id===version.id?'修订后':'源版本'}</span>}<img alt={v.name+' 栅格预览'} src={v.map_preview_url} style={{width:v.width*zoom,height:v.height*zoom}} onError={()=>setError('栅格图像加载失败，请重新扫描并核对版本文件')}/></div>)}{error&&<div className="grid-image-error">{error}</div>}</div>
 <div className="grid-preview-foot"><div className="grid-legend"><span><i className="occupied"/>占据</span><span><i className="free"/>自由</span><span><i className="unknown"/>未知</span></div><div className="grid-zoom"><Button variant="ghost" size="icon-sm" onClick={()=>setZoom(v=>Math.max(.03,v/1.25))} aria-label="缩小地图"><ZoomOut/></Button><span>{Math.round(zoom*100)}%</span><Button variant="ghost" size="icon-sm" onClick={()=>setZoom(v=>Math.min(8,v*1.25))} aria-label="放大地图"><ZoomIn/></Button><Button variant="outline" size="sm" onClick={fit}><Maximize/>适应</Button></div></div></>
}
export default function Workspace2D({section,query,navigate}){
 const [data,setData]=useState(null),[sources,setSources]=useState([]),[cloudBusy,setCloudBusy]=useState(false)
 const [selectedId,setSelectedId]=useState(query.get('version')||''),[search,setSearch]=useState('')
 const [busy,setBusy]=useState(false),[error,setError]=useState(''),[notice,setNotice]=useState('')
 const [editor,setEditor]=useState(null),[confirm,setConfirm]=useState(null),[rename,setRename]=useState(null),[newName,setNewName]=useState('')
 const [checked,setChecked]=useState(new Set()),[compare,setCompare]=useState(false)
 const lock=useRef(false),fetchLock=useRef(false),lastResult=useRef(undefined),alive=useRef(true)
 const refresh=useCallback(async()=>{
  if(fetchLock.current)return
  fetchLock.current=true
  try{
   const [grid,raw]=await Promise.all([api('/overview'),fetch('/api/overview').then(async r=>{if(!r.ok)throw Error('3D 来源列表不可用');return r.json()})])
   if(!alive.current)return
   setData(grid);setSources(raw.items.filter(v=>['maps','processed'].includes(v.category)&&!v.archived&&v.extension==='pcd'))
   setCloudBusy(raw.processing_job?.running===true)
   if(lastResult.current!==undefined&&grid.job.result_id&&grid.job.result_id!==lastResult.current)setSelectedId(grid.job.result_id)
   lastResult.current=grid.job.result_id||null
  }catch(e){if(alive.current)setError(e.message)}finally{fetchLock.current=false}
 },[])
 useEffect(()=>{alive.current=true;refresh();const timer=setInterval(refresh,2500);return()=>{alive.current=false;clearInterval(timer)}},[refresh])
 const versions=(data?.versions||[]).filter(v=>v.archived===(section==='archived')),list=versions.filter(v=>v.name.toLowerCase().includes(search.toLowerCase()))
 useEffect(()=>{setSearch('');setChecked(new Set());setCompare(false)},[section])
 useEffect(()=>{setSelectedId(id=>versions.some(v=>v.id===id)?id:versions[0]?.id||'')},[data,section])
 useEffect(()=>setCompare(false),[selectedId])
 const selected=versions.find(v=>v.id===selectedId),active=data?.versions.find(v=>v.selected)
 const parent=selected?.parent_version?data?.versions.find(v=>v.id===selected.parent_version):null
 const run=async(path,options,success)=>{
  if(lock.current)return false
  lock.current=true;setBusy(true);setError('');setNotice('')
  try{await api(path,options);setNotice(success);await refresh();return true}catch(e){setError(e.message);return false}finally{lock.current=false;if(alive.current)setBusy(false)}
 }
 const ask=(title,description,path,options,success,danger=false)=>setConfirm({title,description,path,options,success,danger})
 const archive=(v)=>run('/versions/'+v.id,{method:'PATCH',body:JSON.stringify({archived:!v.archived})},v.archived?'地图已恢复':'地图已归档')
 const deletion=(ids)=>ask('永久删除 '+ids.length+' 个 2D 版本？','会真实删除这些已归档版本的 PGM、YAML、定位 PCD 和参数记录；源 PCD 和其他版本不会删除，操作无法恢复。','/archive/delete',post({version_ids:ids}),'所选归档已清理',true)
 const job=data?.job||{},blocked=busy||job.running||cloudBusy
 const headings={versions:'2D 地图版本',build:'从 PCD 生成 2D 地图',navigation:'单楼层定位调试',archived:'2D 归档'}
 return <div className="workspace2d">
  <header className="workspace-heading"><div><h1>{headings[section]}</h1></div><div className="heading-actions"><Button variant="outline" onClick={refresh}><RefreshCw/>重新扫描</Button>{section!=='build'&&<Button onClick={()=>navigate('/2d/build')}><Plus/>生成 2D 地图</Button>}</div></header>
  {error&&<Alert variant="destructive" className="workspace-notice" role="alert"><FileWarning/><AlertTitle>操作未完成</AlertTitle><AlertDescription>{error}</AlertDescription><Button variant="ghost" size="icon-xs" onClick={()=>setError('')} aria-label="关闭错误提示"><X/></Button></Alert>}
  {notice&&<div className="workspace-notice success-notice" role="status"><Check/>{notice}<Button variant="ghost" size="icon-xs" onClick={()=>setNotice('')} aria-label="关闭提示"><X/></Button></div>}
  {data?.invalid?.length>0&&<Alert variant="destructive" className="workspace-notice"><FileWarning/><AlertTitle>{data.invalid.length} 个版本文件异常</AlertTitle><AlertDescription>这些版本未加入可用列表，文件没有被删除：{data.invalid.map(v=>v.id+' · '+v.error).join('；')}</AlertDescription></Alert>}
  {job.running&&<div className="grid-job"><Spinner/><div><strong>2D 地图生成中 · {job.progress||0}%</strong><span>{job.logs?.at(-1)}</span><Progress value={job.progress||0}/></div><Button variant="outline" disabled={busy||job.status==='cancelling'} onClick={()=>run('/cancel',post({}),'已请求取消；当前步骤完成后停止')}>{job.status==='cancelling'?'取消中':'取消生成'}</Button></div>}
  {!job.running&&['failed','interrupted'].includes(job.status)&&<Alert variant="destructive" className="workspace-notice"><FileWarning/><AlertTitle>上次生成未完成</AlertTitle><AlertDescription>{job.error}</AlertDescription></Alert>}
  {!data?<div className="page-loading"><Spinner/>正在读取 2D 工作区…</div>:section==='build'?<div className="grid-build-scroll"><GridBuildForm sources={sources} profiles={data.profiles} sourceQuery={query.get('source')} busy={blocked} onBuild={async body=>{if(await run('/build',post(body),'2D 生成已启动，完成后会出现新候选版本'))navigate('/2d/versions')}} onSaveProfile={body=>run('/profiles',post(body),'参数已保存为 YAML 方案')}/></div>
  :section==='navigation'?<Suspense fallback={<div className="page-loading"><Spinner/>加载定位工作台…</div>}><LocalizationPanel version={active} navigate={navigate} otherBusy={blocked}/></Suspense>:<div className="grid-workspace">
   <Card className="grid-version-list"><div className="resource-list-head"><strong>{section==='archived'?'已归档版本':'地图版本'}</strong><Badge variant="secondary">{versions.length}</Badge></div><div className="search"><Search/><Input aria-label="搜索2D地图" value={search} onChange={e=>setSearch(e.target.value)} placeholder="搜索版本名称"/></div>
    {section==='archived'&&<div className="grid-archive-tools"><label><Checkbox aria-label="全选2D归档" checked={list.length>0&&list.every(v=>checked.has(v.id))} onCheckedChange={v=>setChecked(v?new Set(list.map(x=>x.id)):new Set())}/>全选</label><Button variant="outline" size="sm" disabled={!checked.size||busy} onClick={()=>deletion([...checked].filter(id=>versions.some(v=>v.id===id)))}>删除所选</Button><Button variant="destructive" size="sm" disabled={!versions.length||busy} onClick={()=>deletion(versions.map(v=>v.id))}>全部删除</Button></div>}
    <ScrollArea className="grid-version-scroll">{list.map(v=><div className={'grid-version '+(v.id===selectedId?'selected':'')} key={v.id}>{section==='archived'&&<Checkbox aria-label={'选择 '+v.name} checked={checked.has(v.id)} onCheckedChange={value=>setChecked(current=>{const next=new Set(current);value?next.add(v.id):next.delete(v.id);return next})}/>}<button onClick={()=>setSelectedId(v.id)}><img src={v.map_preview_url} alt=""/><span><strong>{v.name}</strong><small>{v.width} × {v.height} · {v.parameters.resolution} m/格</small><span className="version-status">{v.selected?<Badge>当前选用</Badge>:<Badge variant="outline">{v.origin==='2d-edited'?'修订版':'候选版'}</Badge>}<time>{v.created_at?.slice(5,16).replace('T',' ')}</time></span></span></button></div>)}
    {!list.length&&<Empty><EmptyHeader><EmptyMedia variant="icon"><Map/></EmptyMedia><EmptyTitle>{section==='archived'?'暂无归档':'暂无 2D 地图'}</EmptyTitle></EmptyHeader>{section!=='archived'&&<Button onClick={()=>navigate('/2d/build')}><Plus/>生成地图</Button>}</Empty>}
    </ScrollArea></Card>
   <Card className="grid-preview"><div className="grid-preview-head"><div><strong>{selected?.name||'2D 栅格预览'}</strong></div>{selected&&parent&&<Button variant="outline" size="sm" onClick={()=>setCompare(v=>!v)}><GitCompareArrows/>{compare?'单图':'修订对比'}</Button>}</div>
    {selected?<GridPreview version={selected} parent={compare?parent:null}/>:<Empty className="grid-preview-empty"><EmptyHeader><EmptyMedia variant="icon"><Grid2X2/></EmptyMedia><EmptyTitle>请选择地图</EmptyTitle></EmptyHeader></Empty>}
   </Card>
   <Card className="grid-inspector"><ScrollArea className="grid-inspector-scroll">{selected?<><div className="grid-inspector-title"><Map/><Badge variant={selected.selected?'default':'secondary'}>{selected.selected?'当前选用':'候选版本'}</Badge><h2>{selected.name}</h2><p>{selected.source_name}</p></div>
    <div className="grid-details"><h3>地图规格</h3><dl><div><dt>分辨率</dt><dd>{selected.metadata.resolution} m/格</dd></div><div><dt>尺寸</dt><dd>{selected.width} × {selected.height}</dd></div><div><dt>原点 XY</dt><dd>{selected.metadata.origin.slice(0,2).map(v=>v.toFixed(2)).join(', ')} m</dd></div><div><dt>占据 / 未知</dt><dd>{number(selected.occupied_cells)} / {number(selected.unknown_cells)}</dd></div><div><dt>生成方式</dt><dd>{selected.origin==='2d-edited'?'手工修订':'PCD 投影'}</dd></div></dl>{selected.note&&<p>{selected.note}</p>}</div>
    {selected.warning&&<Alert className="grid-caution"><FileWarning/><details><summary>通行性待核对</summary><AlertDescription>{selected.warning}</AlertDescription></details></Alert>}
    <div className="grid-inspector-actions"><Button disabled={blocked||selected.archived||!selected.complete} onClick={()=>setEditor(selected)}><Pencil/>修整 2D 地图</Button>
     {!selected.selected&&!selected.archived&&<Button variant="outline" disabled={blocked||!selected.complete} onClick={()=>ask('选用这个 2D 地图版本？','仅切换地图，不启动定位或导航。','/selection',post({version_id:selected.id}),'2D 地图已选用')}><MapPin/>设为选用地图</Button>}
     {selected.selected&&<Button variant="outline" disabled={busy} onClick={()=>run('/selection',{method:'DELETE'},'已取消地图选用')}>取消选用</Button>}
     {data.previous_id&&<Button variant="outline" disabled={blocked} onClick={()=>ask('回退到上次选用版本？','不会覆盖任何版本文件，也不会触发机器人动作。','/rollback',post({}),'已回退选用版本')}><RotateCcw/>回退上一版本</Button>}
     <Button nativeButton={false} variant="outline" render={<a href={selected.download_url} download/>}><FileDown/>下载完整版本</Button>
     <Button variant="outline" onClick={()=>{setRename(selected.id);setNewName(selected.name)}}><Pencil/>修改名称</Button>
     <Button variant="ghost" disabled={busy||selected.selected} onClick={()=>archive(selected)}>{selected.archived?<RotateCcw/>:<Archive/>}{selected.archived?'恢复版本':'无损归档'}</Button>
     {selected.archived&&<Button variant="destructive" disabled={busy} onClick={()=>deletion([selected.id])}><Trash2/>永久删除此版本</Button>}
    </div><div className="grid-file-links"><h3>版本文件</h3>{['map.yaml','map.pgm','localization.pcd','pipeline.yaml','manifest.json'].map(file=><a key={file} href={'/api/2d/versions/'+selected.id+'/files/'+file} download><FileDown/>{file}</a>)}</div>
   </>:<Empty><EmptyHeader><EmptyMedia variant="icon"><Settings2/></EmptyMedia><EmptyDescription>未选择地图</EmptyDescription></EmptyHeader></Empty>}</ScrollArea></Card>
  </div>}
  <AlertDialog open={!!confirm} onOpenChange={open=>!open&&!busy&&setConfirm(null)}><AlertDialogContent><AlertDialogHeader><AlertDialogTitle>{confirm?.title}</AlertDialogTitle><AlertDialogDescription>{confirm?.description}</AlertDialogDescription></AlertDialogHeader><AlertDialogFooter><AlertDialogCancel disabled={busy}>取消</AlertDialogCancel><AlertDialogAction variant={confirm?.danger?'destructive':'default'} disabled={busy} onClick={async()=>{if(await run(confirm.path,confirm.options,confirm.success)){setConfirm(null);setChecked(new Set())}}}>{busy?<Spinner/>:null}确认{confirm?.danger?'永久删除':''}</AlertDialogAction></AlertDialogFooter></AlertDialogContent></AlertDialog>
  <Dialog open={!!rename} onOpenChange={open=>!open&&setRename(null)}><DialogContent><DialogHeader><DialogTitle>修改版本名称</DialogTitle><DialogDescription className="sr-only">输入新的版本名称</DialogDescription></DialogHeader><Input aria-label="版本名称" value={newName} maxLength={80} onChange={e=>setNewName(e.target.value)}/><DialogFooter><Button variant="outline" onClick={()=>setRename(null)}>取消</Button><Button disabled={busy||!newName.trim()} onClick={async()=>{if(await run('/versions/'+rename,{method:'PATCH',body:JSON.stringify({name:newName})},'名称已更新'))setRename(null)}}>保存名称</Button></DialogFooter></DialogContent></Dialog>
  {editor&&<Suspense fallback={<div className="page-loading"><Spinner/>正在加载编辑器…</div>}><MapEditor version={editor} onClose={()=>setEditor(null)} onDone={async id=>{setEditor(null);await refresh();setSelectedId(id)}} notify={(message,type)=>type==='error'?setError(message):setNotice(message)}/></Suspense>}
 </div>
}
