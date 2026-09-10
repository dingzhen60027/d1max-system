import { Fragment, lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  Activity, Archive, ArrowDown, ArrowUp, Box, Check, ChevronRight, CircleStop, Clock3, Copy,
  Database, FileWarning, FolderOpen, GitCompareArrows, HardDrive, Layers3,
  MapPin, Pencil, Play, RefreshCw, RotateCcw, Save, Search,
  ServerCog, SlidersHorizontal, Sparkles, Trash2, WandSparkles, X, FileDown,
} from 'lucide-react'
import { Alert, AlertDescription, AlertTitle } from '@/components/ui/alert'
import {
  AlertDialog, AlertDialogAction, AlertDialogCancel, AlertDialogContent,
  AlertDialogDescription, AlertDialogFooter, AlertDialogHeader, AlertDialogMedia,
  AlertDialogTitle, AlertDialogTrigger,
} from '@/components/ui/alert-dialog'
import { Badge as UiBadge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Card } from '@/components/ui/card'
import { Checkbox } from '@/components/ui/checkbox'
import {
  Dialog, DialogClose, DialogContent, DialogDescription, DialogHeader, DialogTitle,
} from '@/components/ui/dialog'
import {
  Empty, EmptyDescription, EmptyHeader, EmptyMedia, EmptyTitle,
} from '@/components/ui/empty'
import { Input } from '@/components/ui/input'
import { Progress } from '@/components/ui/progress'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from '@/components/ui/select'
import { Spinner } from '@/components/ui/spinner'
import { Switch } from '@/components/ui/switch'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Textarea } from '@/components/ui/textarea'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'

const PointCloudView = lazy(() => import('./PointCloudView.jsx'))

const ALGORITHM_LABELS = {
  faster_lio: 'Faster-LIO',
  fastlio2: 'FAST-LIO2',
  faster_lio_pgo: 'Faster-LIO + SC-PGO',
}

const MODULE_LABELS = {
  crop_z: 'Z 高度裁剪',
  voxel_downsample: '体素降采样',
  statistical_outlier: '统计离群点滤波',
  radius_outlier: '半径离群点滤波',
}

const clone = (value) => JSON.parse(JSON.stringify(value))

async function api(path, options) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const data = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(data.detail || `请求失败 (${response.status})`)
  return data
}

function formatTime(value) {
  if (!value) return '时间未知'
  return new Intl.DateTimeFormat('zh-CN', {
    month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit',
  }).format(new Date(value))
}

function formatBytes(size) {
  let value = Number(size) || 0
  for (const unit of ['B', 'KB', 'MB', 'GB']) {
    if (value < 1024 || unit === 'GB') return unit === 'B' ? `${Math.round(value)} B` : `${value.toFixed(1)} ${unit}`
    value /= 1024
  }
  return `${value.toFixed(1)} GB`
}

function itemMatchesSection(item, section) {
  if (section === 'maps') return item.category === 'maps' && !item.archived
  if (section === 'processed') return item.category === 'processed' && !item.archived
  if (section === 'planning') return item.category === 'planning' && !item.archived
  if (section === 'archived') return item.archived
  return false
}

function roleLabel(item) {
  return {
    optimized: 'SC-PGO 优化', frontend: 'Faster-LIO 原图', fastlio2: 'FAST-LIO2',
    planning: '规划派生', processed: '点云处理结果', legacy: '历史点云',
  }[item?.role] || '点云'
}

function moduleParameterText(module) {
  if (!module.enabled) return '关闭'
  const value = module.parameters || {}
  if (module.type === 'crop_z') return `${value.min_z}～${value.max_z} m`
  if (module.type === 'voxel_downsample') return `${value.voxel_size} m`
  if (module.type === 'statistical_outlier') return `neighbors=${value.neighbors}, σ=${value.std_ratio}`
  if (module.type === 'radius_outlier') return `${value.radius} m / ${value.min_points} 点`
  return JSON.stringify(value)
}

function Modal({ title, subtitle, onClose, children, wide = false }) {
  return (
    <Dialog open onOpenChange={(open) => !open && onClose()}>
      <DialogContent className={`modal ${wide ? 'wide-modal' : ''}`} showCloseButton={false}>
        <DialogHeader className="modal-head">
          <div><DialogTitle>{title}</DialogTitle><DialogDescription>{subtitle}</DialogDescription></div>
          <DialogClose render={<Button variant="outline" size="icon" className="icon-button" />}><X size={18} /><span className="sr-only">关闭</span></DialogClose>
        </DialogHeader>
        {children}
      </DialogContent>
    </Dialog>
  )
}

function MetadataModal({ item, onClose, onDone, notify }) {
  const [name, setName] = useState(item.name)
  const [note, setNote] = useState(item.note || '')
  const [busy, setBusy] = useState(false)
  const submit = async (event) => {
    event.preventDefault()
    setBusy(true)
    try {
      await api(`/api/maps/${item.id}`, { method: 'PATCH', body: JSON.stringify({ name, note }) })
      notify('地图说明已更新')
      await onDone()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  return (
    <Modal title="编辑地图资料" subtitle={item.relative_path} onClose={onClose}>
      <form className="modal-form" onSubmit={submit}>
        <label><span>显示名称</span><Input autoFocus value={name} onChange={(event) => setName(event.target.value)} /></label>
        <label><span>备注</span><Textarea rows="4" value={note} onChange={(event) => setNote(event.target.value)} placeholder="记录场地、采集方式或质量判断" /></label>
        <div className="modal-actions"><Button type="button" variant="outline" className="button secondary" onClick={onClose}>取消</Button><Button type="submit" className="button primary" disabled={busy}>{busy && <Spinner />}保存</Button></div>
      </form>
    </Modal>
  )
}

function ParameterSwitch({ checked, onChange, title, copy, children, actions }) {
  return (
    <section className={`parameter-block ${checked ? 'enabled' : ''}`}>
      <div className="parameter-switch">
        <Switch checked={checked} onCheckedChange={onChange} aria-label={`${checked ? '关闭' : '启用'}${title}`} />
        <span><strong>{title}</strong><small>{copy}</small></span>{actions}
      </div>
      <div className="parameter-fields">{children}</div>
    </section>
  )
}

function NumberField({ label, value, onChange, min, max, step, disabled = false, unit }) {
  return <label className="number-field"><span>{label}</span><div><Input type="number" value={value} onChange={(event) => onChange(Number(event.target.value))} min={min} max={max} step={step} disabled={disabled} />{unit && <i>{unit}</i>}</div></label>
}

function ProcessingModal({ item, configs, onClose, onStarted, onRefresh, notify }) {
  const initialConfig = configs.find((value) => value.recommended) || configs[0]
  const [name, setName] = useState(`${item.name} · 处理后`)
  const [note, setNote] = useState('')
  const [configId, setConfigId] = useState(initialConfig?.id || '')
  const [saveName, setSaveName] = useState('')
  const [pipeline, setPipeline] = useState(() => clone(initialConfig || { modules: [] }))
  const [busy, setBusy] = useState(false)
  const [submitError, setSubmitError] = useState('')

  const selectConfig = (selectedId) => {
    const selected = configs.find((value) => value.id === selectedId)
    setConfigId(selectedId)
    if (selected) setPipeline(clone(selected))
  }
  const updateModule = (index, values) => setPipeline((current) => ({
    ...current,
    modules: current.modules.map((module, moduleIndex) => moduleIndex === index ? { ...module, ...values } : module),
  }))
  const setModuleParameter = (index, key, value) => setPipeline((current) => ({
    ...current,
    modules: current.modules.map((module, moduleIndex) => moduleIndex === index
      ? { ...module, parameters: { ...module.parameters, [key]: value } }
      : module),
  }))
  const moveModule = (index, direction) => setPipeline((current) => {
    const modules = [...current.modules]
    const nextIndex = index + direction
    if (nextIndex < 0 || nextIndex >= modules.length) return current
    ;[modules[index], modules[nextIndex]] = [modules[nextIndex], modules[index]]
    return { ...current, modules }
  })
  const saveConfig = async () => {
    if (!saveName.trim()) return notify('先填写流水线配置名称', 'error')
    setBusy(true)
    try {
      const saved = await api('/api/processing/configs', {
        method: 'POST',
        body: JSON.stringify({ ...pipeline, id: '', name: saveName.trim(), recommended: false }),
      })
      setConfigId(saved.id)
      setPipeline(clone(saved))
      notify(`流水线已保存为 ${saved.id}.yaml`)
      await onRefresh()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  const submit = async (event) => {
    event.preventDefault()
    setSubmitError('')
    setBusy(true)
    try {
      await api('/api/processing/start', {
        method: 'POST',
        body: JSON.stringify({ source_id: item.id, name, note, config_id: pipeline.id, pipeline }),
      })
      notify('点云处理已开始，完成后会自动进入“处理结果”')
      onStarted()
    } catch (error) {
      setSubmitError(error.message)
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }

  return (
    <Modal title="生成点云处理结果" subtitle={`输入：${item.name}`} onClose={onClose} wide>
      <form className="modal-form processing-form" onSubmit={submit}>
        <Alert className="source-lock"><Database size={17} /><div><AlertTitle>原始 PCD 只读</AlertTitle><AlertDescription>{item.relative_path}</AlertDescription></div><Badge tone="active">不会覆盖</Badge></Alert>
        <div className="profile-row">
          <label><span>完整流水线配置</span><Select value={configId} onValueChange={selectConfig}><SelectTrigger className="pipeline-select"><SelectValue placeholder="选择流水线">{configs.find((config) => config.id === configId)?.name || configId}</SelectValue></SelectTrigger><SelectContent>{configs.map((config) => <SelectItem key={config.id} value={config.id}>{config.name}{config.built_in ? '（内置）' : ''}</SelectItem>)}</SelectContent></Select></label>
          <label><span>另存整条流水线为 YAML</span><div><Input value={saveName} onChange={(event) => setSaveName(event.target.value)} placeholder="例如：d1max 建筑结构保真" /><Button type="button" variant="secondary" onClick={saveConfig} disabled={busy}>另存</Button></div></label>
        </div>
        <Card size="sm" className="pipeline-config-meta"><div><FileDown size={16} /><span><strong>{pipeline.name}</strong><small>{pipeline.description}</small></span></div>{configId && <a href={`/api/processing/configs/${configId}/yaml`} download><FileDown size={14} />下载 YAML</a>}</Card>
        <div className="parameter-scroll">
          {pipeline.modules.map((module, index) => <ParameterSwitch key={module.id} checked={module.enabled} onChange={(enabled) => updateModule(index, { enabled })} title={`${index + 1}. ${MODULE_LABELS[module.type] || module.type}`} copy={`模块 ID：${module.id}`} actions={<span className="module-order"><Tooltip><TooltipTrigger render={<Button type="button" variant="outline" size="icon-xs" onClick={() => moveModule(index, -1)} disabled={index === 0} />}><ArrowUp size={13} /><span className="sr-only">上移模块</span></TooltipTrigger><TooltipContent>上移模块</TooltipContent></Tooltip><Tooltip><TooltipTrigger render={<Button type="button" variant="outline" size="icon-xs" onClick={() => moveModule(index, 1)} disabled={index === pipeline.modules.length - 1} />}><ArrowDown size={13} /><span className="sr-only">下移模块</span></TooltipTrigger><TooltipContent>下移模块</TooltipContent></Tooltip></span>}>
            {module.type === 'crop_z' && <><NumberField label="Z 最小值" value={module.parameters.min_z} onChange={(value) => setModuleParameter(index, 'min_z', value)} min="-100" max="100" step="0.1" unit="m" disabled={!module.enabled} /><NumberField label="Z 最大值" value={module.parameters.max_z} onChange={(value) => setModuleParameter(index, 'max_z', value)} min="-100" max="100" step="0.1" unit="m" disabled={!module.enabled} /></>}
            {module.type === 'voxel_downsample' && <NumberField label="体素边长" value={module.parameters.voxel_size} onChange={(value) => setModuleParameter(index, 'voxel_size', value)} min="0.02" max="1" step="0.01" unit="m" disabled={!module.enabled} />}
            {module.type === 'statistical_outlier' && <><NumberField label="邻域点 neighbors" value={module.parameters.neighbors} onChange={(value) => setModuleParameter(index, 'neighbors', value)} min="2" max="500" step="1" disabled={!module.enabled} /><NumberField label="标准差倍数" value={module.parameters.std_ratio} onChange={(value) => setModuleParameter(index, 'std_ratio', value)} min="0.05" max="10" step="0.05" disabled={!module.enabled} /></>}
            {module.type === 'radius_outlier' && <><NumberField label="搜索半径" value={module.parameters.radius} onChange={(value) => setModuleParameter(index, 'radius', value)} min="0.01" max="5" step="0.01" unit="m" disabled={!module.enabled} /><NumberField label="最少邻点" value={module.parameters.min_points} onChange={(value) => setModuleParameter(index, 'min_points', value)} min="1" max="500" step="1" disabled={!module.enabled} /></>}
          </ParameterSwitch>)}
        </div>
        <div className="output-fields"><label><span>结果名称</span><Input value={name} onChange={(event) => setName(event.target.value)} required /></label><label><span>备注</span><Input value={note} onChange={(event) => setNote(event.target.value)} placeholder="可记录场地和使用目的" /></label></div>
        {submitError && <Alert variant="destructive" className="processing-error"><FileWarning /><div><AlertTitle>处理任务未能启动</AlertTitle><AlertDescription>{submitError}</AlertDescription></div></Alert>}
        <Alert className="processing-note"><AlertDescription>执行顺序完全采用 YAML 中的 modules 顺序。任务目录会保存解析后的 pipeline.yaml（含本次原始 PCD 与输出 PCD 的绝对路径）；{item.fields?.includes('intensity') ? 'intensity 会同步保留。' : '当前输入按 XYZ 输出。'}</AlertDescription></Alert>
        <div className="modal-actions"><Button type="button" variant="outline" className="button secondary" onClick={onClose}>取消</Button><Button type="submit" className="button primary" disabled={busy || !name.trim()}>{busy ? <><Spinner />正在创建任务…</> : '开始处理'}</Button></div>
      </form>
    </Modal>
  )
}

function Badge({ children, tone = 'muted' }) {
  const variant = tone === 'muted' ? 'outline' : tone === 'warning' ? 'outline' : tone === 'active' ? 'outline' : 'secondary'
  return <UiBadge variant={variant} className={`badge ${tone}`}>{children}</UiBadge>
}

function TooltipButton({ label, className = '', variant = 'outline', size = 'icon', children, ...props }) {
  return <Tooltip><TooltipTrigger render={<Button variant={variant} size={size} className={className} {...props} />}>{children}<span className="sr-only">{label}</span></TooltipTrigger><TooltipContent>{label}</TooltipContent></Tooltip>
}

function MapRow({ item, selected, onClick, selectable = false, checked = false, onCheckedChange }) {
  return (
    <div className={`map-row-shell ${selectable ? 'selectable' : ''}`}>
      {selectable && <Checkbox className="map-select" checked={checked} onCheckedChange={onCheckedChange} aria-label={`选择归档地图：${item.name}`} />}
      <Button variant="ghost" className={`map-row ${selected ? 'selected' : ''}`} onClick={onClick}>
        <div className={`map-icon ${item.role}`}><Box size={21} /></div>
        <div className="map-copy">
          <div><strong>{item.name}</strong>{item.recommended && <Sparkles size={14} className="recommended-star" />}</div>
          <span>{roleLabel(item)} · {item.points ? `${item.points.toLocaleString()} 点` : item.size_human}</span>
          <code>{formatTime(item.modified_at)}</code>
        </div>
        <div className="row-flags">
          <div className="row-badges">
            {item.active && <Badge tone="active">当前</Badge>}
            {item.latest && <Badge tone="latest">最新</Badge>}
          </div>
          <div className="row-indicators">
            {!!item.issues.length && <FileWarning size={16} className="warning" />}
            <ChevronRight size={18} />
          </div>
        </div>
      </Button>
    </div>
  )
}

function EmptyStage({ icon: Icon = Box, title, copy }) {
  return <Empty className="empty-stage"><EmptyHeader><EmptyMedia variant="icon"><Icon size={34} /></EmptyMedia><EmptyTitle>{title}</EmptyTitle><EmptyDescription>{copy}</EmptyDescription></EmptyHeader></Empty>
}

export default function App() {
  const [data, setData] = useState(null)
  const [error, setError] = useState('')
  const [section, setSection] = useState('maps')
  const [selectedId, setSelectedId] = useState('')
  const [search, setSearch] = useState('')
  const [algorithm, setAlgorithm] = useState('faster_lio_pgo')
  const [actionBusy, setActionBusy] = useState(false)
  const [compare, setCompare] = useState(false)
  const [modal, setModal] = useState(null)
  const [cleanupOpen, setCleanupOpen] = useState(false)
  const [archiveSelection, setArchiveSelection] = useState(() => new Set())
  const [archiveDelete, setArchiveDelete] = useState(null)
  const [toast, setToast] = useState(null)
  const previousProcessingStatus = useRef(null)
  const sectionRef = useRef(section)

  useEffect(() => {
    sectionRef.current = section
  }, [section])

  const notify = useCallback((message, type = 'success') => {
    setToast({ message, type })
    window.setTimeout(() => setToast(null), 3600)
  }, [])

  const refresh = useCallback(async (quiet = false) => {
    try {
      const result = await api('/api/overview')
      setData(result)
      setError('')
      setSelectedId((current) => {
        const currentSection = sectionRef.current
        if (current && result.items.some((item) => item.id === current && itemMatchesSection(item, currentSection))) return current
        return result.items.find((item) => itemMatchesSection(item, currentSection))?.id || ''
      })
      return result
    } catch (requestError) {
      setError(requestError.message)
      if (!quiet) notify(requestError.message, 'error')
    }
  }, [notify])

  useEffect(() => {
    refresh()
    const timer = window.setInterval(() => refresh(true), 4000)
    return () => window.clearInterval(timer)
  }, [refresh])

  const selected = data?.items.find((item) => item.id === selectedId) || null
  const peerId = selected ? data?.comparisons?.[selected.id] : null
  const peer = peerId ? data.items.find((item) => item.id === peerId) : null
  useEffect(() => setCompare(false), [selectedId])

  useEffect(() => {
    const job = data?.processing_job
    if (!job || job.status === previousProcessingStatus.current) return
    const previous = previousProcessingStatus.current
    previousProcessingStatus.current = job.status
    if (previous === null) return
    if (job.status === 'complete' && job.result_id && ['running', 'cancelling'].includes(previous)) {
      setSection('processed')
      setSelectedId(job.result_id)
      notify('点云处理完成，新 PCD 已保存')
    } else if (job.status === 'failed' && ['running', 'cancelling'].includes(previous)) {
      notify(`点云处理失败：${job.error || '请查看任务日志'}`, 'error')
    }
  }, [data?.processing_job, notify])

  const list = useMemo(() => {
    if (!data) return []
    const values = data.items.filter((item) => itemMatchesSection(item, section))
    const needle = search.trim().toLowerCase()
    return needle ? values.filter((item) => `${item.name} ${item.relative_path} ${item.family}`.toLowerCase().includes(needle)) : values
  }, [data, section, search])

  const archiveItems = useMemo(() => data?.items.filter((item) => item.archived) || [], [data])
  const selectedArchiveItems = useMemo(
    () => archiveItems.filter((item) => archiveSelection.has(item.id)),
    [archiveItems, archiveSelection],
  )
  const sectionCounts = useMemo(() => ({
    maps: data?.items.filter((item) => item.category === 'maps' && !item.archived).length || 0,
    processed: data?.items.filter((item) => item.category === 'processed' && !item.archived).length || 0,
    planning: data?.items.filter((item) => item.category === 'planning' && !item.archived).length || 0,
    archived: archiveItems.length,
  }), [data, archiveItems])
  const allVisibleArchivesSelected = section === 'archived' && list.length > 0 && list.every((item) => archiveSelection.has(item.id))
  const someVisibleArchivesSelected = section === 'archived' && list.some((item) => archiveSelection.has(item.id)) && !allVisibleArchivesSelected

  useEffect(() => {
    const validIds = new Set(archiveItems.map((item) => item.id))
    setArchiveSelection((current) => {
      const next = new Set([...current].filter((itemId) => validIds.has(itemId)))
      if (next.size === current.size && [...next].every((itemId) => current.has(itemId))) return current
      return next
    })
  }, [archiveItems])

  const runtime = data?.runtime || { status: 'idle', logs: [] }
  const runtimeBusy = ['running', 'stopping'].includes(runtime.status)

  const switchSection = (nextSection) => {
    sectionRef.current = nextSection
    setSection(nextSection)
    setSearch('')
    if (!data || nextSection === 'failures') {
      setSelectedId('')
      return
    }
    const candidate = data.items.find((item) => itemMatchesSection(item, nextSection))
    setSelectedId(candidate?.id || '')
  }

  const runAction = async (path, body, success) => {
    setActionBusy(true)
    try {
      await api(path, { method: 'POST', body: body ? JSON.stringify(body) : undefined })
      notify(success)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const updateMetadata = async (item, values, success) => {
    setActionBusy(true)
    try {
      await api(`/api/maps/${item.id}`, { method: 'PATCH', body: JSON.stringify(values) })
      notify(success)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const toggleVisibleArchives = (checked) => {
    setArchiveSelection((current) => {
      const next = new Set(current)
      list.forEach((item) => checked ? next.add(item.id) : next.delete(item.id))
      return next
    })
  }

  const toggleArchiveItem = (itemId, checked) => {
    setArchiveSelection((current) => {
      const next = new Set(current)
      if (checked) next.add(itemId)
      else next.delete(itemId)
      return next
    })
  }

  const openArchiveDeletion = (items, deleteAll = false) => {
    const targets = deleteAll ? archiveItems : items
    if (!targets.length) {
      notify(deleteAll ? '归档区已经是空的' : '请先勾选要永久删除的归档地图', 'error')
      return
    }
    setArchiveDelete({
      deleteAll,
      itemIds: targets.map((item) => item.id),
      count: targets.length,
      bytes: targets.reduce((total, item) => total + (item.size || 0), 0),
    })
  }

  const confirmArchiveDeletion = async () => {
    if (!archiveDelete || actionBusy) return
    setActionBusy(true)
    try {
      const result = await api('/api/archive/delete', {
        method: 'POST',
        body: JSON.stringify({
          item_ids: archiveDelete.deleteAll ? [] : archiveDelete.itemIds,
          delete_all: archiveDelete.deleteAll,
        }),
      })
      setArchiveDelete(null)
      setArchiveSelection(new Set())
      notify(`已永久删除 ${result.deleted_count} 项，释放 ${result.freed_size_human}`)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const copyPath = async (value) => {
    await navigator.clipboard.writeText(value)
    notify('路径已复制')
  }

  const processingJob = data?.processing_job || { status: 'idle', running: false, logs: [] }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand"><div className="brand-mark"><Layers3 size={23} /></div><div><strong>D1 Max 地图工作台</strong><span>3D Map Workspace</span></div></div>
        <div className="active-strip">
          {data?.active ? <><i /><span>当前标记</span><strong>{data.active.name}</strong><code>{roleLabel(data.active)}</code></> : <><b /><span>尚未标记当前地图</span>{data?.recommended && <strong>推荐：{data.recommended.name}</strong>}</>}
        </div>
        <div className="top-actions">
          <Badge tone="blue"><Box size={13} />仅 3D</Badge>
          <Button variant="outline" size="sm" className="button secondary compact" onClick={() => refresh()} disabled={actionBusy}><RefreshCw data-icon="inline-start" size={16} />重新扫描</Button>
          <AlertDialog open={cleanupOpen} onOpenChange={setCleanupOpen}>
            <AlertDialogTrigger render={<Button variant="destructive" size="sm" className="button danger-soft compact" disabled={actionBusy} />}><Trash2 data-icon="inline-start" size={16} />清理进程</AlertDialogTrigger>
            <AlertDialogContent>
              <AlertDialogHeader><AlertDialogMedia><Trash2 /></AlertDialogMedia><AlertDialogTitle>清理 Web 建图进程？</AlertDialogTitle><AlertDialogDescription>只会停止由本地图工作台启动的建图进程，不会删除任何地图或点云文件。</AlertDialogDescription></AlertDialogHeader>
              <AlertDialogFooter><AlertDialogCancel>取消</AlertDialogCancel><AlertDialogAction variant="destructive" onClick={() => { setCleanupOpen(false); runAction('/api/runtime/cleanup', null, 'Web 建图进程已清理') }}>确认清理</AlertDialogAction></AlertDialogFooter>
            </AlertDialogContent>
          </AlertDialog>
        </div>
      </header>

      <section className="runtime-bar">
        <Card size="sm" className={`runtime-state ${runtime.status}`}><span className="runtime-pulse" /><div><strong>{runtime.status === 'running' ? `${ALGORITHM_LABELS[runtime.algorithm]} 运行中` : runtime.status === 'failed' ? '建图流程异常' : runtime.status === 'detached' ? '检测到旧建图流程' : '建图流程未运行'}</strong><span>{runtime.error || (runtime.started_at ? `最近启动 ${formatTime(runtime.started_at)}` : 'ROS Domain 24 · rmw_zenoh_cpp')}</span></div></Card>
        <div className="mapping-pipeline">
          <Card size="sm" className={`pipeline-stage ${runtimeBusy ? 'running' : ''}`}>
            <div className="pipeline-title"><span>1</span><div><strong>选择建图后端</strong><Select value={algorithm} onValueChange={setAlgorithm} disabled={runtimeBusy}><SelectTrigger className="algorithm-select"><SelectValue>{data?.algorithms?.find((item) => item.id === algorithm)?.name || ALGORITHM_LABELS[algorithm]}</SelectValue></SelectTrigger><SelectContent>{data?.algorithms?.map((item) => <SelectItem value={item.id} key={item.id}>{item.name}</SelectItem>)}</SelectContent></Select></div></div>
            {!runtimeBusy ? <Button onClick={() => runAction('/api/runtime/start', { algorithm }, `${ALGORITHM_LABELS[algorithm]} 已启动`)} disabled={actionBusy}><Play data-icon="inline-start" size={15} />开始建图</Button> : <Button variant="destructive" className="stop" onClick={() => runAction('/api/runtime/stop', null, '建图已停止并触发落盘')} disabled={actionBusy || runtime.status === 'stopping'}><CircleStop data-icon="inline-start" size={15} />结束建图</Button>}
          </Card>
          <ChevronRight className="pipeline-arrow" />
          <Card size="sm" className="pipeline-stage">
            <div className="pipeline-title"><span>2</span><div><strong>保存当前地图</strong><small>/d1max/slam/save</small></div></div>
            <Button onClick={() => runAction('/api/runtime/save', null, '地图保存服务调用成功')} disabled={actionBusy || runtime.status !== 'running'}><Save data-icon="inline-start" size={15} />立即保存</Button>
          </Card>
          <ChevronRight className="pipeline-arrow" />
          <Card size="sm" className="pipeline-stage readonly">
            <div className="pipeline-title"><span>3</span><div><strong>3D 结果纳管</strong><small>PCD / PLY 自动扫描，不生成栅格图</small></div></div>
            <Check size={19} />
          </Card>
        </div>
      </section>

      <main className="workspace">
        <Card className="sidebar">
          <Tabs value={section} onValueChange={switchSection} className="sidebar-tabs">
            <TabsList className="section-tabs">
              <TabsTrigger value="maps" className={section === 'maps' ? 'active' : ''}><Database data-icon="inline-start" size={17} />建图结果<span>{sectionCounts.maps}</span></TabsTrigger>
              <TabsTrigger value="processed" className={section === 'processed' ? 'active' : ''}><WandSparkles data-icon="inline-start" size={17} />处理结果<span>{sectionCounts.processed}</span></TabsTrigger>
              <TabsTrigger value="planning" className={section === 'planning' ? 'active' : ''}><Activity data-icon="inline-start" size={17} />规划产物<span>{sectionCounts.planning}</span></TabsTrigger>
              <TabsTrigger value="archived" className={section === 'archived' ? 'active' : ''}><Archive data-icon="inline-start" size={17} />已归档<span>{sectionCounts.archived}</span></TabsTrigger>
              <TabsTrigger value="failures" className={section === 'failures' ? 'active' : ''}><FileWarning data-icon="inline-start" size={17} />异常记录<span>{data?.summary.failure_count || 0}</span></TabsTrigger>
            </TabsList>
            <TabsContent value={section} className="sidebar-tab-content">
              {section !== 'failures' && <div className="search"><Search size={16} /><Input value={search} onChange={(event) => setSearch(event.target.value)} placeholder="搜索名称、算法或路径" /></div>}
              {section === 'archived' && <div className="archive-toolbar">
                <label className="archive-select-all"><Checkbox checked={allVisibleArchivesSelected} indeterminate={someVisibleArchivesSelected} onCheckedChange={toggleVisibleArchives} aria-label="全选当前归档列表" /><span>全选</span></label>
                <span className="archive-selected-count">已选 {selectedArchiveItems.length}</span>
                <Button variant="outline" size="xs" onClick={() => openArchiveDeletion(selectedArchiveItems)} disabled={!selectedArchiveItems.length || actionBusy}><Trash2 data-icon="inline-start" />删除所选</Button>
                <Button variant="destructive" size="xs" onClick={() => openArchiveDeletion(archiveItems, true)} disabled={!archiveItems.length || actionBusy}><Trash2 data-icon="inline-start" />全部删除</Button>
              </div>}
              <ScrollArea className="item-list">
                {section === 'failures' ? data?.failures.map((item) => <div className="failure-row" key={item.id}><div><FileWarning size={18} /></div><span><strong>{item.name}</strong><small>{item.reason}</small><code>{item.size_human} · {formatTime(item.modified_at)}</code></span></div>) : list.map((item) => <MapRow item={item} selected={item.id === selectedId} onClick={() => setSelectedId(item.id)} selectable={section === 'archived'} checked={archiveSelection.has(item.id)} onCheckedChange={(checked) => toggleArchiveItem(item.id, checked)} key={item.id} />)}
                {data && section !== 'failures' && !list.length && <Empty className="list-empty"><EmptyHeader><EmptyMedia variant="icon"><FolderOpen size={25} /></EmptyMedia><EmptyTitle>{section === 'archived' ? '归档区已清空' : '这里还没有点云'}</EmptyTitle><EmptyDescription>{section === 'archived' ? '归档后的地图会显示在这里，可恢复或永久删除。' : '重新扫描后仍会保留原始目录结构'}</EmptyDescription></EmptyHeader></Empty>}
                {data && section === 'failures' && !data.failures.length && <Empty className="list-empty"><EmptyHeader><EmptyMedia variant="icon"><Check size={25} /></EmptyMedia><EmptyTitle>没有空运行目录</EmptyTitle></EmptyHeader></Empty>}
              </ScrollArea>
            </TabsContent>
          </Tabs>
        </Card>

        <Card className="preview-area">
          {selected ? <>
            <div className="preview-toolbar">
              <div><strong>{selected.name}</strong><span>{selected.relative_path}</span></div>
              <div className="toolbar-badges"><Badge tone={selected.issues.length ? 'warning' : 'active'}>{selected.issues.length ? `${selected.issues.length} 项提示` : '文件完整'}</Badge>{selected.recommended && <Badge tone="blue"><Sparkles size={12} />回环验证</Badge>}</div>
              <div className="preview-actions">{peer && <Button variant="outline" size="sm" className={`view-button ${compare ? 'active' : ''}`} onClick={() => setCompare((value) => !value)}><GitCompareArrows data-icon="inline-start" size={16} />{compare ? '单图查看' : selected.category === 'processed' ? '原图 / 处理后对比' : '前端 / PGO 对比'}</Button>}</div>
            </div>
            <div className={`map-stage ${compare && peer ? 'compare' : ''}`}>
              <Suspense fallback={<div className="canvas-overlay"><Spinner />正在载入 3D 模块</div>}>
                <div className="cloud-panel"><PointCloudView url={selected.cloud_url} compact={compare} />{compare && <span className="cloud-label">A · {roleLabel(selected)}</span>}</div>
                {compare && peer && <div className="cloud-panel"><PointCloudView url={peer.cloud_url} compact /><span className="cloud-label">B · {roleLabel(peer)}</span></div>}
              </Suspense>
            </div>
            <div className="preview-foot"><span>左键旋转 · 右键平移 · 滚轮缩放</span><span>高度着色 · Z 轴向上</span></div>
          </> : section === 'failures' ? <EmptyStage icon={FileWarning} title="异常与空运行记录" copy="这些目录没有可用点云，不会进入地图版本列表；Web 不会自动删除它们。" /> : <EmptyStage title="选择一个 3D 点云" copy="支持 PCD 和 PLY；PCD 首次查看会生成轻量预览缓存。" />}
        </Card>

        <Card className="inspector">
          <ScrollArea className="inspector-scroll">
          {selected ? <>
            <div className="inspector-head"><div className={`large-map-icon ${selected.role}`}><Box size={26} /></div><div><span>{selected.family}</span><h2>{selected.name}</h2><div>{selected.active && <Badge tone="active">当前</Badge>}{selected.recommended && <Badge tone="blue">推荐</Badge>}{selected.latest && <Badge tone="latest">最新落盘</Badge>}</div></div></div>
            {!!selected.issues.length && <Alert className="issue-panel"><FileWarning size={18} /><div><AlertTitle>质量提示</AlertTitle>{selected.issues.map((issue) => <AlertDescription key={issue}>{issue}</AlertDescription>)}</div></Alert>}
            <section className="detail-section"><h3>点云信息</h3><dl><div><dt>角色</dt><dd>{roleLabel(selected)}</dd></div><div><dt>点数</dt><dd>{selected.points?.toLocaleString() || '未记录'}</dd></div><div><dt>文件大小</dt><dd>{selected.size_human}</dd></div><div><dt>格式</dt><dd>{selected.extension.toUpperCase()} · {selected.storage || '未知编码'}</dd></div><div><dt>修改时间</dt><dd>{formatTime(selected.modified_at)}</dd></div></dl></section>
            {selected.category === 'processed' && <section className="detail-section processing-details"><h3>处理记录</h3><dl><div><dt>源地图</dt><dd title={selected.source_path}>{selected.source_name || '源文件已移动'}</dd></div><div><dt>原始点数</dt><dd>{selected.input_points?.toLocaleString() || '未记录'}</dd></div><div><dt>减少比例</dt><dd>{selected.removed_percent == null ? '未计算' : `${selected.removed_percent}%`}</dd></div><div><dt>流水线配置</dt><dd>{selected.config_name || selected.profile_name || '旧版参数'}</dd></div></dl><div className="stage-counts">{selected.stage_counts?.map((stage) => <div key={stage.stage}><span>{stage.label}</span><strong>{stage.points?.toLocaleString()}</strong></div>)}</div>{selected.pipeline?.modules ? <div className="parameter-summary">{selected.pipeline.modules.map((module, index) => <Fragment key={module.id}><span>{index + 1}. {MODULE_LABELS[module.type] || module.type}</span><code>{moduleParameterText(module)}</code></Fragment>)}</div> : <div className="parameter-summary"><span>统计</span><code>{selected.parameters?.statistical_enabled ? `k=${selected.parameters.statistical_mean_k}, σ=${selected.parameters.statistical_std_dev_mul}` : '关闭'}</code><span>半径</span><code>{selected.parameters?.radius_enabled ? `${selected.parameters.radius}m / ${selected.parameters.radius_min_points}点` : '关闭'}</code><span>体素</span><code>{selected.parameters?.voxel_enabled ? `${selected.parameters.voxel_leaf}m` : '关闭'}</code><span>Z 裁剪</span><code>{selected.parameters?.crop_enabled ? `${selected.parameters.z_min}～${selected.parameters.z_max}m` : '关闭'}</code></div>}</section>}
            {selected.pair_key && <section className="detail-section"><h3>回环统计</h3><div className="loop-stats"><div className={selected.accepted_loops ? 'good' : ''}><strong>{selected.accepted_loops}</strong><span>接受约束</span></div><div><strong>{selected.rejected_loops}</strong><span>拒绝候选</span></div></div>{!selected.accepted_loops && selected.role === 'optimized' && <p className="muted-copy">文件名是优化图，但本次日志没有接受任何回环，不能按“闭环成功”理解。</p>}</section>}
            <section className="detail-section"><h3>文件路径</h3><div className="path-row"><code title={selected.path}>{selected.path}</code><TooltipButton label="复制点云路径" className="icon-button" onClick={() => copyPath(selected.path)}><Copy size={15} /></TooltipButton></div>{selected.pipeline_path && <div className="path-row pipeline-path"><code title={selected.pipeline_path}>{selected.pipeline_path}</code><TooltipButton label="复制流水线路径" className="icon-button" onClick={() => copyPath(selected.pipeline_path)}><Copy size={15} /></TooltipButton></div>}</section>
            {selected.note && <section className="detail-section"><h3>备注</h3><p>{selected.note}</p></section>}
            <div className="inspector-actions">
              {selected.category === 'maps' && <Button variant="secondary" className="button process wide" onClick={() => setModal({ type: 'processing', item: selected })} disabled={processingJob.running}><SlidersHorizontal data-icon="inline-start" size={16} />{processingJob.running ? '已有点云处理任务运行中' : '配置参数并处理 PCD'}</Button>}
              <Button variant="outline" className="button secondary wide" onClick={() => setModal({ type: 'metadata', item: selected })}><Pencil data-icon="inline-start" size={16} />编辑名称与备注</Button>
              {['maps', 'processed'].includes(selected.category) && !selected.active && <Button className="button primary wide" onClick={() => runAction('/api/active', { map_id: selected.id }, '已标为当前 3D 地图')} disabled={actionBusy}><MapPin data-icon="inline-start" size={16} />标为当前 3D 地图</Button>}
              {selected.archived && <Button variant="destructive" className="button danger-soft wide" onClick={() => openArchiveDeletion([selected])} disabled={actionBusy}><Trash2 data-icon="inline-start" size={16} />永久删除此项</Button>}
              <Button variant="ghost" className="button ghost wide" onClick={() => updateMetadata(selected, { archived: !selected.archived }, selected.archived ? '已恢复到地图列表' : '已无损归档')} disabled={actionBusy}>{selected.archived ? <RotateCcw data-icon="inline-start" size={16} /> : <Archive data-icon="inline-start" size={16} />}{selected.archived ? '恢复显示' : '无损归档'}</Button>
            </div>
          </> : <Empty className="inspector-empty"><EmptyHeader><EmptyMedia variant="icon"><ServerCog size={25} /></EmptyMedia><EmptyDescription>选择点云后查看文件、算法和回环质量</EmptyDescription></EmptyHeader></Empty>}
          </ScrollArea>
        </Card>
      </main>

      {runtime.logs?.length > 0 && runtimeBusy && <div className="log-drawer"><Activity size={16} /><strong>建图日志</strong><code>{runtime.logs.at(-1)}</code><span>PID {runtime.pid}</span></div>}
      {processingJob.running && <div className="processing-drawer"><WandSparkles size={17} /><div><strong>点云处理中 · {processingJob.progress || 0}%</strong><span>{processingJob.logs?.at(-1) || '正在准备'}</span><Progress value={processingJob.progress || 0} className="processing-progress" /></div><Button variant="outline" size="sm" onClick={() => runAction('/api/processing/cancel', null, '已请求取消点云处理')} disabled={actionBusy || processingJob.status === 'cancelling'}>{processingJob.status === 'cancelling' ? '取消中' : '取消'}</Button></div>}
      {error && <div className="connection-error"><FileWarning size={17} />后端连接失败：{error}</div>}
      {toast && <div className={`toast ${toast.type}`}><span>{toast.type === 'error' ? <X size={16} /> : <Check size={16} />}</span>{toast.message}</div>}
      {modal?.type === 'metadata' && <MetadataModal item={modal.item} onClose={() => setModal(null)} onDone={async () => { setModal(null); await refresh(true) }} notify={notify} />}
      {modal?.type === 'processing' && <ProcessingModal item={modal.item} configs={data?.processing_configs || []} onClose={() => setModal(null)} onStarted={async () => { setModal(null); await refresh(true) }} onRefresh={() => refresh(true)} notify={notify} />}
      <AlertDialog open={Boolean(archiveDelete)} onOpenChange={(open) => !open && !actionBusy && setArchiveDelete(null)}>
        <AlertDialogContent className="archive-delete-dialog">
          <AlertDialogHeader>
            <AlertDialogMedia className="archive-delete-media"><Trash2 /></AlertDialogMedia>
            <AlertDialogTitle>永久删除 {archiveDelete?.count || 0} 个归档点云？</AlertDialogTitle>
            <AlertDialogDescription>预计清理 {formatBytes(archiveDelete?.bytes)}。此操作会直接删除磁盘文件及 Web 预览缓存，无法恢复；处理结果的 pipeline.yaml 和 manifest.json 也会一并删除。未归档地图不会受影响。</AlertDialogDescription>
          </AlertDialogHeader>
          <Alert className="permanent-delete-warning" variant="destructive"><FileWarning /><div><AlertTitle>这不是无损归档</AlertTitle><AlertDescription>{archiveDelete?.deleteAll ? '将清空整个归档区，包括当前搜索结果之外的条目。' : '只会删除这次勾选或指定的归档条目。'}</AlertDescription></div></Alert>
          <AlertDialogFooter><AlertDialogCancel disabled={actionBusy}>取消</AlertDialogCancel><AlertDialogAction variant="destructive" onClick={confirmArchiveDeletion} disabled={actionBusy}>{actionBusy ? <><Spinner />正在清理磁盘…</> : '确认永久删除'}</AlertDialogAction></AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  )
}
