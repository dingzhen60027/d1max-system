import { useEffect, useRef, useState } from 'react'
import { Archive, ArchiveRestore, CheckCircle2, Circle, Copy, Database, Download, FileWarning, HardDrive, LoaderCircle, Pencil, Radio, RefreshCw, Search, Square, Trash2 } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Card, CardContent } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Textarea } from '@/components/ui/textarea'
import { Checkbox } from '@/components/ui/checkbox'
import { Switch } from '@/components/ui/switch'
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription } from '@/components/ui/dialog'
import { AlertDialog, AlertDialogContent, AlertDialogHeader, AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel } from '@/components/ui/alert-dialog'
import './bags.css'

const STATUS = { idle: '待录制', preparing: '等待传感器', recording: '正在录制', stopping: '正在保存', complete: '已保存', ready: '已保存', cancelled: '已取消', failed: '录制失败', interrupted: '录制中断', incomplete: '待检查' }
const size = n => { if (!n) return '0 B'; const i = Math.min(3, Math.floor(Math.log(n) / Math.log(1024))); return `${(n / 1024 ** i).toFixed(i ? 1 : 0)} ${['B', 'KiB', 'MiB', 'GiB'][i]}` }
const duration = n => `${Math.floor((n || 0) / 3600).toString().padStart(2, '0')}:${(Math.floor((n || 0) / 60) % 60).toString().padStart(2, '0')}:${Math.floor((n || 0) % 60).toString().padStart(2, '0')}`
const count = n => (n || 0).toLocaleString()
async function api(path, options = {}) {
  const response = await fetch('/api/bags' + path, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } })
  const result = await response.json()
  if (!response.ok) throw Error(typeof result.detail === 'string' ? result.detail : '请求失败，请检查输入')
  return result
}
const Status = ({ value }) => <Badge variant="outline" className={'bag-status ' + value}>{value === 'recording' ? <Radio/> : ['preparing', 'stopping'].includes(value) ? <LoaderCircle className="animate-spin"/> : ['complete', 'ready'].includes(value) ? <CheckCircle2/> : ['incomplete', 'failed', 'interrupted'].includes(value) ? <FileWarning/> : <Circle/>}{STATUS[value] || value}</Badge>

export default function WorkspaceBags({ section }) {
  const [data, setData] = useState(null), [runtime, setRuntime] = useState({ status: 'idle', busy: false })
  const [error, setError] = useState(''), [connectionError, setConnectionError] = useState(''), [notice, setNotice] = useState('')
  const [busy, setBusy] = useState(false), actionLock = useRef(false), initialized = useRef(false)
  const [name, setName] = useState('原始采集'), [groups, setGroups] = useState(['core', 'motion'])
  const [search, setSearch] = useState(''), [selectedId, setSelectedId] = useState(null), [detail, setDetail] = useState(null)
  const [selection, setSelection] = useState([]), [deleting, setDeleting] = useState(null), [editing, setEditing] = useState(null)
  const [revision, setRevision] = useState(0)
  const archived = section === 'archived'
  const refresh = () => setRevision(v => v + 1)
  useEffect(() => {
    let alive = true, timer, cycles = 0
    const controller = new AbortController()
    async function poll() {
      try {
        if (cycles++ % 8 === 0) {
          const result = await api('/overview', { signal: controller.signal })
          if (alive) {
            setData(result); setRuntime(result.runtime)
            if (!initialized.current) { setGroups(result.config.groups.filter(g => g.default).map(g => g.id)); initialized.current = true }
          }
        } else {
          const result = await api('/recording', { signal: controller.signal })
          if (alive) setRuntime(previous => { if (previous.busy && !result.busy) cycles = 0; return result })
        }
        if (alive) setConnectionError('')
      } catch (e) { if (alive) setConnectionError('服务连接中断，录制状态未知。请恢复连接后核对。') }
      if (alive) timer = setTimeout(poll, 1200)
    }
    poll()
    return () => { alive = false; controller.abort(); clearTimeout(timer) }
  }, [revision])
  const items = (data?.items || []).filter(i => i.archived === archived && `${i.name} ${i.note} ${i.path}`.toLowerCase().includes(search.toLowerCase()))
  const focused = items.find(i => i.id === selectedId) || items[0]
  useEffect(() => {
    setSelection([]); setSelectedId(null); setDetail(null)
  }, [section])
  useEffect(() => {
    setSelection(previous => previous.filter(id => items.some(i => i.id === id && !i.active)))
  }, [data, section, search])
  useEffect(() => {
    if (!focused) { setDetail(null); return }
    let alive = true
    setDetail(current => current?.id === focused.id ? current : null)
    api('/' + focused.id).then(result => { if (alive) setDetail(result) }).catch(e => { if (alive) setError(e.message) })
    return () => { alive = false }
  }, [focused?.id, data, revision])
  async function action(fn, message = '') {
    if (actionLock.current) return
    actionLock.current = true; setBusy(true); setError(''); setNotice('')
    try { await fn(); setNotice(message); refresh() } catch (e) { setError(e.message) }
    finally { actionLock.current = false; setBusy(false) }
  }
  function start() {
    return action(async () => setRuntime(await api('/recording/start', { method: 'POST', body: JSON.stringify({ name, groups }) })))
  }
  function stop() {
    return action(async () => setRuntime(await api('/recording/stop', { method: 'POST' })))
  }
  function patch(id, changes) { return api('/' + id, { method: 'PATCH', body: JSON.stringify(changes) }) }
  function removeSelected() {
    const ids = deleting.map(item => item.id)
    return action(async () => {
      await api('/delete', { method: 'POST', body: JSON.stringify({ ids, permanent: true }) })
      // Remove confirmed targets immediately; otherwise clearing focus can issue
      // a detail GET for a just-deleted item before the next catalog refresh.
      setData(current => ({ ...current, items: current.items.filter(item => !ids.includes(item.id)) }))
      setDeleting(null); setSelection([]); setSelectedId(null)
    }, '已永久删除所选 Bag，无法恢复')
  }
  const selectedItems = items.filter(i => selection.includes(i.id))
  const disabled = busy || runtime.busy || !!connectionError || !data
  const configured = data?.config.topics.filter(t => (runtime.busy ? runtime.groups || groups : groups).includes(t.group)) || []
  const topicRows = runtime.busy ? configured.map(t => ({ ...t, ...runtime.topics?.find(r => r.name === t.name) })) : configured
  return <div className="bags-workspace">
    <header className="workspace-heading"><div><span className="eyebrow">ROSBAG</span><h1>{archived ? 'Bag 归档' : '数据采集'}</h1></div>
      <div className="heading-actions"><Badge variant="outline"><HardDrive/>可用 {data ? size(data.disk.free) : '—'}</Badge><Button variant="outline" onClick={refresh} disabled={busy}><RefreshCw/>刷新</Button></div>
    </header>
    <div className="bags-scroll">
      {(error || connectionError) && <div className="bag-alert error" role="alert"><FileWarning/>{error || connectionError}</div>}
      {notice && <div className="bag-alert success" role="status"><CheckCircle2/>{notice}</div>}
      {!archived && <Card className="recording-card"><CardContent>
        <div className="recording-top"><div className="recording-title"><span className="recording-icon"><Radio/></span><div><h2>新建录制</h2><Status value={runtime.status}/></div></div>
          <div className="recording-metrics"><span><small>录制时长</small><strong>{duration(runtime.elapsed_sec)}</strong></span><span><small>已写入</small><strong>{size(runtime.size_bytes)}</strong></span></div>
          {runtime.busy ? <Button variant="destructive" disabled={busy || runtime.status === 'stopping' || !!connectionError} onClick={stop}>{runtime.status === 'stopping' ? <LoaderCircle className="animate-spin"/> : <Square/>}{runtime.status === 'stopping' ? '保存中…' : runtime.status === 'preparing' ? '取消等待' : '停止并保存'}</Button>
            : <Button disabled={disabled || !name.trim()} onClick={start}><Circle className="record-dot"/>开始录制</Button>}
        </div>
        {runtime.error && <div className="bag-alert error" role="alert"><FileWarning/>{runtime.error}</div>}
        <div className="recording-settings"><label className="record-name">录制名称<Input aria-label="录制名称" value={name} maxLength={100} disabled={disabled} onChange={e => setName(e.target.value)}/></label>
          <div className="record-groups">{data?.config.groups.map(group => <div className="record-group" key={group.id}><Switch aria-labelledby={'bag-group-' + group.id} checked={(runtime.busy ? runtime.groups || groups : groups).includes(group.id)} disabled={disabled || group.locked} onCheckedChange={checked => setGroups(previous => checked ? [...previous, group.id] : previous.filter(g => g !== group.id))}/><span id={'bag-group-' + group.id}>{group.name}</span>{group.locked && <Badge variant="secondary">必录</Badge>}</div>)}</div>
        </div>
        <div className="recording-foot"><code title={runtime.output_path || data?.output_root}>{runtime.busy ? runtime.output_path : data?.output_root}</code><Badge variant="secondary">Zenoh · Domain {data?.config.transport.domain_id ?? 24}</Badge></div>
        <details className="recording-details"><summary>话题状态 · {configured.length} 路</summary><div className="bag-topic-grid">{topicRows.map(topic => <div key={topic.name}><code>{topic.name}</code><span>{runtime.busy ? topic.count ? `${topic.hz || 0} Hz` : '等待数据' : topic.required ? '必需' : '可选'}</span></div>)}</div><p>接收频率仅供监测，入包数量以保存后统计为准。</p></details>
        {!!runtime.logs?.length && <details className="recording-details"><summary>运行日志</summary><pre>{runtime.logs.join('\n')}</pre></details>}
      </CardContent></Card>}
      {archived && runtime.busy && <div className="bag-alert"><Radio/>录制仍在进行，请返回采集页停止并保存。</div>}
      <div className="bag-library-head"><div><h2>{archived ? '已归档' : '已录制'}<Badge variant="secondary">{items.length}</Badge></h2></div>
        <label className="bag-search"><Search/><Input aria-label="搜索 Bag" placeholder="搜索名称、备注、路径" value={search} onChange={e => setSearch(e.target.value)}/></label>
      </div>
      <div className="bag-library">
        <section className="bag-list" aria-label="Bag 列表">
          <div className="bag-selection"><Checkbox aria-label="全选当前列表" checked={!!items.length && items.filter(i => !i.active).every(i => selection.includes(i.id))} indeterminate={!!selection.length && selection.length < items.filter(i => !i.active).length} disabled={!items.some(i => !i.active) || busy} onCheckedChange={checked => setSelection(checked ? items.filter(i => !i.active).map(i => i.id).slice(0, 200) : [])}/><span>{selection.length ? `已选 ${selection.length}` : '全选'}</span>
            {archived ? <><Button variant="ghost" disabled={!selection.length || busy} onClick={() => setDeleting(selectedItems)}><Trash2/>删除所选</Button><Button variant="ghost" disabled={!items.length || busy} onClick={() => setDeleting(items.slice(0, 200))}>删除列表全部</Button></>
              : <Button variant="ghost" disabled={!selection.length || busy} onClick={() => action(async () => { for (const id of selection) await patch(id, { archived: true }); setSelection([]) }, '已归档，文件未删除')}><Archive/>归档所选</Button>}
          </div>
          {items.map(item => <div key={item.id} className={'bag-row ' + (focused?.id === item.id ? 'selected' : '')}>
            <Checkbox aria-label={'选择 ' + item.name} checked={selection.includes(item.id)} disabled={busy || item.active} onCheckedChange={checked => setSelection(previous => checked ? [...previous, item.id] : previous.filter(id => id !== item.id))}/>
            <button className="bag-row-main" onClick={() => setSelectedId(item.id)}><span className="bag-file-icon"><Database/></span><span className="bag-row-copy"><strong>{item.name}</strong><span>{duration(item.duration_sec)} · {size(item.size_bytes)} · {item.topic_count} 路话题</span><small>{new Date(item.modified_at * 1000).toLocaleString('zh-CN', { hour12: false })}</small></span><Status value={item.status}/></button>
          </div>)}
          {!items.length && <div className="bag-empty"><Database/><strong>{search ? '没有匹配的 Bag' : archived ? '暂无归档' : '暂无录制文件'}</strong><span>{!archived && !search ? '连接机器狗后可录制。' : ' '}</span></div>}
        </section>
        <aside className="bag-inspector">
          {detail ? <>
            <div className="bag-detail-title"><span className="bag-file-icon"><Database/></span><h2>{detail.name}</h2><Status value={detail.status}/></div>
            <div className="bag-detail-actions"><Button variant="outline" disabled={busy || detail.active} onClick={() => setEditing({ id: detail.id, name: detail.name, note: detail.note })}><Pencil/>名称与备注</Button><Button variant="outline" disabled={busy || detail.active} onClick={() => action(() => patch(detail.id, { archived: !detail.archived }), detail.archived ? '已恢复' : '已归档，文件未删除')}>{detail.archived ? <ArchiveRestore/> : <Archive/>}{detail.archived ? '恢复' : '归档'}</Button>{detail.archived && <Button variant="destructive" disabled={busy || detail.active} onClick={() => setDeleting([detail])}><Trash2/>删除</Button>}</div>
            {detail.warnings.map((warning, index) => <div key={index} className="bag-alert error"><FileWarning/>{warning}</div>)}
            {detail.note && <p className="bag-note">{detail.note}</p>}
            <dl className="bag-summary"><div><dt>时长</dt><dd>{duration(detail.duration_sec)}</dd></div><div><dt>数据量</dt><dd>{size(detail.size_bytes)}</dd></div><div><dt>消息数</dt><dd>{count(detail.message_count)}</dd></div><div><dt>存储</dt><dd>{detail.storage}</dd></div></dl>
            <div className="bag-path"><code>{detail.path}</code><Button variant="ghost" size="icon" aria-label="复制 Bag 路径" onClick={() => action(() => navigator.clipboard.writeText(detail.path), '路径已复制')}><Copy/></Button></div>
            <h3>话题统计</h3><div className="bag-table-scroll"><table><thead><tr><th>话题</th><th>消息数</th><th title="消息数除以整个 Bag 时长，不代表传感器标称频率">包内均频</th></tr></thead><tbody>{detail.topics.map(topic => <tr key={topic.name}><td><code>{topic.name}</code><small>{topic.type}</small></td><td>{count(topic.count)}</td><td>{detail.duration_sec ? (topic.count / detail.duration_sec).toFixed(1) + ' Hz' : '—'}</td></tr>)}</tbody></table></div>
            <details className="bag-files"><summary>文件下载 · {detail.files.length}</summary><p>分片 Bag 请保留 metadata.yaml 和全部数据分片。</p>{detail.files.map(file => <div key={file.name}><span title={file.name}>{file.name}</span><small>{size(file.size_bytes)}</small>{!detail.active && <a href={`/api/bags/${detail.id}/file?name=${encodeURIComponent(file.name)}`} download aria-label={'下载 ' + file.name}><Download/></a>}</div>)}</details>
          </> : <div className="bag-empty"><Database/><strong>{focused ? '正在读取详情…' : '选择一个 Bag'}</strong></div>}
        </aside>
      </div>
      <details className="bag-roots"><summary>存储目录</summary>{data?.roots.map(root => <code key={root}>{root}</code>)}</details>
    </div>
    <Dialog open={!!editing} onOpenChange={open => { if (!open && !busy) setEditing(null) }}><DialogContent className="bag-edit-dialog"><DialogHeader><DialogTitle>名称与备注</DialogTitle><DialogDescription>不重命名原始文件。</DialogDescription></DialogHeader>
      <label>名称<Input aria-label="Bag 名称" value={editing?.name || ''} maxLength={100} onChange={e => setEditing(current => ({ ...current, name: e.target.value }))}/></label><label>备注<Textarea aria-label="Bag 备注" value={editing?.note || ''} maxLength={500} onChange={e => setEditing(current => ({ ...current, note: e.target.value }))}/></label>
      {error && <p role="alert">{error}</p>}<Button disabled={busy || !editing?.name.trim()} onClick={() => action(async () => { await patch(editing.id, { name: editing.name, note: editing.note }); setEditing(null) }, '已保存')}>保存</Button>
    </DialogContent></Dialog>
    <AlertDialog open={!!deleting} onOpenChange={open => { if (!open && !busy) setDeleting(null) }}><AlertDialogContent className="bag-delete-dialog"><AlertDialogHeader><AlertDialogTitle>永久删除 {deleting?.length || 0} 个 Bag？</AlertDialogTitle><AlertDialogDescription>将清除目录内全部数据分片、索引和附带文件，无法从 Web 恢复。</AlertDialogDescription></AlertDialogHeader><div className="bag-delete-list">{deleting?.map(item => <div key={item.id}><strong>{item.name}</strong><code>{item.path}</code></div>)}</div><p>将释放 {size(deleting?.reduce((sum, i) => sum + i.size_bytes, 0))}</p>{error && <p role="alert">{error}</p>}<AlertDialogFooter><AlertDialogCancel disabled={busy}>取消</AlertDialogCancel><Button variant="destructive" disabled={busy || !deleting?.length} onClick={removeSelected}>{busy ? '正在删除…' : '确认永久删除'}</Button></AlertDialogFooter></AlertDialogContent></AlertDialog>
  </div>
}
