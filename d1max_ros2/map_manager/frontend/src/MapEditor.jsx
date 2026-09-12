import { useCallback, useEffect, useMemo, useReducer, useRef, useState } from 'react'
import {
  Check,
  Eraser,
  FileWarning,
  LoaderCircle,
  Minus,
  Pencil,
  Plus,
  Redo2,
  RotateCcw,
  Save,
  Slash,
  Undo2,
  X,
} from 'lucide-react'

import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Textarea } from '@/components/ui/textarea'
import { Dialog, DialogContent, DialogTitle, DialogDescription } from '@/components/ui/dialog'
import { AlertDialog, AlertDialogContent, AlertDialogHeader, AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction } from '@/components/ui/alert-dialog'

// Canvas/stroke editing ported from go2_nav; saves only derived D1 Max versions.
async function api(path, options) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const data = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(data.detail || `请求失败 (${response.status})`)
  return data
}

const COLORS = {
  occupied: '#000000',
  free: '#fefefe',
  unknown: '#cdcdcd',
}

const MODE_LABELS = {
  occupied: '画墙',
  free: '擦除杂点',
  unknown: '设为未知',
}

function editHistory(state, action) {
  if (action.type === 'reset') return { operations: [], redoStack: [] }
  if (action.type === 'commit') return { operations: [...state.operations, action.operation], redoStack: [] }
  if (action.type === 'undo' && state.operations.length) return { operations: state.operations.slice(0, -1), redoStack: [...state.redoStack, state.operations.at(-1)] }
  if (action.type === 'redo' && state.redoStack.length) return { operations: [...state.operations, state.redoStack.at(-1)], redoStack: state.redoStack.slice(0, -1) }
  return state
}

function clamp(value, low, high) {
  return Math.min(high, Math.max(low, value))
}

function drawOperation(context, operation) {
  if (!operation?.points?.length) return
  const points = operation.points
  context.save()
  context.strokeStyle = COLORS[operation.mode]
  context.fillStyle = COLORS[operation.mode]
  context.lineWidth = operation.size
  context.lineCap = 'round'
  context.lineJoin = 'round'
  if (points.length === 1) {
    context.beginPath()
    context.arc(points[0].x, points[0].y, Math.max(0.5, operation.size / 2), 0, Math.PI * 2)
    context.fill()
  } else {
    context.beginPath()
    context.moveTo(points[0].x, points[0].y)
    for (let index = 1; index < points.length; index += 1) {
      context.lineTo(points[index].x, points[index].y)
    }
    context.stroke()
  }
  context.restore()
}

export default function MapEditor({ version, onClose, onDone, notify }) {
  const canvasRef = useRef(null)
  const viewportRef = useRef(null)
  const sourceImageRef = useRef(null)
  const operationsRef = useRef([])
  const activeStrokeRef = useRef(null)
  const saveLock = useRef(false)
  const [discardOpen, setDiscardOpen] = useState(false)
  const [ready, setReady] = useState(false)
  const [loadError, setLoadError] = useState('')
  const [{ operations, redoStack }, dispatchHistory] = useReducer(editHistory, { operations: [], redoStack: [] })
  const [mode, setMode] = useState('occupied')
  const [shape, setShape] = useState('brush')
  const [brushSize, setBrushSize] = useState(3)
  const [zoom, setZoom] = useState(1)
  const [imageSize, setImageSize] = useState({ width: 0, height: 0 })
  const [cursor, setCursor] = useState(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [name, setName] = useState(`${version.name}（2D修订）`)
  const [note, setNote] = useState('手工去除2D杂点并补齐墙体')
  const resolution = Number(version.parameters?.resolution) || 0.05

  const renderCanvas = useCallback((draft = null) => {
    const canvas = canvasRef.current
    const image = sourceImageRef.current
    if (!canvas || !image) return
    const context = canvas.getContext('2d', { alpha: false })
    context.imageSmoothingEnabled = false
    context.clearRect(0, 0, canvas.width, canvas.height)
    context.drawImage(image, 0, 0, canvas.width, canvas.height)
    operationsRef.current.forEach((operation) => drawOperation(context, operation))
    if (draft) drawOperation(context, draft)
  }, [])

  const fitToView = useCallback(() => {
    const viewport = viewportRef.current
    if (!viewport || !imageSize.width || !imageSize.height) return
    const horizontal = Math.max(120, viewport.clientWidth - 48) / imageSize.width
    const vertical = Math.max(120, viewport.clientHeight - 48) / imageSize.height
    setZoom(clamp(Math.min(horizontal, vertical), 0.1, 4))
  }, [imageSize])

  useEffect(() => {
    let cancelled = false
    setReady(false)
    setLoadError('')
    dispatchHistory({ type: 'reset' })
    operationsRef.current = []
    const image = new Image()
    image.onload = () => {
      if (cancelled) return
      const canvas = canvasRef.current
      canvas.width = image.naturalWidth
      canvas.height = image.naturalHeight
      sourceImageRef.current = image
      setImageSize({ width: image.naturalWidth, height: image.naturalHeight })
      setReady(true)
      renderCanvas()
    }
    image.onerror = () => !cancelled && setLoadError('2D 地图加载失败，请确认 PGM 文件完整')
    image.src = `${version.map_preview_url}?editor=${Date.now()}`
    return () => { cancelled = true }
  }, [renderCanvas, version.id, version.map_preview_url])

  useEffect(() => {
    operationsRef.current = operations
    if (ready) renderCanvas()
  }, [operations, ready, renderCanvas])

  useEffect(() => {
    if (!ready) return undefined
    const frame = window.requestAnimationFrame(fitToView)
    return () => window.cancelAnimationFrame(frame)
  }, [fitToView, ready])

  const pointFromEvent = useCallback((event) => {
    const canvas = canvasRef.current
    const bounds = canvas.getBoundingClientRect()
    return {
      x: clamp((event.clientX - bounds.left) * canvas.width / bounds.width, 0, canvas.width - 1),
      y: clamp((event.clientY - bounds.top) * canvas.height / bounds.height, 0, canvas.height - 1),
    }
  }, [])

  const commitStroke = useCallback((operation) => {
    if (!operation?.points?.length) return
    dispatchHistory({ type: 'commit', operation })
  }, [])

  const handlePointerDown = (event) => {
    if (!ready || busy || event.button !== 0) return
    event.preventDefault()
    event.currentTarget.setPointerCapture(event.pointerId)
    const point = pointFromEvent(event)
    const operation = {
      mode,
      shape,
      size: brushSize,
      points: shape === 'line' ? [point, point] : [point],
    }
    activeStrokeRef.current = operation
    if (shape === 'line') renderCanvas(operation)
    else drawOperation(canvasRef.current.getContext('2d'), operation)
  }

  const handlePointerMove = (event) => {
    if (ready) setCursor(pointFromEvent(event))
    const operation = activeStrokeRef.current
    if (!operation) return
    event.preventDefault()
    const point = pointFromEvent(event)
    if (operation.shape === 'line') {
      operation.points[1] = point
      renderCanvas(operation)
      return
    }
    const previous = operation.points.at(-1)
    if (Math.hypot(point.x - previous.x, point.y - previous.y) < 0.5) return
    operation.points.push(point)
    drawOperation(canvasRef.current.getContext('2d'), {
      ...operation,
      points: [previous, point],
    })
  }

  const handlePointerUp = (event) => {
    const operation = activeStrokeRef.current
    if (!operation) return
    event.preventDefault()
    activeStrokeRef.current = null
    commitStroke({
      ...operation,
      points: operation.points.map((point) => ({ x: point.x, y: point.y })),
    })
  }

  const cancelActiveStroke = () => {
    if (!activeStrokeRef.current) return
    activeStrokeRef.current = null
    renderCanvas()
  }

  const undo = useCallback(() => {
    dispatchHistory({ type: 'undo' })
  }, [])

  const redo = useCallback(() => {
    dispatchHistory({ type: 'redo' })
  }, [])

  const reset = () => {
    dispatchHistory({ type: 'reset' })
    activeStrokeRef.current = null
  }

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (busy || /INPUT|TEXTAREA/.test(event.target?.tagName) || event.target?.isContentEditable) return
      if (!(event.ctrlKey || event.metaKey)) return
      if (event.key.toLowerCase() === 'z') {
        event.preventDefault()
        if (event.shiftKey) redo()
        else undo()
      } else if (event.key.toLowerCase() === 'y') {
        event.preventDefault()
        redo()
      }
    }
    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [redo, undo, busy])

  const closeEditor = () => {
    if (busy) return
    if (operations.length) { setDiscardOpen(true); return }
    onClose()
  }

  const save = async (event) => {
    event.preventDefault()
    if (saveLock.current || !operations.length || !name.trim()) return
    saveLock.current = true
    setBusy(true)
    setError('')
    try {
      const result = await api(`/api/2d/versions/${version.id}/edit-2d`, {
        method: 'POST',
        body: JSON.stringify({ name: name.trim(), note, operations }),
      })
      notify(`2D 修订版已保存：修改 ${result.edit_summary.changed_pixels} 个像素`)
      await onDone(result.version_id)
    } catch (requestError) {
      setError(requestError.message)
      notify(requestError.message, 'error')
    } finally {
      saveLock.current = false
      setBusy(false)
    }
  }

  const counts = useMemo(() => operations.reduce((summary, operation) => ({
    ...summary,
    [operation.mode]: summary[operation.mode] + 1,
  }), { occupied: 0, free: 0, unknown: 0 }), [operations])

  return (
    <Dialog open onOpenChange={(open) => !open && closeEditor()}>
      <DialogContent className="map-editor" showCloseButton={false}>
        <header className="map-editor-head">
          <div><DialogTitle>修整 2D 导航地图</DialogTitle><DialogDescription>{version.name}</DialogDescription></div>
          <Button variant="outline" className="icon-button" title="关闭编辑器" onClick={closeEditor} disabled={busy}><X size={19} /></Button>
        </header>

        <div className="map-editor-body">
          <aside className="map-editor-tools">
            <section>
              <strong>修改内容</strong>
              <div className="map-editor-tool-grid">
                <Button variant="outline" className={mode === 'occupied' ? 'active wall' : ''} onClick={() => setMode('occupied')}><Pencil size={17} /><span>画墙</span></Button>
                <Button variant="outline" className={mode === 'free' ? 'active free' : ''} onClick={() => setMode('free')}><Eraser size={17} /><span>擦除杂点</span><small>标记为可通行</small></Button>
                <Button variant="outline" className={mode === 'unknown' ? 'active unknown' : ''} onClick={() => setMode('unknown')}><FileWarning size={17} /><span>未知区域</span></Button>
              </div>
            </section>

            <section>
              <strong>落笔方式</strong>
              <div className="map-editor-shapes">
                <Button variant="outline" className={shape === 'brush' ? 'active' : ''} onClick={() => setShape('brush')}><Pencil size={15} />自由笔刷</Button>
                <Button variant="outline" className={shape === 'line' ? 'active' : ''} onClick={() => setShape('line')}><Slash size={15} />直线补墙</Button>
              </div>
            </section>

            <section className="map-editor-brush">
              <div><strong>笔刷宽度</strong><b>{brushSize} px · {(brushSize * resolution).toFixed(2)} m</b></div>
              <input type="range" min="1" max="40" step="1" value={brushSize} onChange={(event) => setBrushSize(Number(event.target.value))} />
            </section>

            <section className="map-editor-history">
              <strong>编辑历史</strong>
              <div><Button variant="outline" onClick={undo} disabled={!operations.length || busy}><Undo2 size={15} />撤销</Button><Button variant="outline" onClick={redo} disabled={!redoStack.length || busy}><Redo2 size={15} />重做</Button></div>
              <Button variant="outline" className="reset" onClick={reset} disabled={!operations.length || busy}><RotateCcw size={15} />恢复到原始2D地图</Button>
            </section>

            <section className="map-editor-legend">
              <strong>地图颜色</strong>
              <span><i className="occupied" />黑色：墙体 / 障碍</span>
              <span><i className="free" />白色：自由空间</span>
              <span><i className="unknown" />灰色：未知空间</span>
            </section>
          </aside>

          <main className="map-editor-stage">
            <div className="map-editor-stage-bar">
              <div><b>{MODE_LABELS[mode]}</b><span>{shape === 'line' ? '拖动起点到终点' : '按住鼠标拖动'}</span></div>
              <div className="map-editor-zoom"><Button variant="outline" onClick={() => setZoom((value) => clamp(value / 1.25, 0.1, 6))} title="缩小"><Minus size={15} /></Button><b>{Math.round(zoom * 100)}%</b><Button variant="outline" onClick={() => setZoom((value) => clamp(value * 1.25, 0.1, 6))} title="放大"><Plus size={15} /></Button><Button variant="outline" onClick={fitToView}>适应窗口</Button></div>
            </div>
            <div className="map-editor-viewport" ref={viewportRef}>
              {!ready && !loadError && <div className="map-editor-loading"><LoaderCircle className="spin" size={21} />正在加载原始 PGM</div>}
              {loadError && <div className="map-editor-loading error"><FileWarning size={21} />{loadError}</div>}
              <canvas
                ref={canvasRef}
                className={ready ? 'ready' : ''}
                style={{ width: imageSize.width * zoom, height: imageSize.height * zoom }}
                onPointerDown={handlePointerDown}
                onPointerMove={handlePointerMove}
                onPointerUp={handlePointerUp}
                onPointerCancel={cancelActiveStroke}
                onPointerLeave={() => setCursor(null)}
              />
              {ready && <div className="map-editor-canvas-info"><span>{imageSize.width} × {imageSize.height} px</span><span>{resolution.toFixed(3)} m/格</span>{cursor && <span>x {Math.round(cursor.x)} · y {Math.round(cursor.y)}</span>}</div>}
            </div>
          </main>

          <form className="map-editor-save" onSubmit={save}>
            <div className="map-editor-safety"><Check size={17} /><span><strong>仅编辑栅格 · PCD 不变</strong></span></div>
            <label><span>新版本名称</span><Input maxLength="80" value={name} onChange={(event) => setName(event.target.value)} /></label>
            <label><span>修订说明</span><Textarea rows="4" maxLength="500" value={note} onChange={(event) => setNote(event.target.value)} /></label>
            <section className="map-editor-summary">
              <div><span>操作总数</span><strong>{operations.length}</strong></div>
              <p><span><i className="occupied" />补墙 {counts.occupied}</span><span><i className="free" />擦除 {counts.free}</span><span><i className="unknown" />未知 {counts.unknown}</span></p>
            </section>
            {error && <div className="map-editor-error"><FileWarning size={16} />{error}</div>}
            <div className="map-editor-save-actions"><Button variant="outline" type="button" className="button secondary" onClick={closeEditor} disabled={busy}>取消</Button><Button type="submit" className="button primary" disabled={busy || !ready || !operations.length || !name.trim()}>{busy ? <LoaderCircle className="spin" size={16} /> : <Save size={16} />}{busy ? '正在创建版本…' : '保存为新地图版本'}</Button></div>
          </form>
        </div>
      </DialogContent>
      <AlertDialog open={discardOpen} onOpenChange={setDiscardOpen}><AlertDialogContent><AlertDialogHeader><AlertDialogTitle>放弃尚未保存的笔画？</AlertDialogTitle><AlertDialogDescription>源地图不会改变，当前草稿将丢失。</AlertDialogDescription></AlertDialogHeader><AlertDialogFooter><AlertDialogCancel>继续编辑</AlertDialogCancel><AlertDialogAction variant="destructive" onClick={onClose}>放弃草稿</AlertDialogAction></AlertDialogFooter></AlertDialogContent></AlertDialog>
    </Dialog>
  )
}
