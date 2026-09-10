import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js'
import { ArrowDownToLine, Box, RotateCcw, ScanLine, ZoomIn, ZoomOut } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { ButtonGroup } from '@/components/ui/button-group'
import { Spinner } from '@/components/ui/spinner'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'

const HEIGHT_COLORS = [
  '#233b91', '#176bc3', '#15a6dc', '#33cfae', '#8bd548',
  '#e4d33d', '#f29b30', '#e94c36', '#a91835',
].map((value) => new THREE.Color(value))

function percentile(sorted, ratio) {
  const position = (sorted.length - 1) * ratio
  const low = Math.floor(position)
  const high = Math.ceil(position)
  return low === high ? sorted[low] : sorted[low] + (sorted[high] - sorted[low]) * (position - low)
}

function colorize(geometry) {
  const positions = geometry.getAttribute('position')
  if (!positions?.count) return null
  const heights = new Float32Array(positions.count)
  for (let index = 0; index < positions.count; index += 1) heights[index] = positions.getZ(index)
  heights.sort()
  const low = percentile(heights, 0.02)
  const high = percentile(heights, 0.98)
  const span = high - low
  const colors = new Float32Array(positions.count * 3)
  const sampled = new THREE.Color()
  for (let index = 0; index < positions.count; index += 1) {
    const ratio = span > 1e-6 ? THREE.MathUtils.clamp((positions.getZ(index) - low) / span, 0, 1) : 0.5
    const scaled = ratio * (HEIGHT_COLORS.length - 1)
    const lower = Math.min(Math.floor(scaled), HEIGHT_COLORS.length - 2)
    sampled.lerpColors(HEIGHT_COLORS[lower], HEIGHT_COLORS[lower + 1], scaled - lower)
    colors[index * 3] = sampled.r
    colors[index * 3 + 1] = sampled.g
    colors[index * 3 + 2] = sampled.b
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
  return { low, high, actualMin: heights[0], actualMax: heights.at(-1) }
}

function pointTexture() {
  const canvas = document.createElement('canvas')
  canvas.width = 32
  canvas.height = 32
  const context = canvas.getContext('2d')
  context.fillStyle = '#fff'
  context.beginPath()
  context.arc(16, 16, 14, 0, Math.PI * 2)
  context.fill()
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  return texture
}

function CloudTool({ label, onClick, children }) {
  return <Tooltip><TooltipTrigger render={<Button variant="ghost" size="icon-sm" onClick={onClick} />}>{children}<span className="sr-only">{label}</span></TooltipTrigger><TooltipContent>{label}</TooltipContent></Tooltip>
}

export default function PointCloudView({ url, compact = false }) {
  const hostRef = useRef(null)
  const fitRef = useRef(() => {})
  const viewRef = useRef(() => {})
  const zoomRef = useRef(() => {})
  const [status, setStatus] = useState('loading')
  const [points, setPoints] = useState(0)
  const [range, setRange] = useState(null)

  useEffect(() => {
    const host = hostRef.current
    if (!host || !url) return undefined
    setStatus('loading')
    setPoints(0)
    setRange(null)
    const scene = new THREE.Scene()
    scene.background = new THREE.Color('#0d141f')
    const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 5000)
    camera.up.set(0, 0, 1)
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.outputColorSpace = THREE.SRGBColorSpace
    host.appendChild(renderer.domElement)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.screenSpacePanning = true
    const texture = pointTexture()
    let cloud = null
    let grid = null
    let frame = 0
    let disposed = false

    let currentPreset = 'iso'
    const setCameraView = (preset = currentPreset) => {
      if (!cloud) return
      const box = new THREE.Box3().setFromObject(cloud)
      const sphere = box.getBoundingSphere(new THREE.Sphere())
      const radius = Math.max(sphere.radius, 1)
      const verticalFov = THREE.MathUtils.degToRad(camera.fov)
      const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * Math.max(camera.aspect, 0.1))
      const presets = {
        iso: {
          direction: new THREE.Vector3(0.9, -1.25, 0.82),
          up: new THREE.Vector3(0, 0, 1),
        },
        top: {
          direction: new THREE.Vector3(0, -0.001, 1),
          up: new THREE.Vector3(0, 1, 0),
        },
        front: {
          direction: new THREE.Vector3(0, -1, 0.08),
          up: new THREE.Vector3(0, 0, 1),
        },
      }
      const selectedPreset = presets[preset] || presets.iso
      currentPreset = presets[preset] ? preset : 'iso'
      const direction = selectedPreset.direction.normalize()
      const forward = direction.clone().negate()
      const right = forward.clone().cross(selectedPreset.up).normalize()
      const trueUp = right.clone().cross(forward).normalize()
      const corners = [
        [box.min.x, box.min.y, box.min.z], [box.min.x, box.min.y, box.max.z],
        [box.min.x, box.max.y, box.min.z], [box.min.x, box.max.y, box.max.z],
        [box.max.x, box.min.y, box.min.z], [box.max.x, box.min.y, box.max.z],
        [box.max.x, box.max.y, box.min.z], [box.max.x, box.max.y, box.max.z],
      ]
      const tanVertical = Math.tan(verticalFov / 2)
      const tanHorizontal = Math.tan(horizontalFov / 2)
      let distance = radius
      for (const coordinates of corners) {
        const relative = new THREE.Vector3(...coordinates).sub(sphere.center)
        const nearOffset = relative.dot(direction)
        distance = Math.max(
          distance,
          Math.abs(relative.dot(right)) / tanHorizontal + nearOffset,
          Math.abs(relative.dot(trueUp)) / tanVertical + nearOffset,
        )
      }
      distance *= 1.13
      camera.up.copy(selectedPreset.up)
      camera.position.copy(sphere.center).add(direction.multiplyScalar(distance))
      camera.near = Math.max(radius / 1000, 0.01)
      camera.far = radius * 30
      camera.updateProjectionMatrix()
      controls.target.copy(sphere.center)
      controls.update()
    }
    fitRef.current = () => setCameraView(currentPreset)
    viewRef.current = (preset) => setCameraView(preset)
    zoomRef.current = (factor) => {
      const offset = camera.position.clone().sub(controls.target).multiplyScalar(factor)
      camera.position.copy(controls.target).add(offset)
      controls.update()
    }

    new PLYLoader().load(url, (geometry) => {
      if (disposed) {
        geometry.dispose()
        return
      }
      const loadedPositions = geometry.getAttribute('position')
      if (loadedPositions && !(loadedPositions.array instanceof Float32Array)) {
        geometry.setAttribute(
          'position',
          new THREE.Float32BufferAttribute(loadedPositions.array, loadedPositions.itemSize),
        )
      }
      geometry.computeBoundingBox()
      geometry.computeBoundingSphere()
      const heightRange = colorize(geometry)
      const bounds = geometry.boundingBox
      const size = bounds.getSize(new THREE.Vector3())
      const center = bounds.getCenter(new THREE.Vector3())
      const gridSize = Math.max(Math.ceil(Math.max(size.x, size.y) / 10) * 10, 10)
      grid = new THREE.GridHelper(gridSize, THREE.MathUtils.clamp(Math.round(gridSize / 2), 10, 80), '#53647c', '#273247')
      grid.rotation.x = Math.PI / 2
      grid.position.set(center.x, center.y, bounds.min.z - Math.max(size.z * 0.01, 0.02))
      grid.material.transparent = true
      grid.material.opacity = 0.56
      scene.add(grid)
      const radius = Math.max(geometry.boundingSphere?.radius || 10, 1)
      scene.fog = new THREE.Fog('#0d141f', radius * 2.2, radius * 8)
      const material = new THREE.PointsMaterial({
        size: THREE.MathUtils.clamp(radius / 280, 0.035, 0.28),
        sizeAttenuation: true,
        vertexColors: true,
        map: texture,
        alphaTest: 0.35,
        transparent: true,
        opacity: 0.96,
        depthWrite: false,
      })
      cloud = new THREE.Points(geometry, material)
      scene.add(cloud)
      setPoints(geometry.getAttribute('position')?.count || 0)
      setRange(heightRange)
      setStatus('ready')
      setCameraView('iso')
    }, undefined, () => !disposed && setStatus('error'))

    let lastWidth = 0
    let lastHeight = 0
    const resize = () => {
      const width = Math.max(host.clientWidth, 1)
      const height = Math.max(host.clientHeight, 1)
      const dimensionsChanged = Math.abs(width - lastWidth) > 2 || Math.abs(height - lastHeight) > 2
      lastWidth = width
      lastHeight = height
      renderer.setSize(width, height, false)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      if (cloud && dimensionsChanged) setCameraView(currentPreset)
    }
    const observer = new ResizeObserver(resize)
    observer.observe(host)
    resize()
    const animate = () => {
      controls.update()
      renderer.render(scene, camera)
      frame = requestAnimationFrame(animate)
    }
    animate()
    return () => {
      disposed = true
      cancelAnimationFrame(frame)
      observer.disconnect()
      controls.dispose()
      cloud?.geometry.dispose()
      cloud?.material.dispose()
      grid?.geometry.dispose()
      grid?.material.dispose()
      texture.dispose()
      renderer.dispose()
      renderer.domElement.remove()
    }
  }, [url])

  return (
    <div className={`cloud-view ${compact ? 'compact' : ''}`}>
      <div className="cloud-host" ref={hostRef} />
      {status === 'loading' && <div className="cloud-state"><Spinner /><strong>正在生成并载入 3D 预览</strong><span>首次打开 PCD 需要几秒钟</span></div>}
      {status === 'error' && <div className="cloud-state error"><Box size={28} /><strong>点云预览失败</strong><span>请查看后端日志或检查文件格式</span></div>}
      {status === 'ready' && <>
        <ButtonGroup className="cloud-tools">
          <CloudTool label="适应窗口" onClick={() => fitRef.current()}><RotateCcw size={16} /></CloudTool>
          <CloudTool label="等轴视角" onClick={() => viewRef.current('iso')}><Box size={16} /></CloudTool>
          <CloudTool label="俯视" onClick={() => viewRef.current('top')}><ArrowDownToLine size={16} /></CloudTool>
          <CloudTool label="前视" onClick={() => viewRef.current('front')}><ScanLine size={16} /></CloudTool>
          <CloudTool label="放大" onClick={() => zoomRef.current(0.82)}><ZoomIn size={16} /></CloudTool>
          <CloudTool label="缩小" onClick={() => zoomRef.current(1.22)}><ZoomOut size={16} /></CloudTool>
        </ButtonGroup>
        <div className="cloud-metrics"><span>{points.toLocaleString()} 预览点</span>{range && <span>Z {range.actualMin.toFixed(2)} — {range.actualMax.toFixed(2)} m</span>}</div>
        <div className="height-scale"><span>{range?.high.toFixed(1) ?? '--'} m</span><i /><span>{range?.low.toFixed(1) ?? '--'} m</span></div>
      </>}
    </div>
  )
}
