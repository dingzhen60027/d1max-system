import { useEffect, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Textarea } from '@/components/ui/textarea'
import { Badge } from '@/components/ui/badge'

export default function MolaMappingOptions({ value, onChange, disabled }) {
  const [error, setError] = useState('')
  useEffect(() => {
    if (value.config_yaml) return
    let cancelled = false
    fetch('/api/mapping/profiles/mola_lio_lc').then(async response => {
      if (!response.ok) throw new Error('MOLA 配置加载失败')
      return response.json()
    }).then(result => { if (!cancelled) onChange(current => ({ ...current, config_yaml: result.yaml })) })
      .catch(e => { if (!cancelled) setError(e.message) })
    return () => { cancelled = true }
  }, [value.config_yaml, onChange])
  async function validate(download = false) {
    setError('')
    try {
      const response = await fetch('/api/mapping/profiles/mola_lio_lc/validate', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ yaml: value.config_yaml, ...(value.bag_path ? { bag_path: value.bag_path } : {}) }),
      })
      const result = await response.json()
      if (!response.ok) throw new Error(result.detail || '配置校验失败')
      onChange(current => ({ ...current, config_yaml: result.yaml }))
      if (download) {
        const url = URL.createObjectURL(new Blob([result.yaml], { type: 'application/yaml' }))
        const link = document.createElement('a'); link.href = url; link.download = 'mola_lio_lc.yaml'; link.click()
        setTimeout(() => URL.revokeObjectURL(url), 1000)
      }
    } catch (e) { setError(e.message) }
  }
  return <section className="mola-options">
    <div className="mola-module-tags"><Badge variant="secondary">前端 · GICP + IMU</Badge><Badge variant="secondary">后端 · 离线帧间回环</Badge></div>
    <label>rosbag 目录<Input aria-label="MOLA rosbag 目录" value={value.bag_path || ''} disabled={disabled} placeholder="包含 metadata.yaml 的完整目录" onChange={e => onChange(current => ({ ...current, bag_path: e.target.value }))}/></label>
    <details><summary>参数配置 · YAML</summary>
      <Textarea aria-label="MOLA 参数配置" className="mola-config-editor" rows={16} value={value.config_yaml || ''} disabled={disabled} onChange={e => onChange(current => ({ ...current, config_yaml: e.target.value }))}/>
      <div className="mola-config-actions"><Button variant="outline" disabled={disabled} onClick={() => validate()}>校验配置</Button><Button variant="outline" disabled={disabled} onClick={() => validate(true)}>下载配置</Button></div>
    </details>
    {error && <p role="alert">{error}</p>}
  </section>
}
