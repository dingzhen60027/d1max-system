// Read-only formatting for legacy pipelines and standalone result manifests.
// These labels do not register modules with the legacy processing runner.
export const MODULE_LABELS = {
  crop_z: 'Z 高度裁剪',
  voxel_downsample: '体素降采样',
  statistical_outlier: '统计离群点滤波',
  radius_outlier: '半径离群点滤波',
  structure_support: '局部面 / 线结构保护',
}

export function moduleParameterText(module) {
  if (!module.enabled) return '关闭'
  const value = module.parameters || {}
  if (module.type === 'crop_z') return `${value.min_z}～${value.max_z} m`
  if (module.type === 'voxel_downsample') return `${value.voxel_size} m`
  if (module.type === 'statistical_outlier') return `neighbors=${value.neighbors}, σ=${value.std_ratio}`
  if (module.type === 'radius_outlier') {
    if ('radius_m' in value || 'minimum_other_neighbors' in value) {
      return `${value.radius_m ?? '—'} m / 至少 ${value.minimum_other_neighbors ?? '—'} 个其他邻点（不含自身）`
    }
    return `${value.radius ?? '—'} m / ${value.min_points ?? '—'} 点`
  }
  if (module.type === 'structure_support') {
    // Keep the real standalone schema names; do not reinterpret these as
    // statistical/radius-filter parameters or imply semantic object removal.
    return Object.entries(value).map(([name, parameter]) => `${name}=${parameter}`).join(' · ')
  }
  return JSON.stringify(value)
}
