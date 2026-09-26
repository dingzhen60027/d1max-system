import assert from 'node:assert/strict'
import test from 'node:test'
import { MODULE_LABELS, moduleParameterText } from '../src/processing-display.js'

test('legacy radius parameters retain their original display', () => {
  assert.equal(moduleParameterText({ type: 'radius_outlier', enabled: true,
    parameters: { radius: 0.16, min_points: 6 } }), '0.16 m / 6 点')
})

test('standalone radius states other-neighbor semantics without undefined values', () => {
  assert.equal(moduleParameterText({ type: 'radius_outlier', enabled: true,
    parameters: { radius_m: 0.15, minimum_other_neighbors: 3 } }),
  '0.15 m / 至少 3 个其他邻点（不含自身）')
})

test('partial standalone fields are never confused with legacy parameter names', () => {
  const value = moduleParameterText({ type: 'radius_outlier', enabled: true,
    parameters: { radius_m: 0.15, min_points: 6 } })
  assert.equal(value, '0.15 m / 至少 — 个其他邻点（不含自身）')
  assert.doesNotMatch(value, /undefined/)
})

test('structure protection shows actual config names and does not mutate config', () => {
  const module = { type: 'structure_support', enabled: true,
    parameters: { neighbors: 32, support_radius_m: 0.45, query_to_plane_m: 0.025,
      minimum_linearity_fraction: 0.98, query_to_line_m: 0.02 } }
  const before = structuredClone(module)
  assert.equal(MODULE_LABELS.structure_support, '局部面 / 线结构保护')
  assert.equal(moduleParameterText(module),
    'neighbors=32 · support_radius_m=0.45 · query_to_plane_m=0.025 · minimum_linearity_fraction=0.98 · query_to_line_m=0.02')
  assert.deepEqual(module, before)
})

test('disabled and unknown result-only modules remain read-only descriptions', () => {
  assert.equal(moduleParameterText({type:'structure_support',enabled:false,parameters:{neighbors:32}}),'关闭')
  assert.equal(moduleParameterText({type:'custom',enabled:true,parameters:{threshold:0.5}}),'{"threshold":0.5}')
})
