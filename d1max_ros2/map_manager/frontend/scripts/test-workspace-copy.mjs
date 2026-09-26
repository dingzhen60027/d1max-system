// Copy-only checks: no browser, backend, ROS, robot or file mutations.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { parse } from '@babel/parser'

const read = name => readFileSync(new URL('../src/' + name, import.meta.url), 'utf8')
const grid = read('Workspace2D.jsx')
const cloud = read('Workspace3D.jsx')
const bags = read('WorkspaceBags.jsx')

test('all three workspaces remain valid JSX modules', () => {
  for (const source of [grid, cloud, bags]) {
    const ast = parse(source, { sourceType: 'module', plugins: ['jsx'] })
    assert(ast.program.body.some(node => node.type === 'ExportDefaultDeclaration'))
  }
})

test('2D copy keeps coordinate and unobserved-space warnings', () => {
  for (const copy of [
    'Z 使用 PCD 坐标，非离地高度。', '栅格分辨率（m/格）',
    '空白将标为自由空间，通行性需现场核对。', '通行性待核对',
    '仅切换地图，不启动定位或导航。',
  ]) assert(grid.includes(copy), copy)
  assert(grid.includes('{selected.warning}'))
  assert(grid.includes('{job.error}'))
})

test('irreversible deletion remains explicit in every workspace', () => {
  assert(grid.includes('源 PCD 和其他版本不会删除，操作无法恢复。'))
  assert(cloud.includes('无法恢复；处理结果的 pipeline.yaml 和 manifest.json 也会一并删除。'))
  assert(cloud.includes('将清空整个归档区，包括当前搜索结果之外的条目。'))
  assert(bags.includes('将清除目录内全部数据分片、索引和附带文件，无法从 Web 恢复。'))
  for (const source of [grid, cloud, bags]) assert(source.includes('永久删除'))
})

test('algorithm names and quality details are retained as real metadata', () => {
  for (const label of ['SC-PGO 优化', 'Faster-LIO 原图', 'FAST-LIO2',
    'MOLA-LIO 原图', 'MOLA 回环处理', 'LIO-SAM · 单前雷达', 'LIO-SAM · 前后双雷达']) {
    assert(cloud.includes(label), label)
  }
  assert(cloud.includes('{item.name}'))
  assert(cloud.includes('{selected.name}'))
  assert(cloud.includes('{selected.family}'))
  assert(cloud.includes('{issue}'))
  assert(cloud.includes('{selectedAlgorithm.unavailable_reason}'))
})

test('bag data caveats survive copy cleanup', () => {
  for (const copy of [
    '服务连接中断，录制状态未知。请恢复连接后核对。',
    '接收频率仅供监测，入包数量以保存后统计为准。',
    '消息数除以整个 Bag 时长，不代表传感器标称频率',
    '分片 Bag 请保留 metadata.yaml 和全部数据分片。',
  ]) assert(bags.includes(copy), copy)
  assert(bags.includes('{warning}'))
})

test('primary action labels stay short', () => {
  assert(grid.includes("busy?'任务运行中':'生成地图'"))
  assert(cloud.includes("processingJob.running ? '点云处理中' : '处理 PCD'"))
  assert(cloud.includes('<Modal title="处理点云"'))
  assert(!cloud.includes('配置参数并处理 PCD'))
  assert(!bags.includes('开始第一次录制'))
})
