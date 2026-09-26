import assert from 'node:assert/strict'
import { mkdir, writeFile } from 'node:fs/promises'
import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'
const base = 'http://127.0.0.1:18766'
const overview = await fetch(base + '/api/bags/overview').then(r => r.json())
assert(overview.output_root.startsWith('/tmp/d1max-bag-browser-'))
const out = overview.output_root.replace(/\/bags$/, '/screenshots')
await mkdir(out, { recursive: true })
const report = { checks: [], errors: [], screenshots: [] }
const check = text => { report.checks.push(text); console.log('PASS ' + text) }
const browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox', '--disable-dev-shm-usage'] })
const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } })
page.on('pageerror', e => report.errors.push(e.message))
let simulated = { status: 'idle', busy: false }, starts = 0
await page.route('**/api/bags/**', async route => {
  const req = route.request(), url = new URL(req.url())
  if (url.pathname === '/api/bags/recording/start') {
    assert.equal(req.method(), 'POST'); starts++
    const body = req.postDataJSON()
    assert(body.groups.includes('core')); assert(!body.groups.includes('reference'))
    simulated = { status: 'preparing', busy: true, groups: body.groups, name: body.name, topics: [], logs: [] }
    await route.fulfill({ status: 202, json: simulated }); return
  }
  if (url.pathname === '/api/bags/recording/stop') {
    simulated = { status: 'cancelled', busy: false, topics: [], logs: [] }
    await route.fulfill({ status: 202, json: simulated }); return
  }
  if (url.pathname === '/api/bags/recording') { await route.fulfill({ json: simulated }); return }
  if (url.pathname === '/api/bags/overview') {
    const response = await route.fetch(), body = await response.json()
    body.runtime = simulated
    await route.fulfill({ json: body }); return
  }
  assert(!/\/api\/(runtime|localization)/.test(url.pathname), 'No robot mutation allowed')
  await route.continue()
})
const click = name => page.getByRole('button', { name, exact: true }).click()
try {
  await page.goto(base + '/#/')
  await page.locator('.home-recording a').click()
  await page.getByRole('heading', { name: '新建录制', exact: true }).waitFor()
  await page.locator('.bag-row').first().waitFor()
  assert.equal(await page.locator('.bag-row').count(), 3)
  assert(await page.getByRole('switch', { name: /双雷达 · 三路 IMU · TF/ }).isDisabled())
  assert.equal(await page.getByRole('switch', { name: '原厂定位参考', exact: true }).getAttribute('aria-checked'), 'false')
  check('Home/sidebar entry, three existing bags, mandatory sensors, reference output opt-in')

  await click('开始录制')
  await page.getByRole('button', { name: '取消等待', exact: true }).waitFor()
  assert.equal(starts, 1)
  assert(await page.getByLabel('录制名称', { exact: true }).isDisabled())
  await click('取消等待')
  await page.getByRole('button', { name: '开始录制', exact: true }).waitFor()
  check('Start shows preparing, prevents repeat, cancellation restores controls (mock transport only)')

  await page.getByRole('button').filter({ hasText: '一楼大厅 · 测试采集' }).click()
  await click('名称与备注')
  await page.getByLabel('Bag 名称', { exact: true }).fill('QA 已核对 · 一楼')
  await page.getByLabel('Bag 备注', { exact: true }).fill('保留完整原始传感器')
  await click('保存信息')
  await page.getByRole('heading', { name: 'QA 已核对 · 一楼', exact: true }).waitFor()
  await page.locator('.bag-files summary').click()
  const [download] = await Promise.all([page.waitForEvent('download'), page.getByRole('link', { name: '下载 metadata.yaml', exact: true }).click()])
  assert.equal(download.suggestedFilename(), 'metadata.yaml')
  check('Real API rename/note, topic metadata, per-file download')
  await click('归档')
  await page.getByRole('link', { name: 'Bag 归档', exact: true }).click()
  await page.getByRole('heading', { name: 'QA 已核对 · 一楼', exact: true }).waitFor()
  assert.equal(await page.locator('.bag-row').count(), 1)
  await click('恢复')
  await page.locator('.bag-row').waitFor({ state: 'hidden' })
  check('Archive disappears from active list; restore removes archive count immediately')

  await page.getByRole('link', { name: '录制与文件', exact: true }).click()
  await page.locator('.bag-row').first().waitFor()
  for (const width of [1920, 1440, 1024, 768, 390]) {
    await page.setViewportSize({ width, height: 1100 }); await page.waitForTimeout(180)
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'page overflow at ' + width)
    const bounds = await page.locator('.bags-scroll').evaluate(el => ({ scroll: el.scrollWidth, width: el.clientWidth }))
    assert(bounds.scroll <= bounds.width + 1, 'workspace overflow at ' + width)
    const shot = `${width}-bags.png`; await page.screenshot({ path: out + '/' + shot, fullPage: true }); report.screenshots.push(shot)
  }
  check('Five viewport sizes: no page/workspace overflow')

  await page.setViewportSize({ width: 1440, height: 1100 })
  await page.getByRole('checkbox', { name: '全选当前列表', exact: true }).click()
  await click('归档所选')
  await page.getByRole('link', { name: 'Bag 归档', exact: true }).click()
  await page.getByRole('checkbox', { name: '全选当前列表', exact: true }).waitFor()
  await page.waitForTimeout(300)
  assert.equal(await page.locator('.bag-row').count(), 3)
  await click('删除列表全部')
  await page.getByRole('alertdialog').waitFor()
  assert((await page.getByRole('alertdialog').innerText()).includes('永久删除 3 个 Bag'))
  await click('取消')
  assert.equal(await page.locator('.bag-row').count(), 3)
  await page.getByRole('checkbox', { name: '选择 QA 已核对 · 一楼', exact: true }).click()
  await click('删除所选'); await click('确认永久删除')
  await page.getByRole('alertdialog').waitFor({ state: 'hidden' })
  await page.waitForTimeout(300)
  assert.equal(await page.locator('.bag-row').count(), 2)
  await click('删除列表全部'); await click('确认永久删除')
  await page.locator('.bag-row').first().waitFor({ state: 'hidden' })
  assert.equal((await fetch(base + '/api/bags/overview').then(r => r.json())).items.length, 0)
  assert.equal(await page.locator('[role=alert]').count(), 0, 'No stale detail error after deleting selected item')
  check('Deletion requires confirmation; cancel retains data; selected/delete-all remove only isolated fixtures')
  assert.deepEqual(report.errors, [])
} catch (error) {
  report.failure = error.stack; await page.screenshot({ path: out + '/failure.png' }); throw error
} finally {
  await writeFile(out + '/report.json', JSON.stringify(report, null, 2)); await browser.close(); console.log('Artifacts: ' + out)
}
