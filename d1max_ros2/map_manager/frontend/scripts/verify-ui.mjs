import { chromium } from '/home/dndx/go2_nav/tools/map_manager/frontend/node_modules/playwright-core/index.mjs'

const baseUrl = process.env.D1MAX_MAP_MANAGER_URL || 'http://127.0.0.1:8766'
const browser = await chromium.launch({
  executablePath: '/usr/bin/google-chrome',
  headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader'],
})

const result = { consoleErrors: [], api: {}, desktop: {}, processingModal: {}, cleanupDialog: {}, archive: {}, compare: {}, wide: {}, mobile: {} }
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } })
  page.on('console', (message) => {
    if (message.type() === 'error') result.consoleErrors.push(message.text())
  })
  page.on('pageerror', (error) => result.consoleErrors.push(error.message))
  await page.goto(baseUrl, { waitUntil: 'networkidle' })
  await page.locator('.app-shell').waitFor()
  await page.locator('.cloud-metrics').waitFor({ timeout: 30000 })
  const overviewResponse = await page.request.get(`${baseUrl}/api/overview`)
  const overviewData = await overviewResponse.json()
  const deleteGuardResponse = await page.request.post(`${baseUrl}/api/archive/delete`, {
    data: { item_ids: [], delete_all: false },
  })
  result.api = {
    overviewStatus: overviewResponse.status(),
    processedCount: overviewData.summary?.processed_count,
    archivedCount: overviewData.summary?.archived_count,
    archivedProcessedCount: overviewData.items?.filter((item) => item.archived && item.category === 'processed').length,
    emptyDeleteStatus: deleteGuardResponse.status(),
    emptyDeleteDetail: (await deleteGuardResponse.json()).detail,
  }
  await page.waitForTimeout(500)
  await page.screenshot({ path: '/tmp/d1max-map-workspace-desktop.png', fullPage: true })
  result.desktop = await page.evaluate(() => ({
    viewport: [innerWidth, innerHeight],
    scrollWidth: document.documentElement.scrollWidth,
    rows: document.querySelectorAll('.map-row').length,
    tabs: [...document.querySelectorAll('.section-tabs button')].map((node) => node.textContent),
    canvas: Boolean(document.querySelector('.cloud-host canvas')),
    has2dFeature: /2D\s*(栅格|地图|编辑)/.test(document.body.innerText),
    rmw: document.body.innerText.includes('rmw_zenoh_cpp'),
    latestWarning: document.body.innerText.includes('未接受回环约束'),
    shadcnCards: document.querySelectorAll('[data-slot="card"]').length,
    shadcnTabs: document.querySelectorAll('[data-slot="tabs"]').length,
    shadcnSelects: document.querySelectorAll('[data-slot="select-trigger"]').length,
    rowDecorationsContained: [...document.querySelectorAll('.map-row')].every((row) => {
      const rowBox = row.getBoundingClientRect()
      const flagsBox = row.querySelector('.row-flags')?.getBoundingClientRect()
      return !flagsBox || (flagsBox.top >= rowBox.top && flagsBox.bottom <= rowBox.bottom)
    }),
    sizing: {
      topbar: document.querySelector('.topbar')?.offsetHeight,
      runtime: document.querySelector('.runtime-bar')?.offsetHeight,
      sidebar: document.querySelector('.sidebar')?.offsetWidth,
      tabs: document.querySelector('.section-tabs')?.offsetWidth,
      search: document.querySelector('.search')?.offsetHeight,
      mapRow: document.querySelector('.map-row')?.offsetHeight,
      compactButton: document.querySelector('.button.compact')?.offsetHeight,
      cloudTool: document.querySelector('.cloud-tools button')?.offsetHeight,
    },
  }))

  await page.getByRole('button', { name: /配置参数并处理 PCD/ }).click()
  await page.getByRole('dialog').waitFor()
  await page.screenshot({ path: '/tmp/d1max-map-workspace-processing-modal.png', fullPage: true })
  result.processingModal = await page.evaluate(() => ({
    title: document.querySelector('[role="dialog"] h2')?.textContent,
    sourceLocked: document.querySelector('[role="dialog"]')?.innerText.includes('原始 PCD 只读'),
    structuralConfig: document.querySelector('[role="dialog"]')?.innerText.includes('结构保真（推荐）'),
    yamlDownload: Boolean(document.querySelector('[role="dialog"] a[download]')),
    parameterBlocks: document.querySelectorAll('.parameter-block').length,
    orderButtons: document.querySelectorAll('.module-order button').length,
    shadcnDialog: Boolean(document.querySelector('[data-slot="dialog-content"]')),
    shadcnSwitches: document.querySelectorAll('[data-slot="switch"]').length,
    shadcnInputs: document.querySelectorAll('[data-slot="input"]').length,
    sizing: {
      width: document.querySelector('[data-slot="dialog-content"]')?.offsetWidth,
      input: document.querySelector('[role="dialog"] [data-slot="input"]')?.offsetHeight,
      switch: document.querySelector('[role="dialog"] [data-slot="switch"]')?.offsetHeight,
      orderButton: document.querySelector('.module-order button')?.offsetHeight,
    },
    hasTwoDimensionalOutput: /2D\s*(栅格|地图构建|编辑器)/.test(document.querySelector('[role="dialog"]')?.innerText || ''),
  }))
  const firstSwitch = page.getByRole('dialog').locator('[data-slot="switch"]').first()
  const checkedBefore = await firstSwitch.getAttribute('aria-checked')
  await firstSwitch.click()
  const checkedAfter = await firstSwitch.getAttribute('aria-checked')
  await firstSwitch.click()
  result.processingModal.switchToggles = checkedBefore !== checkedAfter

  await page.getByRole('dialog').locator('[data-slot="select-trigger"]').click()
  const configOptions = page.getByRole('option')
  await configOptions.first().waitFor()
  result.processingModal.configOptions = await configOptions.allTextContents()
  result.processingModal.go2Profile = result.processingModal.configOptions.some((value) => value.includes('Go2 默认滤波'))
  await page.keyboard.press('Escape')

  let processingPayload = null
  await page.route('**/api/processing/start', async (route) => {
    processingPayload = route.request().postDataJSON()
    await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ status: 'started' }) })
  })
  const startProcessing = page.getByRole('button', { name: '开始处理' })
  result.processingModal.submitType = await startProcessing.getAttribute('type')
  await startProcessing.click()
  await page.getByRole('dialog').waitFor({ state: 'detached' })
  result.processingModal.requestIntercepted = Boolean(processingPayload?.source_id)
  result.processingModal.submittedModuleCount = processingPayload?.pipeline?.modules?.length || 0

  await page.getByRole('button', { name: /清理进程/ }).click()
  await page.getByRole('alertdialog').waitFor()
  result.cleanupDialog = await page.evaluate(() => ({
    title: document.querySelector('[data-slot="alert-dialog-title"]')?.textContent,
    width: document.querySelector('[data-slot="alert-dialog-content"]')?.offsetWidth,
    actionHeight: document.querySelector('[data-slot="alert-dialog-action"]')?.offsetHeight,
  }))
  await page.getByRole('button', { name: '取消' }).click()

  const compareButton = page.getByRole('button', { name: /前端 \/ PGO 对比/ })
  await compareButton.click()
  await page.locator('.cloud-metrics').nth(1).waitFor({ timeout: 30000 })
  await page.waitForTimeout(500)
  await page.screenshot({ path: '/tmp/d1max-map-workspace-compare.png', fullPage: true })
  result.compare = await page.evaluate(() => ({
    panels: document.querySelectorAll('.cloud-panel').length,
    canvases: document.querySelectorAll('.cloud-host canvas').length,
    labels: [...document.querySelectorAll('.cloud-label')].map((node) => node.textContent),
  }))

  await page.getByRole('tab', { name: /规划产物/ }).click()
  await page.locator('.map-row').first().waitFor()
  result.desktop.planningVisible = await page.locator('.map-row').first().innerText()

  const archiveDeletePayloads = []
  await page.route('**/api/archive/delete', async (route) => {
    const payload = route.request().postDataJSON()
    archiveDeletePayloads.push(payload)
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        deleted_count: payload.delete_all ? 8 : payload.item_ids.length,
        deleted_ids: payload.item_ids,
        freed_size_human: '1.0 MB',
      }),
    })
  })
  await page.getByRole('tab', { name: /已归档/ }).click()
  await page.locator('.map-row').first().waitFor()
  await page.screenshot({ path: '/tmp/d1max-map-workspace-archive.png', fullPage: true })
  const firstArchiveCheckbox = page.getByRole('checkbox', { name: /选择归档地图/ }).first()
  await firstArchiveCheckbox.click()
  const selectedCount = await page.locator('.archive-selected-count').innerText()
  await page.getByRole('button', { name: '删除所选', exact: true }).click()
  await page.getByRole('alertdialog').waitFor()
  const selectedDialogText = await page.getByRole('alertdialog').innerText()
  await page.getByRole('button', { name: '确认永久删除', exact: true }).click()
  await page.getByRole('alertdialog').waitFor({ state: 'detached' })
  await page.getByRole('button', { name: '全部删除', exact: true }).click()
  await page.getByRole('alertdialog').waitFor()
  const allDialogText = await page.getByRole('alertdialog').innerText()
  await page.getByRole('button', { name: '确认永久删除', exact: true }).click()
  await page.getByRole('alertdialog').waitFor({ state: 'detached' })
  result.archive = await page.evaluate(({ selectedCount, selectedDialogText, allDialogText, payloads }) => ({
    rows: document.querySelectorAll('.map-row-shell.selectable').length,
    checkboxes: document.querySelectorAll('.map-row-shell [data-slot="checkbox"]').length,
    toolbar: Boolean(document.querySelector('.archive-toolbar')),
    selectedCount,
    selectedDialogText,
    allDialogText,
    payloads,
  }), { selectedCount, selectedDialogText, allDialogText, payloads: archiveDeletePayloads })

  const wide = await browser.newPage({ viewport: { width: 2048, height: 1118 } })
  wide.on('pageerror', (error) => result.consoleErrors.push(error.message))
  await wide.goto(baseUrl, { waitUntil: 'networkidle' })
  await wide.locator('.cloud-metrics').waitFor({ timeout: 30000 })
  await wide.screenshot({ path: '/tmp/d1max-map-workspace-wide.png', fullPage: true })
  result.wide = await wide.evaluate(() => ({
    scrollWidth: document.documentElement.scrollWidth,
    viewportWidth: innerWidth,
    sidebar: document.querySelector('.sidebar')?.offsetWidth,
    inspector: document.querySelector('.inspector')?.offsetWidth,
    tabs: document.querySelector('.section-tabs')?.offsetWidth,
    previewToolbar: document.querySelector('.preview-toolbar')?.offsetHeight,
    mapRow: document.querySelector('.map-row')?.offsetHeight,
  }))

  const mobile = await browser.newPage({ viewport: { width: 900, height: 900 } })
  mobile.on('pageerror', (error) => result.consoleErrors.push(error.message))
  await mobile.goto(baseUrl, { waitUntil: 'networkidle' })
  await mobile.locator('.app-shell').waitFor()
  await mobile.screenshot({ path: '/tmp/d1max-map-workspace-narrow.png', fullPage: true })
  result.mobile = await mobile.evaluate(() => ({
    scrollWidth: document.documentElement.scrollWidth,
    viewportWidth: innerWidth,
    inspectorBelow: document.querySelector('.inspector').getBoundingClientRect().top >= document.querySelector('.sidebar').getBoundingClientRect().bottom - 1,
  }))

  const ok = result.consoleErrors.length === 0
    && result.api.overviewStatus === 200
    && result.api.processedCount === 0
    && result.api.archivedProcessedCount === 1
    && result.api.emptyDeleteStatus === 400
    && result.api.emptyDeleteDetail.includes('没有可永久删除')
    && result.desktop.scrollWidth === result.desktop.viewport[0]
    && result.desktop.rows > 0
    && result.desktop.canvas
    && !result.desktop.has2dFeature
    && result.desktop.rmw
    && result.desktop.latestWarning
    && result.desktop.shadcnCards >= 3
    && result.desktop.shadcnTabs >= 1
    && result.desktop.shadcnSelects >= 1
    && result.desktop.rowDecorationsContained
    && result.desktop.sizing.topbar >= 72
    && result.desktop.sizing.runtime >= 104
    && result.desktop.sizing.tabs >= result.desktop.sizing.sidebar - 32
    && result.desktop.sizing.search >= 42
    && result.desktop.sizing.mapRow >= 88
    && result.desktop.sizing.compactButton >= 38
    && result.desktop.sizing.cloudTool >= 36
    && result.desktop.tabs.some((value) => value.includes('处理结果'))
    && result.processingModal.sourceLocked
    && result.processingModal.go2Profile
    && result.processingModal.structuralConfig
    && result.processingModal.yamlDownload
    && result.processingModal.parameterBlocks === 4
    && result.processingModal.orderButtons === 8
    && result.processingModal.shadcnDialog
    && result.processingModal.shadcnSwitches === 4
    && result.processingModal.shadcnInputs >= 1
    && result.processingModal.switchToggles
    && result.processingModal.submitType === 'submit'
    && result.processingModal.requestIntercepted
    && result.processingModal.submittedModuleCount === 4
    && result.processingModal.sizing.width >= 900
    && result.processingModal.sizing.input >= 40
    && result.processingModal.sizing.switch >= 20
    && result.processingModal.sizing.orderButton >= 30
    && result.cleanupDialog.title.includes('清理 Web 建图进程')
    && result.cleanupDialog.width >= 480
    && result.cleanupDialog.actionHeight >= 38
    && result.desktop.tabs.some((value) => value === '处理结果0')
    && result.archive.rows === 8
    && result.archive.checkboxes === result.archive.rows
    && result.archive.toolbar
    && result.archive.selectedCount.includes('1')
    && result.archive.selectedDialogText.includes('永久删除 1 个归档点云')
    && result.archive.allDialogText.includes('清空整个归档区')
    && result.archive.payloads.length === 2
    && result.archive.payloads[0].delete_all === false
    && result.archive.payloads[0].item_ids.length === 1
    && result.archive.payloads[1].delete_all === true
    && result.archive.payloads[1].item_ids.length === 0
    && !result.processingModal.hasTwoDimensionalOutput
    && result.compare.panels === 2
    && result.compare.canvases === 2
    && result.wide.scrollWidth === result.wide.viewportWidth
    && result.wide.sidebar >= 380
    && result.wide.inspector >= 400
    && result.wide.tabs >= result.wide.sidebar - 32
    && result.wide.previewToolbar >= 64
    && result.wide.mapRow >= 88
    && result.mobile.scrollWidth === result.mobile.viewportWidth
    && result.mobile.inspectorBelow
  console.log(JSON.stringify({ ok, ...result }, null, 2))
  if (!ok) process.exitCode = 1
} finally {
  await browser.close()
}
