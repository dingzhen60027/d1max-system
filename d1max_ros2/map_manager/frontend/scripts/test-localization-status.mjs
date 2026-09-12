import test from 'node:test'
import assert from 'node:assert/strict'
import { localizationIssue } from '../src/localization-status.mjs'

test('recoverable gap is explicit and requires a new seed', () => {
  const issue = localizationIssue({ state: 'recovering_local', frontend: { recovering: true, last_imu_gap_sec: .065, imu_gap_limit_sec: .05 } })
  assert.match(issue.detail, /65.0 ms.*50.0 ms/)
  assert.match(issue.detail, /重新画初始箭头/)
  assert.equal(issue.destructive, false)
})
test('exhausted recovery and clock faults remain failures', () => {
  for (const reason of ['imu_recovery_limit', 'imu_clock_reset']) {
    const issue = localizationIssue({ state: 'fault', frontend: { fault: true, reason, recovering: true } })
    assert.equal(issue.destructive, true)
  }
})
test('tail-forward gating is not mistaken for the drawn arrow', () => {
  const issue = localizationIssue({ initial_pose_ready: false, frontend_ready: true, head_direction: 2 })
  assert.match(issue.detail, /不是地图箭头方向/)
  assert.equal(localizationIssue({ initial_pose_ready: true, frontend_ready: true, head_direction: 1 }), null)
})
test('local pose faults cannot be hidden by frontend recovery or tracking text', () => {
  const issue = localizationIssue({ state: 'fault', local_fault: 'local_pose_jump', frontend: { reason: 'tracking', recovering: true, fault: false } })
  assert.equal(issue.destructive, true)
  assert.match(issue.detail, /local_pose_jump/)
})
test('navigation filter fault is not reported as a stopped LIO or SDK outage', () => {
  const issue = localizationIssue({ state: 'navigation_fault', frontend: { ready: true }, navigation: { fault: 'global_correction_jump' } })
  assert.match(issue.title, /导航输出/)
  assert.match(issue.detail, /global_correction_jump/)
  assert.doesNotMatch(issue.title, /局部里程计已停止/)
  const reset = localizationIssue({ state: 'navigation_fault', navigation: { fault: 'filter_reset_unknown' } })
  assert.match(reset.detail, /未自动重试/)
  assert.match(reset.detail, /不需要重连 SDK/)
})
