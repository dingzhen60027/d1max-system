import { test } from 'node:test'
import assert from 'node:assert/strict'
import { mcStatus } from '../src/mc-status.mjs'
test('ownership handoff is not MC readiness or an automatic retry loop', () => {
  assert.equal(mcStatus({state:'waiting_app'}).label,'等待 App 释放')
  assert.equal(mcStatus({state:'takeover_pending'}).label,'SDK 接管中')
  assert.match(mcStatus({state:'takeover_verifying'}).detail,/尚不代表 MC/)
  assert.match(mcStatus({state:'takeover_timeout',next_retry_sec:0}).detail,/不自动重试/)
  assert.equal(mcStatus({state:'takeover_rejected',ownership:{error:'APP owns control'}}).detail,'APP owns control')
  assert.equal(mcStatus({state:'waiting_app',stream_fresh:true,observed_hz:50}).label,'50.0 Hz')
})
test('timeout describes missing ACK and an automatic retry, never an explicit rejection', () => {
  assert.equal(mcStatus({state:'failed'}).label, 'MC 未应答')
  assert.deepEqual(mcStatus({state:'retry_cooldown',next_retry_sec:12.2}), {label:'MC 自动重试',detail:'13 秒后重试订阅'})
})
test('actual MC rate is visible even when an ACK is missing', () => {
  const s=mcStatus({stream_fresh:true,observed_hz:49.9,acknowledged:false})
  assert.equal(s.label,'49.9 Hz');assert.match(s.detail,/回执未收到/)
})
test('stale status never shows the previous frequency', () => {
  assert.equal(mcStatus({stream_fresh:false,state:'stream_stale',observed_hz:50},true).label,'MC 数据中断')
  assert.equal(mcStatus({observed_hz:NaN},true).label,'MC 测速中')
  assert.equal(mcStatus().label,'等待 MC')
})
