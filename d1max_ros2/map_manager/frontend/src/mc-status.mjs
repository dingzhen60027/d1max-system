// Status text reflects telemetry, not merely a successful SDK socket write.
export function mcStatus(report = {}, fallbackFresh = false) {
  const fresh = typeof report.stream_fresh === 'boolean' ? report.stream_fresh : fallbackFresh
  if (fresh && Number.isFinite(report.observed_hz) && report.observed_hz > 0) {
    return { label: report.observed_hz.toFixed(1) + ' Hz', detail: report.acknowledged === false ? 'MC 数据已收到，配置回执未收到' : report.rate_ok === false ? 'MC 实收频率偏离 SDK 参考频率' : 'OnMcData 实收频率' }
  }
  const states = {
    disconnected: 'SDK 未连接', waiting_robot_state: '等待状态', waiting_ready: '准备订阅',
    waiting_ack: '等待 MC 应答', waiting_stream: '等待 MC 数据', retry_cooldown: 'MC 自动重试',
    stream_stale: 'MC 数据中断', write_failed: 'MC 发送异常', failed: 'MC 未应答',
    replay_blocked: '回放禁用 MC', streaming_unconfirmed: 'MC 测速中', measuring_rate: 'MC 测速中', ack_off: 'MC 订阅关闭',
    waiting_app: '等待 App 释放', available: 'SDK 待接管', takeover_pending: 'SDK 接管中', takeover_verifying: '核对 SDK 归属',
    takeover_timeout: '接管待核对', takeover_rejected: '接管被拒绝', takeover_write_failed: '接管发送异常',
  }
  return { label: states[report.state] || (fresh ? 'MC 测速中' : '等待 MC'),
    detail: report.ownership?.error || (report.state === 'waiting_app' ? 'App 释放后，SDK 收到控制权可用通知才申请接管' :
      report.state === 'takeover_pending' ? '已申请 TakeControl，等待机器人回执' :
      report.state === 'takeover_verifying' ? '回执已收到，等待连续两帧确认 SDK 归属；尚不代表 MC 已恢复' :
      report.state === 'takeover_timeout' ? '接管状态未知，不自动重试；请核对后重连' : '') || report.write_error || (Number.isFinite(report.next_retry_sec) && report.next_retry_sec >= 0
      ? Math.ceil(report.next_retry_sec) + ' 秒后重试订阅' : '只使用 OnMcData，不替代为低频状态速度') }
}
