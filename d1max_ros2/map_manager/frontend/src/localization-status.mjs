const ms = value => Number.isFinite(value) ? (value * 1000).toFixed(1) : '—'

export function localizationIssue(health = {}) {
  const front = health.frontend || {}
  if (health.state === 'navigation_fault') {
    const reason = health.navigation?.fault || health.navigation?.state || 'unknown'
    const resetUnknown = reason === 'filter_reset_unknown'
    return { title: resetUnknown ? '滤波重置未确认' : '导航输出检查未通过',
      detail: resetUnknown ? '停止并重启定位；未自动重试，不需要重连 SDK。' : `原因：${reason}。核对数据后重新给初值；局部 LIO 状态单独检查。`,
      destructive: true }
  }
  const gap = `IMU 间隔 ${ms(front.last_imu_gap_sec ?? front.max_observed_imu_gap_sec)} ms / 上限 ${ms(front.imu_gap_limit_sec)} ms`
  if (health.state !== 'fault' && (health.state === 'recovering_local' || front.recovering && !front.fault)) {
    return { title: '局部里程计重新初始化', detail: `${gap}。请保持静止，完成后重新画初始箭头；旧定位已撤销。`, destructive: false }
  }
  if (health.state === 'fault') {
    const reason = health.local_fault || front.reason || health.fusion?.message || 'unknown'
    if (reason === 'imu_recovery_limit') return { title: 'IMU 多次中断，已暂停恢复', detail: `${gap}。核对 IMU 数据链路后停止并重新启动定位；SDK 无需重连。`, destructive: true }
    if (reason === 'imu_gap' || reason === 'imu_start_uncovered') return { title: 'IMU 时间覆盖中断', detail: `${gap}。局部里程计已停止；该会话需重新初始化，不能用初值跳过。`, destructive: true }
    return { title: '局部里程计已停止', detail: `原因：${reason}。请核对定位日志；未自动绕过检查。`, destructive: true }
  }
  if (health.initial_pose_ready === false && health.frontend_ready && health.head_direction !== 1) {
    return { title: '初值提交等待机头前向状态', detail: health.head_direction === 2 ? '当前为尾部前向模式，程序不会替你切换机器人模式；这不是地图箭头方向。' : '等待新鲜、明确的机头方向状态。', destructive: false }
  }
  return null
}
