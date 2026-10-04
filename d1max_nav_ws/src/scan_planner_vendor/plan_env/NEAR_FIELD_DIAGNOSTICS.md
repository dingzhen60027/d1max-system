# 原生近身诊断：只读证据，不是碰撞策略

本模块沿用 `GridMap::inspectInflateOccupancy`、`strictRawVoxelStatus`、实际逐束积分和双圆柱几何。没有第二套占据地图，也不清除地面、自体、近身区域或未知格。

## 状态和计数

`inspectInflateOccupancy(position, yaw, true)` 额外返回以**世界整数索引**去重的体素列表和计数。原来的 `counts` 保持不变，仍包括双圆柱重叠。每格携带 `cylinder_mask`：1 后圆柱、2 前圆柱、3 两者。索引不是循环缓冲地址。

原生 `strictRawVoxelStatus` 完全未改：0 自由，1 占据，2 阻塞的不确定状态。诊断另外区分：

| 诊断类别 | 判断依据 | 原生处理 |
| --- | --- | --- |
| `observed_free` | 原生饱和自由阈值 | 0 |
| `occupied` | 超过原生占据阈值 | 1 |
| `never_observed` | 尚为确切的未观测先验 | 2 |
| `observed_insufficient` | 已不为先验，但未达到自由/占据条件 | 2 |
| `invalid` | 非有限值或异常先验以下数值 | 仍阻塞 |
| `outside` | 不在当前原生地图内 | -1 |

`observed_insufficient` 不能单凭 log-odds 解释成“冲突”；有限 hit/miss 见证也不是完整历史。`CollisionEvidence::state()` 的分类优先级不是碰撞函数的早停返回顺序。不得用诊断类别重算、覆盖 `getInflateOccupancy` 的返回值。

## 可选来源记录

`NearFieldDiagnostics` 默认关闭，没有新增默认 ROS 订阅或在线导出服务。离线 probe 显式开启时，实际射线遍历同时旁路记录 ROI 内每格、每雷达最近的 hit 和 miss：来源编号、原始点索引、ring、逐点源时间/映射时间、ray origin、原始终点、裁剪后终点、context/projection sequence、接收与积分调用时间。

`ray_visits` 是捕获到的射线访问次数；`votes` 是通过当前帧去重后进入缓存的票，不等于最终 log-odds 更新次数。hit/miss 同时存在时仍由**原生投票**处理。诊断不参与该计算。缺少或冲突的可选字段只使 `source_fields_available=false`，不改变原有云接收规则。

存储按 ROI、体素数和每次积分事件数设硬上限；每源最多保留每格各一个 hit/miss。超预算累加 dropped 计数，不修改原生射线积分。滑窗先按旧窗口恢复世界索引并删除离开的格子，整图/定位身份 reset 同步清空。记录可能很旧，不是可续期的自由空间或运动许可。除 sidecar 外，两源 pending 和合并 metadata 数组也受现有每源最多 100,000 点的上限约束；诊断开启有额外开销，不适合未经测量直接常开到 NUC。

## 文件级 probe 接口

`offline_projected_rays_probe` 不调用 ROS init，不创建节点或 SDK。原 `configure/rays/tick/summary` 接口保持；可加：

```json
{"diagnostics":{"enabled":true,"center":[-0.22,1.75,0.03],"half_extent":[0.8,0.8,0.65],"max_cells":16384,"max_events_per_integration":1000000}}
```

这段作为 `configure` 的字段，不是独立命令。`effective_parameters_sha256` 必须对应保存的有效参数文件；摘要由离线调用者计算。

在原 query 内加 `detailed:true`；常规时间线用 `export_voxels:false`，最后一次快照输出完整格子。每条最多导出 4096 格，截断会显式标记，分类计数不因导出截断改变。可提供**记录中的** `orientation_xyzw`、`source_stamp_ns`；不提供就标记缺少 roll/pitch，不从 yaw 伪造完整姿态。碰撞模型仍是原生 yaw-only 直立双圆柱。

新增只供离线检查的命令：`raw_query` 按世界索引读取，`context` 调用原生定位上下文方法，`slide` 调用原生滑窗，`reset` 调用原生 reset。它们不是 ROS 命令。

`query_now_ns` 是回放注入的判断时刻；`query_source_stamp_ns` 是查询姿态源时间；`diagnostic_wall_unix_ns` 是本次工具执行时间；`integration_call_start_ns` 不是实时积分完成时间。不得用这些字段替换原始传感器租约。

实测地面、实体自体和安全余量未提供时保持 null/未确认。导出的体素 AABB 和机身坐标角点不能代替物理标定。

## 验证和部署边界

09-27 本轮报告在 `docs/reports/NEAR_FIELD_FEASIBILITY_20260927.md`；固定录包/CDR、前后对照和命令在对应实验目录。构建只写独立实验目录，**没有更新运行 build/install 或发布副本**。GridMap 类布局有新增成员，未来若经授权部署，应一致重编 ABI 下游，不能只替换单个库。本轮逐束后端仍不可运动。
