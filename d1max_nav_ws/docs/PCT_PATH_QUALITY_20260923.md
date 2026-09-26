# PCT 长直走廊路径弯绕修复

日期：2026-09-23。范围：单层离线全局路径与 RViz 预览。未启动 SDK、运动控制、SCAN 或实机导航，未修改点云、tomogram 或 Zenoh 通信方案。

## 结论与原因

地面连通不等于路径形状合理。此前保存的长走廊路径横向摆动约 2 m，但相同起终点的直线能够通过完整通行检查。主要发现：

1. 原生代价插值把整数栅格中心与半格偏移混用：下角点为 `floor(x-0.5)`，权重却为 `x-lower`，产生外推及半格跳变。行代价 `[0,0,50,0,0]` 在 x=2.49/2.50 时分别得到 74.5/25。20 cm 分辨率下，仅移动 2 mm，代价跳变 49.5。已修正为整数格中心的一致插值及梯度；未修改层选择和原始代价。
2. 优化目标偏向沿零碎的低软代价格子走，而不是沿合法的直通方向走。修复插值本身并不足以消除全部弯绕，因此新增独立、可关闭的路径形状整理模块。

原生修复说明：[BILINEAR_PATCH.md](../src/d1max_pct_planner/test/native/BILINEAR_PATCH.md)。修复前源文件及运行库保存在 `log/pct_preview/path_quality_20260923_audit/native_backup_before_fix/`。

## 当前处理链

PCT A* → GPMP 原生曲线 → 原有严格检查 → 同层可视直连简化 → 五次 Bézier 拐角过渡 → 多项式穿格检查 → 最终发布折线独立检查。

- 原生结果不合法时直接失败，不能靠后处理把失败结果伪装成通过。
- 可直接通行时输出 XY 直线；真实拐角使用切向连续、XY 几何 C2 的连接。
- Z 仍跟随 tomogram 地面并保留所选起终点，不宣称三维 C2 或时间轨迹连续。
- 修整必须保持精确起终点，且相对本次原生结果不恶化长度、总转角和所检查的曲率统计。
- 曲线穿越的栅格边界及区间内部、最终发布折线均检查；不是只抽样几个点。
- 无合格候选时明确保留已验证原生结果；不会输出未检查的捷径。
- 只处理同层路径。工作量有点数、长度、可视检查次数上限；预览任务超过 20 秒失败关闭。

实现分离为 `path_quality.py`（指标）、`corridor_refinement.py`（形状整理）、`tomogram_route.py`（集成）。其他配置默认不开启修整。

## 配置与效果

生效配置：`src/d1max_pct_planner/config/preview_flat_floor.yaml`。

```yaml
astar_cost_weight: 1.0
optimizer_cost_margin: 8.0
max_heading_rate: 1.0
path_refinement: visibility_c2
refinement_corner_cut_m: 1.5
```

`optimizer_cost_margin` 是软代价罚项起点，不是几何膨胀；`max_heading_rate` 属于原生优化器内部时间参数，不能解释为机器人实测角速度限制。1.5 m 是转角切入长度上限，实际受相邻线段与碰撞检查约束，并不是保证的最小转弯半径。

对比为“原插值 + A* 2 / margin 8 / heading 10”与“修复插值 + 当前参数 + 显式修整”的整体效果，不能把收益全部归于插值补丁。

| 用例 | 旧 XY 长度 | 新 XY 长度 | 旧曲率 P95 | 新曲率 P95 |
|---|---:|---:|---:|---:|
| 23 m 直走廊 | 24.54 m | 23.33 m | 0.473 /m | 约 0 /m |
| 18 m 直走廊 | 19.13 m | 18.13 m | 1.275 /m | 约 0 /m |
| 11 m 直走廊 | 12.54 m | 11.29 m | 1.593 /m | 约 0 /m |
| 保留用户起终点的约 39 m 路线 | 40.37 m | 38.98 m | 0.635 /m | 0.254 /m |

![相同地图与起终点的路径对比](../log/pct_preview/path_quality_20260923_audit/path_quality_before_after.png)

最终离线套件 `final_corridor_14`：14/14 通过原通行检查，其中 12 组使用修整，`break_B1` 与 `ring_quadrant_4` 没有找到更好的合法候选，保留本次原生路径。不是宣称所有路线均达到最优形状。曲率使用统一 0.1 m XY 弧长重采样估计，非解析最大曲率约束，不能推导高速跟踪能力。

## 验证与复现

代码回归：`src/d1max_pct_planner/test` 与 `tools/pointcloud_preprocessing/tests` 合计 **430 passed, 1 skipped**。包含原生插值的 16 项独立编译测试，路径指标及修整模块测试，以及配置/集成测试。

离线复现（`--output` 必须为新的目录）：

```bash
cd /home/dndx/d1max_nav_ws
python3 tools/benchmark_pct_path_quality.py \
  --output log/pct_preview/path_quality_recheck \
  --astar-cost-weight 1 --optimizer-cost-margin 8 \
  --max-heading-rate 1 --path-refinement visibility_c2
```

保存的最终证据：

- `log/pct_preview/path_quality_20260923_audit/final_corridor_14/summary.json` 及各用例 JSON：算法、参数、路径、质量指标及独立检查。
- `log/pct_preview/20260923_112428_41cd27714bc0/quality_live_verification.json`：保留上次 `path_000038.json` 的起终点，在隔离的实际预览中重新规划，1057 个路径点通过发布后复核，RViz 存在路径订阅。XY 长度 43.8427 m，曲率 P95 0.209 /m。此项验证 ROS 消息链路，不等于检查了 RViz 渲染截图。
- 新预览由 `d1max-pct-preview.service` 管理，仍为 Zenoh / ROS Domain 24 / 私有回环端口 7465；`robot_connected=false`，`motion_enabled=false`。

tomogram 保持不变：

```text
maps/processed/sc_pgo_20260919_pct_flat_floor_v6_20260923/tomogram.npz
SHA256 fb6897abb9624e493bea6f7c6ac723c20627bcd51f0dc97af9e320c6aab59433
```

## 边界与后续

硬通行掩码、阈值 20、0.55 m 顶空要求、0.17 m 地面阶差限制和地图膨胀未改。当前 `unknown_ceiling_policy=allow_unobserved` 仍是既有假设，不能解释成全部净空已经实测确认。

修整在同一合法区域内优先几何简洁，软代价可能上升，也不承诺始终沿走廊正中心；结果保存修整前后的代价审计。这不是放宽硬障碍阈值，但确实改变了软偏好。真正上线前仍需机器人足迹/净空确认、时间参数化、速度与转弯能力约束、实时避障和跟踪测试。本轮只解决离线全局路径不必要的弯绕，不能宣称完整导航已验证。
