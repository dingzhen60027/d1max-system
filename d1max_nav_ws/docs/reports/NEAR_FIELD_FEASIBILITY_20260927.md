# 09-27 近身查询 A～D 离线诊断交接

## 2026-10-09：官方规划语义与可恢复性整改

本节是本轮软件结果；下方保留 09-27 历史记录。证据目录为
`experiments/near_field_feasibility/20261009_recovery/`，仅保存验证材料，不是新运行入口。
主线仍使用同一生产 SCAN、GridMap、跟踪核心和 BT，通信为 Zenoh。

### 官方对照与边界

已逐项对照 [SCAN ROS2 的碰撞查询](https://github.com/wuyi2121/SCAN-Planner/blob/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0/src/planner/plan_env/include/plan_env/grid_map.h)、
[原生局部目标选择](https://github.com/wuyi2121/SCAN-Planner/blob/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0/src/planner/plan_manage/src/scan_replan_fsm.cpp)及
[legbot 的集成查询](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_env/include/plan_env/grid_map.h)。
官方 A*/目标选择/优化使用膨胀障碍查询，不要求整条候选的所有体素均有逐格自由证明。
当前默认参数已经是 `planning_collision_policy=official`、
`trajectory_geometry_policy=official_inflated_geometry`，本轮没有再添加候选未知门禁。
独立的实际指令制动扫掠仍检查原始双雷达、支撑和源时间；候选路径不是运动许可。

### 实际修复

| 断点 | 原因 | 修复及保留边界 |
|---|---|---|
| 混合未知/占据误报未知 | 圆柱、前后圆柱与运动扫掠先遇到未知立即返回，遮住后面的确定占据；绕障反馈只接受确定占据 | 原有限查询统一按占据→越界→未知→自由汇总；占据早退，原几何/源时/时间预算不变，未知仍阻止执行 |
| 直行受阻无法请求新入口 | 原生验证器接受直行占据反馈，tracker 外层和本体却要求角速度非零 | 接通精确匹配的前进直行反馈，保留任务/曲线，进入 WaitingEntry 请求新候选；不重复朝原方向对齐 |
| 障碍移除后窄廊仍卡死 | 侧向目标强制标志只有换参考或移动 0.3 m 后解除；静止窄廊无法满足 | 改成一次受阻证明的一次侧向尝试；拿不到快照不消费，实际尝试后消费，后续环境更新重新检查正常路线；真实占据仍拒绝运动 |
| 诊断混淆规划与执行 | 只显示一个 collision 值，重复圆柱计数被当成唯一体素 | 同体输出 official planning、严格证据、唯一/原始重叠、从未观测/不足/过期、ROI 内外及源时间；ROI 不是自体包络 |

运动日志新增实际访问的唯一体素分项与 `query_complete`；占据早退或计算截止时是部分计数，
不冒称完整体积已验证。未扩大实体滤除盒、未清空近身、未刷新旧时间、未修改物理验收标志。

### 同输入与危险反例

- 冻结默认代码的两个原生混合状态用例均失败：先出现未知时误报 `motion_sweep_unknown_or_expired`，
  前未知/后占据时返回 2；修复后分别正确返回占据，纯未知仍拒绝。
- 完整生产 `getLocalTarget` 方法体的同输入窄廊用例：旧实现 13 检查中 4 失败，修后 13/13。
  无快照不消费请求；取消前不假装尝试完成；持续实体障碍、纯未知两版均拒绝运动。
- tracker 相同直行占据证据：旧本体 0/1、外层 1/2，修后 1/1、2/2；
  unknown、过期、异任务、零命令和倒退反例仍拒绝。反馈是重规划请求，不签发运动许可。
- 原生 GridMap 130、SCAN gtest 141、tracker 163 项通过；另有 149 项 Python 策略/输入/安全回归通过。
  这些是软件回归，不是实机导航验收；没有运行 ROS 启动类测试。
- 补齐旧测试夹具的源时、准备事务证明队列、ACK 应用时刻和规划膨胀层；
  明确接收 owner 心跳不等于实际指令获准，断证据时实测输出仍是 hold/zero。未为测试放松生产判断。

### 两秒真实录制输入

输入为 09-26 原包 `log/scan_input_audit_20260926_E6fZII/bag/bag_0.db3` 的冻结逐束输入，
bag SHA256 `09f6c86f5ff03a0083783434424070e0d7f465fc68619ac683446b84d11aacd4`。
截取原请求前 77 条、35 份原字节 CDR，接收区间
`1790416059387410380–1790416061387410380 ns`；追加两个只读快照，不重投影、不改源时间。
保留该录制的历史分辨率、坐标和标定，不能当作当前实机或当前 5 cm 会话的性能验收。

| 查询 | 官方规划 / 严格证据 | 唯一：自由/占据/未知/越界 | 原始重叠计数 |
|---|---|---|---|
| 新鲜机身包络 | 1 / 1 | 290 / 2 / 836 / 0 | 290 / 2 / 1052 / 0 |
| 新鲜前方 2 m | 0 / 0 | 1152 / 0 / 0 / 0 | 1344 / 0 / 0 / 0 |
| 不追加输入 600 ms 后的前方 | 0 / 2 | 0 / 0 / 1152 / 0 | 0 / 0 / 1344 / 0 |

机身 836 个未知包含 835 从未观测、1 过期自由格；断流尾段的前方 1152 格全部是过期自由证据。
尾段原始缓冲摘要不变，证明显示几何仍通行并不代表数据仍新鲜。
诊断增强前后保持同一原生库，旧字段/原始缓冲/碰撞结果零差异；本轮新库再次截取验证，分类与计数一致。

两个机身占据格来自后雷达，在记录的机身坐标后下方约
`[-0.383,-0.273,-0.463]`、`[-0.405,0.213,-0.463] m`。
缺少同步、可信的地面平面与轮腿实体测量，不能判为自体或地面；该录制不能证明今天实机周围有障碍。
仍需同姿态下测量原点/外参、地面到机身参考高度及轮腿包络，才能正式修正实体自体滤除。

### 复现与范围

构建与对照目录：`/home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7`。
默认更新前完整恢复快照位于其 `before-runtime`；该快照不作为另一默认入口。

```bash
cd /home/dndx/d1max_nav_ws
source /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/environment.sh
ctest --test-dir /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/plan_env --output-on-failure
ctest --test-dir /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/scan \
  -E 'test_planner_startup|test_waypoint_parameters' --output-on-failure
ctest --test-dir /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/tracker --output-on-failure
python3 src/scan_planner_vendor/plan_manage/test/test_blocked_entry_target_no_ros.py \
  --native-build /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/scan
/home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/plan_env/offline_projected_rays_probe \
  < experiments/near_field_feasibility/20261009_recovery/real_short_evidence/after/requests.jsonl
```

精确同输入基线/修复对照（两个基线命令预期失败，不能把退出码 1 当作构建失败）：

```bash
# 使用上面相同的 environment.sh；基线与修复共享同一输入和消息 ABI。
/home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/baseline-scan/test_motion_sweep \
  --gtest_filter=MotionSweep.EarlierUnknownCannotHideRealObstacleAndSuppressLocalRecovery:MotionSweep.RawUnknownFrontDoesNotHideOccupiedRearCylinder
/home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/scan/test_motion_sweep \
  --gtest_filter=MotionSweep.EarlierUnknownCannotHideRealObstacleAndSuppressLocalRecovery:MotionSweep.RawUnknownFrontDoesNotHideOccupiedRearCylinder
python3 src/scan_planner_vendor/plan_manage/test/test_blocked_entry_target_no_ros.py \
  --native-build /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/scan \
  --source /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/before-runtime/source_snapshot/nav_ws/src/scan_planner_vendor/plan_manage/src/scan_replan_fsm.cpp
python3 src/scan_planner_vendor/plan_manage/test/test_blocked_entry_target_no_ros.py \
  --native-build /home/dndx/Documents/Codex/2026-10-02/repo/.scan-mainline-audit-FptyH7/scan
```

实际源码、安装库、ELF、selector 和 Web activation 的更新结果见
[当前主线状态](../status/PROJECT_TAKEOVER_STATUS_20260928.md)。本轮不启动服务、SDK、实机，不自动 push。
保留等待物理验收；不承诺旧录包中的真实未知体积已经被传感器观测。

## 09-27 后续修复更新（历史）

第一轮诊断之后，已完成累计校正失效传播、同 context 更高 sequence 原生重建、逐格自由证据时效与恢复通知、实际初始姿态转向扫掠的源码修复及同源隔离构建。完整前后对照、危险反例及未解决项见 [第二轮修复报告](../../experiments/near_field_feasibility/20260927_fix_HuNTuw/README.md)。该轮当时未部署；当前默认同步事实见本文首节及主线状态。物理验收仍未通过；下文是保留的第一轮诊断记录，不代表后续只改了诊断。

## 第一轮记录

状态：`WAITING_NEAR_FIELD_PHYSICAL_VALIDATION`。未启动生产服务、未连接 SDK、未发布初值/目标/运动，未部署或提交 push。

本轮从已推送的 `832a520` 和实际工作区、已安装二进制、会话、真实录包分别冻结开始。新增诊断和离线回归代码已写入实际源码工作区；运行目录的 build/install 保持原状。不要把源码修改当成部署完成。

## 核心结果

1. 真实录包可复现近身 149/149 次阻塞、前方 2 米 149/149 次自由查询。末次原始计数 `295 / 1 / 1060 / 0` 含重叠；去重后是 **295 自由、1 占据、844 从未观测**，合计 1140 格，重复计数 216 次。末次“已观测但证据不足”为 0；不能将此结论外推到所有时刻。
2. 唯一占据格是 `[-7,17,-6]`，世界 AABB `[-0.56,-0.48] × [1.36,1.44] × [-0.48,-0.40] m`。已追溯后雷达原始回波。格子顶面比本次查询包络底面高约 **2.30 cm**，所以相交是真实的原生查询结果。回波的物理类别、真实地面高度、脚部姿态尚未证明。
3. 完全相同的 310 条原生请求、141 份固定 CDR，在旧二进制、修改版诊断关闭、修改版诊断开启之间，原有输出无差异；新版本开关诊断后的完整原始体素缓冲摘要一致。
4. 两个原生地图反例已复现：同身份校正每次 12 mm、累计 120 mm，可留下新旧占据位置；远处两源持续更新时，1.45 秒未更新的近身自由格仍保留，不能用全流 fresh 证明区域 fresh。另有原地旋转反例：起止朝向均自由，中间朝向会碰撞。这些是独立风险，不是对真实近身阻塞根因的武断归因。

## 为什么没有放开起点

当前证据支持“查询包络内既有实际占据体素，又有大量未观测体积”，不支持“这些都只是腿，删掉即可”。先量测地面与机身高度、两雷达实际外参/可见域、可靠实体几何，才能区分实体自体、地面量化、外部低障碍和安全余量。

因此本轮只修正诊断缺口和测试夹具契约，**没有修改碰撞/投票/阈值、没有清空近身、没有放开未知空间**。已有运动测试默认选中了逐束后端，正确被生产门禁拒绝；测试已分成 legacy 准备成功和当前逐束明确拒绝两条，未为测试放宽门禁。

执行前两个结构性整改已写成独立设计：安全扫描仍取 LIO 筛选点云；tracker 仍用 global odometry。后者需要带时间与校正版本的整条轨迹/状态转换和换轨准入，不能只换话题或 frame 名。

## 交付入口

完整证据目录：
`/home/dndx/d1max_nav_ws/experiments/near_field_feasibility/20260927_8YtiSe/`

- [根因与证据报告](../../experiments/near_field_feasibility/20260927_8YtiSe/root_cause_report.md)：已确认、未确认、同输入对照及性能边界。
- [执行前结构整改](../../experiments/near_field_feasibility/20260927_8YtiSe/architecture_decisions.md)：独立安全观测、连续 odom 控制、转向扫掠和支持几何。
- [物理验收清单](../../experiments/near_field_feasibility/20260927_8YtiSe/physical_validation_checklist.md)：需要补什么，不猜什么。
- [准确复现步骤](../../experiments/near_field_feasibility/20260927_8YtiSe/reproduce.md)：仅文件级工具与明确纯测试，不运行 aggregate CTest。
- [离线分层诊断图](../../experiments/near_field_feasibility/20260927_8YtiSe/diagnostic_layers/index.html)：历史快照，不是实时 RViz 或执行许可。
- `baseline.json`、`effective_parameters.json`、`query_timeline.jsonl`、`query_voxels.jsonl`、`geometric_findings.json`、`time_contract_report.json`、`regression_report.json`、`change_manifest.json` 与 `changes.patch`。

没有实测空场/低障碍/腿部同步真值，没有本包内的原始 IMU、积分完成通知、已接受轨迹和撤销记录。原生查询/AStar/纯契约通过，不等于完整实机 SCAN 导航已验收；这些缺项明确保留，下一轮需另行授权。
