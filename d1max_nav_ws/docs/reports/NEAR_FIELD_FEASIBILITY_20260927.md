# 09-27 近身查询 A～D 离线诊断交接

## 后续修复更新

第一轮诊断之后，已完成累计校正失效传播、同 context 更高 sequence 原生重建、逐格自由证据时效与恢复通知、实际初始姿态转向扫掠的源码修复及同源隔离构建。完整前后对照、危险反例及未解决项见 [第二轮修复报告](../../experiments/near_field_feasibility/20260927_fix_HuNTuw/README.md)。仍未部署，仍等待物理验收；下文是保留的第一轮诊断记录，不代表后续只改了诊断。

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
