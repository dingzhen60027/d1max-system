# SCAN 近身未知体素：成因、修复和验证

分支：`isaacsim-simulation`。比较基线：`5dfaf8cddec952b3453a02e3a231de5228bd6017`。本文记录该基线之后的 UNKNOWN 研究、证据融合及任务复测，历史成功不代表当前连续换轨已经稳定。

**最新完成的正式实测（北京时间 2026-10-07）：v37 `crossing_blocker` 仍失败，当前主要阻断已经在真实行走响应。** 原 Action status 6、`execution_blocked_timeout:waiting_measured_motion_progress`；累计 XY **`.471879 m`**、原 owner credit **8 次**、确认路线弧长 **`.302689 m`**，目标仍距 **15.695410 m**。SDK 原状态在场景源时间 **9.74–37.72 s** 连续正向约 **27.98 s**（1572 个 applied 状态、约 78.602 s steady）；15–35 s 的曲线／运动证明为正、tracker 为 tracking，但实际 policy 输入持续 `.3000000119 m/s`，机身 20 s 累计 XY 仅约 `.000241329 m`。20–30 s 的 600 个实际 PhysX command 都非零，净 XY 仅 **8.754 μm**、四脚没有 `.01 m` 以上的实际 solid 离地样本。因此不能把这一长区间归为 UNKNOWN 撤权、C1 换轨中断或 owner 独立死锁；独立动作历史 A/B 已完成，但基线未复现正式停滞，记忆干预未加入默认控制链。新 exact native root 标签已在正式 first／last NPZ 出现非零 actor ordinal，但没有指定动态遭遇、到达或动态旧 HIT 退役关卡通过，**不能宣布 UNKNOWN 已解决或导航丝滑**。完整形体、IMU、地板与实测停车核对成立，原整体 clean shutdown 失败及硬件停止未确认保持。见 [v37 原失败、真实指令及传感器证据](verification/20261007/campus_crossing_v37.json)和 [分析](SCAN_UNKNOWN_ANALYSIS.md)。

**随后完成的组件 A/B：** [42 秒真实关节回放](verification/20261007/spot_navigation_replay_v37_ab.json)中，基线／动作记忆干预组净前进约 3.47／4.52 m；干预前 4189 次 500 Hz 测量逐值相同，原零命令起点到连续一秒停稳确认分别为 2.056／1.832 s。基线已经能够行走，故未复现正式 v37 的原地站立，不能认定该干预解决了导航。基线 full XYZ 峰值 `.532835 > .50`、实验组 full angular 峰值 `.823027` 均原样保留，后者与 body yaw `.173694` 分开记录。记录仅重构原保存命令事件，在裁剪平地场景执行；默认步态、模型域及选择器未因此改变。

**前次 v36 正式失败保留，实际路线进度与指定演员遭遇已有改善。** 累计 XY 行程 **7.221877 m**、原 owner credit **199 次**、确认路线弧长 **6.643420 m**；终点仍距目标 **9.285196 m**。原 Action status 6，原因 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof`。原 writer 有 **58 段**连续非零输出（原 applied source 首末样本 median **`.570 s`**／max **`5.560 s`**）；27 个精确 handoff ID 中 **13 成功／14 未成功**，初始 ACK 另计。指定行人接近—近距离—离开实际被观察到，137 个 near 样本、采样形体分离下界最低 `.568772 m`；这不证明让行、绕行或到达。完整身体、地板、原生 IMU 同源和实测停车成立，但最终持续 hold、整体 clean shutdown 失败、原硬件停止未确认保留，**当前管线仍未达到连续、丝滑导航，也没有正式横穿通过结论**。见 [v36 原失败及完整证据](verification/20261007/campus_crossing_v36.json)与 [剩余问题分析](SCAN_UNKNOWN_ANALYSIS.md)。

**历史 v33 正式任务失败记录保留。** `.481/.589 m` 标定和原 XYZ 空间前视已实际启用，但累计行程仅 **0.013900 m**，原 owner credit 始终 **0**，任务以 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof` 结束；完整曲线、局部 cap 和 proof 的剩余原因继续审查。v32 的 **0.056649 m** 失败与时间前视证据完整保留。见 [v33 失败证据](verification/20261007/campus_crossing_v33.json)、[v32 失败证据](verification/20261007/campus_crossing_v32.json)及 [下文审查](#v32-横穿失败与-v33-复测结果2026-10-07)。

**本机选择器与历史 v31 实测：** 默认候选仍为 `isaac-candidate-v31-campus`。v31 原 Action 近距实测 **1.040435 m** 后到达；`cancel_and_park` 行程 **1.047985 m** 后取消，最终 **2.018 s 源时间**静止窗口与独立评估通过。v31 横穿累计 **0.403687 m** 后仍因 `execution_blocked_timeout:waiting_measured_motion_progress` 失败：准备证明饥饿已修复，候选入口几何、低速／方向变化及持续进度尚未闭环。其余四关 `pending`，约 332.84 m 仍仅是离线路由。[v31 实测摘要](verification/20261006/campus_navigation_v31.json)和 [横穿审查](verification/20261006/crossing_failure_audit_v31.json)分别记录结果与分析，不能用短测推断动态避障通过。证据目录 `20261006` 保留 UTC 日期。

历史 v30 近距 **0.827614 m** 到达、取消 **1.197793 m / 2.02 s** 通过和横穿 **5.583955 m** 后失败，保留在 [v30 实测摘要](verification/20261006/campus_navigation_v30.json)，不与 v31 合并为整版成绩。

**历史未解决失败：** 旧轮式 `live_view_20261006_001` 实测行驶 **8.0010 m** 后未到达，任务以 `execution_blocked_timeout:waiting_admitted_nonzero_command` 失败。记录中的 generation 7 → 8 换轨未完成；根因尚未确证，不能把这条任务原因当作最终算法归因，也不能把它直接归为近身 UNKNOWN。实测停稳及任务软件退役成立。[随分支保存的失败取证](verification/20261006/live_view_failure.json)保留；历史 v19 三测与 v22 录屏成功不能替代该失败的修复与复测。

仓库内源代码和取证摘要使用相对链接；标明“本机证据”的路径对应未随 Git 发布的原始日志、几何数据或大文件，GitHub 无法直接打开。旧轮式换轨、v31 持续进度、v32/v33 启动和 v36 当前证明／换轨问题的完整任务修复尚未闭环。

该问题首先是地图证据合同与传感器可观测性的矛盾。PCT 使用全局三维地图，但原 SCAN 严格模式只使用实时射线生成的滚动栅格；全局地图的空体积没有进入局部碰撞查询。不能用“全局路线存在”推断机器人周围每个三维体素都有近期激光证据。

## 实际失败证据

保留的 `task_goal_v9_001` 使用真实 PhysX 双三维雷达和 IMU，目标 `(-2.8,-3,0)` 获原 BT 接受，原 PCT 产生 39 点路线，随后 SCAN 返回 `waiting_observed_space`。没有非零执行命令或实际导航运动。

原生碰撞查询与实际保存的射线回放一致：首个候选位置 `(-3.9,-3,0.35)` 的双圆柱包络包含 3740 个唯一体素，1906 个有严格 FREE 证据，1834 个从未观测；实际机器人仍约在 `(-4,-3,0.35)`。后续查询仍有 1832–1834 个从未观测体素，另有 0–2 个旧 FREE 正确过期。扫描源年龄为 100–333.3 ms，所以这次 v9 失败不能归因于整个扫描过期。

按实际机器人实体分类，首次查询的 1834 个未知体素中，404 个完全位于实体内；632 个与实体相交或属于保守边界候选；798 个确定在机器人实体外。后一类中，790 个体素中心被两个传感器的自体几何共同遮挡，8 个中心存在几何视线，但没有保留下来的真实 external-ray／DDA 自由证据。中心可见性不是完整体素自由的证明。这也说明仅豁免实体内部不能解决此问题，更不能把整个安全包络视为机器人实体并清空。

完整分类见 逐体素数据（本机证据：`/home/eric/wjg/d1max-build-isaac/unknown_research/v9_unknown_voxels.data.json`）；保存的原生查询与几何分类源文件在本机 `../d1max-build-isaac/unknown_research/v9_voxel_figure_sources/`。射线采样研究另见该研究目录的 `ray_sampling.md` 和 `scan_geometry_audit.md`。这些是实际射线回放及实际 USD 几何审查，没有插入假射线或修改原始点云。

本机图：真实近身未知体素的三维分布及实体截面，文件 `/home/eric/wjg/d1max-build-isaac/unknown_research/v9_unknown_voxels.png`。

图中保留全部 3740 个原生查询体素；紫、黄、红三类仍是 UNKNOWN。黑线是实测机器人实体，虚线是原查询包络。逐体素分类和文件 SHA256 见 图的取证清单（本机证据：`/home/eric/wjg/d1max-build-isaac/unknown_research/v9_unknown_voxels.provenance.json`）。

## 算法层面的原因

本仓库严格模式要求每个与偏航双圆柱包络相交的体素都成为饱和 FREE，并同时满足源时间和真实接收时间期限。传感器只有离散射线：5° 垂直间隔在 0.5 m 和 1 m 处约对应 4.4 cm 和 8.7 cm 间距，与 5 cm 栅格同量级或更大；自体遮挡还产生长期盲区。加装更多雷达可以改善覆盖，却不能保证每个安全包络体素都被射线持续穿过。

同时，“当前没有回波”和“完整体积已证实为空”不同。未观察空间、弱证据空间、被遮挡空间和过期自由空间应保留不同证据来源。延长全部 FREE 有效期、调低占据阈值、降低机器人高度、关闭严格模式或给扫描重新打时间戳，都没有补足这种证据。

上游 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 的双圆柱模型与本仓库严格已观测 FREE 合同也要区分：[论文](https://arxiv.org/abs/2606.19555)描述基于三维占据地图的稀疏空间碰撞查询；本仓库额外要求实时射线证明每个查询体素。算法模型本身不能让传感器盲区变为已观测。

## 修复：认证静态体积与实时障碍融合

新增显式 `validated_static_prior` 模式，保留原默认模式。SCAN 的碰撞查询读取两个独立证据源：不可变的认证静态三维体积和原有真实射线栅格。先验不是假激光观测，没有 FREE 观测时间，也不进入原传感器计数。

代码入口：[GridMap 证据融合与 lease](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/grid_map.cpp)、[静态先验加载器](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/static_occupancy_prior.cpp)、[USD 完整体积生成](truth_map.py)、[轨迹查询取证](../../d1max_nav_ws/src/scan_planner_vendor/plan_manage/include/plan_manage/trajectory_collision.hpp)、[执行验证器](../../d1max_nav_ws/src/scan_planner_vendor/plan_manage/include/plan_manage/execution_validator.hpp)。基线原已有严格 FREE、精确双圆柱查询、整曲线与偏航扫掠验证及换轨事务；本次新增独立静态证据、稠密 raw 缓存和失败取证，修复源时间及任务进度等合同，不是另搭导航框架或新实现一套完整连续碰撞算法。

| 静态体积 | 实时证据 | 查询结果 |
| --- | --- | --- |
| OCCUPIED | 任意 | 阻断；射线不能清除静态墙体 |
| FREE | 从未观测 | 仅在先验授权和双雷达有效期内放行 |
| FREE | 真实旧 FREE 已过期 | 使用静态空体积证明，不刷新旧观测 |
| FREE | 合法的严格实时 FREE | 使用独立静态证书；不被更短的冗余观测期限截断 |
| FREE | 新 hit、弱证据或冲突 | 阻断，不能由先验覆盖 |
| UNKNOWN／地图外 | 未知／过期 | 继续阻断 |
| 任意 | 源时钟回退、无效接收时间或场景身份撤销 | 继续阻断 |

实时 hit 即使尚未把 log-odds 推至 OCCUPIED，也撤销该世界索引的先验 FREE。撤销记录跨滚动窗口保留，时间过期或离开窗口不会恢复自由；必须由更新源时间的真实 miss 和饱和 FREE 证据解除。同批 hit 优先于 miss。快照共享只读先验，独立复制实时撤销证据。

先验从实际 USD 碰撞几何生成。对整个闭合体素判断：任何静态实体相交或边界接触记为 OCCUPIED；只有完整体素在经验证的封闭空间内、与全部静态实体及不确定边界分离，才记为 FREE；其余 UNKNOWN。薄墙穿过体素中部也判占据，不能只检查八个角点。机器人实体不用于生成自由体积。

加载器检查 manifest/data SHA256、地图版本、坐标系、分辨率、完整维度、编码、数据路径和授权边界。会话绑定源地图版本、定位 epoch、seed、sequence 和时间屏障。目前可执行的坐标合同是经验证的 `map == odom == Isaac world`；定位产生非单位变换时必须撤销旧授权。

场景运行时再核对真实静态碰撞几何。新增、移动、旋转、缩放或移除非机器人碰撞物都会撤销认证；原桥接器禁止继续发布可用定位并发送零命令。偏航包络还绑定有限 roll/pitch。先验不能用于静态地图与实际世界失配的情况。两路实时扫描的源时间与真实接收时间仍受 0.5 s 期限约束，全部依赖先验的轨迹证明也有有限期限。

两个证明都合法时，选择独立静态证书，避免“旧激光 FREE 即将过期时缩短整条曲线证明、过期后却恢复静态证明”的期限跳变。原始实时 FREE 的源时间与接收时间仍必须非零且不在未来；弱证据和真实 hit 仍阻断。新证书使用新证明范围内的实际双雷达期限，新的扫描不能延长已经发出的旧证书。

计算优化缓存同一快照、同一证据范围内重复访问的原始体素判定。每个位置仍检查完整的原双圆柱与扫掠体积，缓存不扩大几何自由范围。真实 hit/miss 更新、地图滑动、上下文撤销、新证明、源期限和实际接收期限都会使对应缓存失效。历史轮式配置的独立运动／慢通道曲线／可选连续性预算分别为 25／120／30 ms；大型 Spot 配置的完整运动扫掠预算为 40 ms，各次封存配置的预算均保持。首次失败查询的位置、偏航和捕获的证据期限被记录，详细诊断在失败证明处理后执行，不用新的查询替换原阻断事实。

## 真值的正确用法与适用范围

仿真真值既用于生成完整空体积证据，也用于独立检查实际机器人运动、地图配准和场景不变性。控制仍由原 BT、PCT、SCAN、跟踪、安全门和唯一 writer 产生；真值不直接写速度、不判定任务成功、不绕过取消或停稳确认。

这份封闭场景证明不能直接带到实机。普通 PCD 表面点或 PCT 可通行地面不足以证明完整三维空体积；实机需要带已观测 FREE 的三维地图，或经过完整性与配准验证的 CAD/体积先验。没有这种证据时，默认严格未知阻断仍合理，应通过传感器布置、主动观测或重新规划解决。

历史轮型的原双圆柱查询从机身下方约 0.25 m 开始，其下界约为地面上 0.10 m，并不覆盖轮胎所有接触位置。直接把下界降到 0.05 m 会因闭体素边界触及占据地板而再次常驻阻断，清除地板体素又会漏掉同格低障碍。该轮式先验适用域因此限制为平地、实际机身高度误差不超过 5 mm，拒绝顶面低于地面上 0.15 m 的非地面实体；当时最低障碍为 0.24 m。3–12 cm 低障碍负测要求拒绝认证，不能称为已支持穿越。当前 Spot 的完整腿部体积与独立地面接触域另见下文，轮式限制不能直接作为机器狗配置或全身证明。

设计参考：[显式自由空间的多分辨率三维概率地图](https://arxiv.org/abs/2010.07929)、[D-Map 的占据与未知空间管理](https://renyunfan.cn/papers/2023tro_dmap.pdf)。这些方法支持区分地图状态与证据来源，不能被用来把没有证据的未知空间直接解释为自由。本次没有更换导航框架或接入其他局部规划器。

## 验证记录

旧 v9 的未知阻断失败证据保留，不重封装成成功记录。v10 加入认证静态体积后，原 SCAN 已产生轨迹，跟踪器接受了三条轨迹；实际执行却暴露 Unix epoch 浮点时间转换问题：同一源时间被误判为未来约 238 ns，writer 未应用非零运动。SDK、BT、SCAN 和跟踪器已改为保留原始整数纳秒，未来 1 ns 和过期 1 ns 的拒绝回归仍通过，没有扩大未来容差或给旧样本重新打时间戳。

v11 已实际收到 212 条非零 applied motion，实测行驶 0.0911 m，但仍因 `waiting_measured_motion_progress` 超时，不能称为导航到达。实时记录有 53 次曲线检查预算失败、24 次刷新预算失败及 25 次未知拒绝；非零控制只占运动阶段约 20%，最高实测速度为 0.04755 m/s。任务软件退役、同一执行身份的 writer 停稳以及 PhysX 实测静止均成立。独立真值审计的 2842 个采样姿态中，机身／脚轮精确几何和车轮保守包络都没有穿入静态障碍；这只是采样姿态审计，不能改变任务失败结论。报告见 v11 原任务结果（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_goal_v11_001/isaac_smoke_report.json`） 和 独立真值审计（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_goal_v11_001/physics/trajectory_audit.json`）。

同一批保存的真实三维射线、同一 493 个均匀直线位置查询中，第一次重复检查优化把中位数从 36.30 ms 降至 28.08 ms，所有 7395 个查询结果相同。该 CPU 回放使用固定取证时钟，仅用于性能比较，不是实时安全有效期证明，也不是 v10 的实际样条。v11 的实时结果表明仍需进一步减少同一证据范围内的重复体素判定；不能扩大原计算预算来把失败变成成功。

v13 将原始体素重复查询改为同一证明范围内的稠密索引缓存，加入缓存清空代数、溢出和快照内存边界测试。同一批 493 个查询重复 15 轮，7395 个 FREE 结果不变，中位数为 14.81 ms，P95 为 15.08 ms，最大值为 15.09 ms。原 v10 中位数为 36.30 ms。真实 v13 任务的曲线检查和刷新预算失败均为零；各通道的原计算预算保留。

v13 实际应用 326 条非零命令，行驶 0.3034 m，最高实测速度 0.1235 m/s，但未到达。跟踪器 50 Hz 的实际时间回调遇到重复仿真源时钟时，会发送零命令并清空速度历史；末条命令原因正是 `waiting_trajectory_clock`。修复后，合法重复 tick 不重新发命令、不推进序号、不刷新时间、不清空速度历史；原命令的实际时间期限仍从第一次发出计算，暂停、撤销、故障和源时间回退立即使它失效。相关原生负测保留。

取消确认另有两个独立问题。原生 ACK 与取消请求可来自同一个源 tick，适配器现以原始整数纳秒检查“不早于请求”，同时保留真实接收次序、严格新代数和所有工作节点退役条件，62 项测试通过。原 SCAN 发送方曾把同 tick 的第二条状态人工加 1 ns；这会被正确的未来样本检查拒绝。发送方已改为保留真实 `now()`，严格未来 1 ns 拒绝不变。v14 曾因 Python 复制安装未重建而运行旧模块；候选组装器现逐字节核对源快照与全部 171 个实际安装 Python 模块，拒绝缺失、残留或过期安装。

v16 的近距离到达和运动后取消分别通过：真实行驶 1.0025 m、0.2763 m，软件退役和实测静止均成立。其跨房间任务仍失败。原因是进度发布方把原始 float32 四元数归一化后写回证据，而任务层要求与同源测量逐项相等。现在数学计算使用规范化位姿，发布的 `RouteProgress` 保留原始位姿及源时间。v17 跨房间行驶 8.8182 m 并成功到达，3153 条进度与原始测量逐项相等；但全局锚点入口尚使用未归一化四元数，收到 7 次假锚点变化，最终 revision 为 1775。该数学入口已修复，原输入有效性限值保持。

### 弯道进度判定的独立算法缺陷

v18 保持固定锚点，2022 条进度全部与同源原始位姿逐项一致，实际行驶 4.5204 m 并穿过门口，仍因进度超时失败。独立回放发现，原任务监督要求累计真实里程至少等于固定路线投影的弧长增量。机器人位于弯道内侧或切过密集折线的内角时，投影弧长可以比实际里程增长更快；二者不能直接比较大小。

这次回放只在 sequence 89、123、141 确认进度。sequence 150 的路线增量为 0.032578 m，而真实行程为 0.028967 m，差 3.61 mm；之后 1869 条进度一直因同一条件拒绝，最终差 38.20 mm。撤销前最后 1 s 仍实测移动 0.1533 m，原 writer 仍应用有效命令。可复现回放与源文件哈希（本机证据：`/home/eric/wjg/d1max-build-isaac/unknown_research/v18_fixed_route_progress_replay.json`）保留了失败事实；回放明确是连续收到的投影会计检查，不冒充 BT 的精确回调调度复现。

修复使用最近一次合法进度的固定物理及路线 witness。沿不可变完整路线，从该 witness 的投影足点到当前足点取前向弦方向，将方向旋转至 odom；真实位移只取同源原始 odom 位姿的净差。每次确认进度同时要求：confirmed arc 增加至少 3 cm、当前 measured arc 增加至少 3 cm、真实净位移在该方向的有符号投影至少 3 cm。三项门槛分别判断，不要求路线弧长等于真实里程。

负位移不截成正数，过去的横移或闭合抖动不按累计路程攒进度，历史 confirmed 高峰不能替代当前 measured 增长。每次确认后一起更新两个 witness 并丢弃余量；同锚点的参考窗口更新不清零 witness，锚点或语义段变化只重建基准，不制造运动。原时间、位姿逐项匹配、帧、变换、速度、分支和门口合同仍先行检查，原 30 s 阻断超时不变。新增 13 项原生回归涵盖内侧圆弧、90° 内切、密集短边、横移、回退、抖动、旧高峰、定位校正及原始位移门槛。

2026-10-05 的验证记录中，六个原生导航包共有 809 个 XML 测试用例，全部通过；原 SCAN 四个无 ROS 回调程序有 156 项检查通过，SDK 的 14 个测试目标通过。修改过的 Python 单元测试为 281 通过、2 项原有跳过，仿真夹具 68 项通过；143 项参考／适配器合跑属于前者子集，不重复累加。这些回归结果不构成最新换轨任务通过。所有仿真结果保持 `physical_acceptance=false`，定位是显式 groundtruth fixture，不构成 Faster-LIO 或实机性能证明。


### 历史 v19 同版任务验证（2026-10-05）

2026-10-05，封存候选 `isaac-candidate-v19` 的三项实际原任务均通过；完整性清单 SHA256 为 `49c969180b0fa7b96dea83f6d5af8a063c299b68bcfc5fef61935eaf34291a1e`。

| 任务 | 原 Action | 实测行程 | 独立审计姿态数 |
| --- | --- | --- | --- |
| 近距离到达 | status 4，`measured_goal_reached` | 1.0245 m | 1127 |
| 跨门到达 | status 4，`measured_goal_reached` | 8.8118 m | 4233 |
| 实测行驶后取消 | status 5，`action_cancelled` | 0.2826 m | 1027 |

三次均由原任务层确认软件退役，writer 给出同一执行身份的实测停止，PhysX 测得静止，运行管理者正常关闭会话。6387 个离散姿态的独立机身／脚轮精确几何和车轮保守包络审计全部与静态障碍分离。跨门口 221 个整机投影采样中，开口余量下界最小为 0.2288 m；最小静态几何 SAT 分离下界分别为机身 0.4797 m、车轮保守包络 0.3997 m，不能当作精确最短距离或连续安全证书。

三次收到的 `RouteProgress` 分别为 344、3443、223 条，均与同源原始位姿七项逐值完全相等；各自锚点和 map geometry revision 在捕获记录中保持为 1。任务取证边界内的 IMU 引用全部匹配独立实际接收；近距离原始状态的 2 条启动前缀在首个 ROS IMU 接收之前，单独用实际 PhysX 日志核对，没有补造接收证据。

有效期拒绝仍保留：跨房间捕获到 16 条 `source_expired_during_check` 轨迹拒绝，另两条正运动证明在观察者收到时恰好达到原期限；接收时刻不能替代实际 writer 消费时刻，不能据此声称过期消费。近距离和取消捕获的证明均有效。近距离及取消没有 UNKNOWN 警告；跨房间保留 10 条泛化未知警告，缺少对应逐体素诊断，不强行归因为 hit、预算或传感器盲区。三次显式预算耗尽日志均为零。修复的是长期盲区造成的常驻阻断，合法的证据过期和未知拒绝继续生效。

历史 v19 验证清单（本机证据：`/home/eric/wjg/d1max-build-isaac/unknown_research/verification_prior_20261005.json`）绑定测试、候选、原始任务报告和独立审计的哈希。[启动与测试说明](README.md)给出默认入口；实际包含机器人、双三维雷达和原生 IMU 的场景已导出为 完整轮式场景 USD（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_cross_room_v19_001/physics/indoor_scene.usda`）。

## 大型 Spot 场景的新增排查（2026-10-06）

完整脚部体积从地面下 2 cm 覆盖到机身上方，不能沿用轮式机器人较短的垂直体积。初版 v23 约 47,472,644 次重复体素遍历使 2000 次查询耗时约 179 ms；有界列聚合仍逐项验证完整高度，重复查询复用同一证据代际，约 24 ms。命中、oracle 改变、滚动窗口和双时钟过期都会撤销缓存；未扩大原检查预算和证据期限。离线结果见 [列缓存验证](verification/20261006/scan_column_cache_offline_v24.json)。

v24 真实运行确认检查耗时降低，但地面接触格仍有 live hit 冲突。后续使用原生雷达的命中 prim 路径定位到采样相位：包装接口给出物理步 END 计数，回波实际对应 BEGIN 姿态。四秒机器狗行走探针中，错位 2 ms 的 END 位姿使地面看似偏移最大 17.679 mm；逐步实测 BEGIN 位姿把误差降到 6.127 µm，见 [相位证据](verification/20261006/spot_lidar_phase_probe.json)。修正保留 native 原始 END 时间和回波，另外绑定真实 BEGIN 姿态及源时间，避免改写测量去迎合地面。

接触状态 3 是完整静态几何认证的固定地面接触域，始终不是激光 FREE；任何非地面静态实体会使对应格为 OCCUPIED，动态可达体积先于接触域否决。仅在显式仿真合同中封存 0.2 mm 数值界限，并逐扫描用原生 floor prim 命中验证；界限不符或身份缺失终止本次物理会话。以上排查与旧轮式 generation 7→8 换轨问题分别取证，尚不能宣称六关卡通过。

v25 仍暴露前后雷达异步到达造成的混合扫描证明。Spot 配置启用 `projected_ray_exact_pair`：只有两个新扫描的真实 BEGIN 源时间完全相等才形成可用于证明的新一代地图。缺失一侧时仍允许真实 hit 否决，不能用单侧或旧扫描续期。v26 的记录中未再出现 UNKNOWN 阻断，但到达任务仍因实测进度不足失败，说明地图问题与物理执行问题须分别验证。

v27 加入步态实测波动的隔离可达模型后，反应扫掠保留 `.902 m` 各向中心可达范围及完整腿部高度，每次约检查 16.3 万体素。915/1453 次运动证明耗尽原预算，3394/5347 次曲线检查在结束时源证据过期；雷达最长实际接收间隔约 `.539 s`。此时诊断中的 UNKNOWN 对应原始未观测格和已经失效的 prior lease，静态接触域本身有效。这属于证据供应与完整查询性能问题，不能通过清空体素或延长租约消除。

另一个独立问题是跟踪器把整条 XYZ 曲线最小的 XY/XYZ 比例用于全程限速，并在加速度和制动距离处重复缩放。v27 的比例约 `.0136`，实际非零水平指令最大仅约 `.000447 m/s`；约 `.066 m` 累计姿态移动主要是站立波动，不是成功行走。Spot 现显式启用 `spatial_planar_braking_envelope`，必须先验证地面支撑，再按当前曲线区间的 XY/XYZ 比例换算水平速度与加速度；后向制动使用实际 XY 弧长，只缩放一次。原 XYZ 曲线、支撑检查、纯竖直区间的零水平权限与三维碰撞体积保留，默认实机模式不启用。离线结果见 [水平包络回归](verification/20261006/spatial_planar_braking_envelope_offline.json)；曲率仍属采样检查，其连续可行性缺项见 [独立审查](verification/20261006/spatial_planar_braking_envelope_review.json)。各版失败及原始记录哈希见 [大场景初测证据](verification/20261006/campus_initial_failures.json)。

v28 完整列批量查询保留旧版全部体素与第一个阻断结果：对照覆盖 161,856 个体素、6,744 列、24 层，冷查询 p95 为 3.577 ms，原 40 ms 预算与最终双时钟检查不变；见 [离线证据](verification/20261006/scan_column_batch_offline_v28.json)。按已认证地面支撑启用逐区间水平制动包络后，原 Action 已驱动真实关节迈步，最高应用指令约 `.1483 m/s`，累计实测行程 `.2818 m`，四脚实际碰撞球均有离地记录。然而任务仍失败：旋转后的世界外接矩形把实际查询形状之外的角落也计入 200,000 格资源上限，导致后续运动证明在查询前拒绝。

v29 将资源计数改为实际完整扫掠体素计数。每个实际查询格仍计入原 200,000 上限，保留完整反应可达范围和腿部高度；仅外接矩形中的形状外角落不再消耗体素预算。超限、计算预算耗尽与最终证据过期仍拒绝。见 [资源计数回归](verification/20261006/motion_actual_volume_budget_offline_v29.json)。该版近距离原 Action 通过，实测行程 `.8115 m`，见 [v29 历史实测](verification/20261006/campus_navigation_v29.json)；随后取消用例仍失败，不能把一次到达解释为整版全通过。

v29 取消会话的失败来自全身几何证书：旧实现旋转球体的局部外接盒，把真实球体之外的盒角也当作脚部最低点，误判超过原 2 cm 地面接触域。v30 使用实际完整 primitive 的支撑函数计算紧致世界 AABB：球体为矩阵行范数乘半径，胶囊加上变换后的轴线段，box 使用行绝对值和乘半尺寸；然后以世界 AABB 八角点保守覆盖整个实体。旋转、非均匀缩放与 shear 均保留，真实脚部越界、低障碍、水平和顶部越界仍拒绝。没有扩大支撑接触域，也没有清除占据体素。见 [实际失败姿态与几何回归](verification/20261006/primitive_world_bounds_offline_v30.json)。

历史 v30 候选的近距离原 Action 行程 `.827614 m` 后到达，取消关卡行程 `1.197793 m` 后退役并完成 `2.02 s` 源时间静止验证；仿真纯 Python 回归 232 项通过。该版 `crossing_blocker` 累计 `5.583955 m` 后仍因实测进度等待超时失败。上述结果说明近身地图证据、查询性能和短测物理执行分别得到验证，仍不证明动态避障、全部关卡、连续曲率可行性或实机验收。原始结果与哈希见 [v30 实测摘要](verification/20261006/campus_navigation_v30.json)。

历史 v30 横穿日志须区分合理动态否决与恢复缺陷：演员预测体积在多段时间内阻断查询，UNKNOWN 不是需要清空的地图盲区。约场景源时间 105 s 后原运动／轨迹证明恢复，但跟随和换轨恢复仍未完成；原 Action 在 phase 源时间 117.6 s 因等待实测进度失败，未触及 281 s 测试观察预算。指定遭遇证据 `observed=false`，实际演员 AABB 分离距离下界最低 2.3097 m；采样无相交不代表动态任务成功。没有以清空 UNKNOWN、缩小动态可达包络或延长原阻断期限掩盖该失败。

按 handoff 身份进一步区分验证通道后，104–115.74 s 日志中的 prepared candidate 运动证明 **445/445** 均为 `motion_check_budget_exhausted`，对应原 **5 ms** 准备通道；handoff 23–53 连续出现 deadline expired。当前轨迹的普通运动证明恢复，不能替代准备候选所需的独立证明。[逐事务取证](verification/20261006/crossing_failure_audit_v30.json)保留对应原始日志哈希。

v31 把准备通道的 5 ms 区分为预留量与执行上限：仍先检查当前命令、续期当前曲线、必要时再检查当前命令，最后才检查准备候选；候选可使用同一 **50 ms** 周期内尚未消耗的时间，最多为已有 motion 通道的 **25 ms** 上限。5 ms 预留、优先级、原始 source／receipt、grant／proof／地图截止时间与发布前序列屏障均保留；没有创建第二个验证周期。原生 ledger **39/39**、非 launch CTest **15/15** 通过，[离线调度验证](verification/20261006/prepared_lane_offline_v31.json)记录代码与安装哈希。v31 横穿实际有 **14 条准备运动证明通过、7 次成功换轨、零准备预算耗尽**，确认 v30 的准备通道饥饿已不再出现。

调度修复仍没有使 v31 横穿成功：累计行程 `.403687 m` 含无效进度运动，确认路线弧长最高约 `.112 m`。新候选入口改变前进方向和低速需求，持续有效跟随仍待解决；成功换轨保留当前输出与真实 writer 应用历史，没有无条件清零 ramp 的证据，也未确证控制死区或实测 twist 噪声是唯一原因。近距 `1.040435 m` 到达与取消 `1.047985 m / 2.018 s` 通过分别取证，不能覆盖横穿失败、其他四关缺项或旧轮式换轨问题，见 [v31 实测](verification/20261006/campus_navigation_v31.json)和 [失败分析](verification/20261006/crossing_failure_audit_v31.json)。


## v32 横穿失败与 v33 复测结果（2026-10-07）

v32 `campus_crossing_v32_001` 的原 Action 失败，累计 XY 行程 **0.05664909496 m**，软件退役成立。原始 owner 的 route progress credit 始终为 **0**，不能把姿态抖动、ACK 或累计里程当作路线进度。源时间 **22–34 s** 的 721 个实际状态中，高层 `vx` 的 min/median/max 为 `0/.003958091/.009858395 m/s`；内部 policy `vx` 为 `−.000881571/.012934382/.036528707 m/s`，零命令比例 **18.169%**，窗口实际净前进仅 **.015897274 m**。该段已在动态预测恢复之后，不能继续把微速停滞唯一归因为 UNKNOWN。IMU 精确匹配、全身证书及 measured stop 成立；原 mock 的 `physical_stop_confirmed=false`、`overall_clean_shutdown=false` 仍保留。[封存失败摘要](verification/20261007/campus_crossing_v32.json)绑定原报告、完整 trace 和 PhysX 测量哈希。

### 动态体积、移动入口与低级伺服的证据范围

原动态 oracle 用六秒可达球的外接立方体直接否决查询，球外的立方体角落会多阻断。v32 保留原六秒半径与外盒，以闭球和**完整闭 voxel AABB** 的最近距离做精确相交；球不相交只撤销动态否决，仍须原静态／实时 FREE 或合法 support contact 证据。旧 schema 继续使用完整 AABB；不完整、畸形 sphere 撤销整个 oracle。演员、上下文、原 source／steady `.20 s` 租约、六秒预测范围、未知优先级和预算保持。[针对性验证](verification/20261007/dynamic_reachable_sphere_v32.json)与 [v31 完整日志反事实](verification/20261007/v31_sphere_full_log_analysis.json)说明这项修复的边界：183 个已记录失败查询中，94 个完整 voxel cover 与球分离、87 个仍确定相交、2 个不确定；这不是对未保存完整 spline 的历史运行作全曲线放行证明。六秒总可达集合仍用于查询，没有按每个未来曲线时刻区分预测，仍可能较保守。

移动入口方向门处理另一个独立缺陷：旧 v31 curve 11 在已有正向 writer 输出时，引入反向的原始测量前缀，产生约 `−3.01045 rad` 转向请求。新门在原 heading threshold 下检查当前／候选需求：当前曲线能继续正向执行而候选超出阈值时，拒绝该候选并保留当前输出历史；静止对齐仍走原保护，不引入“正在移动”的新速度阈值。原 XYZ、`.05` join、source、deadline、lease 和碰撞规则保持。[166 项离线跟踪回归](verification/20261007/tracker_moving_entry_offline.json)只证明这一类方向连续性规则；旧日志未保存全部控制点，不能把合成反例称为原 curve 11 的精确重放。

旧低级 feedforward `v+.25*tanh(v/.005)*exp(−abs(v)/.15)` 在零点附近增益 **51**，正向约 `.015–.077 m/s` 区间非单调。新 `spot_monotone_measured_v2` 以 `u0=2v` 保证有限斜率、严格单调和零输入，保留实测 PI、饱和防 windup、原命令上限及 zero 的积分输出撤回；gain 2 是待标定候选，不是已知 policy 逆模型。官方 12 关节位置／速度、policy previous/current action、13 个实际 primitive 和 500 Hz 机身运动均保留。[组件 A/B](verification/20261007/spot_servo_ab.json)中，旧／新 mode 使用字面相同的 20 s park 历史与 `.075/.15` 直线 schedule，两者运动及原完整范数停车都通过，因此旧映射缺陷**不是已证实的唯一物理失败原因**。v2 两种历史下 `.075` 的停车后净前进分别为 `1.461269/1.441885 m`；但 `.01 m/s` 连续 20 s 无真实腾空脚步，zero 后净位移 **−.002891 m**，仍失败。组件成功不能覆盖 v32 高层主要落在微速域的导航失败；原 producer 的类型／序列化失败及明确离线重放说明保留在证据中。

### 完整 XYZ 曲线上的时间前视启动死锁

[control_audit.py](control_audit.py) 读取 v32 完整 **28,502 条 trace**，解析 21 个完整曲线身份，无冲突或未解析原始身体身份。1,816 个控制记录的实际控制点／knots 重算中，前视位置及速度 residual 均为 **0**。原 owner 的执行身份 credit 为 0；这与机身实际只发生微小站姿变化相符。[公开精确 curve 21 及空间前视回归](verification/20261007/tracker_spatial_control_lookahead_offline.json)保留原 wire geometry、完整 trace 哈希、反事实数值与 114 项 v2／57 项 core／12 项 audit 离线测试证据。完整本机产物位于 `/home/eric/wjg/d1max-build-isaac/control-audit-v32/`：`v32_summary.json`、`v32_selected_full_curves.json` 与 `v32_spatial_lookahead_counterfactual.json`。这些是离线控制重算，不是新导航成功记录。

curve 21 原持续时间为 **25.8009388191 s**，记录的实测投影参数 `u=.006008710643 s`。固定时间前视取 `u+.8=.806008710643 s`，仍处于从真实站高约 `.4824` 向旧名义 `.52` 过渡的入口；它不会因为墙钟已经过去而向曲线后部前进。该点三维速度和水平控制重算如下：

| 同一原始 curve 21 | 时间前视 | `.12 m` 原 XYZ 弧长前视反事实 |
| --- | ---: | ---: |
| 求值参数 s | `.806008710643` | `4.982758844670` |
| XYZ 速度 m/s | `[.003447131,−.000009601,.007912335]` | `[.054332194,.000079905,.000509090]` |
| 原位置反馈与 feedforward 的 world X 请求 m/s | `.004806222` | `.138247763` |
| 前进请求经过**同一原限速** m/s | 小于 `.034480319`，限速未触发 | `.034480319` |

原实际入口局部 cap **`.034480319 m/s`** 明显高于 `.004806222` 请求，因此该记录不是局部限速把本来正常的控制压成微速。闭环的问题是：投影进度依赖真实运动，而固定时间前视又持续请求无法启动稳定脚步的微速，真实进度无法把前视带出入口。旧 `.52 m` 名义高度差加重了入口的 Z 占比，实际原始 `vz` 也保留在曲线边界；不能用伪造零 Z 或删除曲线高度解决。

v33 新 `spatial_control_lookahead` 只把**参考求值点**改为原完整 XYZ 弧长 `s+.15×.8=s+.12 m` 对应的参数。实测 phase/progress、credit、曲线几何、局部制动／曲率 cap、source join、全体积碰撞、handoff 和 STOP 仍按原规则。该配置默认 `false`，新仿真四足 profile 需已验证 support reference 才显式启用；默认实机与旧 sealed candidate 保持原行为。上表 `.138185798 m/s` 的反事实前进请求仍必须受原 `.034480319` cap 限制，不设置最低高层速度，不用前视弧长冒充已经走过的进度。v33 已进行原 Action 端到端复测，结果 **failed**；空间前视本身生效，实际局部 cap 仍极低，详见下节。

### Cold stand 标定保留真实完整身体体积

三份实际 Spot 组件运行的最后 10 s cold park 各有 **5001 个 500 Hz 原样本**，真实 pose／velocity 字面一致。站高范围 **`.4807393253–.4807963669 m`**，中位数 **`.4807447493 m`**；旧 `.52` 与其相差 **39.255 mm**。新生成 profile 采用 `.481`，上方包络同步增为 `.589`，原名义规划顶部 `.52+.55=.481+.589=1.07 m` 保持。初始 spawn `.80`、实际 13 形体及变换、XYZ pose/twist、固定 `floor−.02` 下界、`.12 m` 高度检查、速度权限、STOP 和租约均不改变。旧 v31/v32 `.52/.55` 候选和原失败证据保持不可变。

步态真实 Z 并不固定：`.075` 记录为 `.474432–.539096 m`，`.15` 为 `.489067–.541954 m`。5300 组同源完整形体在原 `.55` 与新 `.589` above 下的离线 containment 均通过；这只证明已采样身体体积，没有提供新场景全程碰撞或未测姿态保证。原 200,000 cell 限额保持：用 v32 最大 6783 个 XY columns 与组件最大 29 个 Z 层算得 **196,707 cells**，只剩 1.65% 余量；更大 yaw／footprint、时限或证据过期仍须拒绝。该数字只是同 footprint 的算术重放，绝不是 v33 运行通过证明。[可复核标定 JSON](verification/20261007/spot_cold_stand_calibration.json)保存原 raw 哈希、统计窗口、计算代码、源影响与新候选重新封存要求。


### v33 的实际空间前视与剩余起步失败

v33 `campus_crossing_v33_001` 原 Action status **6**、结果 **failed**，累计 XY 行程 **`.013899628923 m`**，原因 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof`。全部 **1239 条**原 owner progress 观测的 credit 都为 **0**。配置实测确认 `body_reference_height=.481`、`above=.589`、`spatial_control_lookahead=true`；控制前视没有改写 measured arc 或造出路线进度。[公开摘要](verification/20261007/campus_crossing_v33.json)保存独立评估、原记录／安装 binary／构建与回归日志的真实哈希。

原 trace **line 26061**，source **`1791350080516246284 ns`** 的 curve 13 `tracking` 记录提供了新的直接反例：projected `u=.000813860510 s`，measured XYZ arc **`.000006021634 m`**；空间前视使用原弧长 **`.12 m`**，已经求值到 **`5.101382769973 s`**，前视速度 `vx=.059242400217 m/s`，但同一原局部水平 cap 为 **`.000125981212 m/s`**、局部加速度 cap **`.004199099747 m/s²`**，实际 XY speed **`.000094497535 m/s`**。这次已不再是固定 `.8 s` 前视困在入口：只修正参考求值点及名义高度，仍不能越过当前入口局部可行性／proof 的阻断。为什么 cap 仍极低、哪些实际时段受 collision/tracker proof 否决，须按完整 XYZ 几何与原 source 进一步审查；不在此宣布唯一根因，更不提高 minimum command 或放宽原约束。

Phase 冻结报告的全身认证为 **2214 组／零失败**；较晚最终 bridge status 为 **2295 组／零失败**，两者窗口不同，不相加。完整实际几何审计覆盖全部 **2300 个**保存的原始 trajectory source，最大源间隔 **20 ms**，静态／动态 AABB 重叠均为 0，演员遭遇证书 `observed=false`。这只说明已采样形体分离，不能把未发生遭遇解释为动态避让成功，也没有采样间连续碰撞证明。

IMU 原源时间匹配、身体高度检查及实测停车成立：最终 stop **211 个样本**，完整线速度 **`.007535815 m/s`**、角速度 **`.000992841 rad/s`**。原硬件／mock `physical_stop_confirmed=false`、软件退役成立及整体 `clean_shutdown=false` 均保留。v33 tracker 的 4 个 CTest 组与仿真 Python **254 项**回归通过，仍不能覆盖这次正式导航失败。六关中的其余四关未执行，v31 的近距／取消成功、v32 的旧失败及组件 A/B 界限继续分别记录。

## v34：速度 cap 恢复后的命令中断与性能审计（2026-10-07）

v34 `campus_crossing_v34_001` 原 Action status **6**、结果 **failed**，原因仍为 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof`。累计 XY 行程 **`.215596564502 m`**，原 owner 的 **1358 条**进度观测全部 credit 为 **0**；实际最大完整 XYZ 速度 **`.097849592 m/s`**。累计行程包括站姿运动与反复起停，不能替代固定路线进度或终点到达。[公开 v34 摘要](verification/20261007/campus_crossing_v34.json)绑定原失败报告、完整 37,472 条 trace、18 个完整曲线身份、实测形体、独立评估、控制链审计与组件计时的原始哈希。

新 `manager.fit_low_speed_entry_velocity` 默认为 `false`，仿真四足显式开启。它只在 guided 原参考、合法有限 support floor、原 measured source／yaw gate、incoming 和 measured 的**完整 XYZ**速度均不超过原 `.03 m/s` 且 incoming acceleration 每项严格为零时，把局部 solver 的入口速度／加速度取为零。原初始 XYZ、未来参考 Z、raw pose／twist／source、原 TaggedBspline join witness、`.05 m/s` C1 adoption、全曲线碰撞、command cap、motion sweep 和租约不变；这不是静止证明，也不设高层最小速度。[10 项拟合回归](verification/20261007/low_speed_entry_fit_offline.json)保留低 XY／高 Z 拒绝、原 source 更改拒绝、完整 XYZ 几何及原始 `vz` adoption 等反例。

这次修正确实改变了入口可行性：原 trace **line 674** 的 curve 1，source **`1791351262291382391 ns`**，空间前视仍为原 XYZ 弧长 `.12 m`，局部速度 cap **`.148315661690 m/s`**、加速度 cap **`.346069877278 m/s²`**。该记录的原 reason 是 `geometry_only_no_motion_permission`，高 cap 本身不能赋予运动权限。全程 **87 段**实测非零 writer 输出的源时间跨度 min/median/max 为 **`0/.040/.140 s`**；这是按原 publisher 采样划分的连续片段，不证明采样间全部独立 gates 有效。场景源时间 22–30 s 的 42 段片段中位数 `.040 s`、最长 `.100 s`。原 tracker 中 `permission_or_native_proof_expired` 有 **1423 条**、`source_clock_paused_command_expired` **27 条**；latest negative／过期 native proof 在多次 zero demand／owner hold 之前出现。30 次 handoff grant 中成功 12 次，18 次拒绝含 11 次原 deadline expired；初始 writer ACK 另计。这些直接证据支持继续修复证明更新与实时计算，不能把旧微速映射或步态 C1 不匹配当作这次失败的唯一成因。

SCAN 的 lease 取证必须区分最后一个 query 与完整 check 结束：203 个捕获里 22 次 ray receipt 已到／超过原 **500 ms**；其余 179 次最后 query 尚有正余量，其中有仅 **`.002278 ms`** 等极小值随后在完成前过期，不能解释为期限判断错误。源时间 22 s 以后 10 个捕获已到 500 ms。原 motion proof source lease 多为 `.100 s`，dynamic source／receipt `.20 s`、ray source／receipt、map／body 与 whole-body 期限继续保持；重复发布 wire-identical 证据不能续期。几何采样／枚举缓存可以研究，但最新 negative、source／receipt 身份和原停止屏障必须重新检查。

原 v34 的 **38.002 s 源时间**用了 **154.522857 s wall**，真实时间比 **`.245931`**。双点云 wall 最大间隔 **`.598546/.594873 s`**，已经超过原 ray receipt `.500 s`，而 raw source 时间、native 雷达频率和真实坐标均保留。计时如下：

| 原 v34 测量区间 | 次数 | 总 wall s | 说明 |
| --- | ---: | ---: | --- |
| 两个原 `.002 s` physics step | 9505 | `128.224905` | 包含 native 物理、Python 回调及 10 Hz 实际 BEGIN witness，不是纯 solver |
| 主循环实际 state／UDP／证据 | 1901 | `16.273990` | 50 Hz source；额外 381 次 BEGIN witness 计在上行 |
| 双雷达过滤／打包／UDP | 381 | `7.638985` | 762 个原 native 单传感器帧 |
| 其中 exact self／depth filter | 762 | `7.254209` | 同时包含 native hit-path 拷贝、floor audit 与 13 primitive 检查，未细分 |

相同封存 v34 场景的独立 **5000 步**组件 profile 不运行 ROS 或正式导航，**9.982 s source／39.295210 s wall**，RTF **`.254026`**。主循环物理区间 `33.164410 s`；计得 policy `1.677743 s`（含额外 1000 步 settle）、actor 更新 `2.165217 s`、expiry `.239584 s`、LiDAR gate＋BEGIN witness `1.321461 s`、IMU `.333937 s`。直接相减得到 native／未拆分区间的保守下界 **`27.426467 s`**；policy 的 settle 不在主循环 timer 中，因此不能声称这个余量是精确 solver 耗时。组件 profile 也出现半秒以上点云 wall 间隔，说明仅关闭 ROS 竞争并不足以恢复实时率。首次未封存场景试跑缺少 runtime registry，报 `KeyError:'wheel_radius'`，保留为失败尝试，不混入性能基线。

静态 stage verifier 的**显式 `verify()`**已有 notices/cache，2282 次调用总计 **`.020903 s`**，一次 full audit 和 2281 cache hit；这个计数不含 USD 的 `ObjectsChanged` 监听。后续 `physics_profile_notices` 在 10 s source 中计得 **10,902 次通知／9.793185 s wall**，RTF `.253050`，确认监听开销不可忽略。通知 callback 嵌套在 physics、actor 等区间里，不可再与这些外层计时相加，也不能把未拆分余量全称为 native solver。Render calls 为 0、fabric 未启用。官方 lidar `pause/resume` 只改 Python 布尔，重复调用可以省去但不会据此解释大额 native 余量。USD 的 `enabled` schema 标称可禁用传感器，但本机 native 插件只有二进制；wrapper frame 每步自增不能证明非采集步真的没有 raycast。后续须拆分实际区间或使用经过验证的 native 计数，不减小双雷达 11880 rays／帧或改 source stamp。

官方 [Isaac 6.0.1 性能手册](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html#physics-simulation-optimizations)指出小型 CPU physics workload 可对照 `/persistent/physics/numThreads=0`，使计算在主线程同步执行。Kit 日志的 carb／TBB 24 线程不能当成原 PhysX 实际线程数；实测 readback 的原值为 8。对照必须保持 **500 Hz 物理、50 Hz 官方 policy、双 native 3D LiDAR 10 Hz、native IMU 500 Hz／原输出 100 Hz、50 Hz state＋10 Hz 同源 BEGIN witness、全部 13 形体和 raw XYZ**。本次 `cpu0` 的 RTF `.278791` 略高，但最长点云 wall 间隔 **`.719476 s`**，仍超过原 `.500 s` receipt，未选用；一个组件实验不能推导普遍线程最优值。线程实验只改变调度，不扩大 cap、lease 或 STOP 权限；没有 v35 正式导航通过结论。

### USD 写回加速实验：步态与雷达通过，IMU 姿态失败

直接关闭 USD state writeback 的 `physics_profile_tensor_only` 在 **10 步**后被原 `native_floor_endpoint_error_exceeds_sealed_bound` 拒绝，未产生完整扫描；该失败保留。随后只在每次 native LiDAR BEGIN 前调用原生 `PhysX.update_transformations(False,True,False)`，发布已完成的真实物理姿态，未插值或写入期望 root pose。`.075 m/s`、10 s source 的 `capture_writeback` RTF 为 **`.373829`**；原已登记的测量属性在 disable 前后不一致，触发 100 次完整审计。将 disable 放到原 settle 与属性登记之后的 `enrolled_writeback` 降到一次完整审计，RTF **`.420805`**。这两次是组件实验，下面的独立检查确认它们均没有保持全部传感器语义。[完整审计 JSON](verification/20261007/physics_performance_writeback_audit.json)保存包括后续 velocity-only 对照在内的 57 个 raw 哈希和可复核计算代码。

两次 walking run 的 **600 行实际 trajectory 字面相同**，每行与同源 13 个实际 primitive 的 pose 一致。身体相对首个记录的实际 XYZ 位移为 `[.774506,.113028,.038256] m`，真实 Z 范围 `.480682–.538624 m`；12 个关节都有实际变化，四脚球最大离地分别为 **`.081225/.069133/.071824/.054920 m`**，各有 11–12 次采样 air/contact 转换（10 mm 只用于描述，不是新导航阈值）。200 个实际雷达帧审计 **499,003 个 native floor hit**，最大误差 **`5.024198 μm`** 小于原 `.2 mm`；原 BEGIN／END 差 **`2,000,001–2,000,471 ns`**保留原 2 ms 物理步与 native 浮点时钟误差。真实步态、全身和地板射线通过不证明导航通过，也不能代替 IMU 检查。

关键反例是原生 IMU：native read 仍为 **5000 个 500 Hz**，输出仍为 **1001 个约 100 Hz**，原 source stamp 未改；但两次 walking run 的 orientation 都仅有 **101 种值**、100 次更新，发生在约 `20/120/220…ms`，与 10 Hz USD 写回一致。在有实际 state 的 **500 个相同 physics tick**上，IMU 世界姿态与 raw body 姿态的角误差 median/max 为 **`.001727257/.021671505 rad`**，按真实 body 姿态换算的 IMU gyro 误差最大 **`.006422587 rad/s`**。原 park notices profile 的 orientation 有 1001 种值，同一计算的姿态 max 为 `.000041269 rad`、gyro max 为 `2.913×10⁻⁸ rad/s`；park 与 walking 不是相同命令的策略 A/B，但陈旧姿态的更新时序在 walking 数据中直接可见。配对只用唯一最近 2 ms tick 审计 native 浮点 stamp，最大余差 458 ns；没有改写任何 published timestamp、插值或用 body 真值覆盖 IMU。

本机官方 `IImuSensor.h` 明确速度读自 `IPrimDataReader`，sensor 世界变换取自 `Pose.h`，gyro／acceleration 在局部 sensor frame、orientation 在世界 frame。这与读数时间继续更新而 sensor 方向停留于陈旧 USD 的观测相符；完整 native 实现仍为二进制。因此 **`.420805` 不能作为保持原 IMU 语义的合格加速方案**。后续必须保留原每个物理 substep 的实际 sensor 变换，或者先验证官方支持的原生姿态读取路径；不能用真实机身 quaternion 替换传感器数据、降频、重 stamp、放宽 receipt 或忽略 IMU。原 v34 失败、这些被拒绝的组件实验及未来正式关卡继续分开取证。

最后的 `physics_profile_no_velocity` 保留原 **500 Hz pose writeback／`UPDATE_TO_USD=true`**，只关闭 USD velocity 副本；所有实际 velocity API 继续读 tensor／native，原 PhysX 线程数为 8。RTF **`.413157`**，最长点云 wall 间隔 **`.451119 s`**，notice handler 降为 `3.263238 s`。600 行完整身体、12 joint／policy 记录及 600 行 13 primitive 快照，与前面 walking run **字面完全相同**；native IMU 5000／输出 1001、零 invalid／duplicate、原 source ns 数组字面相同，orientation 恢复为 **1001 种值／1000 次更新**。

IMU phase 须按原生语义核对：100 个已保存的实际 BEGIN（`18+100n ms`）与下一 native IMU END（`20+100n ms`）世界姿态角误差 **median 0／max `5.162×10⁻⁸ rad`**。在 500 个相同 physical END tick，用原 IMU 自己报告的 frame 旋转 raw world angular velocity，gyro 误差最大 **`1.693×10⁻⁸ rad/s`**。若改用 post-step body END quaternion，则会看见原 **2 ms** 姿态阶段差，walking 中最大 `.001203 rad`；这与先前持续 **100 ms** 的陈旧方向不同，不能通过改 IMU 时间戳或 quaternion 消除。100 个捕获 tick 的原 orientation／gyro 与 capture run 字面相同，间隔内姿态恢复实际更新；完整 floor／body 检查继续通过。这项 velocity-only 组件通过上述原 source／phase／真实步态检查；随后封存的 v35 原 Action 复测仍失败，结果在下节记录，组件通过不能代替动态导航验收。

Phase 全身证书 **2189 组／零失败**、较晚 bridge **2277 组／零失败**分别保存。采样碰撞审计覆盖全部 **2282 个**原 trajectory source，最大源间隔 20 ms，静态／动态 AABB 重叠为零、指定遭遇未发生；没有采样间连续碰撞证明。实测 stop **210 个样本**，完整线／角速度 **`.007455075 m/s / .000796454 rad/s`**，IMU 原源匹配、软件退役成立；原 `physical_stop_confirmed=false`、`clean_shutdown=false` 保留。16 个 SCAN CTest 组、4 个 tracker CTest 组和 254 项 Python 回归通过，不覆盖正式任务失败或剩余四关缺项。


## v35：真实路线进度出现后的剩余阻断（2026-10-07）

v35 `campus_crossing_v35_001` 原 Action 仍为 **status 6／failed**，最终原因 **`execution_blocked_timeout:actual_command_blocked:motion_sweep_occupied`**。累计 XY 行程 **`5.706248792438 m`**，实际完整 XYZ 速度最高 **`.489059597 m/s`**；原 owner 最终记录 **157 次 credit**，确认路线弧长 **`5.285179166692 m`**、实测投影 **`5.277163081212 m`**，目标误差 **`10.643651074570 m`**。实际路线进度已出现，但累计行程、步态或部分换轨不能代替原目标成功与指定动态遭遇。Action 实际观察区间 **110.56 s source**，281 s 仅为封存测试观察预算。原失败报告、评估、全部 100,541 条 trace／62 个完整曲线及实际传感器哈希见 [v35 公开报告](verification/20261007/campus_crossing_v35.json)。本机选择器仍为 v31，旧 `.52/.55 m` 封存配置和 v32–v34 失败证据保持。

同一完整原 trace 的 7456 个可解析控制样本，原 lookahead 的完整 XYZ 位置／速度重算残差均为 **0**，没有缺少原 body 或冲突曲线身份。最高局部 cap 仍为 `.148315662 m/s`，该首次样本 reason 为 `geometry_only_no_motion_permission`，所以 cap 仍不能替代授权。原 writer 的 **30 段**连续非零输出按 `applied.source_stamp_ns` 首末样本跨度 min／median／max 为 **`0/.510/8.700 s`**；按 trace 的观测 `source_clock_ns` 则是 **`.020/.500/8.700 s`**。报告同时保留逐段原 ns 与两种边界，跨 trajectory ID 的连续非零输出仍算同一段；这只是实际发布状态的采样跨度，不证明两个样本之间全部权限门持续有效。最长段实际跨 curve 16→19，不能把成功换轨一律解释为无条件清零。

**65 个精确原 handoff ID 中 7 次成功、58 次未成功。** 原拒绝分别为 45 次 `handoff_deadline_expired`、9 次 `owner_handoff_withdrawn:waiting_writer_handoff_outcome`、3 次 `owner_hold_or_phase_barrier`、1 次最终任务 timeout；初始 writer ACK 另有 1 次，总成功 commit 为 8。重发 ACK 按同一 handoff ID 合并；未用重发次数冒充换轨次数。Tracker trace 中仍有 3227 条 `permission_or_native_proof_expired`、664 条 `turn_first_decelerating` 与 29 条 `source_clock_paused_command_expired` 观测；这些 reason 是不同门在不同时间的事实，不能从一个最高速度或 C1 样本推导唯一根因。

v35 同时修复了 exact-pair watchdog 的首包等待和 USD velocity 写回开销。新到首包按自身原 receipt 获得原 **200 ms** 配对机会，保留原 **250 ms** pending drop／**500 ms** evidence deadline，老 orphan 到期时不能消耗另一个刚来的未匹配包；缺 peer 的真实 hit 仍会撤销成对 FREE。性能配置只关 USD velocity 副本，实际 **500 Hz pose 写回仍 true**、PhysX 原线程 readback 8、50 Hz 官方 policy／state、双 10 Hz 三维 native LiDAR、native500→输出100 Hz IMU 和 10 Hz 真实 BEGIN witness 均保持。两项并行变化后的 v35 相对 v34 进度改善不是单一变量的因果 A/B，不能声称某一修改已独自根治走停。

正式 physics 的 **115.142 s source／333.886827 s wall**，RTF **`.344853`**，双雷达 wall 最大间隔 **`.472398/.470960 s`**，均低于原 `.500 s` receipt；仍不能据此推导消费端每次查询／handoff deadline 都有效。28,790 次两步 physics 总 wall **256.686870 s**，5758 次 state／UDP **47.794424 s**，1152 次双点云过滤／打包／UDP **22.715540 s**；内部 ray self/depth 等区间 **21.609787 s** 与双点云外层嵌套，不能相加。静态显式 verify **`.032925 s`**（full audit 1／cache 6909）不含 USD listener；该正式 run 未启用 notice 细分计时，不能把组件 notice 数值填入正式记录。

实际双 native LiDAR 各 **1152 帧**，2304 个 floor 审计中 **5,712,883 hit** 最大误差 **`3.929568 μm`**，小于原 **200 μm**。57,580 个 native IMU／11,517 个输出的原源频率为 **`499.999999/100.001737 Hz`**，invalid／duplicate 均 0，全部 11,517 个输出方向值不同。独立 raw phase 核对保留原 native stamp：673 对实际 BEGIN→下一原 IMU END 的方向误差 median 0／max **`6.664×10⁻⁸ rad`**；3820 对同 tick 机身 raw world omega 按 **原 IMU 自己的 frame** 旋转后，gyro 误差 max **`2.515×10⁻⁸ rad/s`**。这些是实际保存并匹配到的子集，不冒称所有 500 Hz native 读数都存盘。唯一最近 2 ms tick 仅用于离线配对，native 浮点 stamp 最大余差 3784 ns；雷达原 BEGIN／END 差 `2,000,001–2,005,468 ns` 保留，没有重 stamp、插值或替代传感器方向。

Phase body **6795**、较晚 bridge **6907**、最终 physics **6910** 组证书均零失败，实际 Z 范围 **`.479957–.542977 m`**，完整注册的 13 形体、`.481/.589 m` 参考与原 −.02 m 授权 floor slab 保持。全部 **6910 个**原 trajectory source 的采样静态／动态 AABB 重叠为 0，最大源间隔 20 ms；指定 actor 遭遇仍 **`observed=false`**，最小演员 AABB 分离下界 `1.926590 m` 也不能代替同一形状中心的接近—停留—离开证书。Stop **211 个样本**、完整线／角速度 **`.007447834 m/s / .000216747 rad/s`**，IMU 同源与软件退役成立；原 `physical_stop_confirmed=false`、`clean_shutdown=false` 保留。8 个 plan_env CTest 组（projected rays 102 项）、17 个 SCAN CTest 组和 254 项 Python 已有回归通过，不覆盖原任务失败或其余四关未执行。

### 末次 UNKNOWN：有效租约下的原六秒可达球

[原字节捕获](verification/20261007/v35_live_unknown_bounded_capture.json)保存 source 约 **105.163163 s** 的 curve 57 查询和 **105.140000 s** 的实际演员样本。双射线源相等、静态身份有效、动态 oracle 有效，原 receipt／source 剩余分别 **172.496／176.837 ms**；不能将它归为过期 prior。首个 voxel `[-.85,-4.45,-.05]→[-.8,-4.4,0]` 没有 live hit source conflict，实际演员中心 `[0,3.589672,0]` 的原六秒 sphere 半径 **`8.030984581 m`**，完整闭体素到中心最近距离 **`8.029623432 m`**，因此确实相交。原算法将整个 horizon 的可达区域保守合并为 UNKNOWN，这次否决符合当前合同；静态 FREE 不能覆盖有效动态可达否决。

该球包含允许演员未来可达的位置，不能把它当作当前真实演员身体，也不能将此曲线的 UNKNOWN 与最终 current-command `motion_sweep_occupied` 混成同一证据。后续需要基于完整原命令扫掠、曲线时间与演员有界预测逐段分析保守性、权限寿命和换轨拒绝；不能清 UNKNOWN、缩真实身体、删 Z、延 lease 或放宽 STOP／C1 阈值使任务“通过”。现有材料还未给出唯一根因或系统动态导航成功证明。


## v36：实际遭遇与原任务失败（2026-10-07）

原 `campus_crossing_v36_001` 的 Action 为 **status 6／failed**，原因 **`execution_blocked_timeout:waiting_current_collision_and_tracker_proof`**；独立原 `scenario_suite.py evaluate` 同样为 failed。累计 XY **`7.221876985931 m`**、raw 机身净 XY 位移 **`6.641737666187 m`**，原 owner credit **199 次**、最终确认路线弧长 **`6.643419891298 m`**／实测投影 **`6.635886281944 m`**，终点 `[-1.284970,-4.935242,.488786]` 仍距目标 **`9.285195868369 m`**。原 trace 共 **75,820 条**、142 个完整曲线身份，没有曲线身份或原 body source 冲突；5101 个可解析控制样本的原完整 XYZ 前视位置／速度重算残差为 0。七个路线／锚点证据域分别保留，不把累计行程或诊断观测数改写成 owner 进度。原配置 `281 s` 是 phase 源时间观察预算，实际 phase 观察 **86.542 s source**、phase bridge 场景源时间 **89.4 s**，不是行走时长或 wall；较晚 bridge 91.1 s、最终 raw BEGIN 91.28 s 的不同窗口保留。[正式失败报告、重算代码与原始哈希](verification/20261007/campus_crossing_v36.json)绑定原 Action、候选、完整 trace 和实际身体／传感器流。

**三个改动均保留完整三维与原保护。** [完整 XYZ reference/source-travel 域](verification/20261007/full_xyz_reference_domain_offline_v36.json)将 `.15 m/s` guide／command 与隔离 Spot `.50 m/s` native reference／实测源时间移动拒绝域区分，完整原 boundary、优化、fixed-BC timing、最终 derivative、C1 和 writer ACK travel 仍按原 XYZ 处理；超过 `.50` 的实测值拒绝，默认实机域不变。正式实测最高 `.499628686 m/s` 是原步态样本，不是未来速度保证；没有强加最低指令或删除 Z。[封存脚本演员完整未来体积](verification/20261007/scripted_actor_future_v36_offline.json)只对原源时间强制执行的封存 C1 script，枚举原六秒 horizon 的两端、waypoint、loop 与停车边界，完整形体及 XYZ／倾斜 padding 保持；任意非脚本演员仍用原 full XYZ reachable sphere。与 `.20 s` source／receipt、`.30 s` message TTL、原真实 same-step 和 stage ancestor guards 分别核对，不把 TTL 缩成预测 horizon。

[Native actor HIT 身份](verification/20261007/native_actor_hit_provenance_v36_offline.json)在仿真 opt-in 72-byte ray 中保留实际端点与原时间，追加严格排序注册 ordinal；未知、静态、未匹配路径为 0。它不清 raw log odds 或 hit history。只有已有认证静态 FREE、完整未混入零／其他演员的同一非零 actor 真实 HIT 历史、严格更新且未过期的完整 oracle，及整个闭 XYZ voxel 与该 actor 全部区域分离，才允许选择原静态 FREE 证据；混合、缺失、过期、同源／更老 oracle 和资源越界仍否决。v36 原 scan 日志中的未知／占据否决及最终 current proof 等待保持，不能从离线测试成功推断标签生产者已实测正确。实际 first NPZ 未含 ordinal，last 双帧共 `8390/8375` 点的 ordinal 全为 0；四个 NPZ 均未保存原 native hit prim paths。**这限制了回溯归因，需独立真实组件探针核对实际标签来源；不得回填旧标签或直接宣布唯一 label bug。**

原 writer 按与 v35 相同的连续非零定义有 **58 段**，原 applied source 首末跨度 min／median／max **`0/.570/5.560 s`**；观测 source clock 对应 **`0/.560/5.560 s`**。跨 trajectory ID 仍连续算一段。这是已发布状态的采样跨度，不能证明两点间权限持续有效。另按相邻 SDK applied 的 vx／wz 符号分段为 60 个正 vx、2 个 rotate、59 个 hold、零 reverse；该口径把正 vx↔rotate 分开，故不与 58 段相混。27 个精确 handoff 中 **13 成功／14 未成功**（11 deadline、2 owner withdrawal、1 owner hold/barrier），初始 ACK 另 1 次，原 28 个 ACK 不冒充 28 次成功换轨。2881 个非零原状态的 safety_checked 与 permit／curve／motion sequence 均有效且正值，vx 最大 `.148315662`、wz 最大 `.30`，command 原上限保持；这仍不是未来续租授权。

最终停止有清晰的原序列：scene source 约 **55.34 s** 首个 native motion negative 后 SDK zero，随后持续 hold 到 **89.44 s**（原 source 跨度 **34.10 s**，steady 观察跨度约 **100.55 s**）；后续当前曲线 UNKNOWN、owner withdraw／barrier 以及占据否决是不同门的后继事实。全程还出现 source clock pause 导致 command expiry、缺扫掠证明、segment finish 和 permission expiry。原负证明被遵守，不能用 `.15` cap 或有时匹配的 C1 替代碰撞证明，也不能把所有 hold 唯一归为 UNKNOWN、servo 或 C1。连续控制与当前 proof／handoff 的完整链仍需修复，**导航仍不丝滑且未到达**。

**指定动态遭遇这次实际发生。** 原 `/World/Spot/hl_uleg/collisions/mesh_0` 与 `plaza_person/body` 的同一中心对，在场景源时间 **61.018 s** 接近、**62.018–64.280 s** 保持 near、**64.560 s** 离开；137 个 near 样本，最近中心距离 **`1.652737838 m`**。全部 5477 个原 trajectory source 的完整实际形体采样审计无 static／dynamic AABB 重叠，演员分离距离下界最低 **`.568772262 m`**。全 5477 个实测 actor root XYZ 与原封存 source-time script 的最大差 **`.476568 μm`**，小于原 `.0001 m` 合同。这证明规定的采样接近—离开和分离，**不证明行人实际挡住路线、成功让行／绕行、连续碰撞自由或接触验收**。

Phase 全身证书 **5364**、较晚 bridge **5466**、最终 physics **5477** 组零失败，窗口分开。独立全流检查所有 body／13-link／actor 原 tick 一致、实际 body/link pose 字面一致；完整仿射 primitive AABB 重算残差为 0，原候选 envelope 全部通过。真实 raw Z 范围 **`.480028–.542953 m`**，`.481/.589 m` 与固定授权 floor `−.02 m` 保持。12 个实际关节角度／速度和 policy action 均保留；四脚实际 solid 最低 Z 高于 `.01 m` 分别有 **1064／1247／1236／1133** 个采样，不能将实际行走改写成 root teleport 或真值伪造。

正式性能为 **91.262 s source／269.324661 s wall**、RTF **`.338855`**；22,820 次两步 physics wall 合计 **201.114934 s**、4564 次 state／UDP **37.422002 s**、913 次双点云过滤／打包／UDP **25.587337 s**。内部 self/depth 过滤 **24.633312 s** 属于双点云外层，不能重复相加。双雷达 wall 最大间隔 **`.516322/.516786 s`** 已跨原 `.500 s` receipt，因此源／receipt 和短 command lease 的真实过期仍需按实际查询核对，不能延 lease 来消除否决。正式未启用 callback／notice profiling，两项计时为空；显式 static verify **`.031695 s`** 不含 USD listener，不能用它宣称全部静态监听成本可忽略。原 500 Hz pose 写回、线程 readback 8、只关 velocity 副本保持。

双 native LiDAR 各 **913 帧**，1826 个 floor 记录中 **4,533,723 hit** 最大误差 **`4.852557 μm`**，小于原 **200 μm**。45,640 个 native IMU、9129 个原输出的频率约 **500／100 Hz**，invalid／duplicate 为 0，每次输出方向变化。577 对实际 BEGIN→下一原 native IMU END 的姿态误差最大 **`8.941×10⁻⁸ rad`**；3151 对同 tick gyro 按原 IMU frame 核对，最大 **`3.023×10⁻⁸ rad/s`**。唯一最近 2 ms tick 仅作离线关联，原 native f32 stamp 最大余差 3784 ns，原雷达 BEGIN／END 差 **`2,000,001–2,004,333 ns`** 未改。原 owner stop 213 stationary samples 当前全线／角速度 **`.007401856 m/s / .000154388 rad/s`**；另原 2.02 s fence 的 122 条 raw PhysX 记录完整速度最大 **`.007494982 / .001371126`**，均满足原 `.03/.05` 阈值，最大源间隔20ms。IMU 同源和软件退役成立，原 `physical_stop_confirmed=false`、`clean_shutdown=false` 及 Kit `−15` 退出码保留。

[v36 构建及离线回归](verification/20261007/v36_runtime_build_offline.json)保存最终完整 ABI 安装和 58 个 CTest 组／945 个实际 XML 用例零失败、修改范围 Python **347 通过**；原 broad **17 失败／2336 通过／3 跳过**因缺失默认外部 map 首先阻断，未触达更深断言。原 compiler-cache／prefix 与 Humble header-export 两次失败日志和修复范围不删除。该报告在候选封存后加入仓库，**不声称已嵌入 v36 candidate**。所有这些离线结果不覆盖正式 FAILED。上游算法取舍见 [固定 SHA 的 PCT／SCAN／集成／Robot-Nav 原源码审查](UPSTREAM_DESIGN_REVIEW.md)：学习完整局部窗口、源身份、稳定投影及 heading 处理，同时保留本项目完整3D、原 HIT 与严格 proof。当前 ground→body reference 只加一次 `.481 m`，无需照搬旧上游 height offset、去 Z 或最低速度策略。


## v37：连续实际指令下的机器狗站立（2026-10-07）

新版 sealed `campus_crossing_v37_001` 原 Action **status 6／failed**，原因 **`execution_blocked_timeout:waiting_measured_motion_progress`**。独立原 evaluator同样failed；原source观察39.32s，281s只是配置观察预算。累计XY **`.471878535030 m`**、raw body净XY **`.232169987210 m`**，原owner **8次credit**，确认路线 **`.302688736880 m`**／实测投影`.302243793423 m`，终点距目标 **`15.695410068286 m`**，指定演员遭遇未发生。软件退役成立，原hardwarephysical stop／clean_shutdownfalse保留。[v37 原失败、原始哈希、重算代码与精确窗口](verification/20261007/campus_crossing_v37.json)不覆盖v36及更早失败。

**先区分已确证的native身份缺陷与当前主要失败。** [独立native rigid-body-root组件](verification/20261007/native_root_hit_v37_component.json)在原封存演员脚本和真零command下保存200帧原hit prim身份：旧mapper只接受collider leaf，但API实际返回整个已注册actor rigid-body root，全部原actor root hit因此标0。离线只改精确身份映射的比较证实这项遗漏，未知／静态／未注册child／prefix alias和模糊registry仍拒绝；没有用附近XYZ或命名猜测来标演员。新candidate中正式NPZ first0/1分别含actor ordinal1 **3／2点**，last0/1含 **1／2点**，其余0保留。这确认原生标签开始进入实际正式输出，**不证明每条射线和历史cell都正确归属、旧HIT已按完整条件退役、动态关卡通过或UNKNOWN整体已解决**。四个正式NPZ未保存原hit prim路径，v36旧字节不回填；组件没有运行ROS导航，旧v36两个cell的混合／poison／overflow等归因限制仍保留。

**v37主要停滞在有效指令进入真实plant之后。** SDK applied trajectory11从scene source **9.74–37.72s**连续forward，1572个状态、约 **27.98s source／78.602s steady**；原applied自身stamp首末span为28.000s，observer source span为27.980s，分别记录。全控制按v35/v36原连续非零定义为5段（原applied span median `.880s`），3个精确moving handoff全部成功，initialACK另1次。原全32709条trace中38个完整曲线、5个geometryinstall、4个原writercommit、2064个解析XYZ前视重算残差0，curve/body身份无冲突。scene15–35s的trajectory validation为`observed_free`、motion validation为`observed_free_motion_sweep`、tracker为`tracking`；对应实际PhysX robot.command持续约`.13886m/s`，officialpolicy forward始终float32 **`.3000000119`**。这段既不是只有高层demand，也不是UNKNOWN否决、C1换轨中断或owner独立死锁；原BT进度不足失败符合真实测量。

独立20–30s原窗口有 **600/600个实际非零command**，vx min／median／max **`.138856725/.138861414/.138868164m/s`**，policy forward min／median／max均`.3000000119`；原XYspeed median **`8.694×10⁻⁵m/s`**／max`.000183659`，rawfullXYZspeed median `.007452004m/s`，10s原净XY仅 **`8.754μm`**。四脚实际完整球最低Z高于`.01m`的采样均为0。t20与t30的forward积分均 **`.02362913157`**，yawpolicy约 **`−.0080853/−.0080644rad/s`**，原关节／current/previous/inference action和raw姿态／twist已保存。15–35s累计XY约 **`.000241329m`**，已有输入饱和而脚步未恢复的实证；`.00745`完整速度中的站姿接触Z响应不能伪装成路线前进或直接删去Z。

这支持进一步检验官方policy的history／previous-action驻留状态，但**动作记忆解释及reset方案尚未实测确证**。原相同直线park history的旧／新servo `.075/.15`组件A/B都曾通过，`.01`失败，不覆盖当前早期forward／yaw／zero波形。随后独立实验已按同一保存command-event序列和物理初值完成baseline与单因素动作记忆处理；基线已能行走，未复现正式停滞，不能从组件内差异宣布导航修复（见上述42秒回放证据）；保持原500Hz／50Hz、完整12joint／13shape、rawXYZ、`.15/.30` command与真实STOP，不用更高最低指令、无限积分、root速度写入或状态teleport掩盖。早期原短暂fullXYZ最大 **`.5023662918m/s`**超出显式`.50`reference拒绝域，原样保留，不能裁剪；域限制和未来真实plant速度保证也不能混同。

Phase／较晚bridge／最终physics全身证书 **2522／2629／2633**组零失败，所有2633个原body／link／actor同source tick、13primitive仿射AABB残差0、原envelope全部重算通过；rawZ **`.479576–.510891m`**，coldstand `.481/.589`和固定floor `−.02m`保持。原sampled static/dynamic AABB overlap为0，actor分离下界最低6.904087m，encounter false，所以没有动态让行或碰撞接触验收。Nativefloor878帧／2,174,302hit最大误差 **`4.834187μm`**，小于原200μm；双LiDAR各439帧，源／原END和所有点不改。NativeIMU21942／output4389约500／100Hz，无invalid／duplicate且每次方向不同。352原BEGIN→END姿态对误差max **`5.162×10⁻⁸rad`**、1753同tick gyro按原IMUframe误差max **`1.589×10⁻⁸rad/s`**；native stamp离线2ms关联余差最大1862ns，原native时间不重写。

原owner stop **210个stationary samples**当前完整线／角速度 **`.007215418m/s / .006073334rad/s`**；另原2.018s source fence的122条raw记录速度最大 **`.007230361/.011367265`**，均满足原`.03/.05`阈值，最大source间隔20ms。实际RTF **`.357868`**、原43.862s source／122.564577s state wall，双cloud最大wall gap **`.476505/.476631s`**；callback与notice profiling没有开启，显式verify `.023541s`仍不含listener。已有修改范围Python350项及focused37项通过属于重叠验证，不累加；本次只读审计未重测／重建。正式失败、未发生遭遇、原硬件未验收及旧失败证据全部保留。
