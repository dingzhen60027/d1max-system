# 大型园区与系统测试关卡

本场景把同一 BehaviorTree.CPP → PCT → SCAN → tracker → safety → 唯一 SDK-free writer 导航主线接到官方 Spot 的 Isaac PhysX 模型。场景描述、离线检查与关卡入口分别是 [world_builder.py](world_builder.py)、[large_quadruped_scene.json](assets/large_quadruped_scene.json)、[scenario_suite.py](scenario_suite.py)。

**最新完成的正式实测（北京时间 2026-10-07）：v37 `crossing_blocker` 仍失败，当前主要阻断已经在真实行走响应。** 原 Action status 6、`execution_blocked_timeout:waiting_measured_motion_progress`；累计 XY **`.471879 m`**、原 owner credit **8 次**、确认路线弧长 **`.302689 m`**，目标仍距 **15.695410 m**。SDK 原状态在场景源时间 **9.74–37.72 s** 连续正向约 **27.98 s**（1572 个 applied 状态、约 78.602 s steady）；15–35 s 的曲线／运动证明为正、tracker 为 tracking，但实际 policy 输入持续 `.3000000119 m/s`，机身 20 s 累计 XY 仅约 `.000241329 m`。20–30 s 的 600 个实际 PhysX command 都非零，净 XY 仅 **8.754 μm**、四脚没有 `.01 m` 以上的实际 solid 离地样本。因此不能把这一长区间归为 UNKNOWN 撤权、C1 换轨中断或 owner 独立死锁；独立动作历史 A/B 已完成，但基线未复现正式停滞，记忆干预未加入默认控制链。新 exact native root 标签已在正式 first／last NPZ 出现非零 actor ordinal，但没有指定动态遭遇、到达或动态旧 HIT 退役关卡通过，**不能宣布 UNKNOWN 已解决或导航丝滑**。完整形体、IMU、地板与实测停车核对成立，原整体 clean shutdown 失败及硬件停止未确认保持。见 [v37 原失败、真实指令及传感器证据](verification/20261007/campus_crossing_v37.json)和 [分析](SCAN_UNKNOWN_ANALYSIS.md)。

**随后完成的组件 A/B：** [42 秒真实关节回放](verification/20261007/spot_navigation_replay_v37_ab.json)中，基线／动作记忆干预组净前进约 3.47／4.52 m；干预前 4189 次 500 Hz 测量逐值相同，原零命令起点到连续一秒停稳确认分别为 2.056／1.832 s。基线已经能够行走，故未复现正式 v37 的原地站立，不能认定该干预解决了导航。基线 full XYZ 峰值 `.532835 > .50`、实验组 full angular 峰值 `.823027` 均原样保留，后者与 body yaw `.173694` 分开记录。记录仅重构原保存命令事件，在裁剪平地场景执行；默认步态、模型域及选择器未因此改变。

**前次 v36 正式失败保留，实际路线进度与指定演员遭遇已有改善。** 累计 XY 行程 **7.221877 m**、原 owner credit **199 次**、确认路线弧长 **6.643420 m**；终点仍距目标 **9.285196 m**。原 Action status 6，原因 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof`。原 writer 有 **58 段**连续非零输出（原 applied source 首末样本 median **`.570 s`**／max **`5.560 s`**）；27 个精确 handoff ID 中 **13 成功／14 未成功**，初始 ACK 另计。指定行人接近—近距离—离开实际被观察到，137 个 near 样本、采样形体分离下界最低 `.568772 m`；这不证明让行、绕行或到达。完整身体、地板、原生 IMU 同源和实测停车成立，但最终持续 hold、整体 clean shutdown 失败、原硬件停止未确认保留，**当前管线仍未达到连续、丝滑导航，也没有正式横穿通过结论**。见 [v36 原失败及完整证据](verification/20261007/campus_crossing_v36.json)与 [剩余问题分析](SCAN_UNKNOWN_ANALYSIS.md)。

**前次 v35 正式横穿失败保留。** 原 Action status 6，原因 `execution_blocked_timeout:actual_command_blocked:motion_sweep_occupied`。累计 XY 行程 **5.706249 m**、原 owner credit **157 次**、确认路线弧长 **5.285179 m**，终点仍距 **10.643651 m**。连续非零 writer 30 段（原 applied source 首末样本 median `.510 s`／max `8.700 s`）；65 个精确 handoff ID 中 7 成功、58 未成功。完整 6910 个原始 trajectory source 的身体与采样碰撞审计无失败／重叠，指定遭遇 `observed=false`；211 个样本实测停车、IMU 同源与软件退役成立，原 hardware stop 未确认及整体 clean shutdown 失败保留。v31 本机选择器、历史失败与四项未执行关卡保持。见 [v35 完整失败摘要](verification/20261007/campus_crossing_v35.json)及 [剩余阻断分析](SCAN_UNKNOWN_ANALYSIS.md#v35真实路线进度出现后的剩余阻断2026-10-07)。

**前次 v34 正式失败保留。** 低速入口拟合已使原局部 cap 恢复至最高 `.148315662 m/s`，但 87 段非零实际 writer 输出的源跨度中位数 `.040 s`、最长 `.140 s`；累计行程 **0.215597 m**、原 owner route credit 始终 **0**，原 Action status 6，原因 `execution_blocked_timeout:waiting_current_collision_and_tracker_proof`。全身证书、IMU 同源、210 个样本实测停车和软件退役成立；独立采样碰撞审计覆盖 2282 个原状态，无重叠且指定演员遭遇 `observed=false`。整体 clean shutdown 失败及硬件停止未确认保持。v31、v32、v33 历史横穿失败与其余四关未执行状态保留。见 [v34 控制链、失败与性能证据](verification/20261007/campus_crossing_v34.json)及 [审查](SCAN_UNKNOWN_ANALYSIS.md#v34速度-cap-恢复后的命令中断与性能审计2026-10-07)。

v33 使用同一 `.481/.589 m` profile 与原 XYZ 空间前视，累计行程 **0.013900 m**、原 owner credit 为 0，仍因当前 collision/tracker proof 等待超时失败。该次全身、IMU 与实测停车成立，但没有指定遭遇、整体 clean shutdown 或硬件验收。见 [v33 失败摘要](verification/20261007/campus_crossing_v33.json)、[v32 失败摘要](verification/20261007/campus_crossing_v32.json)及 [历史算法审查](SCAN_UNKNOWN_ANALYSIS.md#v32-横穿失败与-v33-复测结果2026-10-07)。

**本机选择器仍为旧封存 v31：`cancel_and_park` 通过、`crossing_blocker` 失败，其余四关未执行。** 取消关卡实测 **1.047985 m**，返回 `action_cancelled`，最终 **2.018 s 源时间**静止窗口及独立评估通过。横穿关卡实测累计 **0.403687 m** 后因 `execution_blocked_timeout:waiting_measured_motion_progress` 失败；准备证明饥饿已修复，但候选入口低速／方向变化与持续进度问题仍未闭环。独立的 v31 原 Action 近距短测实测 **1.040435 m** 后到达，停车、IMU 与全身检查通过；它不是新增的第七个系统关卡。见 [v31 实测摘要](verification/20261006/campus_navigation_v31.json)和 [横穿失败审查](verification/20261006/crossing_failure_audit_v31.json)。文档日期采用北京时间，证据目录 `20261006` 保留 UTC 日期。

历史 v30 `cancel_and_park` 实测累计行程 **1.197793 m**，原 Action 返回 `action_cancelled`，最终 **2.02 s 源时间**静止窗口及关卡评估通过；v30 横穿 **5.583955 m** 后失败，近距短测 **0.827614 m** 到达通过，均保留在 [v30 实测摘要](verification/20261006/campus_navigation_v30.json)。这些历史结果与 v31 分别取证。五点路线 **332.84 m** 仍仅是离线保守静态路线长度，不能表述为机器人已行驶的距离或六关已通过。

## 场景范围

园区平面为 **100 × 80 m**，地板 `z=0`，闭合碰撞天花板高度 **3.4 m**。共有 **85 个静态 box**，另有地板和天花板碰撞体。办公室包含八间房间和 4 m 宽长走廊；仓库有两排封闭货架包络和多个出入口；中心是开阔广场，南部含 1.4 m 窄门、柱群、低障碍和低净空梁。东西连接通路与南北绕行路径连接各区域。区域地面有配色、真实文字网格标牌、灯光、鸟瞰及随机器人更新的跟随相机。

门楣、低障碍、天花板都保留实际三维碰撞体。不可见天花板仍是碰撞面，不因鸟瞰显示隐藏而成为自由空间。本场景只支持经过封存的单层平地；没有楼梯或跨层能力声明。平地支撑合同要求明确的地板、完整机器人碰撞注册和有界动态占据证明。

| 配置 | 当前源码生成值与用途；旧封存候选保持原值 |
| --- | --- |
| 机器人 | `official_spot_physx`，模型及实际碰撞形状由候选准备步骤注册；名义机身 `1.1 × 0.6 × 0.25 m` 不是完整碰撞证明 |
| 初始生成高度 | `0.80 m`；官方模型零关节姿态的足底相对机身约 `-0.692 m`，留下约 `0.108 m` 地面间隙，避免播放首步穿地翻倒；不能当作稳定站高 |
| 机身参考高度 | 新 profile 为 `0.481 m`，cold stand 实测中位数 `.4807447493 m`；旧 v31/v32 为 `.52 m`，旧 `.478–.544` 范围混合了站姿／步态。原导航目标始终为真实地面 XYZ，目标 Z 为 `0` |
| 离线包络 | 半径 `0.65 m`，新参考点下方 `.481 m`、上方 `.589 m`，绝对顶部仍 `1.07 m`；旧 v31/v32 为 `.52/.55 m`。只用于保守可达性检查 |
| 导航名义包络 | 长 `1.25 m`、宽 `0.90 m`，新参考点上方 `.589 m`；完整 13 实际形体与固定 `floor−.02` 下界保持，运行时仍必须取得真实同源全身证书 |
| 控制前视 | 新四足 profile 显式 `spatial_control_lookahead=true`，使用原完整 XYZ 弧长前视 `.12 m`，不改变实测进度或原 cap；默认配置为 `false`，v33、v34、v35、v36、v37 正式横穿仍 failed |
| 低速入口拟合 | v34 显式启用 `manager.fit_low_speed_entry_velocity`，默认 `false`；完整 XYZ 低速、零加速度及原支撑／源合同内只调整 solver 边界，原 raw twist／source、`.05` C1 adoption、全体积碰撞和权限保持 |
| 低级行走 | `spot_monotone_measured_v2`；旧／新 mode 相同直线 A/B 均通过，`.01 m/s` history 用例仍失败，不能以组件成功推断导航成功 |
| 速度界 | 导航 command 封存为 `0.15 m/s`、`0.30 rad/s`；Spot 仿真可达模型另封存 `0.60 m/s`、`0.80 rad/s`，覆盖实测步态瞬时波动；指令权限、原 BT 期限和实机上限保持 |
| v36 参考／移动接纳域 | guide／command 仍 `.15 m/s`、`.30 rad/s`，显式隔离 Spot reference 与原实测 full XYZ source-travel 为 `.50 m/s` 拒绝域；不是新 command 或未来步态速度保证，默认实机域及 full XYZ C1／制动保持 |
| v36 演员未来／旧 HIT | 原封存脚本枚举六秒完整未来体积；只有完整同一非零 actor native HIT 历史、严格更新且有效 oracle 与整个闭3D cell分离才选择原 static FREE；未知／混合／缺失仍否决，不清 raw odds／timestamps |
| 地板物理材质 | 静态／动态摩擦均为 `1.0`、恢复系数 `0`；真实 physics material 绑定与值进入场景封存，匹配独立 Spot 短测地板 |
| 物理与状态 | 500 Hz 物理、50 Hz 原始实测状态；另随 10 Hz 雷达采样增加真实 prephysics BEGIN 的机身、全身及演员见证。两种状态保留各自真实源时间，不把原生帧 END 改写成 BEGIN |
| 三维雷达 | 前后两个，原点约 `(±0.2,0,0.2) m`，360° × 160°，1° × 5°，10 Hz，0.06–35 m；只过滤实际自体几何 |
| IMU | 500 Hz 原生 PhysX 测量，100 Hz 输出目标，保留独立原始传感器时间 |

新标定来自 [三份真实 cold stand 与完整形体分析](verification/20261007/spot_cold_stand_calibration.json)，不是更改 PhysX 身体高度或缩小腿部。新 config、profile、static-prior provenance、运行 YAML 与 session 必须重新封存；旧候选配置不会被启动脚本改写。原命名的 `.52` fixture goal annotation 与实际 Action 的 ground Z=`0` 继续区分。按同 footprint 算出的最大 `196707/200000` cells 不是新运行证明，所有原计算／时效限额保持。当前按用户要求不录制。

## 动态演员与关卡

配置列出 **6 个可用演员（默认启用 4 个；迎面和堵门演员按关卡封存启用）**：广场行人、走廊行人、广场推车、仓库叉车、迎面推车、临时堵门推车。它们由具名 cylinder/box 碰撞形状组成，不声称是人体或车辆动力学模型。精确碰撞路径是 `/World/Dynamic/{actor_id}/{shape_id}`，不能用整个命名空间作为碰撞审计例外。

每段轨迹使用 C1 smoothstep，明确给出原始源时间 waypoint、速度、加速度和 yaw rate 上界。演员不依据机器人位置移动，不沿规划路径瞬移。启动时选定演员集合；启用的演员从加载就持续存在，运动窗口前后停在首点或终点。各目标之间不重置演员、种子或物理时钟；不同关卡创建新会话。

演员封存速度界为解析 C1 上界加显式 `native_physics_velocity_margin_mps=0.02` m/s；解析值另记为 `analytic_max_linear_speed_mps`。该裕量用于本场景 100×80 m、500 Hz 原生 f32 位置量化及 2 ms 速度解算：v23 原生实测走廊行人最大约 `0.763893 m/s`，略高于解析 `0.763636 m/s`；叉车约 `0.600815 m/s`，略高于解析 `0.600000 m/s`。保留全部真实样本并扩大封存可达速度界，不忽略越界样本或修改其源时间。更新后必须建立新候选，旧 v23 记录及合同保持原样；这项局部修正不表示关卡已通过。

| 关卡 ID | 内容 | 静态保守长度 | 源时间预算 | 已完成实测与待复测 |
| --- | --- | ---: | ---: | --- |
| `long_distance_multi_goal` | 同一场景、同一会话按五个原导航 Action 连续访问办公室、仓库、服务区、柱群和广场 | 332.84 m | 3600 s | `pending`，未执行 |
| `crossing_blocker` | 广场行人横穿目标通路，检查实际遭遇、让行或有效绕行 | 16.00 m | 360 s | v31 `.403687 m`、v32 `.056649 m`、v33 `.013900 m`、v34 `.215597 m`、v35 `5.706249 m`、v36 `7.221877 m`、v37 `.471879 m` 均 `failed` |
| `narrow_head_on` | 窄门迎面推车，检查让行或绕行及实体重叠 | 31.38 m | 600 s | `pending`，未执行 |
| `temporary_door_block` | 推车连续驶入、停留、退出门口，检查等待或绕行、恢复到达 | 31.38 m | 600 s | `pending`，未执行 |
| `cancel_and_park` | 原 Action 在首个实测状态后 12 s 源时间请求取消，检查退役、writer 停止及至少 2 s 实测停车 | 18.00 m | 60 s | `passed`，v31 行程 1.047985 m、最终静止 2.018 s |
| `resume_after_block` | 65 s 源时间取消，110 s 后提交新身份任务；旧任务不得恢复执行 | 31.38 m | 600 s | `pending`，未执行 |

这是约 **333 m 的慢速长程系统 benchmark**。按封存 command 上限 `0.15 m/s`，仅平移就至少需要约 **2220 s 源时间**；拐弯、规划、让行及停车另外耗时。每个到达 phase 的观察预算为 `ceil(ceil(静态路线长度 / 0.15) × 1.5 + 120)` s，长程五段分别为 `618/1062/651/819/782` s，整个关卡仍受 `3600 s` 总预算约束。取消 phase 按事件及停止观察设置预算；恢复关卡第一 phase 为 `95 s`，新任务到达 phase 为 `435 s`。较长的测试观察预算不会改变原 Action schema、30 s 执行阻断期限、watchdog、样本时效或 motion lease。阻挡超过原期限且没有有效进展时，原任务应按原保护失败，不能为通过关卡放宽保护。

长度来自 0.25 m 离线检查网格及保守包络。网格半格余量使窄门检查选择绕行，不能据此宣布正式 PCT 的路线或实际通过方式。动态场景必须有实际演员遭遇证据；仅到达目标也可能没有触发预定阻挡。

堵门演员预先封存为场景源时间 `20 s` 开始驶入、`45–70 s` 停在门口、`95 s` 返回首点。按 0.15 m/s 直达门口的最早估算约 `47 s`，与该固定窗口重合；规划等待及绕行会改变实际遭遇，不能保证演员一定挡到机器人。恢复关卡把取消与新任务改为事件源时间 `65/110 s`，给低速接近及退役留下时间，但仍须真实遭遇证明。演员轨迹不在运行中追随机器人重新排程。

关卡取消、恢复事件的源时间原点是第一个 smoke observer 收到的真实状态时间。演员轨迹使用物理场景启动以来的原始源时间；这两种原点在配置合同与原始报告中分别记录；执行记录保留场景 clock anchor、每 phase 的场景源时刻、首个实测状态源原点与实际应用预算，不用墙钟冒充源时间。整图源时间慢于墙钟时，控制保护和样本期限仍保留。

## 可复现命令

以下命令使用本机已经配置的隔离构建目录。先准备 Isaac Sim、Humble/Zenoh、科学库及原生依赖；换机器先参考 [README 的构建说明](README.md#已配置目录与重新构建)。候选、依赖和官方模型缓存是本机产物，不作为 Git 中的已部署版本。

```bash
cd /home/eric/wjg/d1max-system
source simulation/isaacsim/env.sh

# 只读列出六个关卡、地面目标和离线米制长度。
/usr/bin/python3 simulation/isaacsim/scenario_suite.py list

# 选择关卡，并为每次准备、候选和运行使用全新的名字。
export D1MAX_CASE_ID=crossing_blocker
export D1MAX_CASE_CONFIG="$D1MAX_SIM_BUILD/scenarios/crossing_blocker_001.json"
export D1MAX_CASE_CANDIDATE="$D1MAX_SIM_BUILD/isaac-candidate-campus-crossing-001"
export D1MAX_CASE_SESSION="$D1MAX_SIM_BUILD/runs/campus_crossing_001"

/usr/bin/python3 simulation/isaacsim/scenario_suite.py prepare \
  --case "$D1MAX_CASE_ID" --output-config "$D1MAX_CASE_CONFIG"

# 先构建当前导航源码安装（已有匹配安装时不必重复）。
bash simulation/isaacsim/build_local.sh

# 完整重新准备官方模型注册、碰撞场景、PCT 地图、静态先验和候选清单。
/usr/bin/python3 simulation/isaacsim/build_candidate.py \
  --output "$D1MAX_CASE_CANDIDATE" \
  --scene-config "$D1MAX_CASE_CONFIG" \
  --nav-install "$D1MAX_SIM_BUILD/nav/install" \
  --sdk-install "$D1MAX_SIM_BUILD/sdk/install" \
  --localization-install "$D1MAX_SIM_BUILD/localization/install" \
  --pct-vendor "$D1MAX_SIM_BUILD/pct_vendor"

# 可先检查待执行命令；这一步不会启动场景、ROS 或 GUI。
/usr/bin/python3 simulation/isaacsim/scenario_suite.py run \
  --case "$D1MAX_CASE_ID" --candidate "$D1MAX_CASE_CANDIDATE" \
  --session "$D1MAX_CASE_SESSION" --plan-only

# 一次新会话，默认无界面；同一原导航图执行该关卡。
/usr/bin/python3 simulation/isaacsim/scenario_suite.py run \
  --case "$D1MAX_CASE_ID" --candidate "$D1MAX_CASE_CANDIDATE" \
  --session "$D1MAX_CASE_SESSION"

# 评估使用候选内最终封存的配置，包含准备阶段增补的真实模型注册。
/usr/bin/python3 simulation/isaacsim/scenario_suite.py evaluate \
  --prepared-config "$D1MAX_CASE_CANDIDATE/simulation/assets/scene_config.json" \
  --session "$D1MAX_CASE_SESSION" \
  --output "$D1MAX_CASE_SESSION/scenario_evaluation.json"
```

`prepare` 拒绝覆盖输入配置；候选组装拒绝复用目录；运行要求新会话；评估输出也不能覆盖已有报告。模型与地图准备可能留下失败候选，重新尝试使用新的路径并保留原始失败证据。更改机器人姿态、演员、场景几何或源码后，重新组装和封存候选，不能在运行中换成未封存配置。候选准备步骤增补实际机器人注册后，重新计算场景输入的规范配置摘要；候选完整性清单最终绑定实际文件字节。

`run` 可显式添加 `--gui` 请求可见 Isaac 窗口，默认不会打开 GUI。也可使用原入口 `launch.sh --candidate 候选路径 --scenario-case 关卡ID --headless`；`run.py` 在自己的同一 `owned_graph` 中启动 `scenario_suite.py execute`。多点关卡按顺序提交原 Action，每点使用新的 trace/report 文件，物理、导航、路由和演员时钟保持同一会话。没有把多次仿真拼成一次多点测试。

## 结果与证据

每次关卡保留 `scenario_suite/execution.json`、`phase_NNN_report.json`、`phase_NNN_trace.jsonl` 与日志；原图仍写 `run_summary.json`、退役记录和 `physics/trajectory.jsonl`。可选 observer `--source-duration-s` 在原 ROS 源时间预算超限时记录 `smoke_source_timeout`，并通过原 Action 请求取消；默认轮式 smoke 行为保持。失败的原 Action、源时间预算、退役或停车检查会保留失败，不由离线可达性结果覆盖。

评估返回三个状态：`passed`、`failed`、`pending`。退出码分别为 `0`、`1`、`2`。无实际执行、缺原始源事件、缺独立 IMU 匹配、缺完整 2 s 源时间停车窗口、缺全身证书或真实碰撞审计时不能全通过。任一已记录的真实失败优先返回 `failed`。

模型特定高度容差取自封存的 `static_collision_prior_contract.max_body_height_error_m`。较宽的机器狗容差还要求桥接状态中的真实全身证书计数、精确注册摘要和零失败计数；轮式 observer 的默认 0.03 m 高度检查保持。全身包络证书与 writer 停止 ACK 仍不能代替实际接触／碰撞验收。

实际碰撞验收要求 `physics/collision_audit.json` 绑定同一会话、场景字节、实际机器人注册、演员集合、完整物理轨迹哈希与 Action 源时间范围。审计从实际 PhysX link 矩阵与同一步演员根姿态计算所有注册形状的包围盒；严格分离才能在该采样范围内记录零次穿透。包围盒接触或重叠属于尚未解决的潜在重叠，不能冒充已测穿透，也不能填写零次。显式地板支撑接触由独立合同检查，其他低障碍不豁免；没有采样间连续碰撞证明。

四个动态遭遇关卡（横穿、迎面、堵门、恢复）在 `prepare` 时封存 `actor_encounter_contract`，明确要求相应启用演员的精确 ID。旧动态配置缺少此合同会拒绝运行，须使用新名字重新 `prepare`、构建和封存候选。现有 `Audit.sample()` 生产路径直接使用同一步的真实注册 link／演员几何，最终在 `collision_audit.json.actual_actor_encounter` 写入证据；评估器从该审计读取并核验，执行记录中的手填 `observed` 或目标到达不能替代它。

遭遇规则是同一对精确 `robot_path` 与 `actor_shape_path` 的“接近—近距离停留—离开”：形状内部实际中心点间距不大于 **2 m**，连续停留至少 **0.5 s 源时间**；接近和离开分别产生至少 **0.5 m** 距离变化，历时 **1–30 s**，平均距离变化率至少 **0.05 m/s**。接近从 2 m 范围外开始，离开回到范围外；全程最大审计源时间间隔 **0.12 s**。中心点属于真实形状，因此其间距给出形状距离的上界；只凭包围盒距离下界很小不能证明近距离遭遇。算法不能把不同脚、不同演员形状或不同会话的距离拼成一次遭遇。

证据记录实际中心点、精确形状路径、接近／停留／最近／离开时刻、原始 sim/source 纳秒，以及规则、机器人和演员注册摘要。历史与证书数量有明确封存上限；截断、源时间间隔过大、缺完整轨迹或审计失败时，遭遇结果保持 `pending`。评估只接受首个原 Action 提交至最后一个终态之间完整发生的证书；完整、新鲜且覆盖该范围的审计确认没有预定遭遇时，该项为 `failed`。这是**采样几何接近证据**，不单独证明演员挡住了路线、机器人正确让行或不存在真实接触。

长程和取消停车关卡仍保留碰撞、原 Action 与停车要求，不把每个启用演员的近距离遭遇列为必经条件。历史 v30 取消会话的报告状态、速度和 IMU 已逐项匹配原始 PhysX 记录，全身与碰撞审计覆盖 1,107 个实际采样时刻；最终 122 个停车样本覆盖 2.02 s 源时间，最大间隔 20 ms，完整线速度／角速度最大为 `.007147 m/s / .022544 rad/s`。该次演员未接近机器人，取消停车通过不证明动态让行，也不作为当前 v31 取消关卡的结果。**332.84 m 目前只来自离线静态几何检查**。仿真纯 Python 回归 232 项通过，只检查实现与证据规则，不代替实际关卡结果。

v31 取消会话独立评估为 `passed`：行程 **1.047985 m**，原 Action 确认取消，实测停车、IMU 与全身检查通过；122 个最终停车样本覆盖 **2.018 s 源时间**，最大间隔 20 ms。该用例与独立近距到达成功分别记录，均不要求指定动态遭遇，不能替代仍失败的横穿关卡。

历史 v30 横穿会话的独立评估为 `failed`，指定遭遇 `observed=false`；实际演员 AABB 分离距离下界最低 **2.3097 m**，各采样时刻没有静态或动态 AABB 相交。原 Action 在该 phase 的 **117.6 s 源时间**返回失败，早于 **281 s** 测试观察预算。日志中演员预测体积造成多段合法 UNKNOWN 否决；约场景源时间 **105 s** 后当前轨迹证明恢复，准备候选证明仍耗尽 5 ms 通道，不能只归因为动态 UNKNOWN。

v31 prepared 使用原 **50 ms** 轮内未用时间，最多沿用原 **25 ms** motion cap，**5 ms** 预留、当前轨迹优先级和全部 source／receipt／grant／proof／deadline 检查保持；ledger **39/39**、非 launch CTest **15/15** 通过，见 [调度回归](verification/20261006/prepared_lane_offline_v31.json)。v31 横穿实际出现 **14 条准备运动证明通过、7 次成功换轨、零准备预算耗尽**，说明修复已实际生效，但任务仍失败，指定遭遇 `observed=false`。新几何入口改变了可执行前进方向和低速需求，恢复中确认路线进度最高仅约 `.112 m`，累计 `.403687 m` 不能当作有效路线进度。成功换轨保留输出历史，没有无条件清零 ramp 的证据；详细局限见 [v31 失败审查](verification/20261006/crossing_failure_audit_v31.json)。不能将调度修复、近距成功或零采样重叠表述为动态避障已通过。

v32 正式横穿的 IMU、全身证书和 measured stop 成立，原 mock `physical_stop_confirmed=false` 与整体 clean shutdown 失败保留；独立评估仍为 `failed`。源时间 22–34 s 高层 `vx` 中位数 `.003958 m/s`、最大 `.009858 m/s`，原 owner 没有任何路线 credit。curve 21 的完整几何表明，固定 `u+.8 s` 前视需求约 `.004806 m/s`，比原局部 cap `.034480 m/s` 更低；[原 XYZ 空间前视反事实与精确曲线回归](verification/20261007/tracker_spatial_control_lookahead_offline.json)仍按该 cap 限制。v33 随后实际启用新高度与前视，仍因当前 collision/tracker proof 等待超时失败；前视和标定不能被表述为端到端成功。

v33 curve 13 的实际 trace 确认原 XYZ 前视已经推进 `.12 m`、求值参数 `5.101383 s`；前视 `vx=.0592424 m/s`，但同一原局部 cap 仅 `.000125981 m/s`，实际 XY speed `.000094498 m/s`。为什么入口 cap 仍极低及当前 proof 如何影响执行，继续按完整原几何／时序审查；不能增设最低指令掩盖问题。Phase 2214 个全身证书和最终 bridge 2295 个证书均零失败，窗口分别记录；碰撞审计覆盖全部 2300 个原始状态，没有静态／动态 AABB 重叠且指定遭遇未发生。实测 stop 211 个样本、完整线／角速度 `.007535815/.000992841`，软件退役成立，原物理验收标志及整体失败保持。[v33 证据](verification/20261007/campus_crossing_v33.json)记录 4 个 tracker CTest 组及 254 项 Python 回归通过，它们不代替关卡成功。

动态 sphere 精确相交、移动入口方向门和 v2 servo 是三个有针对性的修复；v32 在应用它们后仍失败，因此不能把任何一项当作整条导航链已经修复的证据。相同直线历史下旧／新 servo 都通过，微速用例仍失败，详见 [组件 A/B](verification/20261007/spot_servo_ab.json)、[sphere 验证](verification/20261007/dynamic_reachable_sphere_v32.json)与 [移动入口回归](verification/20261007/tracker_moving_entry_offline.json)。

v34 的入口 cap 已恢复，正式任务仍未形成路线 credit。全身 phase 证书 **2189 组／零失败**，较晚 bridge **2277 组／零失败**，窗口分别保留；全部 **2282 个**原始 source 的采样审计最大间隔 20 ms，无静态／动态 AABB 重叠且指定遭遇未发生。Stop 210 个样本的完整线／角速度为 `.007455075 m/s / .000796454 rad/s`。真实时间比仅 `.245931`，点云 wall 最大间隔约 `.598546 s`，原 500 ms receipt 期限因此会成为实际否决，不能延长期限或将 UNKNOWN 当作 FREE。相同封存配置的 5000 步独立组件 profile 为 `.254026`，native／未拆分区间包含 USD listener 等开销；线程对照的组件结果不代替原 Action 关卡验收。v34 的 16 个 SCAN、4 个 tracker CTest 组及 254 项 Python 回归均通过，正式结果仍为 failed；没有 v35 通过结论。[v34 原始摘要和哈希](verification/20261007/campus_crossing_v34.json)保留取证范围。

后续组件试验不能直接替换关卡合同：CPU0 的点云 wall 最大间隔 `.719476 s`，未选择；USD notice handler 的 `9.793 s` 不是显式 static `verify()` 的 `.016 s`，必须分开计时。完全关闭 USD writeback 被原 floor guard 拒绝；只在 10 Hz native BEGIN 发布真实姿态的 `.075 m/s` 组件虽有真实四足脚步、600 组完整身体证书和 499,003 个 floor hit 通过，IMU 姿态只约 10 Hz 更新，与同 tick 原身体姿态及 gyro 不一致，故 RTF `.420805` 也不是合格的传感器保持方案。[独立审计](verification/20261007/physics_performance_writeback_audit.json)保留全部失败与 raw 哈希。未来原 Action 仍须原 500 Hz physics、50 Hz policy、双雷达 10 Hz、native500→100 Hz IMU、50 Hz state＋10 Hz BEGIN、13 形体、原 lease 与真实 STOP，不能降低传感器或放宽权限掩盖计算延迟。

后续 velocity-only 对照保留原每步 500 Hz 姿态写回，关闭 USD velocity 副本，实际速度继续取 native／tensor。RTF `.413157`、点云 wall 最大间隔 `.451119 s`；完整 600 条实际身体／joint／policy 记录和 13 形体快照与前次 walking run 字面相同，IMU 原 source ns 不变且恢复每 100 Hz 输出的实际方向更新。100 个实际 BEGIN 与原 2 ms 后 IMU END 姿态、500 个 gyro 坐标核对通过；native floor／全身检查保持。此处只记录组件通过检查；随后 v35 原 Action 实际应用该设置，仍 failed，正式性能／传感器与关卡结果在下段分别记录。

v35 正式配置只关闭 USD velocity 副本，保留原 500 Hz pose 写回／官方关节策略和完整测量。实际 RTF **`.344853`**（115.142 s source／333.886827 s wall），双 native 雷达各 **1152 帧**、最大 wall 间隔 **`.472398/.470960 s`**；57,580 个 native IMU 与 11,517 个原输出为约 **500／100 Hz**，没有 invalid／duplicate，方向每次更新。独立原 BEGIN→IMU END 673 对姿态、同 tick 3820 对 gyro 坐标核对通过；没有重 stamp 或用真值替代传感器方向。5,712,883 个原地板 hit 的最大误差 **3.929568 μm** 小于原 200 μm。Phase body 6795／较晚 bridge 6907／最终 physics 6910 组均零失败，窗口分别保留，13 个实际形体与 raw Z 不变。Stop 211 个样本的完整线／角速度为 **`.007447834 m/s / .000216747 rad/s`**；整体失败和物理验收标志仍保留。

原控制链的 7 次成功换轨之外，45 次原 handoff deadline 拒绝、9 次 owner 撤回、3 次 owner hold、1 次最终任务 timeout 仍未闭环；初始 writer ACK 另计，不冒充换轨。最终 Action 的 current-command `motion_sweep_occupied` 与末次曲线 `unknown_or_unobserved` 是不同查询，不能唯一归因为高 C1、servo 或 UNKNOWN。[原六秒可达球捕获](verification/20261007/v35_live_unknown_bounded_capture.json)保留原静态身份、射线／演员源时间和未过期租约，该体素确与原完整可达 sphere 相交；它不是当前实际演员身体碰撞证据。8 个 plan_env CTest 组（projected rays 102 项）、17 个 SCAN CTest 组及 254 项 Python 已有回归通过，不覆盖正式导航失败、遭遇未发生或四关未测。[v35 原始证据和复核代码](verification/20261007/campus_crossing_v35.json)绑定完整 trace、实际形体、传感器、原评估和候选哈希。

即使关卡未来通过，它也只是明确采样范围内的同一仿真结果：没有采样间连续碰撞证明、真实 LIO 精度验收、真实 D1 Max SDK 接管或硬件刹停验收。`physical_robot_acceptance=false` 保持。

v36 原 Action 累计 XY **7.221877 m**、owner credit **199 次**、确认路线弧长 **6.643420 m**，指定 actor sampled encounter 为 **observed=true**，仍以 status 6／`execution_blocked_timeout:waiting_current_collision_and_tracker_proof` 失败。58 段连续非零 writer 的原 applied source 跨度 median **`.570 s`**／max **`5.560 s`**，27 个精确 handoff 中 13 成功／14 未成功；初始 ACK 另计，实际最后持续 hold 34.10 s source，未达到连续、丝滑导航。原 scene source 61.018 s 接近、62.018–64.28 s near（137 样本）、64.56 s 离开，是相同机器人／actor形状中心对；最近中心1.652738m，最小 sampled AABB分离下界.568772m。它不代替让行、绕行、碰撞接触或到达。最终目标误差 **9.285196 m**，四项未运行关卡和 v31 选择器保留。[v36 原失败证据](verification/20261007/campus_crossing_v36.json)与 [原因、保护与取证范围](SCAN_UNKNOWN_ANALYSIS.md)分别记录。

v36 phase／较晚 bridge／最终 physics 全身证书分别 **5364／5466／5477** 组零失败；完整5477个实际 source 的13形体 AABB重算残差0、采样无重叠。Raw Z为`.480028–.542953 m`，cold stand `.481/.589`及原完整腿部、floor `−.02 m`保持；4脚真实flight、12关节／policy记录未替换。实际RTF **`.338855`**、双点云wall最大间隔 **`.516322/.516786 s`**，保留原500Hz pose writeback、只关velocity副本和所有lease，当前仍有真实proof/command expiry。正式callback与USD notice profiling未启用，不能混入组件计时。双native雷达各913帧，4,533,723原floor hit最大误差4.852557μm；45,640 native／9129输出IMU约500／100Hz，577原BEGIN/END姿态和3151同tick gyro核对通过。Owner stop213样本当前全速度`.007401856/.000154388`；原2.02s窗口122raw样本全线／角速度最大`.007494982/.001371126`，原阈值满足。IMU同源、软件退役成立，原physical_stop_confirmed／clean_shutdown均false保留；不能据这些局部通过改写原关卡结果。

此次候选重新封存的修改分别见 [full XYZ reference/source-travel](verification/20261007/full_xyz_reference_domain_offline_v36.json)、[scripted actor 完整未来形体](verification/20261007/scripted_actor_future_v36_offline.json)、[native actor HIT 身份及条件退役](verification/20261007/native_actor_hit_provenance_v36_offline.json)。实际 first NPZ没有actor ordinal，last双帧ordinal均0且未保留原hit prim paths；生产者需独立原生组件核验，不能唯一归因标签或改写旧hit身份。[构建／ABI及回归记录](verification/20261007/v36_runtime_build_offline.json)包含58CTest组／945实际XML用例零失败、修改范围347Python通过，以及原broad17失败／2336通过／3跳过与两次packaging失败。缺默认外部map的17项未到达深入断言，原日志保留；构建记录是candidate封存后新增仓库证据，**不在旧candidate seal内**，不替代正式FAILED。[上游原源码设计研究](UPSTREAM_DESIGN_REVIEW.md)提供固定SHA的PCT+SCAN／集成与Robot-Nav对比，保留3D高度、unknown与物理策略合同，不用上游demo视频替代本关卡验证。当前按用户要求不录制。

v37 的任务状态仍 failed：行程 `.471879 m`、8 次原 credit、确认路线 `.302689 m`、目标误差15.695410m，指定actor遭遇 `observed=false`。原控制5段非零，原 applied source 最大28.000s／观测scene source最大27.980s；3次moving handoff全成功、initial另1次，但scene15–35s正proof／tracking和实际正command下仍几乎站立。原20–30s中600/600 actualcommand非零、policy forward输入全为float32 `.3000000119`，真实净XY8.754μm、四脚>.01m actualsolid lift为0。Action因此按原measured-progress保护失败，不能归为UNKNOWN撤权或owner单独死锁；独立动作历史A/B已完成但未复现正式停滞，不用最低速度／teleport／去Z替代物理验证。[v37 完整失败及窗口](verification/20261007/campus_crossing_v37.json)保留原字节和代码。

正式新版 exact actor root／leaf ordinal1 的 first双帧有3/2点、last有1/2点；[独立真零 native root 组件](verification/20261007/native_root_hit_v37_component.json)与该正式结果分开。旧v36 all-zero标签记录不被改写，未知／静态仍0，完整六秒future与旧HIT条件退役保持；没有完整动态避让或UNKNOWN已解决结论。原phase2522／bridge2629／physics2633全身证书零失败，实际13形体／rawXYZ未缩减，原全速度最高 `.502366292`保留。439帧／sensor、2,174,302 floor hit误差最大4.834187μm；21,942 native／4389 output IMU原352姿态／1753gyro核对通过。Stop210样本当前`.007215418/.006073334`，原2.018s122raw窗口最大`.007230361/.011367265`满足原阈值，software retired成立，hardwarephysical stop／clean_shutdownfalse保持。RTF `.357868`仅是该失败运行性能；已有350Python／37focused重叠通过不覆盖正式failed。原source预算281s与实际观察39.32s区别保留，四项未执行关卡及v31默认选择器保持。

## 地图规模与离线检查

外墙厚度与闭合边界计入后，0.05 m 静态先验形状为 `2010 × 1610 × 73`，共 **236,235,300 个 uint8**，约 **225.29 MiB**，低于 256 MiB 原始体素上限。0.10 m 先验约 **28.61 MiB**。若同时分配 uint8、bool、float32 全体积，假定 dtype 下预算约 **1.417 GB**；这是资源估算，不是已测 RSS。

配置中三维表面采样为 0.10 m，PCT 分辨率为 0.10 m，PCT 单元上限 10,000,000、工作字节上限 2,000,000,000，静态先验保持 0.05 m。地图构建器仍须检查实际尺寸和预算，超限会拒绝候选。`world_builder` 的资源估算、几何及路线检查不分配整个先验，也不上传巨型体素图。

```bash
/usr/bin/python3 simulation/isaacsim/world_builder.py --audit
cd simulation/isaacsim
/usr/bin/python3 -m unittest -v test_world_builder.py test_collision_audit.py test_scenario_suite.py
```

这些测试文件的合成几何／证据夹具只检查生产规则与评估器是否拒绝缺失、外来、损坏或失败证据，不是实际导航实验。实际整图是否通过以各次不可覆盖的会话记录和证据评估为准。
