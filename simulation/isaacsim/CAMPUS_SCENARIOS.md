# 大型园区与系统测试关卡

本场景把同一 BehaviorTree.CPP → PCT → SCAN → tracker → safety → 唯一 SDK-free writer 导航主线接到官方 Spot 的 Isaac PhysX 模型。场景描述、离线检查与关卡入口分别是 [world_builder.py](world_builder.py)、[large_quadruped_scene.json](assets/large_quadruped_scene.json)、[scenario_suite.py](scenario_suite.py)。

**当前状态（北京时间 2026-10-07）：默认候选为 v31，`cancel_and_park` 通过、`crossing_blocker` 失败，其余四关未执行。** 取消关卡实测 **1.047985 m**，返回 `action_cancelled`，最终 **2.018 s 源时间**静止窗口及独立评估通过。横穿关卡实测累计 **0.403687 m** 后因 `execution_blocked_timeout:waiting_measured_motion_progress` 失败；准备证明饥饿已修复，但候选入口低速／方向变化与持续进度问题仍未闭环。独立的 v31 原 Action 近距短测实测 **1.040435 m** 后到达，停车、IMU 与全身检查通过；它不是新增的第七个系统关卡。见 [v31 实测摘要](verification/20261006/campus_navigation_v31.json)和 [横穿失败审查](verification/20261006/crossing_failure_audit_v31.json)。文档日期采用北京时间，证据目录 `20261006` 保留 UTC 日期。

历史 v30 `cancel_and_park` 实测累计行程 **1.197793 m**，原 Action 返回 `action_cancelled`，最终 **2.02 s 源时间**静止窗口及关卡评估通过；v30 横穿 **5.583955 m** 后失败，近距短测 **0.827614 m** 到达通过，均保留在 [v30 实测摘要](verification/20261006/campus_navigation_v30.json)。这些历史结果与 v31 分别取证。五点路线 **332.84 m** 仍仅是离线保守静态路线长度，不能表述为机器人已行驶的距离或六关已通过。

## 场景范围

园区平面为 **100 × 80 m**，地板 `z=0`，闭合碰撞天花板高度 **3.4 m**。共有 **85 个静态 box**，另有地板和天花板碰撞体。办公室包含八间房间和 4 m 宽长走廊；仓库有两排封闭货架包络和多个出入口；中心是开阔广场，南部含 1.4 m 窄门、柱群、低障碍和低净空梁。东西连接通路与南北绕行路径连接各区域。区域地面有配色、真实文字网格标牌、灯光、鸟瞰及随机器人更新的跟随相机。

门楣、低障碍、天花板都保留实际三维碰撞体。不可见天花板仍是碰撞面，不因鸟瞰显示隐藏而成为自由空间。本场景只支持经过封存的单层平地；没有楼梯或跨层能力声明。平地支撑合同要求明确的地板、完整机器人碰撞注册和有界动态占据证明。

| 配置 | 当前值与用途 |
| --- | --- |
| 机器人 | `official_spot_physx`，模型及实际碰撞形状由候选准备步骤注册；名义机身 `1.1 × 0.6 × 0.25 m` 不是完整碰撞证明 |
| 初始生成高度 | `0.80 m`；官方模型零关节姿态的足底相对机身约 `-0.692 m`，留下约 `0.108 m` 地面间隙，避免播放首步穿地翻倒；不能当作稳定站高 |
| 机身参考高度 | `0.52 m`，独立模型短测记录约 `0.478–0.544 m`；原导航目标使用地面 XYZ，因此目标 Z 为 `0` |
| 离线包络 | 半径 `0.65 m`，参考点下方 `0.52 m`、上方 `0.55 m`；用于保守可达性检查，不证明全步态体积 |
| 导航名义包络 | 长 `1.25 m`、宽 `0.90 m`、参考点上方 `0.55 m`；运行时还须检查真实全身证书 |
| 速度界 | 导航 command 封存为 `0.15 m/s`、`0.30 rad/s`；Spot 仿真可达模型另封存 `0.60 m/s`、`0.80 rad/s`，覆盖实测步态瞬时波动；指令权限、原 BT 期限和实机上限保持 |
| 地板物理材质 | 静态／动态摩擦均为 `1.0`、恢复系数 `0`；真实 physics material 绑定与值进入场景封存，匹配独立 Spot 短测地板 |
| 物理与状态 | 500 Hz 物理、50 Hz 原始实测状态；另随 10 Hz 雷达采样增加真实 prephysics BEGIN 的机身、全身及演员见证。两种状态保留各自真实源时间，不把原生帧 END 改写成 BEGIN |
| 三维雷达 | 前后两个，原点约 `(±0.2,0,0.2) m`，360° × 160°，1° × 5°，10 Hz，0.06–35 m；只过滤实际自体几何 |
| IMU | 500 Hz 原生 PhysX 测量，100 Hz 输出目标，保留独立原始传感器时间 |

## 动态演员与关卡

配置列出 **6 个可用演员（默认启用 4 个；迎面和堵门演员按关卡封存启用）**：广场行人、走廊行人、广场推车、仓库叉车、迎面推车、临时堵门推车。它们由具名 cylinder/box 碰撞形状组成，不声称是人体或车辆动力学模型。精确碰撞路径是 `/World/Dynamic/{actor_id}/{shape_id}`，不能用整个命名空间作为碰撞审计例外。

每段轨迹使用 C1 smoothstep，明确给出原始源时间 waypoint、速度、加速度和 yaw rate 上界。演员不依据机器人位置移动，不沿规划路径瞬移。启动时选定演员集合；启用的演员从加载就持续存在，运动窗口前后停在首点或终点。各目标之间不重置演员、种子或物理时钟；不同关卡创建新会话。

演员封存速度界为解析 C1 上界加显式 `native_physics_velocity_margin_mps=0.02` m/s；解析值另记为 `analytic_max_linear_speed_mps`。该裕量用于本场景 100×80 m、500 Hz 原生 f32 位置量化及 2 ms 速度解算：v23 原生实测走廊行人最大约 `0.763893 m/s`，略高于解析 `0.763636 m/s`；叉车约 `0.600815 m/s`，略高于解析 `0.600000 m/s`。保留全部真实样本并扩大封存可达速度界，不忽略越界样本或修改其源时间。更新后必须建立新候选，旧 v23 记录及合同保持原样；这项局部修正不表示关卡已通过。

| 关卡 ID | 内容 | 静态保守长度 | 源时间预算 | v31 实际状态 |
| --- | --- | ---: | ---: | --- |
| `long_distance_multi_goal` | 同一场景、同一会话按五个原导航 Action 连续访问办公室、仓库、服务区、柱群和广场 | 332.84 m | 3600 s | `pending`，未执行 |
| `crossing_blocker` | 广场行人横穿目标通路，检查实际遭遇、让行或有效绕行 | 16.00 m | 360 s | `failed`，行程 0.403687 m 后实测进度等待超时 |
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

即使关卡未来通过，它也只是明确采样范围内的同一仿真结果：没有采样间连续碰撞证明、真实 LIO 精度验收、真实 D1 Max SDK 接管或硬件刹停验收。`physical_robot_acceptance=false` 保持。

## 地图规模与离线检查

外墙厚度与闭合边界计入后，0.05 m 静态先验形状为 `2010 × 1610 × 73`，共 **236,235,300 个 uint8**，约 **225.29 MiB**，低于 256 MiB 原始体素上限。0.10 m 先验约 **28.61 MiB**。若同时分配 uint8、bool、float32 全体积，假定 dtype 下预算约 **1.417 GB**；这是资源估算，不是已测 RSS。

配置中三维表面采样为 0.10 m，PCT 分辨率为 0.10 m，PCT 单元上限 10,000,000、工作字节上限 2,000,000,000，静态先验保持 0.05 m。地图构建器仍须检查实际尺寸和预算，超限会拒绝候选。`world_builder` 的资源估算、几何及路线检查不分配整个先验，也不上传巨型体素图。

```bash
/usr/bin/python3 simulation/isaacsim/world_builder.py --audit
cd simulation/isaacsim
/usr/bin/python3 -m unittest -v test_world_builder.py test_collision_audit.py test_scenario_suite.py
```

这些测试文件的合成几何／证据夹具只检查生产规则与评估器是否拒绝缺失、外来、损坏或失败证据，不是实际导航实验。实际整图是否通过以各次不可覆盖的会话记录和证据评估为准。
