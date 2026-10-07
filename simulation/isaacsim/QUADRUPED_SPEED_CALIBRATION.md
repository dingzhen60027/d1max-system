# 机器狗速度响应与导航配置

本文区分导航需求、官方策略输入、真实机身速度与地图／权限证明。使用官方 Spot 真实关节、100×80 m 园区、双 3D LiDAR 和原生 IMU，PhysX 500 Hz、官方推理 50 Hz；组件标定与原导航 Action 分别验收。

## 最新状态（2026-10-07）

**[v65 取消后重新导航整例通过](verification/20261007/campus_restart_v65_all_actors_goal_success.json)，14/14 原检查通过。** 原取消阶段行程 `2.3507 m`；同一物理会话中的新任务行程 `13.958468 m`、目标误差 `.190361 m`，实际同 pair 的 3 m 曝光、完整身体采样和干净退役均成立。[v64 广场横穿](verification/20261007/campus_crossing_v64_all_actors_goal_success.json)也已 **11/11 原检查通过**，行程 `16.636568 m`、目标误差 `.097536 m`。[v63](verification/20261007/campus_restart_v63_goal_success_encounter_pending.json)到达与完整物理审计通过，但原相遇历史截断，整例保持 PENDING。

| 完整原生物理记录 | v64 横穿 | v65 取消重启 |
| --- | ---: | ---: |
| 500 Hz BEGIN＋真实 END 样本 | 55,391 | 53,293 |
| full XYZ 峰值（域 `.65 m/s`） | `.592872` | `.577411` |
| world yaw 峰值（域 `.8 rad/s`） | `.416219` | `.457694` |
| 最终停车确认（含一秒静止窗） | `2.068 s` | `1.772 s` |
| 确认前额外 XY／绝对 yaw | `.117834 m / .073140 rad` | `.033116 m / .080269 rad` |
| 最终零消费尾部、保持至 END | `6.400 s`，通过 | `5.996 s`，通过 |

full angular norm 峰值分别 `.844533/.854957 rad/s`，仅诊断，不与 yaw `.8` 混比。实测停车读取完整线／角范数，独立仿真审计与原 SDK `measured_stop_confirmed=true`、software retirement 分别表达；原 hardware／physical acceptance 和 supervisor `physical_stop_confirmed=false` 不覆写。此次 body／collision 采样不是连续碰撞证书，也不证明未来速度域保证。

当前正式配置是 **v2、25 Hz 双 LiDAR、`.23/.30` 导航上限、`.38/.50` 内部 policy 上限和 `.65/.8` 隔离工程域**。可移植 `campus_navigation_regression.json` 现显式为 25 Hz／2254；通用大型资产和 `create_large_world()` 仍为 v2／10 Hz。复现须匹配实际封存候选，新源码配置不自动继承旧成功。新 2254 相遇历史容量只匹配 50＋25 Hz 实际快照格的离线采样窗口，不改变在线碰撞、租约、原 `.5 s` 近距或 legacy 指标。v64／v65 RTF `.324902/.323385`，双 ray 最长 wall gap 分别 `.315716/.314679 s` 与 `.302693/.302377 s`；v63 曾超原 `.5 s` receipt 期限，不保证其他负载下丝滑运动。长程多点、楼梯／跨层、实机制动与原厂 SDK 尚未验收。

## 修复与实验的当前边界

- 查询轴内完整真实 link 支撑包络修复了 world-AABB 人工角点的保守假拒；原身体 XYZ、形体、radius／height 和演员审核不缩减。v64／v65 的 8,309／7,994 份实际身体快照均通过，原 v59 超域／可能重叠和关闭失败仍保留。
- [三个 execution 消费者的账本隔离](verification/20261007/paired_writer_execution_scope_fix_offline.json)与 [Python 安全门修复](verification/20261007/safety_writer_execution_scope_fix_offline.json)已在 v65 完整取消／新任务中生效；旧 v53／v57／v59 失败不重判。许可、原 XYZ/C1 与停车限额未放宽。
- [geometry 撤权后的独立 Clock／MC 观察](verification/20261007/geometry_fault_stop_observation_offline.json)只提供原测量供 SDK 判断停止，不能恢复导航或正权限。该故障路径有 56 项离线回归；此次无故障到达／关闭不替代故障注入验收。
- **v3 没有被选择。** [真零输入组件 A/B](verification/20261007/spot_zero_policy_withdrawal_causal_ab.json)确证输入撤回，但两组持续停车／速度域均失败；v3 不能称官方 STAND 或可靠 park。v2 的最终零高层消费仍可能产生测量反馈 policy 输入，不能称 NN 两轴精确零。

[v59 原失败](verification/20261007/campus_restart_v59_actor_overlap_and_body_certificate_failure.json)的 full XYZ／yaw 峰值 `1.064634535/.886969864`、body Cube 假拒、可能演员 AABB 重叠和未确认退役各自保留。没有 contact-force，不能唯一归因于 PI、NN 或演员。以下旧组件与正式试验只适用于其原封存版本，不用新域或容量反算通过。

## 同冻结真零 A/B：输入撤回成立，物理验收失败

[公开原始对照与可检查脚本](verification/20261007/spot_zero_policy_withdrawal_causal_ab.json)状态为 **`INPUT_WITHDRAWAL_CONFIRMED_PHYSICS_FAILED`**。原 v59 到组件基线的 43,410 个 BEGIN 六字段逐值一致；基线到 v3 切换前的 **36,428** 个 position／quaternion／full linear／full angular／policy input／high-level command 逐值一致。全程 43,410 个高层命令、**5,210 份完整六演员记录**和切换前 **4,371 份完整 13-link 快照**分别核对一致。两组件均保留 source 0 起点、native counter 每 tick 加一、43,411 个 BEGIN＋真实 END 样本、4,341 次 source 推理及 100 次 bootstrap 推理；外部 UDP 关闭，原选择的 memory 配置保留，没有额外 memory 实验。

| 原同冻结组件结果 | v2 基线 | 延迟 v3 |
| --- | ---: | ---: |
| 72.856 s 后两轴真零 BEGIN 数 | 6,982 | 6,982 |
| 零高层轴仍有非零 policy 输入，每轴 | 6,982 | 0 |
| 首次连续一秒停稳确认 | 1.000 s，通过 | 1.000 s，通过 |
| 确认后首次再次越过静止范数，源时间 | 82.896 s | 81.178 s |
| 全 XYZ 峰值，原上界 `.65 m/s` | 1.064635 | 1.334362 |
| world yaw 峰值，原上界 `.8 rad/s` | .886970 | 1.377489 |
| 持续真零尾停车 | **FAIL** | **FAIL** |
| 首个采样 possible actor AABB overlap，源时间 | 82.818 s | 82.680 s |

真零 episode 在两个组件中均长 **13.964 s**；v3 末样本虽为零输入和低速，不能抹去中间再次运动与超域。v3 原零尾总 XY 行程 `.978993 m`、绝对 yaw 行程 `1.078463 rad`；基线为 `1.701217 m/.909378 rad`。这些行程是记录全尾诊断，首次确认仍按原 latency 3 s、额外 XY `.5 m`／yaw `.4 rad` 与完整 `.03/.05` 连续一秒判断。**精确零策略输入不是物理 STOP 证书。** v3 在首个采样 AABB overlap 之前已越过静止范数；没有 contact-force 记录，不能唯一归因于 PI、神经策略或演员推动。

证据保存实际两组 manifest、launch console log、原流 hash、审计脚本和 pre／post 源字节快照。原 launch manifest 没有捕获 Python source-tree hash；后续源 hash 闭包不冒称这一字段，也不重写旧记录。此为单次隔离组件对照，不是正式导航或硬件验收，旧 v59 仍 FAILED。

## 停车：确认一秒之后仍须保持

[加强后的原 500 Hz 审计](verification/20261007/query_support_and_zero_tail_offline.json)保留首次完整 `.03 m/s`／`.05 rad/s` 连续一秒确认，并检查余下每个真实 BEGIN 及最终 END 是否仍在原静止域内。**v52 仍 PASS**：确认 `2.138 s`，真零 episode 总长 `6.536 s`，确认后全部实际样本保持静止；确认前额外 XY `.0759005 m`、yaw `.0770616 rad`。**v59 durable zero-tail FAIL**：72.856 s 起初一秒确认通过，但 **82.896 s** 起又有运动，后续 1,356 个样本越过静止范数，真零全尾实际 XY 行程约 `1.701225 m`、绝对 yaw 行程 `.909463 rad`。新错误为 `recorded_stop_then_zero_tail_motion`，不是把初次确认时间或阈值改大。

该附加审计没有改写旧封存 reports。原 latency 3 s、确认前额外 XY `.5 m`／yaw `.4 rad`、完整线／角范数保持；真实停止与 mock `physical_stop_confirmed=false`、software retirement、进程退出分别记录，始终不宣称 hardware acceptance 或连续碰撞自由。

## 当前配置与历史响应

通用生成器与 [大型资产](assets/large_quadruped_scene.json)采用导航 `.23/.30`、内部 policy `.38/.50`、完整 XYZ reference／measured-travel／reachable `.65` 与 yaw `.8`，绑定 [原工程模型证据](verification/20261007/spot_velocity_domain_v47_native_navigation.json)。native actor targets、500 Hz pose／完整物理记录均显式封存；通用场景双 LiDAR 为 10 Hz，最新 v65 作者配置与更新后的可移植回归输入显式为 25 Hz。`.65` 是新隔离候选的工程接纳域，不是未来 full XYZ 保证，不重算旧 `.50/.60` 失败。约 332.84 m 离线路由不继承 v52 的短程成功。

UNKNOWN、完整三维碰撞、`.05` C1、绝对 join 与 source／receipt 有效期都未放宽。v59 地图证明中的 UNKNOWN／过期与本节零轴伺服缺项各自取证；不能用修改 policy 输入绕过原权限撤销。正请求 `.05` 是 memory-only 恢复资格门，**不是最低行走速度**。历史 v41 长正需求固定点、v45 碎片化需求与低速响应的证据范围见后文。

v57 物理记录把两类现象分开：32,887 个真实 500 Hz 样本中，正 vx 消费 9,274 个（约 **28.20%**），98 段连续正 vx 的最长源时间跨度仅 **`.966 s`**；最后正 vx 在 **32.074 s**。**35–60 s 原行走需求与 yaw 均为零**，35–45 s 全 XYZ 速度中位数约 **`.00733 m/s`**、XY 约 **`.000347 m/s`**，是已撤回 WALK 后的站姿响应，不能称持续正需求下的低速策略固定点。较早 **5–12 s** 正 vx 占 90.94%，desired／official policy input 中位数 **`.13821/.32016`**，实测 XY 中位数 **`.04115 m/s`**、净 XY **`.09903 m`**；v52 同源秒数窗口为 **`.22742/.25789/.23238`**、净 XY **`1.44162 m`**。两次任务、命令和权限历史不同，这项比较支持响应与持续性需要标定，不能单独证明唯一物理原因。原 `.05` 正需求＋完整 `.03/.05` 静止资格最长仅 **`.322 s`**，不足一秒，0 次恢复符合原门控。**资格门 `.05` 和真实零命令都不是已标定的最小行走速度。** 本机原流 hash／连续区间比较见 [v57／v52 原物理比较](verification/20261007/spot_v57_v52_physics_response_comparison.json)，公开摘要绑定原记录。

## 已确定的问题

1. `nav_prepare.py` 原先把导航需求固定为 `.15/.30`，覆盖场景配置。现在用封存的 `robot.max_linear_speed/max_angular_speed` 配置整条导航链。
2. 原速度反馈的内部输入上限 `.3/.5` 无独立配置入口。v37 真实运行中，导航需求约 `.13886`、内部输入饱和 `.3`，但关节和机身长期基本不动。现在增加独立、严格验证的 `policy_input_limits`；缺省仍保留历史值。
3. 巡航需求、策略输入、实际机身速度是不同量。步态中的真实 Z 速度必须保留；单看平均前进速度会漏掉瞬时峰值。
4. 完整 XYZ 参考域原 `.50` 小于部分正常步态瞬时速度。历史候选把 reference／measured-travel 与同一封存平台 reachable 域分别绑定为 `.60`；新 v47 工程候选依据完整原 v43 测量绑定 `.65`，完整标记、hash、命令上限和实机拒绝规则保留。原 `.60` 超域运行不重新判定。
5. v38 的优化器 setter 仍拒绝大于 `.5` 的参考域，导致 `.6` 配置在初始求解即抛异常，零 SDK 运动。后续版本修复这处遗漏，并记录异常原因。v38 失败记录保留。
6. v53 取消后重新导航的末段零输出已确证为跨 execution 的 writer commit／ACK 高水位残留，不能归为低级 policy 站立、速度域拒绝或 motion UNKNOWN。跟踪器与 SCAN 已按完整新授权身份隔离账本并保留取消后的迟到 ACK HOLD；离线修复不改变速度、ramp、完整 XYZ/C1 或停车限额。v57 已跨过 native scope 旧阻断；Python 安全门随后修复，v59 phase 1 实际成功 commit 1–28 但整例仍失败。最新 v65 完整取消／新任务按其原 14 项检查通过，不能据此回改旧失败。

## 原始实测

所有速度峰值来自连续 500 Hz 原生机身样本。停稳指真实需求撤回后，完整线速度范数 ≤ `.03 m/s`、完整角速度范数 ≤ `.05 rad/s` 连续保持 1 s；表内时间包含这 1 s 确认窗口。

| 策略直供输入 | 后 4 s 平均 XY 速度 m/s | 全 XYZ 峰值 m/s | 停稳确认 s | 最大停车 XY 位移 m |
|---|---:|---:|---:|---:|
| .3 | .3364 | .50804 | 1.574 | .08504 |
| .4 | .4311 | .57017 | 2.130 | .11306 |
| .5 | .5246 | .64316 | 1.724 | .13021 |
| .6 | .6218 | .73631 | 1.636 | .14910 |

`.5/.6` 的实际峰值超过原 `.60` 模型，因此本次没有选择它们。以上均有四脚真实腾空样本与完整已注册几何认证。单次直供输入不能证明所有历史、转向、恢复过程都有同样响应，也不能推出固定最低 WALK 速度。

新的闭环组件使用需求 `.25 m/s`、内部输入上限 `.4`，保留原 PI/filter：后 4 s 实测前进速度平均 `.27046 m/s`，全 XYZ 峰值 `.57595 m/s`，停车确认 `1.972 s`，额外 XY 位移最大 `.07770 m`。最大路径侧偏 `.06487 m`；按时间积分需求的最大沿向误差 `.32216 m`，超过旧 `.10 m` 跟踪预算，不能宣称跟踪验收通过。

历史组件结果支持继续在原导航链中测试 `.25` 导航需求上限、`.4` 内部上限、`.60` 完整 XYZ 参考域。原 measured spatial phase、换轨/C1、碰撞证据、源时间和停车条件仍负责正式接入；当时没有在安全门之后补最低速度，没有清空 UNKNOWN，没有设置机器人根位姿/速度，也没有选用动作记忆重置。后续 A/B 与资格保护门的证据另见下节。

机器狗需要能持续产生真实步态的控制输入，导航需求、内部策略输入和实测速度必须分别校准。这里的 `.25/.4` 都是上限，不是强制最低步速；闭环组件的完整 XYZ 峰值 `.575947 < .60` 只说明这一组直行试验落在所选域内。现有试验不能证明一个适用于所有启停、转向和历史的最低 WALK 速度，旧 `.10 m` 跟踪误差预算也尚未通过验收。

v39 正式 `crossing_blocker` 已修复 v38 的 setter 域冲突，产生 1284 条非零 SDK 实际输出、149 次原路由进度信用，累计实测 XY 行程 `5.812686 m`，最终仍为 **FAILED**：`execution_blocked_timeout:actual_command_blocked:motion_sweep_unknown_or_expired`，未到达目标。最后非零 SDK 输出在原场景源时间 `33.34 s`；从 `33.36 s` 开始出现该扫掠拒绝，原因摘要记录 4479 条负 motion proof，延续至 `63.90 s`。起始窗口中 tracker 仍 `tracking`，当前曲线 73 仍 `observed_free`，所以当前曲线证明与实际命令扫掠证明必须分别核查。早期曲线 71 的近地 UNKNOWN 属于先前演员未来可达管查询，不能据此解释最终扫掠失败。原日志对 UNKNOWN 固定打印 `first_cell_available=false`，这是只为 occupied 输出首格诊断的限制；最终阻断格和具体成因仍待真实捕获。

v39 原始机身采样还在源时间 `27.46 s` 记录完整 XYZ 峰值 `.609435 m/s > .60`，因此正式速度域也没有全程通过。原 mock 测量停车条件已确认：211 个样本、完整线速度 `.007459 m/s`、角速度 `.000280 rad/s`；Action 和退役记录的 `physical_stop_confirmed` 仍为 false，只有软件退役确认，不能宣称物理验收通过。这些失败、源时间、原始 SHA256 与诊断范围保留在 [v39 失败记录](verification/20261007/campus_crossing_v39_failure.json)。

## 复核入口

`full_scene_replay.py` 启动有界的完整场景组件，`policy_component_summary.py` 解析实际连续采样与停车窗口。运行输入和 Python 源码在各组件开始前另行冻结；新输入使用新结果目录。

公开数字与原始文件 SHA256 见 `verification/20261007/spot_velocity_domain_v38.json`、`spot_velocity_domain_v38_closed_loop.json`；v38 求解失败见 `campus_crossing_v38_failure.json`，v39 实际行走后失败见 `campus_crossing_v39_failure.json`。组件标定、源码测试、安装闭包和正式目标到达分别记录。

## v40 实际目标到达与残留接触阻断

v40 保持同一封存速度输入与园区。原 BT Action 成功到达，终点 `[7.8593,-4.9687,.4878]`，目标 XY 误差 `.14415 m`；完整 body/IMU、测量停车和软件退役成立。原低频观测 full XYZ peak `.595608 m/s`，完整 500 Hz 速度尚未记录，不能由此覆盖 v39 超域或宣称连续峰值验收。原动态精确同 pair 中心距离最小 `2.345758 m`，未达到原 2 m 近距曝光要求；旧完整 case 保留失败。

新原格诊断确认 prior3 地面接触格保留了 clean `plaza_person` HIT；严格更新的完整全 XYZ oracle 已与闭格分离，原 static/dynamic/双雷达租期都有效，原分支仍返回 UNKNOWN。修复只在原 actor-only 身份、完整静态接触证书与全动态否决均成立时恢复地面接触许可；未归属、混合、poison、其它演员、边界相交、过期和静态实体仍拒绝。原激光 odds/HIT/source/receipt 不改变。26 组 CTest、449 原生用例通过，正式修复版须新候选验证。

后续正常导航可以显式封存 `record_full_physics_history=true`，读每个真实 500 Hz BEGIN 与最终 END 的完整位姿、速度和策略输入。这个选项仅添加取证，保持原 UDP、BT 与唯一 writer；原安全零需求立即撤回前进 feedforward。新场景的 3 m 相遇曝光配置将独立绑定模型与实际形体，旧 2 m 结果保持，不修改运动安全或碰撞标准。公开原始 hash 见 `campus_crossing_v40_goal_and_contact_defect.json`。

## v41／v43：完整物理记录保留失败结论

[v41 原失败与站立窗口](verification/20261007/campus_crossing_v41_stand_and_velocity_failure.json)记录 Action status **6**、`execution_blocked_timeout:waiting_measured_motion_progress`，累计 XY **`12.720190 m`**，终点仍距目标 **`4.076640 m`**。原 **53,664** 次真实 native BEGIN 加末次当前 getter END，共 **53,665** 个完整 500 Hz 身体样本；全 XYZ 峰值 **`.605385180 m/s`** 超出 `.60`，没有裁剪。70–90 s 的 **10,001** 次原消费命令全部为正，forward 保持 **`.247192769 m/s`**、内部输入保持 **`.40000000596`**，机身净 XY 仅 **`.007272435 m`**，完整 XYZ 平均约 `.00763 m/s`。这段不能归为 UNKNOWN 撤权或 writer 零命令。原全流停车确认 **2.196 s** 满足停车模型，但速度域及导航失败仍保留。

[v43 原到达与速度失败](verification/20261007/campus_crossing_v43_goal_with_velocity_failure.json)记录 Action status **4**、`measured_goal_reached`，累计 XY **`16.427680 m`**、目标误差 **`.190184 m`**。完整 **48,031** 个 500 Hz 原样本中，全 XYZ 峰值 **`.616161871 m/s > .60`**，因此独立速度域 gate 和整例仍 failed；原停车确认 **1.368 s**、指定 3 m 采样遭遇成立，不能覆盖速度超域。该次使用 `.23` 导航上限／`.38` 内部上限，记忆保护门已配置但实际干预事件为 **0**，到达不能用来证明保护门完成了正式恢复。

## 全园区冻结命令 A/B 与新的恢复资格门

[全园区同冻结命令 A/B](verification/20261007/spot_full_campus_memory_causal_ab.json)使用同一原园区、身体、传感器、500 Hz／50 Hz 相位和冻结命令。原 v41 有 **53,664 ticks**；显式选择真实前缀 **53,660 ticks**，排除末 **4 ticks／8 ms**，先验证完整原 manifest／事件 hash 和所有尾段事件，再截取原 tick 小于前缀的事件，不补命令、零尾或时间戳。基线和实验组各有 **53,660 个 BEGIN＋1 个真实 END**，共用同一冻结输入 SHA。

基线的 **53,660 个 BEGIN** 与原 v41 在六组字段逐值相同：position、quaternion、完整世界线／角速度、official policy input 和实际 highlevel command。首次干预前 **35,520** 个样本，A/B 的上述六组字段逐值相同；这不表示整个 JSON 文件字节相同，epoch、实验描述与回放 authority 等元数据分别保留。实验组仅在原 source **71.040 s**、原推理边界清零一次 12 维 previous/current action memory，原 counter、PI、命令、root／joint 状态均不写。70–90 s 的真实净 XY 从基线 **`.007272435 m`** 变为实验组 **`5.256607278 m`**；原真零消费源至完整范数连续一秒停稳为基线 **2.196 s**、实验组 **1.922 s**。这支持该历史下动作记忆驻留参与了实际站立，并证明一次 memory-only 干预可以恢复；两组全流峰值都仍为 **`.605385180 > .60`**，没有正式导航授权或整例通过结论。

当前实现把正式干预资格收紧为：**原 forward 请求至少 `.05 m/s` 持续一秒**，同时原真实完整线范数不超过 `.03 m/s`、完整角范数不超过 `.05 rad/s` 连续一秒；微小目标修正、零或反向需求不算 stall。source tick 与原纳秒必须按 2 ms 一致，native tick 和 policy counter 的增量必须一致，session／epoch／anchor 固定；身份漂移、回退或计数不符永久关闭干预，合法缺样重新等待完整窗口。只在原 counter `%10==0` 推理边界处理，每个资格 episode 最多一次，不增加最低命令、不推进路线进度、不改变完整 XYZ／C1／证明、速度域或 STOP。保护门 **60** 项加 quadruped **24** 项，合计 **84** 项轻量回归通过；A/B 使用此前冻结的正请求实验 guard，不能把它写成新版正式资格门已经物理验收。正式恢复、速度域、到达与取消／重启按各自封存运行分别核对。

## v45：低进展、碎片化需求与恢复资格

[v45 原六演员失败](verification/20261007/campus_crossing_v45_all_actors_response_failure.json)使用 `.20` 导航上限／`.30` 内部上限。累计 XY **`2.672983 m`**，终点距目标 **`14.682844 m`**，原 Action status 6、`waiting_measured_motion_progress`；指定遭遇和整体干净退役也未通过。原 **28,349** 个完整 500 Hz 样本峰值 **`.448181 m/s < .60`**，真零后完整范数停车确认 **2.058 s** 独立通过，这不覆盖导航失败。

记忆事件为 **0** 与资格门一致：forward ≥`.05` 最长 **`.790 s`**，同时满足正请求与完整线／角静止资格的最长窗口只有 **`.306 s`**。20–30 s 与 40–48 s 原消费正需求占比分别约 86%／87%，但净 XY 只有 **`.043249/.004663 m`**；需求中断、原 steady receipt 过期和低响应均有记录。原双雷达最大 wall 间隔 **`.712242/.709844 s`** 超过 `.5 s` 租期，并有捕获 receipt 过期见证；这支持间歇撤权，不能据 reason 计数推断所有低进展都由过期造成，也不能直接套用 v41 长连续正需求下的固定点因果结论。

## actor 缓存与新 `.65` 工程候选

[缓存 A/B](verification/20261007/spot_actor_handle_cache_causal_ab.json)仅缓存原注册 actor 的 USD 句柄，原 `actor_pose(t+dt)`、500 Hz 写入、形体和冻结命令保持。12 s 回放中，6001 个机身样本的六组实测字段及 720 组完整演员读回相同；actor callback wall 从 **4.148648** 降到 **3.550822 s**，RTF 从 **`.304498`** 到 **`.318513`**。最大 ray wall 间隔仍为 **`.522582 s > .5`**，没有证据表明证明时效已解决。这是组件性能 A/B，没有在线导航到达验收。

[新模型证据](verification/20261007/spot_velocity_domain_v47_native_navigation.json)封存 v43 原 `.23/.38` 导航的完整 XYZ 峰值 **`.616161871`**，新 reference／measured-travel／reachable 工程域为 **`.65 m/s`**、yaw 为 `.8`。明确裕量 **`.033838129 m/s`** 是新隔离模型选择，不是未来速度上界证明；原 `.60` v43 整例仍 failed。导航 `.23/.30` 与内部 `.38/.50` 权限、完整 XYZ/C1、原 lease、停车和几何约束保持。新 token `isolated_spot_reachable65_3m_exposure_v1` 精确绑定 `.65` 模型与原证据／身体 hash；旧 `.60` token 和旧 2 m 结果不改变。v47 原 sealed projection 上界遗漏导致正式失败，旧结果保留；新离线修复及 v52 独立实测见下文。v46 未运行。

## v47 原失败与新 `.65` 投影修复

[v47 原失败](verification/20261007/campus_crossing_v47_projection_domain_failure.json)保留：记录和调用方均为 `.65`，冻结 native measured projection helper 却仍限制隔离 Spot 为 `.60`，在实际投影和地图查询之前拒绝，留下 residual NaN／queries 0。这是命令、参考域、实测搜索域之间的遗漏，不能把所有后继 `source_expired` 归因于真实传感器过期。原 68 个小幅非零消费 ticks／10 个 applied 状态保留，未称全部命令为零，原目标和整例仍失败。

[新投影离线闭环](verification/20261007/spot_reachable65_projection_fix_offline.json)核对 source、mirror、installed header 与实际重新编译的消费者。59 个 CTest wrapper targets 通过，其中 38 个 gtest targets 的 **990 个用例**通过；另有 5 个无 ROS harness 的 **170 项检查**、**14 个 SDK assert executable targets**及 **67 项 installed Python**通过。这些计数口径不同，不相加；两个启动 wrapper 退出成功但 `Ran 0 tests`，不提供启动行为覆盖。新上界只允许完整同 hash、显式隔离 Spot 标记中的 `.65`，旧 `.60` record 仍使用自身域，default/live 不变；原全 XYZ、`.05` C1、绝对 join、UNKNOWN、STOP 和双时钟期限保持。离线报告没有正式通过声明；v52 是另一次新封存候选的实际验证，不重算 v47 或旧 `.60` 失败。

## 原生演员目标与物理等价的边界

[原生 kinematic target 组件 A/B](verification/20261007/spot_native_actor_targets_causal_ab.json)复用同一原 v45 场景、冻结 12 s／6000 ticks 输入和 `actor_pose(t+dt)`。6001 个机身样本的六组完整字段与 720 组演员读回相同，epoch 等元数据不同；actor callback 从 **4.148648** 降到 **1.026224 s**，RTF 从 **`.304498`** 到 **`.339855`**。原 NPZ 的 11 个演员点 XYZ 有差异，最大 **46.671 μm**；其余七组 array fields 相同，不能写全部雷达数组逐字相同。该组件最大 ray wall 间隔 **`.523626 s > .5`**，没有证明实时期限已解决。

新后端 `physics.dynamic_actor_target_backend='native_kinematic_v1'` 显式选用真实 native kinematic targets，原 `t+dt`／order `-1`、CPU view 身份、完整 stage／形体审计与失败关闭保留；当前生成器／默认大型资产显式选择 native 后端；外部 spec 缺少该字段时仍按缓存 USD 分支，无 teleport 回退、无机器人 root／joint 写入。上述组件 A/B 不等于加强 guard 后源码或在线导航的物理验收；v52 的实际几何、速度、到达与停车按其独立运行记录判断。

## v52 正式速度与停车证据

v52 使用 `.23/.30` 导航权限、`.38/.50` 内部 policy 上限和同封存 `.65/.8` 工程模型，不增加最低速度。全 epoch **53,962 ticks／107.924 s source** 的 **53,962 个 BEGIN 加 1 个真实 END（共 53,963 个样本）**按原记录完整审计，全 XYZ peak **`.5693280713 m/s`**、world yaw peak **`.4560123682 rad/s`** 落在对应域内；full angular peak **`.8299419682`** 单独作为诊断，不混作 yaw 失败。STOP 的 **2.138 s** 包含真实完整范数连续一秒确认，额外 XY **`.0759005 m`**、yaw **`.0770616 rad`** 保留原停车限额。

这次原 Action 到达和独立 case gates 通过，记忆干预为 **0**，不能归为正式恢复的物理验证。六演员实际采样形体均参与审计，唯一 required `plaza_person` 的同 pair 接近—停留—离开成立，五背景未声称遭遇。双雷达 wall gap 仍超过原 `.5 s`，因此不宣称持续权限、连续碰撞自由或丝滑控制。v53 取消阶段通过，但重新导航实际行走 `1.889616 m`、359 条非零 applied 状态后，原 Action 以 `execution_blocked_timeout:waiting_admitted_nonzero_command` 失败；运行 exit 1、`clean_shutdown=false` 保留，跨执行身份的 writer commit／ACK 计数残留见原失败取证。
