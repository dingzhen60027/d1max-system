# PCT → SCAN 集成逐段源码审计

审计日期：2026-09-24。对照源是 Robot-Nav/legbot_3D_Nav 的固定提交
`f60606a4903bad58fca813f76072f9940a284d1d`，不是只阅读 README。
此记录不把 Gazebo 演示视为 D1 Max 的实机验收。

## 结论：先对齐验收对象

对方演示是一条运行中的闭环：连续位姿和点云 → 全局参考 → 局部重规划
→ 空间轨迹跟踪 → A1 强化学习运动控制 → 新位姿和新视角。

D1 当前 Web 实时入口是无运动规划预览：定位 → PCT → SCAN → RViz 候选轨迹。
`live_session.py` 不启动轨迹跟踪器；`live_view.py::tick` 持续发布
`execution_frozen=true`。这个隔离不能悄悄去掉，但静止场景的候选几何通过
不等于已完成移动中的路径推进、重规划衔接和跨层跟踪。

## 1. 真正的启动与数据入口

[local_planners.launch](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/launch/local_planners.launch)
选择 SCAN 时先启动 `reference_path_transform`，把 `/pct_path` 转到 `odom`，
再包含 `scan.launch`。默认运动数据是 `/Odometry_gazebo`，点云是
`/livox/Pointcloud2`。

[simulation.launch](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/launch/simulation.launch#L5)
默认启用 Gazebo 真值。FAST-LIO 是另一种显式输入，不能将该默认演示说成
经历了真实传感器漂移和地图重定位。

[scan.launch](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/launch/scan.launch)
将同一个 odom 同时送到 `body_pose` 和 `sensor_pose`，`cloud_is_world=true`。
这是演示中的原点近似，不是可直接迁移给 D1 双雷达的标定方法。

## 2. 地图、PCT 路径与高度约定

[planner_wrapper.py](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/PCT_planner/planner/scripts/planner_wrapper.py#L180)
先选起终点高度层，调用多层 A* 和轨迹优化，再读取优化器的高度。
默认 quintic 分支的
[gpmp_optimizer.cc](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/PCT_planner/planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc#L171)
使用地面高度加 reference height 后平滑；不是所有 PCT 输出都天然等于地面 Z。

[reference_path_transform.cpp](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/src/reference_path_transform.cpp#L34)
在路径到达时以 latest TF 逐点转换；输出 latch 保存，默认 `z_offset=0`。

[SCAN pathCallback](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L411)
还有一次明确的高度对齐：当首点与 odom 高差不超过 0.8 m 时，整条路径平移
这个 Z 偏差，保留相对爬升。它处理的是参考高度约定，不能证明外参正确。

D1 应先量化 rosbag 的 `body_z-ground_z`，分别查看楼层和楼梯，确认雷达位姿、
机身位姿及 PCT 地面参考的含义；不要照搬自动挪高来掩盖错误。

## 3. 全局参考保角，局部轨迹再平滑

[pathCallback 的抽稀](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L438)
保留间隔至少 0.5 m 的点、超过 20° 的拐角及终点，将实际 odom 作为当前起点。

[planGlobalTrajWaypoints](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/planner_manager.cpp#L355)
默认使用 piecewise-linear 全局参考，保留 PCT 转角和楼梯形状。
局部 SCAN B-spline 负责最终平滑，避免又一次高阶全局拟合切过墙角。

D1 当前 `DiscreteReference` 只去掉极近的重复点，保留细碎变化；同时新增
`lambda_reference=20`，而对方没有该代价项。它可能将参考噪声传入曲率，
需要单变量对照，尚不能在未回放前认定为根因。

## 4. 局部目标与进度

[getLocalTarget](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L1095)
将实测位置投影到全局参考；使用 XY 距离加 8 倍 Z 距离避免上下层混淆，
进度单调，沿弧长前看 2 m。若局部目标被占用，沿同一参考寻找前后可用点。

[planFromCurrentTraj](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L866)
在参考路径模式下从实测 odom 位置、速度重规划，保留全局参考，不把旧轨迹
中的计划位置当成机器人真实位置。

## 5. 避障地图和优化的真实能力边界

| 参数 | legbot 演示 | D1 当前无运动预览 |
|---|---:|---:|
| 局部 horizon | 2 m | 6 m |
| 地图范围 | 10×10×5 m | 12×12×6.4 m |
| 体素大小 | 0.10 m | 0.08 m |
| 双圆半径 | 0.15 m | 0.29 m |
| 前后圆偏移 | 0.05 m | 0.20 m |
| Z 碰撞范围 | ±0.03 m | 上 0.15、下 0.45 m |
| 速度上限 | 0.50 m/s | 0.30 m/s |
| 未知体素 | 非占用 | 需要自由空间观测 |

[grid_map.h](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_env/include/plan_env/grid_map.h#L357)
仅查占用/膨胀，没有 D1 的 observed-free 条件。
[cloudCallback](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_env/src/grid_map.cpp#L883)
使用最近 sensor pose，而非精确 cloud 时间配对。

[A*](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/path_searching/src/dyn_a_star.cpp#L161)
以 XY 搜索附加起终点插值 Z；
[B-spline 优化](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/bspline_opt/src/bspline_optimizer.cpp#L1154)
清零 Z 梯度。因此不是任意三维落脚地形搜索。

不能复制更小碰撞包络和未知自由策略，让 D1 看起来更容易绕行。应当修正
真实观测、射线原点、局部范围与可执行段的关系。

## 6. 真正闭环：空间跟踪器和运动控制器

[a1_cmd_adapter.cpp](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/a1_cmd_adapter.cpp#L211)
每 10 ms 执行：按实测 odom 找局部 B-spline 上的单调最近点，前看 0.55 s；
速度前馈叠加位置反馈。朝向误差超过 0.8 rad 时先转向，并反馈执行冻结。
世界速度转换为机身 XY，允许横移；局部终点检查 XY 和 Z。

它还给楼梯设置 0.3 m/s 最低命令速度，这是 A1 RL 的经验设置，不是 D1 通用参数。
[State_RL_test.cpp](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp#L245)
将 `/cmd_vel` 的 vx、vy、yawrate 输入 A1 policy；不能将这层直接移植为 D1 SDK。

D1 现有 `d1max_trajectory_tracker` 是独立的单层实现，当前实时入口没有启动。
其原有 `tracker_core.hpp` 按时间推进，仅用目标 XY 判断到达，垂向误差 0.5 m
触发 single-floor 失败，输出 forward-only；直接拿来跨层会有同 XY 上下楼目标
提前完成、楼梯跟踪脱节等风险。需在无运动输出隔离下补齐并验证契约。

## rosbag 验证顺序

1. 输入和连续性：检查真实 body/雷达位姿、时间、地图坐标、地面高度。
   地图匹配状态与连续位姿有效性分开；不因顶层一个 matcher 瞬时状态丢弃
   导航层已经验证的连续位姿，也不延长过期样本或跨 epoch 沿用旧结果。
2. 局部范围单变量对照：6 m 与 2/2.5 m，保持机身尺寸和未知策略不变。
   记录目标可观测率、接受率、首个失败点、局部进度及 p95 求解耗时。
3. 参考形状单变量对照：原参考与保角抽稀、reference 权重 20 与 0/低值；
   查看局部曲率、净空、地面支撑和是否真正绕过障碍，而非只看 accepted。
4. 运动影子链路：用 bag 实际运动驱动进度和跟踪，记录 global/local arc、
   到参考距离、新旧曲线衔接、建议速度与暂停原因，不连接 SDK。

bag 不会响应建议速度，因此只能验证录制视角下的输入、规划与时序一致性。
绕开录制路线之后的新视角、真实足端接触和运动闭环仍需仿真及后续实机验证。

## 真实移动边界审查与修复

第一轮固定输入 35 s 对照中，6 m / 2 m 严格未知空间和 2 m 仅占据阻挡三组
都没有接受曲线；将标称速度由 0.3 改为 0.6 m/s 后只有 8 条。2 m 已减少
远处未知空间影响，但 `failed_dynamics` 仍占主导。日志明确显示第一轮优化
成功后，被 `moving initial boundary forbids uniform retiming` 拒绝，甚至
真实初速仅约 0.01 m/s 时也出现，不能全部归因于记录中最高 0.496 m/s 超限。

### 原因

1. 上游参数化把采样点与起止导数放在一个超定最小二乘问题中求解，导数只是
   拟合目标，不是等式约束；优化器随后固定前三/末三控制点，不能再修复边界误差。
   [parameterizeToBspline](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/planner/bspline_opt/src/uniform_bspline.cpp)
2. 上游时间可行性初筛按速度/加速度的各坐标分量并带 tolerance 检查；D1 增加的
   最终门限是严格的三维范数控制点上界。初筛通过不等于最终通过，可能完全跳过
   原生时间重分配，然后遇到“移动初态不允许统一减速”的必然拒绝。
3. 原生 `lengthenTime → 重采样 → LS 参数化 → refine` 仍不保证真实起始速度和
   加速度严格一致。上游配置的速度/加速度额外 tolerance 都是 1.0，不能把演示
   接受率直接理解为“整条轨迹严格不超过标称速度”。

### 修复原则与实现

对 guided 路径先固定起止位置、速度和加速度，只拟合中间控制点。对于均匀三次
B-spline，起点三个控制点为 `p−vΔt+aΔt²/3`、`p−aΔt²/6`、
`p+vΔt+aΔt²/3`，因此改变 Δt 后也能保持同一个真实运动初态。

移动中的曲线不再采用统一缩放后拒绝的流程，而采用最多 8 次有界迭代：
根据范数上界增加 knot interval，重新求解硬边界内点拟合，运行原生内点 refine，
然后复核起止边界与全时域速度/加速度上界。因为重优化可能改变空间形状，接受前
仍执行现有完整曲线、机身体积和姿态扫掠碰撞检查。失败不修改输入轨迹，不发布
未经复核的候选。真正静止时保留原先可保持空间曲线不变的统一时间缩放。

若实测初速已经超过标称上限，保持 `v(0)` 与“全时段不超过更低上限”不可能同时
满足；现在明确报告初态超限，不将测量归零、不静默提高整个曲线的限速。真实运动
需要另行设计可观测自由空间内的减速恢复策略，不能把它伪装成普通规划成功。

纯 C++ 回归包括低速移动重分配、真实 0.496 m/s 初速、各轴未超但范数超限、
不同 Δt 下完整位置/速度/加速度边界、优化器破坏边界、失败不修改输入、迭代预算。
本轮编译后 7 个相关测试套件、64 个案例通过；真实 bag 接受率须以第二轮复测为准。

### 第二轮结果及不能掩盖的限制

同一实际输入下，2 m、严格未知空间、0.3 m/s 配置由零条接受曲线变为 26 条，
可见曲线时长约 26.41/35 s，动力学失败约 2.36 s；说明原来的移动时间分配
矛盾确实是主要阻断之一。0.6 m/s 的严格配置只接受 4 条，仍有大量加速度
上界 0.354–0.435 m/s² 超过配置的 0.35，8 次有界迭代不能收敛。不能把
提高标称速度当作修复，也不能删除加速度验收；应后续针对这些失败实例改善
范数动力学目标和时间分配收敛，而非无限增加迭代。

第二轮后审计还发现 FSM 在当前速度与目标方向点积为负时会 `setZero()`。
其中有若干 0.004–0.025 m/s 的真实反向初速因此被改写；上述第二轮结果只能
证明“传入 manager 后的边界保持”，不能宣称所有录制测量边界都被保持。
该旧逻辑由另一独立修复消除，最终结果需以第三轮原始反向初速复核为准。

第三轮移除参考引导分支的反向速度置零后，同一 35 s 记录（310 条 odometry、
126 条点云）接受 27 条曲线，可见接受状态 26.682 s；动力学拒绝 2.085 s，
参考搜索失败 4.203 s。27/27 起始位置与原始源位姿一致，没有改写输入速度，
速度边界最大数值误差约 7.22×10⁻¹⁶ m/s，包含原先被清零的反向小速度。
最大曲线速度 0.30000000000000054 m/s（浮点舍入量级），最大加速度约
0.336985 m/s²；4 次真实初速超过 0.3 的请求明确拒绝，零次时间迭代耗尽。

这仍是 **真实记录输入驱动的局部规划回归**：参考沿录制后续轨迹构造，并不是
PCT 全局规划、定位、机器人响应控制指令的端到端闭环。起始数据源龄最大约
0.22956 s，也不能把该原生隔离回归冒充完整 50 Hz 定位租约验收。
结果保存于 `log/scan_bag_replay/20260925_native_actual_reverse_boundary_01/report.json`，
逐条边界核对与图在 `20260925_native_actual_reverse_boundary_summary/`。

### 连续位姿的消费边界

导航输出的 50 Hz compact pose status 携带真实输出 stamp、epoch、确认 seed
和 0.08 s TTL；5 Hz UI/map-match 状态不是连续输出租约。消费者同时验证
源时间和本地 monotonic receipt，receipt 不能错误沿用普通状态的 0.5 s
超时。慢状态的 `valid=false` 不单独撤销尚有效的高频 pose，但真实 fault、
reset_pending 或 epoch/seed 不一致必须立即撤销。

收到非法格式/远未来时间包时撤销当前授权，但不把非法 stamp 写进顺序水位，
防止正常数据恢复后永远被当作旧包。body Odometry 本身没有 epoch 字段，
因此必须严格新于本地 epoch/sensor barrier，且不能倒序；不能在 epoch 改变后
把延迟的旧 body 数据直接贴上新的 context。这里只管理离线规划/可视化的输入
有效性，软件中 `motion_control_enabled=false` 保持不变，不构成实机运动验收。
