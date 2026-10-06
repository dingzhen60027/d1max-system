# Isaac Sim 三维导航任务测试

分支：`isaacsim-simulation`。比较基线：`5dfaf8cddec952b3453a02e3a231de5228bd6017`。这是同一导航主线的 Isaac Sim 仿真候选与取证分支，当前仍有未解决的换轨失败。

**最新状态（北京时间 2026-10-07）：本机默认候选为 `isaac-candidate-v31-campus`。** Spot 原 Action 近距离到达实测 **1.040435 m**，返回 `measured_goal_reached`；v31 `cancel_and_park` 实测 **1.047985 m** 后返回 `action_cancelled`，最终 **2.018 s 源时间**静止窗口与独立关卡评估通过。两次停车、IMU 与全身检查均通过。v31 `crossing_blocker` 实测累计 **0.403687 m** 后以 `execution_blocked_timeout:waiting_measured_motion_progress` 失败：准备运动证明有 14 条通过、7 次成功换轨、零准备预算耗尽，v30 的准备通道饥饿不再出现，但新候选几何入口的低速／方向变化及持续进度仍需修复。其余四关未执行，不能宣称动态避障或全部六关通过。[v31 实测摘要](verification/20261006/campus_navigation_v31.json)和 [横穿失败审查](verification/20261006/crossing_failure_audit_v31.json)分别保存结果与分析。文档日期为北京时间，证据目录 `20261006` 保留 UTC 日期。

v30 的近距 **0.827614 m** 到达、取消关卡 **1.197793 m** 行程及最终 **2.02 s 源时间**静止窗口通过，横穿 **5.583955 m** 后失败，均保留为 [历史 v30 会话证据](verification/20261006/campus_navigation_v30.json)，与 v31 的独立会话分别记录。

**历史未解决失败：** 旧轮式 `live_view_20261006_001` 实测行驶 **8.0010 m** 后因 `execution_blocked_timeout:waiting_admitted_nonzero_command` 失败。记录显示 generation 7 → 8 的换轨未完成；根因尚未确证，也不能据此断言由 SCAN UNKNOWN 引起。实测停稳与任务软件退役成立。[历史失败取证](verification/20261006/live_view_failure.json)随本分支保存；完整原始日志和大体积测量仍在本机证据目录。

历史 v19 三项任务和 v22 录屏的成功记录保留，适用范围限于各次封存候选与会话。[历史验证摘要](verification/20261005/historical_verification.json)随分支保存；这些成功不能抵消此次失败或证明连续换轨已经稳定。

新增的 **100×80 m 官方 Spot 园区**与六个系统测试关卡见 [大型园区及可复现关卡说明](CAMPUS_SCENARIOS.md)。场景有 85 个静态 box、地板／天花板和 6 个确定性动态几何演员；五点路线 **332.84 m** 目前是离线几何长度；导航 command 限 0.15 m/s／0.30 rad/s，长程观察预算 3600 s 源时间，原 BT 保护期限保持。v31 取消停车关卡通过、横穿关卡失败，其余四关未执行；v30 结果单独保留。旧轮式任务结果与大型机器狗场景的验证分别保留。

大型场景的入口顺序为 `scenario_suite.py list` → `prepare --case ID --output-config 新路径` → `build_candidate.py --scene-config 准备配置 --output 新候选` → `scenario_suite.py run --case ID --candidate 新候选 --session 新会话` → `evaluate --prepared-config 候选内最终配置 --session 会话 --output 新报告`。完整路径、构建参数和待验证证据要求均在上述说明中；每个关卡封存自己的模型、地图和演员集合，多点任务保持同一原导航图与同一物理会话，缺证据保持 `pending`。

本目录把本仓库的导航主线接到本机 Isaac Sim 6.0.1-rc.7，默认使用官方 Spot 与匹配行走策略（500 Hz PhysX、50 Hz 策略、50 Hz 原始机身状态）；旧轮式场景保留。任务仍由原 BehaviorTree.CPP 管理，经过 PCT 全局路线、连续参考、SCAN 三维局部规划、轨迹跟踪、安全门和原 SDK-free writer，最后驱动真实关节。行走策略的实测速度反馈只调节低层策略输入。

本次重点是导航任务和物理执行反馈。定位输入使用 PhysX 测量的真实机器人位姿，标记为 `groundtruth_fixture`；雷达和 IMU 使用实际仿真传感器。Faster-LIO、ICP 和机器人原厂 SDK 的性能不属于这组测试结果。

## 本分支相对基线的变化与边界

基线已具备 BehaviorTree.CPP 任务管理、PCT 全局三维路线、连续参考、SCAN 双圆柱局部规划、整曲线及偏航扫掠验证、换轨事务、跟踪、安全门、唯一 writer 和取消退役机制。本分支沿用这条控制链。

| 本分支新增或修复 | 代码与说明 |
| --- | --- |
| 100×80 m 园区、官方 Spot 真实关节行走、双三维雷达与原生 IMU | [场景](scene.py)、[Spot 控制](quadruped.py)、[原接口桥接](bridge.py)、[候选组装](build_candidate.py)；定位是显式 PhysX 真值夹具 |
| 认证静态空体积与实时射线融合 | [静态先验加载](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/static_occupancy_prior.cpp)、[GridMap 查询](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/grid_map.cpp)、[USD 体积生成](truth_map.py)；真实 hit 撤销先验 FREE，默认严格模式保留 |
| 查询性能与失败取证 | 同一快照的完整列批量查询与稠密体素缓存；资源计数使用实际完整扫掠体积，保留原几何范围、计算预算和证据期限 |
| 准备证明轮内调度 | v31 prepared 使用原 50 ms 轮内未用时间，最多沿用原 25 ms motion cap；5 ms 预留、当前轨迹优先级和全部原 deadline／证据检查保持，见 [离线回归](verification/20261006/prepared_lane_offline_v31.json) |
| Spot 水平跟踪与全身证明 | 已认证平面支撑上的逐区间水平制动包络保留 XYZ 曲线；实际 primitive 支撑函数计算 world AABB，修正旋转局部包围盒导致的脚球假穿地 |
| 源时间、进度与取消修复 | 原始整数纳秒、合法重复仿真 tick、同 tick 取消 ACK、原始位姿证据和规范化计算分离、弯道物理进度判定；细节见 [分析记录](SCAN_UNKNOWN_ANALYSIS.md) |
| 单 RViz 显示与仿真时钟诊断 | RViz 初始化与时效判断修复；下文保留历史 v22 轮式录屏，当前按用户要求不录制 |

Spot 新增 13 个实际碰撞体的同源全身包络证据、固定平面 `support_contact` 体素（不会当作雷达 FREE）、独立演员 oracle 否决和采样几何分离报告。动态消息的 source／steady 租约为 0.20 s，预测包络另覆盖 6 s，必须覆盖原反应与制动模型；两者不会相互续期。缺演员、新增碰撞体、时间倒退、证据过期均拒绝运动。默认 production 严格模式不启用这些仿真证书。

尚未验收：旧轮式 generation 7 → 8 换轨失败及 v31 横穿任务的入口几何与持续实测进度问题；真实 LIO／重定位条件下的先验配准；完整动态避让与四项未运行关卡；坡地、楼梯、跨层执行；原厂 SDK 与实机刹停。当前固定单位 `map←odom`、固定平面接触与仿真演员真值是明确测试条件。早期独立 Spot 组件测试满足原 `.03 m/s / .05 rad/s` 连续一秒阈值用时 2.418 s，更严格静止阈值用时 4.328 s，见 [历史时间窗摘要](verification/20261006/spot_stop_window.json)。v30 与 v31 取消会话分别以至少两秒源时间验证最终停车；均不作为实机制动验收。所有仿真结果保留 `physical_acceptance=false`。

本 README 中仓库内代码与最新失败摘要使用相对链接。标为“本机证据”的绝对路径用于保留原始会话索引，GitHub 无法直接读取；完整视频、运行日志、候选安装及历史测量没有作为仓库文件发布。

## 历史 v19 同版任务实测结果（2026-10-05）

2026-10-05，同一封存候选 `isaac-candidate-v19` 的三项原导航任务全部通过。完整性清单 SHA256 为 `49c969180b0fa7b96dea83f6d5af8a063c299b68bcfc5fef61935eaf34291a1e`。后续下方 `v22` 录屏所用的导航控制安装文件与 v19 相同，安装差异限于 RViz 显示插件。

| 任务 | PhysX 实测行程 | 原 Action 结果 | 原始报告 |
| --- | --- | --- | --- |
| 近距离到达 `(-2.8,-3,0)` | 1.0245 m | `measured_goal_reached`，status 4 | 近距离报告（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_goal_v19_001/isaac_smoke_report.json`） |
| 跨门到达 `(4,-3,0)` | 8.8118 m | `measured_goal_reached`，status 4 | 跨房间报告（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_cross_room_v19_001/isaac_smoke_report.json`） |
| 行驶后通过原 Action 取消 | 0.2826 m | `action_cancelled`，status 5 | 运动取消报告（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_motion_cancel_v19_001/isaac_smoke_report.json`） |

三次均确认任务软件退役、同一执行身份的 writer 实测停止、PhysX 实测静止和正常退出。任务取证边界内的独立 IMU 引用与实际收到的传感器源时间全部精确匹配，最高实测速度均低于原 0.30 m/s 限制。到达使用原任务容差；取消在累计实测行驶超过 0.25 m 后请求，不以目标距离判定取消成功。下方 v22 录屏另核对单个 RViz 与 Isaac Sim 的同会话实时画面。

在这些历史会话中，认证静态体积补足了近身长期盲区的空体积证据，原 v9 的常驻未知阻断得到解决。原 v9 首个候选位置的 3740 个查询体素中，1834 个从未观测，其中 798 个确定在机器人实体外。新增 `validated_static_prior` 读取实际 USD 的完整三维占据与空体积，实时 hit 优先否决，几何、定位身份或扫描期限失效仍阻断。原实机默认严格模式保留。这个结果不表示所有 UNKNOWN 或最新换轨失败都已解决。算法和证据详见 [SCAN 未知体素研究](SCAN_UNKNOWN_ANALYSIS.md)。

任务复测还修复了整数源时间、重复仿真 tick、取消 ACK、原始位姿证据、四元数计算和弯道进度判定。v19 跨房间的 3443 条 `RouteProgress` 与同源原始位姿逐项相等，锚点与 map geometry revision 在本次记录中始终为 1。历史 v9–v18 的失败及分版成功记录保留，不能用它们代替历史 v19 同版三测，也不能据此覆盖最新失败。

独立真值审计覆盖三次运动的机身、脚轮与车轮保守包络；审计检查离散实测姿态，不提供采样间的连续碰撞证明或 PhysX 接触事件记录。2026-10-05 记录的六个原生导航包 809 项测试、原 SCAN 156 项独立回调检查、SDK 14 个测试目标均通过；修改过的 Python 测试为 281 通过、2 项跳过，仿真夹具 68 项通过。完整证据与哈希见 历史 v19 验证清单（本机证据：`/home/eric/wjg/d1max-build-isaac/unknown_research/verification_prior_20261005.json`）。`physical_acceptance=false` 和原 mock 的 `physical_stop_confirmed=false` 保持，实测停稳与硬件验收分别表达。

## 当前机器直接启动

```bash
cd /home/eric/wjg/d1max-system
bash simulation/isaacsim/launch.sh --headless --rviz
```

本机仿真选择器使用已通过 Spot 近距离任务的 `isaac-candidate-v31-campus`。原仓库的单个 RViz 显示三维地图、任务和规划轨迹；在 RViz 设置 XYZ 目标，预览后通过原面板确认执行或取消。目标 Z 是**地面支撑点高度**，本园区地板用 `0`；Spot 名义机身参考高度 `.52 m`，真实步态高度单独保留。两个窗口同时显示使用：

```bash
bash simulation/isaacsim/launch.sh --rviz --render-fps 3
```

复现已通过的 v31 Spot 原 Action 近距离到达（实测行程 `1.040435 m`）：

```bash
bash simulation/isaacsim/launch.sh \
  --candidate ../d1max-build-isaac/isaac-candidate-v31-campus \
  --headless --smoke --goal -43 -4 0 --smoke-duration 300
```

v31 近距离任务沿用默认 smoke 停车观察，其通过结果见 [v31 实测证据](verification/20261006/campus_navigation_v31.json)。v31 调度回归 ledger **39/39**、非 launch CTest **15/15** 通过；仿真纯 Python 回归 **232 项通过**，均不能代替实际关卡验证。成功换轨保留当前输出与真实 writer 应用历史；横穿失败记录不支持“换轨无条件清零 ramp”的结论，新几何入口的前进需求和方向变化须另行调查。

v31 取消会话 `campus_cancel_v31_001` 原 Action 返回 `action_cancelled`，实测行程 **1.047985 m**；最终 122 个停车样本覆盖 **2.018 s 源时间**，最大源间隔 20 ms，独立评估 `passed`，实测停稳、IMU 与全身检查均成立。近距与取消短测未要求演员近距离遭遇，两次成功不能声明动态避障通过。当前可见 v31 Isaac 与单个 RViz 使用 3 fps 绘制以降低 CPU 争用，物理和传感器源频率保持。

历史 v30 `cancel_and_park` 使用专门的两秒源时间停车合同：报告的 991 个状态逐项匹配同源 PhysX 原始姿态和完整三维速度，1,653 个独立 IMU 样本与原生日志一致；1,107 个全身快照审计无失败。最终 122 个停车样本覆盖 2.02 s，最大源间隔 20 ms，完整线速度／角速度最大为 `.007147 m/s / .022544 rad/s`，低于原 `.03 / .05` 阈值。演员离这些短测路线较远，不能由此声明动态避让通过。v29 近距离成功仍保留在 [历史 Spot 短测](verification/20261006/campus_navigation_v29.json)；其后取消关卡出现脚球假穿地，v30 已修正完整 primitive 的世界包围盒，见 [几何回归](verification/20261006/primitive_world_bounds_offline_v30.json)。历史结果见 [v30 实测证据](verification/20261006/campus_navigation_v30.json)，不作为 v31 取消关卡通过证据。约 333 m 和六个系统关卡的范围与入口见 [园区测试说明](CAMPUS_SCENARIOS.md)。当前按用户要求不录制视频。

## 历史 v22 同会话真实录屏（2026-10-05）

`v22` 录制了约 2 分 42 秒的正常速度窗口视频：并排版（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/overview.mp4`）、RViz（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/rviz.mp4`）、Isaac Sim（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/isaacsim.mp4`）。轮式机器人通过原任务接口从 `(-4,-3)` 穿门到 `(4,-3)`，实测行程 8.8165 m，原 Action 返回 `measured_goal_reached`，实测停稳和正常退役均确认。

录制时修复了 RViz 初始化相机切换导致的属性悬空崩溃，以及诊断面板错误使用墙钟判断仿真状态过期的问题；70 项显示插件测试通过。总览包含双雷达的原始实测点云。GUI 使用官方纯绘制更新，物理仍为 120 Hz；6109 次绘制额外物理步数为 0，最长状态和扫描间隔分别约 96 ms、162 ms。录制数据和任务报告的哈希见 录屏验证清单（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/navigation_evidence.json`）。

`record_windows.py capture` 只接收两个显式 X11 窗口 ID，使用同一墙钟时间轴录制，SIGINT 正常封装；`postprocess` 生成并排版。视频时间轴用于对照画面，导航验收仍使用原任务报告及真实传感器源时间。

### 历史轮式夹具的任务命令

以下目标属于 12×10 m 轮式场景。复现时显式选用保留的轮式候选；Spot 园区目标与六关卡入口见 [园区测试说明](CAMPUS_SCENARIOS.md)。

自动近距离到达和取消测试：

```bash
bash simulation/isaacsim/launch.sh --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --smoke
bash simulation/isaacsim/launch.sh --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --smoke --smoke-case cancel
```

跨房间完整执行测试：

```bash
bash simulation/isaacsim/launch.sh --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --rviz --smoke \
  --goal 4 -3 0 --smoke-duration 360
```

另有独立的跨房间路线预览取消用例：

```bash
bash simulation/isaacsim/launch.sh --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --rviz --smoke \
  --smoke-case preview_cancel --goal 4 -3 0 --smoke-duration 120
```

此用例不发送执行确认，要求真实 Action 返回已取消、原工作节点退役且机器人始终静止；它与运动后的取消测试分别记录。

每次使用新会话目录，默认在 `../d1max-build-isaac/runs/isaac_时间/`。终端会打印日志路径。`isaac_smoke_report.json` 检查原任务结果、执行身份、writer 停止反馈和 PhysX 实测静止；不能只凭“发过速度”判定通过。`Ctrl+C` 先让原任务管理者退出，再关闭传感器、物理场景和私有路由。

IMU 取证在固定测量边界冻结状态集合，再给独立 IMU 回调最多 1 s 实际时间完成接收；所有引用时间必须精确匹配实际收到的 IMU，不从原生日志补填。边界和缺失源时间写入 `imu_evidence`，内部缺样仍判失败；该接收等待不延长导航或物理执行的有效期。v19 近距离测试的 841 条原始状态含 2 条早于首个独立 ROS IMU 接收的启动前缀，边界内 839 条全部匹配；这 2 条在原生 PhysX IMU 日志也有精确同源记录，任务发出后的 498 条状态全部匹配。前缀不补填进 ROS 接收集合。

## 历史轮式场景和传感器

场景为 12×10 m 室内空间，含两个门口、隔墙、柜体、低障碍、柱体和真实碰撞天花板。机器人是自建差速轮模型，具有实体轮关节和脚轮。查看 [场景预览](assets/overview.png)，或查看 [静态环境模板 USD](assets/indoor_scene.usda)。包含实体机器人、双雷达、IMU 和全部物理配置的实际初始场景已导出为 完整轮式场景 USD（本机证据：`/home/eric/wjg/d1max-build-isaac/runs/task_cross_room_v19_001/physics/indoor_scene.usda`）；控制与 ROS 会话由启动脚本连接。

| 项目 | 配置 |
| --- | --- |
| 三维激光雷达 | 前后两个 PhysX 多线雷达，水平 360°、垂直 160°，1°×5°角分辨率，10 Hz，0.06–15 m |
| 雷达原点 | 机身坐标 `(0.42, 0, 0.15)` 和 `(-0.42, 0, 0.15)` m |
| IMU | 原生 PhysX IMUSensor 120 Hz 测量，按 100 Hz 目标选取独立样本；机身中心安装，SI 加速度、角速度和测得姿态 |
| IMU 重力约定 | 直立静止时 Z 轴 specific force 约 +9.81 m/s² |
| 物理步进 | 120 Hz；传感器保留原始测量时间，不用接收时刻刷新样本 |
| 机器人 | 实体机身 0.76×0.44×0.20 m，轮半径 0.26 m、轮距 0.54 m，总轮宽 0.60 m |
| 执行限制 | 沿用隔离任务 0.30 m/s、0.50 rad/s 上限 |

完整 XYZ 回波、实际 33 线 ring 编号及每束原点进入原 `per_sensor_rays` 感知接口；三维地图包含地面、墙、柜体和天花板，保留高度、可通行层和头顶净空。自体过滤只依据已知仿真实体形状，不补造被机器人遮挡的自由射线。PhysX 雷达是瞬时扫描，隔离会话显式允许真实零束时间偏移；实机的扫描时长检查仍然保留。

每个雷达的原生采样矩阵为 360×33，在真实 10 Hz 采样的物理步启用完整 PhysX 快照，其余物理步不复用旧扫描。保留原生 `time` 和 `physics_step`，并检查它与实测机身状态来自同一步。IMU 的 100 Hz 是从 120 Hz 原生测量选择的输出目标，相邻样本间隔为 8.33 或 16.67 ms；不会插值或复制样本来冒充机器狗原配置中的 200 Hz。

| ROS 话题 | 内容 |
| --- | --- |
| `/front_lidar`、`/rear_lidar` | 各传感器坐标系中的真实 `PointCloud2` |
| `/d1max/localization/perception/rays_raw` | 原框架所需的每束三维回波、传感器原点和源时间 |
| `/d1max/localization/imu` | `sensor_msgs/Imu`，机身坐标、SI 单位 |
| `/front_lidar/imu`、`/imu_driver/imu_central` | 同一真实 IMU 的接口别名 |
| `/d1max/localization/navigation/state` | 用于任务规划的原 `NavigationState` |
| `/d1max/localization/navigation/local_state` | 连续局部状态和 PhysX 实测速度 |
| `/d1max/live_planning/execution/applied_motion` | 原 writer 已实际应用的隔离执行命令 |
| `/clock` | 全导航会话共用的仿真时钟 |

原机器狗配置中的雷达轴向、Airy96 ring 字段和加速度缩放不能直接套到此轮式机器人。这里明确绑定轮型外参和 SI IMU，不伪造 Airy96 或 Livox CustomMsg。原始雷达话题可用于检查传感器；本次任务管线使用已接入的逐束感知接口，未启动机器狗 `dual_lidar_adapter` 或 LIO 定位节点。

## 隔离运行和版本

Isaac Python 3.12 与 Humble Python 3.10 分进程运行，通过有大小、顺序和时间限制的本机 UDP 传输测量。全部 ROS 通信使用 `rmw_zenoh_cpp`、domain 219 和仅绑定 loopback 的私有路由。固定 UDP 端口为 18741/18742，同机同时运行一组测试。

启动前检查候选安装产物和地图的 SHA256，并由原 `navigation_session.verify()` 封存运行会话。仿真地图来自实际场景的三维表面和源点身份；PCT 原生库在当前机器编译。候选包不修改仓库的默认生产 release 选择器。

暂停、源时钟回退、场景重置或传感器过期都会撤销当前会话。重新测试需退出并创建新会话。命令过期时撤销低层前进需求，轮式驱动置零或 Spot 策略继续处理零需求，停稳判断始终读取测得速度。

Isaac 隔离会话显式使用双时间域的局部地图期限：实际射线的原始源时间和真实回调接收时间分别受原 0.5 s 期限约束。慢速仿真中的新测量可更新经过的体素，重复扫描、远处扫描和暂停均不能延长旧体素的期限。默认实机时间策略保留。任务依赖的纯功能心跳使用单调实际时间，感知、许可和命令仍检查各自的源时间。

无界面模式在场景截图完成后关闭视口，使用 Isaac 6 的 `SimulationManager.step` 直接推进真实物理及其传感器回调，并将源时钟限制为最多实际时间的 1 倍。没有截图请求时，从启动即关闭视口。窗口模式保留正常视口更新；实际运行速度与源频率不同，慢于实时的机器仍可能触发导航的样本过期保护。每次物理结果的 `summary.json` 记录实际时间与仿真时间比例、状态及双雷达的最长发送间隔。

历史 v19 跨房间运行的实际时间／仿真时间比例为 0.99991，实测状态约 60 Hz，最长实际状态间隔 22.91 ms，双雷达最长发送间隔 106.30／107.25 ms；原生 IMU 约 120 Hz，独立输出约 100.009 Hz。近距离与取消测试的比例也约为 1.0。早期带视口短测的比例约 3.01，属于历史性能记录。先前独立移动短测在约 0.288 m 运动后，首末点云回到世界坐标的墙面误差低于 3 mm；它不是当前完整任务的传感器精度验收。CPU 并行负载会影响实际时间指标。

## 已配置目录与重新构建

| 内容 | 本机位置 |
| --- | --- |
| 源码 | `/home/eric/wjg/d1max-system` |
| 构建、Python 科学计算环境、候选及运行日志 | `/home/eric/wjg/d1max-build-isaac` |
| Zenoh、fast_gicp、Livox SDK2 和 ROS 依赖 overlay | `/home/eric/wjg/d1max-deps` |
| Isaac Sim | `/home/eric/isaacsim` |

依赖来源和精确版本见 本机依赖说明（本机证据：`/home/eric/wjg/d1max-deps/README.md`）。当前机器的默认 GCC 9 与 oneTBB 的 `<execution>` 不兼容，因此定位闭包使用 GCC 11；不会修改系统 alternatives。

换机器时先安装 Isaac Sim，并准备 ROS 2 Humble、Zenoh、Livox SDK2、fast_gicp 等依赖的 `setup.bash`。依赖 overlay 和已封存候选不随 Git 上传；此脚本不会自动安装全部依赖。可覆盖以下路径后构建：

```bash
export ISAAC_SIM_ROOT=/absolute/path/to/isaacsim
export D1MAX_SIM_DEPS=/absolute/path/to/dependency-overlay
export D1MAX_SIM_BUILD=/absolute/path/to/simulation-build
bash simulation/isaacsim/build_local.sh
```

`D1MAX_SIM_CC`、`D1MAX_SIM_CXX` 和 `D1MAX_SIM_JOBS` 可覆盖编译器及并发数；默认 GCC 11、2 个作业。导航依赖 Humble 的 Python 3.10，Isaac 使用独立 Python 3.12。构建脚本在隔离环境安装固定版本 numpy/scipy/open3d，并构建 SDK-free 执行目标。

候选完整性清单绑定组装时的真实路径与文件字节，移动机器后须重新构建、重新组装和封存，不能直接复用原清单作为新机证据。下面命令用于当前已准备好依赖的机器：

```bash
bash simulation/isaacsim/build_local.sh
```

脚本保留已有候选。默认仿真候选由构建目录中的 `isaac_fixture.json` 明确选择，并检查其完整性清单哈希。改过传感器、场景或导航代码后，使用 `build_candidate.py --output 新路径` 重新组装，再给 `launch.sh --candidate 新路径`；也可用组装器的 `--select-local ../d1max-build-isaac/isaac_fixture.json` 更新本机仿真选择。PCT 的原生编译证据绑定外部 `pct_vendor` 目录；保留该构建目录，移动到另一台机器时重新编译。

这组仿真不提供实机机器狗几何、实机刹停、真实 SDK 阻塞、楼梯或跨层执行验收。实际测试结果以各会话的报告和测量数据为准。
