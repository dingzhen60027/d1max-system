# Isaac Sim 三维导航任务测试

本目录在 `isaacsim-simulation` 分支接入 Isaac Sim 6.0.1-rc.7，测试原 **BT → PCT 三维全局路线 → 连续参考 → SCAN 三维局部规划 → tracker → 安全门 → SDK-free 唯一 writer**。100×80 m 园区使用官方 Spot、真实关节行走、前后两台三维激光雷达和原生 IMU。定位为显式 PhysX 真值夹具；固定平面支撑、演员脚本和实际碰撞体均参与封存，不代表实机定位或原厂 SDK 验收。

**实测状态：** [v52 广场横穿通过](verification/20261007/campus_crossing_v52_all_actors_goal_success.json)，原 Action 到达且独立 case gates 全部通过，目标误差 `.154777 m`，完整 500 Hz 实测停车确认 `2.138 s`。[v53](verification/20261007/campus_restart_v53_handoff_failure.json) 与 [v57](verification/20261007/campus_restart_v57_stationary_handoff_failure.json) 的取消后重新导航均失败，原失败保持。[v59 取消重启正式复测失败](verification/20261007/campus_restart_v59_actor_overlap_and_body_certificate_failure.json)：新任务真实提交 writer commit `1–28`，已跨过上述三消费者的 execution 水位阻断；完整关卡仍未通过。演员与机身出现采样 AABB 可能重叠后，500 Hz full XYZ 峰值 `1.064634535 m/s`、yaw 峰值 `.886969864 rad/s` 超过原 `.65/.8` 工程域；身体 Cube 的 world-AABB 代理另有保守假拒。未取得接触力，不能据此归为唯一物理根因；原 drain 超时、未确认软件退役和缺失第二阶段终态报告保持失败。 v52 仍出现超过原 `.5 s` receipt 期限的雷达间隔，尚不能称连续丝滑、全部 UNKNOWN 已解决或硬件通过。

**取消重启修复已完成离线验证：** tracker 与 SCAN 按完整 execution 身份隔离 writer commit／ACK 高水位，见 [native 成对回归](verification/20261007/paired_writer_execution_scope_fix_offline.json)；Python 安全门同类 ACK 身份遗漏也已修复，见 [安全门回归](verification/20261007/safety_writer_execution_scope_fix_offline.json)。Tracker 4 个 CTest targets／207 个 gtest、SCAN 18 个 CTest targets／196 个 gtest，以及安全相关 269 项 Python 回归（**包含** focused 69 项）分别通过；两个 SCAN launch wrapper 实际 `Ran 0 tests`，不计启动行为覆盖。v59 已证实三消费者在新 scope 内真实多次提交，完整任务与安全收尾仍失败。新 `spot_monotone_measured_v3` 零需求反馈、身体几何证书修复和 [几何故障停止观测](verification/20261007/geometry_fault_stop_observation_offline.json)处于离线验证阶段，**FORMAL_PENDING**；后者 56 项测试通过，尚未证明新整图退役或停车。历史根因、完整数字和剩余问题集中在 [UNKNOWN 算法分析](SCAN_UNKNOWN_ANALYSIS.md) 与 [机器狗速度标定](QUADRUPED_SPEED_CALIBRATION.md)。

## 复现园区导航

可移植输入为 [campus_navigation_regression.json](assets/campus_navigation_regression.json)。下面复现 `crossing_blocker`；取消后重启改用 `resume_after_block`，并为配置、候选、会话分别选新名字。两关都保持六演员实际启用：`plaza_person` 必须遭遇，其余五个是背景演员，仍参与完整动态预测与碰撞检查。不能把背景演员都计为已遭遇。

先在本机已配置依赖的终端准备输入；`list` 只列关卡，`prepare` 不启动机器人：

```bash
cd /home/eric/wjg/d1max-system
source simulation/isaacsim/env.sh
export D1MAX_CASE_ID=crossing_blocker
export D1MAX_CASE_CONFIG="$D1MAX_SIM_BUILD/scenarios/crossing_local_001.json"
export D1MAX_CASE_CANDIDATE="$D1MAX_SIM_BUILD/isaac-candidate-campus-crossing-local-001"
export D1MAX_CASE_SESSION="$D1MAX_SIM_BUILD/runs/campus_crossing_local_001"

/usr/bin/python3 simulation/isaacsim/scenario_suite.py list \
  --spec simulation/isaacsim/assets/campus_navigation_regression.json
/usr/bin/python3 simulation/isaacsim/scenario_suite.py prepare \
  --spec simulation/isaacsim/assets/campus_navigation_regression.json \
  --case "$D1MAX_CASE_ID" --output-config "$D1MAX_CASE_CONFIG"
```

源码或安装闭包改变后重新构建，再组装独立候选。构建期间应退出已有 Kit／ROS 测试；已核对同版安装时可跳过 `build_local.sh`：

```bash
bash simulation/isaacsim/build_local.sh
/usr/bin/python3 simulation/isaacsim/build_candidate.py \
  --output "$D1MAX_CASE_CANDIDATE" \
  --scene-config "$D1MAX_CASE_CONFIG" \
  --nav-install "$D1MAX_SIM_BUILD/nav/install" \
  --sdk-install "$D1MAX_SIM_BUILD/sdk/install" \
  --localization-install "$D1MAX_SIM_BUILD/localization/install" \
  --pct-vendor "$D1MAX_SIM_BUILD/pct_vendor"
```

先检查启动计划，再启动同一冻结候选。默认无界面；添加 `--gui` 可打开 Isaac 窗口。最后从**候选内最终配置**评估原会话：

```bash
/usr/bin/python3 simulation/isaacsim/scenario_suite.py run \
  --case "$D1MAX_CASE_ID" --candidate "$D1MAX_CASE_CANDIDATE" \
  --session "$D1MAX_CASE_SESSION" --plan-only
/usr/bin/python3 simulation/isaacsim/scenario_suite.py run \
  --case "$D1MAX_CASE_ID" --candidate "$D1MAX_CASE_CANDIDATE" \
  --session "$D1MAX_CASE_SESSION"
/usr/bin/python3 simulation/isaacsim/scenario_suite.py evaluate \
  --prepared-config "$D1MAX_CASE_CANDIDATE/simulation/assets/scene_config.json" \
  --session "$D1MAX_CASE_SESSION" \
  --output "$D1MAX_CASE_SESSION/scenario_evaluation.json"
```

每次使用新目录，不覆盖或改写旧候选和报告。多 phase／取消重启在同一导航图与同一物理会话执行，不拼接独立仿真。改传感器、导航源码或场景后，重新准备、组装和封存；旧成功不能自动归给新配置。关卡详情见 [园区说明](CAMPUS_SCENARIOS.md)，最新状态以本页及各封存证据为准。

## 当前输入与验收边界

| 项目 | 当前可移植输入 |
| --- | --- |
| 物理／策略 | PhysX 500 Hz、官方 policy 50 Hz；原机身状态输出 50 Hz |
| 原始物理记录 | 500 Hz BEGIN＋真实最终 END，完整 XYZ 位姿、线速度和角速度 |
| 双三维雷达 | 前后 PhysX 多线雷达，360°×160°，1°×5°，10 Hz，0.06–35 m |
| 雷达原点 | 机身坐标 `(0.2,0,0.2)` 与 `(-0.2,0,0.2)` m |
| IMU | 原生 500 Hz 测量中选取真实 100 Hz 输出，保留原 source；SI 单位与重力约定 |
| 导航权限 | `.23 m/s`、`.30 rad/s`；内部 policy 上限 `.38/.50` |
| 隔离工程模型 | full XYZ reference／measured-travel／reachable `.65 m/s`，yaw `.8 rad/s`，绑定 [原标定证据](verification/20261007/spot_velocity_domain_v47_native_navigation.json) |
| 地面与身体 | 地面目标 Z=0，参考 body centre `.481 m`，上方包络 `.589 m`；实际 13 个碰撞体与完整腿／脚证书独立检查 |
| 演员更新 | 显式 `native_kinematic_v1`，每原始 tick 按 `actor_pose(t+dt)` 设置真实 kinematic target |

可移植输入和 [默认大型资产](assets/large_quadruped_scene.json)是生成新候选的源码配置；具体正式运行可使用另行封存的作者配置。未运行的长程多点、门口／窄道关卡及约 `332.84 m` 离线路由，不继承 v52 成功。

定位采用已核对的固定 `map←odom` 与 Isaac world 身份。Faster-LIO、ICP、真实重定位和先验配准性能未验收；坡地、楼梯、跨层执行、自由行动的行人、原厂 SDK 阻塞与实机制动也不在这些结果内。三维传感器、完整 XYZ 曲线和高度净空检查保留，平地支撑条件不能外推到楼梯。

源时间、实际接收时间和执行身份分别检查。双射线的原 source／receipt 各受原 `.5 s` 期限约束；动态 oracle 的 source／steady freshness 为 `.20 s`，完整未来形体覆盖另为 **6 s**，两者不能相互续期。原轮内预算、C1 `.05`、绝对 join、完整几何与停车约束不因新模型放宽。重复扫描、暂停、远处新扫描或旧 ACK 都不能延长旧证据；时钟回退、场景重置或身份变化会撤销运动。

UNKNOWN 查询继续保守拒绝。认证静态空体积、平面支撑接触和严格 actor-only 旧 HIT 退役是不同证据：支撑接触不是雷达 FREE；只有精确同演员命中身份、更新完整动态证明与**闭体素完整 XYZ 分离**等条件都满足，才可恢复已认证接触许可。混合、未归属、poison、静态占据、未来 tube 相交或 lease 失效均不借此放行。算法及 v40 原同快照反例见 [UNKNOWN 分析](SCAN_UNKNOWN_ANALYSIS.md)。默认 production 严格模式不启用隔离仿真证书。

原低层实测反馈仅调节官方策略输入，不改写 raw body／joint 状态。正请求 `.05` 是 memory-only 恢复的资格门，**不是最小行走速度**；必须另有完整线／角范数连续一秒静止和严格 source／native counter 身份，fault 永久关闭干预。安全门撤权后仍消费真实零需求，不靠补最低速度继续行走。历史小命令不响应和持续正需求固定点分别取证，见 [速度与 memory A/B](QUADRUPED_SPEED_CALIBRATION.md)。

独立 [500 Hz 物理审计](navigation_physics_audit.py)核对完整原始记录、hash、模型域和真实 zero-tail；停车使用 full norm `.03 m/s`／`.05 rad/s` 连续一秒，原 latency 3 s、额外 XY `.5 m` 与 yaw `.4 rad` 上限保持。v52 的实测 STOP 通过与原 mock `physical_stop_confirmed=false` 分开保留；`physical_acceptance=false` 始终保持。离散实际形体审计不是采样间连续碰撞证明，进程退出码或 software retirement 也不等于硬件停车。

## 单 RViz 与 Isaac 窗口

显式选择候选，避免本机旧 selector 使新源码说明与旧配置混淆：

```bash
bash simulation/isaacsim/launch.sh \
  --candidate "$D1MAX_CASE_CANDIDATE" --headless --rviz
# 同时显示 Isaac 和单个 RViz；这里只限制绘制频率
bash simulation/isaacsim/launch.sh \
  --candidate "$D1MAX_CASE_CANDIDATE" --rviz --render-fps 3
```

RViz 显示三维地图、任务和轨迹。XYZ 目标经原面板预览、确认执行或取消；Z 为**地面支撑点高度**，本园区填 `0`，连续参考只加一次封存身体高度，实际 raw Z 不改。单独 GUI 启动不代替上面的完整关卡评估。当前按用户要求不录制视频。

`Ctrl+C` 先退役原任务管理者，再关闭物理、传感器与私有路由。低层继续处理零需求，停止判定读取真实速度。日志位于会话目录，`isaac_smoke_report.json` 和 case evaluation 检查原 Action／owner／writer／PhysX 反馈，不能仅凭发过速度判定成功。

| ROS 话题 | 内容 |
| --- | --- |
| `/front_lidar`、`/rear_lidar` | 各传感器坐标系中的真实 `PointCloud2` |
| `/d1max/localization/perception/rays_raw` | 原感知接口的三维回波、逐束原点、ring 与原 source |
| `/d1max/localization/imu` | 机身 SI `sensor_msgs/Imu`；`/front_lidar/imu`、`/imu_driver/imu_central` 为接口别名 |
| `/d1max/localization/navigation/state`、`local_state` | 原规划状态和连续局部实测状态 |
| `/d1max/live_planning/execution/applied_motion` | 唯一 writer 的实际应用反馈 |
| `/clock` | 同一仿真会话 source clock |

雷达是 PhysX 瞬时扫描，隔离会话允许真实零束时间偏移；实机扫描时长检查保留。自体过滤依据实际注册形体，不补造遮挡后的自由射线；不伪造 Airy96／Livox 消息来套原机器狗外参。独立 IMU 引用必须匹配实际收到的同源样本，允许的结束接收等待不会补样或延长导航许可。

## 隔离运行与换机构建

| 内容 | 本机位置 |
| --- | --- |
| 源码 | `/home/eric/wjg/d1max-system` |
| 构建、候选、科学计算环境及日志 | `/home/eric/wjg/d1max-build-isaac` |
| Zenoh、fast_gicp、Livox SDK2 和 ROS 依赖 overlay | `/home/eric/wjg/d1max-deps` |
| Isaac Sim | `/home/eric/isaacsim` |

Isaac Python 3.12 与 Humble Python 3.10 分进程，通过有大小、顺序和时间检查的本机 UDP 桥接。ROS 使用 `rmw_zenoh_cpp`、domain 219、loopback 私有路由；UDP 18741/18742，同机只运行一组测试。启动核对安装、地图、配置与源文件 SHA256，再由原 `navigation_session.verify()` 封存会话，不修改生产 release selector。

换机先安装 Isaac Sim、ROS 2 Humble 并准备依赖 overlay 的 `setup.bash`；依赖／候选／大体积测量不随 Git 上传，脚本不自动安装全部系统依赖。本机依赖精确版本在 `/home/eric/wjg/d1max-deps/README.md`。路径可覆盖：

```bash
export ISAAC_SIM_ROOT=/absolute/path/to/isaacsim
export D1MAX_SIM_DEPS=/absolute/path/to/dependency-overlay
export D1MAX_SIM_BUILD=/absolute/path/to/simulation-build
bash simulation/isaacsim/build_local.sh
```

`D1MAX_SIM_CC`／`D1MAX_SIM_CXX`／`D1MAX_SIM_JOBS` 可覆盖编译器与并发；本机默认 GCC 11、2 jobs，避免 GCC 9 与 oneTBB `<execution>` 不兼容，不改系统 alternatives。脚本在隔离环境安装固定科学计算依赖并构建 SDK-free 目标。PCT 原生证据绑定 `pct_vendor`，换机须重新编译、组装和封存，不能把旧清单当新机证明。

已有候选不会被构建覆盖。默认仿真选择由 `$D1MAX_SIM_BUILD/isaac_fixture.json` 明确绑定完整性 hash；可用 `build_candidate.py --select-local 新selector路径` 选择本机夹具。本文命令显式给 `--candidate`，不依赖历史 v31 selector。无界面时关闭视口并推进原物理／传感器回调；可见窗口只改变绘制负载，源频率和 lease 不随慢速墙钟重写。每次 physics `summary.json` 保留 RTF 和状态／双扫描最长 wall 间隔，CPU 争用可能触发真实过期撤权。

## 代码与历史证据索引

| 内容 | 入口 |
| --- | --- |
| 实际物理、官方策略、原协议桥接 | [scene.py](scene.py)、[quadruped.py](quadruped.py)、[bridge.py](bridge.py) |
| 实际 USD 几何与静态先验 | [truth_map.py](truth_map.py)、[GridMap](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/grid_map.cpp) |
| 演员目标与完整六秒预测 | [world_builder.py](world_builder.py)、[dynamic_collision.py](dynamic_collision.py) |
| 候选及模型绑定 | [nav_prepare.py](nav_prepare.py)、[build_candidate.py](build_candidate.py) |
| 完整曲线／真实控制与碰撞审计 | [control_audit.py](control_audit.py)、[collision_audit.py](collision_audit.py)、[navigation_physics_audit.py](navigation_physics_audit.py) |
| 固定 SHA 的开源 PCT／SCAN 思路研究 | [UPSTREAM_DESIGN_REVIEW.md](UPSTREAM_DESIGN_REVIEW.md) |
| UNKNOWN、配对、支撑、换轨与未闭环问题 | [SCAN_UNKNOWN_ANALYSIS.md](SCAN_UNKNOWN_ANALYSIS.md) |
| 低速策略、工程模型、STOP 与 memory 组件实验 | [QUADRUPED_SPEED_CALIBRATION.md](QUADRUPED_SPEED_CALIBRATION.md) |

[旧 v47 投影失败](verification/20261007/campus_crossing_v47_projection_domain_failure.json)和 [新 `.65` 离线修复](verification/20261007/spot_reachable65_projection_fix_offline.json)分别保存；不能把 queries 0 的 sentinel 后继原因全当真实传感器过期。旧 `.50/.60` 模型失败也不由新 `.65` 反算成通过。[native actor target A/B](verification/20261007/spot_native_actor_targets_causal_ab.json)有相同机身／演员实测读回，但 11 个 LiDAR 点 XYZ 最多差 `46.671 μm`，不能称全部 scan arrays 逐字相同；该组件仍有超 `.5 s` ray gap。性能与正式导航范围见上述专题文档。

早期轮式 v19 三项任务、v22 同会话录屏及 Spot v30／v31 短程结果保留在 [历史验证](verification/20261005/historical_verification.json)、[v30](verification/20261006/campus_navigation_v30.json)与 [v31](verification/20261006/campus_navigation_v31.json)。它们只适用于各自封存候选。v31 横穿及旧轮式 generation 7→8 失败等未被历史成功抵消。历史视频和完整 raw logs 只存本机，不随 Git 发布；v22 三路视频位于 `/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/`。

历史轮式场景是 12×10 m、实体差速轮／脚轮、双三维雷达和原生 IMU，见 [预览](assets/overview.png)与 [静态 USD](assets/indoor_scene.usda)。其物理为 120 Hz，雷达 10 Hz／15 m、原点 `(±.42,0,.15)`，100 Hz IMU 从原生 120 Hz 真实样本选取；不能套当前 Spot 参数。需要复现时显式选择旧 `isaac-candidate-v19`：

```bash
# 原到达／运动后取消
bash simulation/isaacsim/launch.sh \
  --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --smoke
bash simulation/isaacsim/launch.sh \
  --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --smoke --smoke-case cancel
# 跨门执行／仅预览取消：两种原 Action 分别验收
bash simulation/isaacsim/launch.sh \
  --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --rviz --smoke \
  --goal 4 -3 0 --smoke-duration 360
bash simulation/isaacsim/launch.sh \
  --candidate ../d1max-build-isaac/isaac-candidate-v19 --headless --rviz --smoke \
  --smoke-case preview_cancel --goal 4 -3 0 --smoke-duration 120
```

分支比较基线为 `5dfaf8cddec952b3453a02e3a231de5228bd6017`。仓库内链接可随源码使用；标为本机的绝对路径仅作原始证据索引，GitHub 无法直接读取，也不能把未保存的硬件／连续碰撞证据补作验收。
