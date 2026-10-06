# Isaac Sim 三维导航任务测试

分支：`isaacsim-simulation`。比较基线：`5dfaf8cddec952b3453a02e3a231de5228bd6017`。这是同一导航主线的 Isaac Sim 仿真候选与取证分支，当前仍有未解决的换轨失败。

**最新状态（2026-10-06）：** `live_view_20261006_001` 实测行驶 **8.0010 m** 后失败，原任务原因是 `execution_blocked_timeout:waiting_admitted_nonzero_command`。记录显示 generation 7 → 8 的换轨未完成；根因尚未确证，不能将超时原因直接当成算法根因，也不能据此断言由 SCAN UNKNOWN 引起。已确认实测停稳与任务软件退役。[最新失败取证](verification/20261006/live_view_failure.json)随本分支保存；完整原始日志和大体积测量仍在本机证据目录。

历史 v19 三项任务和 v22 录屏的成功记录保留，适用范围限于各次封存候选与会话。[历史验证摘要](verification/20261005/historical_verification.json)随分支保存；这些成功不能抵消此次失败或证明连续换轨已经稳定。

本目录把本仓库的导航主线接到本机 Isaac Sim 6.0.1-rc.7 轮式机器人。任务仍由原 BehaviorTree.CPP 管理，经过 PCT 全局路线、连续参考、SCAN 三维局部规划、轨迹跟踪、安全门和原 SDK-free writer，最后驱动 PhysX 轮关节。

本次重点是导航任务和物理执行反馈。定位输入使用 PhysX 测量的真实机器人位姿，标记为 `groundtruth_fixture`；雷达和 IMU 使用实际仿真传感器。Faster-LIO、ICP 和机器人原厂 SDK 的性能不属于这组测试结果。

## 本分支相对基线的变化与边界

基线已具备 BehaviorTree.CPP 任务管理、PCT 全局三维路线、连续参考、SCAN 双圆柱局部规划、整曲线及偏航扫掠验证、换轨事务、跟踪、安全门、唯一 writer 和取消退役机制。本分支沿用这条控制链。

| 本分支新增或修复 | 代码与说明 |
| --- | --- |
| Isaac Sim 轮式物理场景与双三维雷达、原生 IMU 接口 | [场景](scene.py)、[原接口桥接](bridge.py)、[候选组装](build_candidate.py)；定位是显式 PhysX 真值夹具 |
| 认证静态空体积与实时射线融合 | [静态先验加载](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/static_occupancy_prior.cpp)、[GridMap 查询](../../d1max_nav_ws/src/scan_planner_vendor/plan_env/src/grid_map.cpp)、[USD 体积生成](truth_map.py)；真实 hit 撤销先验 FREE，默认严格模式保留 |
| 查询性能与失败取证 | 同一快照及证据范围的稠密体素缓存、原始失败位置和期限记录；原几何范围和计算预算保持 |
| 源时间、进度与取消修复 | 原始整数纳秒、合法重复仿真 tick、同 tick 取消 ACK、原始位姿证据和规范化计算分离、弯道物理进度判定；细节见 [分析记录](SCAN_UNKNOWN_ANALYSIS.md) |
| 单 RViz 与 Isaac Sim 录屏 | RViz 初始化与仿真时钟诊断修复，真实窗口录制；历史 v22 录屏结果见下文 |

尚未验收：generation 7 → 8 换轨失败的根因与修复；真实 LIO／重定位条件下的先验配准；动态遮挡物持续避让；轮胎低部与地面支撑面的通用碰撞合同；坡地、楼梯、跨层执行及机器狗构型；原厂 SDK 与实机刹停。本先验当前要求固定单位 `map←odom`、认证静态场景和有限姿态／高度偏差，不能直接当作实机通用方案。所有仿真结果保留 `physical_acceptance=false`。

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

该命令使用已实测的无界面 PhysX 场景，原仓库的单个 RViz 显示三维地图、任务和规划轨迹。在 RViz 中设置 XYZ 目标，预览路线后通过原面板确认执行或取消。目标的 Z 是**地面支撑点高度**，当前室内地板用 `0`；机器人机身参考点实测高度约 `0.35 m`。上表 v19 三测使用无界面 PhysX；v22 录屏在可见视口完成跨门任务。两个窗口同时显示使用：

```bash
bash simulation/isaacsim/launch.sh --rviz --render-fps 15
```

## 历史 v22 同会话真实录屏（2026-10-05）

`v22` 录制了约 2 分 42 秒的正常速度窗口视频：并排版（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/overview.mp4`）、RViz（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/rviz.mp4`）、Isaac Sim（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/isaacsim.mp4`）。轮式机器人通过原任务接口从 `(-4,-3)` 穿门到 `(4,-3)`，实测行程 8.8165 m，原 Action 返回 `measured_goal_reached`，实测停稳和正常退役均确认。

录制时修复了 RViz 初始化相机切换导致的属性悬空崩溃，以及诊断面板错误使用墙钟判断仿真状态过期的问题；70 项显示插件测试通过。总览包含双雷达的原始实测点云。GUI 使用官方纯绘制更新，物理仍为 120 Hz；6109 次绘制额外物理步数为 0，最长状态和扫描间隔分别约 96 ms、162 ms。录制数据和任务报告的哈希见 录屏验证清单（本机证据：`/home/eric/wjg/d1max-build-isaac/videos/navigation_v22_20261005/navigation_evidence.json`）。

`record_windows.py capture` 只接收两个显式 X11 窗口 ID，使用同一墙钟时间轴录制，SIGINT 正常封装；`postprocess` 生成并排版。视频时间轴用于对照画面，导航验收仍使用原任务报告及真实传感器源时间。

自动近距离到达和取消测试：

```bash
bash simulation/isaacsim/launch.sh --headless --smoke
bash simulation/isaacsim/launch.sh --headless --smoke --smoke-case cancel
```

跨房间完整执行测试：

```bash
bash simulation/isaacsim/launch.sh --headless --rviz --smoke \
  --goal 4 -3 0 --smoke-duration 360
```

另有独立的跨房间路线预览取消用例：

```bash
bash simulation/isaacsim/launch.sh --headless --rviz --smoke \
  --smoke-case preview_cancel --goal 4 -3 0 --smoke-duration 120
```

此用例不发送执行确认，要求真实 Action 返回已取消、原工作节点退役且机器人始终静止；它与运动后的取消测试分别记录。

每次使用新会话目录，默认在 `../d1max-build-isaac/runs/isaac_时间/`。终端会打印日志路径。`isaac_smoke_report.json` 检查原任务结果、执行身份、writer 停止反馈和 PhysX 实测静止；不能只凭“发过速度”判定通过。`Ctrl+C` 先让原任务管理者退出，再关闭传感器、物理场景和私有路由。

IMU 取证在固定测量边界冻结状态集合，再给独立 IMU 回调最多 1 s 实际时间完成接收；所有引用时间必须精确匹配实际收到的 IMU，不从原生日志补填。边界和缺失源时间写入 `imu_evidence`，内部缺样仍判失败；该接收等待不延长导航或物理执行的有效期。v19 近距离测试的 841 条原始状态含 2 条早于首个独立 ROS IMU 接收的启动前缀，边界内 839 条全部匹配；这 2 条在原生 PhysX IMU 日志也有精确同源记录，任务发出后的 498 条状态全部匹配。前缀不补填进 ROS 接收集合。

## 场景和传感器

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

暂停、源时钟回退、场景重置或传感器过期都会撤销当前会话。重新测试需退出并创建新会话。命令过期时物理轮驱动置零，停稳判断继续读取测得速度。

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

这组仿真不提供机器狗几何、实机刹停、真实 SDK 阻塞、楼梯或跨层执行验收。实际测试结果以各会话的报告和测量数据为准。
