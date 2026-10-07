# D1 Max 导航系统 · Isaac Sim 仿真分支

本分支 `isaacsim-simulation` 接入 Isaac Sim 6.0.1-rc.7，保留原 **BT → PCT 三维全局路线 → 连续参考 → SCAN 三维局部规划 → tracker → 安全门 → 唯一 writer**。使用 ROS 2 Humble 和 Zenoh，当前验收范围为单楼层。

100×80 m 园区使用官方 Spot、真实关节行走、双三维雷达和原生 IMU。最新横穿／取消重启关卡启用六个原动态演员；旧 12×10 m 轮式夹具保留作历史回归。定位是明确标记的 PhysX 真值夹具，不代替 LIO、生产 release 或机器狗实机验收。

**最新实测（2026-10-07）：** [v64 广场横穿](simulation/isaacsim/verification/20261007/campus_crossing_v64_all_actors_goal_success.json) **11/11**、[v65 取消后重新导航](simulation/isaacsim/verification/20261007/campus_restart_v65_all_actors_goal_success.json) **14/14** 原检查通过，目标误差分别 `.097536/.190361 m`。两例均完成完整 500 Hz 域、实际 3 m 相遇曝光、最终持续停车和干净软件退役；取消重启在同一物理会话中执行。[v63](simulation/isaacsim/verification/20261007/campus_restart_v63_goal_success_encounter_pending.json)到达与物理审计通过，但原相遇历史截断，整例仍 PENDING。

最新封存任务采用 **25 Hz 双 LiDAR、v2 实测反馈、`.23/.30` 导航上限、`.38/.50` 内部 policy 上限、full XYZ `.65`／yaw `.8` 隔离工程域**。新 case 显式按采样频率封存相遇历史容量 2254；旧容量与历史结果不改。[可移植回归输入](simulation/isaacsim/assets/campus_navigation_regression.json)现显式设置 25 Hz／2254；通用大型场景生成器仍为 v2／10 Hz。新配置须重新组装封存，不自动继承旧成功。[本机隔离仿真 selector](simulation/isaacsim/verification/20261007/local_fixture_selection_v65.json)已核验后选用 v65，生产 release selector 不改；复现时明确指定候选，见 [仿真配置与启动](simulation/isaacsim/README.md)。

主线已修复静态地图与实时障碍融合、原始纳秒时间、真实路线进度及三个执行消费者的账本隔离；未知、真实 HIT、完整 XYZ 身体／动态体积、C1 和原租约继续参与拒绝。v3 真零 policy 的物理 A/B 失败，**没有选作默认或正式改善**。实际 500 Hz 停车通过与 SDK 的硬件验收标志 `false` 分开记录，历史失败不重算成成功。

本机仍慢于实时：v64／v65 RTF `.324902/.323385`，该两例双雷达最长 wall gap 均低于 `.316 s`；v63 曾超过原 `.5 s` receipt 期限，不能保证不同负载下持续授权或丝滑运动。约 333 m 仅有离线路由证据，长距离多点、楼梯／坡地／跨层、自由行人、原厂 SDK 与实机停车尚未验收。离散形体分离也不是连续碰撞证书。完整边界与历史见 [UNKNOWN 算法分析](simulation/isaacsim/SCAN_UNKNOWN_ANALYSIS.md)和 [物理速度标定](simulation/isaacsim/QUADRUPED_SPEED_CALIBRATION.md)。

## 导航主线

```text
RViz 初值和三维目标
        ↓
行为树任务管理 → 全局固定路线 → 连续局部参考
                                      ↓
                          局部候选 → 验证和提交
                                      ↓
                            跟踪 → 安全门 → 唯一 SDK writer
```

全局路线属于 map；感知、局部规划与跟踪使用连续 odom。普通定位校正不创建新任务。候选与已接受轨迹分开，旧结果不能覆盖新任务。执行许可、软件退役和实测停稳分别表达。

- 正式入口：`d1max_nav_ws/tools/navigation_entry.sh`。
- 会话 API：`d1max_pct_scan.navigation_session`。旧 `single_floor_*` 名称是同一实现的兼容名称，不是另一套架构。
- Web 负责连接和启停；RViz 负责初值、XYZ 目标、预览、执行确认及取消。
- 只运行一个 RViz，在同一窗口切换全局和局部视图。
- ROS 2 Humble，通信保持 `rmw_zenoh_cpp`；不替换为 Fast DDS。

## 在另一台电脑准备

本机新增的 Isaac Sim 机器狗三维导航任务测试见
[仿真配置与启动](simulation/isaacsim/README.md)。该隔离测试使用实际三维雷达、
原生 IMU 与 PhysX 四足关节，接入同一 BT/PCT/SCAN 主线；定位状态为明确标记的
仿真真值夹具，测试结果不代替生产 release、LIO 精度或机器狗实机验收。

完整步骤见 [新电脑部署](docs/DEPLOYMENT.md)。先拉源码、设置路径并做只读检查：

```bash
git clone --branch isaacsim-simulation https://github.com/dingzhen60027/d1max-system.git
cd d1max-system
source tools/deployment-env.sh
python3 tools/deployment_preflight.py --scope source
bash tools/build-source.sh --scope nav --output /absolute/path/to/new-build
```

最后一条**只打印构建计划**。补齐目标机依赖、核对计划后，加 `--apply` 才会在全新隔离目录编译；不会启动 ROS 服务、连接 SDK、修改默认 release 或授权运动。构建完成仍需按既有版本合同整套封存和验证，不能把 build 目录当成正式 release。

地图、录包、厂商 SDK、Zenoh/Livox/fast_gicp 安装环境、封存运行包与本机凭据不在 Git 中。它们需要另行提供或在目标机重建。生产默认选择器指向的本机运行包未上传，隔离仿真候选也须在目标机重建；直接 clone 后启动导航会明确失败，不能回退到旧预览或自动选择最新目录。

## 目录

| 目录 | 用途 |
| --- | --- |
| `d1max_nav_ws/src/` | 定位、行为树、规划、跟踪、安全、RViz 与上游源码 |
| `d1max_nav_ws/tools/` | 唯一导航入口、版本校验、地图工程和隔离验证 |
| `d1max_ros2/map_manager/` | Web、地图处理、bag 管理和启动接口 |
| `d1max_ros2/sdk_bridge_ws/` | SDK 读数、唯一运动发送与停止反馈 |
| `d1max_ros2/foxglove_d1max/` | 监控布局、本机连接管理器 |
| `tools/` | 源码同步、新机预检与显式构建 |
| `docs/verification/` | 按版本保留的测试与性能证据，不是实时状态 |

## 文档

- [模块与入口索引](docs/PROJECT_GUIDE.md)
- [当前验收缺项](docs/KNOWN_ISSUES.md)
- [验证范围](VERIFICATION.md)
- [源码来源和同步记录](SOURCE_SNAPSHOT.md)
- [导航主线说明](d1max_nav_ws/src/d1max_pct_scan/README.md)
- [定位坐标与时间合同](d1max_nav_ws/src/d1max_localization/NAVIGATION_ESTIMATION.md)
- [地图 Web](d1max_ros2/map_manager/README.md)
- [SDK 与运动接口](d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/README.md)

第三方源码保留各自 LICENSE、NOTICE 和来源记录；厂商材料需自行合法取得。发布仓库与原开发/运行目录分开，push 不会自动部署或重启机器人。
