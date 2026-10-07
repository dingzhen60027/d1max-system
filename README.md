# D1 Max 导航系统 · Isaac Sim 仿真分支

定位、建图、地图处理和室内外多楼层导航的统一源码仓库。只有一条 BehaviorTree.CPP 导航主线；PCT、SCAN 是当前算法后端，单楼层是当前验收范围。

本分支为 `isaacsim-simulation`，基于最初克隆的 `5dfaf8cddec952b3453a02e3a231de5228bd6017`，保存原导航框架的 Isaac Sim 三维导航测试版本。当前默认场景为 **100×80 米综合园区＋官方 Spot 机器狗**：真实关节行走、双三维雷达、原生 IMU，以及办公区、仓储区、广场、狭窄通道和动态障碍。原有 BT/PCT/SCAN 主线继续使用，旧轮式场景保留用于历史回归。

场景定义 6 个确定性动态演员，默认启用 4 个，关卡按封存配置选择演员；六项系统测试覆盖长距离多点、横穿、迎面会车、临时堵门、取消停车与恢复。约 333 米路线已通过离线静态连通性检查，机器狗大场景物理与传感器测试已完成；整套关卡的实际执行结果单独记录，缺证据保持待验证。见 [园区与测试入口](simulation/isaacsim/CAMPUS_SCENARIOS.md)和 [Spot 验证](simulation/isaacsim/QUADRUPED_VALIDATION.md)。

**当前实测状态（北京时间 2026-10-07）：** [v52 广场横穿](simulation/isaacsim/verification/20261007/campus_crossing_v52_all_actors_goal_success.json)原 Action 到达与完整 case gates 通过。[v59 取消重启](simulation/isaacsim/verification/20261007/campus_restart_v59_actor_overlap_and_body_certificate_failure.json)正式复测 **FAILED**：新任务真实 writer commit `1–28` 已跨过 tracker、SCAN 和 Python 安全门三消费者的 execution 水位阻断，但演员／身体采样 AABB 可能重叠后出现 full XYZ 峰值 **1.064634535 m/s**、yaw 峰值 **.886969864 rad/s** 超域；另有身体 Cube 保守代理假拒与停止收尾缺证据。AABB 可能重叠不等于已取得接触力或连续碰撞证明。原关卡失败及缺失第二阶段终态报告保持，不能称六关全部通过。

新 v3 零需求反馈、身体几何证书修复和 [几何故障停止观测](simulation/isaacsim/verification/20261007/geometry_fault_stop_observation_offline.json)仅完成离线验证，**FORMAL_PENDING**；停止观测 56 项回归通过，新候选整图与停车尚待实测。本机默认仿真 selector 仍为旧 `isaac-candidate-v31-campus`，复现新版本须显式指定封存候选，见 [仿真说明](simulation/isaacsim/README.md)。v31 短程到达与取消等历史结果分别保留，旧失败不会因新修复改为通过；约 333 m 仍仅是离线路由。

近身长期未知阻断的修复依赖认证封闭静态体积与实时障碍融合，仍保留严格未知阻断和证据期限；当前只支持经过验证的固定单位 `map←odom`。定位输入是显式 `groundtruth_fixture`。生产 release、Faster-LIO/ICP 性能、机器狗实机运动、跨层执行与 NUC 实时性均未因此验收。

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

地图、录包、厂商 SDK、Zenoh/Livox/fast_gicp 安装环境、封存运行包与本机凭据不在 Git 中。它们需要另行提供或在目标机重建。默认选择器指向的旧本机运行包未上传，直接 clone 后启动导航会明确失败，不能回退到旧预览或自动选择最新目录。

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
