# D1 Max 导航系统

定位、建图、地图处理和室内外多楼层导航的统一源码仓库。只有一条 BehaviorTree.CPP 导航主线；PCT、SCAN 是当前算法后端，单楼层是当前验收范围。

当前更新包含 2026-10-04 的连续性整改和内存、计算、通信阻塞优化。源码与隔离组件测试已核对，尚未成套部署到生产环境；实机运动、跨层执行、NUC 实时性及 Isaac Sim 接入不能视为已完成。

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

完整步骤见 [新电脑部署](docs/DEPLOYMENT.md)。先拉源码、设置路径并做只读检查：

```bash
git clone https://github.com/dingzhen60027/d1max-system.git
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
