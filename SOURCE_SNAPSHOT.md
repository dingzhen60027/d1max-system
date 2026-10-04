# 源码快照记录

## 当前同步：2026-10-04

以发布仓库 `832a5209d962cf1194fdbfaf5d5ba72c0d15c711` 为起点，同步现有导航主线、Web 启停、SDK bridge、测试与模块说明。开发导航目录基线为 `76e1ccb5a081af13be22eac6c1d7deb31ba5ae61`，分支 `chore/project-layout-20260928`；包含在该目录后续完成的未提交修改，不能把开发 HEAD 当作这批源码的完整身份。

统一提交位置仍为 `/home/dndx/d1max-system`、`main`、`https://github.com/dingzhen60027/d1max-system`。SDK 源码由应用资料包下的 `d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge` 同步，厂商 vendor 头文件和动态库仍排除。

使用既有同步工具的 `--skip-experiments`，本轮不复制实验会话、封存安装包和重复源码快照；既有历史实验资料保留。10-04 测试 JUnit 和抽查源码/ELF 哈希单独保存在 [docs/verification/20261004](docs/verification/20261004/)，记录中的路径为原验证环境，不代表文件已部署。原始地图、bag、大型审计地图、构建产物、厂商库和凭据不上传。

发布副本旧文件覆盖前备份到 `/tmp/d1max-source-backup-7JleBC4P`。原开发目录未移动，仅同步整理两份测试的文件尾空行和一份历史文档的段落空白，不改行为。运行服务及默认 release 指针未变；本仓库不包含指针所需的完整本机运行包。发布副本中的 39 个已迁移旧路径按源码布局核对后清理，Git 历史可恢复；不是删除运行工作区。

以下保留之前各次同步的原始范围和事实，不覆盖当前主线状态。

首次整理：2026-09-10；本次增量同步：2026-09-26（Asia/Shanghai）。

本仓库首次提交是当前工作文件的整体快照，不改变原工作区或导航仓库的提交历史。

本次以发布副本提交 `2027b71921b78df216253c25613341e4ebfa8f9e` 为起点，同步两处运行工作区截至 2026-09-26 的源码、配置、测试和开发记录。主要增量包括定位修复、原生 PCT / SCAN 接线、双雷达逐束射线预览、全局/局部 RViz 布局预设及只读关节遥测接口。
统一提交位置 `/home/dndx/d1max-system`，远程公开仓库 `https://github.com/dingzhen60027/d1max-system`，分支 `main`；公开状态已于本次整理核对。
本次整理主要更新发布副本及其项目级文档，不移动运行目录、不提交/改写原导航仓库历史，不重新部署、启动机器人或停止运行服务。原工作区仅同步整理 `grid_map.cpp` 一行纯空白与三份历史审计 Markdown 的文件尾空行，未改变行为。

当前规划预览为 `LIVE_VISUALIZATION_NO_MOTION`，配置选择 `per_sensor_rays`：保留前后雷达来源、逐点采集时间与各自射线原点，再按对应时刻投影至原生 SCAN 滚动地图。`prepare_motion` 对该后端明确拒绝运动模式。已接入滚动地图、A* 和 B 样条，不代表完整实机闭环或碰撞安全验收完成；近身 occupied/unknown 阻塞仍待解决。详见 [已知问题](docs/KNOWN_ISSUES.md) 和 [验证记录](VERIFICATION.md)。

## 原工作区

- 应用、ROS 接入：`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2`
- 导航：`/home/dndx/d1max_nav_ws`
- 原导航基线：`3751377143c50697c847bc52a88f5146ff65bab3`，分支 `feature/sc-pgo-loop-closure`。本快照另外包含该目录当前尚未提交的配置、算法适配及规划代码。
- SC-PGO：`9b18dcb34e45bb49d73ce2d404b4637f27c6bb98`，原上游 `https://github.com/taehun-kmu/SC_PGO.git`。
- PCT Planner vendor：`35cd73fd82bcd51bc538429294af7646b2a09815`，原本机 vendor 工作树另有 `COLCON_IGNORE`。
- Scan Planner 的来源保留在其目录中的 `UPSTREAM.md`。

本仓库保存实际依赖源码，不保留嵌套 `.git` 或依赖原机器绝对路径的 Git 子模块指针；第三方许可证、NOTICE 和来源文档保留。

## 不上传的内容

- 原始/处理后 PCD、PLY、rosbag、地图处理输出、归档状态、运行 PID 和日志。
- Node/Python 环境、ROS build/install、本机 Zenoh 二进制安装和扩展编译包。
- 厂商 SDK 头文件与动态库、原始 URDF/STL/模型资料包；这些外部材料需要使用者自行合法取得。
- 第三方预编译归档和点云数据压缩包。Faster-LIO 的 TBB 下载位置在 `cmake/packages.cmake` 中保留。
- 浏览器状态、GitHub 凭据、私钥和密码文件。

实验目录仅纳入筛选后的源码、脚本、说明、配置及少量审计记录；不纳入整份外部源码拷贝、实验工作空间、运行输出或临时文件。其中 LIO-SAM 实验工作空间的 `ws/src/LIO-SAM` 外置修改未包含在本快照，跨层地图 NPZ/JSON 处理证据也未上传。保留实验说明不代表相应实验可仅凭本仓库完整复现，本仓库不是自包含部署包。

Foxglove 的 `layouts/D1Max-Current.json` 是此前只读导出的保存记录；源码同步不重新读取或改写 Foxglove 缓存/云端布局。监控入口 `scripts/start_live_monitor.sh` 与定位规划预览入口职责不同；历史控制源码、运动面板或 APP 退出后的 SDK 接管逻辑，均不代表本次整理授权运动或急停解除。

本机配置包含机器人局域网地址及原始路径；它们不是跨机器自动部署配置。提交目录与运行目录相互独立，后续运行目录修改需再次同步和提交。

同步规则与流程见 [tools/README.md](tools/README.md)。本次覆盖前的发布副本备份分别位于 `/tmp/d1max-source-backup-iizObI1g`（主体）及 `/tmp/d1max-source-backup-Zp0dSdVZ`（补充三份配置）；它们不纳入 Git，也不是长期备份。

## 历史整理记录

2026-09-12 基于首次发布 `54e1e4a` 同步 Web 2D/定位、MC SDK 会话、Foxglove 生命周期、Faster-LIO 恢复及高频定位输出。该轮曾在运行工作区修正浏览器回归脚本的过期文案断言；这不是 2026-09-26 整理执行的操作。彼时记录的临时备份 `/tmp/d1max-source-backup-9DQnZBc9` 不纳入 Git，也不作为当前可用的长期备份保证。
