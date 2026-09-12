# 源码快照记录

首次整理：2026-09-10；本次增量同步：2026-09-12（Asia/Shanghai）。

本仓库首次提交是当前工作文件的整体快照，不改变原工作区或导航仓库的提交历史。

本次基于首次发布 `54e1e4a`，同步实际运行工作区中尚未发布的 Web 2D/定位、MC SDK 会话、Foxglove 生命周期、Faster-LIO 恢复及高频定位输出源码和测试。
统一提交位置 `/home/dndx/d1max-system`，远程私有仓库 `https://github.com/dingzhen60027/d1max-system`，分支 `main`。
整理只更新发布副本及其项目级文档，不移动运行目录、不提交/改写原导航仓库历史，不重新部署或停止运行服务。
运行工作区仅同步修正了 `map_manager/frontend/scripts/verify-copy.mjs` 的过期故障文案断言，并增加故障原因和禁用初值按钮的检查；没有修改业务 UI、SDK 或定位算法。

## 原工作区

- 应用、ROS 接入：`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0`
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

Foxglove 的 `layouts/D1Max-Current.json` 是此前只读导出的保存记录；本次与运行工作区中的导出文件同步，未重新读取或改写 Foxglove 缓存/云端布局。旧控制源码虽被保留，当前正式入口仍是 `scripts/start_live_monitor.sh`，不能把历史入口视为恢复运动控制的授权。当前经用户授权的 APP 退出后 SDK 接管逻辑，不代表允许运动或急停解除。

本机配置包含机器人局域网地址及原始路径；它们不是跨机器自动部署配置。提交目录与运行目录相互独立，后续运行目录修改需再次同步和提交。

同步规则与流程已纳入 [tools/README.md](tools/README.md)。本次被覆盖的旧发布副本已暂存于 `/tmp/d1max-source-backup-9DQnZBc9`，没有删除原始数据；该目录不纳入 Git，也不应当作长期备份。
