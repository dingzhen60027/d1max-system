# 源码快照记录

整理日期：2026-09-10（Asia/Shanghai）。

本仓库首次提交是当前工作文件的整体快照，不改变原工作区或导航仓库的提交历史。

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

Foxglove 的当前布局通过只读读取其保存记录导出为 `layouts/D1Max-Current.json`，不改写 Foxglove 缓存。旧控制源码虽被保留，当前正式入口仍是 `scripts/start_live_monitor.sh`，不能把历史入口视为本次恢复运动控制的授权。

本机配置包含机器人局域网地址及原始路径；它们不是跨机器自动部署配置。提交目录与运行目录相互独立，后续运行目录修改需再次同步和提交。
