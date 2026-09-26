# MOLA 离线建图模块 v1

## 使用

Web → **3D 地图 → 建图管理 → MOLA-LIO + 离线回环**。

1. 填写包含 `metadata.yaml` 的完整 rosbag2 目录。
2. 展开 YAML，核对 LiDAR / IMU 话题、`base_frame` 和外参来源。
3. 校验配置，点击“开始离线建图”。完成阶段自动保存，结果进入原始建图点云列表，可对比、处理、归档或作为 2D 地图的输入。

默认方案仍是 Faster-LIO + SC-PGO。MOLA 不替换在线定位、不连接 SDK、不改变 Foxglove、不发机器人指令，不启动 rosbag play。

## 模块边界

| 文件 | 职责 |
| --- | --- |
| `registry.py` | 建图方案登记与统一启停路由；原三个启动脚本保持不变 |
| `configuration.py` | 严格 YAML 模型、参数范围和完整 bag 校验 |
| `inspect_bag.py` | 只读检查消息、逐点时间、IMU 和坐标系前提 |
| `normalize_bag.py` | 按明确单位配置生成任务私有 SI 输入副本，不改原 bag |
| `mola_runtime.py` | 独立 systemd cgroup、任务身份、互斥、取消与恢复 |
| `worker.py` | 调用原生工具，分阶段保存与错误判定 |
| `toolchain.py` | 私有依赖与离线进程环境，不继承实机 ROS 环境 |
| `artifacts.py` | 只把已完成的 PCD 阶段登记进地图列表 |
| `native_live/live_sink.hpp` | 观察 MOLA 原生已配准去畸变输出，异步累积有界全局显示图 |
| `visualization.py` | 将实时显示图发布到原 Foxglove 窗口；不执行 SLAM、定位或控制 |
| `frontend/src/MolaMappingOptions.jsx` | Web 的配置编辑、校验与下载 |

处理顺序：输入检查 → MOLA-LIO GICP+IMU → sm2mm/前端 PCD → FrameToFrameLoopClosure GICP+GNC → sm2mm/回环 PCD。

主配置：`../../config/mapping/mola_lio_lc.yaml`（相对本目录）。上游原生 YAML 不接受浏览器直接执行；Web 参数经过白名单验证，再应用到可信模板，防止上游扩展 YAML 的表达式执行。

新算法通过 registry 登记和独立 adapter 接入，不在 SDK、定位或 Web 页面里增加算法分支脚本。当前 MOLA v1 是一套固定的前后端组合，不宣称任意第三方前后端已兼容。

## 坐标、时间和地图语义

- 使用 3D 模型，禁用平面假设和无来源的 GNSS 约束；不自动找地面、拉平楼层或添加 Z 偏移。
- `bag_tf` 使用录包里的传感器 TF；缺 TF 且 frame 不同则拒绝运行。`fixed` 必须填写真实外参：相对 `base_frame`，顺序 `[x,y,z,yaw,pitch,roll]`，单位米、度。不要把不同传感器的外参默认为零。
- **D1 Airy 特例：点云和 IMU 都标成 `rslidar_head`，但物理坐标轴不同。** 已知原生 D1 话题不允许通过同名 TF 得到单位外参；必须分别设置。D1 默认 YAML 和 `config/mapping/experiments/d1max_903_mola.yaml` 已采用原 `d1max_slam` 的点云四元数标定及 IMU `Rx(+90°)`，加速度 `g→m/s²`。不是任意雷达、固件或另一台设备的通用标定。
- 初始俯仰/横滚来自 IMU 重力初始化，不信任未标定的 orientation 四元数。录包开始需有适合初始化的平稳数据；配置检查不是外参、同步或精度验收。
- 必须有有效逐点时间和陀螺/加速度；不偷偷关闭去畸变。样本校验不等于全包时间覆盖校验。
- 原生 simplemap 同时保留 raw 和 deskewed 观测。回环与 sm2mm **只使用标签 `deskewed`**，不重复混合或再次去畸变。
- simplemap 关键帧间隔相对上一关键帧测量，保留重访区域；前端地图裁剪半径与地图插件的删除半径一致。
- Web PCD 为体素降采样后的 **XYZ** 派生图；原始 rosbag 只读，`.simplemap`、扫描文件和 `.mm` 保留用于后续重新导出。当前 PCD 导出不承诺保留强度。
- 原生 GNC 内点/外点统计是诊断证据，不等于建筑地图正确、多楼层闭环或导航安全已验收。无统计显示未知；零内点不叫闭环成功。

## 保存与生命周期

结果目录：`map_manager/data/mola/<任务 UUID>/`。

- `task.yaml`：本次完整配置，包含已解析的输入路径。
- `frontend.yaml`、`loop.yaml`、`estimator.yaml`、`export.yaml`：真正执行的原生配置。
- `upstream_*.yaml`、`packages.json`、`compat.json`：模板、包版本和兼容补丁来源。
- `raw.simplemap`、`raw_Images/`、`raw.mm`、`raw.pcd`、`trajectory.tum`。
- 开启回环时另有 `optimized.simplemap`、`optimized.mm`、`optimized.pcd` 和原生优化轨迹。
- `manifest.json`、阶段 `.log`：结果状态、回环统计、命令和错误。界面进度是阶段进度，不是扫描帧百分比。

仅完整写出的 PCD 才登记。失败/取消保留已完成阶段，不自动选用地图；归档是无损标记，归档后的永久删除只删除所选 PCD 及其预览缓存，**原生关键帧、日志仍保留**。

生产服务 `d1max-mola-mapping.service` 用 UUID 标记归属，禁止重复启动和运行中切换。取消先通知任务，再停止整个 cgroup；12 秒后兜底清理。未知占用不抢杀、启动回执不明确不重试。Web 正常退出或关闭受管 Web 服务也会停止 MOLA；Web 异常退出后重新打开则按 UUID 核对任务，不创建副本。

MOLA 使用独立离线环境：`rmw_zenoh_cpp`、Domain 214，关闭 discovery、只允许 loopback listener，不连接实机 Domain 24 或实机 Zenoh router。不使用 FastDDS。

## 私有安装与复现

从项目根目录运行（下载需要网络；无需 sudo）：

```bash
python3 d1max_ros2/map_manager/scripts/install_mola_isolated.py --prefix d1max_ros2/map_manager/.local/mola
python3 d1max_ros2/map_manager/scripts/build_mola_compat.py --prefix d1max_ros2/map_manager/.local/mola
```

本机验证版本为 MOLA-LIO 3.0.0、MOLA metric maps 3.0.0、LC 1.2.2、mp2p_icp 2.12.0；具体 deb 完整版本在 `.local/mola/packages.json`。依赖旧机器已有的 Humble、Zenoh 和 Web Python 环境，不是跨发行版独立二进制包。升级后须重新检查兼容性，不能直接覆盖版本后假定可用。

Humble 二进制的 RKNN 兼容故障见 [上游问题 #194](https://github.com/MOLAorg/mola/issues/194)。私有补丁固定源码提交 `e40c814584e5d787e7bf69e287a7c3e8d732e948`，仅将不支持的 RKNN 调用替换为 KNN 后相同半径截断；不关闭半径约束、不替换算法。编译结果只覆盖本模块的库搜索路径，不改 `/opt/ros/humble`。

`.local/`、`data/`、构建和浏览器截图均已加入本目录 `.gitignore`。MOLA 部分包使用 GPLv3；分发二进制时需保留相应许可证和源码/补丁，不能按纯 BSD 项目处理。

## 验证

- `tests/test_mapping_plugins.py`：配置/输入、原生假成功拦截、地图登记/对比、任务恢复和原有方案路由。
- `frontend/scripts/verify-mola.mjs`：API 全拦截的浏览器测试，不发送真实任务。
- `scripts/smoke_mola_offline.py`：生成临时合成 LiDAR+IMU rosbag，实际执行原生链路，输出在 `/tmp/d1max-mola-smoke-*/`，不登记进用户地图。
- `D1MAX_MOLA_QA=1 ... scripts/verify_mola_cgroup.py`：唯一 QA 服务验证重复启动、恢复、主/子进程停止，不启动 ROS 或 SDK。

合成数据验证的是集成正确性，不是实采地图精度或多楼层效果。

### 2026-09-13 实采 903 排查

首轮任务 `49b6b6e25827436b83d4d8f39f273fb7` 已完整执行，但错误地将同名点云/IMU frame 当成相同物理坐标，地图重叠。其 manifest 已标记 `quality.status=rejected`，PCD 只保留作排查，不能用作导航地图；GNC 有效回环为 0。

复用已保存的 SI 副本，修正独立外参后的 96.5 秒短段对比：113 个准静态样本的重力方向偏差中位数由 4.16° 降至 1.51°，90% 分位由 68.04° 降至 4.86°；这是短段外参回归，不是整包精度验收。

修正后的完整任务 `35cc38b8b5fb4a3fa8cfd5a6585e6362` 已完成：前端/回环处理 PCD 各 784,148 点；有效回环仍为 0。全包 509 个相同筛选条件样本的重力偏差中位数 2.59°、90% 分位 5.30°、最大 21.50°，没有轨迹真值精度验收。原生实时显示累积 907 次去畸变扫描更新、334,386 点，队列丢帧 0；日志记录了最终 PCD 不存在时地图已持续增长，不能把“最终结果审阅”当成“实时建图”。

### Foxglove 实时全局图（不是最终 PCD 播放）

运行 `scripts/build_mola_live.py` 构建私有显示 CLI。它只在固定上游 CLI 增加 MOLA `subscribeToMapUpdates` / `subscribeToLocalizationUpdates` 观察者，链接原估计器库，**不修改估计器或 SDK**。输出快照及构建来源均保存在任务目录。

- 原生 `deskewed_scan` 已经在 `d1max_loc_map` 下；直接增量融合，不能再次乘位姿。回调只复制共享数据引用与位姿，计算和文件操作在独立线程。
- 全局显示图按体素去重，默认 0.15 m、每 1 秒更新，超过 100 万点时整体增大体素保留全局范围。显示缓存不是滚动局部地图，也不替代 0.1 m 的最终导出图。
- 队列最多 8 帧，丢帧计数写入 `live/status.json`；显示故障保留到 `live/error.txt`，不反向改动算法。`live/global.bin` 原子替换，`live/trajectory.txt` 为对应实时轨迹。
- 原 Foxglove 左侧只显示这一张增量全局图和轨迹；右侧回放跟随 MOLA 实际处理的传感器时间，不再按独立时钟回放，也不把原始扫描加到左侧。默认不读取任何最终 PCD。
- 显示服务跟随 Web 的 MOLA 任务 UUID；下一次建图会清空旧地图/轨迹，仅替换发布子进程，保留桥接连接和布局，不继续显示上一任务。实验临时目录不自动跟随生产任务。
- 结束后保留最后的实时全局图。当前后端回环仍是离线批处理，**不宣称实时图已随回环重优化**；最终优化 PCD 是单独保存的结果。显式 `--saved-map` 才进入结果审阅模式。
- 显示服务用 Domain 215、localhost Zenoh 17749 和只读 Foxglove `ws://127.0.0.1:8769`；建图仍在隔离 Domain 214，无实机 SDK 或控制发布。
- 停止显示：`systemctl --user stop d1max-mola-view.service`。停止 Web 会联动停止其受管任务；返回实机监控前先停离线显示，不能抢占同一个 8769 端口。

官方流程参考：[MOLA pipelines](https://docs.mola-slam.org/latest/mola_lo_pipelines.html)、[离线回环示例](https://docs.mola-slam.org/latest/tutorial-ouster-mapping-lc.html)。具体接口以本机固定版本源码/帮助为准，不照搬新版本或其他雷达的外参。
