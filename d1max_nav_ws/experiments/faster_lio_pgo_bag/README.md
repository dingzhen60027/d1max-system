# Faster-LIO + SC-PGO：独立 bag 对比实验

使用当前已编译前端、适配器及 SC-PGO。2026-09-17 大回环修复说明见
`../../docs/LARGE_LOOP_REPAIR_20260917.md`；前端初始化与回环配置分别由
`faster_lio_airy96.yaml`、`sc_pgo_d1max.yaml` 管理。此入口不修改标定。
默认恢复既有双雷达输入：`/front_lidar` + `/rear_lidar` + `/front_lidar/imu`。
只有用户明确要求单雷达诊断时才可传入 `--lidar-mode front`，不能静默降级。
必须使用**原始 bag**，不要传入 MOLA 已经转换加速度单位的 `input_si_front`，避免二次乘以 g。

```bash
bash run.sh --bag /path/to/original_bag --output /home/dndx/d1max_nav_ws/maps/runs/NEW_NAME
```

通信中间件固定为 **`rmw_zenoh_cpp`**。这是用户约束，未经明确同意禁止更换为
CycloneDDS、Fast DDS 或其他中间件，也禁止故障时自动回退。
复用项目 `d1max_ros2_env.sh` 和已有本地 Zenoh 配置，实验 Domain 217 保持不变。
只连接 `127.0.0.1:7447` 的现有本地路由；路由未运行时明确拒绝启动，不自动连接实机。
每轮独立保存配置快照，不修改历史运行快照。
按所选雷达模式回放所需传感器，不回放厂家 TF，不改消息原始时间戳；
双雷达模式沿用既有适配器的 ring 偏移及 192 线配置，不另改外参。
PGO 使用运行时快照中的门限，关闭里程计输出的系统时间覆盖；动态 TF 保留输入时间。
历史外参仍待本机核验，本实验不将其称为新标定。

RViz2 显示 PGO 累积地图、最新前端扫描和细轨迹。退出窗口不影响回放；回放结束自动
保存前端 PCD、SC-PGO PCD、逐帧 TUM 里程计及回环事件，再结束算法进程，仅保留 RViz。
`result.json` 验证全程时间覆盖与文件完整性，不等于地图精度验收，也不等于回环成功。

运行目录内：`manifest.json`（输入/二进制哈希）、`config/`、`status.json`、`processes.json`、
`frontend_odometry.tum`、`scans.pcd`、`sc_pgo/optimized_map.pcd`、`sc_pgo/loop_events.csv`、日志。
旧前端有固定 `Log/traj.txt` 输出，启动前保存到 `legacy_logs_before/`；实际本次完整轨迹独立记录。
仅终止本任务持有的子进程，重复启动由 flock 拒绝；不运行历史启动脚本中的全局 pkill。

加 `--save-keyframes` 可保留原始去畸变 body 关键帧（不是原始 bag 的替代品），
后续不必重放完整 bag 即可核对回环配准。完整结束后运行：

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 validate_loop_geometry.py /path/to/run
```

生成独立几何报告：用最后一帧与前 9 个原始关键帧组成的小子图重新配准，对比前端和 PGO 的闭合误差，
并抽样检查地面法向；不会覆盖 PCD，也不会将内部一致性冒充测量基准精度。

只重算后端时，在相同 Zenoh 环境（`ROS_DOMAIN_ID=217`、现有本地路由配置）运行：

```bash
python3 reprocess_keyframes.py /path/to/run --output /path/to/run/NEW_PGO_DIRECTORY
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 validate_loop_geometry.py /path/to/run \
  --pgo-dir NEW_PGO_DIRECTORY --report NEW_GEOMETRY_REPORT.json
```

输出必须为原运行目录下的新子目录；原地图和关键帧不覆盖。重算时关键帧全部保留，
加快候选检测频率以匹配加速输入、末段限速等待几何核验，几何门限不变。
因此这是同一批真实关键帧的后端复验，不冒充另一次完整传感器实时回放。
