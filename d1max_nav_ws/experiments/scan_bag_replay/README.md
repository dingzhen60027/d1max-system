# 真实跨层 rosbag → 生产前端采样

入口 `capture_frontend.py` 只启动生产双雷达适配器和 Faster-LIO，使用当前定位配置，**不是 PCD 定位、规划或运动闭环验收**。不启动 SDK、速度控制、导航服务、RViz、全局匹配或回环；不修改 bag、地图、生产配置。

默认数据是 `slam_raw_20260923_010248_eb79d8`（09-23“跨楼层采集”，960.704 秒）。这个 bag 是当前 SC-PGO 跨层地图的原始来源；前 60 秒只能检验一楼初始运动段，不能代表楼梯和二楼验证。

## 使用

先加载现有 D1 ROS 环境及工作区，`RMW_IMPLEMENTATION` 必须是 `rmw_zenoh_cpp`。输出路径必须是尚不存在的唯一目录。准备模式不启动 ROS：

```bash
python3 /home/dndx/d1max_nav_ws/experiments/scan_bag_replay/capture_frontend.py \
  --output /home/dndx/d1max_nav_ws/log/scan_bag_replay/UNIQUE_PREPARE \
  --prepare-only
```

经当前任务协调者确认没有其他离线回归占用计算资源后，再启动一次真实采样：

```bash
python3 /home/dndx/d1max_nav_ws/experiments/scan_bag_replay/capture_frontend.py \
  --output /home/dndx/d1max_nav_ws/log/scan_bag_replay/UNIQUE_CAPTURE \
  --duration 60 --cloud-hz 5 --max-points 30000
```

私有 loopback Zenoh `domain 226 / tcp/127.0.0.1:17469`，关闭 multicast/gossip，拒绝占用端口与外部配置覆盖，只回放前雷达、后雷达及前雷达 IMU，1x、不循环、不从运动中段跳入、不发布 bag `/tf`、`/tf_static`、`/clock`。沿用生产单常量时间平移（不是硬件时钟标定）。清理仅对本工具创建的进程组进行。

## 数据与边界

- `manifest.json`：输入 metadata、配置/代码/二进制哈希，bag 文件只读大小/时间戳，隔离与采样策略。
- `sample.jsonl` + `clouds/*.npz`：按**相同扫描末尾纳秒时间戳**匹配的去畸变云、原始 LIO Odometry、原始 `lio/local_sample`。默认目标 5 Hz（实际按原扫描间隔选择、不插值），允许 0.5–10 Hz、每云最多 10 万点；保留原记录索引。不是伪造的静态 PCD 扫描。
- `odometry.jsonl`、`local_sample.jsonl`：所有接收到的原生里程计及后验状态。机体系 twist、世界系速度、偏置和重力照存，不改为零。
- `adapted_imu.jsonl`：真实输入的适配后 IMU 及精确时间戳。Odometry 不发布加速度，样本中的 acceleration 为 null；不能填零冒充测量。需要加速度时必须用 IMU、同一时间姿态/偏置/重力明确重构，并标注派生来源。
- `raw_sensor_headers.jsonl`：原始 bag 雷达/IMU header 时间戳，不与平移后的 LIO 时间混淆。
- `status.jsonl`、`input_clock.jsonl`、`result.json`：包含失效状态、重置 epoch、缺配与队列溢出。捕获成功只说明拿到真实样本，**不说明定位或绕障成功**。

写入在有界后台队列处理，总采集输出限 512 MiB。过载直接报告失败，不悄悄丢弃并伪称通过。双雷达去畸变合并云没有保留每个点的原始激光原点；后续只用一个原点进行射线清空是近似，不能当作准确多雷达自由空间证据。

这次测试沿用生产**前雷达 IMU**定位输入；历史建图使用中心 IMU 实验外参及 -13 ms 偏移。两者用途不同，不应通过这个采样工具静默切换生产传感器或宣称复现了原建图标定。

## 原生局部规划器差分回放

`replay_native.py` 读取已经完成且校验通过的 capture，在另一个私有 loopback `domain 227 / 17470` 中启动原生局部规划器。每个 profile 使用完全相同的真实点云、位姿、速度、时间间隔；只对所有消息时间戳加一个常量，使其适配墙钟鲜度保护。云与对应雷达位姿保持相同时间戳，机身位姿和速度使用生产 `body_to_tracking_transform` / `body_state` 转换，包含角速度引起的参考点速度差。

```bash
python3 /home/dndx/d1max_nav_ws/experiments/scan_bag_replay/replay_native.py \
  /home/dndx/d1max_nav_ws/log/scan_bag_replay/COMPLETED_CAPTURE \
  --seconds 35 --profiles strict2 strict2_v06 occupied2_v06 \
  --output /home/dndx/d1max_nav_ws/log/scan_bag_replay/UNIQUE_NATIVE_RUN
```

| Profile | 前瞻长度 | 完整观测空间要求 | 速度上限 |
|---|---:|---|---:|
| strict6 | 6 m | 开 | 生产预览配置（目前 0.30 m/s） |
| strict2 | 2 m | 开 | 生产预览配置 |
| strict2_v06 | 2 m | 开 | 仅实验覆盖 0.60 m/s |
| occupied2 | 2 m | 关（占用障碍对照） | 生产预览配置 |
| occupied2_v06 | 2 m | 关（占用障碍对照） | 仅实验覆盖 0.60 m/s |

所有 profile 保留生产机身尺寸与加速度约束，不解锁实机、不发布速度。`require_observed_free` 也会切换原生严格射线积分分支，因此 strict/occupied 不是仅改变查询阈值的单因素比较。`waiting_observed_space_fraction` 只指状态持续时间，**不是地图中未知体素的比例**。

参考路径取自后来记录到的机身轨迹，是带有未来信息的离线诊断参考，**不是 PCT 的结果，也不是 ground truth**。录制运动不会随规划结果改变，不能测试真实跟踪、闭环绕障或动态障碍响应。此工具可以回答“真实观测和动态初态下，原生规划器在哪一环拒绝”，不能给完整导航签发通过结论。原生 FSM 当前初始加速度为零假设；这是未测边界，不冒充 IMU 测量。

输出保存原生曲线控制点、结点、实际样条采样的速度/加速度、各阶段时长、输入超出配置速度的比例、实际投递延迟和进程清理证据。0 条输出也是需要保留的失败结果，不自动重试或改输入。`summarize_runs.py` 可对同输入哈希、同二进制版本的多次完整回放生成对比图，不覆盖原报告。

汇总还逐条反查原生曲线的初始位置与速度：按原始常量时间平移，在曲线起始前 1 秒的实测位姿里匹配精确位置，然后比较该帧的世界系速度；找不到源帧时标为未核实，不能当作通过。这样既能发现内部错误清零实测反向速度，也不会把计算期间新到达的里程计错误当成原生求解器原本的输入。该审计只检查初态传递，不证明执行时的轨迹拼接连续性。

纯离线测试（不初始化 ROS）：

```bash
python3 -m unittest discover -s /home/dndx/d1max_nav_ws/experiments/scan_bag_replay -p 'test_*.py' -v
```
