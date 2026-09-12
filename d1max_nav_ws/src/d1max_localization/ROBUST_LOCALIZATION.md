# LIO + PCD 定位：当前默认实现

2026-09-12。入口与布局不变：Web 选择单层地图、启动、画机身初值箭头；Foxglove 查看已有定位窗口。
只做定位，不申请 SDK 控制权，不发布运动指令，不启动 Nav2，不改地图文件。
当前新增高频传播/全局 EKF/导航输出分层，详见 [导航输出架构](NAVIGATION_ESTIMATION.md)。

## 为什么替换默认融合链路

旧局部 EKF 积分 MC 三轴速度，静止时约 0.03 m/s 的速度偏差会累积成位置误差；
依赖 Python 转发的 IMU 和严格整帧去畸变门限，遇到数据缺口会持续丢扫描。
旧地图匹配与局部预测断开后，持续放宽 ICP 门限不能修复局部运动估计。

现在采用现有 Faster-LIO 的 IMU 误差状态滤波、在线零偏估计、逐点六自由度去畸变和局部点面匹配。
完整 PCD 的 FastGICP 提供全局 SE(3) 校正。MC 保留 OnMcData 遥测，不进入位置积分，也不用于平移去畸变。
没有用 RobotState 或 OnSpeedData 作为速度降级来源。

## 模块与配置

单一配置：`config/localization.yaml`。
`localization_pipeline.ros__parameters.backend: lio_pcd` 是默认值。
`legacy_ekf` 仅保留旧 Go2 双 EKF 链路回归/显式回退；两个后端不会同时启动。

| 模块 | 职责 |
|---|---|
| dual_lidar_adapter | 原始双 Airy 转换、统一时间偏移、轴向/单位、保留逐点时间；独立 IMU 回调 |
| Faster-LIO robust runtime | 200 个合格 IMU 样本初始化、局部地图、零偏估计、完整去畸变、局部 odometry |
| lio_global_matcher | 对已去畸变扫描做 FastGICP，不重复去畸变；地图质量与初值轮次验证 |
| lio_fusion.py | 纯 SE(3) 对齐、扫描时刻插值、限幅、时效与有界恢复状态 |
| lio_localizer.py | Web 初值邮箱、原始 LIO 扫描时刻地图对齐、已验证校正契约、整体状态 |
| lio_predictor.py / estimation.prediction | LIO 后验 + 最新 IMU 的有界高频传播，不修改 LIO 内部状态 |
| robot_localization ekf_navigation | 预测 twist + 已验证 PCD pose，50 Hz 全局滤波，延迟观测回放 |
| navigation_output.py / estimation.navigation | 重置握手、同时间历史/跳变/时效门控、机身参考、唯一公开动态 TF |

Faster-LIO 的新运行模式是 opt-in；普通建图 launch 的默认模式未切换。
该工作区的 Faster-LIO 含本地改动，不能只安装上游未修改版本后声称配置等效。

## 坐标与时间契约

TF：`d1max_loc_map → d1max_loc_odom → d1max_loc_base_link → d1max_loc_tracking → d1max_loc_lidar`。

- tracking 为原有归一化前 Airy 参考点，不是机身中心、足底或重力对齐后的显示坐标。
- 延续现有建图的前后雷达转换和原始 IMU 轴向转换；不改写 PCD，不给单独点云添加地面俯仰补丁。
- local odom 原点/朝向由启动时决定。地图匹配层的原始锚点关系为
  `T_map_odom = T_map_tracking(scan_time) × inverse(T_odom_tracking(scan_time))`。
- GICP 点云和其局部预测使用同一扫描结束时刻；不拿当前最新位姿冒充历史位姿，不外推未覆盖的历史扫描。
- LIO 驱动高频 IMU 传播；公开 odom→base_link 与 map→odom 由 navigation_output 统一发送；LIO 和全局 EKF 自身 TF 关闭。
- 全局 EKF 只接受预测 twist 与已验证 PCD pose，不重复融合 LIO pose / 原始 IMU / MC；仍存在部分同源相关性，协方差使用保守近似。
- 局部 pose 协方差按真实 IKFoM 顺序和坐标变换输出；地图协方差旋转并保留保守下限，仍不是实机标定的精度保证。
- 原始传感器与主机共享一次性时间偏移估计，不是 PTP 同步。仍保留“外参/时间未实机验收”标记。

## 实时预算与断流

默认预采样 4、局部体素 0.20 m、LIO 计算并行上限 4、局部体素格上限 100000。
IMU 独立回调缓冲 512，LIO 内部 IMU 上限 1024；扫描只保留有限最新帧，迟到帧不积压。
轨迹 5 Hz、最多 1800 点。状态文件由独立有界后台队列写入。

原始 bag 前 20 秒后雷达最大断帧约 0.50 s，前雷达约 0.20 s。
默认双雷达配对，前雷达最多等待后雷达 30 ms；超时保留前雷达单帧，不因缺少后雷达阻塞局部运动估计。
不扩大错误配对范围，不重复发送旧后雷达点；仍沿用前雷达 tracking 坐标与逐点时间。
降级次数在 input_clock.front_only_attempts；视野减少时几何质量门限仍生效。

原始 bag 前 20 秒 IMU 平均约 198 Hz，但在约 15.45 秒处存在约 40 ms 的真实源时间缺口：
不能把“平均高频”当成无缺口。大于 15 ms 记短缺口诊断；
不超过 50 ms 且端点角速度估计的缺口转角不超过 0.08 rad 时，用实际 dt 传播与过程噪声。
不补造 IMU 消息、不填零角速度、不虚报频率。
超过硬限不跨缺口传播。2026-09-12 实机曾出现约 60–65 ms 间隔，旧实现一次超限后永久锁存，
导致里程计和初值提交一直不可用。现在仅对 `imu_gap` / `imu_start_uncovered` 开启有界重新初始化：
撤销旧输出、清空 IMU 积分历史 / 滤波器 / 局部地图，在新局部轮次重新收集静止兼容的初始化样本。
默认一分钟内最多 3 次；用尽预算后停止，不能无限重置掩盖持续丢数。
`localization.reinitialize_on_imu_gap`、`localization.max_imu_reinitializations`、
`localization.imu_recovery_window_sec` 控制此策略；50 ms 硬限没有扩大。
时钟回跳、滤波异常和局部状态跳变仍锁定停止，需要人工核对后重启。
短缺口可传播不等于其间真实运动已知，仍由后续点云几何和全局质量检查约束。

新内部 `lio/local_sample` 将轮次、失效事件和局部里程计放在同一个有序发布流中。
`lio_localizer` 不再用另一条健康状态消息来授权一个无法识别轮次的原始 odometry。
轮次改变时清除地图对齐、历史局部位姿、匹配种子和轨迹；旧轮次消息、重置前排队的初值不能恢复定位。
重新初始化完成后必须重新给地图初值并通过连续匹配确认，不自动沿用上次初值或恢复导航。
头尾切换状态检查仍保留；界面明确提示尾部前向与地图箭头方向不是同一件事，不自动切换机器人模式。
这是数据缺口后的安全恢复，不代表已消除上游 IMU 丢帧，也不能替代外参、时间同步及实机运动验收。

LIO 几何更新随有效 LiDAR 帧，双 Airy 通常约 10 Hz。新增独立 IMU 传播和全局 RL 输出目标 50 Hz；
使用真实新 IMU、LIO 后验和有界末端保持，记录源龄，不把重复发布旧位姿当作高频估计。
200 Hz 左右是该 bag 的 IMU 输入频率。实际输出 Hz 与 MC 实收 Hz 在 Web 分别显示。

## 匹配与恢复

- 初值仍是机身 XYZ/yaw，按配置转换一次到 tracking；只有当前轮次至少 3 次连续确认后才启用全局输出。
- 新增内部 `fused_icp/pose_raw/verified` 消息，携带初值轮次、原始扫描时间与确认数。
  旧轮次结果即使时间新鲜也不得解除失锁。旧 raw pose 不作为新后端的授权输入。
- FastGICP 同时检查收敛、匹配重叠、残差、XYZ/完整旋转创新量与几何可观测性。
  Hessian 按扫描中心和共同场景尺度处理；不按各轴自身对角线归一化而掩盖弱约束方向。
- 校正超过 0.75 s 撤销 tracking；局部 LIO 健康时，从上次可靠地图关系与当前 LIO 位姿预测处重匹配。
- 超过 1 s 开始恢复，间隔至少 3 s、最多 3 次、总窗口 15 s；
  恢复校正范围默认 XY 1.5 m / Z 0.5 m / 旋转 0.7 rad。
- 不是全楼层任意地点的全局重定位。超出地图匹配恢复预算需要重新给正确初值；LIO 本体未就绪时不能靠改初值恢复。

Foxglove 可能保留失锁前的最后画面，以 Web/状态消息的 localized 和数据龄为准。
`navigation_ready` 仍为 false，不能据此授权自主运动。

## 使用与验收

1. 连接实时传感器，Web 选用正确单层地图后启动定位。
2. 启动时保持静止，等实际 IMU 初始化进度完成。
3. 在地图上画机身方向箭头；等待“定位已锁定”后观察 PCD、实时扫描和轨迹。
4. 先做静止、直行、停车、转弯验收，再逐级提高速度。物理外参、振动、时间同步与环境变化必须实机核对。

程序和离线测试完成不等于机器狗的实机高速上限已验收。
合成约 2.06 m/s、1 rad/s 的测试是算法/接口回归，不是实际机器人速度或定位精度承诺。

## 可重复测试

先 source D1 ROS 环境和 nav 工作区。两个脚本各自启动并清理本机独立 Zenoh 域，不访问机器狗：

```bash
D1MAX_QA_BACKEND=lio_pcd D1MAX_QA_MOTION=1 python3 test/smoke_localization.py
D1MAX_QA_BACKEND=legacy_ekf python3 test/smoke_localization.py
python3 test/replay_lio.py /home/dndx/d1max_rosbag903/slam_raw_20260903_232807 20
ctest --test-dir /home/dndx/d1max_nav_ws/build/d1max_localization --output-on-failure
```

运动回归生成逐点不同位姿的扫描和相应 IMU 加速度/角速度，注入静止 MC 速度偏差，
检验轨迹误差、有效输出间隔、匹配暂停后的失锁/恢复轮次，以及长 IMU 缺口的故障停用。
bag 脚本仅验证真实输入兼容性、频率、丢帧/缺口诊断，不提供定位真值。
每次产物在打印的 ARTIFACTS 目录中，保留 report.json 和节点日志。
