# 实机读取外参的隔离候选

日期：2026-09-19。用户授权逐组尝试之前从机器狗读取的标定关系。本目录保存**离线诊断候选**，不代表已验证的生产外参；生成过程不连接机器人、不启动 ROS、不更换通信中间件。

## 执行结果：三组短段均完成，但下沉未解决

三组实际回放均完成，完整性和对照控制检查全部通过。相同 180 秒输入窗口内，各组均接收 1,660 组配对扫描、输出 1,641 个前端状态；共同输出覆盖约 177.800 秒。每组均无回环、无地面约束，保持中心 IMU、1×速度和 Zenoh。

| 组别 | 估计 XY 路程 | 估计原点 ΔZ | 滤波器末端重力倾角 | 点面残差中位数 |
| --- | ---: | ---: | ---: | ---: |
| 原配置复现 | 39.665 m | −0.823043 m | 0.9582° | 0.01980 m |
| 只换本机前雷达 DIFOP 旋转 | 39.679 m | −0.735354 m | 0.9750° | 0.01959 m |
| 上组＋机器实际加载的后→前外参 | 39.584 m | −0.793612 m | 0.7431° | 0.02183 m |

第二组的下降幅值比原配置减少约 10.65%，但各 30 秒分段仍持续下降；第三组没有进一步减小最终下降量。第三组的重力倾角和估计加速度偏置模长有所减小，但点面残差增大、高度仍持续下降，不能用其中一个指标认定其整体更可靠。**局部残差、估计偏置及 ΔZ 都不是独立真值精度；不能把 10.65% 说成定位精度提升。**

这些结果说明：在当前消息基底解释和中心 IMU 链下，直接替换这几组实机读取关系不足以解决问题。它们既不能证明厂家标定一定错误，也不能证明中心 IMU 已损坏，不能作为将候选直接发布到默认建图链路的依据。本轮不继续盲目扩展整包，不以回环或压平处理掩盖剩余问题。

完整数值、分段趋势与检查项见 [对照结果](comparison.md) 和 [机器可读结果](comparison.json)。

### 保存结果

| 组别 | PCD（`scans.pcd` 是同一物理文件的软链接） | 轨迹与配置目录 |
| --- | --- | --- |
| 原配置 | [PCD](../runs/extrinsic_baseline_180s_20260919/scans.pcd) | [运行目录](../runs/extrinsic_baseline_180s_20260919/) |
| 前雷达设备旋转 | [PCD](../runs/extrinsic_device_front_180s_20260919/scans.pcd) | [运行目录](../runs/extrinsic_device_front_180s_20260919/) |
| 前雷达设备旋转＋后雷达运行外参 | [PCD](../runs/extrinsic_device_front_rear_180s_20260919/scans.pcd) | [运行目录](../runs/extrinsic_device_front_rear_180s_20260919/) |

每组只有一份物理 PCD，未保存逐关键帧点云；三组各自完整保存配置、PCD、TUM、状态 CSV、扫描清单和进程日志。没有覆盖现有地图。

### 输入、版本与收尾证据

- 三组 admitted 扫描时间序列、输入/输出点数、前端状态时间序列一致；前端参数差异仅为本次允许的外参项。
- 原配置复现的 `t/x/y/z/gx/gy/gz` 全部 1,641 行与上一轮 `dual_180s` 逐值一致，说明本次实验启动扩展没有改变基准结果。
- 运行时实际加载库均为默认 `install/faster_lio/lib/libfaster_lio_lib.so`，SHA256 `8f7a66b4a545a24fce4ba36cbc9764e16e8a2b9f5361833a376738831111f069`。可执行文件也一致；各自 `runtime_binary_check.json` 是实际 `/proc` 核验，不仅是预期配置。
- 中心 adapter 实收/发布分别为 35,538、35,539、35,539 条。与缓存原 bag 中各自首尾时间范围内的样本数完全一致；末端停止缓冲的 1 条差异不改变限定点云窗口。这里只确认 adapter 实收计数和覆盖，不冒充算法逐条 IMU 接收内容哈希。
- 三组均完成保存、原 bag 文件状态未变、legacy Log 恢复。进程身份/隔离域/端口/默认库与后雷达 TF 日志检查均通过：[基准收尾](baseline_completion_check.json)、[第二组收尾](device_front_completion_check.json)、[第三组收尾](device_front_rear_completion_check.json)。
- 最后核验时自有实验进程全部退出，Domain 219 无残留，7447 端口释放。未连接机器狗，未发送 SDK 命令，未修改默认算法参数或默认安装库。

## 最小三组

| 名称 | 相对前一组变化 | 不变项 |
| --- | --- | --- |
| `baseline` | 无；把当前实际 rear TF 显式写进实验配置 | 原中心 IMU 配置、点云前置旋转、R/t、时间补偿 |
| `device_front_rotation` | 只按本机前雷达 DIFOP q 重新组合雷达→中心 IMU 旋转，差 0.5782528° | 中心位置、数据、时间补偿、历史点云前置旋转、rear TF |
| `device_front_plus_runtime_rear` | 只把 rear→front 换成机器人本次启动日志证明已载入的 OTA R/t；相对原值差 4.9208435° / 0.183559 m | 前一组雷达→中心 IMU R/t、数据、时间补偿、点云前置旋转 |

候选均在 `calibrations/` 下，运行器通过 `--calibration` 选择。主实验需使用同一 bag、同一有效扫描选择、同一前端参数、同一初始段，禁用回环、地面先验和在线外参估计。中间件仍为 `rmw_zenoh_cpp`。

第三组是一个有来源的“rear 整体刚体变换”变量，不把其三轴角度和平移逐一任意翻转。若需要分离 front/rear 相互作用，可在上述结果之后再考虑 `baseline + runtime_rear`，不先做无目的参数网格。

## 严格方向与 schema

列向量定义 `p_target = R_target_source p_source + t_target_source`，所有展平矩阵为逐行 9 元素，平移单位 m。N 是当前归一化点云轴，F 是前雷达内置 IMU 原始轴，L 是原始前雷达点云轴，C 是中心 IMU 原始轴。

```text
R_C_N_device = R_C_F_gyrofit · R_F_L_DIFOP · R_N_L_existingᵀ
```

**保留原点云前置 R_N_L，不再向点云追加 DIFOP 旋转；中心 adapter 不旋转。** 这才能保证本次只改变一处有效物理关系。

```yaml
lidar_extrinsics:
  rear_to_front:
    parent_frame: rslidar_head
    child_frame: rslidar_tail
    rotation: [9 个逐行矩阵元素]
    translation: [x, y, z]
```

`rear_to_front` 表示 `p_front = R_front_rear p_rear + t_front_rear`。实验 launch 应在转换成 xyzw 前验证旋转矩阵属于 SO(3)、平移有限、parent/child 名称匹配；不能隐式正交化任意矩阵。没有这个可选字段时应保留原默认行为。

## 来源与明确不测的配置

- 前雷达 SN `3009bede1530`：2026-09-17 15:29 设备 DIFOP 原始 q。模长 `0.9999999980707005`，仅显式归一化 float32 舍入；不是对畸形旋转矩阵做拟合。
- rear 来源：`localization_runtime_20260917.json.logged_loaded_calibration.rear_lidar_to_front_lidar`。启动日志确认内部配置已被 OTA 覆盖，**不是** ROS 参数服务仍暴露的默认 R/t。其 SO(3) 正交误差 `3.55e-15`，det≈1。
- **拒绝** `front_lidar_to_front_imu` OTA 矩阵：正交误差 `0.0157415321`，不是合法刚体旋转。不能静默 SVD 正交化、转换为 q 或把某一位小数改掉后当成厂家标定。
- 中心 `t_C_N` 是此前名义值，本次固定；DIFOP 的约 4 mm 内置 IMU 杆臂不等于中心 IMU 杆臂，不能直接替换。

设备 q 的应用方向、bag 采集时驱动是否另施加了变换、rear 命名矩阵是否与 bag 原始点基底完全一致仍需验证。候选运行可以提供性能证据，不能仅因短段更好就宣称物理标定已通过。

## 复现与审计

`generate_candidates.py` 只向 stdout 打印“文件名→内容”的 JSON 字典；不写文件、不运行任何节点。`manifest.json` 保存源文件 SHA-256、SO(3) 校验、各组完整有效矩阵和被拒绝数据。

```bash
python3 /home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/extrinsic_trials_20260919/generate_candidates.py
```

本目录没有修改默认映射配置或源 bag。实际回放结论由执行者另记，不能把“候选生成成功”当作“下沉已修复”。

## 本轮实际执行方式

三个候选分别运行同一个 bag 首个配对扫描起的 `[0, 180 s)` 窗口，1×速度，双雷达输入，中心 IMU 原始数值与 −13 ms 时移不变。关闭回环、地面约束、在线外参估计和 RViz，使用同一默认安装 Faster-LIO 库。实验位于 `../runs/extrinsic_*_180s_20260919/`，不进入生产地图目录。

本轮只扩展 `experiments/central_imu_fasterlio_20260918/mapping.launch.py` 的可选后→前配置读取。没有该字段时保留旧值；含字段时校验有限性、维数、SO(3) 与固定父子坐标名称，不修补非法矩阵。新增检查已覆盖原默认值不变、非有限值、反射矩阵、非正交矩阵和错误坐标名称。

实验 `run.py` 在节点就绪、回放开始前记录 `runtime_binary_check.json`，从 `/proc/PID/exe` 和 `/proc/PID/maps` 取得实际可执行文件和实际加载库路径及 SHA256。静态 manifest 的预期文件 hash 不替代这项证据。

逐组命令为：

```bash
cd /home/dndx/d1max_nav_ws
bash experiments/central_imu_fasterlio_20260918/run.sh \
  --bag /home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38 \
  --config /home/dndx/d1max_nav_ws/maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh/config/frontend.yaml \
  --calibration /absolute/path/to/one/candidate.yaml \
  --output /home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/runs/NEW_UNIQUE_RUN \
  --input-window 180 --cloud-selection dual --rate 1 --no-rviz --exit-after-save
```

`--calibration` 必须选上面三份真实文件之一，`--output` 必须是新目录。不要并发运行，会共享旧版 Faster-LIO 的 legacy Log；runner 会备份恢复该日志，只停止自己创建的进程组。

评估工具：`evaluate_trials.py` 比较实际扫描时间/点数、有效参数、时间补偿、运行库及共同时间窗口指标；`verify_trial.py` 在组间检查子进程退出、隔离域无残留、7447 端口释放、默认库未改和候选后雷达 TF 的启动日志。
