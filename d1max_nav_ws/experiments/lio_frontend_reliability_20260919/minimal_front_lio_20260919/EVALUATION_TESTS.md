# 跨 IMU 链公平评估方法与离线测试

本说明只描述评估实现和测试，不包含尚未完成的实际回放结论。

## 比较的物理点必须相同

Faster-LIO 状态位置为 IMU 原点。中心 IMU 和雷达内置 IMU 的位置不同，因此先按实际候选的雷达→IMU 外参变换到前雷达物理原点：

```text
p_W_N = p_W_I + R_W_I · t_I_N
R_W_N = R_W_I · R_I_N
```

N 是现有归一化点云坐标系，其原点仍为前雷达原点。两组世界坐标分别由各自 IMU 初始重力对齐，没有强行对齐 yaw，也没有计算两条不同 world 中 XYZ 的直接差作为定位误差。报告比较各自共同起点后的雷达原点 ΔZ、XY 路程及趋势，并同时列出原 IMU 原点 ΔZ，以看清杆臂转换的影响。

IMU 偏置分量仍保留在各自传感器原始轴中；不能跨传感器直接相减。

## 严格控制与白名单

允许的运行配置变化逐一列举为：IMU 输入话题、原始轴 frame、各链指定时移、雷达→IMU R/t，以及对应的前端 body_frame。中心链要求 −13 ms，内置前 IMU 链要求 0 ms；不是“时移任意变化都可以”。

加速度/角速度尺度、噪声、点云选择、扫描序列与点数、归一化点云配置、后雷达 TF、脚本和实际运行库仍比较。`trial/provenance` 作为已核验不参与执行的描述性元数据公开保留；任何未知新增执行字段仍参与严格比较。

换 IMU 源可能改变静止初始化完成时刻，因此不要求两组总输出帧数或第一条状态完全相同：按扫描末端时间在 100 μs 内一一匹配，比较共同状态范围，并公开各组初始化延迟、共同范围外的首尾行和共同范围内的缺行。各组 CSV/TUM/result 自身必须自洽。仅规范化分析的时间轴，不插值位姿。

历史组可带 `--legacy-reference` 作描述性参考，但该选项**不豁免** launch 不同、运行库证据缺失等失败。

## 运行示例

本轮中心设备链与内置设备链：

```bash
python3 compare_imu_chains.py \
  ../runs/central_device_paired_front_180s_20260919 \
  ../runs/internal_device_paired_front_180s_20260919 \
  --output central_vs_internal_device.json
```

内置 IMU 不变，只比较旋转候选：

```bash
python3 compare_imu_chains.py \
  ../runs/internal_device_paired_front_180s_20260919 \
  ../runs/internal_historical_paired_front_180s_20260919 \
  --reference-role front --candidate-role front \
  --output internal_rotation_pair.json
```

输出 JSON 和同名 Markdown，必须是新路径，不覆盖旧报告。脚本复用相邻 `extrinsic_trials_20260919/evaluate_trials.py` 的读取、完整性与描述性指标函数，但没有修改该旧脚本。

## 已执行的离线测试

```bash
OPENBLAS_NUM_THREADS=1 python3 test_compare_imu_chains.py
```

8 项全部通过：

1. 用两种真实候选 R/t 合成不同 IMU 原点轨迹，存在平移和三轴姿态变化，转换后准确恢复同一雷达原点和姿态。
2. 世界坐标原点平移和任意 yaw 改变后，相对高度变化与 XY 路程保持不变。
3. 当前真实中央/内置候选仅出现白名单允许的执行差异。
4. 非许可的加速度尺度、噪声或未知字段变化被识别。
5. 即使时移在白名单内，内置链错误沿用 −13 ms 仍被角色约束识别。
6. 后雷达外参、launch 改动不会被静默放行。
7. 初始化首帧不同可匹配共同扫描状态，内部缺行会被报告。
8. 支持相同内置 IMU 源的纯旋转候选比较。

测试不创建 ROS 节点、不回放 bag、不连接机器人，不修改默认配置或既有报告。通过这些测试表示评估逻辑经检查，不表示某条实际 LIO 链已经可靠。
