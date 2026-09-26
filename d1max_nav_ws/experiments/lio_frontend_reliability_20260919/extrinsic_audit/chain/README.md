# 中心 IMU 外参代数链核查

2026-09-19。只读核对本轮保存的配置、源代码与先前设备记录；没有启动 ROS、连接机器人、修改默认参数或回放 bag。复现命令：

```bash
python3 /home/dndx/d1max_nav_ws/experiments/lio_frontend_reliability_20260919/extrinsic_audit/chain/audit_chain.py
```

输入 SHA-256、重算数值和条件性矩阵保存在 `chain_report.json`。它们是诊断结果，**不是生产标定文件**。

## 实际链路

所有变换采用列向量 `p_target = R_target_source p_source + t_target_source`。L 为原始前点云，R 为原始后点云，F 为原始前雷达内置 IMU 轴，N 为当前归一化点云轴（原点仍在前雷达），C 为原始中心 IMU 轴。

```text
前雷达：p_N = R_N_L p_L
后雷达：p_N = R_N_L (R_L_R p_R + t_L_R)
Faster-LIO：p_C = R_C_N p_N + t_C_N
世界输出：p_W = R_W_C p_C + t_W_C
```

`R_C_N = R_C_F Aᵀ`，其中 A 是旧前 IMU 适配器的 `Rx(+90°)`：`(x,y,z)→(x,-z,y)`。

关键事实：`R_C_F` 来自两颗 IMU 的角速度拟合，**不是雷达点云到中心 IMU 的直接标定**。当前仍隐含使用 `R_F_L = Aᵀ R_N_L`。因此换中心 IMU 没有独立校准此前的雷达—内置 IMU 关系。

## 已排除的具体编程错误

| 检查 | 结果 |
| --- | --- |
| 配置 `R_C_N` 与 `R_C_F Aᵀ` | 最大差 0 |
| 旋转合法性 | det=1.0000000000000033，正交残差 4.8e-15 |
| launch 自写矩阵→四元数 xyzw→矩阵 | 最大差 2.0e-15 |
| 前后链逐级变换 vs 单次完整复合 | 1000 个随机点最大差 1.3e-15 m |
| 已说明名义杆臂的独立重算 | 差小于 0.5 nm，来自原向量小数舍入 |
| 行列顺序 | Faster-LIO `MatFromArray` 按九元素逐行赋值，与 YAML 一致 |
| 重复变换 | 中心 IMU 适配器不旋转矢量；dual adapter 仅转换 raw→N；LIO 再 N→C，链路不同而非重复 |

`central_imu → d1max_lidar` 静态 TF 和 LIO 的相同参数同时存在，不代表点云被多变换一次。前者发布 TF；实际点云 adapter 的 target 仍是 N，算法直接读数值并仅应用其 N→C 参数。

当前源证据：

- `src/d1max_slam/src/dual_lidar_adapter.cpp:243`：`lookupTransform(target_frame_, cloud.header.frame_id, ...)`；278 行实际变换。
- `experiments/central_imu_fasterlio_20260918/central_imu_adapter.py:52`：中心 IMU 仅缩放各轴，不做轴交换。
- 本轮 `config/mapping.launch.py:62`：旧前 IMU 输入已接到 unused topic；没有两路 IMU 同时写入前端输入。
- `src/faster_lio/include/common_lib.h:45`：逐行矩阵装载。
- `src/faster_lio/include/imu_processing.hpp:312`：去畸变中的 N→C 与逆变换；`src/faster_lio/src/laser_mapping.cc:1351`：输出到世界的同一方向。

## 未验证的物理假设

1. **历史雷达—内置 IMU 旋转**：当前隐含 `R_F_L` 与本机前雷达 SN `3009bede1530` 设备 DIFOP q 按相同物理含义解释时，相差 **0.5782528°**。代数没算错，不等于这个物理旋转正确。
2. **驱动输出基底**：单凭本节代数检查不能排除驱动预处理；并行完成的[扫描线几何核验](../beam/README.md)已强烈支持四条抽查点云没有额外 roll/pitch 或平移，扫描角指纹也支持设备与话题对应。剩余点云 yaw、IMU 消息轴处理和设备 q 方向仍需核验，不能只因地面更平就替换。
3. **中心原点**：`t_C_N=[0.01093696,-0.36597692,0.00263185] m` 是中心 IMU 假设位于厂商 base 原点的名义杆臂，不是实测。gyro fit 不能标定它。
4. **两 IMU 拟合精度**：只能说明两路角速度总体可对齐；并未把整个空间/时延/加速度偏置完整联合标定。

条件性的设备修正链必须是：

```text
R_C_N_candidate = R_C_F_gyrofit · R_F_L_DIFOP · R_N_L_currentᵀ
```

不能把设备原始 q 直接填进 N→C 参数，也不能既替换点云前置旋转又重复补偿同一个角度。

特别说明：**平移不改变静态平面的法向**。所以中心杆臂的未知可以影响运动建图，却不能单独解释开始时约 0.8° 的法向—重力不一致。这个静态差异首先涉及旋转、加速度参考偏置、地面/雷达量测误差及是否真正静止；不能单凭一条地面法向给出唯一完整外参。
