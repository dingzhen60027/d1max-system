# 安全扫描输入合同与执行前整改

## 当前生产实现，不等于预览地图实现

实机运动栈的安全链路是：

`双雷达适配 → Faster-LIO deskewed → navigation_cloud_to_scan → safety_scan → collision_monitor → command_gate`

逐束预览链路是另一条分支：

`双雷达独立回调 → rays_raw（源/时间/原点元数据）→ acquisition-time projector → rays_map → native GridMap`

第二条没有接入第一条安全链路。当前运动入口禁止逐束预览、official 未知空间策略及实验性回波排除用于实机执行；新增的架构检查也不能由 YAML 验收布尔值解除。

## 已核实的缺口

1. **安全点云已被 LIO 筛选。** 定位配置的适配器 `min_range=0.5`、LIO `blind=0.5`、`point_filter_num=4` 位于安全投影上游。安全扫描再写 `range_min=0.15` 不能恢复被丢弃的近身回波。适配器的逐束元数据分支目前同样继承 `min_range`，简单换话题也不能补齐。
2. **融合点云新鲜不等于两雷达都新鲜。** `front_fallback=true` 允许后雷达失流时继续向 LIO 供应前雷达；`deskewed` 没有安全消费者所需的逐源序列、采集始末时间、处理龄和原点证明。
3. **端点投影不是空间覆盖证明。** 未命中角度写 NaN，含义是未知，不是无障碍。一个甚至全部角度有有限端点都不能证明整个机器人扫掠体积已观测。现有 command gate 会拒绝全 NaN，但其“至少一个有限点”仅是扫描消息可用性检查，不是 360° 近身覆盖验收。
4. **时钟只做了软件 epoch 映射。** `estimate_shared_epoch` 保持一个固定偏移，不是硬件时间同步已验收。不能把接收时间改写成源采集时间；`time_alignment_verified=false` 和外参/安全包络未验收标志保持不变。

## 本轮已完成的软件修复

`navigation_cloud_to_scan` 的健康报告原来只看收到消息的时间，全 NaN 扫描仍会报告 `fresh=true`。现在分开报告：

- `source_fresh`：原始消息时间和本机单调接收时间都在源龄预算内。
- `valid` / 兼容字段 `fresh`：源新鲜、投影成功且有有效端点；不代表可运动。
- `unknown_bins`：没有端点证据的角度数。
- `coverage_verified=false`：当前实现不会宣称安全覆盖已验证。

未改点云过滤、障碍判定、未知空间策略、TF 或速度输出。TF 异常/源超时不会因此前一个有效扫描而继续报告健康。

`safety_input_contract.inspect_safety_input()` 检查实际运动参数中投影、碰撞监视器、命令门的输入输出、body/odom 帧和源龄预算。`safety_input_blockers()` 是实现缺项，不接受 `raw_safety_scan=true` 之类的配置冒充代码已经完成。可调用 `motion_stack.describe_parameters()` 离线审计被禁止的运动拓扑；这不是启动授权。

## 下一步安全原始支路的接口要求

必须独立于 SLAM 的抽点/特征筛选，不能为了安全扫描改变定位输入。新支路应显式提供：

1. 每个物理雷达独立的 sensor ID、原始采集始末时间、逐点采集偏移、单调源序列、设备帧与原点；记录固定时钟映射的 epoch/version，并保留原始设备时间。
2. 两源分别统计最后接收、完成处理及源龄。前雷达持续更新不能续租后雷达；重复包、逆序时间、时钟重建、上下文切换必须撤销旧证据资格。
3. 按采集时刻在**连续局部 odom** 中补偿运动，保持同一 body 外参版本。整次处理前后核对时间和上下文，处理完成太晚不得用发布时刻给旧扫描续租。
4. 区分真实障碍端点、有效穿越射线、无返回/未观测；无效回波和 NaN 不能生成最大量程清空射线。双雷达覆盖需落到当前速度/制动距离对应的扫掠体积，不由“360° 雷达”或有限 bin 数替代。
5. 自体排除必须绑定实际安装外参、机身/轮腿几何和姿态来源；完整关节状态缺失时不得通过大范围近身清空伪造覆盖。实测雷达近场能力和遮挡范围缺项继续显式保留。
6. 最终消费者必须同时核对扫描和独立源健康凭证。仅换 `input_topic`、只增加健康 UI、复用 LIO 一条扫描的 header，都不能解除架构 blocker。

## 离线验证

在 `/home/dndx/d1max_nav_ws`：

```bash
PYTHONPATH=src/d1max_navigation:src/d1max_pct_scan python3 -m pytest -q src/d1max_navigation/test/test_cloud_to_scan.py src/d1max_navigation/test/test_command_gate.py src/d1max_pct_scan/test/test_safety_input_contract.py src/d1max_pct_scan/test/test_robot_profile.py
```

本轮结果：204 passed。正例包括近地真实障碍端点保留、正确刚体变换、有效扫描健康；反例包括全 NaN、源时间/接收时间过期或在未来、TF 失败、消费者帧/话题漂移、把话题换为 raw 或伪造验收布尔值。测试也直接执行生产 command gate 的扫描回调（不创建 ROS 节点），确认全 NaN、全无穷和过期扫描仍被拒绝。它们是离线软件回归，不是实机安全验收。
