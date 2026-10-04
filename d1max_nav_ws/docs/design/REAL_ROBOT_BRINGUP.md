# 实机上线步骤

适用于任意已封存的导航 release（下文 `$D1MAX_RELEASE`）。每一阶段通过后再进入下一阶段。
前一阶段未通过时，后面的步骤不会放行运动：monitor 的 v3 通道和 activation 都会拒绝。

```bash
export D1MAX_RELEASE=<已封存 release 的绝对路径>
python3 tools/diagnostics/robot_preflight.py            # 全部只读，rc=0 才算就绪
python3 tools/diagnostics/robot_preflight.py --offline  # 不连狗时只查软件侧
```

预检不启动、停止或重启任何服务，不连接 SDK，不发送命令。每个 FAIL 都会给出由谁处理。

## 0. 连接前（软件侧）

| 项 | 要求 | 处理 |
|---|---|---|
| release | 封存清单全部哈希不变，`activation=false` | 任何改动都要重新封存 |
| Web | 进程启动时间晚于当前入口脚本 | 经 manager 重启 Web（需批准） |
| monitor | 受管 monitor 运行的是 `$D1MAX_RELEASE` 里的 `sdk_monitor_bridge` | 在 `foxglove_d1max/config/monitor-release` 写一行 release 路径，经 manager 停止再启动（需批准） |

monitor 以 release 模式启动时，所有未设置验收绑定的 activation grant 都会被拒绝。
这一阶段只能监视，不能运动。详见 `foxglove_d1max/scripts/SDK_MOTION_STARTUP.md`。

## 1. 连接（只看不动）

1. 机器人上电，站立由 APP 完成；APP 保持在手边，急停随时可用。
2. 网线接到 `manager.json` 的 `robot_interface`，启用 NetworkManager 的 `D1max` 连接。
3. 网线没插时，代理 TUN（如 `Meta`）会接管 `192.168.168.0/24`，TCP 看起来能连上，其实是代理在应答。
   插上网线后，main 表里的 /24 路由优先级更高，流量会走机器人网卡。
   预检的 `route_*` 必须显示经过机器人网卡；如果还是走代理，关闭代理或给该网段加直连规则。
4. 重新运行预检，确认 `robot_link`、`route_*`、`tcp_*` 全部为 OK。
5. 在 Web 定位页启动所选地图的实机定位。检查以下几项：
   - `/d1max/localization/status` 中 `localized=true`；
   - 点云与墙体重合，静止时漂移有界；
   - 前后雷达、MC 数据都新鲜（不是回放）。
6. 这一阶段不发送目标，也不解锁。

## 2. 物理验收（有人值守、急停在手）

v3 在没有验收记录时不会运动，所以测量要走 APP 手动遥控或厂商工具。
不能先把 `*_verified` 改成 true 再去测。每项要保存原始数据文件，记录绝对路径和 SHA256。

| kind | 测什么 | 通过条件 |
|---|---|---|
| geometry | 实际外包络、原地转动扫掠；机身参考原点到地面的高度 | 与 robot profile 中的机身高度、外包络一致 |
| speed | 低速档下 Move 归一化量与实测 vx、wz 的比例、方向和饱和 | 与 SDK 文档一致，或按实测修正 |
| braking | 零速生效后的最大平移、最大转角和总停止时延（取上界，不取均值） | 不超过记录中的保守值 |
| mc_time | OnMcData 时间连续性、端到端延迟上界、静止速度噪声 | 延迟 ≤ 0.25 s |
| raw_ray | 前后雷达的真实射线原点、近身可见性、各自断流时的行为 | 自体、未观测、观测不足三者能区分 |

五份文件齐全后，由操作者本人填写 `robot_id`（从 APP 的设备信息读取）、`calibration_sha256`、`evidence_id`，
以及实测得到的 measurements。只有实测确实满足条件时，才由操作者把对应的 `*_verified` 置为 true。然后用唯一的检查器核对：

```bash
"$D1MAX_RELEASE/sdk/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/execution_acceptance_check" \
  <记录绝对路径> <robot_id> <实际 SDK 版本> <标定 SHA256> <robot profile SHA256>
```

返回 0 只说明记录的结构、身份和文件哈希都正确，不会自动授权运动。

## 3. 低速自主

1. 设置五个 `D1MAX_EXECUTION_*` 验收绑定，经 manager 重启 monitor，让 v3 通道绑定这份记录。
2. 创建 activation，绑定 release、验收记录和当前 SDK 会话。monitor 每次重启会话号都会变，所以 activation 要在最后一步创建。
3. 先在空旷区域测试：短直线、原地转向、取消、急停、APP 抢占、拔掉单个雷达。
   每一项都要确认停止证明和 `shutdown.json`。
4. 以上都通过后，再测障碍、窄通道和长路径。
