# SDK 已连接但 OnMcData 为 0 Hz：主从会话权限

记录日期：2026-09-12（本机时间，Asia/Shanghai）。
状态：本次故障已通过操作员授权的通信重连恢复，实收约 50 Hz；不是固件权限机制已被修改。

后续修订：用户随后明确要求 APP 用完退出后由 SDK 接管。现已增加受配置控制的
`OnControlAvailable → TakeControl → ACK + 两帧归属确认 → 重新开启 MC` 流程，
实现与离线验证见 [当前 SDK 说明](../../README.md)。该自动接管尚未做实机往返验收。
下文保留当时故障与人工重连证据；“不自动提升”的旧限制已由上述明确授权的有限流程替代，
并不允许抢占活动 APP、反复重连、运动或解除急停。

## 结论

这次 SDK 网络连接成功、普通 RobotState 正常，但机器人把该 SDK 连接登记为从会话。
所检查的机器人端 `robot_remote` 在处理 MC 上报配置前检查主会话身份，从会话请求被拦截。
因此 `SetMcConfig(true)` 虽然发送成功，却没有配置回执，也没有 `OnMcData` 数据。

APP 退出后，服务端释放主会话并通知 SDK“控制权可用”，但不会自动将这个旧 SDK 从会话提升为主会话。
确认 APP 已释放后，经用户明确授权重连，新的 SDK 连接被登记为主会话；MC 一次开启成功。

这是已检查的机器人服务端的具体限制，不应推断所有 SDK 版本都要求读取速度必须取得控制权。
不能把“SDK 在线”“MC 配置成功”“实际高频数据到达”当成同一个状态。

## 故障现象与证据

- 普通状态持续到达：`sdk_fresh=true`、`connection_state=4`（CONNECTED）。
- MC：`samples=0`、`observed_hz=0`、`acknowledged=false`、`mc_ready=false`。
- MC 写入无错误，时间戳拒绝计数为 0；界面处于等待 MC / 自动重试。
- 多次重连发生在 APP 主会话仍存在时，新连接仍是从会话，因此没有恢复。
- APP 关闭后旧 SDK 连接仍保留，从会话上的配置重试也没有恢复。

以下为机器人日志摘录，时间是机器人自身的 2026-02-24，不能直接与本机时间混排：

```text
03:00:21.203 Master session already exists, recv new handshake from client 192.168.168.10:36540, as slave
03:15:30.187 Client TYPE_UDP 192.168.144.11:8082 disconnected
03:15:30.194 Notified session 192.168.168.10:36540 of master release
03:18:07.402 Client TYPE_WEBSOCKET 192.168.168.10:36540 disconnected
03:18:11.727 No existing master session, set client 192.168.168.10:58032 as master
03:18:12.468 Received SensorConfig from client 192.168.168.10:58032
```

来源：机器人上的 `/root/.ros/log/robot_remote_2269_1771865610129.log`。
PID、日志名及客户端临时端口仅标识本次事件，后续不能硬编码为当前进程。

本机有界网络跟踪确认厂商 `SetMcConfig(true)` 发出了：

```json
{"type":1008,"sensor":30,"enable":true,"freq":0}
```

以上是提取的关键字段，不是完整协议帧。`freq:0` 是厂商布尔开关 API 的实际编码，不能解释为请求 0 Hz。
原始跟踪临时存放于 `/tmp/d1max-mc-wire-Yo0xYs/sdk-network.trace`，该临时文件不保证长期存在。
诊断用 strace 包装已撤回，不属于正常启动链路。

服务端只读反汇编核对：

- `ClientChannel::Step1` 中 type 1008（0x3f0）进入默认 `IsMaster` 检查。
- 非主会话返回 false；`ProcessMessage` 因此不执行处理传感器配置的 `Step2`。
- 分派地址：0x7d214–0x7d258；主会话检查：0x7db90–0x7dbcc；调用门控：0x7cc88–0x7ccbc。
- 服务端：`/opt/robot/install/robot_remote/lib/robot_remote/robot_remote`。
- SHA-256：`bd7c490b40bfc04c6f9ed84614f10230c718dbbaf7770f2dd2f0f5dd30ef45c3`。

没有修改机器人程序、服务、固件或权限检查。

## 本次恢复操作与验收

用户确认 APP 已关闭；只读日志检查确认主会话释放。
用户明确回复“重联”后，仅重启本机受管通信服务：

```bash
systemctl --user restart d1max-monitor-managed.service
```

这个服务会连同其所属的 Zenoh 中转、SDK 接收器、PCD 发布器、Foxglove 桥和健康探针一起重启，
并非只重新打开 Foxglove 页面。旧进程组被清理后才建立新连接，最终仅有一个 SDK 连接。
没有重启机器人端服务，也没有重启会联动关闭 Web 的 session manager。

本机 2026-09-12 14:28:33–14:28:42 的实际验收快照：

| 项目 | 结果 |
| --- | --- |
| 来源 | `sdk_mc` / `IDataCallback::OnMcData` |
| 状态 | `streaming` |
| 实收频率 | 50.005–50.016 Hz |
| 样本累计 | 650 → 1,100 帧 |
| 配置回执 | `acknowledged=true`、`ack_on=true` |
| 本连接配置请求数 | `total_attempts=1` |
| 重试轮数 | `retry_cycles=0` |
| 时间戳拒绝数 | `timestamp_rejections=0` |
| 新鲜度 / 就绪 | `mc_fresh=true`、`mc_ready=true` |

这是有限时间的实机读取验证，不是长期稳定性、速度精度或快速运动定位的验收。
定位服务在本次重连时保持未启动。没有发送运动、姿态、抢占控制权或解除急停命令。
但必须注意：没有调用 `TakeControl`，不代表连接不可能获得主会话身份；本次新连接由服务端自动分配为主会话。

## 后续遇到相同现象的处理步骤

1. 先区分网络连接与 MC 数据状态，检查实际样本、配置 ACK、频率和新鲜度。不能只看 PID、端口或电量。
2. 确认只有一个 SDK 接收器。查机器人当前会话日志，确认 APP 是否实际断开、主会话是否释放，而非仅凭手机页面关闭。
3. 如果主会话仍属于 APP，不反复重启、不擅自 `TakeControl`，也不替用户断开 APP。
4. 如果 APP 已释放但 SDK 仍是旧从会话，先确认机器人静止、现场独立急停可用、定位已停止，并取得操作员对重连可能获得主会话身份的明确授权。
5. 只重连受管通信服务一次，检查新的主/从身份与 MC ACK、连续样本和实收频率。失败时继续区分会话权限、配置应答与数据处理问题，不循环重启整个系统。
6. 数据就绪后，是否恢复定位由当前任务单独决定；不能自动恢复机器人运动。

本机只读检查命令（UID 1000 的当前部署）：

```bash
jq '{sdk_fresh,mc_fresh,mc_ready,mc_observed_hz,speed_report}' \
  /run/user/1000/d1max-session/monitor-health.json
ss -ntp dst 192.168.168.168:8081
```

健康文件属于运行时快照，需确认监控服务正在运行且文件持续更新；陈旧文件不构成在线证据。
其他用户部署应使用对应 UID 的路径。

如果业务必须同时保留 APP 控制和 SDK 高频读取，需要厂家确认当前固件的只读 MC 通道或支持方案。
本次没有验证 APP 再连接或抢占后的 MC 行为，不能承诺数据不会再次受影响。

## 不应采用的“修复”

- 不因 MC 缺失就调整机器人时间或 chrony。本次确实存在两端时钟日期差，但未改时钟便恢复了 MC，日期差不是这次未开启上报的必要解释。多传感器同步仍需单独验证。
- 不把 `OnSpeedData` 或 1 Hz RobotState 速度作为 MC 缺失时的隐式替代；不补帧、不复用旧速度、不伪报 50 Hz。
- 不在 `OnMcData` 中做 JSON、ROS 发布、文件 I/O 或复杂计算。当前回调只复制数据、入有界队列，工作线程再校验和发布。
- 不因旧 PDF 写了 100 Hz 就将目标或显示值硬改为 100 Hz。所用 0.1.0 SDK 随包文档写 50 Hz，本次实际约 50 Hz。
- 不自动提升权限，不修改厂商头文件虚函数布局，不绕过机器人端权限检查。

## 相关实现与厂家文档

- [监控桥实现](../../src/sdk_monitor_bridge.cpp)
- [MC 状态与恢复逻辑](../../src/mc_report_core.hpp)
- [监控配置](../../config/monitor.yaml)
- 厂家包：`sdk/中狗highlevel-sdk-v0.1.0-Chargingv2/ubuntu22.04/RobotSDK-0.1.0-charging_v2-x86_64`（相对资料包根目录）。
- 随包 `docs/zh/sdk_control_ownership_cn.md`：APP 可以抢 SDK 控制权，SDK 不能抢 APP；初始连接顺序影响归属。
- 随包 `docs/zh/sdk_client_api_cn.md` 的 `SetMcConfig`：上报默认关闭，开启后 50 Hz；第二参数是超时毫秒数，不是频率。
- 随包 `docs/zh/sdk_callback_cn.md`：`IDataCallback`、`OnMcData` 及轻量回调要求。

本记录只更新文档，没有新增自动切换主会话的程序逻辑。
