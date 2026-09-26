# Nav2 速度出口：单 SDK 会话，可选启用

默认仍为监控 + MC 速度读取 + 单向软件急停。新增导航出口复用 `sdk_monitor_bridge` 内已经存在的 `SDKClient`，不再连接第二个 SDK 客户端，不运行旧的 `sdk_state_bridge` 或 `sdk_console_bridge`。当前实机断开；代码、纯状态机和编译检查不代表实机导航验收。

## 三个独立条件

1. SDK 监控进程启动参数 `navigation_control_enabled=true`：只开放导航专用入口，启动仍锁定。托管启动必须通过 `foxglove_d1max/scripts/sdk_motion_startup.py start` 的短时、单次、绑定启动请求的票据；旧的 `D1MAX_NAVIGATION_CONTROL_ENABLED=true` 环境变量不能绕过票据。
2. Nav2 `navigation_command_gate` 的 `motion_enabled=true`，并固定本次定位会话 ID、地图版本、坐标系；定位必须实际回报 `navigation_ready=true`，外参与时间验收标记不能绕过。
3. 操作者调用命令门的 `~/arm`（`std_srvs/SetBool`），命令门检查数据和 SDK 状态后异步申请本 SDK 会话的授权。最终以命令门状态和 SDK `navigation_armed` 为准，受理不等于已解除锁定。

任一层都不会自动站起、切换运动模式、改变速度档、解除急停。APP → SDK 的控制权交接仍完全遵守原有策略；导航模块不新增抢占或重试 TakeControl。

## 实际速度单位

Nav2 和专用 JSON 协议使用 SI：`x/y` 为 m/s，`yaw` 为 rad/s。官方 `sdk_client_api_cn.md` 的 `Move` 接口使用百分比，并非 SI。仅允许通用运动状态、低速档、机头前向时发送：

| SDK 参数 | 换算 | 官方低速满量程 |
|---|---|---|
| `forward_back` | `x / 1.0` | 1.0 m/s |
| `left_right` | `y / 0.5` | 0.5 m/s |
| `yaw` | `yaw / 1.5` | 1.5 rad/s |

调用顺序为 `Move(left_right, forward_back, yaw, 0)`，异步发送。当前默认上限为 0.30 m/s、横移 0、0.50 rad/s。

### 1.5 m/s 硬上限与实测速度保护

导航指令的**平面合速度** `hypot(x,y)` 不能超过 **1.5 m/s**，此上限是 SDK 出口代码常量，不是可调参数；命令接收和最终 `Move` 比例换算前各检查一次。非有限值或超限命令拒绝并取消授权，不通过静默截断掩盖输入问题。

本版仍只允许官方**低速档**，所以可配置前向最大值仅可到 1.0 m/s、横向到 0.5 m/s，默认仍为 0.30/0。**“不超过 1.5”不表示本版把机器人设置为 1.5 m/s**。不会自动改中/高速档，也不会按低速比例给中/高速机器人发送运动指令。

另用 `OnMcData` 的 `v_body[0:2]` 做实测合速度保护。只有通过原有 MC 源时间、接收新鲜度和有限值校验的数据才参与；导航授权期间出现一帧实测合速度大于 1.5 m/s，即锁存 `measured_planar_overspeed_latched`、取消授权，并在仍具备安全发送条件时发送最多三帧零速。恢复到阈值以下不会自动清除锁存，也不会重新行走。无 MC 数据时不拿预测速度或 `OnSpeedData` 代替。

**这是软件指令限幅和反馈停机保护，不是机械/硬件限速器。** 实际瞬时速度、停止延迟、惯性和打滑仍须实机验收；这里没有用放宽阈值的去抖延迟声称绝对保证。故障需人工核对并停止导航、重启监控会话后显式重新授权，重启不会解除机器人急停。

## 会话协议

从 `/d1max/monitor/status` 发现 `session` 和 `service_prefix`，校验时间、模式以及 `motion_control_enabled`。只接受由十六进制会话构成的 `/d1max/monitor/s_<session>` 路径。

- `<prefix>/navigation_arm`：SetBool。成功回执含 `sdk_session`、`arm_generation`。已授权时拒绝再次授权，先取消再重新申请。
- `<prefix>/navigation_velocity`：String JSON。字段为 `sdk_session`、`arm_generation`、递增 `seq`、`command_source`（命令门进程随机标识）、`navigation_session`、`map_version_id`、`stamp`（Unix 秒）、`x/y/yaw`。
- 首条命令固定发布者标识、定位会话和地图版本。旧代次、乱序、过期、未来时间、非有限值、超限或换发布者/地图/定位会话，均不能继续运动。
- 消息源、地图和会话固定用于防止误接和旧消息复活，不替代 ROS 网络访问控制。
- `/d1max/monitor/status` 同时提供 `navigation_limits`（`hard_planar_mps`、`forward_mps`、`lateral_mps`、`yaw_radps`、`required_speed_level`）、`navigation_fault_latched`、`navigation_overspeed_latched`、`navigation_measured_planar_mps`、`navigation_block_reason` 和 `navigation_error`。上游可检查配置一致性和具体拒绝原因；这些附加字段不改变原有授权协议。

## 停止与失效

SDK 出口运行 20 Hz 的独立定时检查。必须同时满足：SDK 已连接、未检测回放、控制权已实际确认且状态新鲜、双急停均明确解除、通用模式/运动状态、低速档、机头前向、MC 新鲜且频率正常、没有锁存故障。

授权后首条命令最多等待 0.6 秒；随后命令时效 0.25 秒。**接收新命令时先检查上一条命令的期限，再决定能否接受**，与定时检查使用同一规则。即使定时器延迟，断流 300 ms 后抢先到达的新消息也不能刷新旧授权；迟到的首条命令同样不能延长 0.6 秒窗口。命令门在 Nav2 空闲时发送零速心跳，不保留上一次非零速度。断流或健康条件失效会取消授权，不自动恢复；如仍能确认本进程拥有控制权且可安全发送，最多发送 3 帧零速。失去控制权后不再向别人的会话发移动指令。

导航启用时请求软件急停会立即锁存导航故障；授权期间观察到软/硬急停触发也会锁存。因此 APP 上释放急停不能让旧导航命令复活。SDK 数据回调仅复制/校验小数据；MC 处理在独立工作线程，异步 SDK 运动发送在 ROS 20 Hz 检查任务，不在 `IDataCallback` 内发网络请求。

### 并发与发送边界

最终健康检查、授权检查和 `Move(..., 0)` 非阻塞提交与工作线程的故障/MC 超速撤销使用**同一把状态锁**，不会再把非零命令取出、解锁，再调用 SDK。发送前再次消费回调否决事件，发送返回后也消费一次，以兼容 SDK 内联执行回调的情况。

SDK 回调不获取这把锁；控制权丢失、严重故障、急停、发送失败等只置位固定大小的原子事件位，诊断详情进入独立邮箱。否决位不会被后续健康数据覆盖，也不依赖诊断队列是否满。避免了持锁调用 SDK 时同步回调反过来等状态锁的死锁。

边界是**提交顺序**，不是网络队列撤回能力：已经提交给 SDK 的一条命令不能撤回；提交期间新到达的撤销事件在返回后生效，禁止下一条非零提交。当前 SDK 没有提供撤销已提交 `Move` 的接口。不得把此机制表述为“收到故障的同一物理时刻已刹停”。

**0.25 秒是 SDK 桥仍运行时的软件超时，不是硬件停止保证。** 进程被 SIGKILL、主机掉电或链路断开时，官方说明最后一条 `Move` 可维持 1 秒。实机测试必须保留人工/机身急停保障。

## 通过托管启动时的可选启用

Web 普通连接仍为监控模式。运动能力显式启动方法以 `foxglove_d1max/scripts/SDK_MOTION_STARTUP.md` 为准：停止旧监控后，由专用启动工具请求现有管理器启动同一个 `d1max-monitor-managed.service`。工具不会自动停止、替换或叠加正在运行的 SDK 会话；不会通过环境变量开启运动。正常启动与运动启动都保持 DISARMED，后者也仍须通过导航执行器/RViz 的显式操作和各级健康检查。

升级前已运行的旧管理器需要按该说明在服务停止后重载；本次修复没有重启管理器或监控，也没有连接机器人。

## 验证边界

`test_navigation_motion` 不链接或连接 SDK，覆盖默认关闭、显式授权、SI 换算、代次/上下文/序号/时间、超限、过期、断连、急停、丢失控制权、MC 断流、最终 SDK 边界硬上限、低速档物理范围、斜向实测超速、过期测量、故障锁存、有限停止和禁止自动恢复。实际移动、刹停距离、速度响应和机器人状态转换仍需实机验收。

`test_navigation_dispatch` 使用相同的串行提交辅助函数与虚拟发送器，确定性覆盖：发送前控制权/故障否决、工作线程撤销与提交的先后顺序、MC 超速、发送过程中内联回调、SDK 等待另一线程回调、多个否决位并发到达、选取后提交前的回调，以及不能自动重获授权。纯 C++ 测试不链接 SDK、不创建 ROS 节点、不连接实机。

本次依据的官方资料位于 SDK 包 `RobotSDK-0.1.0-charging_v2-x86_64/docs/zh/`：`sdk_client_api_cn.md`（`SetSpeed`、`Move`、`SoftEmergencyStop`）、`sdk_control_ownership_cn.md`、`sdk_callback_cn.md`，并对照 `include/robot_sdk/sdk_client.hpp` / `sdk_type.hpp` 的接口和枚举。没有改 SDK 库，也没有新增自动姿态/档位/控制权接管流程。
