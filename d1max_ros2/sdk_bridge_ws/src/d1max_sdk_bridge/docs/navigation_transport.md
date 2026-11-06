# 导航速度出口：单 SDK 会话

## 当前主线 task_follower_v1（2026-10-12）

正式入口使用 `task_writer_enabled=true`、`execution_v3_enabled=false`。
BT 维护任务授权，SCAN 维护当前已接受轨迹的碰撞结论，跟踪器输出运动需求；
Native 将需求绑定到当前轨迹和原始状态后发布 `LocalMotionCommand`，SDK 是唯一速度发送者。

SDK 消费 `TaskExecutionLease` 与 `LocalMotionCommand`，不再要求历史
`MotionValidation`、逐曲线 CAS 或第二套反应/制动扫掠查询。
当前 SCAN 轨迹碰撞、取消、过期命令、急停和失权仍阻止非零输出。
原始状态和命令时间不续写，停止提交与实测停稳仍分别报告。
此软件整改不代表现场运动、制动或跨楼层能力已验收。

### 当前发送调度：新命令驱动、原始期限看门狗（2026-10-18）

正式 `task_follower_v1` 仍只有一个常驻 SDK writer，非零普通发送的实际软件受理
间隔至少 50 ms（最多 20 Hz），但不再让新命令等待另一套固定 20 Hz 轮询相位。
`LocalMotionCommand` 完成原有接收校验后仅递增固定大小的 wake revision 并唤醒
writer；如果上次非零受理已满 50 ms，立即选择最新原始命令，否则只等剩余间隔。
最新槽和 revision 不形成队列，迟到的旧命令仍由 Transport 的原规则拒绝。

周期健康检查不再重复发送一个未变化的非零命令，也不以空闲、HOLD 或过期零消耗
下一条新命令的普通发送机会。初始控制权事务仍能在健康检查中推进。原始命令、
机身状态及任务 lease 的截止时间独立唤醒同一个 writer；真正过期、急停、撤销和
失权仍立即选择零/停止，不等待普通非零额度。严格原始 100 ms body/command
合同不变，不刷新源时、不延长 TTL、不新增曲线或碰撞门控。

正式回调接收一个较新的有效零，即使它的正授权 lease 尚在传输，也保留既有单调
停止屏障并立即唤醒零路径。延期的非零只有其真实因果 lease 到达并重新通过完整
校验后才唤醒普通发送。writer 在状态锁内捕获实际选中 revision；厂商调用期间
到达的新命令不会被旧调用误标为已消费。非零受理间隔以实际软件受理时刻而非
tick 开始时刻计，厂商调用阻塞后仍不补发压缩的非零对。

普通命令和急停/撤销 revision 使用同一个短临界区通知辅助函数，与 writer 的
等待 mutex 配对；避免仅 atomic+notify 在“谓词已检查、尚未进入等待”之间丢失
唤醒而多等一个 watchdog 周期。该 mutex 只保护等待/通知，不覆盖状态处理、
厂商调用或遥测发布。纯并发回归强制制造这一窗口并验证旧/新通知顺序。

生产 `Transport`、原始 typed 命令和生产调度器的同输入对照包含：有效命令在旧
轮询后 1 ms 到达、原始 body 已有 60 ms 源龄时，旧轮询额外等待 50 ms 后过期，
新调度在可用额度下直接处理；20 Hz 连续命令不新增源龄、不出现该相位过期；
真正断流仍按原始截止时间归零。另覆盖突发合并、阻塞期间新 revision、旧身份、
撤销及急停。该回归不创建 ROS 图、不初始化 SDK、不连接或调用真实 Move；它是
软件调度反例验证，不是实机响应、制动或持续导航的验收结果。

### 受理顺序与单 writer 撤销（2026-10-15 实现，2026-10-18 保留）

正式 `task_follower_v1` 使用单个常驻 writer。普通非零发送仍按 steady clock
限定至少 50 ms 间隔；错过周期不补发旧指令。撤许可、失权、急停等事件唤醒
同一 writer，提前唤醒只能处理零指令，不能提高非零发送频率。输入仍为一个
最新命令槽，原始状态/命令期限不续写；10月18日起改用上节的新命令驱动相位。

一次 `TakeControl` 或 `Move` 在状态锁内完成最终核对、捕获不可变参数并登记
writer-wide transaction，这一时刻是**软件受理顺序点**；随后释放通用状态锁
调用厂商 SDK。因此厂商调用阻塞时，MC、新命令和取消仍可处理，不再一起堵塞。
`TakeControl` 返回后重新按最新状态与源时间选取速度，不能发送接管前缓存的速度。
立即返回结果只归属于捕获的 execution/epoch/arm identity，不恢复已撤销的命令。

transaction 必须等**厂商调用栈返回及回调效果完成**两者都发生才退役；
内联 ACK 不得将仍阻塞的调用冒充已结束，也不能让新任务提前获准。
已受理、正在进入厂商 SDK 的操作无法撤回；撤销后的下一次选择不会再发送旧非零。
这不是厂商物理入口、网络写入或停车的有限时保证。唯一 writer 若卡在厂商内部，
也不能及时调用下一条零速度；不能另起竞争 writer 假装解决这个限制。

纯 fake-vendor 回归覆盖阻塞期间撤销仍返回、内联/延迟 ACK、旧身份不复活、
调用栈未退役时新 grant 被拒、返回后重新检查源期限及固定容量。
测试不创建 ROS 图、不初始化 SDK、不连接实机。历史 curve-CAS/JSON 分支保留原提交方式。

既有 SDKCommandTrace 和诊断 JSON 的 `sdk_input_mapping.last_attempt_timing`
记录同一次受理尝试的 command sequence、原始 command/body/deadline 纳秒时间，
以及 source→attempt、body→attempt 和剩余原始期限（毫秒）。这些量绑定已捕获的
execution/epoch/arm identity，不会随新消息或诊断心跳变化；无选定正式命令的
停止/等待零指令明确 `present=false`，时间为 null，而不是伪造零延迟。
这是软件 attempt admission，**不是厂商写入、网络到达或机器人响应时间**。

### 跨话题先后顺序

BT 的任务 lease 与 Native 的 `LocalMotionCommand` 通过不同发布者传输，不能假定
SDK 一定先收到 lease。SDK 为**只差原始正授权 lease 到达**的有效命令保留一个
latest 槽；相同 lease 实际到达后重新执行完整接收校验，原始命令、机身状态和
截止时间保持不变。未知 lease 不提前获得权限，也不推进已接受 sequence/source
下限；已知但过期或非因果 lease 不作为“重排”放行。

新的有效零/HOLD 清除旧待处理非零和旧当前命令；显式 owner HOLD、撤销、MC/状态
断流和任务更换都清除待处理槽。异常、外来或过期零包不能覆盖有效工作。
队列容量固定为一，lease 历史仍固定最多八条。现有诊断
`command_lease_reorder` 提供 deferred/completed/discarded/pending，不增加等待门控。
这修复跨话题顺序造成的无意义零脉冲，不延长真实断流的任何运动期限。

即使零命令自己的 lease 还未到达，已校验的零源时间也建立独立的单调停止屏障：
延迟到达、源时间不晚于该零的非零命令不能借旧 lease 恢复运动。屏障不使用未知
sequence 作为授权下限，不阻止真正较新的有效命令；只有新任务重置屏障。
未来、过期、外来或格式错误的零在建立屏障前即拒绝。较新的待处理正命令保留，
仍必须等自己的真实 lease 到达后完整校验，不能被较早零误清或提前执行。

下文保留旧协议及当时的约束供查阅，不是正式 task_follower 运行图的启用条件。

## 历史 schema-3 执行出口（2026-10-04）

当时的 BT 导航实现使用 `execution_v3_enabled` 和类型化的 `/d1max/live_planning/execution/` 合同；下文 JSON/Nav2 arm 协议为更早的历史兼容入口，不能与 schema-3 同时启用。该版本启动不授权运动，未验收记录拒绝 live grant。

SDK 数据回调只向有界邮箱复制。MC/RobotState 状态处理、typed 输入/撤许可、20 Hz 唯一速度 writer、普通 ROS 服务/图发现、遥测 JSON 发布分别运行：

- typed 输入使用专属 callback group/executor，普通服务或 JSON 不排在它之前。
- writer 使用 steady clock 独立线程，错过周期直接跳过，不补发旧命令。最终检查与异步 `Move(..., 0)` 仍在同一状态锁中串行提交，回调只写原子 veto/ACK 邮箱，不在回调中反锁。
- 状态处理不做 JSON 或 ROS publish。MC/RobotState/SDK 执行状态各用一个 latest 遥测槽，保留原始源时；CAS 回执单独固定 8 槽，满槽锁存否决，不无界排队。
- writer 不直接 publish；JSON 构造、网络发布不持控制状态锁。诊断提供 `writer_last_duration_s`、`writer_deadlines_skipped` 和 `writer_overruns`，这些是实测诊断，不是实时性验收结果。

物理验收 JSON 仍为 schema 3，新增两个必需顶层合同：

```json
{
  "execution_timing": {
    "sensor_source_age_bound_s": 0.5,
    "command_pipeline_bound_s": 0.1,
    "writer_period_s": 0.05,
    "source_time_uncertainty_s": 0.02
  },
  "stationary_evidence": {
    "profile": "general_low_speed",
    "linear_threshold_mps": 0.03,
    "angular_threshold_radps": 0.05,
    "stationary_duration_s": 1.0,
    "reentry_duration_s": 0.6,
    "minimum_new_samples": 3,
    "measured_static_linear_bound_mps": 0.01,
    "measured_static_angular_bound_radps": 0.02,
    "mc_expected_hz": 50.0,
    "mc_min_hz": 40.0,
    "mc_max_hz": 60.0
  }
}
```

以上数字只是**隔离模拟结构示例，不是机器狗实测参数**。`measurements.reaction_bound_s` 必须覆盖 `execution_timing` 四项之和；示例需要至少 0.67 秒。运行的感知源龄、命令管线和 writer 周期不能超过记录预算；命令管线预算至少 0.1 秒，覆盖现有 100 ms 原始运动/proof 源龄。最终 writer 的前后雷达、曲线和运动证明直接使用记录的 `sensor_source_age_bound_s`，不会把后续检查时间当作新观测时间或用管线预算续租。MC 的 `mc_delay_bound_s` 与近似源时基准仍独立绑定停稳证据，不能把它冒充雷达时钟误差，也不能改成 PTP。

停稳阈值必须覆盖实测 MC 静止噪声上界，且不得超过 0.05 m/s / 0.1 rad/s；终止持续至少一秒、静止重接至少 0.6 秒、至少三个新的有效样本。频率范围来自同一验收记录；`SetMcConfig` 仍只有开/关，没有频率设置。适用状态仍是普通模式、正常站立/运动状态、低速档、机头前向，不能把平地记录用于楼梯或趴下。原始 MC 时间、会话/代次、零发送 ACK 和保守采集区间仍逐项核验；零指令或 ACK 不等于已停稳。

旧 live 记录缺少上述任一合同时 fail-closed，不补默认值、不置真原有物理验收标志。`execution_acceptance_check` 输出对应合同，隔离 mock 也加载同一个模型结构，但 `fixture_only` 仍不能用于 live。

**线程隔离不是独立硬件看门狗。** 它解决普通 executor/JSON 排队导致的租约延迟，但厂商 `Move` 调用若阻塞、SDK 内部锁/网络停滞、进程被杀、主机掉电或 OS 调度失效时，软件无法保证 250 ms 内物理停车。已经提交的命令也不能撤回。析构先撤许可并 join 唯一 writer，再进行现有有界零提交事务；不会另起一个竞争 writer 或 detach 一个仍可能提交的线程。若厂商调用本身卡死，join/SDK Disconnect 的有限时间不能证明。独立停车、断网残留指令、制动距离/延迟仍须实际硬件与现场验收；本轮没有启动 SDK、部署或改变任何未验收标志。

纯测试增加 deadline 跳周期、遥测消费者阻塞下 latest 有界覆盖、验收字段缺失/反应预算不足/噪声超阈值/错误模式、记录化停稳阈值和真实超阈值运动撤销。它们不连接 SDK、不启动 ROS 图，不替代现场制动和压力验收。

## 历史兼容入口

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
