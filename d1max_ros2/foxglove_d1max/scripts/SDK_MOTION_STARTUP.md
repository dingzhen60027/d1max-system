# 唯一主线 SDK 启动与执行边界

正式入口是 Web 的“连接机器狗”，随后“启动导航服务”。Web 仅启动数据及导航
服务；RViz 设置初值、三维目标及任务。当前没有已通过的物理验收记录，Web
配置为 `planning_only`：可以定位、全局规划、局部规划和跟踪预览，不能执行运动。

SDK 和导航读取同一份 `d1max_nav_ws/deploy/single_floor_release.json`。
`config/monitor-release` 的 `@mainline` 表示跟随唯一默认发布，不保存另一条过时路径。
文件缺省时也跟随默认；显式 `D1MAX_MONITOR_RELEASE` 或该文件的绝对路径仍可选择
已封存包，启动前重新核对封存文件与真正执行的 SDK ELF。正式入口不再加载或
执行旧 `sdk_bridge_ws/install` 的 SDK。
SDK 参数也来自同一包的封存 `monitor.yaml`：优先已封存 installed-share 配置，
否则使用包内已封存的 SDK source snapshot，不读取可变工作区配置。

只读核对最终目录及完整封存（不连接 SDK、不调用 manager、不 source ROS）：

```bash
/usr/bin/python3 scripts/sdk_motion_startup.py resolve-release
```

后续具备真实验收记录后，必须同时绑定 Web 的执行用途与 SDK 的物理记录身份。
不能仅将用途改成 `execution` 或沿用旧运动票据来获得运动许可。

## 历史运动票据入口（不是正式导航入口）

在 `d1max_ros2/foxglove_d1max` 目录执行：

```bash
python3 scripts/sdk_motion_startup.py check
```

历史 `prepare/start` 与主线 schema-3 grant 互斥，现会在写票据和启动 monitor 前
明确拒绝。不要用这些入口启动当前主线；下面的票据说明仅保留历史兼容语义。

`start` 会通过已有鉴权的本机 manager 启动**同一个**
`d1max-monitor-managed.service`，复用原来的 Zenoh、SDK、Foxglove 和进程组管理。
如果 monitor 已运行、正在启动/停止、有残留进程组、本机 SDK 进程或端口冲突，
会拒绝操作。请通过现有 manager 停止，等状态成为 `stopped` 后重新执行；
脚本不会自动停止、重启、接管现有进程或另外启动 SDK。

运动能力参数固定为前后 `0.30 m/s`、横移 `0.0 m/s`、转向 `0.50 rad/s`。
启动仅开启速度传输接口；SDK gate 初始仍为 `DISARMED`。
脚本不调用起身、步态、模式切换、navigation arm 或解除急停，不发送任何速度。
随后导航流程的独立 `start-motion` 只启动执行节点，仍保持锁定；只有在 RViz
点击“开始执行”并确认后，导航执行节点才检查条件并申请 arm。
启动结果中的 `startup_disarmed` 表示启动配置和 SDK gate 的初始契约，
不是实时机器人状态，也不表示已经获得控制权、站稳、具备定位或可以行走。

其他入口：

```bash
python3 scripts/sdk_motion_startup.py status   # 只读现有manager状态
python3 scripts/sdk_motion_startup.py prepare # 只准备一次性票据，随后执行本CLI的start
python3 scripts/sdk_motion_startup.py cancel  # 只取消未消费票据，不改变运行中的monitor
```

票据放在当前用户独占的运行目录，权限 `0600`，有效期60秒，绑定本次 manager
实例、该工作目录及本次显式 `start` 的 request_id。只有经过 systemd `INVOCATION_ID`、
受管 cgroup 和 manager 的 `last_start {request_id, invocation}` 持久关联核对的
monitor 启动进程可消费。关联仅保留在本次 manager 实例内，结束临时启动任务不会清除；
停止、失败、manager 重启或新 invocation 不会复用旧关联。对已运行 monitor 的
no-op start 也不能改绑该关联。
普通管理界面的连接请求不能消费运动票据。成功或失败消费均为一次性。`start` 请求失败、超时
或不确定时清除本次未消费票据；不会自动重试。下次普通 manager 连接仍默认关闭运动能力。
单次 `prepare` 若未使用，可 `cancel`；过期票据会拒绝启用，重新准备前应取消旧票据。
`check` 与 `status` 不写票据，不访问机器人端点。

### 运行中的旧 manager 如何加载本次更新

`prepare/start` 在写票据或启动 monitor 前要求状态响应具有
`motion_start_binding: 1`。旧 manager 进程未加载更新时会明确拒绝，不会先启 monitor。
不要在现有连接运行中重启 manager：安装脚本 `install-session-manager.py` 的
monitor 和 Web 单元均 `BindsTo=d1max-session-manager.service`，重启 manager
会连带停止这些单元。

由操作者先通过现有管理界面显式停止 monitor 和 Web，确认二者均为 `stopped`、
没有待完成操作或残留进程组，再手动执行下列命令加载更新（脚本不会替你执行）：

```bash
systemctl --user restart d1max-session-manager.service
python3 scripts/sdk_motion_startup.py check
python3 scripts/sdk_motion_startup.py start
```

重载只让 manager 空闲就绪，不自动连接机器人；旧实例票据不能跨重载使用。
无需重新安装服务、改 token、改 unit 文件或更改 Zenoh 配置。

该入口只读取已安装的 manager 凭据，不创建、修改或输出 token，不改变鉴权规则。
不支持以 `D1MAX_NAVIGATION_CONTROL_ENABLED=true` 绕过显式入口。
启动脚本使用独立的 `d1max-sdk-monitor-startup.lock`；SDK core 自己取得
`d1max-sdk-monitor.lock` 会话锁。准备阶段只读探测后者，避免继承同一路径锁
导致SDK主进程重新打开锁时拒绝自己；最终SDK连接互斥仍由core负责。
原有 APP 释放控制权后的监控接管策略保持原配置；接管控制权不等于运动 arm。
原有 `start_live_monitor.sh` 的“不提供运动控制”提示描述默认监控模式；显式启动时
以 SDK monitor 的 `motion-capable, startup DISARMED` 提示及实际状态为准。

## 主线 release 的 schema-3 执行通道（默认连接，运动仍未授权）

导航 v3（BT Owner → Permit/Demand → SDK）只接受已封存 release 中的
`sdk_monitor_bridge`。默认通过 `@mainline` 解析唯一 release；需要显式覆盖时才
在 `config/monitor-release` 写一行 release 绝对路径（或设置
`D1MAX_MONITOR_RELEASE`）。解析失败或封存变化会报告原因，不回退旧 install。连接后：

- `start_sdk_monitor.sh` 额外 source release 的 `interfaces`、`sdk` 两个 install；
- 启动前重算 release 封存清单全部文件的 SHA256，任一变化或 SDK 二进制未封存都拒绝启动；
- 参数追加 `execution_v3_enabled:=true`，`navigation_control_enabled` 保持 false；
  旧运动票据与 v3 互斥（monitor 本身也会拒绝两者同时开启）；
- 物理验收绑定只来自 `D1MAX_EXECUTION_ACCEPTANCE_RECORD / _ROBOT_ID / _SDK_VERSION /
  _CALIBRATION_SHA256 / _ROBOT_PROFILE_SHA256`，必须五项齐全或全不设置。
  不设置时通道处于“未验收”，每个 activation grant 都被 monitor 拒绝，只能监视。

切换通道要停止再启动受管 monitor（经 manager），不会自动重启。

## SDK 契约与当前边界

- 官方 `Move(left_right, forward_back, yaw)` 参数顺序是 **vy、vx、wz**，
  三项为 `[-1,1]` 归一化量，不是 m/s、rad/s。速度转换由 SDK gate 负责。
- 官方 API 文档低档写前后±1 m/s、横移±0.5 m/s、yaw±1.5 rad/s；
  数据类型文档的低档表却写 vx±0.5、vy±1、yaw±2，二者矛盾。
  因此软件夹紧≤1.5 m/s与保守默认值不代表物理速度及停止距离已获实测保证。
  来源：SDK `docs/zh/sdk_client_api_cn.md:453–462`、`sdk_type_cn.md:226–230`。
- 默认仅允许已在通用运动状态、低速度档的机器人。**stairs 默认禁止**。
  SDK 的 `SetMode` 会自动站立，不能用它隐式准备楼梯模式；楼梯速度映射未有明确契约。
  来源：`docs/zh/sdk_state_cn.md:11`、`sdk_type_cn.md:280–307`。
- SDK 文档称最后一条 Move 维持1秒；没有给出硬件停止时间保证。
  软件零速和软件急停依赖有效连接及控制权。来源：`sdk_client_api_cn.md:597`。

离线验证：

```bash
python3 -m pytest -q tests/test_sdk_motion_startup.py
bash -n scripts/start_sdk_monitor.sh
```

测试使用假的 manager/系统进程、SDK exec、ROS预检查和本地端口；关联集成测试
调用真实 manager 逻辑但替换 systemd、健康探针和网络检查，不启动 monitor 或连接机器人。
