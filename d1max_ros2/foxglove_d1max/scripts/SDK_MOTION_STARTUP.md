# 显式启动运动能力，启动后仍未解锁

在 `d1max_ros2/foxglove_d1max` 目录执行：

```bash
python3 scripts/sdk_motion_startup.py check
python3 scripts/sdk_motion_startup.py start
```

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
