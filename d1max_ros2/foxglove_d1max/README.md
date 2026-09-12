# D1 Max · 轻量状态与感知工作台

## APP 退出后 SDK 接管（2026-09-12，用户授权，离线验证）

受管监控配置已启用 `auto_take_control_on_available: true`。收到厂商 `OnControlAvailable` 后
异步申请一次 `TakeControl`；接管回执成功且连续两帧状态确认 SDK 归属，才重新配置 MC。
数据就绪仍以实际 `OnMcData` / ACK / 实收频率为准。APP 再次接管时撤销本机归属，等待下一次释放。
不抢正在控制的 APP，不新增运动 / 姿态 / 模式 / 解除急停，不改变布局。
接管超时状态未知时禁止自动重试，核对后手动重连。详细规则和配置见
[SDK 监控桥说明](../sdk_bridge_ws/src/d1max_sdk_bridge/README.md#authorized-app---sdk-handoff-2026-09-12-offline-validation)。
本次未启动实机通信或定位；APP 往返切换仍待实机验收。

## 局部里程计缺口恢复（2026-09-12，离线验收完成）

旧 LIO 一次 IMU 超限后永久锁定，导致初值无法提交。现保留 50 ms 上限，仅对 IMU 覆盖缺口做有界重新初始化；
旧地图关系和初值失效，完成后必须重新画初值，不自动恢复全局定位。时钟异常、位姿跳变和连续恢复超限仍停止。
Web 显示具体原因和恢复状态，头尾模式检查保留。详见 [实现说明](../../../d1max_nav_ws/src/d1max_localization/ROBUST_LOCALIZATION.md)
和 [验收及收尾记录](../../../d1max_nav_ws/src/d1max_localization/VERIFICATION.md)。
用户已主动断开实机，当前通信 / 定位均已停止，Web 保留；本次不是实机精度或长期稳定性验收。

## MC 启动恢复修订（2026-09-12，已完成本次实机读取验证）

- 故障记录：[SDK 已连接但 MC 为 0 Hz：主从会话权限、证据与恢复步骤](../sdk_bridge_ws/src/d1max_sdk_bridge/docs/incidents/2026-09-12-mc-session-permission.md)。APP 退出不会自动提升旧 SDK 从会话；确认释放后，经用户授权重连，MC 一次启用成功，实收约 50 Hz，已验收 1,100 帧。
- 修复 3 次上报配置超时后永久不再请求的问题：每轮最多 3 次，轮间退避 15／30／60 秒；只发送 `SetMcConfig(true)`，不通过断开 SDK 或抢控制权来自动重试。配置重试不能绕过从会话权限限制。
- 实际 MC 持续到达时停止重发，即使配置回执丢失；稳定 2 秒后恢复重试预算，后续断流可重新请求开启上报。数据过期不补帧、不沿用旧速度。
- 首次连接改为异步，初次离线不阻塞 ROS；仅在 SDK 明确 `DISCONNECTED` 时补发连接请求，不与厂商自动重连并行。可执行文件加进程锁，防止同机重复实例。
- 健康探针在启动定位之前就读取 MC 状态。普通 RobotState 在线不再等同于数据全部就绪；MC 实收、配置回执、频率、重试倒计时分别保留。
- 配置：`../sdk_bridge_ws/src/d1max_sdk_bridge/config/monitor.yaml`。回归覆盖迟到回调、断流、漏 ACK、回放隔离、重复启动和急停不自动重试。

加载部署必须先确认机器人静止、独立急停可用。管理器重启会因 `BindsTo` 联动停止通信和 Web，不能在实机运行时当作普通页面刷新。
本次只验证了有限时间的 MC 数据读取，未验证快速运动定位、长期稳定性或 APP 再次抢占后的数据行为。MC 重连时定位保持停止；后续 LIO 的 IMU 缺口恢复修订见本文顶部，未放宽缺口阈值。
不调用 `TakeControl` 不等于永远不会获得控制权：没有主会话时，新连接可能由机器人直接分配为主会话，操作前必须告知并取得授权。

## 速度来源约定（2026-09-11）

当前唯一速度源为 `IDataCallback::OnMcData` 的 `v_body` / `omega_body`。
`SetMcConfig(true, 0)` 只开启上报，0 是异步超时参数，不是 Hz；不再请求
`OnSpeedData`，不从 1 Hz RobotState 降级取速度。SDK 回调只复制数据，有界队列的
独立工作线程负责时间戳校验与 ROS 发布。配置在 SDK bridge 的 `config/monitor.yaml`。
速度图继续读 `/d1max_sdk_bridge/velocity`；`source=sdk_mc`，布局无需更换。
状态话题沿用 `speed_report_status` 以兼容订阅，实收频率、ACK、样本数分别记录。
MC 原始 ns 时间戳保留；ROS 时间用首次本机接收时刻加源时间增量，明确为近似对齐，
不等于硬件同步。断流不补帧。停止定位后 Disconnect / Connect，再启动定位加载新版本。
9 月 11 日修订时仅做离线验证，没有启动机器人运动或改动单一布局；9 月 12 日 MC 实机读取结果见本文顶部。

## 实机数据链路修订（2026-09-10，历史部署记录）

以下是 9 月 10 日的部署说明，不代表当前 MC 仍未验证；最新 MC 结果见本文顶部，其他未完成的验收不因此视为通过。
当时代码已编译 / 离线验证，运行中的旧会话未被停止；加载部署需先确认机器人静止、
实体急停 / 遥控器可用，再通过管理入口停止定位、Disconnect、Connect。
不要在旧通信会话尚未重启时单独重启定位（新客户端使用 7448）。

- 原先多个本机节点直连机器人 Zenoh，实测网口接收约 960 Mbps。
  现改为 `192.168.168.100:7447 → 本机单个路由 → 127.0.0.1:7448 → 各节点`。
  本机路由与 SDK / bridge 共用 monitor cgroup；端口占用拒绝接管，任一子进程退出停止该组。
  离线 rosbag 的 7447 路由不变。`zenoh-live.json5` 现在要求此受管实机路由存在。
- bridge 加载 nav overlay，解决 `livox_ros_driver2/msg/CustomMsg` schema 不可用。
- SDK 收到首个 RobotState 后等待 1 秒，再开启 MC 上报（接口修订见上）；分别记录写入错误、配置 ACK、
  实际接收样本数与新鲜度到 `/d1max_sdk_bridge/speed_report_status`，不把发送成功当作接通。
- 雷达 / IMU 原始时间比本机落后约 17,320,211 秒。仅定位输入层按单一固定时差转换，
  原始话题和地图不变。该方法不等于硬件同步；参数和限制见 d1max_localization README。

当时回归：40 项 Foxglove 测试通过；实际带宽改善、高频速度 ACK / 数据率及实机输入时序
尚待确认。其中 MC ACK / 数据率已于 9 月 12 日完成本次有限时间验收，其余项目单独验证。该修订没有新增运动 / 控制权 / 急停解除功能，也没有改布局。

## 单一布局约定（2026-09-10）

按用户要求，Foxglove 账号与当前桌面只保留 **D1 Max · 实机工作台**
（`lay_0ebVCw8kOlp4hxiD`）。默认布局、早期感知布局和 3 个阶段备份已通过
原生菜单删除；云端删除同步完成。最近分组已收起，避免同一布局显示两次。
删除前的完整 JSON 恢复文件在
`artifacts/layout-cleanup-2026-09-10T14-27-12.782Z/`。
以后更新这一个最终布局，历史备份只导出为本地 JSON，不再往账号列表新增备份布局。
下文历史迁移脚本曾采用“重命名旧布局再导入”的策略，不能直接用于今后的常规更新。

当前版本 **0.8.0，含 2026-09-10 定位接入**。状态区保留 Connect / Disconnect、Web 启停和单向软件急停；布局比例和两侧视角不变。左侧现为 `PCD · LOCALIZATION`：由 Web 启动定位后显示当前 2D 版本的定位 PCD、实时扫描、位姿和轨迹，不加载历史多层点云。Web 按钮旁定位图标显示新鲜状态：绿=匹配锁定，黄=未锁定 / 降级，灰=未运行 / 无新鲜状态。
已取消控制权申请、站立、趴下、模式切换、方向控制、准备运动、异常复位和急停解除。
**没有 URDF / STL 渲染，没有机器人模型页，也不启动模型资源服务。** 原始模型文件和原始 PCD 均保留，不做删除或覆盖。

## 启动

本机已经安装。日常打开 Foxglove 的 **D1 Max · 实机工作台**，点击右下角 **Connect**，无需先开终端。数据源保持 `ws://127.0.0.1:8769`。

- Connect / Disconnect：启动或清理本机 SDK 接收器、单层 PCD 发布器、只读健康探针和 Foxglove ROS 桥。
- Start Web / Stop Web：独立启动或关闭地图处理 Web；运行后右侧小箭头打开页面。停止会取消进行中的处理和本 Web 启动的建图任务，不删除原图或已完成结果。
- 显示未启动、处理中、停止中、数据正常、等待数据、执行失败、端口占用；悬停看具体原因。数据正常要求当前服务会话中新鲜的 SDK、前后雷达及前后图像，以及 MC 数据、回执与频率检查通过，不能仅凭 PID 或 TCP 接通报正常。

新安装或重新部署时，在本目录执行一次（先准备 ROS / SDK / Node 依赖）：

```bash
python3 scripts/install-session-manager.py
npm run build
npm run local-install
```

安装后刷新 Foxglove。
新环境导入 `layouts/D1Max-Monitor.json`。
旧入口 `start_live_controls.sh` 和 `start_live_view.sh` 仍转到手动监控脚本，不能启用运动控制；不要与 Connect 混用，否则显示端口冲突，需先核对关闭旧终端服务。
旧 `start_sdk_console.sh`、`start_gateway.sh` 已明确拒绝启动；历史控制源码留作开发记录，不在本工作台运行。

实机使用 ROS Humble / Domain 24 / `rmw_zenoh_cpp`，不用 Fast DDS。
Zenoh 直连 `192.168.168.100:7447`，SDK 状态接收连接 `192.168.168.168:8081`。
启动不无条件调用 `TakeControl`，仅在用户授权的 APP 释放通知后申请；不发送机器人动作。
无主会话时机器人可将新连接设为主会话，详见上述故障记录。发现已有 SDK 发布者或 8769 端口占用时拒绝重复启动，不自动杀进程。

## 本机启停管理与清理边界

`config/manager.json` 配置机器人有线网卡、固定端点、端口和启停超时。当前网卡为 `enx6c1ff7bc241e`；换网卡需改此项。Connect 检查链路和到两个机器人地址的实际路由，拒绝代理 TUN 的假 TCP 连通；不会替用户修改系统网络。

轻量管理器为 `d1max-session-manager.service`，仅监听 `127.0.0.1:8771`，登录时启动但保持空闲，**不自动连接机器人或启动 Web**。Foxglove 刷新、卸载面板、关窗口只结束状态轮询，不隐式启停服务；需要关闭后台时使用对应 Stop / Disconnect 按钮。

每个服务有独立操作锁和请求标识，重复点击不会堆进程；请求回执丢失只查询状态，不自动重试操作。停止使用专属 systemd 用户服务 cgroup，包含另开进程会话的子孙进程，SIGINT 后有界等待，超时由 systemd 清理整个所属组。只有进程组清空且端口释放才能重启。未知端口占用不会被自动杀掉或接管。

两个工作单元为 `d1max-monitor-managed.service`（停止上限 15 秒）及 `d1max-web-managed.service`（30 秒）；管理器等待上限 40 秒。管理器崩溃会联动关闭工作单元，重启管理器不会自动恢复机器人通信。Web 上次被中断的处理清单会标为 interrupted，不冒充完成；源 PCD 不覆盖。

`config/manager.local.json` 是每台机器单独生成的 0600 本机能力凭据，禁止提交或共享。构建仅把它写入本地扩展，预览不带凭据；dist、foxe 和该 JSON 均忽略。HTTP 校验 Host、Origin、Bearer、管理器会话及固定请求字段，不能传任意命令、进程号或机器人动作。

Disconnect **不是机器人急停**，也不解除任何急停。急停确认期间界面禁止 Disconnect。关闭通信后软件急停不可用，需使用机身急停。

诊断只读日志：

```bash
systemctl --user status d1max-session-manager d1max-monitor-managed d1max-web-managed
journalctl --user -u d1max-monitor-managed.service -n 50
```

## 布局

- 上方：左侧 PCD 窗口、右侧实时双雷达/前后图像，各占一半；PCD 数据开关不改变布局。
- 下方一排：双相机、两组速度曲线、一个双电池小面板、连接/Web 按钮及单向软件急停。
- 双电池在同一个原生分组中同时显示为两条横向电量条，不再是两个大圆盘。没有新增可切换的业务页面。

只有一个业务页面；电池分组仅含一个固定页且两块电池同时可见，没有说明卡片、原始 JSON、操作面板或 2D 地图。
原来的六个图标已移除，不再显示姿态、控制来源和定位占位图。两个启停按钮占用原来的位置；下方保留软件急停和一个只读硬件急停指示。未知急停显示问号，不显示为安全。

`config/instruments.json` 统一设置单屏分割比例、颜色、速度显示范围与历史窗口。范围是图表量程，不是机器人运动能力或安全限速。
两块电池统一使用原生 `red-yellow-green` 电量色标：低电量红、中电量黄、高电量绿，范围 0–100%，保留刻度和百分比；不是用蓝/金区分电池。色标为连续显示范围，不声称对应厂商低电量告警阈值。
电池分组默认占布局面积 4.8%（此前两个圆盘占 15.84%），减少约 70%；上方保留左右双视图。底部高度经原生桌面检查，避免横条刻度被裁切。使用原生横向 Gauge + 单页分组，不引入新的图表库或机器人订阅。
原生 Gauge / Plot 的字段、配色和窗口可在 Foxglove 面板设置调整；没有主页面 JSON 编辑器。
`config/panel.json` 设置订阅与过期阈值，`config/scripts/instruments.ts` 在 Foxglove 客户端清空过期仪表，不发布 ROS。
SDK 中断而监控心跳仍到达时，超过 2.5 秒清空 Gauge 读数；整个 ROS/网络断开时原生 Gauge 可能保留最后值，必须以 Connect 数据状态和原生连接提示判定其无效，不能视为实时值。
数据未到达时不画成 0，不补造曲线历史。刷新和更改布局均不发送急停请求。

布局升级脚本 `scripts/install-monitor-layout.mjs` 保留两侧用户视角，先备份旧布局，再通过 Foxglove 原生导入/保存；不修改应用缓存。
旧布局导出在 `artifacts/layout-before-instruments.json`；去重配色前备份为 `artifacts/layout-before-battery-colors.json`，本次合并前备份为 `artifacts/layout-before-battery-group.json`，原生菜单也保留对应备份。
原生控件参考 [Gauge](https://docs.foxglove.dev/docs/visualization/panels/gauge) / [Plot](https://docs.foxglove.dev/docs/visualization/panels/plot)；图表语义与未知状态规范见 `CHART_CONTRACT.md`。

## PCD 配置

唯一地图配置：`config/map-view.yaml`。修改后重启地图发布器或整个监控启动器。

- `enabled`：当前为 true，只读取下方配置的一张单层候选图。设为 false 后构建不读取 PCD、启动器跳过地图进程，但左侧窗口仍保留，原文件不删除。
- `node_name` / `status_topic`：当前为 `d1max_floor1_map_view` / `/d1max/maps/floor1/status`，与历史多层发布器区分，状态不会混写。

- `path`：原始或处理后的 PCD 绝对路径。支持 Open3D 可读取的 ASCII、binary、binary_compressed PCD。
- `id`：地图标识，对应 `/d1max/maps/<id>/points`。
- `frame_id`：该 PCD 的显示坐标系。
- `voxel_size`：0 表示显示全部有效点；非零仅对内存中的显示副本降采样，不修改输入文件。
- `publish_period_seconds`：静态点云重发周期，默认 10 秒；使用 transient-local 缓存和独立状态心跳。
- `maps` 支持多个条目。只有经过实际配准的地图才可使用同一坐标系。

当前配置为 `/home/dndx/d1max_nav_ws/maps/runs/20260825_235024/sc_pgo/optimized_map.pcd`，全部显示 **497,844 点**，未降采样、未覆盖源文件。9 月 4 日的 1,300,464 点多层图不订阅、不显示，原文件仍保留。
单层候选的回环日志均为拒绝事件；文件名 optimized 不表示回环已经成功，也不表示地图已经具备导航能力。

`scripts/restore-pcd-window.mjs` 只恢复被移除的左侧空窗口，使用原生导入/保存并备份当前布局；保留其余面板的全部配置、实时视角和精确分割比例，不重启 ROS 或调用机器人服务。
在构建后加 `--load-configured-map`，只切换左侧地图和对应状态来源，按新地图包围盒适配左侧视角，不改变布局或实时感知视角。该脚本会保留原生布局备份。

可只解析文件检查点数和包围盒，不连接 ROS：

```bash
python3 scripts/pcd_map_publisher.py --inspect
```

**单层地图定位已接入，跨楼层 / Nav2 行走未接入。** 在 Web `http://127.0.0.1:8766/#/2d/navigation` 连接、启动定位并提交初值。左侧显示 `/d1max/localization/map_cloud`、`scan_leveled`、`pose` 和 `trajectory`，显示系为 `d1max_loc_map`；不把旧 `d1max_floor1_map` 或实机显示树伪接成同一坐标系。右侧原始雷达 / 视频不变。停止后客户端可能保留旧画面，定位图标和 Web 新鲜度才代表当前有效性。

上面的固定 PCD 配置是旧单层浏览发布器，文件仍保留；定位显示不再订阅它，也不随固定配置切图。定位读取 Web 选用版本的完整 `localization.pcd`，体素降采样只作用于内存匹配地图。算法参数在 `/home/dndx/d1max_nav_ws/src/d1max_localization/config/localization.yaml`；机身速度到前雷达仍是 CAD 近似，需实机标定，绿色定位图标不代表已验证可导航。

SDK 监看桥只请求 `SetMcConfig(true,0,handler)`，在 `/d1max_sdk_bridge/velocity` 发布 `OnMcData` 与真实接收时刻。MC 未收到时明确显示缺失，绝不降级到 RobotState 或 OnSpeedData。当前 LIO 后端不积分 MC 速度；MC 不可用与 LIO 的 IMU／点云故障分别诊断。

`scripts/install-localization-layout.mjs` 先备份当前布局，再通过原生导入 / 保存只更换左侧数据源与定位状态订阅，逐项断言其余面板及分割比例不变。备份在原生菜单“定位接入前备份”及 `artifacts/layout-before-localization.json`。`scripts/localization-layout.mjs` 是可重复执行的数据配置变换，不启动任何 ROS 服务。
当前单层 PCD 位于 `d1max_floor1_map`，实时感知位于 `d1max_lidar`，分别显示；不伪造 map→odom TF，不表示机器人已经定位到建筑中。
以后接入真实定位、楼层标识、跨层拓扑和全局路径后，再在统一坐标系中叠加实际位姿、实时点云和路线。
本次没有生成导航目标，也没有启动定位或导航。

## 状态与软件急停

新 SDK 接收器：`sdk_monitor_bridge`。通过 SDK 回调接收机器人数据，ROS 订阅用于回放检测，唯一 ROS 服务为：

`/d1max/monitor/s_<本次随机会话>/soft_estop`（std_srvs/Trigger）

它只调用 `SoftEmergencyStop(true, 0)`，不接受 false 参数，也没有解除端点。
界面启动、刷新、关页、修改设置均不发送命令。
已经收到有效的“软件急停已触发”状态时显示已触发，不重复发送，不将红色按钮当作切换开关。
请求受理/SDK 回执不等于机器人已停；界面以新鲜 RobotState 显示急停状态。请求后需两份新状态回报才记录“已观察到急停”，即便 ACK 缺失也不会掩盖已收到的 STOP 状态。
反馈未知、过期或请求超时会提示核对实机；不自动重试。此规则只确认 STOP 状态，绝不授予运动许可或恢复机器人。

软件急停解除请交回官方控制端，并先排除原因、核对周边安全。
**软件急停依赖电脑、网络与 SDK，不能替代机身急停，也不是认证安全控制器。**
回放数据不能触发急停。实际 /clock 或后端回放检测会锁存禁用请求；请勿在实机 ROS 域中回放。
回放检测无法覆盖所有无 /clock 且被重命名的播放器，因此回放应使用本机隔离连接。

Foxglove 只监听本机，仅提供 services / connectionGraph 能力，服务名单只有上述急停端点。
没有 clientPublish、参数写入或资产服务能力。ROS 端点过滤不是网络身份认证，勿将机器人 ROS 网络暴露给不可信设备。
配置行为参考 [Foxglove 官方桥接说明](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)。

## 图像显示边界

仍使用现有的前后弧面图像，与双雷达同一个 3D 面板显示，不单独增加“环绕感知”页。
`config/cameras.json` 保存标称视场、投影距离、透明度和显示安装位置。
目前没有实测 CameraInfo，HFOV 111° / VFOV 70° 来自产品资料；安装数值沿用已有显示标定，不加载模型。
这些图像是**近似投影、非 360° 拼接、非带深度的彩色点云**，不能用于测量或直接规划。
`d1max_sensor_rig` 仅为显示参考，TF 与相机校准由 Foxglove User Script 在客户端生成，不发布到机器人。

## 构建与测试

```bash
export PATH=/home/dndx/go2_nav/install/toolchain/node-v24.19.0-linux-x64/bin:$PATH
npm run build
npm test
python3 -m unittest discover -s tests -p test_manager.py -v
npm run package
npm run local-install
```

`scripts/verify_monitor.py` 只订阅生产状态和点云、检查 ROS 图和 WebSocket 暴露能力，绝不调用急停。
`scripts/verify-ui.mjs` 验证两种主题、三种尺寸和五种状态的独立 UI 预览（30 项，先运行 npm run preview）。
C++ 纯状态测试：`../sdk_bridge_ws/src/d1max_sdk_bridge/src/test_monitor_estop.cpp`。
结果与已知限制见 `VERIFICATION.md`。

现场布局导出：`artifacts/monitor-layout-installed.json`；升级前备份：`artifacts/layout-before-monitor.json`。
仅清理本次生成的旧布局菜单备份项，其他布局和 maps 工作区不受影响。
