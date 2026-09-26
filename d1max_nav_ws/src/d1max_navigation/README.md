# D1 Max：Nav2 单层导航

已接通完整 Nav2 软件链：地图、全局/局部代价地图、规划、路径跟踪、任务执行/取消、速度平滑和命令门控。
定位复用已有 Faster-LIO + PCD 匹配 + 高频输出，不另启 AMCL，也不修改原有外参。

**当前机器人断开，默认是独立的软件仿真。软件闭环验证不等于真实定位、避障或实机运动验收。**

## Web 与 RViz 的边界（09-20）

Web 只管理所选地图、定位、Nav2 启停、状态与显式 SDK 运动解锁/锁定；**不在 Web 执行导航目标或取消任务**。
启动时默认打开 RViz2。实际操作使用 RViz 的 `2D Pose Estimate` 设置初值，`Nav2 Goal` 设置目标，
`Navigation 2` 面板的 `Cancel` 取消导航。路径、机身位置、障碍扫描、全局/局部代价地图都在同一份 RViz 配置内。
离线仿真明确标识为模拟数据；实机须先让已有定位链稳定跟踪并通过准入检查，再显式解锁运动。
解锁并不是发目标；存在未取消目标时禁止重新解锁，避免旧任务突然恢复。

软件分层为：定位输出 → Nav2 规划/控制 → 速度平滑 → 官方近场保护 → 命令门控 → 原 SDK 会话。
Web 不计算速度，不替代 Nav2 状态机，也不建立第二条 SDK 连接。1.5 m/s 是不可配置放宽的**指令合速度上限**；
MC 测得超速会锁存停止，但软件无法保证网络断开或主机故障时的真实机械刹停距离。

## 直接使用 RViz

```bash
cd /home/dndx/d1max_nav_ws
./start_navigation.sh start --mode sim
./start_navigation.sh status
./start_navigation.sh stop
```

如果已启动，切换或重新初始化用 `restart --mode sim`，不要同时运行多个启动命令。
地图读取 Web 当前选中的版本；不需要 Web 或机器人运行，不修改地图文件。

1. RViz 的 `02 · 全局代价地图`、`03 · 局部代价地图` 默认开启，包含障碍和膨胀层。
2. 点击工具栏 **Nav2 Goal**，在可通行区域按下并拖出目标朝向。
3. 绿色为全局路径，蓝色为局部路径，橙色轮廓为碰撞包络；软件仿真箭头跟随速度输出移动。
4. 在左侧 **Navigation 2** 面板点击 **Cancel** 取消任务。
5. 仿真中 **2D Pose Estimate** 只重设虚拟起点；选择已知空闲区域，重设前先取消任务。

画面标有 `NAV2 SOFTWARE SIMULATION / ROBOT DISCONNECTED`。
这里的定位、激光扫描与运动是从选中地图生成的理想化测试数据，不能评价实机定位精度。

## 模块分工

| 模块 | 实现 / 输入输出 |
| --- | --- |
| 地图 | 当前版本 `map.yaml` → `/d1max/navigation/map` |
| 定位 | 现有 `d1max_localization`，地图与 PCD 使用同一版本 |
| 障碍观测 | 实机 LIO 点云经 TF 投到机身高度切片 → `/d1max/navigation/scan` |
| 全局代价地图 | 全图，静态地图 + 实时障碍 + 膨胀 |
| 局部代价地图 | 8×8 m 滚动窗口，静态地图 + 实时障碍 + 膨胀 |
| 全局规划 | Navfn A* → `/d1max/navigation/plan` |
| 局部控制 | DWB，20 Hz → `cmd_vel_raw`，可视路径 `local_plan` |
| 速度平滑 | 30 Hz，闭环读取局部里程计 → `cmd_vel_smoothed` |
| 近场停止 | 官方 Nav2 Collision Monitor：扫描过期或进入停止区 → `cmd_vel_collision_checked` |
| 安全边界 | 20 Hz，鲜度/定位/SDK/显式解锁检查 → `cmd_vel_safe` |
| 运动端 | 仿真运动模型；或默认关闭的原 SDK 会话内导航接口 |

上表未写全的话题前缀为 `/d1max/navigation/`。
用户要求的平面速度硬上限是 **1.5 m/s**，不是默认巡航速度。调试默认前向 **0.30 m/s**、横向 0、角速度 ±0.50 rad/s。
`config/navigation_runtime.yaml` 的 `motion_limits` 为统一入口，启动时校验并为 DWB、平滑器、门控生成同一份限速配置；当前 SDK 低速档接口未验收更高速度，配置前向超过 1.0 m/s 直接拒绝。每次启动保存独立配置快照，运行中修改源文件不影响当前会话。
不自动站立、切模式、解除急停或执行后退/旋转恢复。
行为树只有有限重试、清理观测代价层与等待，避免恢复动作突然驱动机器人。

坐标系沿用 `d1max_loc_map → d1max_loc_odom → d1max_loc_base_link`；实机仍由原定位链唯一发布。
机身箭头读取 `/d1max/localization/odometry/global`，不是带雷达偏置的 tracking 位姿。
只有隔离仿真有自己的模拟 TF；不会连接实机路由。

## 接回实机：先定位与代价地图，再放行运动

```bash
# 先在 Web 连接机器人，确认雷达、IMU、MC 数据正常
./start_navigation.sh restart --mode live
```

这会复用现有 SDK 与定位会话，加载相同地图版本，启用真实点云障碍投影和完整 Nav2，
**但默认不允许机器人移动**。没有有效实时数据会拒绝启动，绝不会悄悄退回仿真。
静止完成初始化后，用 RViz **2D Pose Estimate** 在实际位置拖出机头方向。
初值经过原 Web 入口及地图/会话校验，不能把箭头出现当作定位成功。
官方 Navigation 2 面板中的 `Localization active` 在本方案中只是地图服务已激活；
真实定位是否可信要看专用状态提示和 `/d1max/localization/status`，不是该面板标签。

运动必须同时满足三个独立条件：

- SDK 监测启动时显式配置 `D1MAX_NAVIGATION_CONTROL_ENABLED=true`；普通 Web 连接仍为只读。
- 导航以 `restart --mode live --enable-motion` 启动；这个选项仅打开能力，不是解锁。
- 数据、控制权、急停、定位与标定检查全部通过后，显式调用 `/d1max/navigation/navigation_command_gate/arm`（`std_srvs/srv/SetBool`）。

SDK 监测由已有 systemd 用户服务启动，不能只在终端 export 后假定 Web 已继承。
需要启用能力时，先停止导航和定位、在 Web 断开连接，确认 monitor 服务已停止；
通过 `systemctl --user edit --runtime d1max-monitor-managed.service` 添加
`[Service]` 下的 `Environment=D1MAX_NAVIGATION_CONTROL_ENABLED=true`，再由 Web 连接。
恢复只读时把同一配置设为 `false` 并重新连接，不要删除其他已有服务配置。本轮未开启此能力。

SDK 出口复用当前 monitor 进程的主会话；不要再启动一个 SDK console 抢占连接。
原生 `Move` 是归一化指令，不是米/秒。转换仅在官方低速档、普通模式、前向且 SDK 控制权确认时允许。
导航门控使用 SI 单位，SDK 会话接口另外校验会话、代次、命令来源、序号和时效。
SDK 桥正常运行时，指令中断 250 ms 会触发停止；失去健康条件后需要重新显式解锁。
如果电脑进程被强杀或整条链路断开，不能保证 250 ms 内执行 SDK 停止；原生 Move 有最长约 1 s 的有效期，软件门控不是硬件急停。

**当前外参和时间对齐验收标记仍保持未确认，没有为了跑通演示强行改成已验收。**
当前实机运动因此仍受保护。机器狗重新连接后还需核对：

- 真实初值匹配、静止漂移、快走时的定位连续性；
- 2D 地图未知区和实际可通行性（当前地图空白背景被标为自由，不代表有地面）；
- 机器人行走足端包络、点云高度切片、低矮障碍；
- 动态障碍清除/绕行、停止距离、断链和实体急停。

## 配置、隔离与清理

- `config/nav2.yaml`：代价地图、规划器、控制器、速度限制、膨胀与 footprint。
- `config/navigation_runtime.yaml`：坐标系、初始高度种子、点云切片、仿真起点。
- `config/collision_monitor.yaml`：官方近场保护，停止区为已有 footprint 加 padding 后再留 0.10 m；扫描超时 0.5 s。它不代替规划器、硬件急停或实际制动距离验收。
- `behavior_trees/`：单点、多点任务树；`rviz/navigation.rviz`：完整导航布局。
- `navigation_command_gate`：独立命令安全边界；SDK 的可选出口默认关闭。
- 所有模式保持 **rmw_zenoh_cpp、Domain 24**。仿真只连接 `127.0.0.1:7460`，禁止发现和外部上行；实机复用 `7448`。
- Web 启动带 `--web-owned`，导航随托管 Web 停止；独立终端启动不依赖 Web 生命周期。每次会话有新 `navigation_session_id`，过期页面不能指挥后来启动的会话。
- 单一托管服务 `d1max-navigation.service`。重复启动拒绝；关闭 RViz 或任一子进程异常退出会清理整组导航进程。
- 停止不杀共享 Web、SDK、定位；实机导航绑定上游服务，避免它们停止后继续输出旧命令。
- 已实测停止后导航 cgroup 清空、7460 端口释放，再启动成功。当前 Humble RViz/插件在退出时仍可能记录 `-11`；未当作正常退出掩盖，但不会留下本模块的后台进程。
- 配置快照与报告：`log/navigation/`；日志：`journalctl --user -u d1max-navigation.service -n 80 --no-pager`。
- 旧的地图/定位诊断入口见 [LOCALIZATION_TEST.md](LOCALIZATION_TEST.md)。完整导航启动时会清理其旧窗口，避免叠加。

## 离线验收

### 09-20 最终复验（Web 启动 + RViz）

报告：`log/navigation/acceptance_20260920_final_rviz.json`，整体 `passed: true`。

- 全局/局部代价地图、障碍订阅、TF 和 Nav2 Action 服务全部通过。
- 120 点全局路径；约 3 m 的目标 12.23 s 到达，位置误差 0.145 m。
- 取消后安全指令为零，观察期位移小于 0.1 mm。
- 动态障碍在两张代价地图各产生 13 个致命栅格，移除后均清零。
- 近场停止区内 45 个新鲜扫描点，官方节点明确报告 STOP；14 次测试输入均未产生非零下游指令，仿真位移为零。
- 仿真使用独立 Zenoh 7460；未启动 SDK、未向真实机器人发送运动指令。

本次为理想化软件闭环验证，不代表真实传感器、实机定位精度或制动距离已经验收。

### 历史记录

2026-09-18 已在 `grid-0af429985e454a9e99c8aaef`（单层导航地图，2130×1246，0.05 m/格）实际验证：

- 全局与局部代价地图激活，含障碍、内切和膨胀代价；实际订阅障碍扫描。
- `ComputePathToPose` 成功，生成 120 个路径点。
- `NavigateToPose` 成功，约 13.4 s 到达 3 m 目标，位置误差 0.132 m 内，未触发碰撞保护。
- 第二个导航目标取消成功；安全速度为零，停止后观察 0.7 s 无位移。
- Web、SDK、实机定位服务始终未启动，没有向真实机器人发送命令。

证据：`log/navigation/20260918_133708_sim_17846/acceptance.json`。
统一角速度限幅后的第二轮同样成功：12.1 s、误差 0.147 m、取消后零速度与零位移；
报告位于 `log/navigation/20260918_133923_sim_18578/acceptance.json`。
导航模块 218 项单元测试通过，SDK 桥构建及 5 项 CTest 通过。
上述 09-18 报告未单独注入动态障碍。现在验收工具新增：在隔离仿真中插入一个临时障碍，核对两张代价地图新增致命单元，再移除并核对清除；另向官方 Collision Monitor 输入近场障碍与测试速度，核对输出为零和仿真无位移。它只证明软件链路，仍不等于实机避障验收。以新生成的 `acceptance.json` 各项结果为准，不把新增测试代码视为已经运行通过。

09-20 首次新增验收记录 `log/navigation/20260920_214034_sim_13858/acceptance.json`：目标到达及取消通过，但动态障碍删除后两张代价地图各残留同一个栅格，因此整体判定失败。只读复核确认仿真障碍已删除、扫描继续更新；Humble 的射线清除先按 0.35 m 最小距离量化起点，360 束扫描的后方命中射线恰好绕过原障碍边缘单元。隔离仿真改为显式 720 束，与实机投影的 720 分箱一致；没有调用清空代价地图来伪造通过。这个调整不保证实机所有残影消失，实机缺失点云分箱仍是 NaN（未知），绝不凭空转换成自由空间。

第二次 `log/navigation/acceptance_20260920_720_rviz.json` 已验证两张代价地图各新增 13 格、移除后清零；近场停止检查最初把“没有继续收到零 Twist”误当成失败。安装的 Humble Collision Monitor 明确在停稳超过 `stop_pub_timeout` 后停止发送零指令，这是原生行为，并非输出了非零速度。独立限时核查 `log/navigation/collision_stop_audit_20260920_v2.json`：45 个新鲜扫描点进入停止区，官方节点明确报告本次 Polygon STOP，门控持续零输出，仿真无位移。验收现区分“非零泄漏”和“原生停止发布零值”：后者只有同时具备上述明确证据才允许通过；没有放宽保护参数，也没有单凭静默判定安全。

重新验收（先启动 `--mode sim`，不支持对实机执行）：

```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=24
export ZENOH_SESSION_CONFIG_URI=/home/dndx/d1max_nav_ws/src/d1max_navigation/config/zenoh-offline-session.json5
ros2 run d1max_navigation verify_navigation --exercise-sim
```

去掉 `--exercise-sim` 仅做只读检查，不发送导航目标。构建与单元测试：

```bash
colcon build --packages-select d1max_navigation --symlink-install
colcon test --packages-select d1max_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/d1max_navigation --verbose
```
