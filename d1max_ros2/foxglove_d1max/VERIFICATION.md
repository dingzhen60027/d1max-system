# v0.7.4 左侧窗口恢复及单层点云显示 — 2026-09-10 15:27 CST

- 先恢复左侧 PCD 空窗口；随后按用户要求加载 `runs/20260825_235024/sc_pgo/optimized_map.pcd`。当前原生布局 ID 为 `lay_0ebT8DAcov9bFrkG`，已原生保存并刷新验证。
- 左 PCD / 右实时感知各占 50%，外层高度保持 67.92899408284023%；底部各面板和实时视角保留。只为新地图适配左侧相机，状态来源切换到单层专用话题。
- `/d1max/maps/floor1/points` 实际收到 497,844 点，独立坐标系 `d1max_floor1_map`，无伪造地图定位 TF。原始 PCD 未改动；没有加载旧多层话题或开始建图/导航。
- 新单层显示进程 PID 77803（工具会话 73082），节点 `d1max_floor1_map_view`，状态话题 `/d1max/maps/floor1/status`。旧监控父进程及 SDK 未重启，PID 61082 的旧多层发布器未被直接终止，避免父进程 `wait -n` 联动退出。它不在当前 Foxglove 订阅中。
- 后续统一重启监控时，先核对并停止本次独立的单层显示进程，再使用新配置启动，避免重复发布；不要单独杀旧父脚本管理的子进程。
- 30 项 TypeScript 测试、3 项禁止加载测试通过。原生截图 `artifacts/single-floor-loaded-native.png` 已人工检查到左侧真实点云；`artifacts/single-floor-loaded-checks.json` 无页面异常/模型资源请求/状态区溢出，SDK 新鲜。扩展 0.7.4 已重新打包安装。
- 恢复窗口前与加载单层前的备份分别为 `artifacts/layout-before-pcd-window-restore.json` 和 `artifacts/layout-before-single-floor-load.json`；当前导出 `artifacts/monitor-layout-final.json`。导入不重置用户实时相机；用户继续操作视角时不回写旧角度。
- 后续应用户要求启动地图处理 Web（PID 78469、会话 39992），浏览器打开 `http://127.0.0.1:8766`。仅启动页面服务，未替用户执行清理、归档、删除或处理任务。

# v0.7.3 单楼层调试准备 — 2026-09-10 15:13 CST

- 当前布局仍为 `lay_0ebSyr2k5GHPue21`。已通过原生菜单移除 `3D!d1map`，保存、刷新后确认历史多层视图不存在；实时双雷达/图像、速度、双电池和状态/急停保留。
- 默认 `config/map-view.yaml` 设为 `enabled: false`。发布器检查输出 `{"enabled":false,"maps":[]}`；新启动脚本跳过该进程，构建布局不读取原 PCD。
- 本次没有终止监控父进程/SDK；原静态发布进程仍由旧父脚本管理，避免 `wait -n` 的联动退出。当前 Foxglove 已卸载其点云，下次正常启动不再启动该发布器。
- 28 项 TypeScript 测试、3 项禁止加载测试和 shell 语法检查通过。禁用测试确认不调用 PCD 读取、不启动 ROS 节点。
- 原生检查报告：`artifacts/single-floor-monitor-checks.json`；截图：`artifacts/single-floor-monitor-native.png`。检查时 SDK 新鲜，页面无报错、无溢出；未调用机器人服务。
- 历史文件只读核对结果见 `SINGLE_FLOOR_MAPS.md`。没有加载候选、裁剪、覆盖文件或启动新的建图/导航。

# v0.7.2 紧凑双电池历史记录 — 2026-09-10 15:00 CST

- 当前布局 ID：`lay_0ebSyr2k5GHPue21`。双电池为一个固定原生分组内的两条横向电量条，两者同时可见，无需翻页；红黄绿与 0 / 50 / 100% 刻度保留。
- 上方点云铺满宽度，双相机、曲线、电池分组和状态/急停位于底部一排。电池区域约占布局面积 4.8%，相比此前 15.84% 减少约 70%。
- 原生实测底部 28% 高度会裁切 Gauge 刻度，已调整到约 32%；再次保存刷新和截图检查，刻度与安全区完整可见。
- 27 项测试通过，包括电池必须同组、占比不超过 5%、速度不重复、颜色一致、迁移幂等和视角保留。原生页面无报错、无模型资源请求、状态区无溢出。
- 最新原生截图：`artifacts/compact-batteries-native.png`；检查报告：`artifacts/compact-batteries-native-checks.json`。SDK 新鲜状态和实时相机/雷达已显示；本次未发送机器人指令。
- 合并前备份：`artifacts/layout-before-battery-group.json` 及原生菜单“双电池合并前备份”。

# v0.7.1 去重与电量配色历史记录 — 2026-09-10 14:51 CST

- 当前布局 ID：`lay_0ebSwto0Q9ANiqns`。9 个面板：速度与角速度字段各只显示一次，双电池使用相同的 0–100% 红→黄→绿色标。
- 26 项测试通过；原生导入、保存、刷新后的面板数量、字段不重复、色标方向、状态区无溢出检查通过。
- 本次原生截图已收到真实 PCD、双雷达、双视频和 SDK 新鲜状态；不再是上一轮网卡断开的空数据截图。`artifacts/instruments-native.png` 与 `artifacts/instruments-native-checks.json` 现为本次最新结果。
- 只修改显示配置，不触发/解除软件急停，不发送运动指令；原有 PCD / 感知视角保留。
- 修改前备份：`artifacts/layout-before-battery-colors.json`，原生菜单“电量配色改版前备份”。

# v0.7 单屏图表历史记录 — 2026-09-10 13:51 CST

- 当前布局：`D1 Max · 实机工作台`，ID `lay_0ebScrItgvKS1XnK`。单一原生 mosaic，12 个面板、5 个 Gauge，没有 Tab / RawMessages / Markdown / URDF。
- 26 项 TypeScript 测试通过。两主题 × 三尺寸 × 五状态，共 30 项独立预览检查通过（`artifacts/instruments-ui-checks.json`）。
- 原生导入、显式保存、等待持久化、刷新后复核通过；无页面异常、无模型资源请求、无图标或急停区溢出。见 `artifacts/instruments-native-checks.json` / `instruments-native.png`。
- 两侧原有 3D 视角保留；旧布局原生备份和 `artifacts/layout-before-instruments.json` 都保留。当前导出：`artifacts/monitor-layout-final.json`。
- 实机网卡 `enx6c1ff7bc241e` 为 DOWN、没有机器人网段地址；机器人 Zenoh 握手失败。此次原生验收是断流状态，不宣称收到了当前点云、视频或真实仪表更新。未使用模拟值填充实机页面。
- 未知图标、禁用急停和空读数如实显示；SDK 过期且心跳仍到达时的 Gauge 清空逻辑通过隔离脚本测试。全链路失联时原生仪表可能保留最后值，SDK 断流图标标记其不可作实时状态。
- 本次只改显示层，没有发送机器人服务请求、运动、急停或解除；现有监控服务与 Zenoh 配置未重启或修改。无 Fast DDS。
- 下方 v0.6 的真实数据验收是历史记录，不用于证明本次网络已经恢复。

# v0.6 轻量监控历史记录 — 2026-09-10

- 当前布局：`D1 Max · 实机工作台`，ID `lay_0ebSQluxfcmn6nMY`。静态 PCD 与实时双雷达/图像并排，监控与软件急停区贯穿所有标签。
- 已移除操作页、方向盘、控制权/姿态/模式/运动准备/复位/解除急停入口；旧 SDK 控制进程及两个旧网关已退出。新接收器 `sdk_monitor_bridge` 不提供任何运动或恢复接口。
- 已移除 URDF/STL 图层、机器人模型页，停止 8770 模型资源服务；模型源文件保留。重载和标签切换期间无模型资源请求。
- `artifacts/monitor-live-checks.json`：唯一 SDK 发布者；只存在一个会话级 `std_srvs/Trigger` 软件急停服务；订阅仅 `/clock` 与内部参数事件。没有旧控制/租约/速度端点。WebSocket 实际协商只开放 `services`、`connectionGraph`，服务列表仅急停，不支持客户端发布、参数或资产请求。
- 最近一次复核收到 3 份真实 SDK 状态、前后雷达 39 / 41 帧，以及完整 PCD 1,300,464 点。PCD 为独立的 `d1max_pcd_map`，没有伪造 map→odom 或全局定位。
- 新接收器检查时 `sdk_commands_sent=0`，未持有控制权。机器人原有软件急停仍为已触发、姿态趴下。本次验证没有发送实机急停、解除急停、申请控制权或运动。
- 旧监控升级前布局菜单备份项已移除；恢复导出保留在 `artifacts/layout-before-monitor.json`。默认布局和原感知工作台保留。隔离 UI 预览服务也已停止，未留下模型或控制进程。
- `artifacts/monitor-native-checks.json`：实际桌面重载、地图信息、地图与感知切换、总览三路动态画布检查通过；无 pageerror、无模型资源请求、无控制按钮。`monitor-native.png` 为原生桌面截图。
- 23 项 TypeScript 测试通过；两主题、三尺寸、四个标签共 24 项隔离组件检查通过（`artifacts/monitor-ui-checks.json`）。C++ 无 SDK / ROS 的急停状态测试覆盖连接/回放禁用、重复请求、无 ACK 的 STOP 回报、超时、迟到反馈和不自动重试。
- 新 C++ 目标编译通过，Python / shell 语法检查通过。扩展 0.6.0 安装文件与构建产物 SHA256 相同：`dfd81853bc2e007e64c730f10b049a921484c0a88f9d6b21615103e5e1d6adb1`。
- 仍使用 ROS Domain 24 / rmw_zenoh_cpp，不使用 Fast DDS。前后图像仍是标称视场近似投影，不是标定或 360° 拼接。

## 限制

- 尚未实现地图定位、楼层识别、跨楼层路径搜索或导航；此版本仅显示数据。
- 默认 SC-PGO PCD 的该次日志没有接受回环，不能把文件名 optimized 理解为回环成功。
- 新急停通路仅做代码、纯状态、前端模拟及生产接口只读检查；没有为验收而触发实机急停，不宣称急停响应时间、制动距离或断网安全经过实测。不能替代机身急停。
- 历史控制代码保留作开发记录，但本工作台启动器已停用其入口。下方历史说明不适用于当前监控布局。

# v0.5 历史记录 — 2026-09-09

## 已完成

- 新增 SDK 专用控制接收器与无副作用状态机；旧 SDK 控制桥和只读接收器保留。新适配器不自动获取控制权，速度不走通用 `/cmd_vel`，命令反馈与动作完成分开处理。
- 站立状态 1 判定为进行中，命令 ACK 和两份新的匹配 RobotState 才能完成一步；模式/控制权/急停/故障/反馈时效/会话心跳分别检查。转换超时不重试，恢复连接/急停不续跑，LOCKED 需单独确认站立。
- 面板增加逐动作许可、目标/步骤/结果、模式与异常复位入口；保留同屏点云/图像/URDF 和原有布局。扩展 0.5.0 已构建、本地安装并打包。
- 24 项 TypeScript 测试、16 项 Python 网关测试、16 组无 ROS/SDK 的 C++ 状态机场景通过。
- `artifacts/control-mock-checks.json`：隔离 ROS Domain 91 / 本机 Zenoh 上的 11 项完整链路检查通过，包括从请求到 ACK/RobotState 确认、非法切换拒绝、站立中锁住按钮、速度超时归零、租约失效、急停/恢复、释放控制权。测试可执行程序未链接供应商 SDK、只接受 127.0.0.1 参数，没有连接机器人。模拟速度/急停不代表实机动作验证。
- `artifacts/control-ui-checks.json`：明暗主题 × 三种面板尺寸 × 四个标签的 24 项组件检查通过，无横向溢出，安全区固定可见；预览没有 ROS 控制能力。
- 新启动脚本通过 bash 语法检查；新 ROS C++ 目标编译通过。依然使用 `rmw_zenoh_cpp`，没有切换到 Fast DDS。

## 尚未完成的实机验收与阻塞

- 切换前只读检查发现有线网卡 `enx6c1ff7bc241e` 为 DOWN、没有原来的 192.168.168.10 地址；192.168.168.100/168 的路由进入 Meta 虚拟网卡 198.18.0.1。Zenoh 握手失败，代理返回的 TCP connect 不能证明机器人可达。
- Foxglove 桌面调试端口 9224 随后也不可达，未能在实际桌面重载验收新控件。
- **本次没有启动实机 sdk_console_bridge，没有替换原实机只读接收器/8769 桥，没有申请控制权、发送运动或解除急停。** 旧实机进程、模型服务和传感器配置保留。不能把本地安装/模拟验收称作实机控制已接通。
- 待接好网线、机器人在线且静止、重新打开 Foxglove 后，明确替换旧只读进程，再运行 `scripts/verify_live_control.py` 检查启动锁定、未持有本会话控制权、命令计数为 0。这个实机通过报告目前不存在。
- 真机站立、速度、急停响应和断网制动距离均未测试。电脑侧看门狗不能保证网络已断时停止命令能送达；不能替代硬件急停或用于无人看守运行。

下方为历史实机可视化记录，不能用于证明 v0.5 控制验收。

# v0.4.1 历史同屏感知验收 — 2026-09-09

- **D1 Max · 实机工作台** 已保存为 `lay_0ebNYQvNbANVKFYa`。原“感知总览”的同一个原生 3D 面板同时开启前后雷达、前后图像弧面和 URDF；独立“环绕感知”标签及其专用面板已移除。
- 保留现场主视角、面板比例、下方双视频与速度曲线、右侧 SDK 面板。四个标签为感知总览、状态诊断、雷达与 IMU、机器人模型。图像弧面使用 72% 不透明度，配置为 `config/cameras.json` 的 `imageOpacity`。
- 22 项 TypeScript 测试通过；0.4.1 构建和本地安装通过。原生 UI 保存、重载、切到模型再返回后，合并 3D 和两个视频的非空画布均持续变化；无 pageerror，SDK 在线，软件急停按钮仍禁用。画布抽样不是帧率测量。
- 现场布局导出：`artifacts/merged-layout-final.json`；验收报告：`artifacts/merged-ui-checks.json`；实机截图：`artifacts/merged-perception.png`。
- 仅删除本次升级创建的 **D1 Max · 实机工作台（合并前备份）** 菜单中间项，可重新导入 `artifacts/live-layout-before-merge.json` 恢复。其他布局保留。
- 本次没有重启传感器、SDK 接收器、模型服务或实机桥，没有调用机器人动作、控制权或急停接口；SDK 接入和安全设置未修改。继续使用 Domain 24 / Zenoh，不使用 Fast DDS。
- 相机仍为按标称视场估算的显示投影，不是实测标定、360° 拼接或点云着色；URDF 仍为无关节反馈的参考姿态。下方 v0.4 的独立“环绕感知”描述为历史记录，已被此次同屏布局取代。

# v0.4 历史弧面图像 + SDK 验收 — 2026-09-09

## 已完成与证据

- **D1 Max · 实机工作台** 新增默认“环绕感知”标签页；原感知总览、状态诊断、雷达与 IMU、机器人模型全部保留。保存后的布局 ID `lay_0ebNC3Wi9cInZzxh`。
- 原生 3D 两个图像主题分别绑定客户端 `foxglove.CameraCalibration`，距离 2 m、平面投影因子 0。实机前后 JPEG 显示为模型两侧弧面，非截图纹理、非合成视频。
- 相机视场来源：本地产品规格书 HFOV 111° / VFOV 70°；安装位置来源：原始 URDF F/R_IMX415_JOINT，±0.4123 m、z=0.0378 m。内参只按标称视场估算，畸变未实测；没有 ROS CameraInfo，未声称标定或 360° 拼接成功。说明牌明确保留侧面盲区与模型参考姿态。
- 新增 SDK `sdk_telemetry_bridge`，连接 `192.168.168.168:8081`，只接收自动 RobotState 和故障报告。7.08 秒收到 7 份 RobotState，接收时间持续增加，最后一份未过期；连接反馈 connected。详见 `artifacts/live-sdk-checks.json`。
- 实测速度、电池、趴下状态、控制来源、软硬件急停状态已进入紧凑面板；原生速度曲线有数据。该次记录电池 97% / 87%，速度 (-0.010, -0.009) m/s、转向 0.001 rad/s；数值是当时报告，不是持续保证。硬件急停状态为已触发，未解除或发出动作。
- 运行图检查：SDK 接收器 **0 个 ROS 服务**，仅 rclcpp 内部 `/parameter_events` 订阅，无命令订阅；只发布 4 个状态/故障主题。二进制动态符号仅使用 SDK 构造/析构、Connect、Disconnect、SetDataCallback、IsConnected，没有动作/控制权 API 引用。
- 图像脚本在实际 Foxglove 的类型检查中无问题；语义类型检查已补入本地测试，覆盖固定长度矩阵类型和配置类型。
- 22 项 TypeScript 测试通过；SDK C++ 目标编译通过；所有新增启动脚本通过 bash 语法检查。此前 10 项网关安全测试继续保留，网关实现未修改。
- 实际 UI 重载后“环绕感知→感知总览→环绕感知”检查通过：非空弧面画布持续变化，双雷达与两个视频画布也持续变化，SDK 在线、软件急停按钮禁用、无 pageerror。此为画布抽样，不是帧率测量。记录 `artifacts/surround-ui-checks.json`。
- 0.4.0 extension.js SHA-256：`36bbc6919cdee8c9f44f9d8f8bb23fe001f72010e07d05ca9c87ce5581b15ca5`。

## 保存与限制

- `config/cameras.json`：视场、安装点、光学轴、弧面距离和默认视角；`config/scripts/camera-calibration.ts`：JPEG 尺寸读取与客户端校准消息。
- `artifacts/surround-overview.png`：实机弧面、模型与 SDK 状态截图。
- `artifacts/surround-layout-final.json`：现场保存的布局导出；`artifacts/live-layout-before-surround.json`：升级前恢复备份。
- 本次临时布局 **D1 Max · 实机工作台（环绕升级前）** 已从菜单删除，仅清理这个升级中间项；可从升级前 JSON 重新导入恢复。
- `scripts/start_live_view.sh` 现在包括 SDK 只读接收器；已经运行时不要重复启动。`scripts/start_sdk_telemetry.sh` 可单独启动，但已有状态发布者时拒绝重复连接。整套启动脚本通过语法检查；本次各服务是分开启动验证，未为测试重复连接机器人。
- 原始 URDF/STL、机器人 TF、相机编码配置、SDK 控制桥、安全网关、maps 与 bag 未改。SDK 原有控制桥仍不运行；新接收器不调用控制权或动作 API，未做机器人运动试验。
- 接收的姿态状态枚举不包含关节角；URDF 仍是参考姿态。故障报告是事件，不代表完整故障清单；没有控制状态机时，运动就绪/故障锁存保持未知。
- 以下 v0.3/v0.2 为历史记录；“SDK 未接入”限制已由本次只读状态接入更新，控制动作和关节反馈仍未验证。

# v0.3 历史实机接入验收 — 2026-09-09

## 实机已验证

- 原生 **D1 Max · 实机工作台** 已导入、保存并打开，布局 ID `lay_0ebN4hosjqAVuKXf`。新增“机器人模型”近景页，感知总览同时显示原始 D1 Max URDF、双雷达和前后视频。
- 实机通过 `192.168.168.100:7447` 直连 Zenoh；ROS Domain 24 / `rmw_zenoh_cpp`。本机只读 Foxglove 桥 `127.0.0.1:8769`，模型 HTTP 服务 `127.0.0.1:8770`。不使用 Fast DDS。
- 约 6 秒只读订阅：前相机 9.97 Hz、后相机 9.64 Hz（JPEG）；双雷达各 9.47 Hz、86,400 点/帧；前后 IMU 约 128/131 Hz。这是接收端短测，包含订阅启动时间，不是长期吞吐保证。详见 `artifacts/live-inspection.json`。
- 原始 URDF XML 可解析；全部 29 个唯一网格 URL 的 HEAD 返回大小与源 STL 一致，一个小网格完整 HTTP 下载哈希与源文件一致。非允许资源为 404、POST 为 405。实际 Foxglove 成功渲染整机网格。
- 0.3.0 扩展 TypeScript/构建、17 项 TS 测试、10 项 Python 安全测试通过；已打包与本地安装。项目/安装目录 extension.js SHA-256 均为 `bfe322cb221faa5f02f51f2ada1e984ee1f029ac2546b60ee9db24d82625c635`。
- 实际 UI 切换总览→模型→雷达与 IMU→总览后，点云及前后相机的非空画布内容持续变化，后续采样也继续变化；没有 pageerror。此项是画布抽样检查，不是浏览器帧率测量。记录在 `artifacts/live-ui-checks.json`。
- 从运行中实机 bridge 的参数文件核对：capabilities 只有 `connectionGraph`；发布、服务、参数和资产允许列表均为不匹配正则 `a^`。软件急停按钮禁用。
- 本次未启动 SDK 控制桥/实机操作网关，未发送运动、模式切换或急停指令。只新增本机资源服务、只读传感器连接和客户端显示 TF；未修改机器人 TF、原始 URDF/STL、maps 或 bag。

## 限制与排障记录

- 实测 `/joint_states` 发布者为 0。腿部是 URDF 零位参考姿态，不是实时机器人动作；SDK 速度/电量/状态仍未知。
- 模型朝向和挂载位置按已有雷达基线与 CAD 安装点近似显示对齐，未经导航标定验收，不能用于规划或控制；没有声称已接入全局定位。
- 机器人消息头日期与本机日期不同。显示变换保留消息头时间，没有修改机器人时钟。跨设备同步需要另行处理。
- 原始 STL 合计约 126 MB。先尝试桥内 package 资源时遇到中文路径问题，之后一次测试也出现数据流停滞；无法仅据此确定停滞原因。最终把模型移到独立本机 HTTP 服务，关闭实机桥 assets 能力；切页后持续刷新验证通过，复核时桥 TCP Recv-Q 为 0。
- Zenoh 日志偶有 missed sample 警告，本次未做长时间稳定性/控制链路验收。
- 最初尝试在页面 CDP Network 上统计原生 WebSocket 帧未得到事件，不能据此判断流停止；改用非空原生画布抽样，并与 ROS 接收端短测交叉确认。

## v0.3 产物

- `layouts/D1Max-Live.json`：四标签实机原生布局，源配置 `config/live-model.json`、`config/layout.json`。
- `scripts/start_live_view.sh`：一条命令启动本机模型资源服务与实机只读桥；端口占用时拒绝重复启动。
- `dndx.d1max-console-0.3.0.foxe`：紧凑面板安装包；不包含机器人网格，需要项目中的本机资源服务。
- `artifacts/live-overview.png`、`live-model.png`：真实实机画面与原始模型截图。
- `artifacts/live-layout-installed.json`：保留现场视角与面板比例的导入版本；`live-layout-before-http.json` 是资源迁移前的恢复备份。
- 本次临时创建的 **D1 Max · 实机工作台（资源迁移前）** 已通过 Foxglove 菜单删除，恢复数据保留在上述 JSON 备份中；未删除其他布局。
- 下文 v0.2 是历史离线验收，关于“相机尚待实机验证”的历史限制已由本次双相机测试更新；SDK 和机器人动作仍未验证。

# v0.2 历史验收记录 — 2026-09-09

## 已通过

- TypeScript 检查、布局生成、扩展构建、0.2.0 打包和本地安装。
- 15 项 TypeScript 测试：遥测未知/过期、配置限幅、网关会话校验、回放禁控、迟到解锁响应、原生布局引用完整性、SDK JSON 解析、显示 TF 外参与时间戳。
- 10 项 Python 网关安全策略测试。
- 24 项浏览器组件检查：240×520、300×680、420×800，明/暗两套主题，各四个标签；无容器横向溢出，底部安全区可见，只读控件禁用。
- 空数据不伪造速度，非法配置 JSON 拒绝保存；浏览器无 pageerror。
- 实际 Foxglove 原生三标签布局导入、切换、保存成功；旧全页扩展未挂载。
- 真实 bag 传感器片段测试：双雷达原生 3D 可见，显示标定生效，双 IMU 角速度曲线有数据，IMU Plot 无数据匹配警告。
- 实际只读网关收到 3 次心跳：模式 replay（锁存）、未解锁、无会话控制服务、无该网关 /cmd_vel 发布者。
- 正式 Foxglove 软件急停与解锁控件禁用。所有动作测试仅为无 ROS 的单元测试，未发送任何实机动作。
- 布局 **D1 Max · 感知工作台** 已保存，个人布局 ID：lay_0ebMVcde9g8WqXDk。
- 项目与安装目录 extension.js SHA-256 一致：
  `1497330b43d6935f3b08581db38b9c47ae1b69911ae3fd6c4e6bdff8afe7fe58`。

## 清理范围

- 删除旧个人布局 **D1 Max 控制台** 和本次调整期间产生的中间布局。
- 0.1 扩展已被 0.2 替换；旧 App/样式重新实现，自绘点云/相机源码和 Three.js 依赖移除。
- 0.1 安装包与旧截图移出项目，临时恢复目录：`/tmp/d1max-v01-retired.fWgvBK4x`。临时目录不保证跨重启保留；已删除的个人布局没有恢复保证。
- 原有 **D1 Max 双雷达回放**、**默认**、**test** 布局保留。
- maps、原始 bag、点云处理配置、SDK 桥接、安全网关实现未修改。
- 测试用 rosbag 播放已按时停止；隔离组件预览服务测试结束后停止。

## 数据与限制

测试 bag：`/home/dndx/d1max_rosbag903/slam_raw_20260903_232807`。只发布选定的雷达、IMU、静态 TF 和回放 /clock 到本机隔离 Zenoh router，未连接机器狗。

该 bag 没有相机或 SDK 状态，故图像话题不存在、速度曲线无数据、速度/电量/姿态未知。SDK JSON 转换已做单元测试；原生 SDK 速度曲线、双相机及所有机器人动作尚待实机现场验证。

点云为当前传感器扫描，不是建图结果。显示标定仅在 Foxglove 内转换，不向 ROS 发布 TF。消息头时间与 bag 接收时间不同；当前布局不依赖未经验证的 odom→雷达关系。

第一次较长流式测试触及 Foxglove 250 MB 回放缓冲上限，旧消息按原生策略丢弃；最终验收用更短、低速片段。ROS 流的时间轴是接收时间；需要精确 bag 时间轴可直接用 Foxglove 打开 bag 文件。

## 产物

- `layouts/D1Max-ANYmal.json`：完整可导入原生布局；源配置在 config/layout.json。
- `dndx.d1max-console-0.2.0.foxe`：紧凑 SDK 面板安装包，不含测试预览。
- `artifacts/native-overview.png`、`native-imu.png`：实际 Foxglove / 真实 bag 截图。
- `artifacts/native-diagnostics.png`、`native-controls.png`：诊断及禁控界面。
- `artifacts/native-checks.json`、`ui-checks.json`：自动验收结果。
- `artifacts/widget-dark.png`、`widget-light.png`：明确标记的组件模拟状态截图，非实机。
