# 项目入口与职责

2026-09-12。此目录说明当前系统，不把各模块按日期保留的历史验收记录当作实时状态。

## 三个目录的关系

| 目录 | 用途 | 编辑与提交规则 |
| --- | --- | --- |
| `/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2` | 正在使用的 Web、SDK 桥和 Foxglove 工作区 | 运行源码；不上传厂商资料包 |
| `/home/dndx/d1max_nav_ws` | 正在使用的 ROS 建图、定位和规划工作区 | 保留原 Git 分支与本地变更，不重写其历史 |
| `/home/dndx/d1max-system` | 统一源码发布仓库 | 从前两处同步、检查后提交和 push；不是自动部署目录 |

本次没有移动运行目录、重建环境、替换运行产物或重启服务。对发布仓库的修改不会自动部署到机器人。
两边以后只能明确选择一处编辑，再同步；不要分别改同一个文件后直接覆盖。

## 运行入口

| 功能 | 正式入口 / 文件 | 边界 |
| --- | --- | --- |
| 通信启停 | Foxglove Connect / Disconnect；`foxglove_d1max/manager/` | 管理受管 monitor；不是运动控制 |
| Web 启停 | Foxglove Web 按钮；`d1max-web-managed.service` | Web 独立生命周期；停止清理所属任务 |
| 本机管理器安装 | `foxglove_d1max/scripts/install-session-manager.py` | 部署操作，不是日常连接按钮；重装/重启可能联动停止服务 |
| Web 备用终端入口 | 根目录 `start_d1max_map_manager.sh` | 确认受管 Web 已停止再用，避免重复实例 |
| 建图 | Web 3D 建图；`d1max_nav_ws/start_slam*.sh` | 保存原始结果，回放/实机环境需分别核对 |
| PCD 处理 | Web 3D → 原始点云 → 参数与处理 | 原图保留；处理结果另存、可追溯 |
| 2D 地图准备 | Web 2D → 生成 / 修整 / 选用 | 同版保留定位 PCD；栅格自由区不是已验收的可通行区 |
| 定位 | Web 2D → 单楼层定位调试 | 选定版本 → 新会话 → 等待有效局部数据 → 拖箭头初值 |
| 查看定位 | Foxglove 唯一工作台左侧 `PCD · LOCALIZATION` | 静态地图、实时扫描、位姿和轨迹；旧画面不代表数据新鲜 |

上述相对 `foxglove_d1max/` 路径位于 `d1max_ros2/` 下。根目录 `start_d1max_communication.sh` 是旧的手动通信诊断入口，不能与受管实机连接链路混用。
历史控制网关、URDF、姿态/运动入口不属于当前正式监控链路，不因保留源码而获得执行授权。

## 本机端口

| 地址 | 提供什么 |
| --- | --- |
| `http://127.0.0.1:8766` | 地图 Web：2D / 3D / 定位任务 |
| `ws://127.0.0.1:8769` | Foxglove ROS 数据桥；不是机器狗 IP，不是独立定位算法 |
| `http://127.0.0.1:8771` | 带本机凭据的固定启停接口；不是公开管理 API |
| `tcp/127.0.0.1:7448` | 受管实机 Zenoh 汇聚路由；节点不各自重复连接机器人 |

ROS 2 Humble / Domain 24 / `rmw_zenoh_cpp`，不改用 Fast DDS。离线回放使用独立配置，不直接套用实机端口和可控制入口。

## 配置索引

| 配置（仓库相对路径） | 负责范围 |
| --- | --- |
| `d1max_ros2/map_manager/config/pipelines/*.yaml` | 原始 PCD → 模块顺序/参数 → 独立处理结果 |
| `d1max_ros2/map_manager/config/navigation2d.yaml` | PCD 到栅格的过滤、投影和分辨率 |
| `d1max_nav_ws/src/d1max_localization/config/localization.yaml` | 当前定位后端、输入约定、传播、匹配、全局滤波、输出门控 |
| `d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/config/monitor.yaml` | MC 接收、重试、会话接管和遥测配置 |
| `d1max_ros2/foxglove_d1max/config/manager.json` | 本机管理器固定进程/端口/超时 |
| `d1max_ros2/foxglove_d1max/config/zenoh-*-live.json5` 与 `zenoh-live.json5` | 实机路由/客户端通信 |
| `d1max_ros2/foxglove_d1max/layouts/` | 已导出布局和模板，不是云端实时备份 |

Web 运行时会保存地图处理 recipe、定位会话配置及状态到本机数据目录；这些产物不进入 Git。
`manager.local.json` 是每次安装生成的本机能力凭据，不能复制进仓库或分享。

## 定位模块边界

局部 Faster-LIO 已经融合雷达和 IMU。新增传播器从每帧校正后的惯性状态，使用其后的 IMU 做有界高频预测；PCD 匹配提供较慢的全局约束。全局 EKF 和输出层负责融合、时间对齐、失效保护及唯一 TF 发布。

- 高频局部输出：`/d1max/localization/odometry/local`，机身参考，连续 odom 系。
- 高频全局输出：`/d1max/localization/odometry/global`，同一机身参考，map 系。
- SDK MC 是实际速度遥测，当前默认定位不积分 MC 速度。
- 配置 50 Hz 不是新激光观测有 50 Hz；数据缺失时应撤销有效输出。

接口和坐标公式以 [NAVIGATION_ESTIMATION.md](../d1max_nav_ws/src/d1max_localization/NAVIGATION_ESTIMATION.md) 为准。
旧 `legacy_ekf` 是历史回归后端，不应当作当前默认路线。待修复问题与实机验收见 [KNOWN_ISSUES.md](KNOWN_ISSUES.md)。
