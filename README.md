# D1 Max System

D1 Max 系统源码与配置：建图、PCD 处理、2D 地图准备、单楼层定位、SDK 数据接入和 Foxglove 监控。
当前整理基线：2026-09-12。主分支 `main`，统一发布仓库 `dingzhen60027/d1max-system`。

## 当前使用边界

- ROS 2 Humble、Domain 24、`rmw_zenoh_cpp`；不使用 Fast DDS。
- Foxglove 为单屏：左侧单层候选 PCD，右侧实时双雷达与前后图像，底部相机、速度曲线、紧凑双电池、Connect / Disconnect、Web 启停及单向软件急停。
- 本机管理器登录后空闲等待按钮，管理通信和 Web 的专属进程组；Web 单独管理定位会话。重复启动、未知进程占用和未完成的停止不能当作成功。
- SDK 速度统一使用 `OnMcData`；回调轻量复制，工作线程校验及发布，显示实际接收频率，不用 1 Hz 状态速度冒充高频流。
- 当前配置允许 APP 释放后的单次 SDK 接管申请：必须收到可接管事件，再检查回执与归属；不主动抢占 APP。接管不是数据就绪，也不是运动授权。
- 不发送运动、姿态切换、模式切换或急停解除命令，不加载 URDF/STL。
- 软件急停不能替代机身急停。历史控制/GUI 源码仅作历史保留，不属于当前监控启动链路。
- 当前默认定位为 Faster-LIO + PCD 匹配，附有界 IMU 高频传播和 robot_localization 全局滤波；目标输出 50 Hz，实际速率和有效性另行检查。
- 尚未完成实机高速鲁棒性、外参和时间对齐验收；`navigation_ready=false`。没有接入 Nav2 行走或跨楼层导航。已知问题见 [待办与验收边界](docs/KNOWN_ISSUES.md)。

## 目录

| 目录 | 内容 |
| --- | --- |
| `d1max_nav_ws/` | Faster-LIO、FAST-LIO2、SC-PGO，传感器适配、建图及规划源码 |
| `d1max_nav_ws/src/d1max_localization/` | 局部里程计、高频传播、PCD 匹配、全局融合与可信输出 |
| `d1max_ros2/map_manager/` | 2D / 3D 入口首页、点云与模块化处理、PCD 转栅格、2D 修整与版本归档 |
| `d1max_ros2/sdk_bridge_ws/` | SDK 到 ROS 2 的状态接收、单向急停接入与测试 |
| `d1max_ros2/foxglove_d1max/` | Foxglove 扩展、布局、相机投影、PCD 发布、连接与仪表配置 |
| `d1max_ros2/config/` | ROS/Zenoh 通信配置 |
| `docs/` | 模块入口、当前边界与待办 |
| `tools/` | 安全的运行工作区 → 发布仓库同步工具 |

地图处理流程入口：`d1max_ros2/map_manager/config/pipelines/`。每次处理的输入、输出、模块顺序与参数可由一份 YAML 表达，输出另存，不覆盖原 PCD。

Web 首页可选择 2D 导航或 3D 地图。2D 参数位于地图 Web 的 `config/navigation2d.yaml`，生成参数与修整笔画保存在各版本 `pipeline.yaml`，并保留 `localization.pcd` 用于匹配。Web 负责选图、启停定位和箭头初值，Foxglove 负责显示；选图和定位不会让机器狗运动。

Foxglove 的 `layouts/D1Max-Current.json` 是已保存的布局导出，`layouts/D1Max-Monitor.json` 是构建模板。本次没有重新导出或更改桌面/云端布局；账号中的布局不会自动跟随 Git 同步。历史 JSON 留在源码中不等于重新导入多个布局。

## 数据与外部依赖

仓库不包含 PCD/PLY、rosbag、处理结果、构建缓存、浏览器配置、凭据、厂商 SDK 二进制或厂商模型/资料包。配置保留本机路径用于记录当前状态，不代表数据已上传。

SDK 编译需要另行从厂商提供的资料包安装头文件和对应架构动态库到 `d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/vendor/robot_sdk/`，该目录不纳入版本控制。

导航依赖源码的原有许可证和 NOTICE 保留；不对第三方代码重授许可证。来源记录见 `SOURCE_SNAPSHOT.md`。

## 使用

本仓库是从当前运行工作区整理出的独立提交目录，**没有移动或替换正在运行的原目录**。现有脚本仍含本机绝对路径，新机器部署前需核对路径、ROS/SDK 依赖和网络配置，不能直接启动连接未知机器人。

先看 [项目入口与职责](docs/PROJECT_GUIDE.md)，再进入对应模块：

- `d1max_ros2/foxglove_d1max/README.md`
- `d1max_ros2/map_manager/README.md`
- `d1max_nav_ws/README.md`
- [当前定位架构](d1max_nav_ws/src/d1max_localization/NAVIGATION_ESTIMATION.md)

整理与发布流程见 [tools/README.md](tools/README.md)；来源见 [SOURCE_SNAPSHOT.md](SOURCE_SNAPSHOT.md)，检查结果见 [VERIFICATION.md](VERIFICATION.md)。

本次整理、提交和推送不会启动建图、导航或机器人动作。
