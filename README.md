# D1 Max System

D1 Max 当前系统的源码与配置快照：建图、地图处理 Web、SDK/ROS 2 接入和 Foxglove 实机状态工作台。

## 当前使用边界

- ROS 2 Humble、Domain 24、`rmw_zenoh_cpp`；不使用 Fast DDS。
- Foxglove 为单屏：左侧单层候选 PCD，右侧实时双雷达与前后图像，底部相机、速度曲线、紧凑双电池、状态图标及单向软件急停。
- 实机监控入口不申请控制权、不发送运动、姿态切换或急停解除命令，不加载 URDF/STL。
- 软件急停不能替代机身急停。历史控制/GUI 源码仅作历史保留，不属于当前监控启动链路。
- 当前是单楼层建图导航调试准备；地图显示不等于定位成功，没有伪造静态地图到实时雷达的 TF，也没有宣称全局导航已完成。

## 目录

| 目录 | 内容 |
| --- | --- |
| `d1max_nav_ws/` | Faster-LIO、FAST-LIO2、SC-PGO，传感器适配、建图保存及规划相关源码 |
| `d1max_ros2/map_manager/` | 3D 地图管理 Web、模块化 YAML 点云处理、结果归档与删除 |
| `d1max_ros2/sdk_bridge_ws/` | SDK 到 ROS 2 的状态接收、单向急停接入与测试 |
| `d1max_ros2/foxglove_d1max/` | Foxglove 扩展、布局、相机投影、PCD 发布、连接与仪表配置 |
| `d1max_ros2/config/` | ROS/Zenoh 通信配置 |

地图处理流程入口：`d1max_ros2/map_manager/config/pipelines/`。每次处理的输入、输出、模块顺序与参数可由一份 YAML 表达，输出另存，不覆盖原 PCD。

Foxglove 的 `layouts/D1Max-Current.json` 是本次提交时读取的现场布局；`layouts/D1Max-Monitor.json` 是构建模板，两者用途不同。以后在 Foxglove 界面修改布局，需要重新导出才会进入 Git。

## 数据与外部依赖

仓库不包含 PCD/PLY、rosbag、处理结果、构建缓存、浏览器配置、凭据、厂商 SDK 二进制或厂商模型/资料包。配置保留本机路径用于记录当前状态，不代表数据已上传。

SDK 编译需要另行从厂商提供的资料包安装头文件和对应架构动态库到 `d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/vendor/robot_sdk/`，该目录不纳入版本控制。

导航依赖源码的原有许可证和 NOTICE 保留；不对第三方代码重授许可证。来源记录见 `SOURCE_SNAPSHOT.md`。

## 使用

本仓库是从当前运行工作区整理出的独立提交目录，**没有移动或替换正在运行的原目录**。现有脚本仍含本机绝对路径，新机器部署前需核对路径、ROS/SDK 依赖和网络配置，不能直接启动连接未知机器人。

详见各模块 README，尤其：

- `d1max_ros2/foxglove_d1max/README.md`
- `d1max_ros2/map_manager/README.md`
- `d1max_nav_ws/README.md`

本次整理、提交和推送不会启动建图、导航或机器人动作。
