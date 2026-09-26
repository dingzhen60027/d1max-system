# D1 Max：地图与定位诊断入口（不含完整导航）

完整 Nav2 导航现在请使用 [README.md](README.md) 中的 `start_navigation.sh`。
本文件保留之前地图/定位诊断入口的说明，不应当成完整导航的启动方法。

本模块是 2D 导航的第一阶段：Nav2 `map_server`、生命周期管理、RViz2 初值入口。
定位继续使用原来的 Faster-LIO + PCD 匹配 + 受控高频输出；不再启动 AMCL。
**尚未启用规划、避障或运动控制，不代表整套导航已完成实机验收。**

## 使用

```bash
cd /home/dndx/d1max_nav_ws
./start_nav2_localization_test.sh start --mode offline
./start_nav2_localization_test.sh status
./start_nav2_localization_test.sh stop
```

离线模式只读 Web 当前选中的完整地图版本，不需要 Web 运行。RViz 自动缩放到全图；
地图显示项标记 `OFFLINE`，画面状态标记 `OFFLINE - MAP ONLY / MOTION DISABLED`。
不伪造 TF、机器人位姿或轨迹，不保留待提交初值。
使用独立回环地址 Zenoh 路由器 `127.0.0.1:7460`，无机器人上行、无发现广播。

重新连接机器人后，先在 Web 连接数据链路，确认雷达、IMU、MC 状态正常，然后：

```bash
./start_nav2_localization_test.sh restart --mode live
```

实机模式复用现有 `127.0.0.1:7448` Zenoh 链路。如果尚未定位，通过 Web 原有受托管 API
启动所选地图对应的定位；如果正在同一版本定位则复用，不擅自终止或替换别的定位会话。
静止完成 LIO 初始化，保持机头前向；RViz 点击 **2D Pose Estimate**，在机器人所在位置
按下鼠标并沿实际机头朝向拖出箭头。箭头只是匹配初值，不会让机器狗移动。

初值经过地图版本、会话、数据时效与姿态检查，通过 Web 的同一个入口交给匹配器。
发出请求、初值被接收、连续匹配通过是不同状态；不能把鼠标箭头当作已经定位成功。
初值接口状态见 `/d1max/navigation/initial_pose_status` 或服务日志；最终定位看
`/d1max/localization/status` 的 `localized`，同时核对点云与墙体重合、静止漂移、慢速运动连续性。

画面状态为只读状态指示：离线橙色、待初值青色、定位确认后绿色、数据超时或异常红色。
数据超过 1.5 秒不更新即标为过期；箭头/轨迹可能保留上一帧，不能单凭残留画面判断正常。
绿色只代表当前定位输出，不代表外参/时间对齐已验收；所有状态均注明 `MOTION DISABLED`。

## 显示与坐标

| 对象 | 输入 | 说明 |
| --- | --- | --- |
| 2D 栅格 | `/d1max/navigation/map` | 当前版本 `map.yaml`，保留分辨率与原点 |
| 实时点云 | `/d1max/localization/lio/deskewed` | 前后雷达原定位链路；青色、不累积历史帧 |
| 初值预览 | `/d1max/localization/scan_initial_preview` | 橙色，仅预览，不代表匹配通过 |
| 机身箭头 | `/d1max/localization/odometry/global` | `d1max_loc_base_link`，只保留一个箭头 |
| 轨迹 | `/d1max/localization/trajectory` | 细紫线，现有雷达 tracking 点轨迹 |
| 参考 PCD | `/d1max/localization/map_cloud` | 默认关闭，避免遮住 2D 地图 |

固定坐标系 `d1max_loc_map`；沿用原定位唯一 TF 发布者的
`d1max_loc_map → d1max_loc_odom → d1max_loc_base_link → d1max_loc_tracking`。
`/pose` 目前是 tracking 点而非机身；这里不拿它画机头，避免约 90° 方向误解。
不增添假的 `map` 恒等变换，不把地面强制归零，不修改雷达/IMU 外参。

初值高度由 `config/localization_test.yaml` 的 `initial_pose_z` 配置，当前为原地图坐标中的
机身高度种子 `0.0`，不是测出的地面。换楼层或地图时必须核对这一项。
RViz 箭头只提供 XY 与 yaw，协方差不用于调整现有匹配器搜索范围。

## 生命周期与边界

- 独立模块 `d1max_navigation`；不修改建图、Foxglove 布局和原始地图。
- 固定使用 `rmw_zenoh_cpp`、Domain 24；禁止 DDS 替换。
- systemd 单一服务 `d1max-nav2-localization-test.service`，重复启动拒绝，关闭 RViz 即清理
  本测试的地图节点、生命周期节点、初值桥和离线路由器。
- `stop` **不关闭**共享 Web、SDK 或现有定位。它们仍由 Web 原来的启动/停止入口管理。
- 实机服务绑定定位与监测服务；上游被停止时同步关闭本测试，避免把旧位姿留作有效状态。
- 配置快照保存在 `log/nav2_localization_test/`；日志用
  `journalctl --user -u d1max-nav2-localization-test.service -n 80 --no-pager` 查看。
- 外参与时间对齐验收标记不变。`navigation_ready=false` 时不能解除运动保护。
- 当前 2D 地图的空白曾被设为自由区，不代表已测到地面；规划/控制接入前还须核对未知区、
  障碍、机器人 footprint、障碍高度、膨胀层、速度限幅、急停和掉线保护。

## 构建与测试

加载项目 ROS 环境后运行：

```bash
colcon build --packages-select d1max_navigation --symlink-install
colcon test --packages-select d1max_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/d1max_navigation --verbose
```

离线验收只证明地图、生命周期、RViz 和接口防护可用；实际定位精度和运动中稳定性需要
连接机器人后完成，不用旧 rosbag 的结果冒充实机测试结果。

### 2026-09-18 离线检查记录

- 构建成功；导航模块 82 项测试、Web 定位接口 11 项测试通过。
- Nav2 地图服务激活，生命周期管理器确认 bond 和 active 状态。
- 实收地图：`grid-0af429985e454a9e99c8aaef`，2130×1246，0.05 m/格，
  原点 `[-85.95, -14, 0]`；50,047 个占据格与 2,603,933 个自由格，和版本记录一致。
- RViz 已启动并订阅地图，状态 Marker 为 `OFFLINE - MAP ONLY / MOTION DISABLED`。
- 重复启动被拒绝；停止后测试 cgroup 清空、离线路由端口释放；实机链路缺失时启动被拒绝。
- Web、SDK 监测、实机定位服务保持未启动；没有控制速度或导航目标话题。
- 实机初值匹配、静止漂移、行走稳定性仍未验收；不要将上述离线检查当作可自主导航证明。
