# D1 Max · 轻量状态与感知工作台

当前版本 **0.7.4**。左侧显示 8 月 25 日单层候选 PCD（497,844 点），不加载历史多层点云；右侧实时感知和底部面板保持原布局。来源与限制见 [SINGLE_FLOOR_MAPS.md](SINGLE_FLOOR_MAPS.md)。
已取消控制权申请、站立、趴下、模式切换、方向控制、准备运动、异常复位和急停解除。
**没有 URDF / STL 渲染，没有机器人模型页，也不启动模型资源服务。** 原始模型文件和原始 PCD 均保留，不做删除或覆盖。

## 启动

在本目录运行：

```bash
bash scripts/start_live_monitor.sh
```

Foxglove 连接 `ws://127.0.0.1:8769`，选择 **D1 Max · 实机工作台**。
新环境导入 `layouts/D1Max-Monitor.json`。
旧入口 `start_live_controls.sh` 和 `start_live_view.sh` 现在都转到此监控启动器，不能再启用运动控制。
旧 `start_sdk_console.sh`、`start_gateway.sh` 已明确拒绝启动；历史控制源码留作开发记录，不在本工作台运行。

实机使用 ROS Humble / Domain 24 / `rmw_zenoh_cpp`，不用 Fast DDS。
Zenoh 直连 `192.168.168.100:7447`，SDK 状态接收连接 `192.168.168.168:8081`。
启动不申请控制权、不发送机器人动作；发现已有 SDK 发布者或 8769 端口占用时拒绝重复启动，不自动杀进程。

## 布局

- 上方：左侧 PCD 窗口、右侧实时双雷达/前后图像，各占一半；PCD 数据开关不改变布局。
- 下方一排：双相机、两组速度曲线、一个双电池小面板、状态图标及单向软件急停。
- 双电池在同一个原生分组中同时显示为两条横向电量条，不再是两个大圆盘。没有新增可切换的业务页面。

只有一个业务页面；电池分组仅含一个固定页且两块电池同时可见，没有说明卡片、原始 JSON、操作面板或 2D 地图。
图标从左上到右下依次为：SDK 新鲜度、姿态枚举、控制来源、软件急停、硬件急停、未接入地图定位。
图标悬停可查看完整含义；问号为未知，勾为已知状态，红色停止符号为急停已触发，不仅依赖颜色。
姿态图标仅为枚举示意，不是机器人模型或关节姿态。

`config/instruments.json` 统一设置单屏分割比例、颜色、速度显示范围与历史窗口。范围是图表量程，不是机器人运动能力或安全限速。
两块电池统一使用原生 `red-yellow-green` 电量色标：低电量红、中电量黄、高电量绿，范围 0–100%，保留刻度和百分比；不是用蓝/金区分电池。色标为连续显示范围，不声称对应厂商低电量告警阈值。
电池分组默认占布局面积 4.8%（此前两个圆盘占 15.84%），减少约 70%；上方保留左右双视图。底部高度经原生桌面检查，避免横条刻度被裁切。使用原生横向 Gauge + 单页分组，不引入新的图表库或机器人订阅。
原生 Gauge / Plot 的字段、配色和窗口可在 Foxglove 面板设置调整；没有主页面 JSON 编辑器。
`config/panel.json` 设置订阅与过期阈值，`config/scripts/instruments.ts` 在 Foxglove 客户端清空过期仪表，不发布 ROS。
SDK 中断而监控心跳仍到达时，超过 2.5 秒清空 Gauge 读数；整个 ROS/网络断开时原生 Gauge 可能保留最后值，必须以带问号的 SDK 断流图标和原生连接提示判定其无效，不能视为实时值。
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

**当前没有地图定位或跨楼层导航。**
当前单层 PCD 位于 `d1max_floor1_map`，实时感知位于 `d1max_lidar`，分别显示；不伪造 map→odom TF，不表示机器人已经定位到建筑中。
以后接入真实定位、楼层标识、跨层拓扑和全局路径后，再在统一坐标系中叠加实际位姿、实时点云和路线。
本次没有生成导航目标，也没有启动定位或导航。

## 状态与软件急停

新 SDK 接收器：`sdk_monitor_bridge`。只订阅回放检测和 SDK 数据，唯一 ROS 服务为：

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
npm run package
npm run local-install
```

`scripts/verify_monitor.py` 只订阅生产状态和点云、检查 ROS 图和 WebSocket 暴露能力，绝不调用急停。
`scripts/verify-ui.mjs` 验证两种主题、三种尺寸和五种状态的独立 UI 预览（30 项，先运行 npm run preview）。
C++ 纯状态测试：`../sdk_bridge_ws/src/d1max_sdk_bridge/src/test_monitor_estop.cpp`。
结果与已知限制见 `VERIFICATION.md`。

现场布局导出：`artifacts/monitor-layout-installed.json`；升级前备份：`artifacts/layout-before-monitor.json`。
仅清理本次生成的旧布局菜单备份项，其他布局和 maps 工作区不受影响。
