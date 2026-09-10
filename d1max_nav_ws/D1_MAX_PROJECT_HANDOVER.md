# D1 Max 导航项目迁移与交接文档

更新日期：2026-08-28  
当前主机用户：`dndx`  
当前主工作空间：`/home/dndx/d1max_nav_ws`  
机器人通信目录：`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2`

## 1. 交接范围

本文档覆盖以下内容：

- 新PC的Ubuntu、ROS 2和编译依赖配置。
- PC与D1 Max的有线、无线网络配置。
- Zenoh ROS 2传感器通信。
- D1 Max高层SDK到ROS 2的安全桥接和Qt控制中心。
- 双Airy96雷达、双IMU、TF和URDF。
- Faster-LIO、FAST-LIO2、SC-PGO回环、SCAN-Planner和PCT Planner。
- rosbag录制、离线回放、地图保存和迁移。
- 当前已知限制、验收步骤和故障排查。

迁移包不包含约24 GB的rosbag。bag已经移到：

```text
/home/dndx/d1max_rosbags
```

bag需要单独复制。不要把bag重新放进项目源码压缩包。

## 2. 系统架构

```text
D1 Max NX侧ROS 2
  前后Airy96点云 + 前后IMU + 静态TF
                    |
                    | Zenoh tcp://192.168.168.100:7447
                    v
PC: rmw_zenoh_cpp / ROS_DOMAIN_ID=24
                    |
                    v
dual_lidar_adapter -> Faster-LIO或FAST-LIO2
                    |
                    +-> SC-PGO回环与全局地图
                    +-> SCAN-Planner局部3D规划
                    +-> PCT离线全局3D路径

D1 Max RK/SDK服务
  有线 192.168.168.168:8081
  无线 192.168.234.1:8081
                    |
                    v
d1max_sdk_bridge行为状态机 -> ROS 2服务/状态/受控cmd_vel
                    |
                    v
Qt控制中心或后续导航控制器
```

重要边界：传感器ROS 2链路和高层SDK链路不是同一个端点。`192.168.168.100:7447`用于Zenoh ROS 2；`192.168.168.168:8081`用于有线SDK。

## 3. 当前已验证环境

| 项目 | 当前值 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| 架构 | x86_64 |
| ROS 2 | Humble |
| GCC | 11.4.0 |
| CMake | 3.22.1 |
| Python | 3.10.12 |
| Qt | 5.15.3 |
| PCL | 1.12.1 |
| Eigen | 3.4.0 |
| NumPy | 1.21.5 |
| SciPy | 1.8.0 |
| Open3D | 0.14.1 |
| RMW | `rmw_zenoh_cpp`，随项目本地前缀提供 |
| ROS Domain | 24 |
| SDK版本观测值 | `0.1.0-charging_v2` |
| SDK协议观测值 | `1.2.0` |

迁移目标机优先使用相同的Ubuntu 22.04、ROS 2 Humble、x86_64架构和用户名 `dndx`。项目中仍有硬编码的 `/home/dndx` 路径。若用户名或目录改变，必须按第6节替换路径。

SDK桥接源码当前携带的机器人SDK动态库只有：

```text
vendor/robot_sdk/lib/x86_64/librobot_sdk.so.0.1.0
```

因此当前迁移包不能直接在ARM主机上构建SDK桥。ARM目标机需要厂商对应架构的SDK库。

## 4. 迁移包内容

迁移包应包含：

- `d1max_nav_ws/src`、配置、启动脚本和现有地图。
- `d1max_ros2/local`中的本地Zenoh运行时。
- `d1max_ros2/config`、通信脚本、URDF源码、SDK桥接源码和Qt源码。
- `/home/dndx/.local/ros-humble-gtsam`，用于SC-PGO。
- `/home/dndx/go2_nav/thirdparty/livox-sdk2-install`，用于`livox_ros_driver2`链接。
- 本交接文档。

迁移包排除：

- `/home/dndx/d1max_nav_ws/bags`。
- 顶层Colcon生成目录 `build`、`install`、`log`。
- `d1max_ros2/sdk_bridge_ws`和`urdf_ws`中的生成目录。

目标机必须重新执行Colcon构建，不能直接复用带绝对路径的旧 `install/setup.bash`。

## 5. 新PC基础环境

### 5.1 安装ROS 2

按ROS 2 Humble官方方式在Ubuntu 22.04安装 `ros-humble-desktop`。确认：

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
```

输出必须为 `humble`。

### 5.2 安装基础依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-open3d \
  libeigen3-dev libpcl-dev libyaml-cpp-dev \
  libgoogle-glog-dev libceres-dev libopencv-dev libapr1-dev \
  libboost-all-dev qtbase5-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-rviz2 \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

初始化rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd /home/dndx/d1max_nav_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

`rmw_zenoh_cpp`已经以本地前缀形式放在 `d1max_ros2/local`。如果目标机能够从软件源安装，也可以安装系统包，但不要同时混用不同版本的Zenoh库。

## 6. 解压目录和路径适配

推荐保持当前目录结构：

```text
/home/dndx/d1max_nav_ws
/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2
/home/dndx/.local/ros-humble-gtsam
/home/dndx/go2_nav/thirdparty/livox-sdk2-install
```

在目标机解压：

```bash
tar --zstd -xf d1max_project_migration_20260828.tar.zst -C /home/dndx
```

如果目标机用户名不是 `dndx`，至少检查并替换以下位置：

```bash
rg -n '/home/dndx' \
  /path/to/d1max_nav_ws \
  '/path/to/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2'
```

重点文件：

- `d1max_nav_ws/start_slam.sh`
- `d1max_nav_ws/start_slam_pgo.sh`
- `d1max_nav_ws/build_slam_pgo.sh`
- `d1max_nav_ws/start_scan_planner.sh`
- `d1max_nav_ws/record_slam_bag.sh`
- `d1max_nav_ws/run_fastlio2_single_bag.sh`
- `d1max_nav_ws/build_pct_planner.sh`
- `d1max_nav_ws/src/d1max_slam/launch/mapping_pgo.launch.py`
- `d1max_nav_ws/src/d1max_pct_planner/d1max_pct_planner/plan_offline.py`

`livox_ros_driver2`迁移时推荐显式设置：

```bash
export LIVOX_SDK_ROOT=/home/dndx/go2_nav/thirdparty/livox-sdk2-install
```

## 7. PC与机器狗网络

### 7.1 有线网络

当前PC有线地址约定：

```text
PC:          192.168.168.10/24
ROS 2/NX:   192.168.168.100:7447
SDK有线:    192.168.168.168:8081
```

先找目标机网卡名：

```bash
ip -brief link
nmcli device status
```

创建或修改NetworkManager连接：

```bash
IFACE=<目标机有线网卡名>
sudo nmcli connection add type ethernet ifname "$IFACE" con-name D1max \
  ipv4.method manual ipv4.addresses 192.168.168.10/24 \
  ipv4.gateway '' ipv4.dns '' 2>/dev/null || true
sudo nmcli connection modify D1max \
  connection.interface-name "$IFACE" \
  ipv4.method manual ipv4.addresses 192.168.168.10/24 \
  ipv4.gateway '' ipv4.dns ''
sudo nmcli connection up D1max
```

`d1max_ros2/setup.sh`当前硬编码网卡名 `enx6c1ff7bc241e`。目标机网卡名不同就先修改该变量，否则脚本会拒绝继续。

网络检查：

```bash
ip route get 192.168.168.100
ping -c 2 192.168.168.100
timeout 2 bash -c '</dev/tcp/192.168.168.100/7447'
timeout 2 bash -c '</dev/tcp/192.168.168.168/8081'
```

### 7.2 无线SDK

机器狗无线SDK端点：

```text
192.168.234.1:8081
```

无线只替换SDK桥的 `robot_ip`。传感器Zenoh是否经无线可达取决于机器狗网络配置，不能默认把 `192.168.168.100`直接换成无线SDK地址。

## 8. Zenoh与ROS 2通信

加载环境：

```bash
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
```

应得到：

```text
ROS_DOMAIN_ID=24
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_ROUTER_CONFIG_URI=.../d1max_router.json5
```

启动机器人ROS 2通信：

```bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0'
./d1max_ros2/setup.sh
./d1max_ros2/start.sh
```

检查话题：

```bash
ros2 topic list
ros2 topic info -v /front_lidar
ros2 topic info -v /rear_lidar
timeout 10 ros2 topic hz /front_lidar
timeout 10 ros2 topic hz /rear_lidar
```

停止：

```bash
./d1max_ros2/stop.sh
```

不要同时启动两个监听本机7447端口的 `rmw_zenohd`。Qt联合启动文件会自行管理一个Router；如果已经执行 `start.sh`，应启动桥接和GUI单独入口，而不是再启动第二个联合Router。

## 9. 传感器输入契约

| 输入 | 话题 | 频率约值 | frame |
|---|---|---:|---|
| 前Airy96点云 | `/front_lidar` | 9.9 Hz | `rslidar_head` |
| 后Airy96点云 | `/rear_lidar` | 9.8 Hz | `rslidar_tail` |
| 前IMU | `/front_lidar/imu` | 200 Hz | 以实际消息为准 |
| 后IMU | `/rear_lidar/imu` | 200 Hz | 以实际消息为准 |

点云字段应包含：

```text
x y z intensity ring timestamp
```

双雷达适配输出：

```text
/d1max/slam/points
/d1max/slam/imu
```

融合规则：

- 前后雷达按传感器时间戳配对。
- 两颗雷达统一到 `d1max_lidar`。
- 后雷达ring增加96，融合ring范围为0到191。
- 点级相对时间保留为浮点毫秒 `time`。
- 只向同一个LIO提供一套正确变换的IMU。

传感器时钟与PC时钟曾观测到约200.47天差值。不能把传感器时间戳和PC `now()`生成的动态TF直接混用。录包、回放和TF必须保持同一时间基准。

## 10. 构建顺序

### 10.1 URDF

```bash
source /opt/ros/humble/setup.bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/urdf_ws'
rm -rf build install log
colcon build --symlink-install
```

### 10.2 SDK桥和Qt

```bash
source /opt/ros/humble/setup.bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws'
rm -rf build install log
colcon build --symlink-install \
  --packages-select d1max_sdk_bridge d1max_sdk_gui
```

### 10.3 SLAM核心

```bash
cd /home/dndx/d1max_nav_ws
rm -rf build install log
export LIVOX_SDK_ROOT=/home/dndx/go2_nav/thirdparty/livox-sdk2-install
./build.sh
```

### 10.4 Faster-LIO与SC-PGO

```bash
cd /home/dndx/d1max_nav_ws
./build_slam_pgo.sh
```

### 10.5 PCT Planner

```bash
cd /home/dndx/d1max_nav_ws
PCT_BUILD_JOBS=4 ./build_pct_planner.sh
```

### 10.6 SCAN-Planner

```bash
source /opt/ros/humble/setup.bash
cd /home/dndx/d1max_nav_ws
source install/setup.bash
colcon build --symlink-install \
  --packages-up-to d1max_scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

每个构建阶段完成后重新执行：

```bash
source /home/dndx/d1max_nav_ws/install/setup.bash
```

## 11. URDF和点云可视化

```bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0'
./d1max_ros2/visualize_lidar.sh
```

RViz检查项：

- 前后点云均存在，不应只有180度视场。
- 两颗雷达看到的同一面墙不能形成平行双墙。
- Fixed Frame和点云frame一致。
- URDF不能因为中文路径导致mesh加载失败；脚本会创建ASCII临时别名。

当前双雷达外参已经按单雷达bag和双雷达bag排查过，但迁移后仍需用静止场景复核，不应仅凭TF存在就认为外参正确。

## 12. SLAM运行

### 12.1 Faster-LIO

```bash
cd /home/dndx/d1max_nav_ws
./start_slam.sh faster_lio true
```

Faster-LIO是当前主要建图前端。

### 12.2 Faster-LIO加回环

```bash
cd /home/dndx/d1max_nav_ws
./start_slam_pgo.sh true
```

输出目录：

```text
/home/dndx/d1max_nav_ws/maps/runs/<时间戳>/
```

优化地图通常位于：

```text
sc_pgo/optimized_map.pcd
```

### 12.3 FAST-LIO2

```bash
cd /home/dndx/d1max_nav_ws
./start_fastlio2.sh true
```

不要同时运行Faster-LIO和FAST-LIO2。FAST-LIO2当前没有接入回环。

### 12.4 保存地图

```bash
cd /home/dndx/d1max_nav_ws
./save_map.sh
```

建图启动后保持机器狗静止至少2到5秒完成IMU初始化，再缓慢移动。快速转向、强振动、玻璃、大面积动态人员和时间戳错误都会降低地图质量。

## 13. rosbag

完整SLAM原始输入必须包含：

```text
/front_lidar
/rear_lidar
/front_lidar/imu
/rear_lidar/imu
/tf
/tf_static
```

推荐把新bag继续录到项目外：

```bash
cd /home/dndx/d1max_nav_ws
D1MAX_BAG_DIR=/home/dndx/d1max_rosbags ./record_slam_bag.sh slam_raw
```

数据率实测约42 MiB/s，约148 GiB/h。录制前预留空间，结束时只按一次Ctrl+C并等待 `metadata.yaml` 完成写入。

离线单雷达FAST-LIO2：

```bash
cd /home/dndx/d1max_nav_ws
./run_fastlio2_single_bag.sh \
  /home/dndx/d1max_rosbags/<bag目录> front true
```

bag回放使用隔离Domain 42，实时机器人使用Domain 24。不要让实时输入和bag输入进入同一SLAM实例。

bag单独迁移：

```bash
rsync -avh --info=progress2 \
  /home/dndx/d1max_rosbags/ \
  <目标机>:/home/dndx/d1max_rosbags/
```

## 14. 地图和规划

当前地图目录随项目迁移。当前代表性闭环地图：

```text
/home/dndx/d1max_nav_ws/maps/runs/20260825_235024/sc_pgo/optimized_map.pcd
```

当前第一层外轮廓路径：

```text
floor1_perimeter_loop.csv
floor1_perimeter_loop_z050.csv
```

`floor1_perimeter_loop_z050.csv`的全部路径点使用 `map` 坐标系固定 `z=0.50 m`。它用于当前规划展示和后续接口测试；真正执行前必须确认定位输出、机器人基座高度定义和局部避障一致。

PCT地图构建示例：

```bash
cd /home/dndx/d1max_nav_ws
./build_d1max_pct_map.sh \
  /home/dndx/d1max_nav_ws/maps/runs/<运行>/sc_pgo/optimized_map.pcd \
  /home/dndx/d1max_nav_ws/maps/<输出目录>
```

路径可视化：

```bash
source /opt/ros/humble/setup.bash
source /home/dndx/d1max_nav_ws/install/setup.bash
ros2 launch d1max_pct_planner pct_visualize.launch.py \
  pcd:=/home/dndx/d1max_nav_ws/maps/runs/20260825_235024/sc_pgo/optimized_map.pcd \
  path:=/home/dndx/d1max_nav_ws/maps/pct_20260825_235024/floor1_perimeter_loop_z050.csv
```

PCT原生GPMP轨迹平滑在保守膨胀图上曾出现切角进入障碍的情况。执行路径前必须逐段碰撞复核，不能只看规划器返回成功。

## 15. SCAN-Planner

Faster-LIO后端：

```bash
cd /home/dndx/d1max_nav_ws
./start_scan_planner.sh faster_lio true
```

FAST-LIO2后端：

```bash
./start_fastlio2.sh true
./start_scan_planner.sh fastlio2 true
```

SCAN-Planner当前只做局部3D规划和可视化，不应直接驱动机器狗。当前主要输入：

| 后端 | Odometry | 世界系点云 |
|---|---|---|
| Faster-LIO | `/Odometry` | `/cloud_registered` |
| FAST-LIO2 | `/d1max/fastlio2/odometry` | `/d1max/fastlio2/world_cloud` |

当前碰撞包络约为 `0.98 x 0.58 x 0.55 m`，需要用真实门宽、楼梯和腿部摆动继续标定。

## 16. SDK桥接和行为状态机

SDK桥是专有SDK与ROS 2之间的唯一边界。Qt程序不直接链接SDK。

安全规则：

- 启动只连接，不自动获取控制权。
- 启动不发送运动指令。
- SDK回调只代表命令收到，完成必须由约1 Hz的RobotState确认。
- 错误、Fatal、状态过期、失去控制权、超时和软件急停都会取消迁移并阻断速度。
- `/cmd_vel`只有在确认SDK持有控制权且机器人处于GENERAL模式时才会转发。
- Error/Fatal故障不能通过ROS盲目清除，必须先处理实体故障并重启桥。
- 当前机器人固件不提供可用关节状态，桥接不发布 `/joint_states`。

联合启动Router、桥接和Qt：

```bash
source /opt/ros/humble/setup.bash
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws/install/setup.bash'
ros2 launch d1max_sdk_gui sdk_control_center.launch.py
```

无线SDK：

```bash
ros2 launch d1max_sdk_gui sdk_control_center.launch.py \
  robot_ip:=192.168.234.1
```

如果Zenoh Router已经由 `d1max_ros2/start.sh`启动，则分别启动桥和GUI：

```bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0'
./d1max_ros2/start_sdk_bridge.sh
```

另一个终端：

```bash
source /opt/ros/humble/setup.bash
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws/install/setup.bash'
ros2 launch d1max_sdk_gui sdk_gui.launch.py
```

导航准备服务：

```bash
ros2 service call /d1max_sdk_bridge/prepare_navigation \
  std_srvs/srv/Trigger '{}'
ros2 topic echo /d1max_sdk_bridge/behavior_state
ros2 topic echo /d1max_sdk_bridge/ready_for_navigation
```

`prepare_navigation`依次确认：

```text
TAKE_CONTROL -> STAND -> GENERAL_MODE -> SET_SPEED
```

只有 `/d1max_sdk_bridge/ready_for_navigation=true` 才允许后续控制器提交速度。

## 17. 机器狗侧环境要求

PC迁移不要求修改机器人固件。机器人侧应保持：

- NX侧双Airy96驱动和ROS 2发布节点正常。
- NX侧Zenoh端点 `192.168.168.100:7447`正常。
- RK/SDK服务的有线端点 `192.168.168.168:8081`正常。
- 无线SDK端点需要时为 `192.168.234.1:8081`。
- 前后雷达静态外参保持与当前TF一致。
- 机器人供电、电池、急停和物理安全状态正常。

当前记录只确认SDK版本 `0.1.0-charging_v2`和协议 `1.2.0`，没有在本交接文档中写入未经再次核验的RK/NX固件编号。迁移前后不要顺带升级固件。若必须升级，先通过厂商官方工具导出RK、NX和运动控制固件版本，并确认SDK、ROS 2话题和外参兼容性。

## 18. 目标机只读验收

按以下顺序验收，不要一开始就获取机器人控制权。

### 18.1 网络

```bash
ping -c 2 192.168.168.100
timeout 2 bash -c '</dev/tcp/192.168.168.100/7447'
timeout 2 bash -c '</dev/tcp/192.168.168.168/8081'
```

### 18.2 ROS 2

```bash
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
ros2 topic list
ros2 topic info /front_lidar
ros2 topic info /rear_lidar
```

### 18.3 数据率

```bash
timeout 10 ros2 topic hz /front_lidar
timeout 10 ros2 topic hz /rear_lidar
timeout 10 ros2 topic hz /front_lidar/imu
```

### 18.4 TF

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_link rslidar_head
timeout 5 ros2 run tf2_ros tf2_echo base_link rslidar_tail
```

### 18.5 SDK只读连接

启动桥后检查：

```bash
ros2 topic echo /d1max_sdk_bridge/connection_state_text
ros2 topic echo /d1max_sdk_bridge/robot_state
```

此阶段不要调用 `take_control`、`prepare_navigation`或发送 `/cmd_vel`。

### 18.6 静止SLAM

机器人保持静止，启动Faster-LIO，确认IMU初始化、点云和里程计稳定，再低速移动数米。若静止状态里程计明显漂移，先检查时间、IMU单位和外参，不要继续调规划器。

## 19. 常见故障

### 19.1 ROS 2无机器狗话题

检查：

- PC地址是否为 `192.168.168.10/24`。
- `RMW_IMPLEMENTATION`是否为 `rmw_zenoh_cpp`。
- `ROS_DOMAIN_ID`是否为24。
- `rmw_zenohd`是否运行且7447端口没有被第二个Router占用。
- `d1max_router.json5`是否仍连接 `tcp/192.168.168.100:7447`。

### 19.2 SDK连接失败但点云正常

点云和SDK端点不同。检查 `192.168.168.168:8081`，不要只检查 `.100:7447`。

### 19.3 只有半圈点云或双墙

检查 `/front_lidar`、`/rear_lidar`是否都在发布，检查静态外参和两雷达时间配对。单雷达建图正常而双雷达形成平行墙时，优先判定外参错误，不要先调LIO参数。

### 19.4 RViz黑屏或无响应

先停止旧RViz和旧SLAM实例，确认Fixed Frame存在，降低点云显示历史长度，并检查GPU/OpenGL。不要同时开启多个高密度点云和多个RViz。

### 19.5 地图漂移或墙壁重叠

按顺序检查：

1. 单雷达建图是否稳定。
2. 两雷达外参是否正确。
3. 点级时间和帧配对是否正确。
4. IMU单位和坐标方向是否正确。
5. 传感器时钟是否被错误混入PC时间。
6. 最后才调整LIO、回环和滤波参数。

### 19.6 rosbag无法回放

确认bag目录存在 `metadata.yaml`，使用Domain 42隔离回放，显式传入bag绝对路径。bag已移出工作空间，脚本的 `latest` 默认搜索空的 `d1max_nav_ws/bags`，因此迁移后优先传绝对路径。

## 20. 当前功能状态

已具备：

- 双Airy96和IMU输入适配。
- Faster-LIO建图。
- FAST-LIO2可选前端。
- Faster-LIO加SC-PGO回环。
- 3D PCD地图保存。
- PCT离线规划和RViz可视化。
- SCAN-Planner局部3D规划集成。
- SDK到ROS 2行为状态机桥接。
- Qt安全控制中心。
- 完整原始SLAM rosbag录制脚本。

尚未完成：

- 基于已有3D地图的可靠全局重定位。
- 规划路径到机器狗运动控制的闭环跟踪。
- 实时局部动态避障和速度仲裁。
- 跨楼层拓扑、楼梯和电梯状态机。
- RK/NX/运动控制固件版本的正式配置清单。
- 长时间实机自主导航安全验证。

因此当前系统不能被描述为“已经可以无人值守自主导航”。

## 21. 安全要求

- 首次迁移验收必须架空或留有实体急停人员。
- SDK桥默认无控制权是安全设计，不得为了方便改成启动自动获取控制权。
- 服务调用返回成功不代表动作完成，必须等待RobotState确认。
- `/cmd_vel`必须经过状态机、控制权、模式、故障和超时门控。
- 规划成功不代表路径可执行，必须进行机器人包络碰撞检查和局部避障。
- 楼梯、玻璃、窄门和人员密集区域必须单独验收。
- 不要在迁移过程中升级机器狗固件。

## 22. 交接验收记录

建议接收人在首次部署后填写：

| 项目 | 结果 | 备注 |
|---|---|---|
| 目标机系统和架构 |  |  |
| 有线网卡和PC地址 |  |  |
| `.100:7447`可达 |  |  |
| `.168:8081`可达 |  |  |
| 前后点云频率 |  |  |
| IMU频率 |  |  |
| 前后雷达TF |  |  |
| SDK只读状态 |  |  |
| Faster-LIO静止稳定性 |  |  |
| 单雷达/双雷达地图一致性 |  |  |
| 地图保存 |  |  |
| PCT路径可视化 |  |  |
| Qt状态机日志 |  |  |
| 实体急停测试 |  |  |

## 23. 关键入口索引

| 功能 | 入口 |
|---|---|
| Zenoh环境 | `d1max_ros2/d1max_ros2_env.sh` |
| 启动通信 | `d1max_ros2/start.sh` |
| 停止通信 | `d1max_ros2/stop.sh` |
| URDF/雷达RViz | `d1max_ros2/visualize_lidar.sh` |
| SDK桥 | `d1max_ros2/start_sdk_bridge.sh` |
| Qt控制中心 | `d1max_sdk_gui/sdk_control_center.launch.py` |
| 核心构建 | `d1max_nav_ws/build.sh` |
| Faster-LIO | `d1max_nav_ws/start_slam.sh` |
| FAST-LIO2 | `d1max_nav_ws/start_fastlio2.sh` |
| 回环建图 | `d1max_nav_ws/start_slam_pgo.sh` |
| 保存地图 | `d1max_nav_ws/save_map.sh` |
| 录制bag | `d1max_nav_ws/record_slam_bag.sh` |
| PCT构建 | `d1max_nav_ws/build_pct_planner.sh` |
| SCAN-Planner | `d1max_nav_ws/start_scan_planner.sh` |
| 当前算法汇总 | `d1max_nav_ws/CURRENT_ALGORITHM_CONFIGURATION.md` |

