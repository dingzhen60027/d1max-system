# D1 Max 新电脑部署

这份说明用于把当前导航主线迁到另一台电脑，先完成源码、依赖和构建准备，再进入版本封存及隔离验收。仓库不是自包含运行镜像，不能只 clone 后启动旧 release。

当前验证环境为 Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10、x86_64 和 Zenoh。其他系统或 CPU 架构需重新验证；所有 native 组件在目标机编译，不复制开发机 ELF。Isaac Sim 暂未接入，本文准备的是已有导航框架，不声称仿真已经跑通。

## 获取源码和设置路径

```bash
git clone https://github.com/dingzhen60027/d1max-system.git
cd d1max-system
git rev-parse HEAD
source tools/deployment-env.sh
python3 tools/deployment_preflight.py --scope source
```

路径模板按当前 checkout 自动设置 NAV、APP、MAPS 和 PCT 根目录，保留用户显式覆盖值。它只设置路径，不加载 ROS 或实机连接配置。重新进入终端时再次 source；不要把旧电脑的 `/home/dndx/...` 写入新电脑配置。

源码可以从本仓库直接开发。若继续使用另一个开发目录，明确选择一处编辑，再用 [源码同步工具](../tools/README.md) 发布，避免两边独立修改同一文件后覆盖。

## 目标机依赖

| 部分 | 需要提供 |
| --- | --- |
| ROS 导航 | Humble、colcon、CMake/make、C++17、Python 3.10 开发头文件、BT.CPP v3、Nav2 util/lifecycle/collision monitor、robot_localization、消息/TF/RViz 依赖 |
| 定位和点云 | PCL、Eigen、fast_gicp 的 Humble 安装前缀、Livox SDK2 的目标机头文件和库 |
| 原生算法 | OpenSSL、nlohmann-json、Qt5、Boost、OpenCV、TBB、glog、gflags、yaml-cpp 的开发包及 binutils/readelf；PCT 的 GTSAM 4.1.1、OSQP、pybind11 等源码已随仓库保留，构建时重编 |
| Python 导航 | 系统 Python 3.10 下的 NumPy、SciPy、PyYAML、Open3D；不能用 Isaac 内嵌 Python 替代 ROS Python |
| 通信 | 对应 Humble 的 `rmw_zenoh_cpp` 与 Zenoh vendor 库；不默默降级为 DDS |
| SDK | 厂商同版本头文件和当前 CPU 架构的 `librobot_sdk.so`，合法取得，不进入 Git |
| Web | Node/npm 与锁文件、独立 Python venv；后台依赖见 `map_manager/requirements.txt` |

ROS 依赖先用 package.xml 检查，不直接执行无人审核的安装：

```bash
rosdep check --from-paths d1max_nav_ws/src --ignore-src --rosdistro humble
```

个别厂商或上游依赖可能没有 rosdep key，须按其来源说明单独配置。`fast_gicp` 不在本仓库；外部 ROS overlay 的 `local_setup.bash` 在预检前显式 source，构建工具也接受 `--dependency-prefix`。Livox SDK2 与厂商机器人 SDK 是两项不同材料。

设置实际前缀后检查：

```bash
export D1MAX_RMW_PREFIX=/absolute/path/to/humble-zenoh-prefix
export LIVOX_SDK2_PREFIX=/absolute/path/to/livox-sdk2-install
source /opt/ros/humble/setup.bash
# 如 fast_gicp 在单独 overlay，先 source 该 overlay 的 local_setup.bash。
/usr/bin/python3 tools/deployment_preflight.py --scope nav
```

`--scope source/nav/sdk/pct/web/all` 分别检查对应范围。`source` 通过只证明文件齐全；预检不会导入 ROS、连接机器人、安装依赖或校验物理能力。动态库的完整加载闭包由后续封存入口检查，不能以“文件存在”替代运行验收。

SDK 的归一化目录为：

```text
robot_sdk/
  include/                     厂商头文件，保留原有子目录
  lib/x86_64/librobot_sdk.so    对应目标 CPU 的厂商库及必要版本链接
```

构建时用 `--sdk-vendor-root /absolute/path/to/robot_sdk`。现有 SDK CMake 在配置阶段要求厂商库，即使只想编译 SDK mock，也不能假定纯源码 clone 已满足要求。

## 源码构建

[build-source.sh](../tools/build-source.sh) 默认 dry run，要求新输出目录。scope 可选 `nav`、`sdk`、`pct`、`web`、`all`；仅 `--apply` 才复制筛选后的源码并构建。默认并行度 2，可显式降低，防止内存不足。

```bash
# 先打印计划；output 必须不存在，且在源码与依赖目录之外。
bash tools/build-source.sh --scope nav \
  --output /absolute/path/to/build-20261004 \
  --rmw-prefix "$D1MAX_RMW_PREFIX" \
  --livox-prefix "$LIVOX_SDK2_PREFIX" --jobs 2
```

核对计划后，在原命令追加 `--apply`。完整源码构建（包括 SDK、Web）还需提供厂商 SDK：

如果 fast_gicp 仅存在于外部 overlay，下面的 nav/all 构建命令都追加
`--dependency-prefix /absolute/path/to/fast_gicp-overlay`。预检前 source 只影响预检；
构建工具会清理继承的 AMENT/CMake 路径，不能依赖先前终端 source 的环境。

```bash
bash tools/build-source.sh --scope all \
  --output /absolute/path/to/build-all-20261004 \
  --rmw-prefix "$D1MAX_RMW_PREFIX" \
  --livox-prefix "$LIVOX_SDK2_PREFIX" \
  --sdk-vendor-root /absolute/path/to/robot_sdk \
  --jobs 2 --apply
```

该工具不是第二套导航入口：只准备同一主线的产物，不启动节点、不生成执行许可、不更新 selector、不连接 SDK，不把输出声明为已封存运行包。Web 范围会依据锁文件 `npm ci --ignore-scripts` 后构建静态资源；Python venv 和系统依赖另行准备。

产物分别位于 `<output>/nav/install`、`<output>/sdk/install`、`<output>/pct_vendor` 和 `<output>/web/dist`。后续核对产物时，显式 source 对应 overlay，并设置 `PCT_PLANNER_ROOT=<output>/pct_vendor`，不能继续导入源码目录的旧库。

本轮 GridMap 缓冲布局已改变，必须成套重编 `plan_env → path_searching → bspline_opt → scan_planner` 及调用者，不能只复制一个 `.so`。源码构建工具使用完整包依赖闭包。PCT 自带 `-march=native`，其 GTSAM/原生库必须在目标 CPU 重建，且保留编译和链接证据，避免与系统 GTSAM 混装。

## 地图和录包迁移

Git 排除 PCD、PLY、bag、NPZ、Web data 和原封存安装包。至少准备以下完整材料，不能只复制一个地图截图或最终 Path：

| 材料 | 必须一起保留 |
| --- | --- |
| 定位地图 | 实际使用的原始 PCD、来源与 SHA256、轴向/外参和定位参数 |
| 规划地图 | PCD、source_identity、manifest、route.yaml、conditioning 与 PCT tomogram；保留原始来源和支撑面标签 |
| bag 回归 | 整个 bag 目录、metadata.yaml、所有 db3/mcap 分片及采集快照 |
| Web 地图库 | 需要的 processed/derived 目录、对应 manifest 和可选列表元数据；不是 session PID 或活动锁 |

现有 09-23 跨层旧预览模板引用 `maps/processed/sc_pgo_20260923_crossfloor_complete_001/` 和 Web 下 `data/processed/20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/`。这只是历史路径索引，正式版本用自己的 `release.json`/map manifest 决定实际地图，不能把旧模板当作运行事实。

拷贝前后对实际 PCD、NPZ、manifest 和 bag 分片运行 `sha256sum` 对照。manifest 有旧绝对路径时，在新复制、未封存的版本中调整目标机路径绑定，再核对内容身份、重新计算 provenance/hash 和封存；原 seal 不能继续沿用。既有 rebase 工具还要求旧 source 路径可读并且哈希一致，不能假定换机后直接执行就可用。不修改点云坐标、不压平定位地图、不伪造 TF。数据处理变化属于新地图版本，不能覆盖既有来源记录。

不要迁移 `.venv`、`build/install`、PID、活动锁、运行 session、`manager.local.json`、SSH 密钥或账户凭据。Foxglove 布局可以按文件导入，云端或桌面编辑不会自动更新本仓库。

## 正式版本准备

正式运行仍使用 `d1max_nav_ws/tools/navigation_entry.sh`，它加载明确选择、哈希封存的整套版本，而不是现场 checkout。默认 `deploy/single_floor_release.json` 指向旧电脑未上传的 `single_floor_execution_20261002_entry_v1`，且这批新源码已超出该版本。

新机必须完成以下检查后才配置 Web 或运行导航：

1. 以当前提交成套构建接口、native、tracker、BT、SDK、RViz/application 和定位依赖，记录实际导入及 ELF。既有 loader 还要求 `$D1MAX_NAV_ROOT/install/local_setup.bash`，外部 build 输出不会自动放到该位置；必须在新机部署组装时明确准备匹配的基础 overlay，不链接旧 install。
2. 按现有 release descriptor 布局整理对应组件、冻结源码/工具和明确地图。普通源码 build 的目录结构不等于 release 结构；不能随便补一个空 release.json 绕过它。
3. 在同一目标机环境下按 `$D1MAX_NAV_ROOT/tools/release/seal_single_floor_release.py` 生成新的完整性封存，校验启动及科学计算库闭包。封存只证明版本一致，不是物理验收。
4. 显式设置 `D1MAX_RELEASE=/absolute/path/to/sealed-release`，执行 `bash "$D1MAX_NAV_ROOT/tools/navigation_entry.sh" --check-release`。这条只核对 bytes/import/RMW，不启动 ROS 或 SDK。
5. 通过隔离回归后再决定本机 activation；规划与执行分别绑定 SDK 会话和证据。不得修改验收 false 来取得执行权限。

这里没有新增自动组装/激活完整 release 的工具，也没有把开发机绝对路径 seal 当作可搬运镜像。封存、整图、运动和制动验收的缺项见 [KNOWN_ISSUES.md](KNOWN_ISSUES.md)。如果这些条件缺失，完成构建后停在源码准备状态，不回退到旧预览伪装成功。

## Web 和显示

Web 后台 Python 与 ROS Python 分开。可在应用目录创建自己的 venv，再安装已有 requirements；前端用已提交 lockfile，而不是 `npm install` 自动升级算法或 UI 依赖。选择 Node 版本时遵守 lockfile 中 Vite 等组件的 engines 要求。

根脚本 `start_d1max_map_manager.sh` 是备用入口，需先确认没有受管 Web 实例。后端实际读取 `$D1MAX_APP_ROOT/map_manager/frontend/dist`，外部 `<output>/web/dist` 需在明确的 Web 部署步骤中复制到对应位置，源码构建工具不会自动覆盖它。运行前 source 路径模板、提供 `D1MAX_MAP_MANAGER_PYTHON` 和地图目录；单纯页面可用不代表导航 release 已就绪。安装本机管理器会创建新的本机凭据，原电脑凭据不要复制。

Web 只做连接和启停；初值、XYZ 目标、确认与取消在唯一 RViz 完成。不要同时使用旧 `motion_coordinator`、Nav2 历史入口或额外 SDK sender。

## Isaac Sim 后续工作

在另一台电脑确认 GPU、驱动和 Isaac Sim 安装版本后，再给现有主线增加仿真传感器与执行适配。当前既有 mock 是逻辑夹具，不是 Isaac 物理闭环。

轮式机器人需要独立几何/运动 profile、实测物理反馈、统一仿真时钟与暂停/重置合同，以及原始雷达和 IMU 字段适配。保留 BT、原生规划器、跟踪核心和唯一 writer，不能另搭一套导航器或把模拟命令当作真实停稳测量。Isaac 与 Humble/Zenoh 的版本兼容、场景地图身份和整链时效均待单独验收。
