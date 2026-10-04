# D1 Max 地图工作台 · 2D / 3D

该工具参考 `/home/dndx/go2_nav/tools/map_manager` 的 Web 工作台布局和交互，面向
D1 Max 当前建图链路做了适配。首页提供独立的 **2D 导航** 与 **3D 地图** 入口。

## 导航主线（2026-10-04）

`/#/planning` 只有一个启动入口：BehaviorTree.CPP →
`tools/navigation_entry.sh` → `d1max_pct_scan.navigation_session`。
架构面向室内外、多楼层导航，单楼层只是当前测试范围；页面范围来自实际版本，
通用名称不代表楼梯或室外运动已经验收。新旧名称调用同一实现、同一个任务管理者。
Web 负责显式连接、启动、停止和打开唯一 RViz；初值、三维目标、执行确认及取消由 RViz/BT 管理。
启动默认尝试打开全局布局，局部布局在同一个 RViz 面板切换。关闭 RViz 不取消核心任务。

- `/api/navigation-session/{overview,connect,start,stop,view/global,view/local}` 是主线生命周期 API，不提供目标或运动授权接口。
- 旧 `/api/single-floor` 是同一 runtime 的兼容 URL；`single_floor_session`、旧 dispatcher、systemd 单元名和 `data/single_floor` 路径保留版本/所有权兼容，不启动第二套服务、不迁移或遗失现有会话。
- 旧 `/api/live-planning/start` 返回 410，不回退到旧预览。旧状态与停止 API 仅用于核对、清理历史会话。
- 页面区分默认选择、Web 配置和实际运行版本。发布校验由单 worker 有界缓存执行；启动时再次同步检查，不用旧快照放行。
- 停止先让核心协调退役，再核对所属进程组和唯一视图；旧双窗口单元只作所有权核对后的清理兼容。软件退役与实测停稳分别记录。未知进程不清理、重复启动不叠加。
- Web 只有一个「打开 RViz」按钮。兼容的 `view/global` / `view/local` API 向同一窗口发显示请求，不能启动第二个进程；旧插件缺少切换合同会明确要求更新版本，不退回双开。
- 私有 `data/single_floor/activation.json` 必须绑定已封存版本。只规划用途可使用 `sdk_session_policy: bind_current_on_start`，在启动时只读捕获当前数据会话并固定到本次 session；执行用途仍要求固定 SDK 会话及物理验收记录。
- 新公开 dispatcher 的私有配置同时固定 `entrypoint_sha256` 和 `compatibility_entrypoint_sha256`，后者绑定实际调用的旧 loader；旧封存合同不改。RViz 重开仍使用该会话记录的入口闭包，不随新配置切换。
- 浏览器实际使用 `frontend/dist`，修改 JSX 后必须重新构建；源码更新不等于静态网页或导航发布包已更新。

页面及后端接线不构成实机验收，也不会自动连接 SDK、发送初值、目标或运动命令。
验证：`tests/test_single_floor.py`、`tests/test_mainline_release.py`、`tests/test_mainline_views.py`、
前端 `scripts/test-navigation-launch-status.mjs`；`scripts/verify-live-planning.mjs` 使用实际 dist 和全部拦截的 API，仅读取静态页面，不接触生产后端。

## 页面层级

- 首页 `/#/`：仅加载地图索引，不加载 Three.js、PCD 或 ROS。
- 3D `/#/3d/maps`：全局侧栏切换原始点云、处理结果、规划产物、归档、异常；中间始终是当前类别的列表、预览和详情。建图启停/保存收进“建图管理”对话框。
- 2D `/#/2d/versions`：地图版本、生成 2D 地图、导航准备、独立归档。可从 3D 详情直接携带源 PCD 进入生成表单。
- 数据采集 `/#/bags/recordings`：原始 rosbag 录制、文件详情、下载、名称备注；`/#/bags/archived`：恢复、选择删除或列表全部删除。
- 使用真实 shadcn/ui Base 组件：Sidebar、Breadcrumb、Card、Select、Dialog、AlertDialog、DropdownMenu、ScrollArea 等，保留现有 3D 数据和点云处理功能。参考 [Sidebar](https://ui.shadcn.com/docs/components/base/sidebar) 与 [Breadcrumb](https://ui.shadcn.com/docs/components/base/breadcrumb)。
- 首屏按需加载；只有进入 3D 才下载点云渲染模块，只有点击修整才加载 2D 编辑器。

## 范围

- 自动扫描 `/home/dndx/d1max_nav_ws/maps`，不移动、不重命名原文件；
- 分类展示 Faster-LIO、FAST-LIO2、SC-PGO 优化图和 PCT 规划派生点云；
- 读取 PCD/PLY 点数、大小、时间、回环接受/拒绝次数和完整性提示；
- 浏览器内 Three.js 高度着色 3D 预览；
- 同次 Faster-LIO 与 SC-PGO 结果可并排比较；
- “建图结果”只展示原始建图 PCD；处理后的点云独立进入“处理结果”栏；
- 参考 Go2 参数提供统计滤波、半径滤波、体素降采样和可选 Z 高度裁剪，参数方案可保存复用；
- 每个处理结果记录源文件、逐阶段点数和完整参数，并可与原图并排比较；
- Web 启动/保存/停止 D1 Max 建图，固定使用 `rmw_zenoh_cpp` 和 ROS Domain 24；
- “当前地图”、名称、备注和归档是无损元数据，不改写原始地图目录。

## Rosbag 录制与管理

独立模块 `backend/bags/`，唯一内置录制配置为 `config/recording.yaml`。
不启动 SDK、不抢占控制、不切换 SLAM、不修改机器人时间或传感器值。
直接使用 Zenoh 客户端连接配置中的机器人路由，默认 Domain 24，**不使用 Fast DDS**。
只打开 Web 不会创建 ROS 节点；点击“开始录制”才启动短时检查和录制进程。

- 固定选择：`/front_lidar`、`/rear_lidar`、`/front_lidar/imu`、`/rear_lidar/imu`、`/imu_driver/imu_central`；这五路必须有真实消息。TF 与 TF static 同时录制，但不阻止无 TF 的原始数据采集。
- 默认开启 MC：`/odom/mc_odom`。可关闭；勾选后同样要求有真实消息。
- 图像可选：前后 `/front_camera/image_compressed`、`/rear_camera/image_compressed`。
- 原厂 `/odom/localization_odom` 默认关闭，仅用于结果对照，不是真值，也不会被自动接入我们的定位。
- 保持原始消息值、时间戳、frame 与逐点字段；不执行 IMU 单位转换或外参变换。
- SQLite3、不压缩、默认 16 GiB 分片 / 512 MiB 缓存；开始至少保留 5 GiB，剩余不足 1 GiB 自动停止。
- 20 秒内未收到所有必选数据则失败，不创建假成功的空包；录制中必选数据断流 10 秒自动停录，并标注不完整。
- 停止优先 SIGINT，等待缓存及 `metadata.yaml` 落盘。强制退出或任何必录话题最终消息数为零，均不能标为成功。监测 Hz 是接收统计，保存后的消息数才是实际入包统计；“已保存”不等于传感器质量或零丢包已经验收。
- 关闭浏览器标签页不会停止录制；使用“停止并保存”。关闭 Web 服务会请求保存并清理所属进程。进程身份使用 PID + Linux 启动标识 + job 路径校验，互斥文件锁阻止并发录制；遇到外部 CLI 录制只报冲突、不强杀。
- Web 建图/定位与此原始录制入口互斥，防止录进回放数据或启动脚本干扰录制。现有 CLI `record_slam_bag.sh` 保留不变。

新文件默认保存到 `/home/dndx/d1max_nav_ws/bags/`。自动索引该目录以及
`/home/dndx/d1max_rosbag903/`、`/home/dndx/d1max_rosbags/` 中的旧包（最多三层，不跟随符号链接）。
`D1MAX_BAG_DIR` 可设置新录制目录；`D1MAX_BAG_LIBRARY_ROOTS` 以冒号分隔额外管理目录，设为空可禁用额外目录。
不移动、不重命名原始 bag；显示名、备注、归档写入 `data/bags/catalog.json`。
只能删除已归档、非录制中、未被进程占用的目录；批量删除先核对全部目标，再永久删除。
不会删除管理根目录或其他 bag，包含符号链接的目录拒绝删除。

每个新包附带 `recording.yaml`、`d1max_recording.json`、录制日志和 `snapshots/`。
快照保存的是 PC 本地配置和最新本地审计文件，带来源及 SHA256；**不是本次机器人 OTA 自动实读或标定正确性的保证**。
跨文件分片的 bag 下载时保留 `metadata.yaml` 与全部 `.db3`。
浏览旧包仅读取索引和文件大小，不加载点云或扫描整份数据库。

离线验收（全部使用临时目录，不需要机器狗）：

```bash
cd d1max_ros2/map_manager
python -m unittest discover -s tests -v
source ../d1max_ros2_env.sh
/usr/bin/python3 scripts/verify_bag_recording.py
# 实际 Zenoh / ROS / rosbag，只连接 loopback，隔离 Domain 184。

# 浏览器测试另开临时 QA 后端（使用带 FastAPI 的 Python）
python scripts/serve_bag_qa.py
# 另一个终端
cd frontend
npm run test:bags:browser
```

处理结果默认保存到 `d1max_ros2/map_manager/data/processed/<时间_任务ID>/`。其中
`processed_map.pcd` 是新的二进制 PCD，`manifest.json` 记录来源、参数和各处理阶段
点数。源 PCD 始终只读；D1 Max 原图存在 `intensity` 时，处理结果也会保留该字段，
体素降采样时对体素内的强度取均值。

## 模块化点云流水线

MOLA 离线建图已作为独立可选方案接入“3D → 建图管理”；默认 Faster-LIO + SC-PGO 不变。
完整配置、模块边界、输出和隔离说明见 [MOLA 模块文档](backend/mapping/README.md)。
它直接处理 rosbag 文件，不替换当前实机定位或 Foxglove 链路。

### LIO-SAM 离线实验结果

LIO-SAM 目前只纳管完成后的结果，不加入正式建图启动后端。3D 原始点云中分别标注
“LIO-SAM · 单前雷达”和“LIO-SAM · 前后双雷达”，均为六轴中心 IMU 适配实验，
原生地图是角点/平面特征集合，不是全部原始雷达回波。不会自动选用或推荐实验地图。

完成的独立实验可用下列命令登记（`--sensor-mode front` 或 `dual`，须与运行 manifest 的
`input` 中前后雷达声明一致；`completion.map` 必须指向该次 `map/GlobalMap.pcd`）：

```bash
cd d1max_ros2/map_manager
python3 -m backend.mapping.lio_sam_artifacts \
  --run /absolute/path/to/completed-lio-sam-run \
  --maps-root /home/dndx/d1max_nav_ws/maps --sensor-mode dual
```

只复制最终 `GlobalMap.pcd` 到全新的 `maps/lio_sam/<run>/`，最后写完成清单；源实验文件不动，
同名目标拒绝覆盖。Web 仅依据完成清单读取这一份图，不递归把 `trajectory.pcd`、
`transformations.pcd`、`CornerMap.pcd`、`SurfMap.pcd` 或关键帧放入地图列表。
清单记录来源、bag、传感器、版本与 SHA256。登记后刷新页面即可，无需重启建图/通信。

每条处理流程都是一个完整 YAML，而不是散落在程序中的参数。内置配置位于
`config/pipelines/`，Web 另存的配置位于 `data/processing_configs/`。YAML 中
`modules` 的排列顺序就是实际执行顺序，每个模块都拥有独立的 `enabled` 和
`parameters`。当前注册模块为：

- `crop_z`：Z 高度裁剪；
- `voxel_downsample`：体素降采样；
- `statistical_outlier`：统计离群点滤波；
- `radius_outlier`：半径离群点滤波。

每次 Web 任务都会在结果目录写入已解析的 `pipeline.yaml`，其中 `input.path` 和
`output.path` 是本次实际使用的绝对路径。因此只凭这一份文件就能完整复现从原始
PCD 到处理 PCD 的过程：

```bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0'
PYTHONPATH=d1max_ros2/map_manager \
  python3 \
  -m backend.pointcloud_pipeline \
  --config d1max_ros2/map_manager/data/processed/<任务>/pipeline.yaml
```

也可以复用模板并覆盖输入输出：

```bash
PYTHONPATH=d1max_ros2/map_manager \
  python3 \
  -m backend.pointcloud_pipeline \
  --config d1max_ros2/map_manager/config/pipelines/structural_preservation.yaml \
  --input /path/to/raw.pcd --output /path/to/processed.pcd
```

## Go2 2D 地图流程迁移

已迁移的是 Go2 **离线地图准备与版本管理**，不是 Go2 机器人的导航启动脚本，也不会复制或修改 Go2 地图数据。

1. 选择未归档的原始/处理后 PCD；配置可选统计与半径滤波、Z 高度切片、XY 栅格分辨率、边界、每格点数阈值。
2. 生成完整候选版本；不自动选用。定位 PCD 保留 3D 高度与强度，只有投影阶段切 Z，禁用滤波时原样复制 PCD。
3. 修整支持画墙、擦除杂点、未知区域、自由笔刷、直线、撤销/重做、缩放；坐标按原始 PGM 像素保存，不保存浏览器截图。编辑永远另存版本，原图、原坐标与定位 PCD 不变。
4. 检查修订对比，再手动选用；支持回退上一版本、重命名、下载完整 ZIP、无损归档与恢复。
5. 归档区可删除所选或全部永久删除。只删除明确列出的已归档版本及其配套文件，禁止删除当前选用版本；不会删除源 PCD。

默认配置位于 `config/navigation2d.yaml`；用户方案为 `data/navigation2d/profiles/*.yaml`。
版本位于 `data/navigation2d/versions/grid-<id>/`，包含：

- `map.pgm` / `map.yaml`：ROS 栅格及分辨率、原点、阈值。
- `localization.pcd`：该版本配套定位点云。
- `pipeline.yaml`：输入点云、完整滤波与投影参数、累计修整笔画、输出路径。
- `manifest.json`：来源、父版本、参数、像素统计、生成时间。

`data/navigation2d/state.json` 只记录本工作区选用与归档，不写 `maps/active`，不启动 Nav2。
`D1MAX_MAP_MANAGER_DATA` 可以为测试指定完全独立的数据目录。

**坐标与可通行性注意：** Z 上下限是 PCD 绝对坐标，不是离地高度，必须先确定单层范围。静态点云没有射线观测信息，空白不等于可通行：默认保留未知，也提供 Go2 兼容的空白设自由方案。输出占据=0、自由=254、未知=205；`free_thresh=0.196`，修正 Go2 原 `.25` 阈值会把 205 误识别为自由的问题。地图文件格式参考 [Nav2 Map Server](https://api.nav2.org/nav2-rolling/html/md_nav2_map_server_README.html)。

单个 YAML 可以重放从原始 PCD 到地图及全部手工修订的流程，输出目录必须不存在，防止覆盖：

```bash
cd d1max_ros2/map_manager
/path/to/map_manager_venv/bin/python -m backend.grid_maps \
  --config /path/to/version/pipeline.yaml \
  --output-dir /path/to/new-output-directory
```

原始 PCD 必须仍存在。输出不自动注册或选用，不发送机器人指令。2D 生成与 3D 点云处理共用单任务锁，重复请求返回 409；取消/关闭会请求停止，重启后未完成任务标记中断，不自动重跑。

**单楼层 Nav2：** `/#/2d/navigation` 包含“定位 / 初始位姿”和“Nav2 启动”两个页签。定位页保留现有 LIO + PCD 链路与 RViz 风格的初值箭头；Nav2 页仅负责启动 / 停止、运行状态、限速，以及 SDK 显式解锁 / 锁定。默认勾选“同时打开 RViz2”。**导航目标通过 RViz2「Nav2 Goal」发布，任务取消通过 RViz2「Navigation 2」面板操作**，路径和全局 / 局部代价地图也在 RViz2 查看。Web 不提供导航目标表单或任务取消入口。无桌面环境时可取消勾选，以无窗口方式启动。

默认 **离线仿真** 使用独立的回环 Zenoh 路由，不连接 SDK。**实机** 要求先启动同一选用地图的真实定位，SDK / 雷达数据新鲜且不是回放。勾选“启用本次 SDK 运动能力”只创建能力，不会自动解锁；仍需显式确认“解锁 SDK 运动”，然后在 RViz2 中发布目标。只读 SDK 后台不能解锁，Web 不会替你改后台配置、自动起立、切换姿态或解除急停。定位、标定、运动状态、控制权及数据龄不满足门限时保持锁定，不自动重试。速度硬上限为 **1.5 m/s**，实际速度也受 Nav2 与 SDK 配置中的更低门限约束。

模块边界：`backend/navigation.py` 仅调用固定的 `start_navigation.sh`；`d1max_navigation/navigation_commands.py` 执行带导航会话 ID / 地图 ID 核验的 ROS 命令；控制门与 SDK 适配器各自独立核验。API 为 `/api/navigation/{overview,start,stop,command}`，其中 `command` 仅允许 `arm` / `disarm`；`goal` / `cancel` 及任何坐标字段均返回 422。请求不接受任意 ROS 名称、文件路径、Shell 命令或速度参数。停止只清理所属 Nav2 进程组，不断开共享 SDK / 定位。Web 启动的 Nav2 带 `--web-owned` 并绑定 Web 服务生命周期；Web 正常 / 异常退出均不遗留它的节点，外部 CLI 启动的会话不被 Web 退出误停。运行期间锁住地图版本；重复启动、过期会话、地图不匹配、回放充当实机均拒绝。

定位读取当前版本的 `localization.pcd`，源点云不覆盖；运行期间锁住选用版本，禁止切图、取消选用、归档 / 删除该版本。初值仅进入匹配种子，连续确认后才发布全局 TF / 可信位姿。断流不冒充成功，失败不自动重试。机身到前雷达外参仍为 CAD 近似，实机精度尚未验证。

初值 API 显式传 `reference: body`，位置和方向都指机身，后台依据同一份 YAML 组合一次固定机身→tracking 外参；不自动估计地面、不附加 Z 偏移、不修改地图。像素/地图坐标纯函数在 `frontend/src/lib/pose-estimate.js`，手势组件在 `PoseEstimateMap.jsx`，ROS 变换在导航包 `initial_pose.py`，职责分离。匹配器诊断显示未收敛 / 门限拒绝 / 过期等原因；橙色 `scan_initial_preview` 是未验证的显示数据，不能用于导航。

唯一算法配置为 `/home/dndx/d1max_nav_ws/src/d1max_localization/config/localization.yaml`，每次启动保存到 `data/localization/<session>/`，另存状态、初值回执与日志。`d1max-localization-managed.service` 使用独立 cgroup，绑定 Web 生命周期；停止等待 12 秒后清理其全部子孙节点。停止定位不关闭共享监看链路，关闭 Web 会停止定位。API 为 `/api/localization/{overview,config,connect,start,stop,initial-pose}`，配置凭据仅在后端使用。

离线算法与帧说明见 `/home/dndx/d1max_nav_ws/src/d1max_localization/README.md`。Web 回归：`python -m unittest discover -s tests`；浏览器检查：`frontend/scripts/verify-localization.mjs`（POST 全部拦截，不启动实机）；进程清理检查：`D1MAX_LOCALIZATION_QA=1 python scripts/verify_localization_cgroup.py`（独立 QA 单元，不操作生产服务）。

## 启动

```bash
cd '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0'
./start_d1max_map_manager.sh
```

浏览器访问 `http://127.0.0.1:8766`。

当前机器会优先使用本工具自己的 `.venv`，若尚未创建，则兼容使用现有 Go2 Map
Manager 的 Python 环境。新机器可以执行：

```bash
python3 -m venv d1max_ros2/map_manager/.venv
d1max_ros2/map_manager/.venv/bin/pip install -r d1max_ros2/map_manager/requirements.txt
```

## 前端开发

```bash
cd d1max_ros2/map_manager/frontend
npm install
npm run dev
```

生产构建：

```bash
npm run build
```

后端会直接托管 `frontend/dist`。

## 当前地图梳理规则

- `maps/runs/<run>/d1max_map_*.pcd`：Faster-LIO 前端地图；同一运行只把最后落盘的
  文件视为最终结果，较早文件标记为保存快照。
- `maps/runs/<run>/sc_pgo/optimized_map.pcd`：SC-PGO 输出。只有
  `loop_events.csv` 中存在 `accepted` 才标记“回环验证”。
- `maps/fastlio2_*/*.pcd`：FAST-LIO2 结果。
- `maps/pct_*/*.ply`：PCT 地图、可通行性或路径预览，只作为规划派生数据。
- 空目录或只有日志、没有 PCD/PLY 的运行放入“异常记录”，不会自动删除。

截至 2026-09-04：

- 最新完整测试为 `runs/20260904_143406_bag_faster_lio_pgo_first_config`；其
  SC-PGO 输出存在，但 6 个候选全部因 fitness 被拒绝，因此不能称为闭环成功。
- `runs/20260824_175852/sc_pgo/optimized_map.pcd` 的日志记录了 8 个接受约束，
  Web 将其标记为当前“回环验证推荐”。
- `maps/latest` 仍指向 `runs/20260825_235024`。Web 不会擅自修改这个历史软链接。

## 安全约定

- “清理进程”只结束由当前 Web 后端直接启动并持有的进程组；
- 开始建图仍调用 D1 Max 原启动脚本，脚本会按自身规则清理旧 SLAM 实例；
- 结束建图先发送 `SIGINT`，给地图节点足够时间落盘，之后才升级信号；
- 无损归档只写 `d1max_ros2/map_manager/data/state.json`，不会删除点云。
- 点云处理写入独立目录，不覆盖、移动或删除任何原始 PCD；同一时刻只运行一个处理任务。

## 验证

```bash
/path/to/map_manager_venv/bin/python -m unittest discover -s tests -v
```

前端 `scripts/verify-portal.mjs` 是包含生成/编辑/删除的浏览器集成测试，**只允许运行在 `/tmp/d1max-web-migration-*` 的隔离地图后端**，拒绝在真实地图目录运行。默认端口 8767，生产端口仍为 8766。验证记录见 `VERIFICATION_2D.md`。
