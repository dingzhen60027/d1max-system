# D1 Max 地图工作台 · 2D / 3D

该工具参考 `/home/dndx/go2_nav/tools/map_manager` 的 Web 工作台布局和交互，面向
D1 Max 当前建图链路做了适配。首页提供独立的 **2D 导航** 与 **3D 地图** 入口。

## 页面层级

- 首页 `/#/`：仅加载地图索引，不加载 Three.js、PCD 或 ROS。
- 3D `/#/3d/maps`：全局侧栏切换原始点云、处理结果、规划产物、归档、异常；中间始终是当前类别的列表、预览和详情。建图启停/保存收进“建图管理”对话框。
- 2D `/#/2d/versions`：地图版本、生成 2D 地图、导航准备、独立归档。可从 3D 详情直接携带源 PCD 进入生成表单。
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

处理结果默认保存到 `d1max_ros2/map_manager/data/processed/<时间_任务ID>/`。其中
`processed_map.pcd` 是新的二进制 PCD，`manifest.json` 记录来源、参数和各处理阶段
点数。源 PCD 始终只读；D1 Max 原图存在 `intensity` 时，处理结果也会保留该字段，
体素降采样时对体素内的强度取均值。

## 模块化点云流水线

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

**定位已集成，导航执行仍未接入：** `/#/2d/navigation` 现在为“定位调试”。Web 可连接只读数据后台、启动 D1 双 EKF + GICP，并像 RViz2 一样按下选位置、拖动指向机头、松开提交初值（Esc 取消）。离线仅记录箭头；精确 XYZ / 角度在高级设置。Foxglove 原左侧 PCD 窗口看结果。不启动 Nav2、不发送速度 / 姿态目标，`navigation_ready` 始终为 false。

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
