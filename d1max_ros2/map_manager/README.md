# D1 Max 3D 地图工作台

该工具参考 `/home/dndx/go2_nav/tools/map_manager` 的 Web 工作台布局和交互，面向
D1 Max 当前真实可用的 3D 建图链路做了适配。

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

本版本刻意不包含 PGM/YAML 栅格地图、2D 编辑器、2D 地图构建、Nav2 激活或
2D 目标点功能。D1 Max 尚未接好对应定位/导航链路之前，页面不会显示虚假的可用操作。

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
