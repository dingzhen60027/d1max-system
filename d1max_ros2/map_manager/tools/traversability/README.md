# D1 Max 双层地面提取与几何可通行地图

这是独立、离线的处理模块；不连接 SDK、不发布速度、不启动导航或定位、不修改源 PCD。

## 当前任务：只提取地面

入口配置为 `../../config/traversability/sc_pgo_0904_ground_only.yaml`，与旧的机器人通行性评估完全独立。

`extract_ground.py` 依次进行：读取原始 XYZI → ROI 与法向估计 → 两层参考平面带筛选 → 采集轨迹辅助筛选楼梯踏面 → 稀疏孤立点与小连通片清理 → 原始点导出。

所有参数在一个 YAML 内。上下层平面、楼梯区域针对本次地图确定，不是任意地图的自动楼层识别器。楼梯轨迹高度差由地面和位姿估计，仅用来排除悬空水平物体；没有机身宽度、净空、障碍膨胀或通行成本。不会改原始坐标，也不插值补洞。

```bash
OMP_NUM_THREADS=6 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/d1max-trav-mpl /usr/bin/python3 d1max_ros2/map_manager/tools/traversability/extract_ground.py --config d1max_ros2/map_manager/config/traversability/sc_pgo_0904_ground_only.yaml
OMP_NUM_THREADS=6 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 d1max_ros2/map_manager/tools/traversability/test_ground_extraction.py
/usr/bin/python3 d1max_ros2/map_manager/tools/traversability/publish_viewer.py --result d1max_ros2/map_manager/data/traversability/sc_pgo_20260904_143406_ground_only
```

保存到 `data/traversability/sc_pgo_20260904_143406_ground_only/`：

- `ground.pcd`：总地面，原始 XYZI 的精确子集；`lower_ground.pcd`、`upper_ground.pcd`、`stairs_ground.pcd` 分开保存。
- `ground_colored.ply`：按楼层/楼梯着色，不表示通行风险。
- `ground_points.npz`：原始点索引、区域标签、XYZ、intensity 和坐标系，可追溯每个点。
- `pipeline.yaml` / `report.json`：完整参数与源文件哈希，源码和产物均保留。

查看地址：`http://127.0.0.1:8766/traversability/sc_pgo_20260904_143406_ground_only/`。
单独的 `ground_viewer/` 渲染真实点，不使用旧版栅格方片或背景网格。可调整点大小（只改显示，不改变 PCD），查看双层、3D 和楼梯。此产物不是避障代价地图。重跑会更新该派生目录；新版本可在 YAML 改输出目录，原始 PCD 始终只读。

## 旧任务：机器人几何通行性评估（保留）

## 输入与配置

旧通行性配置入口：`../../config/traversability/sc_pgo_0904_width_aware.yaml`。

`sc_pgo_0904_two_floors.yaml` 和同名输出保留为外接圆基线，`_width_aware` 为朝向矩形结果；当前用户需求请查看上面的 `_ground_only`。

源地图准确指向 `20260904_143406_bag_faster_lio_pgo_first_config/sc_pgo/optimized_map.pcd`，1,300,464 点。
源文件 hash 写入报告，生成前后校验。优化轨迹只用于筛选有采集路径证据的支撑连通分量，不被当成障碍穿透许可。

上下层是相对高度，不假定现场楼号。主地面参考平面由源点云拟合，只用于区域筛选，不旋转、不压平原始坐标。折返楼梯 ROI 由原始采集轨迹及点云检查得到，保留在配置中。

## 模块顺序

`load_input`：ROI / 有限值 / 体素降采样；法向量只筛选支撑候选，障碍检测保留所有结构点。

`build_surfaces`：多高度切片、候选支撑高度、真实上方观测、净高、高差、支撑邻域、主楼层去重。没有地面观测的格子不填补；没有顶部观测不能变成绿色。

`candidate_graph`：相邻网格支撑面连接、最大高差、采集轨迹种子。只连四邻接，不切角、不跨楼板、同一 XY 不能竖直跳层。

`assess_footprint`：默认朝向矩形，检查完整长宽包络的 24 个朝向；旧配置保留外接圆模式。方向矩形检查包括前后空间，不是只检查宽度。楼梯保持条件通行；本模块没有实现登梯步态或接触规划。

`final_graph_report`：在最终障碍过滤后重新检查连通性，不把原始几何图的连通性当成最终结果。

## 明确边界

- 规格书站立尺寸 0.930 × 0.480 × 0.585 m；当前每侧余量 0.05 m，标称宽度包络 0.58 m、长度 1.03 m，另计栅格占据单元的尺寸误差。垂直余量 0.08 m。
- 旧模式把机身膨胀为半径 0.603 m 的圆，窄路会被明显保守排除。当前矩形模式中，0.8 m 直走廊可沿长轴通行，但不能横向转身；0.3 m 开口依然禁止（回归用例已覆盖）。
- 最大坡面 45° 与连续台阶高 25 cm 是不同指标；当前作为几何候选模型参数，不构成实机安全认证，黄色楼梯需复核。
- `heading_mask` 与 `observed_heading_mask` 是 `uint32`，第 k 位对应 `heading_angles_rad[k]`（矩形前后对称，模 pi）。`heading_checked=false` 的楼梯不能解释为已通过朝向碰撞检查。下游必须消费朝向约束；只有 XYZ 路线不能验证窄路转弯。
- 支撑邻域观测覆盖 85% 是工程建模参数，不等于逐足安全验证。空洞不生成地面，缺失处不作为自由空间。
- PCD 没有传感器射线的自由空间观测；不宣称静态 PCD 可完全证明所有遮挡区域安全。
- 本次源地图接受回环为 0，已有倾斜、重影、累计误差不会因为生成可通行图而消失。
- 地图为静态几何候选，尚未集成动态障碍、步态、摩擦、载荷或控制链路。

## 运行

在项目根目录：

```bash
OMP_NUM_THREADS=6 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/d1max-trav-mpl /usr/bin/python3 d1max_ros2/map_manager/tools/traversability/build_map.py --config d1max_ros2/map_manager/config/traversability/sc_pgo_0904_width_aware.yaml
/usr/bin/python3 d1max_ros2/map_manager/tools/traversability/test_assessment.py
/usr/bin/python3 d1max_ros2/map_manager/tools/traversability/publish_viewer.py --result d1max_ros2/map_manager/data/traversability/sc_pgo_20260904_143406_width_aware
```

输出留在 `data/traversability/sc_pgo_20260904_143406_width_aware/`。默认重新运行覆盖该专用生成目录中的同名派生产物；如需新版本，用 `--output` 指定新目录。原始地图始终只读。

## 产物

- `traversability.npz`：最终节点 XYZ、网格索引、状态、代价、所属区域、净高、支撑覆盖率、候选边和连通分量。代价无穷大的节点不可规划；黄色不是自动执行许可。
- `height_layers.npz`：中间多层高度与顶部观测；不是最终可通行状态。
- `traversability.pcd/.ply`：同坐标系彩色可视化；不能替代定位 PCD。
- `lower.ply / upper.ply / stairs.ply`：分区域预览。
- `pipeline.yaml / report.json / two_floors_overview.png`：可复现配置、源文件指纹、审计数据与总览。

发布脚本把静态查看副本放进现有 Web 的 `frontend/dist/traversability/`，无服务重启、无额外后台进程。现有 Web 运行时地址为：

`http://127.0.0.1:8766/traversability/sc_pgo_20260904_143406_width_aware/`

重新构建 Web 前端会清空 dist；之后重新执行 `publish_viewer.py` 即恢复入口，不需要重新处理地图。持久结果在 data 中不受影响。
