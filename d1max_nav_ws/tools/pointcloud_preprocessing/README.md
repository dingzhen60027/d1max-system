# 建筑结构保留：PCD 离群预处理

独立于 PCT、ROS、SDK 和 Web 任务执行器。一个 YAML 定义原始 PCD、参数、保护策略及结果库目录：

```bash
python3 /home/dndx/d1max_nav_ws/tools/map/preprocess_structure_cloud.py \
  --config /home/dndx/d1max_nav_ws/tools/pointcloud_preprocessing/configs/sc_pgo_0919_structure.yaml
```

## 当前处理

1. 保留原始 binary PCD 记录与全部字段，只用 XYZ 计算。非有限坐标单独归档。
2. 半径候选：15 cm 内少于 3 个**其他点**；统计候选：24 个近邻平均距离超过全图均值 + 3 倍标准差。二者是几何异常候选，不是“人物标签”。
3. 局部结构保护：在候选点附近拟合实际局部面/线。候选点本身必须贴合该面/线，且具有足够数量、跨度和空间支持，才被保留。不能因为其下方恰好有平地，就保护悬空高点。
4. 仅删除未获结构支持的候选。所有邻域均基于原始有限点，不逐轮缩蚀，不把删除结果反馈成新一轮离群依据。
5. 输出新版本，保留 `processed_map.pcd`、`pipeline.yaml`、manifest 和 `audit/removed_points.pcd`、完整原始索引/原因。保留点坐标、强度及其他记录字节不变。

没有全局高度裁剪、体素均值重采样、地面压平、曲面平滑、补洞或坐标系转换。结构保护为局部几何约束，不是建筑语义模型；不能保证所有真实细节必定保留。合成地面/墙面/坡面/细线回归，以及真实地图对比用于约束误伤。

半径与统计离群的基础思路见 [Open3D 官方说明](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud_outlier_removal.html)。此处为 SciPy KD-tree 的原始索引实现，明确排除自身计数，并增加局部支撑保护，不依赖 Open3D 的字段重写。

## 能力边界

单张融合 PCD 中，密集人物残影可能与柜子、柱子或其他静态障碍有相似几何。因此本配置只清除缺少支撑的异常点，**不声称全部去人影**。当前 run 没有逐关键帧扫描，不能直接做多时刻可见性/射线验证。真正的时序去动态需要重新取得配准且去畸变的扫描及传感器位姿。

被删除的位置不自动等于“已证实可通行”。预处理不更改 PCT 阈值、膨胀参数或当前导航地图，不发送运动指令。原始文件始终只读，删除点归档可恢复。

## 模块与结果库

- `pcd_io.py`：严格 binary PCD、字段/VIEWPOINT/记录字节保真、拒绝覆盖。
- `algorithms.py`：候选筛选和局部面/线保护，仅返回原始索引。
- `runner.py`：配置校验、资源/删除比例保护、输入哈希、独立版本、完整结果发布。
- `configs/*.yaml`：所有算法参数。

最终结果进入现有 Web 的 `map_manager/data/processed/<新版本>/`，`manifest.json` 完成后才被识别。删除点放在同版本 `audit/` 下，不作为另一张建图结果刷入列表。`source_id` 关联原始 SC-PGO 地图，支持已有的前后对比。

schema 为 `d1max.structure_cleaning/v1`，不是旧 Web runner 支持的 `d1max.pointcloud_pipeline/v1`。可查看配置并用上面的独立命令复现，不能假称旧 UI“开始处理”已支持这些新模块。每次执行分配新版本，绝不自动替换当前地图。

测试：

```bash
cd /home/dndx/d1max_nav_ws
PYTHONPATH=tools python3 -m pytest -q tools/pointcloud_preprocessing/tests
```

## 单层平地规划副本（显式场景先验，2026-09-22）

另一个独立 schema `d1max.flat_floor_planning/v1` 仅在用户确认**当前楼层地面水平**时使用。不是上面只删点的结构清理，也不是 SLAM/外参修复。

入口配置 `configs/sc_pgo_0919_flat_floor.yaml` 同时定义输入 PCD、对应优化轨迹、地面模式、近地孤点/碎簇保护及 PCT 构建参数：

```bash
cd /home/dndx/d1max_nav_ws
PYTHONPATH=tools:src/d1max_pct_planner OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=4 \
  python3 -m pointcloud_preprocessing.flat_floor_runner \
  --config tools/pointcloud_preprocessing/configs/sc_pgo_0919_flat_floor.yaml
```

复跑须在 YAML 指定新的输出目录；已有版本拒绝覆盖。`--pcd-only` 可只生成派生 PCD，不构建 PCT。旧 Web runner 尚不能执行此 schema，不能直接用旧“开始处理”代替该入口。

流程：轨迹附近真实点拟合局部楼面 → 有界楼面高度场 → 有实际楼面支撑的列做 Z 平移 → 小残差楼面整平 → 清理近地孤点/碎簇 → 官方方程 PCT → 原生 A* / GPMP。**没有沿轨迹刷自由空间，没有新增地面点，也没有继续缩小膨胀。**

- `flat_floor.py` 是纯数组算法；参数全部在 YAML，可独立关闭碎簇阶段。
- 碎簇必须有实际地面支撑；连接墙柱、有较大空间尺寸或可靠水平面证据的低物体保留。仅靠静态几何仍不能完美区分所有小实物与残影。
- `structural_obstacles.py` 是后置的独立低位结构判定模块，配置为 `obstacle_cleanup`，默认关闭。它不移动或新增点；输出同长度保留/移除/保护 mask，并由 runner 归档被移除的原始记录。
- 当前单层配置使用 `support_mode: metric_disk`：11 cm 内至少 3 个实测楼面点，同时 22 cm 内至少 10 点。原来的“15 cm 格内至少 3 点”会在格边界漏掉地面证据，因此不能把换了网格原点造成的计数变化当作真正无地面。
- 4–70 cm 的候选回波，若没有连续多高度的竖直墙/柱证据或可靠低箱顶证据，才清理；低箱顶的稳健拟合还要求局部邻域至少 80% 支持。墙、细柱、真实低箱体和未知区域都有独立回归用例。
- 静态稀疏 PCD 无法保证区分全部真实小物体和残影。此配置是离线先验地图清理；实机仍须使用实时障碍感知，不得以清理后的地面替代在线碰撞防护。
- `write_derived_z` 只改所选原记录的 Z，XY/强度/其他字段保持原字节；原 PCD 不变。非刚性副本无单一 sensor VIEWPOINT，显式重置并记录。
- 域外、地下返回仅从单层规划副本排除；原记录分别归档。移除点、碎簇 mask、地面 anchors、原索引、配置和哈希均保留。
- 输出专用坐标系 `d1max_flat_floor_planning`。**不得把它当作原始 3D 定位匹配地图，不存在与原图等价的单一刚体 TF。** 它仅服务当前离线平地全局规划实验；室外坡面/楼梯不能沿用这一先验。

预览（私有 Zenoh，无 SDK/运动）：

```bash
scripts/planning/start_pct_preview.sh start --config /home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/preview_flat_floor.yaml
```

已有本工具预览时先 `scripts/planning/start_pct_preview.sh stop`，避免堆叠。新几何必须重新规划，不复用旧图曲线。详细诊断见 `log/pct_preview/flat_floor_20260922_audit/RESULT.md`。

连通验收单独运行 `python3 -m d1max_pct_planner.map_acceptance --tomogram TOMOGRAM --trajectory KITTI_POSES --output NEW_AUDIT_DIR`。它使用已知、可通行地面的四邻接图及高度阶跃限制，不允许借对角接触跨墙。沿历史轨迹关联到同一分量只是离线覆盖检查，不是机器人净空证明；还须对实际端点运行原生规划和完整曲线碰撞检查。
