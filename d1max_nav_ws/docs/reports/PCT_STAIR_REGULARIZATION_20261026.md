# PCT 楼梯支撑规整与平台衔接修复

日期：2026-10-26。范围：现有跨层规划地图、地面坐标对应、目标选取、原生 PCT 和连续局部参考。不修改定位 PCD、SCAN 碰撞策略、控制器或 SDK。

## 原因与修复

| 已复现问题 | 修复 |
|---|---|
| 原有处理修正了平地，但保留了楼梯扫描起伏 | 分别拟合两段梯段的实测支撑包络与平台，仅对现有支撑记录作有界 Z 修正；保留 XY、墙体、扶手及边界 |
| 规整后的支撑跨越 PCT slice 边界，原生优化层号来回跳 | 离线生成时保留源点 slice 归属，并逐格核对原有已知／可通行支撑不丢失；不是增加运动门控 |
| 同一平台被两个 slice 表示，层间接点锁死导致急弯 | 在两层共同合法、实测地面同高的区域联合平滑，保持原净空与路径检查，不合并不同台阶／楼层 |
| 正好从入口到出口规划时，零长度平地段仍调用 A* | 核对同一端点后省略空段，保留真实楼梯段和任务语义 |
| 新规划高度与原图、RViz 选点及支撑索引不一致 | 增加逐源点高度对应关系；地面路线和选点共用映射，局部支撑仍发送原图实测记录 |
| 最近点高度对应在相邻回波间突跳 | 同一支撑面内进行有界连续加权，不混合平台／梯段归属，不生成新的传感器点 |
| 新映射误丢远离支撑面的墙体／扶手显示格 | 阻塞结构独立保留原坐标显示，不把结构当作地面支撑 |

这不是按“标准楼梯尺寸”生成假台阶。当前点云不足以可靠恢复每级踏步的高度和深度，因此只规整全局搜索使用的支撑包络；不会将此包络作为雷达点云、机身位姿或 TF。

## 同输入结果

| 项目 | 修改前 | 修改后 |
|---|---:|---:|
| 十组原生 PCT 请求成功 | 8/10 | 10/10 |
| 楼梯路径累计绝对转角 | 579.97° | 372.99° |
| 楼梯曲率 P95 | 2.917 /m | 1.394 /m |
| 楼梯峰值曲率 | 6.753 /m | 6.490 /m |
| 楼梯 XY 路径长度 | 15.427 m | 15.906 m |
| 完整跨层请求耗时，上／下 | 3.635／3.544 s | 3.762／3.751 s |

耗时为本次同机单次热请求，含原图支撑验证，不是 P95。平台绕弯更平顺，路径约增长 3.1%，没有宣称所有曲率尖峰消失。

- 六条平地路径、跨层路线的平地段、楼梯 ROI 外五通道及可通行掩码完全一致。
- 仅 1,805 条现有源记录改变 Z；没有新造点或修改 XY。原图 17,841 个近垂直结构点无改动。
- 保留旧 5,950 条已分类支撑；完整映射 6,550 条源记录，修改行在输出 PCD 中逐条命中。
- 1,644 个真实合法支撑面的选点／路线高度差：最大 7.43 cm → 0。
- 四条正反向路线、850 个生产参考窗口全部取得支撑，覆盖楼梯入口、平台、出口；机身高度仅补偿一次。
- 合并定向测试 345 通过、1 跳过。首次测试收集缺少 ROS overlay 的失败日志保留，补齐环境后重新执行，未修改业务代码规避测试。
- 危险反例包含柱体、缺支撑、净空不足、不同高度台阶／楼层、无映射观测和伪造输出坐标；均不因规整被开放。

## 边界

窗口验证使用生产参考管理器，状态是沿结果路线采样的合成位置，不是机器狗运动反馈。本轮没有运行实时 SCAN、重放整包、启动导航或控制机器狗，因此不能据此宣布实机上下楼已验收。

先前实测“机身位置被判占据”的原始回波没有录包，本轮地图修复不能替代该故障的传感器归因；未清除近身占据、未修改未知空间语义或碰撞阈值。

## 证据与复现

证据根目录：`/home/dndx/d1max_nav_ws/experiments/stair_regularization_20261026/`。

- `regression/comparison_slice_stable_final.json`：原生十组前后对照、平地一致性。
- `regression/reference_windows_slice_reviewed.json`：850 个窗口及源点身份。
- `geometry_comparison_slice_stable.json`：点云、PCT 地面起伏及修改范围。
- `selection_mapping_review_after.json`：3D 选点与路线一致性。
- `independent_preprocessing_review.json`：独立审查及已修遗漏。
- `regression/combined_overlay_final.xml`：合并单测。
- `regression/installed_final/`、`regression/installed_vs_candidate.json`：实际默认安装代码／地图重跑十组，全部成功，十条路径哈希逐条等于候选结果。
- `deployment/`：精确安装计划、原默认包备份、重定向与数值一致性证明。

已同步同一默认目录 `experiments/navigation_runtime`，两个默认选择器共同指向清单 `3b3dea33570e1cd75a169bad4fad63395445cd3cbb2e0be8b8ab0ac0d369f276`。未新增入口，未启动导航或重连 SDK。源码快照、实际 Python 安装副本及地图一起更新；控制器和 SCAN 二进制、参数保持原值。

封存时发现两个既有楼层支撑文件首次成为本地依赖，以及一个 10 月 23 日旧字节码缓存。保留原安装证据，用 `finalize_default.py` 精确补入两个已核对哈希的依赖，并将该缓存移至 `deployment/before_finalize/` 可恢复备份后完成封存；没有通过删除依赖检查放行。

在仓库根目录运行以下离线命令，不启动 ROS 图或 SDK；输出须使用新目录，避免覆盖证据：

```bash
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 experiments/stair_regularization_20261026/regression/pct_candidate_v2.py \
  --python-source installed \
  --map-directory experiments/navigation_runtime/map/crossfloor_20261002 \
  --output /tmp/d1max-pct-stair-new-check

/usr/bin/python3 experiments/stair_regularization_20261026/finalize_default.py --verify
```

地图生成器保留配置与逐点修改证据；不要用当前已规整包再次累积修正。需要重建时，以 `deployment/before/artifacts/map/crossfloor_20261002` 中的变更前文件结合未变更的包依赖恢复完整基线，再运行 `tools.pointcloud_preprocessing.stair_regularization_runner`。

## 追加：楼梯可通行轮廓的规整

本段对应后续“楼梯可通行部分形状崎岖”的修复，证据目录为
`experiments/stair_corridor_20261026/`；不覆盖上述前一轮结果。

### 实际原因

旧处理只接受距离旧可通行支撑点 16 cm 内的一跳邻居，导致一个真实且连续的楼梯支撑小片被漏选。
例如源记录 881288 的法向、残差和范围均合格，但距离旧种子 21.15 cm；它通过两个分别为
15.03 cm、15.83 cm 的真实邻接边与已有支撑连接。因此问题是离线支撑选取不完整，不能靠
降低所有障碍代价或涂绿地图修复。

### 修改边界

- 沿同一梯段的实测点邻接关系扩展支撑，保留原 16 cm 邻接距离、法向、8 cm 残差和边界条件。
- 仅修正上梯段漏选的 8 条原始记录，最大 Z 修正 6.98 cm；XY 和其他字段不变，没有增加或删除点。
- 旧 6,550 条映射、旧 1,805 条高度修正逐字段不变；映射增加至 6,558 条。
- PCT 从新的真实 PCD 重新生成，不单独修改代价值或显示颜色；墙、扶手、天花板及楼梯外地图不变。
- 不将规划包络当作物理斜坡或雷达观测，原始定位 PCD、实时 SCAN 和控制器保持原值。

| 上梯段可通行轮廓指标 | 修改前 | 修改后 |
|---|---:|---:|
| 唯一物理支撑格 | 235 | 249 |
| 最窄轮廓宽度 | 0.492 m | 0.728 m |
| 沿梯段宽度变异系数 | 0.1524 | 0.0831 |
| 轮廓最大侧向跳变 | 0.353 m | 0.157 m |
| 异常相邻地面高度跳变 | 0.206 m | 0.090 m |

这些是 PCT 支撑轮廓统计，不是楼梯实体净宽或机器人安全余量。原有已知格、可通行格无损失；
新增 74 个 layer 格对应 14 个唯一物理 XY 支撑格，不能把重复 slice 当成额外通道面积。

### 路线不因地图补齐而退化

同输入验证发现，直接重新运行原生 GPMP 会选择另一条更贴墙的楼梯曲线；仅统一同高层号也未完全解决。
这两种试验均未部署。最终把原先较好的 **原生 PCT 已发布楼梯折线** 编译进同一地图包：

- 在新地图及楼梯掩膜上重新逐弦检查，绑定当前地图、参数、入口／平台／出口及原始数据身份。
- 保存的是折线，不继承旧五次曲线或动力学连续性证明；平台采用原路线已验证的实际连接点。
- 仍由同一全局规划服务连接平地段；固定楼梯段无需再次搜索，异常清缓存后也不会变回另一条曲线。
- 任意楼梯中途目标仍走原生 PCT，未用固定完整连接段的结果冒充任意目标质量证明。
- 实时障碍仍由 SCAN 处理；“通常没有杂物”没有转化为忽略实时障碍的规则。

候选完整回归 10/10 成功，十条发布路线的 XYZ 和层编号与原稳定版本逐点完全一致；
另四个楼梯中途起终点请求 4/4 成功。366 项定向软件测试通过。原生十组对照包含
六条平地路线、完整跨层双向和楼梯入口到出口双向，未运行 ROS 图、SDK 或物理运动。

关键证据：`bridge/compiled_candidate_audit.json`、`geometry/connected_support_patch_review.json`、
`regression/final_comparison.json`、`regression/final_map_comparison.json`、
`regression/final_native/`、`regression/partial_final/`、`regression/software_final.xml`。

首次合并测试命令引用了不存在的测试文件，未执行任何测试；该记录保存在
`regression/software_initial_missing_path.xml`。更正为实际测试文件后，366 项全部通过。

### 当前默认包与复现

本次已同步源码快照、实际安装模块和地图，当前默认清单为
`2fa4f254c3e1a90ddd6cf02d572562e2d297909787afff2389188d7d98f317ec`。
两处默认选择器一致，38 个实际替换目标及完整清单核验通过；变更前文件保存在
`experiments/stair_corridor_20261026/deployment/before/`。没有新入口、分支、服务启动或 SDK 重连。

安装后直接加载默认安装代码和默认地图重跑 14 组请求，全部通过，路线 XYZ/层号逐项等于候选结果。
四条跨层／楼梯路线的 850 个生产参考窗口全部取得原始支撑，入口、平台、出口切段完成。
这些窗口使用沿路线采样的合成位置，不是实机运动、传感器时序或动态避障验收。

- 实际安装验证：`regression/installed_final/`、`regression/partial_installed/`。
- 安装结果对照：`regression/installed_vs_candidate.json`、`regression/partial_installed_vs_candidate.json`。
- 当前窗口证据：`regression/final_reference_windows.json`；同目录旧 `reference_windows.json` 属于被否决试验，不用于本次结论。
- 重建命令与封存输入：`bridge/reproduction.json`。部署后已实际重建，PCD、映射、PCT 数值及连接段全部一致，见 `bridge/reproduction_executed.json`；旧文件中的 `command_executed=false` 是命令准备时的历史记录。

从仓库根目录核对当前默认安装，不启动任何运行服务：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/src/d1max_pct_planner:$PWD/src/d1max_pct_scan:$PWD" \
/usr/bin/python3 experiments/stair_corridor_20261026/sync_default.py --verify
```

部署准备阶段曾引用到旧试验的窗口证据，已在安装前中止该冻结并保留到
`deployment/aborted_evidence_freeze/`；随后改为显式核对候选身份与实际成功结果，再重新冻结和安装。
没有把被否决地图部署到默认包。

## 追加：楼梯居中与倾斜姿态下的转向修正

对应“楼梯尽量走中间、避免左右摇摆”。本段为上述规整包之后的新一轮，证据在
`experiments/stair_centering_20261026/`。不把离线几何改善当作实机无摆动证明。

### 三层修改

1. **固定全局参考**：从保留的墙、扶手垂直结构提取两侧位置，离线拟合梯段中线。
   仅调整原 737 点中两段梯内的 283 点 XY；最大横移下段 13.80 cm、上段 8.93 cm。
   Z、层归属、入口、平台、出口及梯段端点切向保持；原 PCD 和 PCT 所有数值通道逐字节不变。
   拟合不进入实时循环，不因每帧点云变化移动全局路线。
2. **SCAN 接入**：去掉楼梯参考必须近似恒定斜率的错误假设，沿实际参考的分段高度生成前向接入。
   由真实支撑归属互斥选择平地或梯段初始化，避免同一窗口被连续整形两次。
   保留实测起点、原尾部、P/V/A 边界及原生碰撞检查，不清除台阶或未知体素。
3. **跟踪转向**：修正倾斜机身时把世界坐标角速度 Z 分量当作航向变化率的错误。
   用机身前向轴的水平投影计算航向变化率，避免侧倾运动产生虚假的左右转向阻尼。
   原始角速度、源时间、停稳证据、控制增益、限幅和平滑器均不变。

### 同输入结果与边界

| 梯段指标 | 修改前 | 修改后 |
|---|---:|---:|
| 下段平均离中线距离 | 14.26 cm | 7.77 cm |
| 下段峰值曲率 | 0.697/m | 0.227/m |
| 上段平均离中线距离 | 6.92 cm | 6.31 cm |
| 上段累计转向 | 103.51° | 50.74° |
| 上段峰值曲率 | 3.906/m | 1.110/m |

连续路线到实际墙栏回波的最小 XY 距离两段均改善；该值是中心线到有限回波的距离，
不是完整机身安全间隙。下段原入口和出口偏侧，为柔和回到中间，总转角从 6.47° 增至约 27°，
但峰值曲率下降；不能宣称所有角度指标都下降。平台转弯未改，也未锁死机器人航向。

- 实际 PCT 与原始支撑验证：完整/平地/双向 10 组，楼梯中途起终点 4 组，全部通过。
- 6 条平地路线以及跨层路线中的平地段逐点不变；4 条楼梯相关路线的 844 个生产参考窗口通过。
- 原生 GridMap 与固定 P/V/A 曲线反例：旧版 7/9，新版 9/9；独立复核通过。
- SCAN 41/41、跟踪器 10/10 CTest 通过；其中跟踪核心 114/114，新建来源合同等 Python 测试 93/93。
- 8,100 条同输入平地控制输出逐位一致；倾斜航向的 3 个旧版失败反例在新版通过。
- 新居中路线另取 196 个显式侧偏状态，核对前向接入种子；未将这些合成状态当作真实反馈，
  未将种子曲线当作最终 SCAN B 样条或 SDK 运动验收。

### 默认包

同一默认目录已同步 26 个源文件快照、安装模块、头文件、三个实际 ELF 及地图路线产物；
默认清单 SHA 为 `4c985adb4be073f531ed1f666e7b377beef0d59bd923eb50acaebf25ddab2043`。
两处默认选择器一致。备份在 `experiments/stair_centering_20261026/deployment/before/`，
支持按完整清单人工恢复；不是自动回滚事务。

没有启动导航、RViz 或运动；既有 SDK 会话、SDK 二进制、地图数值、障碍膨胀及验收标志保持不变。
仍需现场确认上下楼实际摆动、传感器时间和动态障碍表现，尤其不能把本轮数学修正认定为全部现场摆动的唯一原因。

当前版本核对命令（旧轮 `--verify` 绑定旧默认，不再用于核验当前包）：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/src/d1max_pct_planner:$PWD/src/d1max_pct_scan:$PWD" \
/usr/bin/python3 experiments/stair_centering_20261026/sync_default.py --verify
```

同输入证据索引：`reviewed_evidence.json`；几何对照：`regression/reference_geometry_comparison.json`；
控制对照：`control/heading_rate_regression.json`；原生构建与依赖：`native_build/provenance.json`。

安装后直接使用默认安装 Python 与默认地图重新计算 14 组请求，全部成功，XYZ 和层号逐项等于
审查过的候选；26 个实际安装目标及完整清单复核通过。记录为
`regression/installed_verification.json`，没有以工作区导入代替已安装代码验证。
