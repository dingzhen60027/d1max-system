# D1 Max 跨楼层 PCT 全局规划 —— 阶段性交接文档（2026-09-23，第 2 版）

> **一句话状态**：还没有跑通完整的四段路线。目前最好的变体 **W** 已连续通过前三段（lower_floor、stair_lower、stair_upper），在最后一段 upper_floor（二楼走廊）A* 失败。
>
> 本阶段按用户要求**停止修复，只记录问题**。第 5 节列出全部未解决问题和证据，第 8 节给出建议的排查顺序。
>
> 本目录所有脚本都是**离线实验**：只读地图，只在内存中重建 tomogram，不修改正式配置和 map_manager 数据，也不向机器人发送任何东西（`planning_only`，`execution_authorized: false`）。

---

## 0. 各段当前状态总览（最重要）

| 变体 | 点云清洗 | 删点数 | lower_floor | stair_lower | stair_upper | upper_floor |
|---|---|---|---|---|---|---|
| A（现行正式配置） | 无 | 0 | ✔ | ✘ A* | 未跑到 | 未跑到 |
| O | ROR 0.15/8 | 78282（7.8%） | ✔ | ✔ | ✘ 曲线（平台鬼影） | 未跑到 |
| R | 可见性 10/20，球形保护 0.12 | 3370 | ✔ | ✔ | ✘ A*（楼梯顶鬼影，见问题 1） | 未跑到 |
| **W** | **可见性 10/20，不保护** | **7329** | ✔ | ✔ | ✔ | **✘ A*（二楼走廊，见问题 2）** |
| V / Y / Z | 可见性，保护收紧 | 4564–6551 | ✔ | ✘ 曲线（见问题 3） | 未跑到 | 未跑到 |

表格说明：
- 共同参数：res 0.10，slope 0.60，safe_margin/inflation 0.10。完整结果见第 4 节。
- **注意**：`plan_crossfloor` 按顺序规划各段，遇到第一段失败就停止。因此 **upper_floor 只在 W 中被测到过**，其他变体里这一段的情况未知。
- 用 `upper_frontier.py` 可以对单独一段跑 A*（见第 7 节），不受前面各段的影响。

## 1. 任务与约束

- 目标：一台 D1 Max 四足，在 SC-PGO 优化地图上用 PCT 规划一楼 → 楼梯 → 二楼的全局路径。
  - 前端是 faster_lio，后端是 SC-PGO 回环。之前的单层方案可以作参考。
- 楼梯尺寸：级高 14 cm，踏面 30 cm，坡度 atan(0.14/0.30) = **0.437 rad（25°）**。
  - 整体高度接近 3.6 m。实测下层地面 ≈ **-0.63**，平台 ≈ **1.30**，上层地面 ≈ **3.15**，层间约 3.78 m。
  - 锚点 Z 是**地面高度**，不要改成 3.6。
- 输入点云（SC-PGO 09-23 13:26，已经过 map_manager 处理）：
  `…/d1max_ros2/map_manager/data/processed/20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd`（1,078,761 点）。
- 原始建图 run：`/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/`
  - `sc_pgo/Scans/NNNNNN.pcd`：986 个关键帧，IMU/body 系 C。
  - `sc_pgo/optimized_poses.txt`：每行一个 3×4 位姿。
  - `config/calibration.yaml`：`lio_extrinsic`。
- 路径说明：`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_nav_ws` 是指向 `/home/dndx/d1max_nav_ws` 的软链接。
- 录制轨迹关键帧参考：

  | 关键帧 | 位置 / 动作 |
  |---|---|
  | 720–768 | 一楼走到楼梯脚 |
  | 768–822 | 下梯段 |
  | 822–882 | 平台掉头 |
  | 882–936 | 上梯段 |
  | 936–954 | 楼梯顶 → exit |
  | 954–985 | 二楼走廊 → goal |

## 2. 相关代码与配置（本阶段均未修改）

| 内容 | 路径 |
|---|---|
| 规划器包 | `/home/dndx/d1max_nav_ws/src/d1max_pct_planner/d1max_pct_planner/` |
| 分段规划 | `crossfloor_route.py`：`LEGS`，以及 `plan_crossfloor` 的循环（约第 279 行，按段依次执行） |
| A* + GPMP 曲线逐格校验 | `tomogram_route.py`：`expand_native_curve`、`TomogramRoute.plan` |
| CPU 建 tomogram | `cpu_tomography.py`：`tomogram_from_points`、`traversal_cost`、`_inflate` |
| 路线配置 | `src/d1max_pct_planner/config/crossfloor_route_20260923.yaml` |
| 正式 tomogram 配置 | `src/d1max_pct_planner/config/official_crossfloor_20260923.yaml`（res 0.20，slope 0.40，m/i 0.20） |
| 原生 PCT（vendor） | `/home/dndx/d1max_nav_ws/src/pct_planner_vendor`。GPMP 每约 10 个 A* 节点采样一次，改这个需要重编 C++ |

四段路线：

| 段 | 起点 → 终点 | 锚点 XYZ | 层 |
|---|---|---|---|
| lower_floor | start → entry | start [-25.32, 45.76, -0.68]，entry [-30.64, 47.94, -0.62] | L4 |
| stair_lower | entry → landing | landing [-28.74, 55.05, 1.37] | L4 → L8 |
| stair_upper | landing → exit | exit [-30.67, 48.21, 3.15] | L8 → L12 |
| upper_floor | exit → goal | goal [-25.69, 45.88, 3.13] | L12 |

锚点处理：
- 实验中锚点会吸附到 0.4 m 内、地面高差 ±0.15 m 以内的有效格，实际移动 ≤ 0.14 m。
- 在 W 中吸附后的 goal 为 (-25.80, 45.75, 3.129)。

规划器严格规则（**不要为了让它通过而放宽**）：
- 楼梯段 `path_refinement='none'`。
- GPMP 五次曲线只要经过任何一个 cost > 20 的格子，就报 `invalid_curve / pct_cost_blocked`，refinement 不能把它救回来。

PCT 规则速记：
- tomogram 数据 `data[5,L,X,Y]` = [膨胀后 cost, dx, dy, ground, ceiling]。
- 切片顶 = `-3.0 + 0.5 + 0.5·L`（L4 = -0.5，L8 = 1.5，L12 = 3.5）。每格 ground 取切片顶以下的最高点。
- flat：grad² ≤ (1.2·res·tan(slope))²。
- crossable：相邻高差 ≤ 0.17，且 7×7 邻域内至少 8 个 flat。
- 既不 flat 也不 crossable 记为 S；净空 < 0.55 记为 I，两者都算障碍。
- 膨胀（m/i = 0.1）：距离 ≤ 0.1 → 50，0.2 → 25，0.3 → 0。
- A* 可通行阈值为 cost ≤ 20。

## 3. 已确认的根因与证据

### 根因 1：坡度阈值低于楼梯坡度（已解决）
- 楼梯实测坡度 0.437–0.447 rad，而 `slope_max_rad = 0.40`，整段梯段被判为 S。
- 调到 **0.55–0.60** 后梯段正常。
- 证据：`stair_profile.py`、`render_stair.py`、`stair_r10.png`。

### 根因 2：跟随机器人的操作人员留下的鬼影点（清洗方法已做出，参数未定）
- 在累积 PCD 中，这些点在楼梯脚、平台、楼梯顶形成稀疏的"墙"。
- 按关键帧逐帧统计，确认是人：

| 位置 | 范围 | 有点的关键帧 | 其余时间 |
|---|---|---|---|
| 楼梯脚鬼影盒 | x -31.2..-30.0，y 48.9..50.6，z -0.5..0.1 | 只在 kf 738–822（上楼回看时） | — |
| 平台 S 形块 | x -29.7..-29.1，y 54.5..55.2，z 1.42..2.8 | 只在 kf 819–843、858、930–936 | kf 846–855、861–885 期间邻近地面每帧 4000+ 点，但该块为空 |

- 可见性投票：
  - 鬼影：F/H ≈ 60–110。
  - 静态结构：一楼地面 F/H 中位数 1.15，平台地面 F 中位数 0，踏面 F/H 0.72。
- map_manager 的 `pipeline.yaml` 也写明，单个累积 PCD 分不出密集鬼影，所以必须用关键帧射线做 free-space 投票。

## 4. 参数扫描完整结果（`native_sweep.py`，真实原生 `plan_crossfloor`）

CROP = [-40,-4,-3] → [26,60,9]；"m/i" 指 safe_margin / inflation。
可见性规则写作 `vis(ratio, fmin, protect[, protect_dz])`，含义见第 6 节。

| 变体 | 设置 | 结果 |
|---|---|---|
| A | 现行：r.20 slope.40 m.20/i.20（WIDE） | stair_lower A* 失败 |
| B / C | r.20 slope.60，m.20/i.20 或 m.10/i.10 | stair_lower A* 失败 |
| D | r.10 slope.40 m.20/i.20 | landing 无有效格 |
| E / F / G | r.10 slope.40，m.10/i.15、m.10/i.10、dh.25 | stair_lower A* 失败 |
| H / I / J | r.10，slope .55/.60/.65，m.10/i.10 | stair_lower A* 失败（鬼影堵楼梯脚） |
| K / M | r.10 slope.60 m.20/i.20；r.15 slope.60 m.15/i.15 | landing 无有效格 |
| L | r.10 slope.60 m.15/i.15 | stair_lower A* 失败 |
| N / Q | ROR 0.15/8，slope .55 / .40 | stair_lower A* 失败 |
| **O** | slope.60 m.10 + ROR 0.15/8 | stair_upper invalid_curve：曲线切过平台 S 形鬼影块，L8 [109,595] |
| P | slope.60 m.15 + ROR | stair_lower invalid_curve，L4 [95,532]（楼梯脚） |
| **R** | slope.60 m.10 + vis(10,20,0.12) | **stair_upper A* 失败**（问题 1） |
| S | slope.60 m.15 + vis(10,20,0.12) | stair_lower invalid_curve，L4 [99,532]（楼梯脚） |
| T | slope.55 m.10 + vis(10,20,0.12) | stair_lower invalid_curve，L4 (-29.26, 50.40, -0.63)，cost 50 |
| U | slope.60 m.20 + vis(10,20,0.12) | goal 无有效格（m.20 下二楼走廊太窄） |
| V | slope.60 m.10 + vis(10,20,0.06)，删 6551 | stair_lower invalid_curve，**L7 (-28.77, 53.40, 0.92)**，cost 50（问题 3） |
| **W** | slope.60 m.10 + vis(10,20,0)，不保护，删 7329 | 前三段 ✔；**upper_floor A* 失败**（问题 2） |
| X | slope.60 m.10 + vis(6,20,0.06)，删 8864 | stair_lower invalid_curve，L5 (-29.41, 51.50, -0.07)，cost 50 |
| Y | slope.60 m.10 + vis(10,20,0.12,dz 0.05)，删 4564 | stair_lower invalid_curve，与 V 同一格 L7 (-28.77, 53.40) |
| Z | slope.60 m.10 + vis(10,20,0.12,dz 0.03)，删 5231 | 同上 |

日志文件：`sweep_NOPQ.log`、`sweep_VWX.log`、`sweep_YZ.log`。A–M 和 R–U 的结果只在终端输出过，已整理到上表。

## 5. 未解决问题（详细记录，均未修复）

### 问题 1：楼梯顶残留鬼影点被"保护"下来，R 在 stair_upper 被挡

- **现象**：
  - R 的 stair_upper A* 失败。A* 访问集能爬上上梯段，最高到达 ground 2.8–3.4，范围 x[-31.40, -30.70]、y[51.35, 51.75]，也就是最后一级踏步。它没能上到二楼楼板（y < 51.35）。
- **沿轨迹逐格数据**（`upper_frontier.py R stair_upper 828 956 3`）：

  | 关键帧 | 位置 | L12 ground | cost | 状态 |
  |---|---|---|---|---|
  | kf924 | (-31.05, 51.52) | 2.91 | 19.1 | 有效，已到达 |
  | kf927 | (-31.10, 51.34) | 3.13 | **50** | 无效 |
  | kf930 | (-31.15, 51.08) | 3.17 | 19.1 | 有效，未到达 |
  | kf933 | (-31.17, 50.88) | 3.26 | 10.4 | 有效，未到达 |

- **规则码**：R 的 L12 在楼梯顶 x -31.0 ~ -31.1、y 51.1 ~ 51.5 是一排 S 和膨胀格 `@`（`codes_cached.py R 11,12 -31.6,50.3 -30.5,52.0`）。
- **原因**：格子 (-31.00, 51.15) 里有一个 **z = 3.24 的点，F = 123，H = 3**，是典型的鬼影，比楼板 3.15 高 9 cm。
  - 这个格的 ground 因此变成 3.24，和旁边 3.05（踏步沿）、3.15（楼板）形成高差，被判为 S，再加膨胀，就把楼梯顶封住了。
  - 这个点满足删除候选条件，但 **0.12 m 球形保护**的邻域里有楼板持久点（z 3.15，H ≥ 45），所以没被删。
  - 同格还有 3.18/3.16 两个点（F = 122，H = 18，F/H ≈ 6.8 < 10），不是候选。
- **旁证**：
  - O（ROR）删掉了该区域 3 个点（z 2.88–3.29），A* 能过楼梯顶。
  - W（不保护）也通过了 stair_upper。
- **尝试过的方案**：椭球保护（水平 0.12 × 垂直 0.05 或 0.03，变体 Y/Z），思路是只让"同一表面"的点来保护候选。但 Y/Z 先在 stair_lower 撞上了问题 3，所以**还不知道椭球保护能不能解决楼梯顶**。
  - 可以用 `upper_frontier.py Y stair_upper …` 单独验证 stair_upper 的 A*。

### 问题 2：W 的二楼走廊 upper_floor A* 失败（原因未查明）

- **现象**：
  - W 的 upper_floor 只在 L12 上搜索，访问 1436 格，范围 x[-31.80, -26.60]、y[44.35, 51.75]，到不了 goal (-25.80, 45.75)。
- **沿轨迹逐格数据**（`upper_frontier.py W upper_floor 948 985 2`）：
  - kf952 (-30.97, 49.06)：cost 50，但旁边有路能绕过。
  - kf972 (-27.23, 46.21)：已到达。
  - **kf974 (-26.23, 45.99)：cost 50，无效**。
  - kf976 (-25.18, 45.78) 和 kf980 (-23.16, 45.40) 有效，但没到达。
  - 结论：大约在 **x ≈ -26.2 ~ -26.6** 有一道横跨走廊的障碍线。
- **规则码**（`codes_cached.py W 12 -27.6,44.6 -24.8,47.2`）：
  - 走廊中间（y 45.3–46.2）散布着许多 1–5 格的 **S 小岛**，每个周围有 `,`（flat 但膨胀后 > 20）。
  - 典型位置：(-26.6 ~ -26.4, y 45.8–46.0)、(-26.3, 45.3–45.5)、(-26.0, 44.9–45.3)、(-25.8 ~ -25.6, 45.8–46.1)。
  - 在 m/i 0.10 下，每个小岛会封住约 0.3 m 的圆，这些小岛连成一串，封住了走廊。
- **已知数据**：
  - box x[-27.2, -25.4]、y[45.2, 46.8] 内共 631 个点，**全部在 z 3.1–3.3**，z > 3.3 一个都没有，所以不是悬空的高鬼影。
  - W 规则在这个 box 里只删了 7 个点，都在 z 3.25–3.27，F ≥ 10H。
- **可能的原因（都未验证）**：
  - (a) 贴地的低矮临时点。例如操作人员的脚，比楼板 3.13 高 12–17 cm，造成 ground 高差超过 0.17，形成 S。
  - (b) 多次经过没对齐，形成双层地面。
  - (c) 可见性投票的 ROI 到 x = -25.5 为止，goal 附近和更远的二楼走廊（x > -25.5）完全没清洗。
  - (d) 这道障碍在未清洗的地图上本来就存在。之前的变体都没跑到 upper_floor，所以不知道。
  - (e) 被 W 的激进删除（不保护）间接造成。
- **还没做的对照**：
  - 在 R 的缓存上单独跑 upper_floor：`upper_frontier.py R upper_floor 948 985 2`（`cache_R.npz` 已存在）。
  - 在未清洗的 I 上也跑一次：先 `curve_diag.py I upper_floor` 生成缓存。
  - 目的是判断 (d)/(e)。

### 问题 3：楼梯段 GPMP 曲线很脆弱，invalid_curve 的位置随参数跳动

- **现象**：
  - A* 能找到路径，但五次曲线会切到楼梯侧边或楼梯脚的 cost 50 格。
  - 失败格随变体变化：

    | 变体 | 失败格 |
    |---|---|
    | P | L4 [95,532] |
    | S | L4 [99,532] |
    | T | L4 (-29.26, 50.40) |
    | V / Y / Z | L7 (-28.77, 53.40, 0.92)，下梯段中段 |
    | X | L5 (-29.41, 51.50)，第一级附近 |

- **非单调**：
  - W 删掉的点包含 V/Y/Z 删掉的所有点，W 的 stair_lower 通过，V/Y/Z 却失败。
  - R 删得最少，stair_lower 反而通过。
  - 可见结果对地图的小变化很敏感，不是简单的"多删一些就好"。
- **可能的原因**：
  - m/i 0.10 下梯段可用走廊很窄。
  - GPMP 大约每 10 个 A* 节点（0.1 m 分辨率下约 1 m）采样一次，拟合出的五次曲线容易切角。
  - 另外，V/Y/Z 在 (-28.77, 53.40) 附近可能保留了楼梯侧的鬼影点，而 W 把它们删了。
- **尚未诊断**：`curve_diag.py Y stair_lower` 可出图；`codes_cached.py Y 6,7,8 -29.5,52.5 -28.2,54.5` 可看逐格规则。
- **可选方向**（都需要负责人决定，不能私自放宽严格规则）：
  - 调整 `astar_cost_weight` 或 `optimizer_cost_margin`，让 A* 更靠走廊中线。
  - 改 vendor GPMP 的采样间隔，需要重编 C++。
  - 把楼梯两侧残留的鬼影清干净，让走廊变宽。

### 问题 4：可见性清洗会误删墙体和楼板边缘（W 最严重）

- 不保护的规则（W，删 7329 点）会删掉一些墙状条带（`votes_r10.png`）。
  - 下层：x -33 ~ -34 / y 48–49.5；门口 y ≈ 48.2–48.8；x -33 / y 46。
  - 上层：z ≈ 3.1 楼板边 y 44–45.5，以及 z ≈ 4.7 的一簇。
- 加 0.12 m 保护后（R，删 3370）只剩少量误删，但又带来问题 1。
- **风险**：如果某段墙被大面积删掉，tomogram 里可能出现"穿墙"通道。**W 被正式采用前，必须人工复核这些删除**。可以用 `render_votes.py 10 20 out.png 0` 出图。

### 问题 5：m 0.15 / 0.20 的余量还不够

- m/i 0.15（P、S）在楼梯脚被挡；0.20（U、K、D、M）连锚点都找不到有效格。
- 目前能通的都用 m/i 0.10。这意味着机体离障碍只留约 10–20 cm 的膨胀余量。**是否可以接受，要结合 D1 Max 的机身宽度和控制误差来确认**。

### 问题 6：尚未验证的项

- `_check_stair_profile` 等全路线检查还没执行过，因为没有一个变体四段全部通过。
- 正式流程（新 processed map，正式 tomogram 配置，route_trial 审计 `ok`）尚未开始。
- 小 bug（不影响结论）：
  - `final_check.py` 在文件关闭后访问 `a['center']`。
  - `upper_frontier.py` 的 KF1 必须 ≤ 985（位姿共 986 行），否则会报 IndexError。

## 6. 可见性清洗（`visibility_clean.py` + `native_sweep.visibility_keep`）

- **传感器原点**：机身前后各一台半球 RoboSense Airy（96 线）。
  - 前雷达在 C 系下为 `t_C_N` = [0.0109, -0.366, 0.0026]（calibration.yaml `lio_extrinsic`）。
  - 后雷达为 `R_CN·[-0.003,-0.7323,0.003] + t_C_N` ≈ [-0.014, 0.366, -0.007]（localization.yaml `rear_to_front_translation`）。
  - 机身前向为 C 系的 -y。
- **点归属**：y < -0.366 的点归前雷达，y > 0.366 的归后雷达，中间的点（0–0.2%）跳过。
- **投票**：
  - ROI 为 [-34.5, 44, -1.4] → [-25.5, 57.5, 4.9]（**注意 x 上限 -25.5，没有覆盖 goal 以东的二楼走廊**），体素 0.10 m。
  - 用原点距 ROI 15 m 以内的关键帧，共 427 帧（kf 248..985）。
  - 射线裁剪到 ROI，从 max(进入点, 0.30 m) 开始，到 (L - 0.15 m) 结束，每 0.04 m 采样一次。
  - 每帧对每个体素只计一次 free / hit。
- **输出**：`visibility_votes.npz`，每点一个 `free / hit / in_roi`，与 processed_map.pcd 的点顺序一致。运行约 119 s。
- **删除规则** `vis(ratio, fmin, protect[, protect_dz])`：
  - 候选：`in_roi & F ≥ ratio·max(H,1) & F ≥ fmin`。
  - 保护：候选点 `protect` 米内有持久点（`H ≥ 10 & F < 3H`）就保留。
  - 给了 `protect_dz` 时，保护邻域改为椭球：水平半径 protect，垂直半径 protect_dz。
- **已试过的组合**：

  | 规则 | 删点数 |
  |---|---|
  | (10,20,0.12) | 3370 |
  | (10,20,0.06) | 6551 |
  | (10,20,0) | 7329 |
  | (6,20,0.06) | 8864 |
  | (10,20,0.12,0.05) | 4564 |
  | (10,20,0.12,0.03) | 5231 |

- **目前没有一组规则同时满足"楼梯顶鬼影被删（问题 1）"和"不误删墙（问题 4）"**。

## 7. 脚本清单（本目录）

| 脚本 | 用途 / 用法 |
|---|---|
| `native_sweep.py [变体…]` | 主扫描：内存建 tomogram → 锚点吸附 → 真实 `plan_crossfloor`。<br>override 支持 `ror`、`vis`、`anchor_xy`。变体 A–Z 定义在 `VARIANTS` 中。<br>会自动用原生环境 re-exec；import 前设 `SWEEP_CHILD=1` 就不会触发。<br>每个变体都要重建 0.1 m tomogram，需要几分钟，建议加 `timeout`。 |
| `upper_frontier.py 变体 段 KF0 KF1 [步长]`（新） | **对单独一段**跑原生 A*（不受前面各段影响），打印访问集的高度分带范围。<br>同时沿录制轨迹打印每层的 ground / cost / 有效性 / 净空，R 表示已被 A* 访问。 |
| `curve_diag.py 变体 段… [--landing=x,y]` | 单段诊断：A* 节点、曲线逐格校验、完整 plan，出图 `curve_<V>_<leg>.png`。<br>tomogram 缓存为 `cache_<V>.npz`，已支持 `vis` 变体。<br>**改了变体参数要删掉对应缓存**。当前已有缓存：O、R、W，每个约 19 MB，可以删。 |
| `codes_cached.py 变体 层 lo_x,lo_y hi_x,hi_y` | 在缓存 tomogram 上逐格打印规则字符：`.` flat、`c` crossable、`,`/`x` 膨胀 > 20、`I` 净空、`S` 坡度/台阶、`h` 有效性掩码拒绝；轨迹格用 `#@!` 标出。 |
| `visibility_clean.py [out.npz]` | 关键帧射线 free/hit 投票（第 6 节）。 |
| `render_votes.py ratio fmin out.png [protect]` | 被删点的俯视（三个 z 段）和侧视，外加 log(F/H) 直方图。 |
| `stair_profile.py`、`render_stair.py` | 梯段 Z 剖面和 raw/膨胀 cost 图（根因 1）。 |
| `foot_codes.py`、`foot_bands.py`、`render_foot.py`、`corridor_side.py` | 楼梯脚和走廊的点高度分带、逐格规则（根因 2）。 |
| `diagnose_stair_cells.py`、`stair_connectivity.py`、`isolate_blocker.py`、`native_frontier.py`、`reach_replica.py` | 早期诊断脚本。 |

图片：
- `stair_r10.png`、`foot_*.png`、`side_lower.png`：根因证据。
- `votes_r10.png`、`votes_r10_p12.png`：清洗效果。
- `curve_O_*.png`：O 的曲线诊断。

## 8. 建议的下一步（按优先级）

1. **先判断问题 2 的性质**，这一步最省时间，缓存已有：
   - `python3 upper_frontier.py R upper_floor 948 985 2`
   - 再对未清洗的变体（如 I）做同样检查。
   - 如果 R/I 的 upper_floor 能通，说明是 W 的激进删除造成的；如果不通，就是二楼走廊本身的问题，按问题 2 的 (a)–(c) 排查：
     - 逐格列出点的 z，方法同问题 1 的逐点排查。
     - 把 `visibility_clean.py` 的 ROI 扩到 x ≤ -20，覆盖二楼走廊。
2. **验证椭球保护能否解决问题 1**：`python3 upper_frontier.py Y stair_upper 828 950 3`。
3. **诊断问题 3**：`curve_diag.py Y stair_lower`，看 (-28.77, 53.40) 挡住的是真实的扶手/墙，还是残留鬼影。
4. 选定一组清洗规则后，复核墙体误删（问题 4），并和负责人确认 m/i 0.10 是否可以接受（问题 5）。
5. 四段全部 `ROUTE OK` 后落地正式流程：
   - 把清洗后的点云做成新的 processed map，或者在 map_manager 里加一个 visibility 清洗步骤。
   - 修改 `official_crossfloor_20260923.yaml`：`slope_max_rad` 0.55–0.60，`resolution` 0.10（ROI 缩到 CROP 以控制内存），`safe_margin/inflation` 0.10。
   - 重建 tomogram，跑 `crossfloor_route` 并输出到 route_trial，确认审计结果为 `ok`。

## 9. 注意事项

- 不要为了"通过"去放宽 `expand_native_curve` 的逐格校验，或给楼梯段开 refinement。严格校验的目的是保证路径真正落在可通行面上。
- 锚点 Z 用实测地面高度（start -0.68，entry -0.62，landing 1.37/1.30，exit 3.15，goal 3.13），吸附容差 ±0.15 m。
- 不同变体的点集不同，tomogram 网格原点可能相差半格（例如 O 的 shape 是 634 列，R 是 639 列）。跨变体比较同一个格子时，要用世界坐标，不要直接用格子索引。
