# D1 Max PCT Planner

This package keeps upstream PCT Planner isolated in `pct_planner_vendor` and adds:

- CPU tomography for systems where the NVIDIA driver/CUDA is unavailable.
- Offline global path generation through the upstream C++ planner.
- ROS 2 map/path publication and RViz2 visualization.

The upstream project is GPLv2. This integration does not replace localization,
local obstacle avoidance, gait control, or footstep planning.

## 低资源全局规划（2026-09-25，当前实现）

本轮改的是数据存储、跨语言复制和重复计算，不是裁小地图、降低地图分辨率、
减少路径采样或放宽障碍/净空/跨层检查。Zenoh、定位与 SCAN 算法不变。

- A* 不再为整幅分层栅格预分配 `Node`。只为本次搜索触及的格子创建节点，
  成功和失败均释放；只保留独立的路径结果。大查询后的哈希桶会收缩。
- A* 与优化器共享只读 double 栅格；查询器 getter 不再复制整幅地图。
  一次 A* 直接供 GPMP 使用，取消为了获取元数据重复执行的搜索。
- Python→C++ 地图输入改用只读 stride 引用，避免六张 Eigen 临时整图；
  C++ 返回后仍独立持有数据，修改或释放 Python 数组不会改变原生地图。
  旧地图句柄采用共享所有权，重新加载地图后也不会留下悬空引用。
- 跨层四段路线只保留三份 lazy 地图资源；楼梯上下段共享同一遮罩。
  最多缓存两段**固定楼梯门户之间**的已验证曲线，普通起终点路线仍重新求解。
  缓存绑定不可变地图、完整配置和精确门户，命中后仍做原图/遮罩、拼接和高度检查。
- 路径校验去掉逐点小数组与诊断字典的无用构造；同层使用等价单状态检查，
  跨层保留状态机。批量地面检查每块最多 4096 点，仍覆盖格边两侧和角点。
  平滑复用的是几何计算，不复用上一请求的安全结论。
- 地图配置与运行时的未知顶空策略、最大台阶高度必须一致，否则初始化拒绝。

### CPU 与线程预算

独立配置：[config/compute_budget.yaml](config/compute_budget.yaml)。默认全局规划 worker
最多使用 **2 个逻辑 CPU、OpenMP 2 线程、BLAS 1 线程、nice 5**；不是独占两个物理核。
`cpu_ids: []` 从已允许的 cpuset 选择；部署时可以按 NUC 拓扑设置明确的逻辑 CPU ID。
`D1MAX_PCT_COMPUTE_CONFIG=/绝对路径/预算.yaml` 可选择另一份完整预算，重启会话后生效。

预算覆盖离线 RViz 预览与实时跨层全局规划的独立计算进程，不改变定位、局部规划、
ROS 主线程、RViz 或用户 shell 的 affinity/nice。数值线程环境在 spawn 前设置，
worker 还会检查并约束已存在的每条线程；宿主 OpenMP 绑核覆盖项不带入 worker。
spawn 环境修改在本包锁内执行并在异常时恢复；这不是对外部代码任意并发 spawn 的沙箱。

### 实图复测结果

同一台 i9-14900HX、同一张 0.1 m 分辨率的双楼层地图、相同端点与参数。
地图有 24 个高程切片（不是 24 个建筑楼层），每份地图 10,183,104 个栅格。
下表使用同一无 cProfile 的计时脚本比较；峰值是该离线进程，不是整个导航系统。

| 项目 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 83.04 m 跨层：加载地图 + 首次求解 | 8.07 s | 3.60 s |
| 83.04 m 跨层：重复求解 | 4.05 s | 0.93 s |
| 7.00 m 单层：地图已加载 | 522 ms | 46 ms |
| 同一用例序列的峰值 RSS | 7.42 GiB | 3.84 GiB |

另用默认 **2 逻辑 CPU / nice 5** 限额独立测试：跨层重复求解 0.897 / 0.897 s，
单层 45.3 / 46.0 ms。真实 `native_worker` 的两个线程均限制在 CPU 0/1、nice 5，
跨层重复请求 0.893 s；该 worker 常驻约 2.75 GiB、初始化峰值约 3.58 GiB。
父进程 affinity/nice 不变，测试 worker 正常退出。

跨层 3030 点和单层 227 点的 XYZ/层号哈希与优化前完全一致；
跨层仍检查 940 个层栅格、2029 次边界采样和 8 次层转换。
反向、变更终点与重新初始化的结果也与新实例逐点一致。
最终 `d1max_pct_planner/test` 与 `d1max_pct_scan/test` 共 **916 项通过**；
Python 包已重新安装，C++ 原生库已定向重新编译并通过 GTSAM ABI 核对。
本次最长的楼层查询实际创建 26,018 个搜索节点，而不是预分配整图千万个节点。
剩余重复规划时间主要在完整曲线展开、校验和走廊平滑；不能把几十毫秒的
native A* + GPMP 耗时冒充整个可交付路径的时延。

原始证据：`experiments/pct_latency_audit_20260925/report.json`；
最终证据：同目录 `final/report.json`、`crossfloor.json`、
`crossfloor_two_cpus.json`、`short_two_cpus.json`、`worker_budget.json`。

继续保持一个常驻计算进程、单请求串行、最新目标替换与超时回收机制。
取消/失败不能把旧结果提交给新目标。NumPy 字节数、数学栅格数、实际搜索节点数
在状态中分别报告，不能把 `owned_dense_grid_bytes=0` 误解成 native 总内存为零。

**部署边界：** affinity/nice 不是 cgroup 独占资源或内存硬配额。最坏不可达搜索仍可能
遍历全图；当前没有设置会改变可达性语义的隐式节点截断。NUC 上须结合实际 CPU、
内存和定位/局部避障并发负载核验整机峰值，不能用本机离线成绩承诺 NUC 硬实时。
现有 native/GTSAM 构建包含 `-march=native`，迁移时必须在目标 NUC 或匹配其 ISA/ABI
的构建环境重新编译，不能直接假设本机 `.so` 通用。

基准只使用真实保存的地图和端点，不连接 ROS、SDK 或机器人：

```bash
python3 src/d1max_pct_planner/tools/benchmark_live_map.py --case crossfloor \
  --repeat 3 --no-profile --verify-reuse --cpu-budget \
  --compare log/pct_performance_20260924/wall_crossfloor_multigoal.json
```

`--cpu-budget` 仅约束基准子进程；报告包括 wall time、CPU time、线程数、实际 affinity、
RSS，以及逐点/层号哈希和安全校验结果。地图构建、ROS 启动、消息调度和 RViz 渲染不计入规划耗时。

## 历史性能基线（2026-09-24，已由上文实现替代）

`tools/benchmark_live_map.py` 只在独立子进程使用真实 09-23 tomogram 与保存端点，
不启动 ROS、SDK 或机器狗连接。基准含 cProfile 开销，不能当作硬实时上界。
本轮保留原生搜索、GPMP、全部边界/地面验证及路径语义，只消除元数据探针的
无用全图初始化、重复数组分配和几何采样中的小数组开销。

| 冷规划用例 | 修改前 | 修改后 | 等价性 |
| --- | ---: | ---: | --- |
| 7.00 m 单层 | 3.549 s | 2.182 s | 227 点、层编号及校验结果完全一致 |
| 83.04 m 跨层 | 12.411 s | 9.627 s | 3030 点、层编号及校验结果完全一致 |

两者均核对完整点列/层编号 SHA-256，不以缩短路径、放松安全阈值或跳过检验换时间。
结果位于 `log/pct_performance_20260924/`。上述表是第一阶段同配置 cProfile 冷基准，
不是下表的无 profiler 实际 wall time。

第二阶段采用**一个常驻子进程、一个不可变地图/配置快照、最多四个 lazy native 分段地图**。
只复用地图和搜索器，不复用上一个目标的路径。单实例串行，不跨地图/会话复用；改配置须新建实例。
由于上游 A* 失败后不保证 reset，任何失败请求都会丢弃 native 状态。取消/替换计算中目标、
硬故障和退出均终止自有子进程，完全回收后才启动新 worker；空闲成功 worker 可接收新 generation。
全局就绪依据独立的 50 Hz 位姿租约，不再把 5 Hz UI 的瞬时匹配状态当成位姿中断。

| 无 profiler，真实地图 | 加载 + 快照 | 首次规划 | 后续规划 | 常驻 RSS / 初始化峰值 |
| --- | ---: | ---: | ---: | ---: |
| 7.00 m 单层 | 0.496 s | 1.360 s | 0.530 / 0.522 s | 2.00 / 3.20 GiB |
| 83.04 m 跨层 | 0.493 s | 7.311 s | 3.917 / 3.976 s | 6.22 / 7.42 GiB |

这是本机离线规划器 wall time，**不包括 ROS 启动、消息调度、父进程转换与 RViz 渲染**，也不是硬实时保证。
真实正向/反向/变更终点请求均与新实例逐点对比，完整路径/层编号不变；缓存始终最多四项。
代价是内存驻留增加，因此保持 lazy、单 worker、不重叠回收，不能在小内存机器照搬跨层缓存容量。
374 项 PCT 回归通过，1 项按环境跳过；72 项 worker/global 聚焦回归通过。
另测真实跨层路线父进程的原坐标验证需约 142–153 ms，已移出 ROS callback：
单线程验证器仅保留一个运行任务和一个最新待办，失效 generation 永不提交。
离线提交耗时首次 4 ms、后续约 0.03 ms，5 ms 轮询最大间隔 5.7 ms；
这是离线调度核验，不冒充 ROS/硬件时延保证。所有动态位姿/身份检查仍在提交时执行。
3030 个 PoseStamped 的构造也在线程中预制，owner 仅给共享 header 更新提交时间；
造消息加双序列化原本需首次约 60 ms、后续 32 ms，移出构造后双序列化单独约 13–27 ms。
Body 样本另按源时间单调接收，已确认地图/epoch/seed 改变后抬高屏障，排除队列中的旧位姿。
仍保留起点移动 0.3 m 的重算门限与总时限，高速移动中给新远目标可能需重新规划；
这里没有用旧起点直接放行，也没有跳过地面、完整曲线、障碍或跨层校验。

```bash
python3 src/d1max_pct_planner/tools/benchmark_live_map.py --case short \
  --compare log/pct_performance_20260924/before_short.json
python3 src/d1max_pct_planner/tools/benchmark_live_map.py --case crossfloor \
  --compare log/pct_performance_20260924/before_crossfloor.json
python3 src/d1max_pct_planner/tools/benchmark_live_map.py --case crossfloor \
  --repeat 3 --no-profile --verify-reuse \
  --compare log/pct_performance_20260924/before_crossfloor.json
```

## RViz MoveIt 风格空间手柄与全局路径预览

**当前默认：完整 PCD → PCT tomography → 原生 A*/quintic GPMP → RViz。**
按 [PCT 上游](https://github.com/byangw/PCT_planner) 的分层算法计算真实地面、上方观测、坡度/台阶、通行代价、膨胀和层简化；本机无 CUDA，tomography 是明确标记的 NumPy/SciPy CPU 移植，不是运行了官方 GPU 节点。

可复现建图配置：[config/official_single_floor.yaml](config/official_single_floor.yaml)。
运行/选点配置：[config/preview.yaml](config/preview.yaml)。
完整实现、验收和局限：[官方流程接入说明](/home/dndx/d1max_nav_ws/docs/20260922_pct_official_workflow.md)。

这个入口只做全局路径，不运行模拟运动、SCAN 跟踪器或机器狗 SDK；不依赖里程计。与下文在线参考路径服务分离，使用独立 `/d1max/pct_preview/*` 话题和 localhost:7465 Zenoh，会话不会发到机器狗或原导航执行器。

```bash
/home/dndx/d1max_nav_ws/start_pct_preview.sh start
/home/dndx/d1max_nav_ws/start_pct_preview.sh status
/home/dndx/d1max_nav_ws/start_pct_preview.sh stop
```

操作顺序：

1. 默认启动后直接显示 **START / GOAL** 两套手柄，不必先选中雷达点。点击 **3D Start**（B）或 **3D Goal**（N）可聚焦对应手柄；清空后可用它们重新放置。已存在的位置不会被重置。
2. 默认 **贴地模式**：保持 RViz **Interact**，抓中心球在地图 XY 平面移动，红 X/绿 Y 箭头精调，Z 跟随实测可通行表面。绿到黄的方格是当前 PCT 可行区域，红格不可行，暗色原始点云仅作背景，不表示能走。
3. 需要三轴编辑时切换 **自由 XYZ**：中心球沿当前视平面移动，Shift 左拖改变视线深度；精确升降用蓝 Z 箭头。三个圆环调整编辑姿态。控件采用 [MoveIt 2 Humble 末端交互控件](https://github.com/moveit/moveit2/blob/humble/moveit_ros/robot_interaction/src/interactive_marker_helpers.cpp) 同类原生实现，不需要安装整套 MoveIt。
4. 左侧“精确坐标”可展开输入并“应用坐标”；贴地时 Z 只读，自由 XYZ 时可编辑 Z。默认“自动跟随当前地面”，也可明确选择 tomography 编辑层；切换模式/层不会搬动已选位置。分层编号不等于建筑楼层编号。“总览”按起终点和路径一起取景。
5. 点“规划路径”，从选定起点执行 PCT 搜索与 GPMP 优化。路径保持显示，不随里程计超时/到点判断消失。
6. 拖动或应用新位置会撤销旧路径；编辑过程不反复求解，确认后再次点击规划。“清空”清除起终点和路径。

**旋转环当前只调整编辑预览姿态。** PCT 此入口约束的是 XYZ 路径，不约束机器人终点 yaw/roll/pitch；输出 Path 的姿态仍按路径切线计算，不强行把预览朝向塞到路径端点。编辑姿态单独存储四元数及 `orientation_revision`，旋转或“姿态归零”不改变 XYZ、规划版本或有效路径；数值修改 XYZ、重新聚焦和贴地也不把姿态复位。状态与导出文件显式标注 `orientation_semantics: editor_preview_only_xyz_planner`。

这里仍是地面机器人全局路径预览，不是空中自由飞行。自由 XYZ 模式保留输入坐标，但偏离实测地面超过默认 **0.08 m** 会提示“高度不在表面”；可明确点击“贴回表面”恢复。贴地模式编辑时自动派生 Z，不挪动 XY；缺少地面或高代价区域仍拒绝。自由 XYZ 中容差内的精确端点在输出路径中原样保留。

原始点云与高程格有采样差异，0.08 m 是输入一致性容差，不是越障能力。当前入口已替换旧单层适配器：读取完整 PCD 构建的实测多层 tomography，不伪造“地面 +2 m”顶棚，不叠加旧单层栅格的额外腐蚀。没有顶棚回波仍是“上方未观测”，不是已证实净空；预览默认保留官方对未观测上方的处理，`unknown_ceiling_policy: reject` 可收紧。离线通过不等于实机安全认证。

新建位置的初值由 `placement_anchor_xy` 附近的已测地面提供，只是便于开始编辑的位置，不是要选取某个原始点；终点初值放在起点附近。`place_endpoints_on_start: true` 默认直接放置两套手柄，但不触发规划。已有位置再次激活时，XYZ 保持不变。Humble 的交互标记显示使用 `Interactive Markers Namespace: /pct_preview_points`，不能使用 ROS 1 的 `Update Topic` 配置。

自由模式六轴控件使用 RViz 原生 `autoComplete` 生成粗箭头/旋转环，中心支持 `MOVE_ROTATE_3D`；贴地模式中心为世界 XY 的 `MOVE_PLANE`，保留 X/Y 箭头和 Z 旋转环。`handle_scale_m` 控制尺寸，地图不参与点选。原生几何测试调用本机 Humble `libinteractive_markers`，不是只检查自制图标。

工程分层：`official_pipeline.py/cpu_tomography.py` 管配置化地图生成；`tomogram_map.py` 统一地图判定；`tomogram_selection.py` 管分层选点；`tomogram_route.py` 管原生求解及全曲线检查；`native_runtime.py` 管原生库 ABI 校验；`preview_server.py` 管 ROS；`d1max_pct_rviz_tools` 管控件；`preview_session.py` 管独占进程。既有在线 PCT→SCAN 入口仍保留旧适配器，本轮未静默迁移实机链路。

每次成功规划在 `log/pct_preview/<session>/path_<revision>.json` 保存点选 XYZ、完整三维路径、地图哈希和规划信息；不覆盖地图 PCD。配置集中在 `config/preview.yaml`，RViz 初始布局在 `rviz/preview.rviz`。

离线 RViz 的全局规划分组移植自 [legbot_3D_Nav 的 navigation.rviz（f60606a4）](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/legbot_bringup/rviz/navigation.rviz)：淡化高度点云、8 cm 高度体素、红色全局路径。原通行代价显示保留在默认关闭的“通行代价”组；检查代价时关闭“分层体素 · 高度”以免两种颜色叠加。跨层配置提供总览、俯视和楼梯三个保存视角。保留三轴交互手柄，不引入上游的 A1 模型、Gazebo 里程计或不存在的局部轨迹；路径仅显示抬高 18 cm，规划坐标不变。

历史记录：下面链接属于之前的旧单层适配/MoveIt 风格版本，不是本轮完整 tomography 验收，也不能用旧 18.453 m 路线证明新地图的长路线通过。本轮还发现该旧运行环境串载 ROS GTSAM；当前已隔离到编译对应的 bundled GTSAM 4.1.1，并在加载后核对实际库路径。

- [隔离后台接口验收](/home/dndx/d1max_nav_ws/log/pct_preview/20260922_150653_732b52091496/isolated_validation/acceptance.json)
- [隔离进程与私有端口清理记录](/home/dndx/d1max_nav_ws/log/pct_preview/20260922_150653_732b52091496/isolated_validation/runner_report.json)

运行中的 RViz 另行确认了手柄更新订阅、反馈发布、初始化服务客户端及兼容 QoS，日志确认初始化完成。最初对可操作窗口做完整自动验收时，坐标被并发拖动改变，因此该次全流程中断；随后改到私有端口完成后台验收，不继续抢占用户坐标。[实际窗口连接检查及中断记录](/home/dndx/d1max_nav_ws/log/pct_preview/20260922_150653_732b52091496/acceptance.json)。后台无 RViz 测试不冒充人工鼠标手感或实机导航验收；桌面截图接口拒绝访问，未声称取得实际窗口截图。

后续自动验收使用 `tools/verify_preview_isolated.py --output-dir <新的输出目录>`（先加载本项目 ROS 环境），不要在用户操作的窗口运行会清空/改写坐标的 `preview_verify`。该工具只在随机独占 localhost 端口启动 Zenoh + 后端 + 验收客户端，禁用发现；结束时仅清理自己创建的进程及子进程，不重启或访问当前 RViz 会话。

## PCT → SCAN online reference route (2026-09-22)

`pct_route_server` runs the **upstream native `OfflineElePlanner` plus quintic
GPMP optimizer**, not the earlier Web A* demonstration. Its native library is
isolated in a spawned process so a slow C++ solve cannot stop the ROS heartbeat.
There is no velocity/SDK publisher in this package.

The single-floor map adapter loads the previously generated measured-ground
`planning_grid.npz`. Its support and obstacle masks come from the 09-19 SC-PGO
PCD and 09-21 ground extraction. This is an explicitly labeled CPU **map
adapter**, not a claim that upstream GPU PCT tomography generated these masks.
No holes are filled or unknown cells marked free. The existing raw/optimized
PCDs are never changed.

### Interfaces

All topic names and the session ID are configurable in
`config/pct_scan_single_floor.yaml`.

| Interface | Default topic | Contract |
| --- | --- | --- |
| Localized body odometry | `/d1max/localization/odometry/global` | `nav_msgs/Odometry`, `d1max_loc_map` → `d1max_loc_base_link`; valid quaternion, finite XYZ, fresh receipt and ROS stamp |
| RViz goal | `/d1max/pct_scan/goal` | `geometry_msgs/PoseStamped`, map-frame XY; goal Z is sampled from measured ground |
| Cancel | `/d1max/pct_scan/cancel` | `std_msgs/Empty`; revoke task and publish empty references |
| Debug path | `/d1max/pct_scan/global_path` | `nav_msgs/Path`, **ground Z**, same map frame |
| SCAN input | `/d1max/pct_scan/reference_path` | `d1max_planning_interfaces/ReferencePath`: session ID, generation, path |
| Task heartbeat | `/d1max/pct_scan/task` | `std_msgs/String` JSON at 5 Hz; session, generation, active, issued_at, heartbeat_at, frame_id, target_xyz, state |
| Diagnostics | `/d1max/pct_scan/global_status` | JSON readiness/failure and checked-route metrics |

Configure RViz's `2D Goal Pose` (`rviz_default_plugins/SetGoal`) tool to publish a
**PoseStamped** to the goal topic (not `Publish Point`, and not Nav2's NavigateToPose action). A frame
mismatch is rejected; the server does not rename frames or invent a transform.
The input odometry must already be in the PCD map frame. A startup UUID can be
generated automatically, but the complete PCT/SCAN/tracker launch must supply
one shared explicit `session_id`.

Each new goal first revokes the previous task using an inactive generation.
Only after native optimization **and whole-route checks** pass is a second,
new generation activated with `issued_at` set to the current ROS time. Then
its typed reference is published. Late worker replies and canceled generations
cannot reactivate a task. Active heartbeat timestamps/target remain immutable;
`heartbeat_at` advances. Stale odometry, backward clock jumps, cancellation,
worker failure, planning timeout or arrival clear the reference.

A separate steady-clock translation watchdog defaults to
`progress_timeout_s: 20.0` and `progress_distance_m: 0.10`. Only measured XY
displacement from an anchor refreshes it; yaw, commanded speed and repeated
small odometry jitter do not. Initial turning can use the 20-second budget.
On prolonged no-progress the route becomes `blocked`, withdraws task permission
and clears SCAN's reference with a newer generation. It never retries or
reactivates itself. Inactive time is ignored and an explicit new task starts a
fresh budget. This does not change footprint geometry or obstacle thresholds.

SCAN's `navi_mode=3` adds body height itself. Therefore online path Z is sampled
from the measured tomogram, omitting both the old wrapper's `+0.5 m` and the
native quintic reference-height offset. `target_xyz` is the endpoint ground Z
plus the explicit `body_height_m: 0.55`. The old offline CLI keeps its legacy
height behavior unless `ground_z=True` is selected programmatically.

### Offline / ROS usage

Use the existing Zenoh environment; this package changes no middleware settings.

```bash
source /opt/ros/humble/setup.bash
source /home/dndx/d1max_nav_ws/install/setup.bash
ros2 launch d1max_pct_planner pct_route.launch.py
```

The standalone launch is only a path server; it does not start localization,
SCAN, a tracker, a robot SDK session, or navigation motor control. The parent
single-floor launch coordinates those independently.

Optional reusable native tomogram artifact (trusted local pickle only):

```bash
ros2 run d1max_pct_planner pct_prepare_grid \
  --grid /home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260919_ground_20260921/global_path_reachable/planning_grid.npz \
  --output /home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260919_ground_20260921/pct_native/planning_tomogram.pickle
```

The online server builds the same in-memory payload directly from its configured
NPZ; it does not require this optional export. Do not load untrusted pickle files.

### Verification and limitations

- 19 tests cover grid/world conversion, unknown/blocked preservation, exact
  line supercover (including diagonal corner cuts), local height steps,
  restoring/validating native rounded endpoints, no unsafe-result fallback,
  native bounds protection, and same-cell crash avoidance.
- Native PCT test on the real measured map:
  start `(-4.1000015, 6.9, -0.6375016 ground)` →
  goal `(-5.5000015, 10.5, -0.7147216 ground)`;
  **3.903 m**, 27 crossed cells checked, minimum source-grid clearance **0.447 m**.
  These coordinates are test input, not a hard-coded route. Each RViz goal is
  freshly solved from the latest valid odometry.
- The previous 39 m corridor demonstration is **not automatically approved**:
  native GPMP crossed blocked cells and was rejected by the validator.
  A one-cell extra optimization guard now narrows PCT's search corridor; it
  does not expand the measured free space or silently fall back to A*.
- The map retains extraction gaps and nonzero ground height variation. The
  20 cm mask and clearance are not a complete swept-footprint, gait, negative
  obstacle or physical stopping-distance certification. Real driving requires
  the separate calibrated localization, live perception and safety gates.
