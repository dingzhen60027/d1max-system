# 走廊贴墙与楼梯局部阻塞修复（2026-10-23）

## 结论与边界

本轮修复三个已复现的软件问题：长路线居中整体回退、首末台阶误标平地、不可移动参考端点的错误朝向预检。保持同一主线、地图、机身尺寸、膨胀量、平地控制器和 SDK 配置。没有通过清点、缩小机身、改验收标志或新增运动检查使测试通过。

**不等于整段楼梯已经可通过。** 实际原始 PCD 的 20 个静态局部窗口，零末速阶段探针仍只有 8 个通过优化阶段；这些采用合成参考姿态，且没有运行完整 FSM、实时感知、曲线提交与跟踪，不是导航通过率。补充使用生产非终点末速边界的阶段实验为 7/20。其余失败不能被隐去，也不能把软件回归写成上下楼验收。

用户批准正常停止导航、保留 SDK 连接、同步唯一默认包，不自动重启导航。上述操作已经完成，部署证据以 `experiments/corridor_stair_fix_20261023/deployment` 为准。

## 默认包交付状态

| 项目 | 核对结果 |
|---|---|
| 同一默认包 | `experiments/navigation_runtime`，没有新入口或分支 |
| 新 manifest SHA-256 | `7e9b165739679d612be60ccb95c7cb3c4cf697cd38057cfb4a444202f51c4994` |
| 原基线 SHA-256 | `48e150335b2afb23b6051fb23c275dcb98e6dd37133eab38eea5c4bc4a8159d6` |
| 安装范围 | 14 个明确列出的源文件／测试；26 个安装项，包含 3 个重链 ELF |
| SDK | PID `1175012`、启动 ticks `8671369` 保持不变；未重连、未改变 SDK 产物 |
| 运动配置 | SHA-256 仍为 `e9738b287d8d27a6f5cd18faee7b17d1843cf4d437e44434e490cf68cefa6038` |
| 导航服务 | Web 正常停止流程完成；直接子进程及归属进程组全部退出；未自动重启 |
| 安装版实际导入 | PCT 4/4 路线与源码修复结果逐点相同；楼梯 12/12 路线、层号及分段语义相同，全部从默认安装目录导入 |

软件退出不等于物理停稳：停止报告为 `software_retired=true`、`cleanup_confirmed=true`、`physical_stop_confirmed=false`。最后只读查询 Web 为 `stopped`，启动等待项为 SDK 状态数据，不是包哈希不一致。保留原 SDK 进程不代表已经核实机器人网络、MC 回调或现场姿态。

## 已修复

| 层级 | 原因 | 修改 | 同输入证据 |
|---|---|---|---|
| PCT 路线质量 | 约 55 m 路线的一处约 0.5 m 曲线段触碰原 cost50 格，三个整路偏移候选全部被拒，整路回到贴边路线 | 原候选成功时不变；全失败时，仅对冲突段附近偏移做平滑退让，最多四轮；继续整条多项式检查 | 第6条日志派生路线距离指标 P10 0.159→0.594 m；前三条路线 XYZ/layer 哈希不变 |
| 楼梯语义 | 楼层高度搜索带覆盖首末台阶，先按 Z 归类导致台阶收到平地支撑 | 楼梯域内结合已有入口、出口支撑高度分类；普通入口地面和域外地面不变 | 8 组首末台阶修为 partial_stair，4 组平地不变；原图支撑→类型消息→参考回调 12/12 |
| SCAN 种子搜索 | 不可移动起终点被要求按“起点直指远端”的方向摆放长方体；斜放碰墙就直接终止，未尝试通道实际走向 | 分别允许真实不可移动起点、终点用原生完整相邻边检查恢复搜索；内部可移动锚保留旧优先策略 | 1.03×0.40 m 机身在 0.60 m 入口，旧算法误拒；新算法双向路径通过原生检查；25 项测试通过 |
| 诊断 | official 模式不维护 strict-free 缓存时钟，却用零缓存时钟计算源龄 | 使用查询快照实际时钟；缺源、未来源龄显示 nan；增加有界占据见证和参考坐标 | 14 项诊断回归；原始缓存、版本和碰撞结果不变 |

PCT 数字表示选定 slice 到不可通行格中心的距离，**不是机身到实体墙的净空**。第6条起点从现场 native 日志栅格中心重建，目标 XY 来自日志；不是完整现场位姿重放。局部最小距离仍受原地图约束，不保证任何选点都能居中。

上游 ROS2 A* 也使用起终点弦方向，但会移动被占据的端点；本工程必须保留真实端点，因此不能直接照搬该移动行为。本轮只适配这个差异，仍保留每条边、实际机身转入及最终曲线的生产检查。参见[上游源码](https://raw.githubusercontent.com/wuyi2121/SCAN-Planner/ros2-community/src/planner/path_searching/src/dyn_a_star.cpp)。

## 防止平地回退

- PCT 已成功的前三条现场日志派生路线完全相同；四条普通楼层入口/出口路线完全相同。
- 首版 A* 修复一度改变内部锚优先顺序，使静态楼梯窗口 8/20 退为 7/20；该版本没有发布。
- 收窄为按端点显式恢复后，20 个窗口的引导点列及锚点调用序列全部与原版一致，恢复原来的 8/20；其中 `down_410` 的五次内部锚调用逐项相同。
- 生产 GridMap、A*、B-spline、楼梯支撑、原生跟踪器测试分别执行。没有完整 bag 重放，没有 ROS 图、SDK 或机器人闭环仿真参与这些离线测试。
- 碰撞包络仍是 1.03×0.40×0.50 m；原始地图、墙体、台阶、未知证据策略、运动比例和控制参数不变。

## 未解决项

零末速阶段探针的 12 个未通过窗口已经逐项定位：

| 阶段 | 数量 | 窗口 |
|---|---:|---|
| 起点实体占据查询 | 4 | `up_2307`、`up_2894`、`down_327`、`down_851` |
| 没有满足当前选取条件的局部目标 | 4 | `up_2562`、`down_493`、`down_576`、`down_914` |
| A* 达到 400 ms 截止时间 | 2 | `up_2244`、`down_244` |
| A* 搜索集耗尽 | 1 | `up_2811` |
| Rebound 初始化失败 | 1 | `up_2728` |

2,181 次端点查询的原位置和 5 cm 缓存中心地形拟合全部有效；这些失败不是未知格拦截，也不是地形拟合失败后错误回到直立姿态。前三处起点阻塞涉及参考地面以上约 0.35～0.67 m 的点，未关联到已分类支撑点；`down_851` 的唯一弦向碰撞格与楼梯支撑点关联，5 cm 格触及合成机身包络下边缘。原始点云与假设姿态并非同一份现场测量，不能据此将点直接归为自体或清除。实机会话还显示过起点 1～17 个唯一占据体素，但缺少完整同源快照确定其类别。

额外验证了“放宽近端目标搜索”而没有直接部署：`down_493` 的约 0.5 m 可用段被当前远端搜索下界排除。允许回退到该段后，零末速阶段实验可从 8/20 提升到 9/20；但使用生产非终点末速、同步重建 seed 后，新增短段仍未通过原生碰撞检查，整体仍为 7/20，没有增益。已有正式侧向备用搜索在此例也失败。因此没有用强制零末速或只通过目标选择来宣称连续上下楼可用，也没有将该实验修改加入生产。

这些静态阶段结果不包含完整 FSM 的备用策略和实测姿态，既不能作为实机成功率，也不能直接断言每个窗口现场必定失败。

近端反例中的 `failed_final_collision` 由 `initControlPoints` 的控制多边形分段检查返回，当时还没有生成优化后整条曲线；不能将这条错误名误解释为最终曲线验证失败，也没有证据据此绕过原生检查。

本轮新诊断输出 `query_xyz`、坐标系、地图版本、姿态、机身范围、支撑机身 Z、前后雷达源时间、最多16个唯一占据体素位置及省略数；完整唯一体素计数和重叠计数保留。后续需要对失败瞬间的实时查询核对，而不是要求人工填一个固定楼梯宽度。用户“约三个机身宽”仅作为线索，未当作标定或强制通行依据。

## 复现及证据

工作目录：`/home/dndx/d1max_nav_ws`。

```bash
python3 -B experiments/pct_local_centering_20261023.3JGUxK/probe.py --before --output /tmp/pct_before.json
python3 -B experiments/pct_local_centering_20261023.3JGUxK/probe.py --output /tmp/pct_after.json
python3 -B experiments/stair_endpoint_ownership_20261023/native_probe.py --before-endpoint-classifier --output /tmp/treads_before.json
python3 -B experiments/stair_endpoint_ownership_20261023/native_probe.py --output /tmp/treads_after.json
python3 -B experiments/stair_chord_heading_20261023/run_probe.py
python3 -B experiments/stair_chord_heading_20261023/run_actual.py --before --scope
python3 -B experiments/stair_chord_heading_20261023/run_actual.py --scope
python3 -B experiments/stair_chord_heading_20261023/compare_actual.py
python3 -B experiments/stair_chord_heading_20261023/run_failure_diagnostics.py
python3 -B experiments/stair_chord_heading_20261023/summarize_failure_diagnostics.py
python3 -B experiments/stair_chord_heading_20261023/run_short_target_probe.py
python3 -B experiments/stair_chord_heading_20261023/summarize_short_target_probe.py
python3 -B experiments/pct_local_centering_20261023.3JGUxK/probe.py --installed --output /tmp/pct_installed.json
python3 -B experiments/stair_endpoint_ownership_20261023/native_probe.py --installed --output /tmp/treads_installed.json
python3 -B experiments/corridor_stair_fix_20261023/sync_default.py --verify
```

- `experiments/pct_local_centering_20261023.3JGUxK/report.json`：路线、障碍保留、耗时与130项测试口径。
- `experiments/stair_endpoint_ownership_20261023/comparison.json`：12组真实地图、原图支撑合同；相关137项测试。
- `experiments/stair_chord_heading_20261023/comparison.json`：20窗口逐项前后对照；`after.xml` 为25项原生测试。
- `experiments/stair_chord_heading_20261023/failure_diagnostics_summary.json`：12窗口原生查询、唯一体素和原始源点关联。
- `experiments/stair_chord_heading_20261023/short_target_audit.json`：近端目标方案的正反例；生产非零末速未取得收益，未部署。
- `experiments/corridor_stair_fix_20261023/installed_pct_verification.json`：默认安装目录实际导入及 4+12 条路线一致性。
- `experiments/corridor_stair_fix_20261023/deployment/install_plan.json`：明确源文件、消费者 ELF、静态库、SHA 与回退备份。不可把历史录包或软件阶段测试当作现场验收。
- `experiments/corridor_stair_fix_20261023/deployment/independent_review.json`：独立核对选择器、运动配置、原生依赖、SDK 进程和交付范围。

同步复用既有默认包事务，重链 `plan_env→path_searching→bspline_opt→SCAN/跟踪器`，不建立第二入口。静态库 build/install SHA、消费者重链时间及 ELF 身份均核对；Web activation、release selector、motion profile 三方绑定保持一致。SDK PID/启动身份及其映射依赖不变。

## 追加：楼梯膨胀排查与连续参考修复

### 更正静态实验的输入口径

上文 7/20、8/20 及其失败分类是当时**未经历史残影清理的原始 PCD**实验记录，不是现场成功率，也不是当前规划地图同源输入的结果。后续核对 `record_lineage.npz` 发现：12 个失败窗口涉及的92个唯一原始点中，91个属于已经由多帧可见性证据移除的历史残影。原探针把这些点重新注入了静态障碍图，因此不能据它们证明当前膨胀量过大。

修正的实验只按 `retained_source_indices` 选择原始记录：1,078,761→1,027,644点，XYZ和其余字段逐位保留，不使用压平坐标；也没有把另外1,034条恢复扫描帧记录混入这份输入。数据身份及缺项见 `experiments/stair_inflation_sensitivity_20261023/retained_source_manifest.json`。**这只纠正离线输入，不改变生产双雷达实时地图，不清除任何实时近身占据。**

### 膨胀结论

- 当前长方体为0.93×0.30×0.50 m，水平每侧余量0.05 m；查询包络为1.03×0.40×0.50 m。原生 OBB 查询使用原始占据格，不是再把已膨胀格重复膨胀一遍。
- 余量0.05/0.025/0 m的同输入原始点云实验没有稳定、单调收益。修正输入而不缩小余量，20个窗口就有17个通过原生优化。
- 18项窄墙反例保留：20 cm通道始终阻塞；40 cm通道受5 cm余量约束；50 cm直通可行但机身斜摆仍可能碰墙。不能将“画面红色少了”当作机身可通过。
- 楼梯显示采用的朝向与实际倾斜包络查询不同，显示的膨胀图不能单独证明规划查询被堵死。本轮未以改色、缩小机身或修改未知策略冒充修复。

### 本轮实际代码修改

| 问题 | 修改 | 同输入证据 |
|---|---|---|
| 台阶相邻点高差导致错误末端速度 | 非终点楼梯窗口沿已验证支撑面的平均坡度生成速度方向；保留XY方向、速度大小、真实起始边界及终点零速 | `down_724` 原末速近70°向下，造成重定时失败；20窗口17→18，所有旧成功保留 |
| 将控制多边形误当真实机身曲线 | 仅楼梯支撑作用域，代理折线首次报碰撞时，按该候选真实 knot interval 用原生包络核对整条 cubic；实际曲线清空才省去无效代理 A* | 20窗口18→19，两轮一致；`up_2728`约400 ms初始化超时→约16 ms完成优化；种子、目标和边界不变 |
| 新参考被拒却已经覆盖旧参考 | 非空提案统一在参考账本替换和取消副作用之前预检；安装复用准备结果，避免时效边界上的二次拒收；坏候选不改旧参考、曲线和任务 | 真实callback/CAS抽取105项：旧默认11项失败→新0项失败 |
| 参考发布端与接收端范围不一致 | 共享原有1.4 m水平对应范围和0.3 m竖直范围，正式会话生成对应的1.431782 m三维接收范围，不再落入隐藏1 m默认值 | 正式prepare-session及连续参考62项通过；不改变碰撞间隙和运动许可 |

新版实际曲线核对不是下游新增运动检查，而是优化器内部对原有控制多边形近似的修正。真实占据、严格未知、取消、求解期限、最终曲线碰撞和动力学检查仍按原生语义处理。没有增大400 ms预算。

平地16窗口在两次修复前后均16/16通过，最终轨迹逐点相同。独立旧API、错误dt、真实箱体、严格未知、取消和期限反例通过。平地控制器、SDK、运动比例、地图和机身尺寸均未修改。

原生优化器9组CTest通过。首次完整SCAN CTest的33组非启动测试通过，另两项launch测试因当前环境没有注册`scan_planner`包而在启动前失败，未启动节点；本轮不将它们计作通过，也不为补齐统计启动生产会话。最终回归明确排除launch测试，独立核对默认包的安装身份与导入。

### 仍须明确保留的问题

`down_851`仍未通过：该阶段记录为起始曲线附近的包络与保留台阶体素相交；下文继续检查已将相交定位到合成起点本身，不能归因于优化器把实测机身转歪。输入姿态是静态探针按路线构造的姿态，不是失败瞬间的实测机身姿态；不能借此删除该台阶或缩小模型。本轮没有完整运行上下楼 FSM/双雷达/SDK 闭环，19/20只表示固定静态窗口的原生算法阶段结果，**不表示上下楼已经实机验收**。

最近现场会话 `.4` 的连续 `reference_geometry_rejected` 与上述参考替换问题吻合；`.5` 的多次 `failed_reference_start_occupied` 仍缺少同时间完整雷达、位姿和地图快照来分类。历史残影输入修正不能解释或清除这部分实时占据。

### 追加证据与复现

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_surface_terminal_20261023/run_comparison.py
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_curve_proxy_20261023/run_comparison.py
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_curve_proxy_20261023/run_floor_control.py
ctest --test-dir experiments/stairs_continuity_20261022/native/build/bspline_opt --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
```

- `experiments/stair_inflation_sensitivity_20261023/{audit.json,retained_source_manifest.json,narrow_wall_results.json}`：原坐标输入谱系、膨胀敏感性和窄墙反例。
- `experiments/stair_surface_terminal_20261023/{comparison.json,floor_control_comparison.json}`：末速修复；`real_target_harness/result.json`为生产取目标方法53项检查。
- `experiments/stair_curve_proxy_20261023/{comparison.json,floor_control_comparison.json,test_optimizer_budget.xml}`：真实曲线与代理折线对照；14项原生优化器测试。
- `experiments/stair_runtime_reference_20261023/{before_full.json,after.json,python.xml}`：参考接收同输入前后对照。抽取执行真实回调和生产CAS，但时钟/发布/规划接收端为被动替身，不是完整ROS闭环。
- `experiments/stair_surface_terminal_20261023/deployment`：本次唯一默认包的准备清单、安装项、旧文件可恢复备份与安装核对。旧一轮manifest及SDK进程描述是历史状态，不可当作本轮仍在线的证据。

本次已同步原默认包，manifest为`556177d87325744b67b1b3616a1d22ba0f75bc0397d37604145e33a155d75d0c`，两个现有选择器均已更新。16个明确源文件对应31个安装项（含3个重链ELF、优化器库与头文件）；其余已封存文件、运动配置、SDK、地图及验收状态逐项保持原值。备份在本轮`deployment/before/`，未创建新入口或自动重启。安装时导航与SDK均未运行；本轮没有执行SDK退出或重连。

安装版四个Python模块的实际导入路径以及生成的`scan.yaml`对应范围见`deployment/installed_imports.json`。所有模块从原默认包安装目录导入，不借用开发源码。复核默认包可执行：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_surface_terminal_20261023/sync_default.py --verify
```

## 追加：剩余下楼窗口与近身查询（2026-10-23 夜间）

基线为上节 `556177d...` 默认包。此次先核对正式 measured-heading / align-then-forward 参数，再区分静态合成位姿、实测机身和搜索中间锚点；没有启动导航、重连 SDK 或让机器人运动。

### `down_851` 的失败不能作为删除台阶的依据

20 个静态探针起点均由 `ground_source_xyz + [0,0,0.5]` 构造，姿态也来自支撑拟合，并非实测 odom。正式 FSM/worker 使用的是采集的机身状态。使用正式朝向策略仍为19/20，固定 P/V/A 边界保持不变。

唯一失败窗中，保留的原始点 `209880` 位于箱体底面下方36.24 mm；包含它的5 cm体素则与箱体底面交叠约0.814 mm。原生SAT、RPY和高度转换一致，没有重复膨胀。保持XY、RPY、参考和地图不变，仅对**合成试验起点**加1/2/5/10/30.6 mm，完整原生求解均通过，约17 ms。这是输入敏感性诊断，不是生产修复：没有抬高实测位置、删掉踏面或缩小机器人。

证据：`experiments/stair_remaining_20261023/remaining_audit.json`、`height_sensitivity_summary.json`。这组结果只能说明合成起点擦碰离散体素，不能证明真实上下楼已经通过。

### 现场日志的17格不是实际机身占据的直接证据

现场会话 `7d34bfabe5b2424fb310a61bc99bf6af` 的 rebo2143 报告点在实际起点前方约0.30 m，查询yaw与机头差约89°；它是 `buildCollisionAwareReference` 的内部detour锚点。后续唯一1格也属于搜索锚点查询。已记录的占据格用原生GridMap复现后，在对应锚点/chord方向下相交，而在实际机身姿态下不相交。

这不能推出完整实时地图无障碍，更不能推出这些点属于轮腿。历史日志没有保存完整同时间双雷达射线和地图快照；单个witness的81组小范围位姿敏感性检查也不能替代缺失数据。证据：`experiments/stair_live_occupancy_20261023/known_witness_results.json`。旧日志 `source_query_ns=0` 的负源龄同样不能当作雷达陈旧证据。

### 已确认的楼梯平台目标筛选矛盾

原目标筛选要求端点在路线切线和起终点直线chord两个朝向都无碰撞；原生A*现在已支持固定端点沿真实入射边接入，直线chord不是必须执行的机身姿态。生产GridMap定向反例中，90°路线方向可通，63.2°chord方向碰墙，旧目标筛选拒绝，但原生A*可合法接入。

修复限定在有效的楼梯支撑作用域：目标仍按真实路线切线查占据，完整搜索、优化和曲线碰撞保留，不再额外要求任意直线chord朝向。平地继续使用原筛选。此反例与`down_851`是不同问题，不将它冒称为现场17格的唯一根因。

### 本轮交付及验证结果

1. **楼梯平台目标筛选已修复。** `getLocalTarget`仅在已经绑定有效楼梯支撑查询时使用实际入射边语义；普通平地仍要求原来的朝向条件。原生平台反例由拒绝目标变为完整求解通过，包含固定起终点、原生A*、cubic拟合、rebound、重定时及整条机身曲线核对。约35 ms为该反例单次观察值，不是实机P95。
2. **近身诊断分清查询对象。** 同时记录最后失败的搜索锚点、求解起点、实测机身位置/RPY/源时间/坐标系及各自占据证据；拼接预测状态不冒充实测状态。唯一体素列表上限由16增至64，保留完整唯一计数、原始重叠计数和省略数。提高坐标输出精度，避免远离原点时把5 mm的打印舍入当作几何偏差。
3. **诊断不改变碰撞结果。** 只在求解失败结果已经确定后由worker执行，最多1 Hz，沿用原求解预算；预算不足、取消或超时不追加完整查询。原始地图、未知分类、机身尺寸、膨胀量、控制参数和SDK均未改变。没有新增下游运动检查或清空近身区域。

| 对照范围 | 结果 | 能证明什么 |
|---|---|---|
| 实际生产取目标函数、同输入前后对照 | 56项：旧版1项失败，新版全部通过 | 楼梯平台接线生效，不只是独立辅助函数修改 |
| 原生楼梯搜索/整曲线正反例 | 9项通过；目标选择另25项通过 | 可用平台能接入；实体端点、封路、旋转扫掠碰墙仍拒绝 |
| 原坐标保留点云的20个上下楼窗口，两轮前后对照 | 每轮均19/20，原成功集合全部保留 | 新修改未破坏已有窗口；未用合成起点加高来凑20/20 |
| 平地16个原生窗口 | 16/16；目标、guide、seed、末速、最终轨迹及查询结果逐项相同 | 此次楼梯修复没有改变已验证平地算法输出 |
| 最终构建CTest | plan_env 7/7、path_searching 5/5、bspline_opt 9/9、SCAN非启动测试34/34 | 最终原生库和消费者构建一致；不包含完整ROS或实机闭环 |

首次plan_env全量回归有一项仍断言诊断上限为16而失败，原日志保留；将该测试扩展为75格并验证64格上限、完整计数和省略数后，最终7组全部通过。未为通过测试改变占据语义。新诊断的6项专项测试覆盖实际机身与搜索锚点区分、拼接状态、显式RPY、预算不足及陈旧/错误坐标状态。

### 已同步唯一默认包

新manifest：`c8abcc52f6fe965cf1bee128c87fd9904ad81df75b5d4b08a6d0bb9a5dba69e8`。

- 原默认目录仍为`experiments/navigation_runtime`，没有新分支或第二入口。
- 13个明确源码路径对应21个安装项；SCAN、local_follower、trajectory_tracker均完成必要重链，静态依赖与最终96次窗口对照使用的版本一致。
- 两个现有选择器已指向新manifest。最终清单10,332项，旧10,329项全部保留，新增3项诊断源/头文件；中间封存输出9,261不是最终成员数。
- 独立检查21个安装项的source/stage/default/manifest哈希，以及3个ELF的Build ID和代码段；SDK、地图、接口与运动配置1,203个受保护成员逐项未变。
- `physical_acceptance=false`未改。本轮没有启动/重启导航、连接SDK、下发目标或运动；默认包已更新不等于已完成现场上下楼验收。
- 本轮旧文件备份：`experiments/stair_remaining_delivery_20261023/deployment/before/`；旧事务备份未覆盖。

### 剩余现场缺项

软件已修复确定的目标筛选矛盾和诊断混淆，但**近身误阻塞是否彻底消失、上下楼是否能完整执行仍未验证**。现有现场日志没有完整同时间双雷达射线与地图快照，无法判定17格的全部物理来源。接下来需要失败瞬间的实测机身姿态、两雷达源时间/原点/回波和对应原生地图；新默认包已能直接区分真实机身占据与内部搜索失败，不必再从错误名称猜测轮腿回波。

`down_851`继续保留原始失败结果：已确认合成高度敏感，尚无该窗口真实机身姿态证明应当放行。没有把“约50 cm”当精确外参，也没有删除楼梯支撑体素。

### 本轮复核命令与证据

工作目录为`/home/dndx/d1max_nav_ws`。以下不会启动ROS服务或连接SDK；测试输出使用新临时目录，不覆盖既有对照证据。

```bash
ctest --test-dir experiments/stairs_continuity_20261022/native/build/plan_env --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/path_searching --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/bspline_opt --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -B src/scan_planner_vendor/plan_manage/test/test_blocked_entry_target_no_ros.py --native-build experiments/stairs_continuity_20261022/native/build/scan_planner
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_remaining_delivery_20261023/sync_default.py --verify
```

- `experiments/stair_remaining_delivery_20261023/{stairs_comparison.json,final_floor_comparison.json}`：最终依赖版本下上下楼和16个平地窗口对照，记录输入与库哈希。
- `experiments/stair_remaining_delivery_20261023/{target_before,target_after}`：实际生产函数的同输入前后结果。
- `experiments/stair_target_heading_20261023/comparison.json`：完整原生平台求解及危险反例、准确构建与执行命令。
- `experiments/stair_live_occupancy_20261023/diagnostic_verification.json`：诊断专项、原生OBB查询及只读证据。
- `experiments/stair_remaining_delivery_20261023/{independent_delivery_audit.json,staged_plan_review.json,deployment/install_plan.json}`：独立代码/版本审查、允许修改范围和实际安装清单。

静态窗口脚本为保护证据会拒绝覆盖已完成的输出；要重做整组前后对照，应使用独立证据目录，而不是删除本轮结果。

## 追加：楼梯障碍物附加膨胀分段调整（2026-10-24）

### 官方参照与本轮修改范围

已核对 SCAN 上游 [main 参数](https://github.com/wuyi2121/SCAN-Planner/blob/main/src/planner/plan_manage/launch/advanced_param.xml) 与其仓库的 [ROS2 community 参数](https://github.com/wuyi2121/SCAN-Planner/blob/ros2-community/src/planner/plan_manage/config/planner.yaml)：两者 XY 圆柱核半径均为0.25 m、双中心偏移0.18 m；Z 参数不同。该圆柱核包含机身避碰几何，不能直接解释为“机器人外再膨胀25 cm”，也不能原样代入D1的定向长方体模型。ROS2分支是社区移植，不冒称原作者已针对D1验证。

本轮只调整**障碍物的附加XY余量**：平地仍每侧5 cm；具有原始支撑点归属的楼梯与楼梯平台每侧2.5 cm。名义长方体0.93×0.30×0.50 m、机身高度、Z包络、自体滤除、运动配置及原始占据点均未改变。对应配置空间查询长宽：平地1.03×0.40 m，楼梯0.98×0.35 m。没有把减少附加余量伪称为精确测量了轮腿外包络。

### 实际规划和显示如何一致

- 正式会话将`stair_planar_clearance_m`投影为原生GridMap参数；不是仅修改RViz Marker大小。
- 同一个入口/出口窗口内，按查询XYZ最近的同层原始支撑点所属路段决定余量；其中的普通平地区域保持原值。无支撑、不同坐标系和含糊的平地/楼梯等距归属不缩小。
- 目标选择、A*、优化、整曲线检查、当前曲线查询及近身诊断使用同一尺寸选择。原始占据/未知类别不变，没有增加第二套运动碰撞门控。
- 查询缓存纳入实际长宽，作用域退出恢复旧设置，刷新地图快照清除旧路段绑定，防止楼梯尺寸串到下一个平地任务。
- 显示异步传递同一采样的尺寸、yaw、源时间和坐标系；不以另一次回调的朝向拼接旧尺寸。显示采用5 cm格，减少2.5 cm余量不保证每个姿态下都恰好少一圈格子；实际SAT查询仍按新尺寸执行。显示仍是当前yaw下的可视化，不冒称逐点三维姿态碰撞证明。

`optimization.dist0`保持0.28 m：单独降到上游0.20 m的同输入实验没有改善目标或最终轨迹，它不是本轮硬膨胀修复的替代品。

### 同输入对照与未解决项

- 原生GridMap/SCAN合成窄楼梯：5 cm附加余量拒绝，2.5 cm通过目标选择、A*、B样条优化、重定时及整曲线检查；原始占据数组不变。该栅格边界用例不是实机楼梯净宽或轮腿通过能力的证明。
- 实体墙、过高台阶、头顶障碍、转向中途扫到障碍的反例仍被拒绝。OBB专项45/45，体素/缓存32/32。
- 最终原生依赖版本，20个原坐标上下楼窗口两轮对照均19/20；7个窗口目标延伸、10条最终轨迹变化。起点及原始点云保持一致，原来成功的窗口未退化。
- 16个平地窗口两轮对照均16/16，目标、guide、seed、末端边界和最终轨迹逐项一致。配置和会话生成测试91/91通过。

实验夹具原来把原始map坐标点云标成了生产odom frame。本轮**前后两组统一修正实验frame为map**，不改变任何XYZ或生产TF，再验证作用域实际选择了楼梯0.98×0.35 m。旧标签混用的结果不能证明本轮新参数已生效。

`down_851`仍保留原始失败：它是前述合成起点与离散踏面体素的竖向擦碰，不能通过减小横向附加膨胀解决。没有抬起真实/合成起点、压平楼梯、删除踏面或清空近身占据来凑20/20。以上是静态原生组件结果，不是实机上下楼完成记录。

证据与准确编译/输入信息：

- `experiments/stair_clearance_sensitivity_20261024/`：参数敏感性、原生体素/OBB测试、上游对照。
- `experiments/stair_padding_20261024/final/scope_comparison.json`：最终依赖版本的20楼梯/16平地前后两轮、原始地图哈希及起点核对。
- `experiments/stair_padding_20261024/final_enabled_margin/`：两组均启用原生路段作用域、只改变5 cm→2.5 cm的独立对照。
- `experiments/stair_obstacle_inflation_20261024/`：最终构建/测试记录、唯一默认包安装事务及旧文件备份；没有新的启动入口。

最终非启动CTest：GridMap 7/7、A* 5/5、优化器9/9、SCAN 35/35、跟踪器10/10；混合路段/同XY异层/作用域恢复专项10/10。首次完整CTest发现被动worker接线夹具没有同步新增的clearance provider接口；已补齐，并验证成功、拒绝、取消及异常后均释放该绑定。生产碰撞逻辑没有为通过测试而改成无条件放行。首次失败日志与最终成功日志均保留。

只读/隔离复核命令（先在工作区执行`source /opt/ros/humble/setup.bash`）：

```bash
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/plan_env --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/tracker/build/d1max_trajectory_tracker -LE launch_test --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_padding_20261024/run_scope_comparison.py --baseline-clearance 0.05 --output-dir experiments/stair_padding_20261024/recheck_20261024
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_obstacle_inflation_20261024/sync_default.py --verify
```

窗口脚本只允许新的证据目录；`--verify`用于已授权部署的安装事务复核，不启动ROS/SDK，但其继承的封存核对可能更新审计文件，不应称为纯只读操作。

### 本轮默认包交付

- 默认目录继续使用`experiments/navigation_runtime`，新manifest为`2a4216bcf520f79e6bce3c56b36867a5ea4baa65bfc57c3e1a45bec0e8b41088`；两个现有选择器已同步。
- 18个明确源文件对应34个部署项：24个源/安装映射、3个重链ELF、5个native库/头文件及2个CHECKED_HASH缓存。独立stage审查确认与最终窗口实验依赖哈希一致。
- 最终封存10,333项，原10,332项均保留；唯一新增成员为原生楼梯膨胀合同测试。`robot_profile`、`single_floor_session`和正式`navigation_session`实际从默认安装目录导入，新`release_preflight/scan.yaml`含3个原生楼梯余量参数；不是只复制源码。
- motion profile SHA保持`e9738b287d8d27a6f5cd18faee7b17d1843cf4d437e44434e490cf68cefa6038`；SDK、地图、接口、原控制策略和未验收标志未改。未启动/重启导航、SDK或RViz，未发目标或运动。
- 旧文件与选择器备份保留于`experiments/stair_obstacle_inflation_20261024/deployment/before/`。没有新分支、新入口、自动push或自动恢复旧任务。

默认包同步完成不等于现场上下楼完成：尚有上述`down_851`合成起点擦碰，以及未覆盖的真实轮腿外包络/姿态证据，需在后续现场验证中区分处理。

## 追加：下楼窗口的竖向体素量化修复（2026-10-24）

本节接续上述`2a4216bc...`基线，更新`down_851`的结论。不是继续缩小横向膨胀，也不是将合成起点抬高。原始点、起点XYZ/RPY、机器人尺寸及原始占据概率均保持不变。

### 根因与正式修复

原PCD第209880点在合成机身底面以下36.243 mm；它是全图落入该5 cm体素的唯一原始点，并且属于该段的原始支撑点。整格向上覆盖后与倾斜机身相交0.814 mm。生产SAT计算正确，问题是竖向离散的保守外扩，不是TF错误、重复高度补偿或重复膨胀。

正式方案保留5 cm粗地图，只在有原始楼梯/平台归属的碰撞查询中，将占据格按真实回波划分为**5×5×2.5 cm的上下半格**。XY仍覆盖完整原格，机身长宽高、侧向余量及接触容差不变。该点对应下半格与机身底面相隔约23.807 mm，原生SAT判定不相交；上半格若存在真实命中仍参与碰撞。

- 在双雷达粗格计票去重之前合并所有原始命中；旧深度/原始点云入口同样记录，不能用最后一次命中覆盖历史障碍。
- 原始占据、从未观测、已观测但证据不足的类别不变。没有命中位置来源或混有旧粗格写入时，保留整格碰撞；未命中的半格不冒充观测自由空间。
- 证据随原生滑动地图清理、复用和固定快照复制；不会借用前一个任务的楼梯归属。只有真实清空证据、重置或滑出地图才退役历史命中。
- 目标选择、A*、优化、候选及当前曲线沿用同一生产GridMap查询。没有新增跟踪器或SDK二次碰撞门控。
- 平地查询仍走旧算法；没有缩小XY格、清除踏面、修改自体滤除或改动控制策略。显示的粗格证据与实际半格相交分别记录，不能把粗格仍有回波误称为仍发生碰撞。

先前2×2×2的各向细分实验保留在`first/`，**未部署**；最终仅Z细分，避免额外改变楼梯横向障碍外形。这是D1的量化适配，不冒称上游SCAN的默认实现。

### 最终原生同输入对照

| 范围 | 结果 |
|---|---|
| 20个上下楼窗口，前后各两轮 | 19/20 → 20/20；`down_851`完整通过搜索、B样条优化、重定时与整曲线检查 |
| 16个平地窗口，前后各两轮 | 全部通过；目标、guide、seed、末速及最终轨迹逐项相同 |
| 关闭细分的36窗与旧默认结果 | 几何字段完全相同；新旧全部原始占据哈希和起点相同 |
| 侧墙、低矮物体、顶障、同格第二个相交回波、粗格历史混入 | 5类均继续拒绝；旋转扫掠等原生整曲线反例也通过 |
| 新专项 | OBB 59/59、体素SAT 34/34、射线67/67；独立新增专项20/20 |

最终Z-only楼梯求解P95约52.15 ms、最大120.21 ms（编译并行负载下）；不是硬实时承诺。部分楼梯候选因碰撞几何更准确而改变，`up_2244`耗时高于旧版，未掩盖成“完全无性能代价”；仍在原400 ms整轮预算内，未改预算。

12×12×3 m、5 cm地图中，每格2字节证据带来writer加三个快照共26.37 MiB额外固定存储。原生双雷达合成射线ABBA测试中，三槽复制P95约7.66～8.11 ms → 8.94～9.09 ms，融合P95约1.74～1.77 ms → 1.86～1.95 ms。该测试不是录包或实机吞吐证明；没有将整张地图分辨率减半而扩大8倍体素数量。

### 复现与适用边界

证据目录：`experiments/stair_vertical_contact_20261024/vertical_only/`，包含原始点归属、SAT几何、原生链接命令及库哈希、前后输入/输出和危险反例。此前`first/`与`performance_idle.json`保留；安装证据位于`experiments/stair_vertical_delivery_20261024/deployment/`。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
ctest --test-dir experiments/stairs_continuity_20261022/native/build/plan_env --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_vertical_contact_20261024/run_native_comparison.py --output-dir experiments/stair_vertical_contact_20261024/recheck_vertical
```

复现目录必须尚未完成，脚本拒绝覆盖既有证据。上述不启动ROS服务、不连接SDK；全窗口使用原PCD与生产算法，但起点高度和姿态仍是明确标注的合成夹具。**软件中剩余的下楼窗口已通过，不等于实机上下楼已经验收**。历史现场17格的完整源回波仍缺失，不能据此声称已经证明全部近身占据都是自体回波。真实上下楼运动、轮腿/载荷包络和姿态测量仍需现场核对，未修改验收标志。

### 唯一默认包同步

已更新原`experiments/navigation_runtime`，manifest为`28ed67754ad43101dc00a86ff4e8564bca7be881e27261e48c3c54b3f79e071b`；两个现有选择器同步，封存成员仍为10,333项。12个明确源文件对应22个安装项（14个源码/配置映射、3个重链ELF、5个native库/头文件），没有新分支、第二个入口或自动push。

最终完整非启动CTest为plan_env 7/7、path_searching 5/5、bspline_opt 9/9、SCAN 35/35、tracker 10/10；正式会话及机身配置Python测试91/91。安装库与最终`vertical_only/`实验的库哈希一致；生成的`release_preflight/scan.yaml`已启用竖向细分，平地箱体1.03×0.40 m、楼梯附加余量0.025 m保持原值。

SDK、地图、接口、运动配置和`physical_acceptance=false`均保持不变。没有启动服务、重连SDK或发送目标/运动指令。精确旧文件和选择器备份保存在`experiments/stair_vertical_delivery_20261024/deployment/before/`，未删除或覆盖上一轮备份。

安装核对命令：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/stair_vertical_delivery_20261024/sync_default.py --verify
```

此命令核对部署事务并可能更新审计记录，不是纯只读操作；不启动ROS或SDK。独立源码与封包审查分别保存在`deployment/independent_source_review_vertical.json`、`deployment/stage_review_vertical.json`。

最终独立安装审计`deployment/installed_review_vertical.json`逐项通过：全部10,333个成员、22个安装目标和两个选择器一致，4个Python模块实际从默认安装目录导入。SDK 42项、地图4项、接口1,155项及运动配置保持原哈希；排除新会话身份和相同内容制动夹具的生成标识后，生成SCAN参数仅新增本次开关，跟踪器参数实质零变化。

## 追加：全局与局部路径主动远离障碍（2026-10-24）

### 目标与两层根因

本轮基线为上节`28ed67754ad43101dc00a86ff4e8564bca7be881e27261e48c3c54b3f79e071b`。用户需要的是有空间时主动留出间距，而不是继续增大硬膨胀、将本来可通过的楼梯堵死。原机器人尺寸、平地/楼梯附加余量、Z半格修复、地图、控制策略和SDK均保持原值。

- **全局PCT**：原居中只有1.0、0.7、0.4三档位移。固定起终点的短贴墙路径可能全部超过原曲率/绕行条件，直接保留贴墙原线；且原生路线没有与可见性简化路线一样参与居中候选比较。
- **局部SCAN**：原rebound距离项依赖碰撞发现后生成的障碍约束。没有碰撞但已经贴边的曲线可能没有这类约束；refine阶段也没有主动侧向间距项。提高`dist0`不能保证这种轨迹主动远离障碍。

这与上游[ROS2 community优化器](https://github.com/wuyi2121/SCAN-Planner/blob/ros2-community/src/planner/bspline_opt/src/bspline_optimizer.cpp)的rebound约束生成方式相符。以下软间距改善是本工程适配，不冒称上游默认配置。

### 正式修改

1. PCT保留原生、简化和各自居中后的全部有效候选，按现有长度、间距、转弯综合评分选择。只有不超过两个端点过渡长度的短平地段、旧三档几何失败时，最多增加三次小幅回溯。原端点、XYZ支撑、楼层及楼梯规则不变；可选改善失败不会使已有可行路线失败。
2. SCAN在原搜索、优化、重定时及整曲线检查成功后，执行一次有界的侧向间距改善。按曲线法向查询同一个生产GridMap和完整机身包络，不使用地面最近点距离；优先争取硬包络外15 cm舒适余量，窄通道只平衡现有空间，不把15 cm变成通行条件。
3. 改善只改变中间XY控制点，保留首末三个控制点、Z及时间节点；平滑位移场并比较转弯代价。同一组截面使用2.5 cm侧向探测步长，候选不能降低最差截面的侧向余量，避免用平均距离掩盖局部更贴墙；这只决定是否采用可选改善，不决定原轨迹是否可行。仍由SCAN原有动力学与整曲线/共同前缀检查接纳，不在跟踪器或SDK新增运动碰撞检查。
4. 可选改善使用30 ms子预算、最多12截面、768次查询和3次递减尝试；父求解预算不足时直接跳过。改善失败、超时或接入不满足时保留本次已经通过的基准曲线，而不是清空当前轨迹或令任务失败。取消仍按原合同处理。

参数为`optimization.soft_clearance_distance: 0.15`、`optimization.soft_clearance_budget_ms: 30.0`。这里的15 cm不是保证达到的实体离墙距离，也不是膨胀半径。

### 全局同图结果

使用默认地图与真实PCT组件，前后各30次任务均成功；25条原本合适的路线几何完全相同，5次重复短贴墙路线均改善。

| 任务 | 修改前 → 修改后 |
|---|---|
| 2 m短贴墙路线平均切片间距 | 0.1823 → 0.2598 m，起终点不变 |
| 二楼平地入梯路线平均切片间距 | 0.8368 → 0.8994 m |
| 同条入梯路线峰值曲率 | 0.4332 → 0.4108 /m |
| 同条入梯路线总转角 | 1.0374 → 0.8761 rad，长度仅增加1.52 cm |
| 12条上下楼首末阶及平台入口任务 | 全部成功；8条真实台阶段几何不变 |
| 热启动24次P95 | 1.5828 → 1.5834 s |

切片间距是距PCT禁行格中心的距离，不是实测机器人外壳离墙距离。增加原生候选有计算成本：部分路线变慢，含冷启动最大耗时1.638 → 1.972 s；不能因整体P95近似不变就声称逐条零成本。

PCT按native vendor隔离进程运行合计628通过、1跳过；原单进程出现一次旧vendor ABI混用，单独使用正确vendor后该项通过。原失败日志保留，未关闭ABI核验。证据为`experiments/pct_obstacle_distance_20261024/{comparison,validation}.json`。

### 局部正式尺寸与原生窗口对照

最终版本同时核对实测朝向、路径切向及静止入口的朝向合同，侧向探测与正式整曲线查询使用同一套朝向解释。所有结果使用最终优化库`793895de06f0662a2efb62b664ba558c3958ec8e5f871cf185873744ee312449`；此前迭代结果保留，但未作为最终交付证据。

| 验证 | 最终结果 |
|---|---|
| 原PCD的20个楼梯、16个平地窗口，前后各两遍 | 全部通过，重复输出几何一致；基准几何与旧默认包一致 |
| 可选改善 | 4窗改善，32窗保留原曲线，其中1个楼梯窗因改善超时回退；没有将原成功任务变失败 |
| `down_576`最小采样侧向余量 | 2.5 → 10 cm |
| 3个平地改善窗最小采样侧向余量 | 10 → 15 cm、10 → 12.5 cm、12.5 → 15 cm |
| 侧墙、低矮物体、顶障、同格第二回波、粗格历史混入 | 5类危险反例仍被原生碰撞判断拒绝 |
| 正式1.03×0.40×0.50 m包络贴墙内弯，30次 | 全部改善；独立5 mm采样最小侧向余量0.5 → 6.5 cm，改善耗时P95 9.77 ms |
| 平均间距改善但最差截面变差的反例，30次 | 全部保留原成功曲线，不接受劣化替换 |

“侧向余量”是指定姿态、原Z、生产包络向曲线法向平移后仍可行的采样距离，不是点到墙的欧氏距离，也不是连续全曲线最小实体间距证明。原PCD窗口使用明确标注的合成机身姿态；不等于真实上下楼运动验收。

在36窗对照中，改善阶段最大楼梯耗时30.007 ms、平地14.005 ms；整轮最大约165.97 ms、35.64 ms，均在原父预算内。30 ms为合作式期限，不能抢占一次正在执行的原生调用。实际manager回调的注入阻塞反例耗时37.739 ms后保留基准，父预算仍剩357 ms；不能声称任何情况下绝不超过30 ms。

保留端点PVA、Z和时间节点不代表所有中间曲率都下降。36窗中的峰值加速度最大增量0.008814 m/s²、jerk峰值最大增量0.010374 m/s³，峰值曲率最大增量0.7365 /m；均继续经过原动态判断及既有曲率软代价比较。正式贴墙内弯例的峰值曲率0.474 → 0.621 /m，既有曲率罚均为0。不能把这些静态验证写成实机平顺性已经验收。

专项原生测试15/15通过；真实manager质量段/动态方法、生产GridMap和整曲线查询组合验证5/5，覆盖接纳、子预算超时、父预算不足、公共前缀拒绝及动态拒绝。最终非启动CTest：plan_env 7/7、path_searching 5/5、bspline_opt 10/10、SCAN 35/35、tracker 10/10；会话参数与机器人配置Python测试91/91。

### 复现与证据

- 全局：`experiments/pct_obstacle_distance_20261024/`。
- 原生局部：`experiments/obstacle_standoff_delivery_20261024/final_minmargin/comparison.json`。
- 正式尺寸重复与劣化反例：`experiments/soft_clearance_20261024/measured_quality_minimum{_final,_counterexample}_results.json`。
- 独立审查、实际manager回调、封包：`experiments/obstacle_standoff_delivery_20261024/deployment/`。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
ctest --test-dir experiments/stairs_continuity_20261022/native/build/bspline_opt --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/obstacle_standoff_delivery_20261024/run_native_soft_comparison.py --output-dir experiments/obstacle_standoff_delivery_20261024/recheck_new
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/obstacle_standoff_delivery_20261024/manager_quality_probe.py --report experiments/obstacle_standoff_delivery_20261024/recheck_manager.json
```

使用新的输出目录；以上运行原生组件，不启动ROS服务、不连接SDK。改动仅改善规划器候选质量，没有增加执行层运动碰撞检查、清空近身占据、放开未知空间或改变机器人模式与控制参数。

### 默认包交付

已同步原`experiments/navigation_runtime`，manifest为`6c1e2998e34c4bf7d425a5893b336a41ec9c087eaa59427ba93d2cb8fc6ce27c`，两个原有选择器同步。14个明确源文件对应26个安装项：18个源码/安装映射、3个重新链接的ELF、3个原生库/头文件及2个checked-hash缓存。默认封存成员为10,337项，原成员全部保留。

同步时修正了PCT安装映射：`corridor_centering.py`与`tomogram_route.py`同时更新默认包的真实`application/install`模块和源码快照，不能只更新快照就声称部署完成。新原生库、重链ELF和测试证据使用同一构建闭包；未建立第二个入口或新分支。

旧文件及选择器备份为`experiments/obstacle_standoff_delivery_20261024/deployment/before/`；旧轮次备份未覆盖。未启动或重启导航、RViz、SDK，未发送初值、目标或运动指令，未commit/push。既有运动配置哈希`e9738b287d8d27a6f5cd18faee7b17d1843cf4d437e44434e490cf68cefa6038`与未验收标志保持不变。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/obstacle_standoff_delivery_20261024/sync_default.py --verify
```

此命令核对安装项、manifest和选择器，可能更新审计记录；不会启动服务或运动。

最终独立安装审计见`deployment/installed_review_final_minmargin.json`：10,337个成员逐项哈希一致、26安装目标与三个ELF身份/原生依赖匹配；PCT两模块实际从默认安装目录导入，加载代码与新源码一致。排除新会话/滑动坐标名称及派生制动文件身份后，生成SCAN参数只增加上述两个软间距参数，跟踪器行为参数不变。SDK 42项、地图4项、接口1,155项、运动配置及验收状态均未改变。

## 追加：离障优化的连续性复查（2026-10-24）

本节复查上节`6c1e2998...`版本，重点不是再扩大障碍，而是避免可选改善挤占轨迹交付、或者因体素小变化引入左右偏移。检查与修复不启动实机，不更改平地控制参数、PCT地图、硬包络、楼梯Z细分及原始占据判定。

### 确认的时序与几何风险

1. **源龄与计算预算并非同一件事。** 正式路径每轮使用原始实测位置起算，不走历史未来时刻splice链。原生`RollingCandidateConnection`反例中，同一可行曲线在源跨度380 ms、父预算仍余55 ms时可以接收；追加30 ms改善后源跨度410 ms、父预算仍余25 ms，却被原有源跨度合同拒收。只有“计算没超时”不足以说明结果还能接收。
2. **20 ms交付预留不足以覆盖原生接收复核。** 完整`certifyCandidateAdoption`对照中，基准350 ms加owner复核30 ms成功；中间追加30 ms改善则总410 ms，父期限已耗尽。这是注入时钟的软件反例，不冒称某次实机停顿已经由此定因。
3. **小幅体素变化可使软偏好反复换边。** 同一原始直线、同一1.03×0.40×0.50 m包络，两侧墙面的一格5 cm墙片左右交替，原软优化中点分别偏移±2.261 cm、曲线方向约±3.66°。两条都碰撞可行，仍可能给跟踪增加不必要调整。静态同输入重复一致不能替代此项检查。
4. **可选阶段前置读取也必须计时。** 注入首次源时钟读取35 ms阻塞时，原子预算在读取之后才建立，可能又追加约9 ms优化。无法抢占阻塞调用，但返回后不应重新获得完整优化时间。

### 最小修复边界

- 仅为软改善增加执行上下文：正式worker/rolling/preview沿用既有120 ms原生复核预算，加原20 ms交付余量，共预留140 ms。父期限或原始源龄余量不足以容纳改善时，直接交付原已通过曲线；不增加任务等待，不改变运动接纳阈值。
- 源龄使用同一ROS时钟的原始采集戳；CPU预算仍用steady时钟。查询、可选动力学和可选整曲线检查都能退出改善。回退、非有限时间只放弃改善；不更新原戳、不把暂停回放当作新数据，也不转换两种绝对时钟。
- 子预算覆盖首次源读取，超时后保留原曲线，不追加第二份优化预算。30 ms仍是合作式期限，单次阻塞或调度延迟不可抢占，不声称硬实时最坏耗时保证。
- 软居中使用与地图分辨率一致的连续死区：左右可用余量相差不超过一格时不追逐，超过部分再平滑施加位移。**这不忽略任何障碍物，也不改变碰撞检查**；明显贴墙仍改善，窄通道无法增加舒适余量时仍保留原路径。
- 起终点PVA、Z、节点时间及跟踪器限速历史保持原合同；没有新增执行层碰撞门控、SDK限制或另一套控制链。

本轮独立证据位于`experiments/soft_clearance_continuity_20261024/`。首次与后续修复结果分文件保存，不能用最后通过的子预算内测试覆盖首次读取漏计的失败证据。

### 最终复核结果与适用范围

| 检查 | 结果 |
|---|---|
| 原PCD、生产GridMap/SCAN的20个上下楼窗口 | 20/20仍可接纳；没有因舒适间距而封闭原可行通道 |
| 同一套16个平地窗口 | 16/16仍可接纳；3窗改善，其他13窗保持基准曲线 |
| 侧墙、低物体、顶障、同格第二回波、粗格历史混入 | 5类危险反例仍由原生碰撞判断拒绝 |
| 真实manager质量段及接收方法 | 9/9通过，包含源龄不足、owner复核预算、首次/后续读取阻塞和拒绝改善后的基准保留 |
| 一格墙面变化左右交替 | 两次均保留原始直线，不再产生相反的软偏移 |
| 正式尺寸贴墙内弯重复30次 | 30/30改善，独立5 mm侧移采样最小余量0.5→10 cm；可选改善耗时P95为10.48 ms |
| 原生专项与最终重链组件 | 21/21专项；bspline CTest 10/10、SCAN 35/35、tracker 10/10 |

完整36窗对照位于`final_first_read/comparison.json`，使用库`ef69197a...`。该项比较同输入关闭/开启软改善，并与此前冻结的无改善曲线逐点核对；不是将两个完整默认包作为黑盒连续运行。旧版时序缺陷和换边缺陷的同输入前后证据分别保存在本节列出的独立反例中。之后生产代码只修正两处退出原因标签，将CPU期限耗尽正确标为`quality_deadline`，没有改变几何或接纳逻辑；最终库为`3bdf59c45dc125cdfab776b7945e14d8dcc8a2b6883aff42d19b7c9e91cb5fe6`。`final_reason_verified/bridge.json`用源码反向替换哈希证明差异范围，并以最终库重跑原超时楼梯窗`up_2562`：基准几何逐点一致、仍可接纳、父预算有效。最终manager 9项及组件测试均使用最终库。

原PCD窗口使用合成机身姿态，源龄/阻塞测试使用明确注入的时钟，不能写成实机已经连续完成上下楼。上述10.48 ms仅为可选改善阶段，不是完整规划频率或端到端延迟。普通时间充足的周期仍会支付这部分计算成本；新增退出策略避免在计算或源龄已紧张时再追加该成本，不承诺任何机器负载下零停顿。

本轮没有修改PCT算法、全局地图、平地/楼梯硬包络、碰撞语义、跟踪控制参数或SDK。沿用上节全局远离障碍的优化，独立复核确认其楼梯分段规则未改变。原生接收与旧轨迹保持仍按既有生产合同工作，不靠伪造时戳、保留真实碰撞轨迹或放开未知空间获得通过。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
ctest --test-dir experiments/stairs_continuity_20261022/native/build/bspline_opt --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/tracker/build/d1max_trajectory_tracker --output-on-failure
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/soft_clearance_continuity_20261024/manager_continuity_probe.py --report experiments/soft_clearance_continuity_20261024/recheck_manager.json
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/soft_clearance_continuity_20261024/run_native_soft_comparison.py --repeat 1 --output-dir experiments/soft_clearance_continuity_20261024/recheck_windows
```

复现应使用新的报告名和输出目录，保留原证据；以上不启动ROS生产服务、不连接SDK。

### 本轮默认包同步

已同步唯一默认目录`experiments/navigation_runtime`，manifest为`b5eb12b16e7ecc5c5f391bec4bf7761a6bfc15eef7c479c0683c7e1eb657b9a0`。本轮7项源文件对应14项安装映射，3个ELF已重链核对；其中两个跟踪器ELF内容与原版相同，实际变化12项。没有新建入口或分支，没有commit/push。

`deployment/independent_installed_review_final.json`确认：10,337个封存成员全部哈希一致且无增删；两个既有选择器仅更新manifest引用；SDK 42项、接口1,155项、地图4项、application 321项及84项配置成员保持不变。13份生成配置/布局/制动合同排除新会话身份及派生hash后内容一致，运动配置仍为`e9738b287d8d27a6f5cd18faee7b17d1843cf4d437e44434e490cf68cefa6038`，未验收标志不变。

旧安装文件与选择器备份在`experiments/soft_clearance_continuity_20261024/deployment/before/`，没有覆盖前轮备份。同步前后均未运行导航或SDK；没有自动重启、恢复旧目标或发送运动。下一次通过原Web/统一入口启动将加载本轮默认包。软件回归通过与实机上下楼连续性验收仍分别记录。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B experiments/soft_clearance_continuity_20261024/sync_default.py --verify
```

此命令核对安装及封包并可更新审计文件，不启动服务。

## 追加：实测速度突然归零的日志复核（2026-10-24）

输入为真实会话 `7d34bfabe5b2424fb310a61bc99bf6af` 的既有日志，运动记录来自 **10 月 23 日**，不是对上述新默认包的实测。历史 SCAN ELF 为 `25de3673…`，本轮安装前默认为 `8dce8783…`；定位生产者及感知 Python 的相关代码则与历史运行件一致。`session.version_id` 是地图身份，不是发布版本。

### 已证实的中断链

- **16:12:57.136**：`tracker.log:14` 后雷达源龄 619.532 ms，原地图证据期限已过 19.532 ms，触发零输出。同期 `scan.log:924–925` 求解仅 25.084 ms，参考窗口尚余约 3.645 m；不是曲线没算出或窗口走完。随后 `tracker.log:17` 从约 0.01347 m/s 重新起步，之前约 0.299 m/s。
- **16:23:36.920**：`localization.log:7296` 的 IMU 年龄 100.204 ms，`local_valid=true`、无故障，但发布不可用通知。`tracker.log:1352` 接收时上一条已签发状态仍剩 89.549 ms 原始有效期，却被提前撤销。这是生产者与消费者的期限语义不一致。
- 控制周期统计来自 488 条 `ControlTiming`：P95 50.147 ms、最大 50.946 ms。突停边沿指令加速度约 6 m/s²、jerk 约 120 m/s³，而非停边的正常限速变化明显更小。这里是**指令差分**，不是实测物理加速度；控制定时器未出现同量级阻塞。

83 条中断日志包括：34 条地图源龄/证明到期，20 条局部状态不可用（含 4 条 reason 仍写 tracking、相邻诊断明确 unavailable），20 条 SCAN occupied，3 条实际接入无效，1 条窗口耗尽，1 条方向等待，4 条到达/停稳。日志存在降频，不将这些计数当成全部实际停车次数，也不把 occupied 全当成假障碍。

### 本轮实际修复与验证

仅修定位生产者：成功发布 typed 状态后记录其原始 `source_stamp/valid_until`；同会话、地图、epoch、seed，且本地估计仍有效无 fault 时，不因下一次预测暂缺输入而提前撤销尚未到期的旧合同。**不重发旧姿态、不刷新时间戳、不延长期限**。原期限到期仍撤销，重置、硬故障、无效测量和身份改变仍立即撤销。

使用生产 predictor → output tick 的相同输入，旧默认两个边界用例均失败，修复后通过；短缺数在原期限内恢复不会先发不可用，持续缺数仍在原期限发一次不可用。另覆盖发布失败、重复源不续期及身份变更。根代理对定位与感知相关 168 项回归通过；定位专项扩展 210 项通过，独立审查的 10 项边界通过。这些集合有重叠，不相加为总数。

感知仅增加固定大小的被动诊断：状态源戳、期限、接收时间、拒收次数和历史缺口，沿用原 1 Hz 日志。**未修改 60 ms 插值限制、600 ms 雷达期限、SCAN 碰撞结论、障碍膨胀、跟踪器参数、SDK 或地图。**

### 尚未解决，不能作为本轮已修复项

1. 感知发生过 60.238592 ms 状态间隔超过 60 ms 后整帧丢弃，也存在更长的真实历史缺口及 300 ms 原始雷达源跳。既有日志无法逐帧区分生产缺样、传输遗漏或消费拒收，新增诊断用于闭合这一证据链，不声称丢帧已修复。
2. 另一些零速附近没有同期投影丢帧；双雷达完整积分约 111–123 ms，完成时源龄约 402–431 ms，后续更新间隔/单源滞后使旧证据跨过期限。不能仅提高 SCAN 求解频率或放宽源龄宣称解决，地图流水线仍需专项优化与实测复核。
3. 唯一窗口耗尽前有普通 map anchor 更新的参考候选与高频曲线更新发生版本竞争；该参考接纳问题尚未修改。本轮也没有足够证据判定 20 次 occupied 是否误报。

本轮软件证据位于 `experiments/zero_speed_source_20261024/`，保留旧实现失败 `before_tests.xml`、修复源码回归 `source_tests.xml`、既有日志分析 `log_analysis.json`。分析文件中的整会话地图统计包含停机尾段，**不能把它的全时段源龄分位数当成运动期间延迟**。原始事件、行号和输入哈希用于复核。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src/d1max_pct_scan:$PWD/src/d1max_localization:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  src/d1max_localization/test/test_navigation_output.py \
  src/d1max_localization/test/test_realtime_navigation_output.py \
  src/d1max_pct_scan/test/test_perception_ray_projector_boundary.py \
  src/d1max_pct_scan/test/test_ray_projection.py \
  src/d1max_pct_scan/test/test_odom_ray_projection.py
```

未启动导航、连接 SDK、发送目标或运动指令；软件边界回归不能代替实机速度连续性验收。

### 默认包同步结果

本轮五个源码/测试文件已同步到同一 `experiments/navigation_runtime`，没有新入口或分支。新 manifest 为 `63fe28c975582ba51bf6736aef964d40a4e5f2a8c1108540c66a7eda99167ca0`；两个既有选择器同值，Web 只读状态也确认选中此包、校验通过，SDK 未连接且导航未运行。实际安装目录导入的同一组 168 项回归通过（`installed_tests.xml`），不是只测可编辑源码。

同步备份位于 `experiments/zero_speed_source_20261024/deployment/before/`。本轮保留全部 ELF、地图、SDK、配置及原未验收标志；只更新两个运行 Python 模块、对应源码快照/测试和一致的 Python 缓存。退出/重启不会自动恢复旧目标或运动。

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/zero_speed_source_20261024/sync_default.py --verify
```

该命令核对默认包及选择器，可更新审计文件；不启动服务，不连接机器狗。

## 2026-10-24：雷达历史投影与融合缓冲连续性

本轮从默认包 `63fe28c975582ba51bf6736aef964d40a4e5f2a8c1108540c66a7eda99167ca0`
继续修复；只涉及感知输入的历史接纳和原生 GridMap 的内存生命周期。
没有修改跟踪控制律、速度/加速度参数、楼梯支撑与碰撞模型、规划频率、SDK 或地图。
没有启动导航、连接机器狗或发送指令，也没有新增完整 rosbag 回放。

### 已落实的两项修复

1. **历史位姿不再错误沿用当前控制期限。** 投影原来使用 `LocalNavigationInbox`，
   晚于 100 ms 控制期限抵达的历史状态，即使仍在原有 400 ms 输入窗口内也会被拒收。
   这会在人为制造的历史空洞处丢弃整帧雷达投影。现在仅投影使用独立的
   `LocalNavigationHistoryInbox`；控制/参考仍使用原来的当前状态合同。
   历史类没有 `usable/task_usable` 运动/任务接纳接口，所有原始 source、IMU、
   posterior、valid_until 均保持不变；身份、传播界限、硬重置和单调顺序仍检查。
2. **融合输入改为固定缓冲复用。** 前后雷达各一个 pending 槽，另有一个未发布解码
   scratch；完整验证成功后才交换进入 pending。消费后只退役 ready 标记，保留容量，
   不再每帧反复释放/申请大块向量。坏包、旧包不覆盖已接收的有效帧；旧缓冲不能续期
   或再次融合。每批仍受原有 100,000 点/8 MiB 输入上限约束。

没有保留无收益的 DDA 微优化；逐束遍历、端点命中、自由证据、子体素和膨胀计算未改变。
已有的融合完成即发布快照、按剩余周期唤醒与 ingress drain 并非本轮新增修复。
实际默认地图融合配置为 **10 Hz**，不是早期设计文档中的 5 Hz；本轮没有改动该配置。

### 同输入证据与边界

| 检查 | 修改前 | 修改后 |
|---|---|---|
| 生产投影回调：11 个原始状态，4 个延后 120–180 ms 抵达 | 丢 4 个状态，形成 100 ms 洞，整帧投影失败 | 11 个保留，原始时间不变，投影成功 |
| 真正缺样、超过输入期限、硬重置后的旧帧 | 拒绝 | 仍拒绝 |
| 711 帧真实 CDR 的累计 C++ 内存申请量 | 5.657 GB | 3.759 GB，减少约 33.6% |
| 同次计数的 C++ 堆峰值 | 76.43 MB | 78.28 MB，固定缓冲多占约 1.85 MB |
| 走廊及六个楼梯窗口：820 帧 CDR、307 次更新 | 基线 | 全部碰撞相关状态、原始源身份与快照零差异 |

内存计数是 C++ `new/delete` 申请统计，不是进程 RSS；不包括 C `malloc`/特殊对齐分配。
累计申请量下降不能写成峰值内存下降，也不能直接换算为整链延迟降低。
逐更新对照覆盖 raw occupancy、竖向 hit subvoxels、严格/规划膨胀及计数、free stamps、
context/epoch/seed、双雷达原始源时间和真实 `copyCollisionSnapshotTo` 结果。
楼梯输入来自已有 raw+历史位姿诊断窗口，不代表当前实机上下楼完成验收。

根代理复审的 264 项 Python 测试通过，包含历史接纳、投影交付、精确投影、参考传递、
楼梯支撑与平台衔接。首次收集因测试环境缺少 planner 的 `PYTHONPATH` 失败，补齐后通过；
失败记录和通过记录都保留，未将它统计为算法回归。

证据集中于 `experiments/ray_fusion_continuity_20261024/`：
`history_admission_comparison.json`、`native/recorded_allocation_comparison.json`、
`full_state_corridor/`、`full_state_stair_{entry,lower_flight,landing,platform_turn,upper_flight,exit}/`、
`root_source_review_pass.xml`。原生 before 库 SHA 与本轮默认库一致：
`add1f38999c33f80e081a8dec9e89221d77117a59e4e5b4e9f693e72b9c3ee2d`。

**仍需区分的原因：** 旧日志中的 60.238592 ms 等真实历史间隔，缺少逐包接收证据，
不能全部归因于本次发现的晚包误拒收；60 ms 几何插值界限没有扩大。
真实传感器断流、真实障碍和原始证据过期仍可能产生停车，不能通过伪造时间、重复旧帧
或取消碰撞结论来宣称“永不归零”。本轮修复消除已证实的软件误丢帧并减少融合内存抖动，
实机连续性仍需要下一次运行的数据确认。

### 独立计时与编译闭包

最终计时使用 `native_timing_corridor_clean/timing.json`，AB/BA 四个进程运行于
20:02:27–20:03:13；同时暂停其他代理的构建、复制和哈希。入口处理包含 CDR 读取、
反序列化、解码及接纳，P50 **1.750 → 1.122 ms**、P95 **3.017 → 2.143 ms**。
融合及既有查询 P95 **53.922 → 55.412 ms**；整批平均墙钟 **11.427 → 11.358 s**。
因此只能确认入口处理和累计分配改善，**没有证据表明融合主计算或整链吞吐显著加快**。
首轮计时与约 3.88 s 的部署准备重叠，其“5.4% 整批改善”结论已作废；原数据及
`native_timing_corridor/concurrency_note.json` 保留，不挑选较好的一轮充当最终结论。

`plan_env → path_searching → bspline_opt → SCAN → tracker` 已按同一源码完整重编译。
无 ROS 的 CTest **67/67** 通过：plan_env 7、path_searching 5、bspline_opt 10、
SCAN 35、tracker 10；两个 ROS launch 测试未启动。控制源码没有变化；依赖原生库的
已有 ELF 进行配套重链，防止更改 GridMap 内部布局后混用旧 ABI。

### 本轮默认包交付与复现

已安装到原有 `experiments/navigation_runtime`，两个既有启动选择器均更新为
`c031d25c37a6a590245adcb37923c03a1ef10854b3c930261cdafdd3147ee1de`。
Web 的只读 `/api/navigation-session/overview` 返回同一 manifest、`readiness=ready`、
`startup_closure_verified=true`；导航为 `stopped`、SDK 未连接、运动未授权。
没有创建入口或分支，没有启动生产服务。

独立交付审计核对 10,343 个封存成员及 35 个明确安装目标；7 个地图文件、42 个 SDK
文件、85 个配置文件保持不变。10 份生成的节点参数在排除生成会话身份后语义一致。
实际安装路径导入的 264 项 Python 回归再次全部通过，证据为
`root_installed_tests.xml` 与 `deployment/independent_installed_audit.json`。
源码回归与安装回归使用同一组测试，不将两者相加为不同用例数。

原文件及选择器备份位于
`experiments/ray_fusion_continuity_20261024/deployment/before/`，部署前模块另封存于
`deployment/prepared_baseline/`。本轮仅做软件交付，不更改任何物理验收标志，
不恢复旧目标或执行许可，也不声称已完成实机连续行走验收。

默认包一致性复核（不会启动导航或 SDK）：

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
source experiments/navigation_runtime/application/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/ray_fusion_continuity_20261024/sync_default.py --verify
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/ray_fusion_continuity_20261024/verify_delivery.py \
  --manifest c031d25c37a6a590245adcb37923c03a1ef10854b3c930261cdafdd3147ee1de
```

从实际安装目录复现投影、参考及楼梯边界回归：

```bash
# 沿用上面的工作目录与三个 setup；不添加可编辑源码到 PYTHONPATH。
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  src/d1max_pct_scan/test/test_local_navigation_state.py \
  src/d1max_pct_scan/test/test_perception_ray_projector_boundary.py \
  src/d1max_pct_scan/test/test_ray_projection.py \
  src/d1max_pct_scan/test/test_projector_completion_delivery.py \
  src/d1max_pct_scan/test/test_ray_projection_exact_performance.py \
  src/d1max_pct_scan/test/test_continuous_reference_transport.py \
  src/d1max_pct_scan/test/test_reference_portal_continuity.py \
  src/d1max_pct_scan/test/test_stair_support.py
```

融合延迟的剩余项仍为主计算及真实调度长尾；本轮不得用入口节省约 0.9 ms 的 P95
替代该问题的解决。上述既有录制输入只验证软件语义和组件耗时，不代表完整实时链路，
下一次实机的零速事件仍需对齐原始源时间、投影拒收原因、地图完成时间及实际发送记录。

## 2026-10-24：原生地图融合主计算优化

本轮基线是上轮已交付的 `c031d25…` 默认包，而不是 Git HEAD。正式运行代码只修改
`plan_env/src/grid_map.cpp` 的编译方式：对既有逐体素 visitor 使用 GCC/Clang 内联提示。
当前 `-O3` 原来仍将该 visitor 拆成独立函数，每次融合产生数百万次内层调用。
现在消除这一调用开销，射线访问顺序、预算、投票、原始源时间和碰撞结果不变。

没有修改控制器、速度/加速度、楼梯支撑与净空、障碍膨胀、参考窗口、地图分辨率、
融合频率、源龄期限、SDK 或物理验收标志。前一轮的历史投影接纳与固定输入缓冲修复
完整保留。没有通过减少射线、扩大未知空间放行范围或伪造新鲜时间来换取速度。

### 性能定位与保留范围

- 核对实际默认配置为 **12×12×3 m、5 cm、10 Hz、6 m 水平更新范围**。旧实验中的
  8 m 窗口不能用作当前性能结论；本轮重新生成同一批录制输入的实际参数版本。
- 基线分段计时中，射线遍历约占融合主计算的 **87%**。因此针对内层调用，而不是
  改控制参数或只调整地图显示。
- 同轴 DDA 分组、逐束隔离内核、延后时间证据更新三个候选虽有正确性验证，但实际
  更慢，全部撤出生产。实验副本和失败计时保留，不选择较好单轮冒充最终收益。
- 离线 probe 增加显式主计算计时及 scratch 状态摘要；诊断不进入生产热路径。

### 同输入前后对照

使用已有真实 CDR，保持点、原始源时间、坐标身份和输入次序不变。走廊 711 帧、
六个楼梯窗口 109 帧，共 **820 帧 / 307 次更新**；每次更新检查 raw occupancy、
子体素、膨胀及计数、free 源时间、身份、真实碰撞快照，以及临时计数、访问标志、
缓存队列和恢复标记。最终逐帧状态摘要 **零差异**。

性能单独运行，不混入状态摘要计算、CPU 采样、编译、复制或部署；新旧程序按 AB/BA
交替。固定高性能核四轮，未固定 CPU 和固定节能核各两轮，分别统计，不混合样本。
预先约定每进程首三次更新为初始化期，同时保留全部样本，未删除异常值。

| 稳定运行阶段的融合主计算 | 修改前 | 修改后 | 变化 |
|---|---:|---:|---:|
| 高性能核 P50 | 48.909 ms | 44.842 ms | −8.32% |
| 高性能核 P95 | 51.374 ms | 47.014 ms | −8.49% |
| 高性能核 P99 | 53.014 ms | 48.076 ms | −9.31% |
| 未固定 CPU P95 | 54.334 ms | 50.430 ms | −7.18% |
| 节能核 P95 | 91.953 ms | 83.165 ms | −9.56% |

包含初始化的高性能核全部样本：P95 **51.698 → 47.195 ms**，P99
**62.309 → 59.288 ms**，最大值 **88.197 → 84.866 ms**。
必须同时保留反例：高性能核稳定阶段最大值 **54.752 → 63.604 ms**，出现一个
修改后的尖峰；未固定 CPU 同指标 **58.467 → 53.766 ms**。因此 P95/P99 改善成立，
但不能声称最坏长尾或实机突然零速已经消除。

这些数字仅为生产 GridMap 的融合主计算，不包括 ROS 调度、生产快照发布、通信、
控制和 SDK；既有碰撞查询另外计时。离线注入时钟不等于真实墙钟，旧
`writer_transaction` 值不参与结论。高性能核/节能核差异说明调度影响很大，
**不能据此断言历史实机当时恰好运行在节能核**。本轮没有添加生产绑核策略。

### 回归与复现

完整重建 `plan_env → path_searching → bspline_opt → SCAN → tracker` 静态链接闭包。
原生无 ROS 回归 **67/67** 通过（7+5+10+35+10），包含楼梯支撑、净空、滚动参考、
轨迹和跟踪核心；两个 ROS launch 测试未启动。本轮未连接机器狗、启动导航或发送指令，
也未新增完整 rosbag 回放。

证据目录：`experiments/fusion_tail_20261024/`。其中 `native/delivery.json` 固定源码、
静态库、ELF 和测试日志；`final_full_state_*` 保存七个场景的前后状态；
`final_timing_p2`、`final_timing_unpinned`、`final_timing_e16` 保存每轮全部计时。
默认基线库 SHA 为 `6e2a33942cddd19d05f61c2f62850da2db496d673e6a7612575cc60c531e60fd`，
最终库 SHA 为 `3090a4d02c0bd44a221861c1255e53fb06b3d14f2d2c319c9c869db7d050f38d`。

复现融合计时（纯离线内存程序，不启动 ROS/SDK；输出目录必须不存在）：

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/fusion_tail_20261024/compare_core_timing.py \
  --audit experiments/fusion_tail_20261024/final_full_state_corridor \
  --output /tmp/d1max-fusion-core-recheck-20261024 --cpu 2 --repeats 4
```

CPU 2 对应本机高性能核；换电脑需先核对核心拓扑，不能把它当作通用生产配置。
同一命令去掉 `--cpu` 可复现独立的未固定核心实验，需使用另一个新输出目录。

### 默认包同步与保留项

已在同一 `experiments/navigation_runtime` 安装，manifest：
`d0d9393c33e84167bc9fcd577a919d28771167b0c891a0220d683742bd53f6c1`。
两份既有选择器同值。Web 只读 overview 先返回正常的后台 `checking`，随后确认同一
manifest、`ready`、`startup_closure_verified=true`；导航 `stopped`、SDK 未连接、
运动未授权。未通过启动服务来完成该核查。
安装范围为 2 个源码快照、3 个静态库和 7 个配套 ELF；
其中只有 4 个目标实际内容改变，其余保持同内容的配套重链产物。
独立核对 **10,343 个封存成员**通过，7 个地图文件、42 个 SDK 文件、85 个配置文件
保持不变；10 份生成节点参数除会话身份外语义一致。上轮两个历史投影运行模块的
SHA 及实际安装导入路径核对通过，没有回退成旧实现。

部署前文件与选择器备份在 `experiments/fusion_tail_20261024/deployment/before/`；
精确安装清单、输入和性能证据已冻结，不会把其他工作区改动一起发布。
`final_audit.json` 汇总全部输入、源码、二进制、逐轮指标和反例；
`deployment/independent_installed_audit.json` 是独立安装后核查结果。

```bash
# 沿用前面的 ROS / interfaces 环境，再加载安装的应用路径。
source experiments/navigation_runtime/application/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/fusion_tail_20261024/sync_default.py --verify
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/fusion_tail_20261024/verify_delivery.py \
  --manifest d0d9393c33e84167bc9fcd577a919d28771167b0c891a0220d683742bd53f6c1
```

融合主计算的分位延迟已有可复现改善，但实际执行器调度、快照发布、输入源龄以及
实机速度连续性尚不能由该离线计时证明。本轮保持这些缺项，不宣称“不会再突然归零”。

## 2026-10-24：融合交付、快照占用与换轨回退导致的零速

本轮以已部署的 `d0d9393…` 为基线，不再改融合内核。重点是完整地图算完以后，
能否及时进入当前曲线的碰撞判断，以及换轨失败时是否错误丢失仍有效的当前曲线证据。
控制律、限速与加速度参数、楼梯支撑/姿态/净空、PCT、膨胀、源龄阈值和 SDK 均不变。

### 真实日志结论与证据边界

最近有运动记录的是 10-23 的 `7d34bfabe5b2424fb310a61bc99bf6af` 会话，早于本轮基线。
只分析 16:12:53—16:25:10 的活动窗口，排除关停后源龄持续增长的尾部。
83 条限频记录的运动→零速事件中，34 条涉及原始雷达源过期；这些事件的证明本身
仅有 0.738—50.115 ms，不能把它们解释成“控制器没有及时续发证明”。
其中 8 条事件前，投影日志已记录了比当前证明更新的源，证明投影之后存在软件交付迟滞。

例如 16:21:40.721 的 tracker 第 965 行使用了 604.380 ms 前的前雷达源；
perception 第 814 行在约 177 ms 前已发布更新的前雷达源。同一邻近窗口 writer
记录 144.786 ms 融合、24.389 ms 后处理，共 169.171 ms。
日志约 1 Hz，不能进一步把这一事件唯一归因到某把锁；这些旧日志也不能冒充
当前默认包的实机验收。其余占据、定位源失效等事件分别保留，不改成可通行。

### 修复后的线程与版本边界

1. **完整融合优先交付当前曲线快照。** 原来在发布快照前，先阻塞获取验证器账本锁，
   仅为了询问慢速候选是否需要第三槽。现在先发布快速槽并通知当前曲线检查；
   慢速需求用非阻塞查询。锁忙表示“暂未取得提示”，不是“没有需求”：保留一次
   pending，使用原有 10 ms 重试，不等下一次雷达融合、不分配第四槽、不刷新源时间。
2. **诊断不占用碰撞快照。** 先拷贝任务/机身状态，再借快照执行原有检查；完整检查
   返回即释放读槽，然后记录快照流量和发布诊断。诊断账本只在锁内复制值，JSON
   格式化及日志输出在锁外，避免慢日志连带阻塞机身状态和命令回调。
3. **回退证据按曲线身份和序号单调合并。** 原曲线已续期到 seq 10，换轨被拒时
   原来却被候选缓存里的 seq 1 覆盖，直接触发 `waiting_fresh_local_map` 和零速度。
   现在恢复该曲线的最新结论，保留原截止时间。等待换轨 ACK 期间，原曲线最新结论
   另随该次交换保存，不能被 8 项候选缓存淘汰。成功提交或取消即释放，不能污染新轨迹。

该改动没有新增碰撞检查、没有把候选失败升级为整任务失败，也没有放宽过期规则。
`checkAt` 内原有的有序证明/命令发布仍在事务内：本轮不声称全部 ROS I/O 都已无阻塞，
不把锁外发送引入取消和旧命令复活的竞态。

### 同输入对照及危险反例

| 实际生产代码/生产组件的确定性输入 | 修改前 | 修改后 |
|---|---|---|
| 账本锁被另一线程持有，融合已结束 | 快速快照被慢提示挡住 | 快速快照立即走独立交付路径，保留慢请求重试 |
| 当前查询完成，外层诊断被挂住，另一槽仍被 owner 借用 | 诊断期间两次发布均不能取得槽 | 同样两次更新均能发布，原源时间保持不变 |
| 原曲线 seq 10 仍有效，seq 1 已过期，新曲线提交被拒 | 恢复 seq 1，剩余时间变为 −20 ms，发零速 | 保留 seq 10 剩余 160 ms，同输入继续前进 |
| 等 ACK 时原曲线收到 seq 11 占据结论，再收到九个其他曲线证明 | 候选缓存淘汰负结论后可能恢复旧正结论 | 原曲线负结论仍在交换记录中，正确停止 |

以上是确定调度/真实跟踪核心的对照，不是伪造实机延迟分位数。原始默认头文件与最终
相同 11 个回退用例对照：**6 项失败 → 0 项失败**；最终完整 follower **74/74**。
保留了中间版只修 max-sequence 仍有两项缓存淘汰失败的证据，最终已修复。
验证还覆盖原期限自然过期、旧负结论乱序、更新负结论、同序号重复、不同 anchor、
取消后的迟到 ACK，以及成功换轨后原曲线结论不污染新曲线。

生产 GridMap/CollisionSnapshotPool/ExecutionValidator 内存回归 **18/18**，覆盖读者
不能被覆盖、异常释放、同源快照、慢提示重试和诊断阻塞。精确提取的完整 writer/reader
生产方法另做时序注入，使用真实互斥锁和 promise 确定发生顺序；测试等待上限只用于
发现死锁，不作为“延迟降低多少毫秒”的性能数据。

证据目录：`experiments/zero_speed_delivery_20261024/`。`log_causality.json` 包含每条
历史事件的上下游日志行；`native/delivery.json`、`local_follower/delivery.json` 固定
源码、ELF、前后输入及测试结果；`independent_review.json` 为独立审查。

纯离线复现，不连接机器狗、不启动 ROS 图：

```bash
cd /home/dndx/d1max_nav_ws
python3 -B experiments/zero_speed_delivery_20261024/log_causality_audit.py
python3 -B src/scan_planner_vendor/plan_manage/test/test_snapshot_writer_handoff_no_ros.py
python3 -B src/scan_planner_vendor/plan_manage/test/test_snapshot_reader_scope_no_ros.py
experiments/zero_speed_delivery_20261024/local_follower/after/test_local_follower_final \
  --gtest_filter='LocalFollowerRollback.*'
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner \
  -LE launch_test --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/tracker/build/d1max_trajectory_tracker \
  --output-on-failure
```

要复现失败对照，将 writer 测试的 `--fsm`、`--validator` 指向证据目录
`native/before/` 对应文件；reader 使用 `--source native/before/execution_validator.hpp`
完整路径；跟踪器运行 `local_follower/before/test_local_follower_final` 同一 filter。
这些 baseline 命令预期非零退出，不能当作最终版本回归失败。

### 本轮默认包交付

已同步同一 `experiments/navigation_runtime`，未创建新入口/分支。manifest：
`238e07f529bdbe0dd7794d08d0473bbe8bd148c3abedd5a6b44fe3db92fed549`。
既有两份选择器同值；Web 只读 overview 已返回该版本 `ready`、
`startup_closure_verified=true`，导航及连接均 `stopped`、运动未授权。

实际安装 14 项：10 项源码/测试/CMake 快照、2 个配套安装头文件、2 个 ELF。
独立核对 10,345 个封存成员，7 个地图文件、42 个 SDK 文件、85 个配置文件不变；
10 份生成参数除新会话身份外等价。三个碰撞/搜索/优化静态库与前版逐字相同；
隔离构建依赖与默认消息库一致，未把未部署分支的 ABI 混入正式程序。
SCAN 和 tracker 已重编；全套原生回归 **69/69**（7+5+10+37+10）通过，
包括楼梯支撑、转接、净空、旧任务退役及平地控制。未运行 ROS launch 测试。

- 安装 SCAN SHA：`877b2354b84eb7c8fa2ee41551ba1cfeadc860f9aea6bab136eac589b0d1468c`。
- 安装 follower SHA：`c65d59960a8dd6f2b3f53f927da81f12e60c99453cbe4b9a81df118542a53b8b`。
- 原默认包文件、清单及选择器备份：`experiments/zero_speed_delivery_20261024/deployment/before/`。
- 安装后独立核对：`deployment/independent_installed_audit.json`。

复核交付（只读，不启动服务）：

```bash
cd /home/dndx/d1max_nav_ws
python3 -B experiments/zero_speed_delivery_20261024/sync_default.py --verify
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
source experiments/navigation_runtime/application/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B \
  experiments/zero_speed_delivery_20261024/verify_delivery.py \
  --manifest 238e07f529bdbe0dd7794d08d0473bbe8bd148c3abedd5a6b44fe3db92fed549
```

本轮消除了已复现的错误零速和快照交付阻塞路径，但没有进行实机运动验收。
真实采集断流、原始源过期、真实占据和 SDK 失权仍不能忽略；也不宣称所有剩余延迟尖峰
已消失。保留新增的 hint 忙计数和快照借出/释放时间，供下一次实际运行区分新鲜数据
未抵达、融合超时、快照占用及发送端问题。未启动生产服务、SDK、RViz，未提交或 push。

## 2026-10-24：滑动窗口、换轨与控制下发的连续性复核

以实际默认包 `238e07f…` 为基线，检查的是“新窗口已经产生，但后续轨迹为什么
仍然接不上”，不是再次提高规划频率或放宽源龄。共修改四个生产文件；控制律、
速度/加速度/jerk 参数、SDK 映射、楼梯模型和三个算法静态库均保持原样。

### 三个已复现并修复的时序问题

| 边界 | 原行为及后果 | 修复后的所有权规则 |
|---|---|---|
| 新窗口到达 → 原生候选验证 | 普通滚窗清除正被独立检查线程验证的候选；计算完成后身份已不存在，结果被丢弃 | 同任务、同坐标锚、同语义段，仅窗口代次递增时，保留一个正在检查的候选及其原期限；新结果进入既有的单个最新排队槽 |
| 旧窗口提交确认 → 新窗口准备 | B 窗口成功确认迟到，误清 C 窗口的待接收、重试和异步准备状态 | 提交 B 的有效几何，但只退休与 B 身份一致的待提交项；C 及实时进度保持不变 |
| 接入拒绝 → 下一候选 | 已无法接入的固定曲线不断收到新证明，永久占据换轨预约；更新候选无法进入，旧曲线最终耗尽 | 临时状态缺失继续等待实际新状态；确定不能接入的候选只释放自己的预约，保留当前轨迹、后续候选及控制器历史 |

普通窗口更新不是新任务；候选失败不是停止当前有效轨迹的理由。控制输出仍由
控制周期推进，新确认到达不直接重置速度或加速度状态。SDK 发送仍沿用已有的
最新命令及定相发送机制，没有增加第二层碰撞判断。

楼梯入口、平台、出口的语义段切换不套用“普通滚窗”保留规则；不同锚、不同任务、
取消和真实负结论仍按原合同退休。没有续写原始传感器时间，也没有延长候选、证明
或命令的有效期限。尚未取得新曲线时不能无限外推有限旧曲线。

### 同输入对照与连续输出验证

- Python 参考窗口竞争：原默认模块 **4/8 失败 → 8/8 通过**；另有参考、滚窗、
  楼梯平台、分段和部分楼梯合同 **401/401 通过**。
- 原生生产 GridMap/验证器：同一完整检查中插入滚窗，修改前 `proof_delivered=false`，
  修改后为 `true`，后续窗口可提交；原源时间 `100.050 s`、原 offer 截止
  `100.320 s` 均不变。候选专项 **22/22**，含连续五次滚窗、真实占据、取消及段切换。
- 跟踪器预约：相同三项输入 **2 项失败 → 0 项失败**；完整 follower **77/77**。
  拒绝不可接入候选后下一条可进入；临时状态过期则等真正的新状态，不提前取消。
- 全套原生回归 **69/69**（地图 7、搜索 5、优化 10、SCAN 37、Tracker 10）通过。
  原有“滚窗必须清候选”的测试断言按已证实的新合同改为“保留候选 → 真负结论退休
  → 新候选匹配实际当前曲线”；被动回调夹具仅补相应存储字段，未删减危险反例。

另行构建启用真实 SDK **传输核心**的内存联合测试，未创建 SDK 会话。运行
SDK 时序 **15/15**、滚窗换轨 **5/5**、实测起点闭环 **2/2**。十组候选/ACK 相位中，
注入 10 Hz 候选流、50 Hz 采集、20 Hz 控制，并包含坏候选和提交拒绝：

| 联合测试量 | 结果 |
|---|---:|
| 稳态内部意外零速次数 | 0 |
| 最低稳态前进指令 | 0.2999568 m/s |
| 全程最大线加速度 | 0.35 m/s² |
| 全程最大线 jerk（加速度变化率） | 0.70 m/s³ |
| 最大角加速度 / 角 jerk | 0.02047 rad/s² / 0.61520 rad/s³ |
| 映射后 SDK 幅度一阶 / 二阶变化率 | 不超过原配置 1.6 /s、12 /s² |

这些检查覆盖起步、换轨及回退，不是只判断速度非零。10 Hz 是测试注入的结果流，
**不能视作生产求解吞吐率测量**。闭环中的机器人响应为明确假设，指令平滑不等于
实机加速度已验收。真实过期、碰撞、取消及失权仍会停止；没有用旧数据掩盖它们。

### 证据与复现

证据固定在 `experiments/window_handoff_20261024/`：三个 `delivery.json` 保存原默认
输入、修改前失败与修改后结果；`regression.json` 保存联合测试 ELF、实际默认控制
配置、逐相位指标及原始 XML；`independent_review.json` 为交叉审查记录。
日志保留了结果提取器首次遇到 ANSI 色码的解析错误；22 项测试本身均成功，修正
提取器后对原日志、XML、ELF 逐 hash 核对重新收集，没有覆盖或伪造失败结果。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
# 生产组件内存回归；不初始化 ROS，不连接 SDK。
export D1MAX_ROLLING_SOURCE=/home/dndx/d1max_nav_ws/experiments/navigation_runtime/source_snapshot/nav_ws
experiments/window_handoff_20261024/sdk_timing_build/test_native_sdk_timing
experiments/window_handoff_20261024/sdk_timing_build/test_rolling_follower_native
experiments/window_handoff_20261024/sdk_timing_build/test_measured_start_rolling
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner \
  -LE launch_test --output-on-failure
ctest --test-dir experiments/stairs_continuity_20261022/tracker/build/d1max_trajectory_tracker \
  --output-on-failure
```

本轮证明的是三条确定的错误时序已修复，且联合组件的正常换轨未引入零速与超限
加速度变化；不把它解释为所有实机停顿已经消除。实际传感器丢帧、调度长尾及现场
制动表现仍须由后续实机记录核对。

### 同一默认包已同步

默认 manifest 为 `cea324056bbf614af5168d34731951370b07ae400319b6dd90753079cb276ed8`，
仍使用 `experiments/navigation_runtime` 和原入口，没有创建分支或另一份启动包。
更新 15 个目标：9 份源码/测试快照、2 份实际 Python 模块、2 份安装头文件、2 个 ELF。
两份默认选择器同值；独立检查全部 **10,346 个封存成员**通过，7 个地图、42 个 SDK
文件、85 个配置均未改，10 份生成节点参数除会话身份外等价。

实际导入的两个参考模块位于默认包 `application/install`，SHA 与审核后的源码一致。
Web 只读 overview 返回新 manifest、`ready`、`startup_closure_verified=true`，
导航与数据连接均 `stopped`，运动未授权。未启动导航、RViz、ROS 图或 SDK，未提交/push。
旧文件和选择器位于本轮 `deployment/before/`；部署清单和最终核对分别为
`deployment/install_plan.json`、`deployment/independent_installed_audit.json`、
`deployment/reference_imports_web.json`。

```bash
cd /home/dndx/d1max_nav_ws
python3 -B experiments/window_handoff_20261024/sync_default.py --verify
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
source experiments/navigation_runtime/application/install/local_setup.bash
python3 -B experiments/window_handoff_20261024/verify_delivery.py \
  --manifest cea324056bbf614af5168d34731951370b07ae400319b6dd90753079cb276ed8
python3 -B experiments/window_handoff_20261024/verify_reference_imports.py
```
