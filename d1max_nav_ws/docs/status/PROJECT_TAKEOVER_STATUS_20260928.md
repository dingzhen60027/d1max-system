# D1 Max 单楼层导航接手状态（2026-09-28）

## 主线命名更新（2026-10-04）

架构目标是室内外、多楼层导航；单楼层仅是当前测试范围。公开名称统一为
`d1max_pct_scan.navigation_session` / `tools/navigation_entry.sh`，Web 使用
`/api/navigation-session`。新旧名称透传同一实现、锁和 BT 任务所有权，旧封存包
仍精确加载自己记录的旧模块。没有修改 selector、封存清单、楼梯范围或物理验收标志，
也没有构建/激活新的导航发布包。以下版本名和历史命令保留，不能当成另一条架构。

## 主线更新（2026-10-03，历史记录）

用户确认当前 BehaviorTree.CPP 导航框架为唯一开发主线，正式会话保持
`d1max_pct_scan.single_floor_session`，统一入口保持 `tools/single_floor_entry.sh`。
后续单层、跨层、室内外能力沿同一套模块合同推进，不另建任务管理者、导航上层或
竞争 SDK 的运动链；算法实现可替换，主线架构不分叉。当前 Git 分支继续使用，
没有新建、切换或合并分支。

截至本次更新，开发版定位、独立连续局部状态、固定全局路线、实测进度参考管理、
生产 GridMap/SCAN、候选验证交接、跟踪及唯一 SDK writer 已接入统一候选包。
`experiments/single_floor_execution_20261003_system_v2` 是主线的隔离构建与验证产物，
不是第二套系统。旧预览和旧协调器仅保留兼容，不作为正式任务/运动入口。

新隔离整链已有长直线地图校正、静态/动态绕障、断流恢复、取消和封路退出记录；
这不抹除下方 09-28 的失败证据，也不代表物理导航或流畅性已验收。
局部资源等待的最新修复已完成构建及生产方法纯回归，尚未跑新版整图；此前静态箱体
最长约 1.8 秒停顿不能直接宣称消除。50 Hz 定位链已接通，硬 20 ms 连续输出仍未通过。
具体实现与证据见 [实施报告](../reports/D1MAX_SINGLE_FLOOR_EXECUTION_20260927.md)。

当前先收敛框架、模块衔接和局部连续性，按用户要求暂不继续完整 rosbag 验证。
默认 `deploy/single_floor_release.json` 尚未切换，新候选没有自动部署或物理授权。
源码主线、实际运行 release、软件回归及现场验收是四种不同状态。

---

以下保留 2026-09-28 原始核查，供追溯当时版本；其中“最新”“未实施”等表述仅指当日。

写给：项目负责人和接手本项目的工程师。

本次核查时间：2026-09-28 10:06–10:10，Asia/Shanghai。依据当前磁盘源码、实际安装产物、构建日志和测试 JSON；不是依据对话摘要认定完成。此次只读核查运行代码，新增本文；未重编译、重跑导航图、连接 SDK 或修改物理验收标志。

## 1. 当前结论

工程已具备正式任务与执行链，并有短路线软件闭环通过记录；**稳定、流畅的单楼层导航尚未验收通过**。最新长直线复测失败，静态和动态箱体绕障没有通过记录。下一阶段应收敛单楼层整链问题，不扩大跨楼层执行范围。

“源代码已修改”“安装产物已更新”“整链验证通过”目前是三个不同状态，不能合并报告。

## 2. 已有系统与当前入口

正式入口为 `python3 -m d1max_pct_scan.single_floor_session`，具有 prepare / seal / verify / run / view。当前会话要求原坐标 floor1 地图和 `stairs_enabled=false`。

执行职责：BT 唯一任务所有者 → 常驻 PCT 全局规划 → map/odom 坐标锚与局部参考窗口 → GridMap / SCAN → 原生验证与 tracker 接纳 → BT 提交 → tracker 需求 → 实际运动扫掠验证 → 安全门控 → SDK 唯一写者。

- 全局：原坐标一楼子图，`native_astar_checked_smooth`；不能把它称为 GPMP 已修复。
- 局部：连续 odom 坐标，已提交几何与新候选分离；窗口长度 2 m。
- 验证：未知空间不放行，源时间和任务/执行身份显式核对。
- 测试：Domain 219、loopback Zenoh、控制驱动的机器人替身及模拟传感器/SDK。这些测试不证明真实 Faster-LIO/GICP 定位与机器人执行全流程已经验收。
- 物理参数：`src/d1max_scan_planner/config/d1max_robot.yaml` 仍将 body_reference_height_m=0.55 标为未标定。用户提供的雷达离地约 0.5 m、轮距约 0.5 m 是现场信息，不能据此替代完整轮腿扫掠包络及制动测量。

## 3. 核对到的整链结果

下表时间为报告的整个测试耗时，包含启动、执行确认和停止阶段，不是净运动时间。

| 证据 | 场景 | 结果 |
|---|---|---|
| `experiments/single_floor_execution_20260927/suite_02/suite.json` | 短路线、取消、后雷达短断流、地图校正、终点朝向、保护距离外封路 | 六例均达到各自测试终止条件；取消/封路是预期退出，不是导航到达 |
| `experiments/single_floor_execution_20260927/suite_07_control_join/suite.json` | 长直线 | 到达；2.986 m / 41.027 s，尚不满足流畅性目标 |
| 同上 | 长路线地图校正 | 到达；2.975 m / 41.038 s，全局路线哈希与直线案例相同 |
| 同上 | 静态箱体 | 失败；0.882 m 后受阻，`actual_command_blocked:motion_sweep_occupied` |
| 同上 | 动态箱体 | 失败；0 m，`follow_trajectory_timeout:waiting_recheck` |
| 同上 | 不可观测起点 | 按预期拒绝，0 m；不能把该 exit_code=0 解释为导航到达 |
| `experiments/single_floor_flow_20260928/long_straight_00_newnative/graph_report.json` | 最新长直线 | **失败**；2.856 m / 53.027 s，`execution_blocked_timeout:waiting_admitted_nonzero_command` |

所有这些报告的 `physical_acceptance` 和 `physical_stop_confirmed` 均为 false；模拟测量停稳与实机停稳必须区分。

最新失败的具体证据：

- 15.5967 s 的 BT permit 首次提交 reference_generation=3、trajectory_id=8。
- SCAN 局部目标约为 (-0.116, -2.91, 0.561)，随后打印 `MEASURED_GOAL_REACHED` 并进入 WAIT_TARGET。
- 21.3952 s tracker 报 `local_segment_finished_waiting_replan`。
- 最终局部位置 (-0.116168, -2.853111, 0.557843)；相对最终目标 XY=(0,-3.15)，水平误差约 **0.3188 m**，超过 0.20 m 到达门槛。
- 之后没有提交下一代有效参考，任务耗尽受阻预算失败。

## 4. 主要未完成问题及证据强度

### A. 局部窗口耗尽与最终到达之间有衔接漏洞（最高优先级）

源码 `src/d1max_pct_scan/d1max_pct_scan/continuous_reference_node.py:79` 仍以 `confirmed_arc_m - last_window_arc >= 1.0` 判断窗口续发；`:231` 在 **BT 提交时**写入 last_window_arc。窗口几何却在更早的 propose 时生成。

因此晚提交会减少剩余行程，可能在触发下一窗口前先到达局部终点。现有日志与这一机制吻合；**缺少提交瞬间完整进度记录，因果链仍需用延迟提交回归测试封闭，不能写成已修复**。参考节点源码和安装副本 SHA256 相同，该旧条件确实还在当前版本。

建议：根据已接受窗口的剩余路线弧长续发，保留任务身份、最终段覆盖、坐标锚、观测新鲜度和提交协议；不能靠扩大最终目标容差掩盖停在中途的问题。

### B. 感知证据时延与轨迹切换仍影响连续性

最新测试曲线检查 P95=17.50 ms、检查时最老射线源龄 P50/P95=359.21/440.08 ms；相对 500 ms 上限余量偏小。当前会话融合配置仍为 **5 Hz**，此前讨论的改成 10 Hz 尚未实施。最新测试 tracker 记录 accepted=11、rejected=25，但拒绝原因尚未完整输出；这些计数不能直接当作独立曲线成功率。

下一步应记录融合、快照、验证、回执和最终控制的时序，并补足具体拒绝原因。不能仅凭旧日志比例把最新失败全部归于过期，也不能扩大 TTL 或保留过时运动指令来制造连续性。

### C. 局部曲线可行与实际控制可行仍不一致

静态箱体失败明确来自实际指令扫掠占据拒绝；局部候选通过并不保证前进、转向及刹停扫掠可执行。需要核对规划、跟踪与扫掠使用的同一机器人几何、速度和制动假设。动态箱体最新完整套件尚未起步，不能报告为已验证动态绕障。

## 5. 已改但未完成交付的代码

| 文件 / 改动 | 磁盘核查状态 |
|---|---|
| `execution_validator.hpp` 补 nlohmann/json.hpp include | 05:19 的 native 构建成功，修复此前编译失败 |
| `test_reference_callback_no_ros.py` 补 stub 成员 | 05:20 单独重跑通过；旧 LastTestsFailed.log 仍保留旧失败，不应单看它判定当前失败 |
| `execution_validator.hpp` + `scan_replan_fsm.cpp` 快照发布唤醒验证线程 | 源码 05:52 更新；当前 scan_planner_node 安装产物仍为 05:19，未见包含该改动的新构建或整链结果 |
| `tracker_core.hpp` 候选拒绝原因与 trial 原因回传 | 源码 05:41 更新；tracker 安装产物仍为 03:00，未完成重新构建和整链验证 |
| tracker 准备回执、permit 拒绝原因、节点状态输出 | 仍有泛化拒绝原因，prepare 仍为 0.2 s 周期；此前计划尚未完整落地 |
| 参考窗口续发修复 | 未实施 |

已有 native 测试证据：plan_env 6/6、path_searching 4/4、bspline_opt 4/4；scan_planner 15 项首跑有一个 stub 编译失败，随后该项单独通过。两项 ROS 图测试被排除。这些证据不覆盖 05:52 的新代码。

工作区大量导航包、docs 和 experiments 尚未纳入 Git 跟踪。不能用 `git diff` 的空白或单个 HEAD 描述整个工作版本；接手及后续封存需保留文件清单、哈希和安装产物对应关系。当前已有备份 `experiments/single_floor_flow_20260928/before_sources.tar.gz`，不覆盖它。

## 6. 跨楼层与旧研究的地位

- `maps/processed/sc_pgo_20260923_crossfloor_complete_001/route_001/audit.json` 的状态为 `validated_offline_route`，execution_authorized=false；跨层全局静态几何路线有通过记录。
- 当前正式说明见 `docs/design/CROSSFLOOR_GLOBAL_PLANNING.md`。旧 `experiments/crossfloor_pct_20260923/HANDOVER.md` 早于正式跨层处理链，不应作为最新总状态。
- 外部研究 workflow 的 journal 保存六份 research 和一份 critique。终态 report=null，部分 agent/复核/综合失败；不能声称已经获得完整、交叉验证过的最终架构报告。本次没有重新发起该 workflow。

## 7. 接手后的执行顺序

1. 固定当前失败为回归用例；记录提交时的窗口起止弧长、真实进度和下一窗触发原因。
2. 修复窗口续发与最终到达衔接，补晚提交、窗口尾端、最终段和坐标校正测试。
3. 完成诊断输出，重建 native / tracker / application 对应隔离安装；核对源与安装哈希后再测，避免用旧二进制验证新源码。
4. 先复测长直线和地图校正，分别报告启动时间、净运动时间、停顿时长/次数、终点误差和路线身份；多次重复才能说明稳定性。
5. 统一候选与实际控制可执行性，验证静态箱体、动态箱体，再回归取消、断流、未知空间、停稳和任务退役。
6. 单楼层软件整链达标后，再组织实机几何、速度映射、制动、MC 时间和原始射线验收；不把模拟记录改成物理通过。

历史实施说明：`docs/reports/D1MAX_SINGLE_FLOOR_EXECUTION_20260927.md`、`experiments/single_floor_execution_20260927/BT_SDK_IMPLEMENTATION.md`。最新整链失败证据优先于这些早期文字中的成功描述。
