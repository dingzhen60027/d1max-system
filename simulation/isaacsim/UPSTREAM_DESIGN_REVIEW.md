# PCT 与 SCAN 上游设计审查

审查日期：2026-10-07。结论来自固定提交的源码和原论文；本轮没有编译、执行上游脚本、启动仿真、安装模型或替换本项目。

**最值得吸收的是三维参考路线的几何表达、明确的高度合同、实测位置驱动的窗口推进，以及规划与步态控制的分层。** 上游演示的连续运动不能直接证明当前问题已解决：原版 SCAN 的膨胀查询并不要求未知体素具有观测 FREE 证明；一些集成演示直接发布曲线位姿；另一些采用最低行走速度、弱化 Z 投影。本项目的真实关节、完整 13 形体、原始 Z、源时间／接收时间证明和实测停车仍须独立验收。

**本项目仍未通过正式动态导航关卡。** v36 累计 XY 行程 7.221877 m、199 次原路线 credit，指定演员遭遇成立，但持续 hold 后任务 FAILED；见 [v36 完整失败审计](verification/20261007/campus_crossing_v36.json)。随后 v37 修复了 PhysX 返回刚体根路径而旧代码只识别 collider 子路径的问题，350 项相关 Python 回归通过，正式首尾雷达也出现真实演员标签；原动态关卡却在到达演员附近前，以 `execution_blocked_timeout:waiting_measured_motion_progress` 失败，累计 XY 行程仅 0.471879 m。不能以该轮验证动态占据退役或宣称流畅导航；见 [v37 完整失败与持续有效指令下的原地站立证据](verification/20261007/campus_crossing_v37.json)。下文 v35 数值保留为研究时的历史证据；上游设计比较也不是本地通过证明。

## 1. 固定来源与审查范围

所有 GitHub 代码链接指向下列不可变 SHA；分支名只说明取样来源。外部源码目录为 `/home/eric/wjg/d1max-build-isaac/upstream-research/`，各 checkout 均为 detached HEAD、depth 1、源码 sparse checkout。大型点云、模型权重和视频没有用于本轮验证。

| 来源 | 取样分支 | 固定提交 | 本轮定位 |
| --- | --- | --- | --- |
| [byangw/PCT_planner](https://github.com/byangw/PCT_planner) | `main` | `35cd73fd82bcd51bc538429294af7646b2a09815` | 原作者全局多层规划 |
| [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) | `main` | `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` | 原版 ROS 1 局部规划／地图／控制器 |
| 同一 SCAN 仓库 | `ros2-community` | `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0` | ROS 2 社区移植与传感器接线 |
| [yagami-light7/PCT-SCAN-ROS2](https://github.com/yagami-light7/PCT-SCAN-ROS2) | `main` | `1cf45a1c12ca785ce533293302be09d51daf0ccb` | PCT→SCAN 桥接、参考折线、Building 演示 |
| [lovelyyoshino/3D-Nav-ROS2](https://github.com/lovelyyoshino/3D-Nav-ROS2) | `main` | `46b3d9a6668e01cd9c9377f381252b323f728fad` | PCT＋EGO、A1 RL 控制；并非 SCAN 集成 |
| [Robot-Nav/legbot_3D_Nav](https://github.com/Robot-Nav/legbot_3D_Nav) | `main` | `f60606a4903bad58fca813f76072f9940a284d1d` | ROS 1／Gazebo Classic／A1 集成 |
| 同一 Robot-Nav 仓库 | `ROS2` | `fe6aed6c5595773a1a9421d94117914bb27b1ec7` | ROS 2／GO2、路线／速度适配与 RL 合同 |

论文主源为 [PCT 原论文 arXiv:2403.07631v1](https://arxiv.org/abs/2403.07631v1)和 [SCAN 原论文 arXiv:2606.19555v1](https://arxiv.org/html/2606.19555v1)。本轮区分源码事实、作者报告的实验和对本项目的推论，没有把 README 视频、源码存在或离线测试文件等同于已复现实测。

## 2. 高度与速度合同：不能统一假设 PCT 输出地面 Z

PCT 的 tomogram 保存 traversability、地面与顶板高度，按坡度、台阶、净空等能力参数筛选通行区域；Building 示例的 `interval_min=.50`、`interval_free=.65`、`step_max=.17` 是示例机器人的能力设定，不是任意四足的物理保证。它适合生成多层三维路线，不能替代在线动态感知和脚部接触验证。[tomogram 核心](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/tomography/scripts/kernels.py#L100-L224)、[示例能力参数](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/tomography/config/scene_building.py#L8-L22)。

**原版 PCT 两个优化分支的输出 Z 语义不同。** 默认配置 `use_quintic=True`；该分支使用 `GetHeight()+reference_height_`，默认偏置 `.1 m`，再进行受顶板限制的高度平滑。非 quintic 的 WNOA 分支直接取地面高度，相关高度平滑仍注释。Python wrapper 将优化器返回高度导出为三维路线，不能仅凭 `/pct_path` 名称认定它是地面线。[默认分支](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/planner/config/param.py#L1-L3)、[quintic 高度](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc#L167-L178)、[默认偏置](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h#L62-L68)、[WNOA 分支](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.cc#L211-L220)、[导出路径](https://github.com/byangw/PCT_planner/blob/35cd73fd82bcd51bc538429294af7646b2a09815/planner/scripts/planner_wrapper.py#L89-L118)。

原版 SCAN 收到外部路线后统一加 `body_height`，示例值 `.4 m`；ROS 2 社区分支保留了此行为。**推论：** 如果直接接入上述默认 quintic 结果，平地名义中心可能成为地面＋`.1+.4=.5 m`，而不是调用方以为的 `.4 m`。必须检查具体 producer、优化分支与已存数据，不能从桥接注释倒推原版 PCT 语义。[ROS 1 路线入口](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp#L357-L400)、[ROS 2 路线入口](https://github.com/wuyi2121/SCAN-Planner/blob/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0/src/planner/plan_manage/src/scan_replan_fsm.cpp#L339-L358)、[body 参数](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml#L41-L50)。

Robot-Nav 的 ROS 1 集成改为路线接收时对齐一次 `odom.z−first_path.z`，保留相对高度变化，不再另加身体常量。这揭示了高度合同错位会拖住跨层进度，但自动对齐需要起始位置／楼层可信，不能每帧拿测量重新移动地图或隐藏真实 Z。[一次高度对齐](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L390-L437)。

本项目 `.481 m` 来自官方 Spot 冷启动稳定站立实测，`.589 m` 上方包络保留原名义绝对顶高 `1.07 m`；13 个实际形体和脚部接触容许量没有缩小。该标定与“动态调 Z 让投影容易前进”不同，历史 `.52/.55` 候选也保留。见[冷站标定与体素预算证据](verification/20261007/spot_cold_stand_calibration.json)。

**追加实审：当前项目正式链没有照搬上游的高度叠加。** 下列引用是本项目当前工作树源码；审查时 Git HEAD 为 `36dbb98396dd829a79c187f835d4aea211b60e3c`。PCT worker／TomogramRoute 显式使用 `ground_z=True`；[planner_core.py](../../d1max_nav_ws/src/d1max_pct_planner/d1max_pct_planner/planner_core.py#L194-L206) 按真实保留层重取 measured ground，替换上游 `.1 m` 参考高度及历史 wrapper `.5 m` 偏置。完整 quintic XY 分区再从 [tomogram.surface_ground_z](../../d1max_nav_ws/src/d1max_pct_planner/d1max_pct_planner/tomogram_route.py#L74-L100) 取得 Z；[连续参考窗口](../../d1max_nav_ws/src/d1max_pct_scan/d1max_pct_scan/continuous_reference.py#L317-L338) 只在 `z+self.body_height` 应用一次身体高度，随后按原 map-to-odom 转换。上述三个文件字节均与该 HEAD 相同，SHA256 分别为 `6500e9b0b20d46d1677004bd552e984f0b4341f10b7af95e4e6e68afc8c3f661`、`17409c7d5bc48774ca301cdfe7d097db33a9751a1ab0f2255ab99fe360edbd31`、`3640cbe13f42046395c027958d70a4b759eb4ab5284d3f1dfe6464741afe3119`。

[参考 transport](../../d1max_nav_ws/src/d1max_pct_scan/d1max_pct_scan/continuous_reference_transport.py#L48-L62) 要求 native Z offset 为 0；[SCAN execution 初始化](../../d1max_nav_ws/src/scan_planner_vendor/plan_manage/src/scan_replan_fsm.cpp#L118-L121) 也拒绝非零 offset，而 [body_center 路线入口](../../d1max_nav_ws/src/scan_planner_vendor/plan_manage/src/scan_replan_fsm.cpp#L579-L589) 再检查一次。当前正式链未发现重复加高证据，不能再统一减 `.1/.5 m` 或把 Z 拉平。旧 no-motion 的 ground 路线 preview 是另一项显式偏置合同，不能混用于正式 body-center 执行。

速度也必须分层：参考路线的时间参数、局部 B-spline 的导数、身体 `vx/vy/wz` 指令、policy 输入、真实全 XYZ twist 是不同量。PCT 的离线路线优化并没有给出 Spot 低速跟踪／停车能力；地面机器人的 Z 由接触与步态产生，三维规划仍须约束高度、净空、坡度和完整体积，不能把空中轨迹的 `vz` 直接当成四足可执行自由度。

## 3. 全局路线截取与局部窗口

原版 SCAN 对外部点按约 `.5 m` 间距采样，再构造全局时间曲线；局部目标由起点在全局曲线上最近投影后，沿完整 XYZ 弧长累加到 planning horizon。这个分层值得保留；但按距离稀疏化可能丢短拐角，最近点全域搜索也需要防止回环／跨层段别名。重规划时还可能用旧曲线导数替代实测速度，不能将该值误当 actual twist。[局部目标与速度初值](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp#L824-L866)、[全局投影／XYZ 窗口](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp#L981-L1082)。

yagami 集成先检查有限坐标、加身体偏置、间距筛选，再使用三维点到线段距离做 RDP 简化；全局参考改为逐段线性 XYZ，避免拟合参考曲线越过原折线。局部窗口执行结束时，如果全局还有后续段则继续 REPLAN。**限制：** RDP 在间距筛选之后，不能保证被先删掉的短角点；全局折线拐角速度不连续，应只作为路线几何，不能直接当满足 C1 的物理换轨。[三维简化](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/include/plan_manage/reference_path_utils.h#L28-L140)、[折线生成](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/include/plan_manage/polyline_trajectory_utils.h#L14-L60)、[作为 global reference](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/src/planner_manager.cpp#L331-L356)、[继续下一窗口](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/src/scan_replan_fsm.cpp#L654-L670)。

Robot-Nav main 额外保留超过 20° 的三维角点，并对全局 route progress 做单调约束，投影距离同时包含 XY 与加权 Z，局部窗口仍按三维弧长推进。可借鉴的是几何保真、保角和局部化投影；还需要限制跨段跳跃、保留路线身份与实际 progress credit，不能只用 `max(previous,t)` 证明机器人走过了路线。[保角点](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L438-L466)、[单调投影与 XYZ 窗口](https://github.com/Robot-Nav/legbot_3D_Nav/blob/f60606a4903bad58fca813f76072f9940a284d1d/src/SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp#L1095-L1140)。

yagami 的 `DynamicLayerTomogramPlanner` 根据实际切片数建图、检查相邻高度有效性，并提供通道中心偏置。这里的 DynamicLayer 指层数适配，源码没有因此获得动态演员轨迹预测；中心偏置可改善全局参考净空，但原真实占据仍必须独立检查。[层数／有效网关／搜索代价](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/integration/pct_scan_bridge/pct_scan_bridge/dynamic_planner.py#L13-L81)。

## 4. UNKNOWN、raw 占据、膨胀与动态残留

原版 SCAN 原始 log-odds 初始为 UNKNOWN，膨胀 buffer 初始为 0；`isKnownFree` 与 `isUnknown` 有独立定义。但 front/rear 双圆柱的 `getInflateOccupancy` 只读膨胀位，不调用 observed FREE 判断。**所以地图内未知格若未被占据邻居膨胀覆盖，该查询可返回 0。** 地图外返回 −1；这与本项目“未知不能直接取得执行权限”的合同不同。[初始化](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L104-L111)、[raw 状态与膨胀查询](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/include/plan_env/grid_map.h#L303-L382)。ROS 2 社区版保留这种查询方式。[移植版查询](https://github.com/wuyi2121/SCAN-Planner/blob/d0b921c9b05a6d291d144d60882b2e0e88d2c0e0/src/planner/plan_env/include/plan_env/grid_map.h#L371-L388)。

动态旧 OCC 主要依靠新 ray MISS 降低 log-odds，跨过占据阈值时递减膨胀引用计数；滑窗回收地址会重置该区域。审到的融合路径没有“仅因时间过去就删除旧占据”的规则，也没有将演员运动方向转换为未来时空可达集合。按示例 `pmax=.98, pocc=.80, pmiss=.30`，纯 MISS 条件下饱和 log-odds 从约 3.892 降到阈值 1.386 以下需要 3 次融合更新；这只是算术，遮挡、未观测、混入 HIT 或每批多数投票都会改变实际清除时间。[阈值穿越／膨胀计数](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L270-L292)、[HIT／MISS 批融合](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_env/src/grid_map.cpp#L655-L687)、[概率参数](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml#L66-L71)。

原论文的边界虚拟 FREE 层只用于搜索假设，恢复的最终路线须回到真实滑窗内；不能把这项设计解释为给 UNKNOWN 空间执行授权。论文报告的是通过 log-odds 适应缓慢移动的人体干扰，并未给出本项目双传感器同源配对、冻结先验身份或未来六秒 actor oracle 的证明合同。[SCAN 论文 §V-C、§VI-D](https://arxiv.org/html/2606.19555v1#S5.SS3)。

本项目应继续分开诊断三件事：raw UNKNOWN 缺观测、raw OCC 的历史或当前实测、机器人形体导致的 inflated OCC。只修膨胀 buffer 不会清除源占据；只把未知标 FREE 会消除必要阻断。可信静态先验应排除动态演员注册体积，paired ray 的 fresh MISS 才能作为在线清除依据；机器人自身返回、地板与非支撑障碍也要保持各自真实语义。

v35 最后一次 bounded query 命中**原六秒未来动态可达球**，即使当前演员没有实际贴近机器人也可能正确拒绝 UNKNOWN。该查询与最终 current-command `motion_sweep_occupied` 分别记录。上游的当前占据图不具备同一预测合同，不能靠切换成“inflate=0 就走”来宣称根治。

**后续实现与适用范围：** v36 已为封存、由仿真实际执行的 C1 脚本演员增加完整六秒、全 XYZ 的连续包络，保留原始源时间、接收时间和当前实际位姿校验；不把普通行人的未知意图当成该脚本。另增加原生 HIT 的精确演员归属，只有纯该演员的历史冲突、更新鲜的有效演员证据、完整未来包络与闭体素分离及独立静态 FREE 先验同时成立，才使用静态证明；原 raw log-odds 和 UNKNOWN 均未清空。见 [脚本未来包络离线证据](verification/20261007/scripted_actor_future_v36_offline.json)和 [原生命中归属离线证据](verification/20261007/native_actor_hit_provenance_v36_offline.json)。v36 正式运行仍失败，10 秒组件实测才发现 native 返回刚体根路径，v37 改为只接受注册根或精确子碰撞体；相似前缀、未注册子体和歧义均拒绝。见 [根路径组件证据](verification/20261007/native_root_hit_v37_component.json)。v37 又被实测运动进度阻断，这些改动仍不能表述为完整动态导航通过。更一般的方向性预测、滚动扫掠、未知行为和 stopping tube 仍需另行设计与回归。

## 5. Tracker、连续控制与低速死锁

原版 SCAN closed-loop 在偏航误差大时冻结执行时间并原地转向，否则按 ROS 时间递增 `exec_time`，使用 XY 导数前馈＋位置反馈，转换成身体 `vx/vy/wz`；完成判断也是该局部轨迹时间与 XY 误差。它没有在这段控制器中用实际行程推进执行时间。时钟持续走可让速度前馈离开静止端，但也可能在机器人停住时沿轨迹时间跑远；不能直接取代本项目实际三维 progress 与过期撤权。[原版控制循环](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/closed_loop_controller.cpp#L182-L224)。

Robot-Nav ROS2 改用实测最近点＋时间前视，并明确提及“小非零指令→policy 不踏步→最近点仍 t0”的启动死锁。它默认 `tracking_z_weight=0`、`use_goal_z=false`，楼梯最低速度 `.30 m/s`，另有可配的最低行走速度，后者默认 0。可参考问题描述，**不能直接复制去 Z 或强制最低速度**：当前证明授予的 cap、可执行净空、末端微速和真实停车都可能被这种处理改变。[参数与投影](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp#L100-L175)、[最近点／最低速度处理](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp#L391-L469)。

这个适配器还有可研究的独立机制：目标戳绑定停止 ACK、偏航转向进入／退出滞环、近 180° 时固定转向方向、指令加速度限制。发送零指令与身体停稳仍是不同证据；单纯 goal ACK 不能取代真实 full-norm stop、父任务退役和新目标身份隔离。[目标停止与 ACK](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp#L345-L378)、[平滑输出](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp#L201-L219)、[转向滞环](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/scan_planner/algorithm/plan_manage/src/go2_cmd_adapter.cpp#L481-L501)。

对本项目的建议是先让局部曲线的起点／高度／时间参数与实际 plant 相容，再在已验证的真实 XYZ 窗口里选取足够空间前视，形成有上限、能制动的连续需求。证明到期或真实碰撞必须立即停止；有效新证明可在不重置路线和实际状态的情况下续接。C1 检查应比较真正可执行的边界并保留原限制，不能只清零起始速度以使拟合好看。v35 的 30 段输出、中位源跨度 `.510 s`、最长 `8.700 s` 和 58 次未成功换轨说明已有持续跟随，也仍有证明／换轨阻断；不能把所有失败唯一归因 UNKNOWN、高 C1 或 policy。

**v37 对上述推论的实际区分：** 原 writer 在场景源时间 9.74–37.72 s 连续正向输出，15–35 s 的候选／运动证明持续 `observed_free`／`observed_free_motion_sweep`，tracker 持续 `tracking`。同期真实输入约 `.13886 m/s`、内部 policy 输入精确为 float32 `.3000000119`，实际净 XY 位移仅约 `18.4 µm`；物理和推理计数仍推进，12 个关节位置变化范围只有约 `6.19e−5–1.51e−4 rad`。最终是原实测进度 watchdog 先超时撤权，随后 tracker 退休与零输出。4 次 writer ACK 的全 XYZ C1 误差最大 `.02483 < .05`，没有负 ACK。这一段已经排除 UNKNOWN 撤权、换轨中断或仿真停时作为主要解释，却不能仅从观测确定神经策略内部机制。动作历史是否形成站立吸引子需要独立物理 A/B，不能据此加入最低速度或宣布策略记忆重置为解决方案。原数据、采样与证据限制见上述 v37 报告。

随后完成的[独立动作历史 A/B](verification/20261007/spot_navigation_replay_v37_ab.json)只重构原保存事件，在裁剪平地上以真实关节执行 42 s。基线和记忆干预组均能行走，净前进约 3.47／4.52 m；它没有复现正式停滞。干预前 4189 次测量逐值相同，支持组件内差异由该干预造成，仍不足以宣布原导航根因闭环。基线 full XYZ 峰值 `.532835 > .50`、干预组 full angular 峰值 `.823027` 及单独的 body yaw `.173694` 均保留，两个模型域不混同；原零命令起点的停车确认分别 2.056／1.832 s。该实验未改变默认步态或放大导航模型域。

## 6. 步态模型与实验到底证明什么

| 源码／实验 | 能确认的内容 | 不能据此确认的内容 |
| --- | --- | --- |
| 原版 SCAN `go2_kinematic_sim`＋`go2_gait_publisher` | 指令转世界坐标后积分 XY／yaw；正弦关节动画 | 动力学速度响应、真实抬脚接触、完整形体、惯性停车 |
| yagami Building 默认启动 | `open_loop`、局部 sensing 默认关闭、紧凑演示 footprint；直接发布 XYZ 曲线 odom | 真实机器人沿楼梯执行、感知闭环、四足控制可行性 |
| lovelyyoshino 3D-Nav | PCT＋EGO，A1 RL history／实际关节目标源代码 | 该版本作为 SCAN 集成的有效性、当前 Spot 低速能力、独立实机完整验收 |
| Robot-Nav main | A1 12 关节 RL／Gazebo 源码、高度和 route 改造；作者报告导航结果 | 相同 Isaac Spot 传感器、cap、体积和动态预测合同下的复现 |
| Robot-Nav ROS2 | GO2 ros2_control、观测顺序／history／模型格式合同及关节输出 | 权重直接装到 Spot 可用、低速任意 history 都能走、仅零指令就已停稳 |
| PCT／SCAN 原论文 | 作者分别报告全局三维规划与 Go2 导航实验 | 本地封存候选在同一场景、同一控制器／时效条件下已经通过 |

原版 Go2 demo 的 `x/y/yaw += cmd*dt` 与正弦 gait 发布器明确不是 PhysX 关节 plant。[运动学模型](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/go2_kinematic_sim.cpp#L108-L135)、[动画关节](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/go2_gait_publisher.cpp#L120-L144)。

yagami Building 默认 `controller_mode=open_loop`、`enable_local_sensing=false`，半径／偏置 `.12/.12` 被源码标为 visual-demo footprint；open-loop 从 B-spline 计算位姿并直接发布 odom。改成 closed-loop 才启动另一个固定 Z 的运动学积分模型，仍没有关节动力学。不能混淆两条启动路径。[Building 参数](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/integration/pct_scan_bridge/launch/building_pct_scan.launch.py#L31-L46)、[示例形体／sensing 警告](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/integration/pct_scan_bridge/launch/building_pct_scan.launch.py#L98-L135)、[控制器选择](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/launch/run.launch.py#L167-L220)、[曲线直接发布 odom](https://github.com/yagami-light7/PCT-SCAN-ROS2/blob/1cf45a1c12ca785ce533293302be09d51daf0ccb/src/planner/plan_manage/src/open_loop_controller.cpp#L78-L128)。

lovelyyoshino 版本的 planner 实际是 `EGOPlannerManager`，traj server 发布完整三维 PositionCommand；RL 分支以历史观测调用 policy 并产生 12 关节目标，但本轮检查的该 `real==true` 动作分支被注释。只能确认这些具体路径，不能据此否认作者其他实验，也不能将 README 实机表述当成本地已复现。[EGO 管理器](https://github.com/lovelyyoshino/3D-Nav-ROS2/blob/46b3d9a6668e01cd9c9377f381252b323f728fad/src/planner/plan_manage/src/ego_replan_fsm.cpp#L54-L73)、[XYZ 指令](https://github.com/lovelyyoshino/3D-Nav-ROS2/blob/46b3d9a6668e01cd9c9377f381252b323f728fad/src/planner/plan_manage/src/traj_server.cpp#L174-L231)、[RL 动作路径](https://github.com/lovelyyoshino/3D-Nav-ROS2/blob/46b3d9a6668e01cd9c9377f381252b323f728fad/src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp#L125-L164)。

Robot-Nav ROS2 读取真实 IMU、12 个关节位置／速度／力矩，按 decimation 同步推理，再输出关节 q／kp／kd；模型 profile 显式记录 45 维观测、10 帧 term-major history 和动作缩放。该部署合同值得学习，但传感器四元数顺序、关节顺序、PD、步长、history 布局和训练命令域均需逐项匹配；GO2 模型不能视为官方 Spot 的即插即用替换，也不能从源码判定最低有效步速。[原始状态接口](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/rl_quadruped_controller/src/FSM/StateRL.cpp#L395-L428)、[关节动作输出](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/rl_quadruped_controller/src/FSM/StateRL.cpp#L451-L503)、[模型合同](https://github.com/Robot-Nav/legbot_3D_Nav/blob/fe6aed6c5595773a1a9421d94117914bb27b1ec7/src/go2_description/config/moe_cts_77k/config.yaml#L1-L30)。

SCAN 论文的仿真使用 MARSIM；作者另报告 Go2＋Mid-360＋FAST-LIO2 的实机实验及缓慢动态人体干扰。它支持研究方法有实际应用，但不是本地 Isaac 500 Hz 关节模型、双 native LiDAR、原生 IMU、严格证据时效下的等条件对照。[SCAN 实验设置与实机段](https://arxiv.org/html/2606.19555v1#S6)。PCT 论文报告的是全局多层导航能力，也没有因此提供当前动态局部执行链的证明。[PCT 论文](https://arxiv.org/abs/2403.07631v1)。

## 7. 可实施建议与判定标准

1. **先审 PCT→SCAN 高度合同。** 为具体路线生产版本注明 `terrain_z`、偏置／高度平滑及 `body_reference_z`，只转换一次；起点楼层匹配错误要拒绝。继续保留本项目 `.481/.589` 标定、真实 XYZ、13 形体和旧候选，新增楼梯／顶板／跨层原始几何回归。
2. **采用保角的三维路线表达与有限窗口。** 避免全局平滑穿越原路线，简化后逐段检查完整身体扫掠；窗口以有身份的实际单调路线 credit 推进，跨层重叠与回环不准跳段。局部 C1／制动验证独立于全局折线。
3. **把所有阻断按同一个 cell／source 分解。** 记录 raw 状态、HIT／MISS、inflation 来源、传感器源时刻、演员时域、实际 command sweep 与候选曲线 sweep；修复 stale 动态占据需要 fresh 真实观测或有明确边界的时空模型，不能 TTL 到期自动置 FREE。
4. **用连续合法需求验证低速控制域。** 重放真实 `vx/vy/wz` 与停车历史，逐级检查高层需求、policy 输入、12 关节／四脚与 full XYZ twist；空间前视和重定时先解决需求生成死锁，不能以最低速度、无限积分、root 位姿／速度写入补偿。原 cap、C1 和证明到期 STOP 继续生效。
5. **把改善与通过分开。** 正式大场景至少要同时满足实际路线 credit、任务到达、指定动态遭遇、完整身体无碰撞、地板／原生 IMU 同源、连续有效跟随、实测停稳及软件退役。静态图上的 global path、RViz odom 动画、关节 policy 组件 A/B 或某项单测通过均不替代这些条件。

上游源码审查本身没有运行或替换外部导航工程。上述后续实现独立记录各自源码、组件和正式失败证据；当前选择器和生产 release 未因研究或离线测试切换，动态大场景任务仍未通过。
