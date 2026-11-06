# 楼梯正式链路接入与原生诊断

日期：2026-10-21。唯一入口仍是 `tools/navigation_entry.sh` / `navigation_session`；没有新增导航器、分支或 SDK writer。

## 结论

第一阶段补齐楼梯参考、原始支撑点、原生跟踪合同和 SDK 模式事务；第二阶段进一步修复原生 SCAN 的楼梯来源索引与机身俯仰碰撞模型。**当前同一约23.15 m楼梯通道的上下行原生顺序几何求解均已到达终点容差内；实际双雷达上楼数据的6个窗口全部完成搜索、优化及整曲线检查。** 这不是实机跨层验收，SDK STAIR 的 Move、比例、零速保持及停稳时序证据仍不完整。第二阶段的依据、限制与复现见文末。

**当前默认运行策略以第四阶段为准：全程STAIR，不自动切模式；按用户明确要求复用通用速度映射，空STAIR测量记录不再阻断该运行路径。** 测量证据仍如实保留未完成状态，不作为已完成实机上下楼的声明。

未启动完整生产导航、SDK会话或运动，未修改物理验收标志。第二阶段一次宽范围CTest误包含两项旧launch测试，曾短暂启动旧默认SCAN节点后立即结束；其零断言结果不计入验收，之后全部显式排除launch测试。占据判断仍只在生产SCAN内完成，没有新增跟踪器碰撞门控，也没有清空楼梯、近身或未知区域。

## 第一阶段接线（历史分段模式策略）

1. BT 仍持有唯一任务与执行身份，PCT 的 `RouteSnapshot` 保留楼层、上下楼、平台及入口出口。
2. 参考管理器按实测 XYZ 到达相邻路段入口；普通同层边界维持连续切段。特殊边界发 `ModeTransitionRequest`，绑定任务、路线、路段、SDK 会话与执行代次。
3. 现有 SDK writer 协调零速、原始 MC 停稳证据、一次 `SetMode`、ACK 与之后的新 RobotState；用 `ModeTransitionState` 返回真实确认。等待模式时保留任务，停止非零输出；同为 STAIR 的平台过渡不重复调用 `SetMode`。
4. 模式确认后准备下一局部窗口，仍由原生局部几何 owner 提交。候选失败可重试，不能提前覆盖旧参考；新目标、取消、过期或旧代次反馈不能恢复旧任务。
5. 仅规划会话使用明确的几何预览过渡，不创建 SDK 模式反馈或运动许可。

普通楼层支持路径不变。新增启动前生成接口往返检查，避免新 Python 接旧 typesupport；`SDKExecutionState` 新增真实模式、模式已确认和切换中字段，原 `general_low_speed_confirmed` 含义不变。

## 地面与机身坐标

全局路线仍在原始定位 `map`，局部参考及支撑点在连续 `odom`。机身参考只增加一次高度：

`source_ground → + body_height(world Z) → map→odom → body_reference`

楼梯原始区域同时含墙、栏杆，不能整片标为地面。新增静态索引按原生 PCT 单元约定关联实测支撑高度，保留原始 XYZ；61452 个楼梯区域原始点中关联5950个表面候选。入口、出口还复用相邻楼层已经分类的原始地面点，修复先前遗漏。

每个局部窗口按三维距离关联，既有12 cm邻近阈值不变；这只证明附近存在原始表面返回，不证明落脚、净空或登梯能力。楼梯 SupportReference 的旧平地坡度/台阶能力字段为0；NativeScan 使用来源合同，历史平地验证器不能把它当作通行证明。

## 第一阶段验证结果与边界（修复前记录）

| 检查 | 结果 | 能证明什么 |
|---|---|---|
| 真实楼梯参考窗口 | 上楼734/734，下楼734/734 | 原始支撑关联与入口出口接续；最大距离9.16 cm |
| 非恒等 map/odom 回调 | 1468/1468，回投误差最大2.25e-14 m | 同时含平移、roll/pitch/yaw，源时间/版本不变，高度一次 |
| 支撑选择开销 | P95约0.9 ms，最大424点/包 | 静态窗口查询，不是SCAN整轮耗时 |
| 原生箱体静态姿态查询 | 上楼23/32、下楼22/32占据 | 原始PCD与当前箱体确有交集，不是未知体素额外否决 |
| 原生局部搜索/优化诊断 | 楼梯20窗口：14个目标占据，6个参考修复失败，均未进入成功优化 | 不只检查全局中心线；使用生产target selection、GridMap、A*与优化器 |
| 同组件平地正控制 | 16/16完成参考、Rebound与Refine；约3.1–6.7 ms | 排除诊断程序整体失效；不能替代动态导航 |
| 箱体障碍反例 | 两个清空姿态加20×20×30 cm障碍，状态0→1，唯一占据175 | 没有通过清点或伪造free取得通过 |
| 反向一致性 | 同姿态yaw+π共32组状态和计数一致 | 上下楼采样差异不是箱体前后不对称 |

这些原生实验使用原始静态 PCD，包括可能的历史物体/残影；**不能推断实时雷达局部地图必定失败，也不是闭环实机成功率。** 未把录包既定运动冒充导航执行。

## 第一阶段发现的问题

### 1. 楼梯上的机身包络与占据交集

默认长方体1.03×0.40×0.50 m，机身参考高0.50 m，下边界相对参考点−0.25 m。部分更高台阶进入长箱体底部；入口、平台还有贯穿多个高度的占据，不能统一视为地面或自身回波。

官方Go2 ROS2参数的高度、下沿与机器人尺度不同；论文采用“名义机身高度减可跨步高”解释下沿。D1的URDF躯干虽更薄，但不包含所有适用姿态下的腿轮及载荷扫掠，不能直接用躯干尺寸替代整个箱体。当前未发现确定的雷达/base双重高度补偿错误。

后续需核对阶高/踏步深度、楼梯净宽、机身参考及实际腹部/载荷下沿；针对确认过的姿态适配碰撞体，而非删除台阶或强制调绿。

### 2. SDK楼梯运动语义

官方 `sdk_client_api_cn.md` 将 Move 描述为通用模式，`sdk_type_cn.md` 又描述 STAIR 支持移动，存在矛盾。必须核对当前固件实际能力、速度比例、切换与零速保持。

已有 `monitor.yaml` 增加两个可选空参数：`execution_mode_capability_record`、`execution_mode_capability_sha256`。正式 launcher 已会加载这份配置，无第二入口。仅在有绑定机器、SDK/固件、profile与原始测量文件的记录时才能确认楼梯模式能力；默认没有记录。软件事务成功路径使用隔离 mock 验证，不冒充实机证据。原始 MC 时钟关系未验证时，实机停稳仍只能报告未确认。

## 复现

工作目录 `/home/dndx/d1max_nav_ws`，数据与诊断产物在 `experiments/stairs_chain_20261021/`。

```bash
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/native/install/local_setup.bash
source experiments/stairs_chain_20261021/interfaces/install/local_setup.bash
export PYTHONPATH="src/d1max_pct_planner:src/d1max_pct_scan:src/d1max_localization:$PYTHONPATH"
/usr/bin/python3 experiments/stairs_chain_20261021/check_stair_support_windows.py
/usr/bin/python3 experiments/stairs_chain_20261021/check_stair_callback_frames.py
/usr/bin/python3 experiments/stairs_chain_20261021/build_static_probe.py
/usr/bin/python3 experiments/stairs_chain_20261021/run_static_geometry.py
/usr/bin/python3 experiments/stairs_chain_20261021/check_geometry_contract.py
/usr/bin/python3 experiments/stairs_chain_20261021/build_solve_probe.py
/usr/bin/python3 experiments/stairs_chain_20261021/run_solve_probe.py
/usr/bin/python3 experiments/stairs_chain_20261021/run_floor_solve_probe.py
```

协议/参考回归使用 `test_segment_transition.py`、`test_stair_support.py`、`test_reference_support.py`、`test_continuous_reference*.py`、`test_interface_preflight.py`；编译回归分别在该目录 `tracker/build/d1max_trajectory_tracker`、`bt/build/d1max_navigation_bt`、`sdk/build/d1max_sdk_bridge` 执行 `ctest --output-on-failure`。这些测试不连接实机。

默认包的同步只替换明确清单中的源码、生成接口与对应二进制；备份位于 `deployment/before`，校验用 `sync_default.py --verify`。默认目录、入口、Zenoh、地图文件与物理验收状态保持不变。

## 第一阶段交付状态（历史基线）

- 已同步既有 `experiments/navigation_runtime`，两处既有默认选择器均指向新封存哈希 `d556ff877e68aa7ba4ae38be17fb2f7adafe1a8eda2ac2bf568d22368518b571`；原封存哈希为 `7abbf4aca62422195557ed11a4ae91274eeb019f2588373e2768136d3e3bfea9`。
- 1138项显式安装条目包括完整生成接口、对应BT/SDK/跟踪ELF与源码；没有只更新Python而留下旧消息布局。生产SCAN库及地图字节保持不变。
- 安装前源码回归465通过、3项因未加载完整发布环境跳过；安装后使用**默认包实际Python导入与生成接口**，同集468/468通过。SDK 21/21、BT 6/6、tracker 10/10 ctest通过。
- 相同输入前后：旧版本对楼梯支撑语义直接拒绝；新版本1468个真实窗口完成来源关联及坐标传输。原生碰撞行为未修改，不能把支撑关联通过写成楼梯求解通过。
- 交叉审查修复STAIR缓存比例污染下一次GENERAL任务的问题；映射与模式许可现都要求匹配当前真实模式。普通平地控制策略没有重写。
- 未启动/重启生产服务、SDK或实机，未提交或push。`physical_acceptance=false`及空的楼梯物理能力记录保持原状。

## 第二阶段：SCAN楼梯求解与D1碰撞模型适配

### 官方行为与适配边界

核对上游[ROS2分支](https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community)，固定提交`d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`，避免拿不同版本说明混用。官方Go2采用双圆柱水平近似；`planner.yaml:22–26`的半径0.25 m、中心偏移0.18 m属于Go2参数。`body_height=0.4`在FSM中用于地面参考转机身参考，不等于碰撞包络高度。

`grid_map.cpp:208–224,265–268`把原始占据向上0.1 m、向下0.4 m膨胀；反过来在机身中心查碰撞，相当于机身下面0.1 m、上面0.4 m，不能把上下沿颠倒。官方搜索使用高度引导，优化器抑制Z梯度，并非把台阶全部当自由空间。直接照搬Go2下沿或只用D1躯干CAD会遗漏轮腿/载荷。

本次保留D1既有完整长方体**1.03×0.40×0.50 m**，下沿/上沿各0.25 m。新增完整roll/pitch/yaw的原生OBB查询是D1适配，不声称官方已有此实现；不缩箱体、不删除台阶、不扩大自体清除盒、不修改未知空间策略。

### 已确认根因与生产修改

| 根因 | 修改 | 保持不变 |
|---|---|---|
| 历史平地`MotionSupport`拒绝合法楼梯来源合同 | 独立`PlanningSupport`识别原始楼梯支撑；平地仍委托原逻辑 | 不把楼梯来源当登梯能力证明 |
| 只携带中心线窄条支撑，侧向搜索缺少真实踏步上下文 | 从已分类原始点提取1.5 m上下文，按XYZ关联楼层 | 12 cm源点关联阈值、原始XYZ、非楼梯区域不变 |
| 水平长箱体的前下沿插入上级台阶 | 当前位置使用原始实测姿态；未来位置用原始踏步的局部稳健平面估计姿态 | 地面和机器人位置都不被旋转、压平或改高 |
| 搜索、求解、提交前重查使用的姿态不一致 | 各私有快照绑定同版本支撑；worker完成后owner再次绑定再接纳 | 无新增tracker/SDK碰撞层 |
| 后续支撑窗口可挤掉旧曲线支撑 | 每条候选/当前曲线固定自己的不可变支撑索引 | 普通窗口滚动不改旧曲线坐标含义 |
| 旧splice可能跨支撑段复用姿态解释 | 仅跨语义旧splice候选被拒；同源跨generation、普通平地不变 | 正式当前状态滚动规划不使用此splice，不受影响 |

地形估计只使用已分类表面点：0.6 m邻域、三次稳健最小二乘；当前实测姿态在0.5 m内平滑过渡到预测姿态。5 cm网格缓存每读者独立、最多4096项，不与地图写入线程共享。支撑不足回到原直立几何，不返回虚假的FREE。

OBB对体素采用15轴SAT；零roll/pitch保留原快速路径。启用楼梯姿态时角度采样半径包含高度方向（约0.606 m），平地保留约0.552 m。查询、原始证据计数和失败诊断使用同一个姿态；快照复制不继承旧provider。这里验证的是**密集离散完整OBB查询**，没有证明相邻样本间任意姿态的连续扫掠，也不是轮腿落脚规划。

### 同输入证据

| 对照 | 修复前 | 修复后 |
|---|---|---|
| 相同64个楼梯姿态、同一完整箱体 | yaw-only有22个占据 | RPY为0；原生唯一体素计数与独立SAT一致 |
| 6个真实双雷达短窗，原始占据不变 | 下段/上段起点分别77/80个占据 | 按实测姿态均为0；其它4窗前后均0 |
| 6个真实上楼位姿，从静止重新规划 | 旧水平箱体阻塞楼梯 | 目标选择→A*→Rebound→重定时→整曲线检查6/6；约24–191 ms |
| 同上楼姿态直接要求反向 | — | 5/6；陡梯上当场转180°的旋转碰撞仍拒绝 |
| 同一raw场景，假设已经朝下 | — | 6/6，但这是明确标注的反事实几何，不是真实下楼录包 |
| 原地图约23.15 m完整楼梯通道顺序规划 | 同尺寸yaw箱体中途停止 | 上下行均到达终点0.20 m容差内；状态由已接受曲线P/V/A推进，不沿全局线强行挪动 |
| 每个raw窗加入20×20×30 cm实体箱 | — | 6/6检出；原始占据哈希不变 |
| 上下方向、上下梯段新增箱体/侧栏 | — | 8/8整曲线由clear变blocked；屋顶障碍原生回归也拒绝 |

原始bag为`slam_raw_20260923_010248_eb79d8`。117帧短窗请求中109帧按雷达采集时间投影积分；8帧因历史位姿插值缺口排除，未使用bag接收时间配旧点云。姿态采用历史TUM及同窗PGO变换，机身参考使用既有名义关系，不冒充物理外参验收。真实raw求解从实测位置/姿态重新静止起步，初始v/a为0，不能当作历史动态速度闭环。

修复前静态PCD包含已经在既有可见性审计中剔除的残影：146个非支撑见证中129个来自已移除索引。顺序几何实验复用那份已有索引，仅移除6450条记录，其余1072311条XYZ逐字节不变；没有为本轮实验临时删障碍。保留原始失败结果，不把其误写成当前实时雷达的必然失败。

### 时序、回归与仍未完成项

- 1468个真实地图参考窗口、非恒等map/odom转换全部通过；最大回投误差2.84e-14 m，高度只补偿一次。上下文包最多2924点，选择P95约7.90 ms。
- 整段顺序几何实验允许新候选失败时使用仍通过碰撞查询的当前曲线。存在400 ms共享期限超时，P95约400 ms；这证明该输入上可贯通，不证明每轮实时成功，更不等于实机运动顺滑。精确次数和P50/P95/P99见`spatial_sequential_summary.json`。
- 回归包括完整包络、缺失/弱/未观测证据不混淆、私有快照、支撑版本固定、平地恢复、取消迟到不复活、owner接纳时绑定姿态。接线harness直接编译生产方法，但不把passive seam测试说成完整ROS运行；原生求解与接线结果分别记录。
- 最终隔离构建：`plan_env` 7/7、`scan_planner`非launch的35/35 CTest通过；其中完整长方体38项、原生follower 84项、姿态scope 6项、owner接线36项、worker接纳59项；Python支撑回归24/24通过。两项旧launch测试的零断言结果明确不计入。
- SDK登梯能力、轮腿/载荷实测包络、外参、当前固件模式切换与制动保持仍需现场验收；保持`physical_acceptance=false`，没有通过置真标志解锁。当前只更新软件及默认包，不启动实机。

### 第二阶段复现与默认包

工作目录仍为`/home/dndx/d1max_nav_ws`。所有新证据在`experiments/stairs_scan_adaptation_20261021/`，原失败对照未覆盖。原生实验固定实际链接库哈希，命令与输入来源见`raw_stair_solver.summary.json`及顺序实验构建记录。

```bash
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/native/install/local_setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
ctest --test-dir experiments/stairs_scan_adaptation_20261021/native/build/plan_env --output-on-failure
ctest --test-dir experiments/stairs_scan_adaptation_20261021/native/build/scan_planner -LE launch_test --output-on-failure -j2
PYTHONPATH="src/d1max_pct_scan:src/d1max_pct_planner:src/d1max_localization:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q src/d1max_pct_scan/test/test_stair_support.py src/d1max_pct_scan/test/test_reference_support.py
/usr/bin/python3 experiments/stairs_scan_adaptation_20261021/sync_default.py --verify
```

同步仍覆盖唯一`experiments/navigation_runtime`，无第二入口、无新分支。显式清单同时更新Python源码、原生头文件、`plan_env → path_searching → bspline_opt → scan_planner`完整ABI依赖及ELF，不单独替换一处脚本。默认地图、Zenoh、接口、SDK、控制器参数和物理验收状态保持不变；同步的文件哈希、两处选择器及可回退备份由该目录`deployment/`记录。最终同步结果以`--verify`和现有`deploy/single_floor_release.json`为准，不把实验产物冒充已部署。

## 第三阶段：全程固定爬楼梯模式

本节记录上一轮状态；下文第四阶段根据用户明确授权接通通用运行参数复用，已取代本节“空能力记录阻断运动”的行为。

用户决定不在楼梯入口、平台或出口切换运动模式。唯一正式导航入口现在使用`execution_mode_policy: fixed_stair`：

| 层级 | 固定STAIR下的行为 |
|---|---|
| 会话配置 | 同一策略写入session、reference和模拟SDK；正式monitor使用相同策略 |
| 地形与SCAN | 走廊仍是平地、楼梯仍是楼梯；`required_mode`保留既有几何含义，不更换支撑点来源或碰撞算法 |
| 参考分段 | 实际XYZ达到相邻段入口后准备下一段；不发`ModeTransitionRequest`，不等模式ACK，不因换模式要求停稳 |
| 行为树 | 首次接管接受真实确认的STAIR状态，不要求把`general_low_speed_confirmed`伪置真 |
| SDK唯一writer | 启动前由操作者选好STAIR；核对真实STAIR状态，不自动起身，不发送SetMode，不切回GENERAL |
| 指令映射 | 固定使用STAIR自身的映射；平地、上下梯及平台标签不重置或切换SDK映射 |

新旧策略只用于兼容既有封存会话，没有增加启动入口或导航任务管理者。正式默认固定STAIR；旧会话缺少新字段时保留历史`segment_modes`语义，不静默重解释。急停、取消、失权及原始命令期限未改变；实机没有在本轮被连接、接管或启动。

### 已验证

- Python会话配置、上下楼分段、平地支撑不被改写、候选拒绝重试与迟到反馈：409/409。
- BT：6/6 CTest；任务授权37项通过。新增STAIR首次接管用例在旧条件下失败，修复后通过。
- SDK：Release编译、21/21 CTest通过；固定模式不产生SetMode，错误GENERAL不被当STAIR，急停/失权语义和历史分段策略保持。
- 生产SCAN、规划地图、控制器、接口和Zenoh保持第二阶段字节不变；仅更新固定模式所需Python配置、BT与SDK产物。

### 仍需区分的实际限制

**固定模式接线完成，不代表STAIR实机Move已获得完整依据。** 默认的`execution_mode_capability_record`及其哈希仍为空，因此现有SDK发送层仍不放行STAIR非零运动，不能把本次修改宣传为实机立即可走。本轮没有新增模式切换验收要求；固定STAIR只读取STAIR自身的Move、零速保持和速度映射记录，不再要求GENERAL或mode_switch记录。记录缺失时保留实际模式已确认与运动能力未确认两个事实，不冒用GENERAL比例、不修改物理验收标志。

第三阶段同步证据、文件备份及复现位于`experiments/fixed_stair_20261021/`：

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/native/install/local_setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
source experiments/navigation_runtime/application/install/local_setup.bash
ctest --test-dir experiments/stairs_chain_20261021/bt/build/d1max_navigation_bt --output-on-failure
ctest --test-dir experiments/fixed_stair_20261021/sdk/build/d1max_sdk_bridge --output-on-failure
/usr/bin/python3 experiments/fixed_stair_20261021/sync_default.py --verify
```

旧阶段的同步器保留作历史对照，后续默认包验证以本阶段同步器为准。更新不会修改正在运行的旧会话；本轮不重启服务、不恢复旧目标、不发送运动命令。

## 第四阶段：固定STAIR实际Move链路与通用速度映射复用

用户明确要求全程使用爬楼梯模式，速度比例沿用已使用的通用模式。当前正式配置为：

```yaml
execution_mode_policy: fixed_stair
execution_stair_runtime_parameter_source: general_runtime_reuse
```

### SDK依据与原阻断原因

同一厂商SDK包的`docs/zh/sdk_type_cn.md`将`SPORT_MODE_STAIR=3`定义为支持移动，`MOTION_STATUS_STAIR=7`表示登阶运动状态；`example/control.cpp`选择`SetMode(3)`后仍通过同一个`Move(left_right, forward_back, yaw)`移动。`sdk_state_cn.md`说明切模式会自动起身，因此固定策略不自动调用`SetMode`或姿态接口。

`sdk_client_api_cn.md`的Move说明只提“通用模式”，与类型说明不一致；速度表也存在版本内差异。因此此处不宣称STAIR速度经过物理标定，而是明确执行用户选择的通用运行参数复用。

此前发送层把STAIR的独立测量记录与运行参数就绪绑定。即使真实模式是STAIR且已获得控制权，空记录仍会阻止非零指令；此外单独线性STAIR比例分支不能完整继承现有通用幅度映射。现在将运行参数选择、真实模式确认、物理测量证据分开表达，不伪造验收记录。

### 正式发送链

1. 跟踪器输出原有连续前进与转向需求；不新增横移、不改控制频率和控制策略。
2. 唯一SDK writer读取真实STAIR状态、当前任务和最新有效需求。
3. `general_runtime_reuse`使用与原通用模式相同的完整映射函数，包括轴比例、连续幅度响应和既有零指令处理；不是另建STAIR控制器。
4. 同一20 Hz发送链调用`Move(0, forward_fraction, yaw_fraction, 0, callback)`；平地、楼梯和平台不改变映射或切模式。
5. 普通停止仍发送`Move(0,0,0)`，保留发送结果和实际停稳反馈。取消、急停、失权、过期命令及发送失败不允许旧非零指令继续发送。

空`execution_mode_capability_record`不再阻断这一明确选用的运行参数路径；`physical_acceptance_verified`仍为false。比例相同不代表实际速度必然相同，SDK写回调也不等于机器人已运动或停稳。

### 默认入口与版本边界

Web使用的真实参数链是`managed monitor → start_sdk_monitor.sh → sdk_arguments(False) → 默认包sdk/install内的monitor.yaml`。只改工作区或source snapshot不够；本轮同步安装配置、封存源码及SDK ELF，并核对CLI不会覆盖固定模式或比例来源。仍只有原导航入口、原SDK会话和原writer。

### 软件验证结果

- 相同STAIR反馈、相同空能力记录：旧`evidence_record`路径拒绝grant；新显式运行参数复用路径完成一次接管，生产Transport与共享mapper连续输出60/60帧非零写入意图。
- 地面、楼梯、平台、二楼连续换段，SDK幅度与原GENERAL映射逐帧相等；精确零命令没有被最小响应幅度抬高。
- 新策略覆盖取消、后续MC停稳窗口、失权、急停、发送失败、命令/任务租约过期；未复用旧非零速度，未把发送成功当实测停稳。
- SDK Release全量构建及21/21 CTest通过；Python配置与分段回归410项通过；真实Web启动参数构造76项通过。SDK测试Release仍使用`-UNDEBUG`，断言实际执行。
- Web启动测试首次从导航仓库运行有1项因`manager`导入路径缺失失败；回到其所属`foxglove_d1max`工作目录后76项通过，未修改生产逻辑掩盖环境问题。
- 未改SCAN、地图、跟踪控制器、接口、时序参数或Zenoh；比例复用通过原SDK映射函数，不增加另一套平滑器或发送线程。

验证和同步记录位于`experiments/fixed_stair_runtime_20261021/`。未启动服务、未连接SDK、未发送实机运动指令；未把软件回归记作实机上下楼验收。前面阶段同步器为历史记录，当前验证使用：

```bash
cd /home/dndx/d1max_nav_ws
/usr/bin/python3 experiments/fixed_stair_runtime_20261021/sync_default.py --verify
ctest --test-dir experiments/fixed_stair_runtime_20261021/sdk/build/d1max_sdk_bridge --output-on-failure
```

## 第五阶段：楼梯连续参考与实时性整改（2026-10-22）

本轮限定于入口、平台、出口的连续衔接和原生 SCAN 求解；保留既有平地控制、SDK 幅度映射、20 Hz 发送及单一入口。不启动导航服务、不重连 SDK、不发送运动。冻结的默认清单 SHA256 为 `2f92aa1cf8ee3cf591344ed2b83fce2e53fef124d76f784936e2e9740412d71a`，工作区已有修改全部保留。

### 确定根因与修改范围

| 原行为 | 本轮修改 | 不变项 |
|---|---|---|
| 参考窗口在语义段末截断，SCAN 把入口/平台当作有限尾端；接近段尾才准备下一段 | 固定 STAIR 下，正式 4 m 窗口提前包含相邻楼梯/平台/出口。逐点保留原始段语义，实际进度仍属于当前段 | 不延伸全局路线、不跳楼层、不把未来几何当实测进度；纯平地窗口不改 |
| 候选耗时跨过入口后，真实经过入口的事实丢失；重锚提交也可能覆盖该事实 | 记录真实入口事件，使用新鲜当前样本继续相邻段投影；候选保存自己的坐标锚与观测状态 | 不刷新旧时间戳、不用 XY 最近点跳段、不改正常模式切换策略 |
| 含相邻地形的窗口被当成新任务/硬替换，重复清理规划状态 | 原生滚动参考按同任务、同锚、有序 XYZ 重叠和相邻标签分类；待接纳参考与执行参考独立 | 新目标、取消、坐标身份改变仍退役旧工作；平地原分类不变 |
| 跨段候选失败后，下一窗口丢失与当前曲线的衔接身份，反复等待 | 保存已分类前驱及其与当前曲线的关系；后续候选可继续推进，实际提交仍原子替换 | 不把 pending 当 active，不接受旧任务迟到结果，不添加碰撞层 |
| 楼上目标与当前 XY 接近，跟踪器提前套用终点减速 | 非最终曲线仅在同目标高度时使用终点 XY 减速 | 有限局部曲线的原停止尾端、平地速度/加速度/jerk 参数不变 |
| 异步 worker 快照只携带姿态查询，新高度引导未进入正式求解线程 | 同一私有快照在 prepare 和 worker 求解期间持有完整楼梯查询作用域；结束时先恢复查询器再归还快照 | 不共享可变查询缓存，不延长源时间，不把实验成功冒充正式接线成功 |

### 混合参考的源点合同

`SupportReference` 增加 `SupportRegion[] regions`，每段明确原始点数组的半开索引范围及楼层/地形归属；包头仍表示当前实测段。旧单段包的 `regions` 为空，完全保留原处理分支。

参考管理器从已核对的原始点按对应分类取源，随后仅应用该参考绑定的刚体 `map→odom`。不使用压平后的 Z 冒充原始支撑，不造平面、不删台阶、不发布非刚性 TF；地面到机身高度只加一次。SCAN 使用生产 `PlanningSupport` 和原生 `GridMap`，跟踪器仅解码一致的来源合同，不新增运动碰撞检查。

跨段提前看几何与实测切段是两件事：前者让 SCAN 提前规划，后者确认机器人实际走到了哪段。取消或旧参考退役后，迟到候选不能恢复旧任务。

### 已完成的软件证据

- 参考链组合回归 354 项通过，覆盖入口延迟、候选失败、重锚、撤回、乱序和迟到提交。
- 会话参数、类型预检和支撑合同专项 62 项通过；接口同时更新 Python/C++ typesupport，新增混合支撑类型不与旧安装产物混用。
- 使用实际原地图、上/下行路线、入口/平台/出口各前 1 m、前 0.3 m及精确边界，共 18 个正式 4 m 窗口通过。逐区域原始点回投误差最大约 `2.84e-14 m`，无跨楼层点归属错误，源时间、路线哈希及候选身份保持。
- 上述 18 窗使用沿真实路线生成的合成位姿，只验证几何与生产参考接线；不是实机运动，不以此声称上下楼成功。

原始失败、同输入性能对照、构建和类型证据在 `experiments/stairs_continuity_20261022/`。本节后续结果以同目录部署验证及最终原生测试记录为准。

### 原生查询和搜索优化的分层对照

首先优化完整 RPY 长方体查询的环形数组访问，保持相同体素、SAT、未知/占据语义及计数；单独应用该等价优化时，上下行顺序几何的接受序列保持。其次只在楼梯支撑已关联时，以现有稳健地形拟合指导 A* 内部点高度：原始起终点不移动，缺支撑时回到原搜索平面，最终仍查询完整原生包络。纯平地不安装该 provider；新快照不继承旧支撑，嵌套 scope 退出恢复原绑定。

下表是环形查询与支撑搜索修改的同输入对照，先于下文数值修复；不能与动态参考实验混算。

| 同输入对照 | 修改前 | 查询与搜索修改后 |
|---|---:|---:|
| 顺序上楼求解中位数 | 38.1 ms | 15.5 ms |
| 顺序下楼求解中位数 | 41.9 ms | 16.5 ms |
| 上楼尝试数／共享期限超时数 | 207／16 | 201／14 |
| 下楼尝试数／共享期限超时数 | 218／14 | 205／14 |
| 两向完整通道顺序几何 | 均完成 | 均完成 |
| 真实双雷达短窗＋同姿态假想反向，中位数 | 85.8 ms | 34.8 ms |
| 上述短窗 P95 | 173.8 ms | 67.9 ms |
| 上述短窗求解通过数 | 11/12 | 11/12 |

顺序实验由已接受曲线的 P/V/A 推进，并非直接将状态移到全局参考线上；仍只是静态几何实验，不是物理运动闭环。真实录制上楼短窗为 6/6，另外六个同姿态反向请求为假想输入、5/6；保留同一个碰撞失败。整段顺序 P95 仍约 400 ms，说明极端慢求解没有全部消除，不能把中位数改善宣传为硬实时保证。尝试次数不同是几何推进后的结果，不是删除失败样本。

曾试过逐踏步最近原点高度引导，但下楼未完成，已弃用；失败结果保存在 `after_source_height/`，未进入默认源码。最终使用均值地形引导，不更改原始碰撞地图。

该阶段原生非 launch 回归共 53 个 CTest 通过（7＋5＋8＋33），包括完整包络、同 XY 不同楼层、缺支撑回退、嵌套作用域和快照隔离。跟踪器 10/10 CTest 通过，包括 floor→stair→floor2 换轨保留速度限制器、速度/加速度/jerk 连续及零横移。加入箱体/栏杆的 8/8 危险反例仍被原生碰撞拒绝。最终数值修复后的构建与动态结果见下文。

另对 18 个混合窗口做了独立冷启动求解：生产原生混合合同 18/18 有效，但只有 8/18 得到完整有效曲线。该实验统一采用合成地图朝向、零起终点速度，部分朝向与路径相差超过 160°，且每窗重新装载静态占据；其余为几何/优化失败，约 6～12 ms，不是超时。保留全部失败记录，不据此声称 18/18 导航成功，也不把它等同正式连续行走状态。

### 正式异步线程与上楼入口数值失败

独立审查发现，异步求解快照原先只携带姿态查询，没有带上新增的支撑高度查询。现在同一 `StairAttitudeScope` 从 worker 准备贯穿求解，结束时先恢复两个查询器，再归还快照。生产回调抽取测试在修改前出现 6 项失败；修改后原有 59 项和新增 27 项通过，覆盖正常、失败、取消、准备异常和求解异常。该测试只证明实际代码接线与释放次序，不代替几何求解。

正式动态上楼实验还复现出入口静止起步时连续 5 次 `LBFGS -1008`。参考存在、起点未占据、耗时约 9 ms；零末速度符合生产窗口规则，不能改成 0.3 m/s 来迁就旧夹具。失败残留曲线虽然无碰撞，但最高速度/加速度约 0.374 m/s、0.919 m/s²，不能直接当作成功。

追到标量线搜索后，确认 vendored `CUBIC_MINIMIZER2` 用被覆盖为 `theta/s` 的 `a` 判断区间方向，正确条件应使用步差 `d>0`。实测数据中旧实现将下一步选为 0，落到更新后的区间外；恢复上游条件后下一步为 0.0009872188，位于区间内。镜像负方向同样复现，普通二次函数结果不变。[上游固定版本](https://github.com/chokkan/liblbfgs/blob/5ad02fbefefdeff339ab03635e673571055a0644/lib/lbfgs.c#L1085-L1105)

修正仅由有效楼梯支撑作用域启用，平地保留现有分支；没有增加公开参数、运动许可或新的碰撞层，也没有接受失败返回码。原始起终状态、零末速度、固定 Z、代价权重和共享 400 ms 期限保持，仍走原生整曲线检查及重定时。尝试过的记忆重启无效，保留在实验目录，未进入生产。

标量回归 `test_lbfgs_cubic_bound` 的 7 项通过；同一上楼初始状态原先反复失败，修正后首次求解成功，随后 4 次按接受曲线推进的求解也通过重定时和完整曲线检查。只有第一次是严格相同状态的前后对照，不能把随后已经前进的状态称为同输入。解析梯度额外核对 36 组通过；不将其扩展声称动态碰撞项已获数学证明。

### 圆滑入口的实际切段修复

继续使用正式参考回调后，发现第二个独立问题：上楼曲线绕过入口中心，按 5 ms 间隔采样的最近距离为 12.928 cm，原切段要求进入 12 cm 球，因此始终没有记录跨段事件。机器人几何状态已沿有效曲线进入楼梯，参考仍停留在旧地面段，最终第 40 轮投影失败。不是提高采样频率就能解决，也不能靠修改 ACK 或源时间掩盖。

固定 STAIR 的相邻楼梯边界现在按真实连续 XYZ 样本穿越局部几何截面交接，并核对下一段原始前缀的有界投影。仍使用原有 0.3 m 对应范围，不扩大入口球；不搜索任意楼层或分支，不把跨段预览当作已走过。反向退回撤销未提交的穿越事件。纯平地和历史模式事务逻辑不变；没有新增运动检查。

原 41 个样本和 ACK 顺序原样回放：第 21 轮识别穿越，第 24 轮提交楼梯段，原第 40 轮失败位置正常推进。参考链最终 362/362 通过，新增错层、旁支、旧源时间、身份变化和退回反例；独立复核其中 96 项通过。

### 最终双向连续几何结果

`final_portal/dynamic_up.json` 与 `dynamic_down.json` 使用正式 `ContinuousReference`、正式支撑构建和生产原生 SCAN/GridMap。每轮机器人位置、速度和加速度来自上一条接受曲线；候选提交后才更新参考归属，不把状态直接挪到全局路线上。

| 指标 | 上楼 | 下楼 |
|---|---:|---:|
| 连续推进轮数 | 199 | 198 |
| 新曲线接受 | 167 | 175 |
| 候选失败后沿仍可用旧曲线推进 | 32 | 23 |
| 因参考切段导致中断 | 0 | 0 |
| 求解中位数 | 约 53 ms | 约 53 ms |
| 求解 P95 | 约 400 ms | 约 107 ms |
| 触及原共享期限 | 13 | 6 |

上行按 `lower_floor → stair_lower → stair_upper → upper_floor`、下行反向通过，覆盖入口、两梯段之间的实际平台连接及出口。当前原路线把平台连接几何放在两个楼梯段之间，没有单独的 `landing` 标签；本轮没有虚构该标签。加入箱体和栏杆后的 8/8 反例仍被生产整曲线碰撞拒绝。

最终原生非 launch CTest **54/54**（7＋5＋9＋33），跟踪器同闭包重链后 **10/10**；会话、类型、支撑专项 **62/62**。跟踪器保留既有平地控制参数、速度限制器历史和零横移；SDK、地图及物理验收标志不变。

上述动态结果是静态原地图上的连续规划几何实验，不是完整 ROS 行为树/SDK/物理动力学闭环，也不等于实机到达并停稳。397 轮均推进不代表实机速度不会波动。上楼仍有 400 ms 长尾，保留超时和旧曲线使用记录，不以延长期限或提高标称频率宣称问题全部消失。真实双雷达短窗最终仍为上行 6/6、同姿态假想反向 5/6，原碰撞失败保留；实机上下楼和动态障碍效果仍需现场确认。

最终统一汇总为 `experiments/stairs_continuity_20261022/final_portal_report.json`，分位数按 `(n−1)×q` 线性插值。覆盖约 23.153 m 楼梯几何范围；每 0.5 s 的合成状态推进是实验步长，不是生产规划频率。较早的 `final_native_report.json` 保留入口修复前的失败，不能拿它当最终结果。

### 同步与复现边界

同步仍覆盖唯一 `experiments/navigation_runtime`。显式清单同时包括参考 Python、新消息完整生成物、`plan_env → path_searching → bspline_opt → scan_planner`、跟踪器和对应源码；不只替换某个 Python 或 ELF。同步器检查当前包未运行，备份精确目标，重新封存并更新两处既有选择器。物理验收状态保持不变。

安装复核还发现 Humble 在 Python 包目录复制了一份带构建路径 RUNPATH 的消息生成库。暂存时改用同构建、同 ELF identity 的正规无 RUNPATH 库，并逐个检查接口 ELF，避免默认包隐式依赖实验构建目录。部署计划、精确备份和安装后验证保存在 `experiments/stairs_continuity_20261022/deployment/`；安装是否完成以 `sync_default.py --verify` 的实际结果为准。

主机当日自动更新了 `liblzma5`（`5.2.5-2ubuntu1.1 → 5.2.5-2ubuntu1.2`），唯一外部文件差异已由 dpkg 所属、包版本和完整性核对；同步脚本只接受这一个固定旧/新哈希对，不跳过其它文件检查。

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/native/install/local_setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
source experiments/navigation_runtime/application/install/local_setup.bash
/usr/bin/python3 experiments/stairs_continuity_20261022/sync_default.py --verify
/usr/bin/python3 experiments/stairs_continuity_20261022/summarize_final_portal.py
ctest --test-dir experiments/stairs_continuity_20261022/native/build/scan_planner -LE launch_test --output-on-failure -j2
ctest --test-dir experiments/stairs_continuity_20261022/tracker/build/d1max_trajectory_tracker --output-on-failure -j2
PYTHONPATH="src/d1max_pct_scan:src/d1max_pct_planner:src/d1max_localization:$PYTHONPATH" \
  /usr/bin/python3 experiments/stairs_continuity_20261022/check_real_transition_windows.py
```

不以软件通过替代固件 STAIR 行走、实际上下楼、平台转弯及停止保持的现场验收；这些仍须在真实机器人上分开确认。
