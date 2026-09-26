# PCT 完整离线流程与 RViz 选点修正

日期：2026-09-22。范围：现有 SC-PGO PCD 的全局路径预览；不启动机器人、定位、SCAN 执行器、SDK 或速度发布。

## 1. 实际处理链

原始 PCD → 可选保守预处理 → tomography 地面/上方观测 → 坡度、台阶、净空代价 → 官方代价膨胀 → 保留有效层 → 原生 PCT A* → quintic GPMP → 完整曲线检查 → RViz Path。

依据 [PCT Planner 上游](https://github.com/byangw/PCT_planner)，固定源码版本 `35cd73fd82bcd51bc538429294af7646b2a09815`。本机无可用 CUDA/CuPy，所以 tomography 是 NumPy/SciPy CPU 移植；通过独立标量核测试核对算法，不声称已在 GPU 上逐位对比。搜索与优化仍调用上游原生 C++ 模块，没有换成 Web A*。

本轮输入：

`maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo/optimized_map.pcd`

源文件 SHA256：`67475e7325b2b0b0686a08e7eacd0e56c3eed42b107d178018f37af514bfd0c2`。

输出目录：`maps/processed/sc_pgo_20260919_pct_official_20260922/`。包含安全加载格式 `tomogram.npz`、兼容官方格式的可信本地 `tomogram.pickle`、可视预览 `traversable.pcd`、配置快照和 `manifest.json`。使用了 1,127,596 个有限点，41 个初始切面简化为 9 个保留层；层编号不是建筑楼层编号。多个切面会重复表示同一地面，不能把层栅格计数相加称为面积。

## 2. 一个配置可复现地图

[构建配置](/home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/official_single_floor.yaml) 定义 PCD、输出、预处理、栅格分辨率、切面间距、坡度/台阶/净空、膨胀、资源上限和导出。

```bash
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
source /home/dndx/d1max_nav_ws/install/setup.bash
ros2 run d1max_pct_planner pct_build_official_map \
  --config /home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/official_single_floor.yaml
```

默认只移除非有限点，不启用 ROI、离群点删除或体素均值降采样；这些选项已经模块化，可显式配置。不会自动补洞、把缺失区域判为自由、压平地面或改写原始 PCD。输出独占创建，重建会分配新后缀，不覆盖旧版本。预览配置需要明确指向新输出，不自动切换地图。

当前 resolution=0.20 m，slice_dh=0.50 m，最小净高=0.55 m，step_max=0.17 m；这是离线测试配置，不是 D1 机械能力认证。地图自身已包含官方膨胀，运行时不再追加旧适配器的 clearance/erosion。

上方无回波保留 NaN；默认 `allow_unobserved` 沿用上游未观测顶棚的可候选语义，但不把它标为已证实净空。需要保守筛选时在预览中设置 `unknown_ceiling_policy: reject`。本轮通过的短路线沿途都有顶棚观测。

## 3. 为什么之前很容易 INVALID

| 原因 | 当前处理 |
| --- | --- |
| 看到的是原始散点，判定却来自旧地面栅格 | 直接显示规划器使用的真实 tomography；原始 PCD 降为暗色背景 |
| 地图边距之外又加一圈腐蚀，部分通道被重复缩窄 | 仅使用 PCT 生成的代价及统一阈值，不追加旧 mask |
| 视平面拖动无意改变 Z | 默认贴地 XY 操作，Z 由同一表面派生；自由 XYZ 保留供精调 |
| 所有失败混成 INVALID | 区分没有地面、PCT 高代价、净空不足、离地、层切换、路径越界、运行库故障 |
| 选点、显示和优化使用的层可能不同 | 同一 TomogramMap 判定，端点/路径携带真实保留层 ID，后台核对地图哈希 |

默认拖中心球就是移动地图 XY，红/绿轴精调。切换自由 XYZ 才出现 Z 轴及完整旋转环；自由模式高度不合法时坐标仍保留，明确点击“贴回表面”才纠正。模式/编辑层切换不搬动已有点。旋转环是编辑预览，不是终点姿态约束。

绿到黄方格表示合法代价，红格表示阻塞；同一物理表面跨层重复显示时去重，避免一处同时红绿闪烁。该去重仅作用显示，不改规划层和 mask。无支撑空洞不会被涂绿，也不会把 XY 自动挪到附近通道。

## 4. 已修复的运行库冲突

PCT 原生模块按 bundled GTSAM **4.1.1** 编译，但 ROS 环境的 `LD_LIBRARY_PATH` 使同 SONAME 的 GTSAM **4.2.0** 抢先加载。真实对比显示：错误环境报告零次迭代/零误差；正确库环境实际执行优化。

`native_runtime.py` 读取编译头、CMake link/flags、安装库与构建库哈希。仅给 PCT 后台及其 worker 在启动 Python **之前**配置独立库路径；导入前后检查 `/proc/self/maps`，不接受版本不符、已替换或残留临时实验库。不会修改用户登录环境、RViz、SDK 或 ROS/Zenoh 配置。直接在已启动 Python 内改 `LD_LIBRARY_PATH` 不算修复。

## 5. 验证结果与尚未解决的边界

184 项 Python 回归通过；33 项 RViz Qt/pluginlib 测试通过；22 项私有 ROS 接口验收通过。Python 测试在正确 bundled 运行库的独立解释器中执行。完整私有 ROS [验收记录](/home/dndx/d1max_nav_ws/log/pct_preview/official_20260922_validation/acceptance.json) 和 [进程清理记录](/home/dndx/d1max_nav_ws/log/pct_preview/official_20260922_validation/runner_report.json) 已保存；不把后台协议验收冒充人工鼠标手感或实机导航验收。

实图短路线：从 `[-4.1000015, 6.9, -0.6173909]` 到 `[-6.5, 12.1, -0.6326333]`，保留层 2，正确 ABI 下原生优化 10 次迭代，长度约 **6.57 m**，通过 quintic 连续曲线及精确端点连接段检查，39 个检查格，最小已测净高约 **2.52 m**。不再沿用错误 ABI 下的 6.65 m 数值。

**已知剩余问题：长路线的原生 GPMP 仍可能切进高代价格。** 大约 18 m 的层 2→1 测试不通过严格曲线检查，当前拒绝发布成功路径。诊断还发现上游 `GetValueBilinearSafe` 的索引/权重可能外推；仅在 `/tmp` 的对照修正改善优化，但仍有越界，未将实验库装入生产。不能把上游软障碍代价理解为硬碰撞保证。

因此本轮没有降低地图阈值、强行压回可行区、用 raw A* 折线冒充平滑路径，也没有宣称任意长路线均通过。后续应在独立原生规划模块中校正插值、暴露障碍项/节点密度参数并做连续约束验证；不能靠随意清理 PCD 或扩大选点容差来掩盖曲线问题。

补充有界诊断：仅在 `/tmp/pct-gpmp-engineering.GQDCCG/` 测试了三组独立原生修正，发现优化 factor 的可变层上下文及最终层标签也会影响结果。最后一组报告原生离散点全部合法，但连续曲线仍拒绝；拒绝位置存在同高相邻层，因此还需要核对沿曲线真正的 gateway 切换时刻。只固定在相邻输出点的中点切层可能保守拒绝，不能把每次拒绝都断言为物理撞墙。实验只保存摘要及源码差异，没有保存完整轨迹，不能视作长路线验收通过；这些补丁均未迁入生产。

## 6. 启停与工程边界

```bash
/home/dndx/d1max_nav_ws/start_pct_preview.sh start
/home/dndx/d1max_nav_ws/start_pct_preview.sh status
/home/dndx/d1max_nav_ws/start_pct_preview.sh stop
```

后台：`preview_session` → 私有 loopback Zenoh router + PCT server/worker + RViz；只清理由本次会话拥有的进程，重复启动拒绝，关闭 RViz 后收尾。中间件始终是 `rmw_zenoh_cpp`。配置/日志/成功路径保存在独立会话目录。

本轮已启动会话 `log/pct_preview/20260922_160917_b866fbaae932/`。只读检查确认实际窗口订阅了真实通行图、阻塞图和交互标记，RViz 初始化服务完成；后台 ready、GTSAM 4.1.1、两端点可通行、运动禁用。原始背景发布 563,798 点，所选层 2 发布 4,696 个可通行格和 17,692 个已测阻塞格。用户开始操作后未清空或修改其坐标。此检查不等于原生桌面截图/人工拖动手感验收。

当前完成的是**离线完整地图转换与全局规划预览**。既有在线 PCT→SCAN/SDK 链路仍保留，未将此次离线配置静默推入实机执行。未连接机器人，未发送控制指令。
