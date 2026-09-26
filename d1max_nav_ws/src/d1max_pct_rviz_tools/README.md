# RViz 三维空间位置与 PCT 规划面板

PlanningPanel 与三维编辑工具提供全局路径预览，不发布导航 action、Pose 或速度，不连接 SDK。显式导航执行使用下文独立的 MotionControlPanel；预览配置不加载该面板。

- 工具 `d1max_pct_rviz_tools/Start3D`（3D Start，B）一按即发布 Empty 到 `/d1max/pct_preview/activate_start`。
- 工具 `d1max_pct_rviz_tools/Goal3D`（3D Goal，N）一按即发布 Empty 到 `/d1max/pct_preview/activate_goal`。
- 这是独立地图空间位置，不依赖雷达点或鼠标命中物体。服务端保留已存在的位置，否则创建默认空间标记。收到新状态确认后自动聚焦 8 m，并异步返回默认 Interact，直接拖动 XYZ 箭头。
- 工具配置默认 Interact。若服务未响应，3 秒超时并返回交互；不发布任何机器人命令。
- 面板插件 `d1max_pct_rviz_tools/PlanningPanel` 提供 XYZ 三位小数编辑（默认 0.01 m 步长，范围 ±10000 m）、应用、恢复、显式贴地、聚焦、规划、清空。聚焦 Orbit 视角距离为 8 m。
- 面板“放置起点/终点”与工具使用同一接口；没有标记时也能输入 XYZ 后点“应用坐标”直接创建。坐标不存在不再禁用编辑框。
- 精确 XYZ 编辑默认折叠，点击“精确坐标”展开；输入仍有三位小数、步长选择和草稿保护。
- “总览起终点与路径”同时取起终点及已规划 Path 包围盒，按相机视角和屏幕宽高比留边显示，不只盯住局部手柄。它只改变视图，不重新规划或修改点位。
- 姿态以 R/P/Y 简要显示，明确仅编辑器预览。Empty `reset_start_orientation` / `reset_goal_orientation` 仅归零手柄姿态，不修改 XYZ 或已规划路径；当前 PCT 不约束终点 yaw。
- 面板仅以 `/d1max/pct_preview/status` 的 canonical start_xyz / goal_xyz 更新选择。它不依赖可能过期的 selected_* latched 点，因此 Clear 后不会被旧点复活。status 需 reliable/transient_local、5 Hz；2 秒无更新禁用操作。
- status mode 必须 GLOBAL_PATH_PREVIEW_ONLY，含 frame_id/ready/state/reason/start_xyz/goal_xyz。
- 正在输入或尚未应用的坐标不被 5 Hz 状态覆盖。Apply 等待对应坐标回执，超时 2 秒回到服务端权威值；编辑时禁止规划和贴地，避免误用未应用数值。
- ROS 回调只写共享收件箱，Qt 定时器在 GUI 线程更新界面；不额外创建节点或 executor。

面板发布 Empty 到 `/d1max/pct_preview/plan`、`clear`、`snap_start`、`snap_goal`。贴地动作明确点击才发生。

Humble 的 InteractiveMarkers Display 必须使用 `Interactive Markers Namespace: /pct_preview_points`，而不是无效的 Update Topic 配置。回归测试实例化真实 Humble Display 并加载该配置验证绑定。

限制：独立空间位置可移到空中、墙内或地图之外。服务端须独立验证坐标、地面高度和可达性；只有明确“贴地”才调整 Z。XYZ 三位小数是输入精度，不代表点云几何有毫米测量精度。

## 手柄鼠标操作

Humble 原生 MOVE_ROTATE_3D：中心普通左拖在视平面平移；Shift + 左拖沿视线前后移动，**不是地图 Z**；Ctrl + 左拖绕相机水平/竖直轴转动；Ctrl + Shift + 左拖绕视线转动。地图轴精确操作直接拖 XYZ 箭头或对应旋转环。面板常显“中心拖移 · 箭头微调 · 圆环旋转”，完整快捷键放悬停提示。

依据 [Humble InteractiveMarkerControl 实现](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/interactive_markers/interactive_marker_control.cpp) 的 handleMouseMovement、moveViewPlane、moveZAxisRelative、rotateXYRelative、rotateZRelative；不把相机深度移动误称全局 Z 升降。

## 贴地 / 自由 XYZ 与编辑层

- 默认贴地模式：中心沿地图 XY 平面移动，服务端根据所选官方 tomogram 表面派生 Z；不吸附 XY、不自动跨墙或跨层。数值 Z 只读，应用 XY 后回显真实表面高度。
- 自由 XYZ：保留完整空间位置及姿态预览；Z 可编辑，显式“贴地”仍可用。切回贴地并不会偷偷改已有点；若已有高度不合法，提供“贴回表面”恢复入口。
- 下拉“编辑层”是后续放置/编辑的层约束，不是把点搬到另一个楼层。`-1` 表示自动跟随当前表面，其余是压缩分层切片 ID，不等于物理楼层编号。
- 模式发布 String 到 `/d1max/pct_preview/selection_mode`（ground/free）；编辑层发布 Int32 到 `/d1max/pct_preview/layer`。均等待 status 确认，2 秒未确认则保持原设置，不自动重试。
- 有未应用坐标时禁用切换。外部修改模式/编辑层时保留输入草稿，锁定 Apply/Plan，明确提示先恢复后重新编辑。
- status 提供 selection_mode、active_layer、available_layers=[{id,label}] 和 start_validation/goal_validation={valid,reason,layer_id}。每个端点显示自己的所属层、可通行性或简短中文原因；完整原因保留在悬停提示。已明确不合法的端点不启用规划。

## 实机导航执行面板

`d1max_pct_rviz_tools/MotionControlPanel` 仅在显式导航配置中加载，RViz 配置必须绑定 `Session ID`。面板订阅 `/d1max/live_planning/motion_status`（String JSON），只接受 schema 1、本会话、严格递增源时间戳；源时间和本地收包时间均须在 0.6 秒内。状态包含 phase、reason、stop_reason、generation、armed、can_execute、velocity、max_speed、max_yaw、single_floor_only、gate_reason 和 acceptance_blockers。

“开始执行”要求新鲜的 can_execute、未使能、同层限制及空阻塞列表。确认框默认取消，明确低速、平地、现场监护和物理急停；确认结束后再次核对状态、代次和限速。面板只发布意图到 `/d1max/live_planning/motion_command`，JSON 字段为 session_id、stamp（Unix 秒）、id（32 位 UUID hex）、action（execute/stop）和 generation，不连接 SDK、不调用使能服务。速度、定位、控制权和实际执行许可由后端独立检查。

“停止导航”在状态断流时仍可发送，绑定配置中的会话和最后一次代次（尚未收到状态时为 0）；后端应按会话处理停止，不依赖旧代次仍有效。发送请求不代表已确认物理停止，紧急制动使用物理急停。

`NavigationDiagnosticsPanel` 的 `Motion Capable: true` 仅将标签改为“定位与规划监视”；它仍验证原有被动 diagnostics 消息，不增加任何控制接口。
