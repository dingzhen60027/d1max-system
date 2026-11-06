# 导航行为树链路修复与默认包同步

日期：2026-11-04。范围：本轮行为树审查确认的软件交接问题；不启动生产服务、不连接 SDK、不发送目标或运动指令。

## 结果

修复已进入唯一默认包 `/home/dndx/d1max_nav_ws/experiments/navigation_runtime`，没有新增入口或分支。

- 原 manifest：`8cdeecaaef1138fa8c0c12992b8b0e4741e26e2d346c306c512fcd1467a59926`
- 新 manifest：`ccbb4033a08a3a24301a55a87d6a935785dc78af22cee6fcca06468713c4b24d`
- Web activation、命令行 selector、两层启动脚本绑定一致；实际 Web 加载校验通过，命令行验证 10,000 个封存成员。
- 同步包含运行 Python、BT/跟踪器/SCAN 生产 ELF、相关头文件和源码快照。未修改运动参数、楼梯几何、地图、SDK、碰撞语义或验收标志。

## 按执行顺序修复

| 环节 | 原问题 | 本轮处理 |
|---|---|---|
| Web → 默认入口 | 发布清单已更新，但兼容入口哈希仍旧，启动前即拒绝 | 发布工具统一更新 Web/CLI 派生绑定；先校验封存文件，不在运行时自动信任未知修改 |
| 全局路线 → 连续跟踪 | 已有固定路线仍依赖全局规划心跳；监控恢复与 BT 恢复串行等待 | 按 BT 阶段选择依赖；计算和取消中的 Compute 继续监督，路线提交后不把全局心跳作为局部跟踪输入。新鲜状态恢复窗口并行进行，原恢复总预算仍有效 |
| 新任务 → 参考管理 | 旧路线退役未完成时新路线被拒收，随后不再发送，永久等待参考 | 仅对明确“尚未接管、等待旧路线退役”回执单槽重试。同一 task/route/hash，新 delivery sequence；真正退役由原接收者确认；取消或超时不复活旧任务 |
| 局部轨迹 → 跟踪状态 | 许可过期实际无输出，却仍显示 following；预览可能被误认执行或到达 | 几何就绪、执行许可、正在跟踪分开表达；初次预览仍能进入授权，执行后失去有效许可显示等待，不冒充到达 |
| SCAN 提交 → RViz | 大轨迹显示消息在执行锁和控制回调组内同步发布 | 固定一槽不可变消息，独立 20 ms 显示回调，锁外发布；取消清未发槽。运动命令和提交 ACK 顺序不变 |
| 目标到达 → 停稳 → 完成 | 一次性 StopReport 被当成 400 ms 实时流；重复到达心跳移动停止边界；BT 延迟消费又丢失完成事实 | 首个到达事件固定停止边界。BT 验证实测停稳后，适配器用同身份 terminal-goal 许可消费原事件；当前位置/朝向仍须有效。BT 收到完成时检查原时间，之后按任务身份保留事实，不刷新时间、不重复要求一次性事件“永远新鲜” |

到达链无相互等待：

`局部到达请求 → SDK 停稳报告 → BT 验证停稳并发布终态许可 → 适配器复核实际终点 → Follow 完成 → BT 收尾`

新目标清理旧任务缓存；迟到回执、旧定位身份、错误停止身份不能完成新任务。取消不被迟到的成功回调覆盖。普通局部候选失败仍保留可用旧曲线，不重新计算已提交全局路线。

## 验证证据

以下各组有重叠，不叠加成一个“整链实机成功”数字。

| 验证 | 结果与适用范围 |
|---|---|
| 参考交接/到达同输入对照 | 原版 27 失败、82 通过 → 修复版 109 全通过；其中 StopReport 专项原版 7 失败 → 25 全通过 |
| 邻接 Python 回归 | 602 通过，覆盖连续换目标、参考窗口、退役、楼梯平台交接 |
| BT 原生单元 | 426 通过；真实到达回调原失败场景修复，健康组件 32 通过 |
| 跟踪核心 | 447 通过；同输入 85 项由 5 失败变为全部通过，保留速度/加速度及换轨相关回归 |
| SCAN 生产组件集成 | 252 通过：执行账本 155、跟踪集成 85、滚动跟踪 5、快照交付 7。使用生产 GridMap/优化器库，不换简化地图 |
| 显示接线及并发 | 原版 3 项接线失败 → 修复后 3 通过；慢显示发布时并发提交、取消、新任务不占执行锁，队列容量固定为一 |
| Web 生命周期 | 268 通过；启动/停止/重启外部 systemd、HTTP、SDK 操作均模拟。另单独调用真实 `_activation()`，修复前入口身份报错，部署后通过 |
| 部署工具 | 9 通过；错包、修改入口、权限错误、符号链接等负例不静默绑定 |

初轮两类测试环境问题没有掩盖：滚动集成漏传原 CMake 的 `D1MAX_ROLLING_SOURCE`，补齐后同一个 ELF 5/5 通过；依赖测试浮点时间微小倒退，修正测试时刻，不放宽生产时钟规则。原始失败记录保留。

证据入口：

- [部署校验与完整差异](/home/dndx/d1max_nav_ws/experiments/bt_lifecycle_repair_20261104/verification.json)
- [部署源码/产物范围](/home/dndx/d1max_nav_ws/experiments/bt_lifecycle_repair_20261104/scope.json)
- [参考交接与停稳回归](/home/dndx/d1max_nav_ws/experiments/reference_delivery_recovery_20261104/review.json)
- [BT 与健康回归](/home/dndx/d1max_nav_ws/experiments/bt_contract_repair_20261104/review.json)
- [跟踪回归](/home/dndx/d1max_nav_ws/experiments/system_navigation_review_20261104/local_feedback/review.json)
- [SCAN 集成与显示回归](/home/dndx/d1max_nav_ws/experiments/system_navigation_review_20261104/local_feedback/display/review.json)
- [生产编译输入核对](/home/dndx/d1max_nav_ws/experiments/system_navigation_review_20261104/local_feedback/compile_inputs.json)
- [Web 部署后真实加载](/home/dndx/d1max_nav_ws/experiments/bt_lifecycle_repair_20261104/web_activation_after.json)

## 无运动复核命令

```bash
cd /home/dndx/d1max_nav_ws
python3 -B experiments/bt_lifecycle_repair_20261104/sync_default.py verify
bash tools/navigation_entry.sh --check-release
/home/dndx/go2_nav/build/env/map_manager_venv/bin/python -B \
  experiments/bt_lifecycle_repair_20261104/run_web_checks.py after
ctest --test-dir experiments/bt_contract_repair_20261104/build --output-on-failure
```

参考交接同输入回归（不初始化 ROS 图、不连接 SDK）：

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  experiments/reference_delivery_recovery_20261104/run_tests.py -q -p no:cacheprovider \
  src/d1max_pct_scan/test/test_route_delivery_recovery.py \
  src/d1max_pct_scan/test/test_formal_follow_status.py \
  src/d1max_pct_scan/test/test_arrival_stop_event.py
```

对照原版时在 `run_tests.py` 后添加 `--baseline`，其失败是预期的修复前对照，不应拿来运行机器狗。

## 边界与后续验收

- 这是生产代码构建、被动回调、原生组件及部署校验，不是实际上下楼验收；本轮没有完整 rosbag 回放或实机运动。
- 已修复上述确定的交接缺陷，不能据此宣称所有历史速度归零都已消失。真实数据过期、碰撞、失权及急停仍保留原处理。
- 显示已不占执行锁，但运动命令/ACK 仍保持原同步顺序，共享执行器和中间件长阻塞的端到端影响仍需真实运行测量。未将其冒充已证实的全部停顿根因。
- 既有平地控制参数、楼梯支撑/通道处理和生产地图库字节保持不变；回归通过不能替代现场效果确认。
- 精确旧文件与选择器备份保留在 `experiments/bt_lifecycle_repair_20261104/before` 和 `binding_backups`。回退须成套处理，不自动恢复运动许可或旧目标。本轮未 commit/push。
