# Web 启停与观察节点退出修复（2026-10-25）

## 范围与交付

本轮只修改启动/取消状态收尾及两个观察节点的 ROS 关闭逻辑。未连接机器人，
未启动导航核心、RViz 或 SDK，未提交初值/目标/运动命令。仅实际停启了空闲 Web。
没有修改地图、PCT/SCAN、跟踪控制、楼梯参数、权限或物理验收标志。

仍使用原入口及 `experiments/navigation_runtime`，没有新建运行分支。

- 原 manifest：`cea324056bbf614af5168d34731951370b07ae400319b6dd90753079cb276ed8`
- 新 manifest：`39e8cc7131c13c54a0da279555338667b30fcc9d725fa977c4742482c62aa710`
- 两份默认选择器已同步；Web 实际加载新进程后返回 `ready`、
  `startup_closure_verified=true`，导航 `stopped`，SDK 未连接、运动未授权。

## 修复

| 问题 | 原因 | 修复 |
|---|---|---|
| 启动中立即退出，可能吞掉本次启动结果或首次退出后不能再启动 | 已接纳新会话，但用旧会话 `prior.phase=stopped` 作为收尾依据 | 当前会话身份独立核对。用户取消且当前 core/view 进程组全部清空后允许再次启动；缺 ACK 时明确保留“是否启动未知”，不宣称任务退役或物理停稳。旧 stopped 兜底只用于相同会话 |
| 两观察节点退出出现 `rcl_shutdown already called` | `rclpy.ok()` 与 shutdown 间，信号线程可先关闭 context | 使用 `try_shutdown()`；仅容忍 context 已关闭且错误确为重复 shutdown 的竞态。真实中间件错误、Python 错误及回调异常仍传播 |

Web 修改位于 `map_manager/backend/single_floor.py`；观察节点为
`committed_route_view.py`、`component_health_monitor.py`。

没有把清理证明缺失变成新的永久禁用条件。身份被更换、仍存在归属进程或视图、
非用户取消的无 ACK 退出继续保留其真实不确定状态；不能冒充已完成清理。

## 同输入对照与启停检查

| 检查 | 修改前 | 修改后 |
|---|---|---|
| 新增 Web 启动取消/重启回归 | 2 失败、1 通过 | 全部通过，包含在 Web 254 项中 |
| 两观察节点同输入退出回归 | 2 失败、12 通过 | 14 通过，包含在观察节点/核心生命周期 106 项中 |
| 安装版本 Web 启停、默认包、RViz 归属管理 | — | 254 通过，外部服务操作均隔离/mock |
| 安装版本观察节点、核心退出和清理 | — | 106 通过，不初始化 ROS 图 |
| 真实 Web 服务启动 → 退出 → 再启动 | — | 退出 `Result=success`、`ExecMainStatus=0`、`MainPID=0`；重开后新 invocation/API 正常 |

Web 实际新进程 PID 依次为 332654、333340。独立连接管理器保持 PID 2003，
SDK monitor 始终 inactive/PID 0。浏览器页面恢复为“未启动 / 机器狗未连接”，
没有卡在启动、退出或版本变化状态。未连接时启动按钮禁用是当前正常状态，
没有将其当作实机启动成功。

## 默认包一致性

更新 13 个目标：9 份源码/安装/测试文件及 4 个既有 Python 缓存。
除实际 Web 入口外，同时更新了默认包中两份偏旧的 Web 源码快照。
所有缓存验证 checked-hash、源码摘要及 code object，避免只改 `.py` 却导入旧缓存。

独立重算全部 10,349 个封存成员；原 10,346 个成员无删除。85 个配置、42 个 SDK
安装成员、80 个 SDK 源码成员、1,490 个原生组件成员及 68 个楼梯/跨层成员保持原 hash。
10 份生成 YAML、制动模型及 RViz 布局去除会话身份/派生 hash 后等价。
选择器除 manifest 引用外不变，私有 activation 权限仍为 0600。

## 证据与复现

证据目录：`experiments/startup_exit_20261025/`。

```bash
cd /home/dndx/d1max_nav_ws
# 原版对照会在日志中出现预期失败；runner 校验这些失败恰是目标问题。
python3 -B experiments/startup_exit_20261025/run_regression.py before
python3 -B experiments/startup_exit_20261025/run_regression.py installed
python3 -B experiments/startup_exit_20261025/sync_default.py --verify
```

`before_regression.json`、`installed_regression.json` 保存命令、原始日志和摘要；
`deployment/independent_installed_audit.json` 保存独立全量核对；
`deployment/web_first_start.json`、`web_restarted.json` 保存实际 Web invocation 和 API 状态。
安装前精确文件、manifest、选择器和 preflight 位于 `deployment/before/`，
可依 `plan.json` 逐项恢复，不能在运行中只回退部分文件。

本轮结论限于上述软件缺陷、隔离启停流程及实际 Web 重启；未进行带机器人数据的
导航/RViz 实启动，也不宣称实机全链路已验收。未提交或 push。
