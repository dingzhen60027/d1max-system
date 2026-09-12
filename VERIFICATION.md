# 提交前检查 — 2026-09-12

本次在 `/home/dndx/d1max-system` 发布副本检查当前源码，没有连接机器人、启动实机节点、改动地图或重启受管服务。
运行工作区仅同步更新浏览器回归脚本的过期文案断言，不改变业务逻辑。

## 本次结果

| 范围 | 检查 | 结果 |
| --- | --- | --- |
| Foxglove | `npm ci --ignore-scripts`、`npm test`、`tsc --noEmit` | 41 项 TypeScript 测试通过，类型检查通过 |
| 本机生命周期 / MC 健康 / PCD 禁用加载 | `python3 -m unittest discover -s tests -p 'test_*.py' -v` | 18 项通过，使用 fake system / 临时目录 |
| Web 后台 | 现有地图 venv 中 `python -m unittest discover -s tests -v` | 29 项通过；没有删除用户地图 |
| Web 状态与箭头 | `node --test scripts/test-pose-estimate.mjs scripts/test-mc-status.mjs scripts/test-localization-status.mjs` | 12 项通过 |
| Web 生产构建 | `npm ci --ignore-scripts`、`npm run build` | 通过；点云分块大于 500 kB 警告保留 |
| Web 页面回归 | 构建后 `npm run test:ui-copy` | 8 类检查通过；390–1920 px；`errors=[]`、`unsafe=[]`，无真实写请求 |
| SDK 纯逻辑 | C++17 / `-UNDEBUG` 编译运行 MC、单向急停、会话、APP 接管测试 | 4 个可执行测试全部通过，不链接厂商 SDK、不连接机器人 |
| 定位纯逻辑 / 消息边界 | `pytest -q -p no:cacheprovider test --ignore=test/test_supervisor.py --ignore=test/test_icp_fusion_bridge.py` | 93 项通过；加载 ROS 消息类型，不创建 ROS 端点 |
| Shell | 排除 planner vendor 后，项目脚本 `bash -n` | 38 个通过 |
| 源码同步 | 校验和 dry run、源/目标重叠拒绝检查 | 无待复制差异；未自动删除发布目录文件 |

浏览器脚本首次运行缺少新副本的 `dist`，构建后发现旧断言仍期待“局部里程计异常”。
现仅将断言限定到实际告警区的“局部里程计已停止”，并检查具体故障原因和初值按钮禁用；最终回归通过。
测试截图和编译产物位于忽略目录或独立 `/tmp`，不进入 Git。

## 上传范围与边界

- 暂存树扫描常见私钥、GitHub Token、AWS Key、带凭据 URL 和配置密钥赋值，没有发现匹配项；这不是穷尽式安全审计。
- 核对 SDK vendor、`manager.local.json`、地图、Web 状态、构建缓存等排除规则；暂无待提交二进制差异、外部符号链接或子模块指针。
- 全部纳入文件均小于 10 MiB；第三方既有源码、论文和许可证保留，不改写原导航仓库历史。
- 新的模块目录/配置索引、安全同步工具、已知问题清单和来源记录已纳入本仓库。
- 本次没有重编全部导航 C++ / 第三方依赖，没有跑会创建 ROS 端点的两组 Python 测试或全链路 smoke，也没有执行新的实机运动/定位/急停验收。
- 先前的 149 项 Python、C++ 构建与合成 50 Hz 测试记录保留在定位模块 `VERIFICATION.md`，不是本次重测结果。轨迹清空、协方差简化与未标定等限制见 [KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)。

以下为首次提交检查，按日期保留，不代表当前实时状态。

# 历史提交前检查 — 2026-09-10

- Foxglove：30 项 TypeScript 单元测试通过；对应运行工作区源码与本快照逐文件一致。
- PCD 发布器：3 项禁用加载测试通过，禁用时不会读取 PCD 或启动 ROS。
- 地图 Web：3 项归档删除测试通过，测试仅操作自动创建的临时目录；没有清理用户地图。
- 软件急停：独立 C++ 安全状态测试编译及运行通过，使用纯逻辑状态，不连接 SDK 或机器人。
- 本项目 33 个 Shell 脚本通过 `bash -n` 语法检查。
- 待提交内容按规则扫描私钥、GitHub Token、AWS Key、带凭据的 URL、明文密码/密钥赋值，未发现匹配项；这不是穷尽式安全审计。
- 运行数据、厂商 SDK/模型、编译产物和环境不进入 Git；单个纳入文件均小于 10 MiB。
- 当前 Foxglove 布局已独立导出；原工作区、Git 历史、PCD、实时进程未因本次整理而移动、覆盖或停止。

本次未重新编译所有导航/第三方依赖，也未执行新的实机建图、定位、运动或急停操作。导航与环境的历史实测记录以各模块文档为准，不能把源码成功提交视为导航功能验收。
