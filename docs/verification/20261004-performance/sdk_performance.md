# SDK buffer and publication optimization — 2026-10-04

范围为既有 SDK bridge 源码、隔离构建与纯逻辑测试。没有连接 SDK、接管控制、机器人运动、安装、部署、服务重启、提交或推送；物理验收状态不变。

实际修改目录：`/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge`。只读基线为 `/home/dndx/d1max-system` 的 `b344f52501851a109c5b55220e7f7b75fd638962`，基线 SDK 文件无未提交改动。生成接口头文件来自既有 experiments 隔离构建。

## 实际热点和改变

- 200ms main tick 原来同步发布 connection/behavior/speed/status，以及启用时的 joint status。Zenoh publish 阻塞会暂停主 executor 的 ownership/event 消化。现在这些最新状态分别进入固定单槽，由既有 telemetry worker 发布。JSON 与字符串仍在状态锁外构造，原生成/接收/采集时间保留，没有新增线程或 writer。单次 transition event 路径保持独立。
- `report=mc_` 原在 writer 共用状态锁下复制 deque 历史。现在固定 512 条 MC timing ring 保留原有 2 秒窗口、频率诊断、时间戳拒绝与 stale 门槛；diagnostics 只复制 176 字节 Snapshot 和必要字符串。
- writer 的 typed telemetry 进入 latest mailbox 时移动消息；CommitOutbox 保留 8 槽，只合并全字段一致重发及同一不可变结果的 false→true vendor ACK 更新。迟到旧 ACK 不降级；身份、版本、采集时间、期限、速度或结果变化均不能合并，第 9 个不同结果仍返回 Overflow 并锁存安全 veto。
- ACK drain 由动态 deque 与消息深复制改为固定 8 条本地数组和移动传递。先发布事务 ACK，再发布 latest typed state 与 JSON；SDK MC clock epoch 字符串仅在 generation 变化时重建。

已有唯一 20Hz steady writer、独立 typed input executor、同锁最终检查/Move 提交、重放/撤销/许可/MC 失鲜/停止证据合同没有放宽。writer 未新增，deadline 继续跳过错过周期，不补发陈旧命令。

## 纯 core 计时与分配证据

Intel i9-14900HX / GNU C++ 11.4.0 / `-O3 -DNDEBUG` / CPU 2。相同 benchmark 源码编译到基线与当前生产 core 头文件；MC 入口设置 `noinline,noclone`，28 个进程（计数和无 allocator hook 计时，各 7 次交替前后运行）checksum 都是 20111600000。最终计时在 root 明确释放的团队静默窗口完成。初次并发构建期间的试测时延未用于本报告。

| 生产 core 路径 | 基线中位 ns/op | 当前中位 ns/op | 比例 |
| --- | ---: | ---: | ---: |
| mc_accepted_sample_50hz | 13.1182 | 6.79501 | 1.93× |
| mc_diagnostic_snapshot_50hz | 79.2116 | 19.4225 | 4.08× |
| commit_enqueue_drain_distinct | 470.084 | 331.411 | 1.42× |

100 万次 accepted MC sample：31,251 次堆分配、16,000,144 字节累计请求 → 0。50 万次 MC diagnostic snapshot：3,000,000 次分配、1,066,000,000 字节累计请求 → 500,000 次分配、10,000,000 字节（剩余是 `streaming_unconfirmed` 状态字符串）。20 万条不同 commit 的入队/drain：3,075,056 次分配、382,602,472 字节累计请求 → 1,400,000 次分配、61,800,000 字节；夹具创建消息自己的分配仍计入两侧。

MC diagnostic 路径峰值 C++ 请求堆字节 4,244 → 20，commit 路径 15,549 → 2,781。该计数不包含分配器元数据、栈、ROS/SDK/Zenoh 内存。**MC 固定对象从 336 增到 8,464 字节（+8,128），以预留最坏 512 条记录消除动态分配。** 两版 benchmark peak RSS 均为 3,584 KiB，没有证明完整服务 RSS 下降。这些纳秒级 microbenchmark 改进不能换算为整导航速度或真实 stop 延迟。

全部原始 28 次结果、时延范围、分配计数、源文件 SHA256、硬件与编译参数见 [sdk_buffer_benchmark.json](./sdk_buffer_benchmark.json)。开发目录的 `run_sdk_buffer_benchmark.sh` 只编译/执行纯 core 二进制，该脚本及开发机隔离构建树没有随本证据目录发布；生产 probe 源码保留在 SDK package 的 `src/benchmark_execution_buffers.cpp`。重新计时时需保持其他构建和性能测量静默。

## 验证与负例

`sdk_monitor_bridge` 在 `/tmp/d1max-sdk-architecture-gtUR7F` Release 隔离目录编译通过，最终 CTest **14/14** 通过。已有部分 binary 未构建导致首次 full CTest 2 个 Not Run，补建后最终全部通过；没有执行 install 目标。仅保留既有 Time/OpenSSL/历史 indentation 编译警告。

新增/扩大测试覆盖：真实 Transport first commit→vendor ACK 与同事务 outbox 合并输出相等；1 万重复 ACK 只占 1 槽；迟到 ACK 不降级；SDK session/control epoch/commit/source/期限/applied/版本各变更不合并；第 9 不同结果 Overflow；故意阻塞消费端时生产者继续完成 1 万次 latest mailbox 更新且原 source/expiry 不改变；固定 MC 环形存储 wrap、512 flood 上限、2 秒 prune、断连清空；Snapshot streaming/stale/replay 原合同继续通过。

独立代理只读交叉审查了 CommitOutbox 全部不可变字段与 MC/source/arrival/anchor 保留，未发现本轮新增正确性问题。

## 尚不能由软件测试证明的边界

Zenoh/middleware publish 仍可能阻塞现有 telemetry worker、推迟 ACK 送达，并在不同事务积压超限时触发安全 veto；线程销毁 join 也不保证期限。SDK 头文件仅给出 Move(..., timeout_ms=0, WriteHandler)，没有可证明的调用返回/内部锁最长阻塞承诺。保留同锁 final submit 意味着 vendor Move 卡住时不能通过另开 writer 或解锁重发去伪造安全边界。供应商阻塞、OS 调度、进程死亡、硬件独立停止与真实 250ms/物理制动界限仍需现场/供应商证据；这些测试不能把任何物理验收标志置真。
