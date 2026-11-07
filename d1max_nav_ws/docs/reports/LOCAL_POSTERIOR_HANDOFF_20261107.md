# 下台阶时局部位姿跳变：原因与修复

## 实测事实

11 月 7 日会话 `54529669636c4d69b325a709816ce2ef` 的任务 `.5` 为下楼任务。
14:36:39.060～39.335，SCAN 报接入残差 0.185524～0.363778 m；这些不是相邻两帧定位跳变量。
随后一次接续搜索失败并超出求解期限；39.537 定位输出报 `local_pose_jump`，
39.619 行为树以 `hard_localization_lost` 结束任务。
当时最后有效 IMU 源龄约 21 ms，LIO 后验源龄约 301 ms。
日志未记录被拒绝的两帧姿态，不能据此证明实机下降了多少，或把物理落阶认作已确认根因。

## 修改

原检查把两次预测的位姿差除以约 20 ms 的输出间隔；新 LIO 后验对较早预测的修正
也被当作该间隔内的实际运动，且一次越界会锁存为硬失锁。

- 同一后验的预测仍核对实际运动；后验更新交给已有运动传播和有界修正处理。
- 保留原有 0.25 m / 0.15 rad 修正范围及原始源龄，没有新增楼梯专用放宽参数。
- 单次预测异常拒收，不更新位姿、时间和有效期限；新的正常观测可在同一身份内恢复。
  持续异常时原有效输入按原期限过期，暂停等待；不能靠等待变长而接纳异常坐标偏移。
- 显式上游故障、epoch 变化和重新初值继续沿用原重置合同。
- 轻量 `LocalStateTransition` 记录两帧位置、姿态、目标/后验/IMU 原始时间、变化量和裁决。
  相同拒收原因最多每秒一条，恢复事件即时记录。

修改范围仅定位的 `estimation/navigation.py`、`estimation/continuity.py`、
`navigation_output.py` 及相应离线检查。规划、参考、控制、SDK、地图和所有参数保持原值。

## 验证及边界

证据目录：`experiments/local_posterior_handoff_20261107/`。
冻结旧包同输入的上/下 18 cm 后验修正及单异常样本恢复共四例全部失败；
修复后通过。真实生产预测器与导航输出核心同时覆盖平地、转弯、上下台阶、
俯仰变化、延迟后验、漏一帧扫描、持续大偏移、源失效和重定位。
`localization_tests.xml` 记录本轮被动定位检查；`before.json`、`after.json`
记录连续平地、转弯和上下楼运动的逐输出校验和。

未录制本次原始传感器包，因此这是可复现的软件故障对照，不是对整次现场运动的重放，
也不能宣称实际下楼已验收。超出原有修正范围的估计仍等待可信新数据；
SCAN 搜索失败和轨迹接入偏差的独立问题没有在本次修改中被隐瞒或绕过。

复现（不初始化 ROS、不连接 SDK）：

```bash
cd /home/dndx/d1max_nav_ws
source /opt/ros/humble/setup.bash
source experiments/navigation_runtime/interfaces/install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/home/dndx/d1max_nav_ws/src/d1max_localization:$PYTHONPATH"
python3 -m pytest -q src/d1max_localization/test/test_navigation_estimation.py \
  src/d1max_localization/test/test_navigation_continuity.py \
  src/d1max_localization/test/test_navigation_output.py \
  src/d1max_localization/test/test_realtime_navigation_output.py
python3 experiments/local_posterior_handoff_20261107/compare_transition.py \
  --output experiments/local_posterior_handoff_20261107/reproduced.json
```
