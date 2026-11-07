# D1 Max 导航系统

https://github.com/user-attachments/assets/98762b15-f5b5-4052-ac03-6ae482ae10e1

面向 D1 Max 轮足机器人的导航系统，覆盖点云建图、定位、路径规划与运动控制，面向室内外及多楼层场景。

[快速开始](#快速开始)　 /　 [导航链路](#导航链路)　 /　 [项目文档](#项目文档)

## 导航链路

**定位** → **全局规划** → **局部规划** → **轨迹跟踪** → **SDK 控制**

行为树负责管理任务与目标切换。Web 用于连接和启动服务，RViz 用于初始定位、三维选点和查看规划结果。

| 模块 | 当前实现 |
| --- | --- |
| 建图与定位 | 双雷达与 IMU、Faster-LIO、SC-PGO、全局重定位 |
| 全局规划 | 基于 PCT 构建可通行地图，规划地面、楼梯与平台路线 |
| 局部规划 | 基于 SCAN 滚动地图，结合实时环境生成局部避障轨迹 |
| 跟踪与控制 | 跟踪局部轨迹，生成前进与转向指令，通过 SDK 控制机器狗 |

PCT 和 SCAN 是当前规划后端，任务管理与算法实现分离。全局路线使用 `map` 坐标，局部参考与跟踪使用连续 `odom` 坐标。

## 快速开始

获取源码并检查工程文件：

```bash
git clone https://github.com/dingzhen60027/d1max-system.git
cd d1max-system

source tools/deployment-env.sh
python3 tools/deployment_preflight.py --scope source
```

生成导航源码的构建计划：

```bash
bash tools/build-source.sh --scope nav --output /absolute/path/to/new-build
```

该命令默认只显示计划；配置好依赖后追加 `--apply` 才会开始构建。完整步骤见 [部署指南](docs/DEPLOYMENT.md)。

地图、录包、厂商 SDK 和本机封存运行包不包含在 Git 中，需要另行准备。

## 项目结构

```text
d1max_nav_ws/   定位、行为树、规划、跟踪与 RViz
d1max_ros2/     Web、SDK 桥与 Foxglove 管理
tools/          环境检查、源码构建与同步工具
docs/           部署、交接与验证记录
```

导航统一入口：[`d1max_nav_ws/tools/navigation_entry.sh`](d1max_nav_ws/tools/navigation_entry.sh)

## 项目文档

[部署指南](docs/DEPLOYMENT.md)　 /　 [模块与入口](docs/PROJECT_GUIDE.md)　 /　 [导航主线](d1max_nav_ws/src/d1max_pct_scan/README.md)　 /　 [验证与已知问题](docs/KNOWN_ISSUES.md)

<sub>已开展平地和楼梯实机测试；完整跨楼层执行、动态避障恢复及长期稳定性仍在验证。第三方组件保留各自的许可证与来源说明。</sub>
