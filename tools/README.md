# 同步与发布

在统一发布仓库运行；源目录不会被写入。默认只比较校验和：

```bash
bash tools/sync-workspaces.sh \
  --app-root '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0' \
  --nav-root /home/dndx/d1max_nav_ws
```

审核差异后，在同一命令末尾添加 `--apply`。仅复制源码、配置和根目录两个入口脚本：

- 每次覆盖前，将发布目录旧文件备份到脚本打印的独立 `/tmp/d1max-source-backup-*`；不是地图或运行环境备份，临时目录会被系统清理。
- 不使用 `--delete`。源目录消失的文件不会自动删除发布目录副本，必须人工确认移除；发布目录有未提交修改时先检查，备份不能代替冲突审查。
- 不复制嵌套 Git / 子模块指针，保留依赖源码和许可证；不跟随外部符号链接。
- 不复制地图、录包、Web 运行状态、构建依赖、厂商 SDK、模型或本机凭据。规则集中在 [snapshot.rsync-filter](snapshot.rsync-filter)，Git 另有 `.gitignore` 防线。
- 不做 Git 暂存、提交、push，也不操作系统服务、机器人和 Foxglove 在线布局。

## 提交检查顺序

1. `git status --short`、`git fetch origin`，核对远程与现有变更。
2. 同步前先 dry run，核对源目录与发布目录的差异；应用后再次 dry run 应无待复制项。
3. 更新根目录 `SOURCE_SNAPSHOT.md`、`VERIFICATION.md` 和 `docs/KNOWN_ISSUES.md`，避免旧说明覆盖当前事实。
4. 跑相关离线单元测试和构建。Web 的 `test:ui-copy` 读取 `frontend/dist`，必须先 `npm run build`；该脚本仅用本机 fixture，不能改为生产 API。
5. `git add` 后检查 `git diff --cached --check`、文件清单、大小、二进制及凭据；忽略规则无法保护已经跟踪的文件。
6. 说明性 commit，正常 fast-forward push；不强推、不改写原运行工作区历史。检查远程 HEAD 与提交一致。

源码整理不要求启动或重启实机。需要连接机器人、重新部署、替换地图或发布布局时，另行确认操作范围。
