# Upstream

- Repository: https://github.com/liangheming/FASTLIO2_ROS2
- Imported package: `fastlio2`
- Commit: `f516daac08bc46e50e814a2e7d6c8352ed8141bb`
- Imported: 2026-08-03
- License: MIT (`LICENSE`)

Only the FAST-LIO2 mapping package is imported. The upstream `pgo`, `hba`,
`localizer`, and `interface` packages are intentionally excluded so this
mapping backend does not add GTSAM or alter the existing localization stack.

Build integration uses the vendored Sophus 1.22.10 headers in
`third_party/Sophus`; algorithm source files otherwise track the upstream
commit above.
