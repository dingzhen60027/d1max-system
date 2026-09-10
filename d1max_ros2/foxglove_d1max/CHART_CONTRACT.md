# v0.7.2 compact dual-battery contract

Surface: existing native Foxglove workbench, not a standalone HTML analytics report.
Question: can the operator see the building, local sensing, measured motion, battery and emergency state without switching pages?
Takeaway: monitoring only; images and clouds are live/observed, no navigation or movement control is implied.

| Visual | Source and grain | Type / scale | Encoding / unknown |
| --- | --- | --- | --- |
| Building | configured PCD, all finite points | native 3D, static map frame | height coloring; no robot/model overlay |
| Local perception | two live PointCloud2 + JPEG | native 3D + Image | preserve original calibration caveats in tooltip/metadata |
| Measured speed | SDK RobotState, approximately 1 Hz | 30 s Plot ONLY; m/s vs rad/s separate | each field appears once; remove duplicate motion Gauges |
| Battery 1 / 2 | same SDK state, two current values | two native horizontal Gauge bars inside ONE always-visible native group, 0–100 percent | identical red-yellow-green charge scale; B1/B2, percent units and 0/50/100 ticks; missing remains missing |
| Posture / source / SDK | enum/connection telemetry | icon indicators, not a 3D robot | distinct symbols, tooltip/aria state, never infer joint pose |
| Soft/hardware stop | explicit emergency enum | icon indicators + one-way stop button | red stop octagon, neutral unknown ?, released check; not color alone |

Use native Gauge / Plot primitives; the custom extension is only a compact status pictogram and safety strip.
Palette: blue/cyan and gold for speed series; neutral grid/background. User-requested battery semantics explicitly use the native red-yellow-green colormap: low→middle→high charge on the same 0–100 scale, not categorical B1/B2 colors. This is a continuous display scale, not claimed manufacturer alarm thresholds. Red is also retained for emergency semantics.
Footprint: full-width PCD/live views above one bottom instrument row. Both battery bars share one small native group (one fixed group tab, no alternate page). Group occupies 15% of width × 32% of height = 4.8% of the mosaic, down from 22% × 72% = 15.84% (about 70% smaller). The bottom row needs 32% height so native bar ticks do not clip at the actual 1600×934 desktop viewport. Status/estop stays visible beside it; camera pair gets 40% of bottom width. No blank sidebar and no large battery dials.
No prose cards, tabs, markdown panels or raw JSON on the main page. Keep only terse metric labels/units and safety labels; full meanings remain accessible via tooltip/aria.
At least 8–12 real temporal points before assessing trend shape; no synthetic history in the live workspace. Keep longer diagnostic scope in saved backup only.
QA: actual desktop reload, all clouds/images visible, native gauges respond to true data, no text collisions, no robot command calls, no URDF/assets, no motion interfaces; preview tests cover unknown/stale/replay/emergency states.
