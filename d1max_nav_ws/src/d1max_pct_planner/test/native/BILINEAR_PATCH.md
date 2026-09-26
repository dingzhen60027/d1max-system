# PCT native cell-centre interpolation correction

Date: 2026-09-23. Scope: `pct_planner_vendor/planner/lib/src/map_manager/dense_elevation_map.cc` only.

## Fault and coordinate contract

The wrapper maps world XY to rounded integer grid centres. A* copies integer
`Node::idx` into `PathPoint::x/y`; GPMP copies those positions into its state
without adding 0.5. Thus the cost samples are at integer coordinates.

Both `GetValueBilinear` and `GetValueBilinearSafe` previously selected a lower
corner with `floor(x - 0.5)` but weighted that corner with `x - lower`. Weights
could reach 1.5; the result was extrapolation with half-cell discontinuities,
not continuous bilinear interpolation. For an x-axis row `[0, 0, 50, 0, 0]`,
the old code returned 74.5 at x=2.49 and 25 at x=2.50. At 20 cm resolution,
2 mm of motion changed the scalar by 49.5 cost units.

The GPMP obstacle factors call the safe method. Its gradient is recomputed
from the same four corner costs: cached incoming gradient arrays are not used
for that result. Within an old branch the derivative matched the extrapolated
scalar, but at the half-cell jump there was no matching finite derivative.
Changing the incoming gradient signs would not repair this issue.

## Minimal change

- Interpolate at integer cell centres using `floor(x)` and fractions in [0, 1].
- Preserve the existing per-corner layer selection and its cost values.
- Clamp out-of-range coordinates to the border; the outward derivative is
  zero. At the boundary itself return the interior one-sided derivative.
- Handle one-cell dimensions as constant and reject nonfinite coordinates.
- Derive gradients analytically from the returned scalar, including the clamp.

No cost threshold, legal-cell mask, inflation, clearance, ground-step check,
or trajectory validation has changed. The scalar has the convex-hull property
for its four selected corners. It is continuous for a fixed height hint/layer
selection; existing discrete layer changes retain their existing semantics.
An optimizer still requires independent collision validation.

## Isolated regression

From `/home/dndx/d1max_nav_ws`:

```bash
python3 -m pytest -q src/d1max_pct_planner/test/test_native_bilinear_cost.py
```

The fixture directly compiles the production `.cc` with Eigen headers and the
standalone C++ probe into pytest's temporary directory. It does not use ROS,
GTSAM, installed planner `.so` files, or a robot connection, and never replaces
any runtime library. Sixteen cases cover both API variants: constants, planes,
integer samples/peaks, half-cell continuity, boundaries/singleton dimensions,
finite-difference gradients, unchanged layer selection, and nonfinite input.

Run this regression and the native before/after route acceptance suite before
replacing any built planner library. Keep the old runtime binaries and map hash
for a controlled comparison; this patch alone does not prove all path-waviness
issues are resolved.
