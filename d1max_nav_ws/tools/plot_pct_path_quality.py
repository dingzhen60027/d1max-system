#!/usr/bin/env python3
"""Plot saved offline results, without rerunning native planning or ROS."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src/d1max_pct_planner"))
from d1max_pct_planner.tomogram_map import TomogramMap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a fresh output file")
    tomo = TomogramMap(WS / "maps/processed/sc_pgo_20260919_pct_flat_floor_v6_20260923/tomogram.npz",
                       max_ground_step_m=.17)
    cells = np.argwhere(tomo.ground_known[0])
    world = tomo.center + (cells - tomo.offset) * tomo.resolution
    valid = tomo.valid[0][tuple(cells.T)]
    fig, axes = plt.subplots(2, 1, figsize=(14, 11), constrained_layout=True)
    for ax, case in zip(axes, ("straight_23m", "preserved_user_39m")):
        before = json.loads((args.before / (case + ".json")).read_text())
        after = json.loads((args.after / (case + ".json")).read_text())
        assert before["passed"] and after["passed"]
        a = np.asarray(before["result"]["path"])[:, :2]
        b = np.asarray(after["result"]["path"])[:, :2]
        assert np.array_equal(a[[0, -1]], b[[0, -1]])
        # A rigid view rotation only; no route/map geometry modification.
        direction = a[-1] - a[0]
        direction /= np.linalg.norm(direction)
        basis = np.stack([direction, [-direction[1], direction[0]]], axis=1)
        transformed_world = (world - a[0]) @ basis
        first, last = (a - a[0]) @ basis, (b - a[0]) @ basis
        low = np.minimum(first.min(0), last.min(0)) - [1.2, 1.5]
        high = np.maximum(first.max(0), last.max(0)) + [1.2, 1.5]
        region = ((transformed_world >= low) & (transformed_world <= high)).all(1)
        for mask, colour, label in ((region & valid, "#dce6dc", "PCT legal"),
                                     (region & ~valid, "#d9b3b3", "PCT blocked")):
            ax.scatter(*transformed_world[mask].T, s=9, marker="s", c=colour, label=label, linewidths=0)
        ax.plot(*first.T, color="#4169b1", lw=1.8, label="Before: native 2/8")
        ax.plot(*last.T, color="#d66319", lw=2.2, label="After: repaired native + checked corridor refinement")
        ax.scatter(*last[[0, -1]].T, c=["#187638", "#69288d"], s=45, zorder=5)
        q0, q1 = before["quality"], after["quality"]
        ax.set_title(
            f'{case}: XY length {q0["xy_length_m"]:.2f} -> {q1["xy_length_m"]:.2f} m; '
            f'curvature P95 {q0["curvature_p95_per_m"]:.3f} -> {q1["curvature_p95_per_m"]:.3f} 1/m',
            loc="left", fontsize=12)
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]),
               xlabel="Along endpoint chord [m]", ylabel="Lateral coordinate [m]")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.15)
        ax.legend(loc="upper left", ncol=2, fontsize=9)
    fig.suptitle("Same PCT map and exact endpoints | strict collision checks retained\n"
                 "XY geometry comparison; not robot dynamics or navigation certification", fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
