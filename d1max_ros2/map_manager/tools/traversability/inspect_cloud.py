"""Read-only source inspection; generated diagnostics are written separately."""
from pathlib import Path
import argparse
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.signal import find_peaks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    xyz = np.asarray(cloud.points)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    poses = np.loadtxt(args.poses).reshape(-1, 3, 4)[:, :, 3]
    edges = np.arange(np.floor(xyz[:, 2].min()), np.ceil(xyz[:, 2].max()) + .05, .05)
    counts, _ = np.histogram(xyz[:, 2], edges)
    peaks, _ = find_peaks(counts, distance=8, prominence=1500)
    data = {"source": str(args.pcd), "points": len(xyz),
            "bounds": [xyz.min(0).tolist(), xyz.max(0).tolist()],
            "percentiles": np.percentile(xyz, [1, 5, 25, 50, 75, 95, 99], axis=0).tolist(),
            "z_peaks": sorted([[float(edges[p] + .025), int(counts[p])] for p in peaks], key=lambda a: -a[1]),
            "pose_bounds": [poses.min(0).tolist(), poses.max(0).tolist()], "poses": len(poses)}
    print(json.dumps(data, indent=2), flush=True)
    (args.out / "inspection.json").write_text(json.dumps(data, indent=2), encoding="utf8")
    plt.style.use("dark_background")
    sample = xyz[::max(1, len(xyz)//180000)]
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    for ax, dims, title in zip(axes.flat[:3], [(0, 1), (0, 2), (1, 2)], ["XY - height color", "XZ", "YZ"]):
        ax.scatter(sample[:, dims[0]], sample[:, dims[1]], c=sample[:, 2], s=.16, cmap="turbo", vmin=-6, vmax=6, rasterized=True)
        ax.plot(poses[:, dims[0]], poses[:, dims[1]], color="white", linewidth=.5, alpha=.8)
        ax.set_aspect("equal"); ax.set_title(title); ax.set_xlabel("XYZ"[dims[0]] + " (m)"); ax.set_ylabel("XYZ"[dims[1]] + " (m)")
    axes[1, 1].plot(edges[:-1], counts)
    axes[1, 1].set_title("Height histogram (5 cm bins)"); axes[1, 1].set_xlabel("Z (m)")
    fig.tight_layout(); fig.savefig(args.out / "source_inspection.png", dpi=150); plt.close(fig)
    small = cloud.voxel_down_sample(.16)
    small.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.35, max_nn=35))
    pts, normal = np.asarray(small.points), np.asarray(small.normals)
    flat = np.abs(normal[:, 2]) > .94
    floor = small.select_by_index(np.flatnonzero(flat))
    planes = []
    for _ in range(5):
        if len(floor.points) < 500: break
        plane, inliers = floor.segment_plane(.055, 3, 500)
        inpts = np.asarray(floor.points)[inliers]
        planes.append({"plane": plane.tolist(), "points": len(inliers), "median": np.median(inpts, 0).tolist(), "bounds": [inpts.min(0).tolist(), inpts.max(0).tolist()]})
        floor = floor.select_by_index(inliers, invert=True)
    print("PLANES", json.dumps(planes, indent=2), flush=True)
    data["horizontal_planes"] = planes
    (args.out / "inspection.json").write_text(json.dumps(data, indent=2), encoding="utf8")


if __name__ == "__main__":
    main()
