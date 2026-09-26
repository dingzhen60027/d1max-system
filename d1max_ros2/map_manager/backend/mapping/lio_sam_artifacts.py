"""Explicit, completed LIO-SAM result import; no algorithm runtime integration."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil


RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,159}$")
MANIFEST = "lio_sam_manifest.json"


def completed_artifact(directory: Path):
    """Allow only the declared global feature map, never keyframes/trajectory."""
    directory = Path(directory)
    manifest_path = directory / MANIFEST
    path = directory / "GlobalMap.pcd"
    if (not RUN_ID.fullmatch(directory.name) or directory.is_symlink()
            or manifest_path.is_symlink() or path.is_symlink()):
        return None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if (manifest.get("schema") != "d1max.lio_sam_artifact.v1"
                or manifest.get("status") != "complete"
                or manifest.get("output_file") != "GlobalMap.pcd"
                or manifest.get("sensor_mode") not in {"front", "dual"}
                or not path.is_file()
                or path.stat().st_size != manifest.get("size_bytes")):
            return None
        return path, manifest
    except (OSError, ValueError, AttributeError, TypeError):
        return None


def completed_artifacts(root: Path):
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        return
    for directory in sorted(root.iterdir()):
        if directory.is_dir() and (result := completed_artifact(directory)):
            yield result


def publish_run(run: Path, maps_root: Path, sensor_mode: str) -> dict:
    """Copy one finished result into a new directory, preserving original data.

    This does not select the map, change Web state, or modify the source run.
    The manifest is written last, so incomplete copies are never cataloged.
    """
    if sensor_mode not in {"front", "dual"}:
        raise ValueError("sensor_mode must be front or dual")
    run = Path(run).resolve(strict=True)
    if not RUN_ID.fullmatch(run.name):
        raise ValueError("Unsupported run directory name")
    source_manifest = run / "manifest.json"
    source = run / "map" / "GlobalMap.pcd"
    if source_manifest.is_symlink() or source.is_symlink() or (run / "map").is_symlink():
        raise ValueError("Source manifest/map must not be a symbolic link")
    original = json.loads(source_manifest.read_text(encoding="utf-8"))
    completion = original.get("completion") or {}
    if not str(completion.get("status", "")).startswith("completed "):
        raise ValueError("Only explicitly completed LIO-SAM runs can be imported")
    if Path(completion.get("map", "")).resolve() != source:
        raise ValueError("Completion map must be this run's map/GlobalMap.pcd")
    input_description = str(original.get("input", ""))
    mentions_rear = "rear" in input_description.lower()
    if "front" not in input_description.lower() or mentions_rear != (sensor_mode == "dual"):
        raise ValueError("sensor_mode disagrees with the run manifest input description")
    if not source.is_file() or source.stat().st_size == 0:
        raise ValueError("Completed GlobalMap.pcd is missing or empty")
    with source.open("rb") as stream:
        header = b""
        for _ in range(100):
            line = stream.readline(65536)
            header += line
            if line.startswith(b"DATA ") or not line:
                break
    if b"FIELDS x y z" not in header or b"DATA " not in header:
        raise ValueError("GlobalMap.pcd does not contain the expected PCD header")

    maps_root = Path(maps_root).resolve(strict=True)
    library = maps_root / "lio_sam"
    if library.is_symlink():
        raise ValueError("LIO-SAM library must not be a symbolic link")
    library.mkdir(exist_ok=True)
    destination = library / run.name
    destination.mkdir(exist_ok=False)
    output = destination / "GlobalMap.pcd"
    digest = hashlib.sha256()
    with source.open("rb") as reader, output.open("xb") as writer:
        for block in iter(lambda: reader.read(1024 * 1024), b""):
            writer.write(block)
            digest.update(block)
    shutil.copystat(source, output)
    source_time = datetime.fromtimestamp(source.stat().st_mtime).astimezone().isoformat(timespec="seconds")
    manifest = {
        "schema": "d1max.lio_sam_artifact.v1", "status": "complete",
        "output_file": "GlobalMap.pcd", "sensor_mode": sensor_mode,
        "source_run": str(run), "source_pcd": str(source),
        "source_bag": original.get("bag"), "source_manifest": str(source_manifest),
        "input": input_description, "mode": original.get("mode"),
        "loop_enabled": bool(original.get("loop_enabled")),
        "upstream_commit": original.get("upstream_commit"),
        "created_at": source_time, "imported_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "size_bytes": output.stat().st_size, "sha256": digest.hexdigest(),
        "note": "实验结果；六轴中心 IMU 适配。原生角点/平面特征地图，非原始全量点云。定位精度尚未验收。",
    }
    with (destination / MANIFEST).open("x", encoding="utf-8") as writer:
        json.dump(manifest, writer, ensure_ascii=False, indent=2)
        writer.write("\n")
    return {"path": str(output), "relative_path": str(output.relative_to(maps_root)),
            "sha256": manifest["sha256"], "sensor_mode": sensor_mode}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--maps-root", type=Path, required=True)
    parser.add_argument("--sensor-mode", choices=("front", "dual"), required=True)
    args = parser.parse_args()
    print(json.dumps(publish_run(args.run, args.maps_root, args.sensor_mode), ensure_ascii=False, indent=2))
