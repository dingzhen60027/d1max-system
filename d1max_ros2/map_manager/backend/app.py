from __future__ import annotations

import csv
import hashlib
import json
import math
import mmap
import os
import re
import shutil
import struct
import threading
import uuid
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .pointcloud_pipeline import (
    PipelineCancelled,
    PipelineError,
    list_pipeline_configs,
    resolve_pipeline,
    run_pipeline,
    save_user_pipeline,
    validate_pipeline,
    write_pipeline,
)
from .runtime import RuntimeManager


APP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(os.environ.get("D1MAX_PROJECT_ROOT", APP_ROOT.parents[1])).resolve()
NAV_ROOT = Path(os.environ.get("D1MAX_NAV_ROOT", "/home/dndx/d1max_nav_ws")).resolve()
MAPS_ROOT = Path(os.environ.get("D1MAX_MAPS_ROOT", NAV_ROOT / "maps")).resolve()
DATA_ROOT = Path(os.environ.get("D1MAX_MAP_MANAGER_DATA", APP_ROOT / "data")).resolve()
CACHE_ROOT = DATA_ROOT / "cloud_cache"
PROCESSED_ROOT = Path(os.environ.get("D1MAX_PROCESSED_ROOT", DATA_ROOT / "processed")).resolve()
STATE_PATH = DATA_ROOT / "state.json"
RUNTIME_PATH = DATA_ROOT / "runtime.json"
PIPELINE_BUILTIN_ROOT = APP_ROOT / "config" / "pipelines"
PIPELINE_USER_ROOT = DATA_ROOT / "processing_configs"
SUPPORTED_SUFFIXES = {".pcd", ".ply"}
ID_RE = re.compile(r"^[a-f0-9]{16}$")

DATA_ROOT.mkdir(parents=True, exist_ok=True)
CACHE_ROOT.mkdir(parents=True, exist_ok=True)
PROCESSED_ROOT.mkdir(parents=True, exist_ok=True)
PIPELINE_USER_ROOT.mkdir(parents=True, exist_ok=True)


class RuntimeRequest(BaseModel):
    algorithm: Literal["faster_lio", "fastlio2", "faster_lio_pgo"]


class ActiveRequest(BaseModel):
    map_id: str


class MetadataRequest(BaseModel):
    name: str | None = Field(default=None, max_length=100)
    note: str | None = Field(default=None, max_length=500)
    archived: bool | None = None


class ProcessingRequest(BaseModel):
    source_id: str
    name: str = Field(min_length=1, max_length=100)
    note: str = Field(default="", max_length=500)
    config_id: str = Field(min_length=2, max_length=64)
    pipeline: dict[str, Any]


class ArchiveDeleteRequest(BaseModel):
    item_ids: list[str] = Field(default_factory=list, max_length=500)
    delete_all: bool = False


runtime_manager = RuntimeManager(NAV_ROOT, RUNTIME_PATH)
preview_lock = threading.Lock()
archive_delete_lock = threading.Lock()
processing_lock = threading.Lock()
processing_cancel_event = threading.Event()
processing_job: dict[str, Any] = {
    "running": False,
    "status": "idle",
    "progress": 0,
    "stage": "idle",
    "logs": [],
    "error": None,
    "result_id": None,
}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def human_size(size: int) -> str:
    value = float(size)
    for unit in ("B", "KB", "MB", "GB"):
        if value < 1024 or unit == "GB":
            return f"{int(value)} B" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024
    return f"{value:.1f} GB"


def read_state() -> dict[str, Any]:
    default = {"active_id": None, "items": {}}
    if not STATE_PATH.is_file():
        return default
    try:
        value = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        return {**default, **value} if isinstance(value, dict) else default
    except (OSError, json.JSONDecodeError):
        return default


def write_state(value: dict[str, Any]) -> None:
    temporary = STATE_PATH.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, STATE_PATH)


def model_dict(value: BaseModel) -> dict[str, Any]:
    return value.model_dump() if hasattr(value, "model_dump") else value.dict()


def processing_snapshot() -> dict[str, Any]:
    with processing_lock:
        return json.loads(json.dumps(processing_job, ensure_ascii=False))


def set_processing_job(**values: Any) -> None:
    with processing_lock:
        processing_job.update(values)


def log_processing(message: str, *, progress: int | None = None, stage: str | None = None) -> None:
    with processing_lock:
        processing_job.setdefault("logs", []).append(message)
        processing_job["logs"] = processing_job["logs"][-80:]
        if progress is not None:
            processing_job["progress"] = progress
        if stage is not None:
            processing_job["stage"] = stage


def stable_id(relative_path: str) -> str:
    return hashlib.sha1(relative_path.encode("utf-8")).hexdigest()[:16]


def parse_timestamp(text: str) -> str | None:
    matches = re.findall(r"20\d{6}[_-]?\d{6}", text)
    if not matches:
        return None
    compact = matches[-1].replace("-", "_")
    try:
        value = datetime.strptime(compact, "%Y%m%d_%H%M%S").astimezone()
    except ValueError:
        return None
    return value.isoformat(timespec="seconds")


def pcd_header(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"format": "pcd", "points": None, "fields": [], "data": None}
    try:
        with path.open("rb") as stream:
            for _ in range(80):
                raw = stream.readline()
                if not raw:
                    break
                line = raw.decode("ascii", errors="replace").strip()
                if not line or line.startswith("#"):
                    continue
                key, _, value = line.partition(" ")
                key = key.upper()
                values = value.split()
                if key == "FIELDS":
                    result["fields"] = values
                elif key == "POINTS" and values:
                    result["points"] = int(values[0])
                elif key == "WIDTH" and result["points"] is None and values:
                    result["points"] = int(values[0])
                elif key == "DATA" and values:
                    result["data"] = values[0].lower()
                    result["header_bytes"] = stream.tell()
                    break
    except (OSError, ValueError):
        result["error"] = "PCD 头无法读取"
    return result


def ply_header(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"format": "ply", "points": None, "fields": []}
    try:
        with path.open("rb") as stream:
            for _ in range(100):
                line = stream.readline().decode("ascii", errors="replace").strip()
                if line.startswith("format "):
                    result["data"] = line.split()[1]
                elif line.startswith("element vertex "):
                    result["points"] = int(line.split()[2])
                elif line.startswith("property "):
                    result["fields"].append(line.split()[-1])
                elif line == "end_header":
                    break
    except (OSError, ValueError):
        result["error"] = "PLY 头无法读取"
    return result


def loop_summary(run_root: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    path = run_root / "sc_pgo" / "loop_events.csv"
    if not path.is_file():
        return counts
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
            for row in csv.DictReader(stream):
                event = str(row.get("event") or "").strip()
                if event:
                    counts[event] = counts.get(event, 0) + 1
    except OSError:
        return {}
    return counts


def friendly_name(path: Path, role: str, run_id: str) -> str:
    stamp = parse_timestamp(path.name) or parse_timestamp(run_id)
    time_label = stamp[5:16].replace("T", " ") if stamp else run_id
    labels = {
        "optimized": "SC-PGO 优化地图",
        "frontend": "Faster-LIO 前端地图",
        "fastlio2": "FAST-LIO2 地图",
        "planning": "PCT 规划点云",
        "legacy": "历史点云",
    }
    return f"{labels.get(role, '3D 点云')} · {time_label}"


def classify(path: Path) -> dict[str, Any]:
    relative = path.relative_to(MAPS_ROOT)
    parts = relative.parts
    first = parts[0] if parts else ""
    name = path.name.lower()
    category = "maps"
    family = "历史 / 未分类"
    role = "legacy"
    run_id = first
    loop_counts: dict[str, int] = {}
    pair_key = None

    if first == "runs" and len(parts) >= 3:
        run_id = parts[1]
        family = "Faster-LIO + SC-PGO"
        pair_key = run_id
        run_root = MAPS_ROOT / "runs" / run_id
        loop_counts = loop_summary(run_root)
        if "sc_pgo" in parts and "optimized" in name:
            role = "optimized"
        elif name.startswith("d1max_map") or "faster_lio_map" in name:
            role = "frontend"
        else:
            role = "legacy"
    elif first.startswith("faster_lio_pgo"):
        family = "Faster-LIO + SC-PGO"
        pair_key = first
        loop_counts = loop_summary(MAPS_ROOT / first)
        role = "optimized" if "optimized" in name else "frontend"
    elif first.startswith("fastlio2"):
        family = "FAST-LIO2"
        role = "fastlio2"
    elif first.startswith("pct_"):
        family = "PCT 3D 规划"
        category = "planning"
        role = "planning"
    elif len(parts) == 1:
        family = "历史根目录"

    return {
        "category": category,
        "family": family,
        "role": role,
        "run_id": run_id,
        "pair_key": pair_key,
        "loop_counts": loop_counts,
    }


def iter_cloud_files() -> list[Path]:
    result: list[Path] = []
    if not MAPS_ROOT.is_dir():
        return result
    for directory, names, files in os.walk(MAPS_ROOT, followlinks=False):
        current = Path(directory)
        names[:] = [name for name in names if name not in {"workspace", ".cache"}]
        for filename in files:
            path = current / filename
            if path.suffix.lower() in SUPPORTED_SUFFIXES and not path.is_symlink():
                result.append(path)
    return result


def scan_processed_maps(custom: dict[str, Any]) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    if not PROCESSED_ROOT.is_dir():
        return items
    for manifest_path in PROCESSED_ROOT.glob("*/manifest.json"):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if manifest.get("status") != "complete":
            continue
        path = manifest_path.parent / str(manifest.get("output_file") or "processed_map.pcd")
        if not path.is_file() or path.suffix.lower() not in SUPPORTED_SUFFIXES:
            continue
        identity = f"processed/{manifest_path.parent.name}/{path.name}"
        item_id = stable_id(identity)
        override = custom.get(item_id, {}) if isinstance(custom.get(item_id), dict) else {}
        stat = path.stat()
        header = pcd_header(path) if path.suffix.lower() == ".pcd" else ply_header(path)
        points = header.get("points")
        input_points = manifest.get("input_points")
        removed_percent = None
        if isinstance(input_points, int) and input_points > 0 and isinstance(points, int):
            removed_percent = round((input_points - points) * 100 / input_points, 1)
        issues: list[str] = []
        if header.get("error"):
            issues.append(str(header["error"]))
        if removed_percent is not None and removed_percent > 95:
            issues.append("处理后点数减少超过 95%，请复核滤波参数")
        source_fields = manifest.get("source_fields") or []
        missing_fields = [field for field in source_fields if field not in (header.get("fields") or [])]
        if missing_fields:
            issues.append(f"处理结果未写入扩展字段：{', '.join(missing_fields)}")
        created_at = manifest.get("created_at") or datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds")
        items.append({
            "id": item_id,
            "name": override.get("name") or manifest.get("name") or f"处理结果 · {format_timestamp_label(created_at)}",
            "note": override.get("note", manifest.get("note", "")),
            "archived": bool(override.get("archived", False)),
            "relative_path": identity,
            "path": str(path),
            "extension": path.suffix.lower()[1:],
            "size": stat.st_size,
            "size_human": human_size(stat.st_size),
            "modified_at": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds"),
            "created_at": created_at,
            "points": points,
            "fields": header.get("fields", []),
            "storage": header.get("data"),
            "complete": not bool(header.get("error")),
            "issues": issues,
            "accepted_loops": 0,
            "rejected_loops": 0,
            "cloud_url": f"/api/maps/{item_id}/cloud.ply",
            "category": "processed",
            "family": "点云处理",
            "role": "processed",
            "run_id": manifest_path.parent.name,
            "pair_key": None,
            "loop_counts": {},
            "source_id": manifest.get("source_id"),
            "source_path": manifest.get("source_path"),
            "source_name": manifest.get("source_name"),
            "source_fields": source_fields,
            "parameters": manifest.get("parameters", {}),
            "stage_counts": manifest.get("stage_counts", []),
            "input_points": input_points,
            "removed_percent": removed_percent,
            "profile_name": manifest.get("config_name") or manifest.get("profile_name"),
            "config_id": manifest.get("config_id"),
            "config_name": manifest.get("config_name"),
            "pipeline": manifest.get("pipeline"),
            "pipeline_path": str(manifest_path.parent / str(manifest.get("pipeline_file") or "pipeline.yaml")),
        })
    return items


def format_timestamp_label(value: str) -> str:
    try:
        stamp = datetime.fromisoformat(value)
        return stamp.strftime("%m-%d %H:%M")
    except (TypeError, ValueError):
        return "未命名"


def scan_maps() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    state = read_state()
    custom = state.get("items", {}) if isinstance(state.get("items"), dict) else {}
    paths = iter_cloud_files()
    pcd_stems = {str(path.with_suffix("")) for path in paths if path.suffix.lower() == ".pcd"}
    items: list[dict[str, Any]] = []
    for path in paths:
        relative = str(path.relative_to(MAPS_ROOT))
        # Explicit PLY siblings created only for CloudCompare are previews of
        # their PCD pair, not independent map versions.
        if path.suffix.lower() == ".ply" and (
            name_is_pair_preview(path.name) or str(path.with_suffix("")) in pcd_stems
        ):
            continue
        stat = path.stat()
        header = pcd_header(path) if path.suffix.lower() == ".pcd" else ply_header(path)
        classification = classify(path)
        item_id = stable_id(relative)
        override = custom.get(item_id, {}) if isinstance(custom.get(item_id), dict) else {}
        points = header.get("points")
        issues: list[str] = []
        if header.get("error"):
            issues.append(str(header["error"]))
        if points is not None and points < 100_000 and classification["category"] == "maps":
            issues.append("点数偏少，可能是中断测试或局部片段")
        accepted = classification["loop_counts"].get("accepted", 0)
        rejected = sum(
            count for event, count in classification["loop_counts"].items()
            if event.startswith("reject_")
        )
        if classification["role"] == "optimized" and accepted == 0:
            issues.append("SC-PGO 未接受回环约束，优化图不代表闭环成功")
        created_at = parse_timestamp(path.name) or datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds")
        items.append({
            "id": item_id,
            "name": override.get("name") or friendly_name(path, classification["role"], classification["run_id"]),
            "note": override.get("note", ""),
            "archived": bool(override.get("archived", False)),
            "relative_path": relative,
            "path": str(path),
            "extension": path.suffix.lower()[1:],
            "size": stat.st_size,
            "size_human": human_size(stat.st_size),
            "modified_at": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds"),
            "created_at": created_at,
            "points": points,
            "fields": header.get("fields", []),
            "storage": header.get("data"),
            "complete": not bool(header.get("error")),
            "issues": issues,
            "accepted_loops": accepted,
            "rejected_loops": rejected,
            "cloud_url": f"/api/maps/{item_id}/cloud.ply",
            **classification,
        })

    items.extend(scan_processed_maps(custom))

    # In a run with multiple front-end saves, the newest capture is the final
    # one; earlier copies remain visible but are marked as snapshots.
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for item in items:
        if item["pair_key"] and item["role"] == "frontend":
            grouped.setdefault((item["pair_key"], item["role"]), []).append(item)
    for group in grouped.values():
        group.sort(key=lambda item: item["modified_at"], reverse=True)
        for snapshot in group[1:]:
            snapshot["issues"].append("同次运行的较早保存快照")
            snapshot["snapshot"] = True

    verified = [
        item for item in items
        if item["role"] == "optimized" and item["accepted_loops"] > 0 and not item["archived"]
    ]
    recommended_id = max(verified, key=lambda item: item["modified_at"])["id"] if verified else None
    latest_candidates = [
        item for item in items
        if item["category"] == "maps" and not item.get("snapshot") and not item["archived"]
    ]
    latest_id = max(latest_candidates, key=lambda item: item["modified_at"])["id"] if latest_candidates else None
    for item in items:
        item["recommended"] = item["id"] == recommended_id
        item["latest"] = item["id"] == latest_id
        item["active"] = item["id"] == state.get("active_id")

    items.sort(key=lambda item: item["modified_at"], reverse=True)
    failures = scan_incomplete_runs(items)
    return items, failures


def name_is_pair_preview(filename: str) -> bool:
    return filename in {"faster_lio_map.ply", "sc_pgo_optimized_map.ply"}


def scan_incomplete_runs(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not MAPS_ROOT.is_dir():
        return []
    populated = {item["run_id"] for item in items if item["category"] == "maps"}
    candidates: list[Path] = []
    candidates.extend(path for path in MAPS_ROOT.glob("fastlio2_*") if path.is_dir())
    run_root = MAPS_ROOT / "runs"
    if run_root.is_dir():
        candidates.extend(path for path in run_root.iterdir() if path.is_dir())
    failures = []
    for path in sorted(candidates, reverse=True):
        if path.name in populated:
            continue
        files = [entry for entry in path.rglob("*") if entry.is_file()]
        total = sum(entry.stat().st_size for entry in files)
        failures.append({
            "id": stable_id(str(path.relative_to(MAPS_ROOT)) + "/"),
            "name": path.name,
            "path": str(path),
            "modified_at": datetime.fromtimestamp(path.stat().st_mtime).astimezone().isoformat(timespec="seconds"),
            "size": total,
            "size_human": human_size(total),
            "reason": "目录内没有可用 PCD/PLY" if total == 0 else "只有日志或中间文件，没有地图点云",
        })
    return failures


def find_item(item_id: str) -> dict[str, Any]:
    if not ID_RE.fullmatch(item_id):
        raise HTTPException(status_code=400, detail="地图 ID 非法")
    items, _ = scan_maps()
    item = next((value for value in items if value["id"] == item_id), None)
    if item is None:
        raise HTTPException(status_code=404, detail="地图不存在或文件已移动")
    return item


def comparison_for(item: dict[str, Any], items: list[dict[str, Any]]) -> dict[str, Any] | None:
    if item.get("category") == "processed":
        return next((value for value in items if value["id"] == item.get("source_id")), None)
    pair_key = item.get("pair_key")
    if not pair_key:
        return None
    wanted_role = "frontend" if item["role"] == "optimized" else "optimized"
    candidates = [
        value for value in items
        if value.get("pair_key") == pair_key and value["role"] == wanted_role and not value.get("snapshot")
    ]
    return max(candidates, key=lambda value: value["modified_at"]) if candidates else None


def convert_pcd_to_ply(source: Path, destination: Path, max_points: int = 450_000) -> None:
    metadata: dict[str, list[str]] = {}
    data_offset = 0
    with source.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline()
            if not raw:
                raise ValueError("PCD 缺少 DATA 头")
            line = raw.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            metadata[key.upper()] = value.split()
            if key.upper() == "DATA":
                data_offset = stream.tell()
                break

    fields = metadata.get("FIELDS") or metadata.get("FIELD") or []
    sizes = [int(value) for value in metadata.get("SIZE", [])]
    types = metadata.get("TYPE", [])
    counts = [int(value) for value in metadata.get("COUNT", ["1"] * len(fields))]
    points = int((metadata.get("POINTS") or metadata.get("WIDTH") or ["0"])[0])
    storage = (metadata.get("DATA") or [""])[0].lower()
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD 不包含 x/y/z 字段")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD 字段定义不完整")
    if storage == "binary_compressed":
        raise ValueError("暂不支持 binary_compressed PCD 预览")

    offsets: dict[str, tuple[int, str]] = {}
    cursor = 0
    format_map = {
        ("F", 4): "f", ("F", 8): "d",
        ("I", 1): "b", ("I", 2): "h", ("I", 4): "i",
        ("U", 1): "B", ("U", 2): "H", ("U", 4): "I",
    }
    for field, size, value_type, count in zip(fields, sizes, types, counts):
        code = format_map.get((value_type.upper(), size))
        if code:
            offsets[field] = (cursor, "<" + code)
        cursor += size * count
    if not all(axis in offsets for axis in ("x", "y", "z")):
        raise ValueError("PCD x/y/z 字段类型不受支持")

    stride = max(1, math.ceil(points / max_points))
    packed = bytearray()
    if storage == "binary":
        expected = data_offset + points * cursor
        if source.stat().st_size < expected:
            raise ValueError("PCD 二进制数据长度不足")
        with source.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as mapped:
            for index in range(0, points, stride):
                base = data_offset + index * cursor
                xyz = [struct.unpack_from(offsets[axis][1], mapped, base + offsets[axis][0])[0] for axis in ("x", "y", "z")]
                if all(math.isfinite(float(value)) for value in xyz):
                    packed.extend(struct.pack("<fff", *xyz))
    elif storage == "ascii":
        field_indexes = [fields.index(axis) for axis in ("x", "y", "z")]
        with source.open("rb") as stream:
            stream.seek(data_offset)
            for index, raw in enumerate(stream):
                if index % stride:
                    continue
                values = raw.split()
                try:
                    xyz = [float(values[field_index]) for field_index in field_indexes]
                except (IndexError, ValueError):
                    continue
                if all(math.isfinite(value) for value in xyz):
                    packed.extend(struct.pack("<fff", *xyz))
    else:
        raise ValueError(f"不支持的 PCD DATA 类型: {storage or 'unknown'}")

    count = len(packed) // 12
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {count}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    ).encode("ascii")
    temporary = destination.with_suffix(".tmp")
    with temporary.open("wb") as stream:
        stream.write(header)
        stream.write(packed)
    os.replace(temporary, destination)


def cache_path(item: dict[str, Any]) -> Path:
    source = Path(item["path"])
    identity = f"{item['relative_path']}:{source.stat().st_size}:{source.stat().st_mtime_ns}"
    suffix = hashlib.sha1(identity.encode("utf-8")).hexdigest()[:12]
    return CACHE_ROOT / f"{item['id']}-{suffix}.ply"


def path_is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def path_disk_usage(path: Path) -> int:
    """Count bytes about to be unlinked without following symlinks."""
    try:
        if path.is_symlink() or path.is_file():
            return path.lstat().st_size
        if not path.is_dir():
            return 0
        return sum(
            entry.lstat().st_size
            for entry in path.rglob("*")
            if entry.is_symlink() or entry.is_file()
        )
    except OSError:
        return 0


def prepare_archive_deletion(item: dict[str, Any]) -> dict[str, Any]:
    """Resolve one archived item to the narrowest safe on-disk target."""
    source = Path(item["path"])
    if source.is_symlink() or not source.is_file():
        raise HTTPException(status_code=409, detail=f"文件已移动或不是普通文件：{item['name']}")

    try:
        source_resolved = source.resolve(strict=True)
    except OSError as exc:
        raise HTTPException(status_code=409, detail=f"无法确认文件路径：{item['name']}") from exc

    if item["category"] == "processed":
        processed_root = PROCESSED_ROOT.resolve()
        run_id = str(item.get("run_id") or "")
        output_dir = PROCESSED_ROOT / run_id
        if (
            not run_id
            or output_dir.is_symlink()
            or not output_dir.is_dir()
            or output_dir.resolve().parent != processed_root
            or not path_is_within(source_resolved, output_dir.resolve())
            or not (output_dir / "manifest.json").is_file()
        ):
            raise HTTPException(status_code=409, detail=f"处理结果目录未通过安全校验：{item['name']}")
        target = output_dir
        target_kind = "directory"
    elif item["category"] in {"maps", "planning"}:
        maps_root = MAPS_ROOT.resolve()
        if not path_is_within(source_resolved, maps_root):
            raise HTTPException(status_code=409, detail=f"地图文件不在受管目录中：{item['name']}")
        target = source
        target_kind = "file"
    else:
        raise HTTPException(status_code=409, detail=f"不支持删除此类条目：{item['name']}")

    return {
        "item": item,
        "target": target,
        "target_kind": target_kind,
        "bytes": path_disk_usage(target),
    }


def write_processing_manifest(directory: Path, value: dict[str, Any]) -> None:
    path = directory / "manifest.json"
    temporary = directory / "manifest.tmp"
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def load_processing_cloud(path: Path) -> tuple[Any, dict[str, float] | None]:
    """Read XYZ plus intensity from an uncompressed PCD into an Open3D cloud.

    Intensity is carried through Open3D filters as a normalized grayscale color,
    then restored when the processed PCD is written. This keeps the D1 Max's
    x/y/z/intensity schema instead of silently dropping the fourth field.
    """
    import numpy as np
    import open3d as o3d

    metadata: dict[str, list[str]] = {}
    data_offset = 0
    with path.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline()
            if not raw:
                raise RuntimeError("PCD 缺少 DATA 头")
            line = raw.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            metadata[key.upper()] = value.split()
            if key.upper() == "DATA":
                data_offset = stream.tell()
                break

    fields = metadata.get("FIELDS") or metadata.get("FIELD") or []
    sizes = [int(value) for value in metadata.get("SIZE", [])]
    types = metadata.get("TYPE", [])
    counts = [int(value) for value in metadata.get("COUNT", ["1"] * len(fields))]
    points = int((metadata.get("POINTS") or metadata.get("WIDTH") or ["0"])[0])
    storage = (metadata.get("DATA") or [""])[0].lower()
    if not {"x", "y", "z"}.issubset(fields) or not points:
        raise RuntimeError("PCD 不包含有效的 x/y/z 点")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise RuntimeError("PCD 字段定义不完整")

    offsets: dict[str, tuple[int, str]] = {}
    cursor = 0
    numpy_types = {
        ("F", 4): "<f4", ("F", 8): "<f8",
        ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4",
        ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
    }
    for field, size, value_type, count in zip(fields, sizes, types, counts):
        dtype = numpy_types.get((value_type.upper(), size))
        if dtype and count == 1:
            offsets[field] = (cursor, dtype)
        cursor += size * count
    if not all(axis in offsets for axis in ("x", "y", "z")):
        raise RuntimeError("PCD x/y/z 字段类型不受支持")

    wanted = ["x", "y", "z"] + (["intensity"] if "intensity" in offsets else [])
    if storage == "binary":
        expected = data_offset + points * cursor
        if path.stat().st_size < expected:
            raise RuntimeError("PCD 二进制数据长度不足")
        values: dict[str, Any] = {}
        with path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as mapped:
            for field in wanted:
                offset, dtype = offsets[field]
                values[field] = np.ndarray(
                    (points,), dtype=np.dtype(dtype), buffer=mapped,
                    offset=data_offset + offset, strides=(cursor,),
                ).astype(np.float64, copy=True)
    elif storage == "ascii":
        with path.open("rb") as stream:
            stream.seek(data_offset)
            table = np.loadtxt(stream, ndmin=2)
        values = {field: table[:, fields.index(field)].astype(np.float64, copy=False) for field in wanted}
    else:
        raise RuntimeError(f"点云处理暂不支持 DATA {storage or 'unknown'} PCD")

    xyz = np.column_stack([values[axis] for axis in ("x", "y", "z")])
    mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[mask]
    if not len(xyz):
        raise RuntimeError("PCD 没有有限坐标点")
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    intensity_meta = None
    if "intensity" in values:
        intensity = values["intensity"][mask]
        intensity = np.nan_to_num(intensity, nan=0.0, posinf=0.0, neginf=0.0)
        minimum = float(intensity.min())
        maximum = float(intensity.max())
        span = maximum - minimum
        normalized = (intensity - minimum) / span if span > 0 else np.zeros_like(intensity)
        cloud.colors = o3d.utility.Vector3dVector(np.repeat(normalized[:, None], 3, axis=1))
        intensity_meta = {"minimum": minimum, "span": span}
    return cloud, intensity_meta


def write_processed_pcd(path: Path, cloud: Any, intensity_meta: dict[str, float] | None) -> list[str]:
    import numpy as np

    xyz = np.asarray(cloud.points, dtype=np.float64)
    fields = ["x", "y", "z"]
    columns = [xyz]
    if intensity_meta is not None and cloud.has_colors():
        normalized = np.asarray(cloud.colors, dtype=np.float64)[:, 0]
        intensity = normalized * intensity_meta["span"] + intensity_meta["minimum"]
        columns.append(intensity[:, None])
        fields.append("intensity")
    matrix = np.column_stack(columns).astype("<f4", copy=False)
    count = len(matrix)
    field_count = len(fields)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {' '.join(['4'] * field_count)}\n"
        f"TYPE {' '.join(['F'] * field_count)}\n"
        f"COUNT {' '.join(['1'] * field_count)}\n"
        f"WIDTH {count}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\nDATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(matrix.tobytes(order="C"))
    return fields


def process_point_cloud_worker(
    request: dict[str, Any], source_item: dict[str, Any], output_dir: Path, result_id: str,
) -> None:
    output = output_dir / "processed_map.pcd"
    resolved_pipeline = resolve_pipeline(request["pipeline"], Path(source_item["path"]), output)
    manifest: dict[str, Any] = {
        "schema": 1,
        "status": "running",
        "id": result_id,
        "name": request["name"],
        "note": request.get("note", ""),
        "created_at": now_iso(),
        "source_id": source_item["id"],
        "source_name": source_item["name"],
        "source_path": source_item["path"],
        "source_fields": source_item.get("fields", []),
        "config_id": resolved_pipeline["id"],
        "config_name": resolved_pipeline["name"],
        "pipeline_file": "pipeline.yaml",
        "pipeline": resolved_pipeline,
        "output_file": "processed_map.pcd",
        "stage_counts": [],
    }
    try:
        output_dir.mkdir(parents=True, exist_ok=False)
        write_pipeline(output_dir / "pipeline.yaml", resolved_pipeline)
        write_processing_manifest(output_dir, manifest)
        result = run_pipeline(
            output_dir / "pipeline.yaml",
            on_event=lambda message, progress, stage: log_processing(
                message, progress=progress, stage=stage,
            ),
            cancelled=processing_cancel_event.is_set,
        )
        manifest.update({
            "status": "complete",
            "completed_at": now_iso(),
            **result,
        })
        write_processing_manifest(output_dir, manifest)
        set_processing_job(
            running=False, status="complete", completed_at=now_iso(), error=None,
            result_id=result_id, output_path=str(output),
        )
    except PipelineCancelled:
        manifest.update({"status": "cancelled", "completed_at": now_iso()})
        if output_dir.is_dir():
            write_processing_manifest(output_dir, manifest)
        log_processing("点云处理已取消；原始 PCD 未改动", stage="cancelled")
        set_processing_job(running=False, status="cancelled", completed_at=now_iso(), error=None)
    except Exception as exc:
        manifest.update({"status": "failed", "completed_at": now_iso(), "error": str(exc)})
        if output_dir.is_dir():
            write_processing_manifest(output_dir, manifest)
        log_processing(str(exc), stage="failed")
        set_processing_job(running=False, status="failed", completed_at=now_iso(), error=str(exc))


@asynccontextmanager
async def lifespan(_: FastAPI):
    runtime_manager.recover_stale_state()
    yield
    runtime_manager.close()


app = FastAPI(title="D1 Max 3D Map Workspace", version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {
        "ok": MAPS_ROOT.is_dir(),
        "project_root": str(PROJECT_ROOT),
        "nav_root": str(NAV_ROOT),
        "maps_root": str(MAPS_ROOT),
        "processed_root": str(PROCESSED_ROOT),
        "rmw": "rmw_zenoh_cpp",
    }


@app.get("/api/overview")
def overview() -> dict[str, Any]:
    items, failures = scan_maps()
    active = next((item for item in items if item["active"]), None)
    recommended = next((item for item in items if item["recommended"]), None)
    latest = next((item for item in items if item["latest"]), None)
    comparisons = {
        item["id"]: peer["id"]
        for item in items
        if (peer := comparison_for(item, items)) is not None
    }
    return {
        "workspace": {
            "project_root": str(PROJECT_ROOT),
            "nav_root": str(NAV_ROOT),
            "maps_root": str(MAPS_ROOT),
            "mode": "3d-only",
        },
        "summary": {
            "map_count": sum(item["category"] == "maps" and not item["archived"] for item in items),
            "processed_count": sum(item["category"] == "processed" and not item["archived"] for item in items),
            "planning_count": sum(item["category"] == "planning" and not item["archived"] for item in items),
            "archived_count": sum(item["archived"] for item in items),
            "failure_count": len(failures),
            "total_size": sum(item["size"] for item in items),
            "total_size_human": human_size(sum(item["size"] for item in items)),
        },
        "active": active,
        "recommended": recommended,
        "latest": latest,
        "items": items,
        "failures": failures,
        "comparisons": comparisons,
        "runtime": runtime_manager.snapshot(),
        "processing_job": processing_snapshot(),
        "processing_configs": list_pipeline_configs(PIPELINE_BUILTIN_ROOT, PIPELINE_USER_ROOT),
        "algorithms": [
            {"id": "faster_lio", "name": "Faster-LIO", "description": "双 Airy96 实时建图"},
            {"id": "fastlio2", "name": "FAST-LIO2", "description": "双雷达无回环建图"},
            {"id": "faster_lio_pgo", "name": "Faster-LIO + SC-PGO", "description": "建图并检测回环"},
        ],
    }


@app.get("/api/maps/{item_id}/cloud.ply")
def cloud(item_id: str) -> FileResponse:
    item = find_item(item_id)
    source = Path(item["path"])
    if source.suffix.lower() == ".ply":
        return FileResponse(source, media_type="application/octet-stream", filename=source.name)
    destination = cache_path(item)
    with preview_lock:
        if not destination.is_file():
            try:
                convert_pcd_to_ply(source, destination)
            except (OSError, ValueError, struct.error) as exc:
                destination.unlink(missing_ok=True)
                raise HTTPException(status_code=422, detail=f"3D 预览生成失败: {exc}") from exc
    return FileResponse(destination, media_type="application/octet-stream", filename=f"{item_id}.ply")


@app.post("/api/active")
def activate(request: ActiveRequest) -> dict[str, Any]:
    item = find_item(request.map_id)
    if item["category"] not in {"maps", "processed"}:
        raise HTTPException(status_code=409, detail="规划派生点云不能标为当前建图结果")
    state = read_state()
    state["active_id"] = item["id"]
    write_state(state)
    return {"active_id": item["id"]}


@app.patch("/api/maps/{item_id}")
def update_metadata(item_id: str, request: MetadataRequest) -> dict[str, Any]:
    item = find_item(item_id)
    state = read_state()
    metadata = state.setdefault("items", {}).setdefault(item_id, {})
    if request.name is not None:
        name = request.name.strip()
        if not name:
            raise HTTPException(status_code=400, detail="名称不能为空")
        metadata["name"] = name
    if request.note is not None:
        metadata["note"] = request.note.strip()
    if request.archived is not None:
        metadata["archived"] = request.archived
        if request.archived and state.get("active_id") == item_id:
            state["active_id"] = None
    write_state(state)
    return {"id": item_id, "metadata": metadata}


@app.post("/api/archive/delete")
def delete_archived(request: ArchiveDeleteRequest) -> dict[str, Any]:
    """Permanently remove archived point clouds and their generated previews."""
    with archive_delete_lock:
        items, _ = scan_maps()
        archived = {item["id"]: item for item in items if item["archived"]}

        if request.delete_all:
            requested_ids = list(archived)
        else:
            requested_ids = list(dict.fromkeys(request.item_ids))
            if len(requested_ids) != len(request.item_ids):
                raise HTTPException(status_code=400, detail="待删除列表包含重复地图")

        if not requested_ids:
            raise HTTPException(status_code=400, detail="没有可永久删除的归档地图")
        invalid_ids = [item_id for item_id in requested_ids if not ID_RE.fullmatch(item_id)]
        if invalid_ids:
            raise HTTPException(status_code=400, detail="待删除列表包含非法地图 ID")

        known = {item["id"]: item for item in items}
        missing = [item_id for item_id in requested_ids if item_id not in known]
        if missing:
            raise HTTPException(status_code=404, detail="部分地图不存在或文件已移动，请重新扫描")
        not_archived = [item_id for item_id in requested_ids if item_id not in archived]
        if not_archived:
            raise HTTPException(status_code=409, detail="只能永久删除已归档的地图")

        # Validate every target before unlinking the first one. This prevents a
        # mixed request from deleting safe entries before an unsafe path fails.
        plans = [prepare_archive_deletion(archived[item_id]) for item_id in requested_ids]
        cache_files: dict[str, list[Path]] = {
            item_id: [path for path in CACHE_ROOT.glob(f"{item_id}-*.ply") if path.is_file() or path.is_symlink()]
            for item_id in requested_ids
        }
        freed_bytes = sum(plan["bytes"] for plan in plans)
        freed_bytes += sum(path_disk_usage(path) for paths in cache_files.values() for path in paths)

        deleted_ids: list[str] = []
        try:
            # Avoid racing a first-time PCD-to-PLY preview conversion.
            with preview_lock:
                for plan in plans:
                    target = plan["target"]
                    if plan["target_kind"] == "directory":
                        shutil.rmtree(target)
                    else:
                        target.unlink()
                    for cached in cache_files[plan["item"]["id"]]:
                        cached.unlink(missing_ok=True)
                    deleted_ids.append(plan["item"]["id"])
        except OSError as exc:
            state = read_state()
            for item_id in deleted_ids:
                state.setdefault("items", {}).pop(item_id, None)
                if state.get("active_id") == item_id:
                    state["active_id"] = None
            write_state(state)
            raise HTTPException(
                status_code=500,
                detail=f"已删除 {len(deleted_ids)} 项后磁盘清理失败：{exc}",
            ) from exc

        state = read_state()
        if request.delete_all:
            stale_archived_ids = [
                item_id for item_id, metadata in state.get("items", {}).items()
                if isinstance(metadata, dict) and metadata.get("archived")
            ]
        else:
            stale_archived_ids = []
        for item_id in set(deleted_ids + stale_archived_ids):
            state.setdefault("items", {}).pop(item_id, None)
            if state.get("active_id") == item_id:
                state["active_id"] = None
        write_state(state)

        return {
            "deleted_count": len(deleted_ids),
            "deleted_ids": deleted_ids,
            "freed_bytes": freed_bytes,
            "freed_size_human": human_size(freed_bytes),
        }


@app.post("/api/processing/start", status_code=202)
def start_processing(request: ProcessingRequest) -> dict[str, Any]:
    source = find_item(request.source_id)
    if source["category"] != "maps":
        raise HTTPException(status_code=409, detail="请选择建图结果中的原始 PCD 作为处理输入")
    if Path(source["path"]).suffix.lower() != ".pcd":
        raise HTTPException(status_code=409, detail="当前处理链只接受原始 PCD")
    try:
        pipeline = validate_pipeline(request.pipeline)
    except PipelineError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    job_id = uuid.uuid4().hex[:8]
    directory_name = f"{datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')}_{job_id}"
    output_dir = PROCESSED_ROOT / directory_name
    result_id = stable_id(f"processed/{directory_name}/processed_map.pcd")
    payload = model_dict(request)
    payload["name"] = request.name.strip()
    payload["note"] = request.note.strip()
    payload["pipeline"] = pipeline

    with processing_lock:
        if processing_job.get("running"):
            raise HTTPException(status_code=409, detail="已有点云处理任务正在运行")
        processing_cancel_event.clear()
        processing_job.clear()
        processing_job.update({
            "running": True,
            "status": "running",
            "id": job_id,
            "source_id": source["id"],
            "source_name": source["name"],
            "result_id": None,
            "progress": 0,
            "stage": "queued",
            "logs": ["任务已进入处理队列"],
            "error": None,
            "started_at": now_iso(),
        })
    thread = threading.Thread(
        target=process_point_cloud_worker,
        args=(payload, source, output_dir, result_id),
        name=f"point-cloud-processing-{job_id}",
        daemon=True,
    )
    thread.start()
    return processing_snapshot()


@app.post("/api/processing/cancel")
def cancel_processing() -> dict[str, Any]:
    snapshot = processing_snapshot()
    if not snapshot.get("running"):
        raise HTTPException(status_code=409, detail="当前没有运行中的点云处理任务")
    processing_cancel_event.set()
    set_processing_job(status="cancelling", stage="cancelling")
    log_processing("已请求取消，将在当前处理步骤结束后停止")
    return processing_snapshot()


@app.post("/api/processing/configs")
def save_processing_config(request: dict[str, Any]) -> dict[str, Any]:
    try:
        return save_user_pipeline(PIPELINE_USER_ROOT, request)
    except PipelineError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.get("/api/processing/configs/{config_id}/yaml")
def download_processing_config(config_id: str) -> FileResponse:
    config = next(
        (value for value in list_pipeline_configs(PIPELINE_BUILTIN_ROOT, PIPELINE_USER_ROOT)
         if value["id"] == config_id),
        None,
    )
    if config is None:
        raise HTTPException(status_code=404, detail="流水线配置不存在")
    path = Path(config["config_path"])
    return FileResponse(path, media_type="application/yaml", filename=path.name)


@app.post("/api/runtime/start", status_code=202)
def start_runtime(request: RuntimeRequest) -> dict[str, Any]:
    try:
        return runtime_manager.start(request.algorithm)
    except (RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post("/api/runtime/save")
def save_runtime_map() -> dict[str, Any]:
    if runtime_manager.snapshot().get("status") != "running":
        raise HTTPException(status_code=409, detail="Web 当前没有运行中的建图流程")
    try:
        return runtime_manager.save()
    except (OSError, RuntimeError) as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post("/api/runtime/stop")
def stop_runtime() -> dict[str, Any]:
    return runtime_manager.stop()


@app.post("/api/runtime/cleanup")
def cleanup_runtime() -> dict[str, Any]:
    before = runtime_manager.snapshot()
    after = runtime_manager.stop(timeout=8.0)
    return {
        "stopped": before.get("status") in {"running", "stopping"},
        "scope": "web-managed-process-group",
        "runtime": after,
    }


@app.exception_handler(Exception)
async def unhandled_error(_, exc: Exception) -> JSONResponse:
    return JSONResponse(status_code=500, content={"detail": str(exc)})


DIST_ROOT = APP_ROOT / "frontend" / "dist"
if DIST_ROOT.is_dir():
    app.mount("/", StaticFiles(directory=DIST_ROOT, html=True), name="frontend")
else:
    @app.get("/")
    def no_frontend() -> dict[str, str]:
        return {"message": "Frontend not built. Run npm run build in d1max_ros2/map_manager/frontend."}
