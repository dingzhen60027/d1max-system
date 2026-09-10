from __future__ import annotations

import argparse
import copy
import hashlib
import mmap
import os
import re
from pathlib import Path
from typing import Any, Callable

import yaml


SCHEMA = "d1max.pointcloud_pipeline/v1"
CONFIG_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{1,63}$")
EventCallback = Callable[[str, int, str], None]
CancelCallback = Callable[[], bool]

MODULE_SPECS: dict[str, dict[str, Any]] = {
    "crop_z": {
        "label": "Z 高度裁剪",
        "parameters": {"min_z": (-100.0, 100.0), "max_z": (-100.0, 100.0)},
    },
    "voxel_downsample": {
        "label": "体素降采样",
        "parameters": {"voxel_size": (0.02, 1.0)},
    },
    "statistical_outlier": {
        "label": "统计离群点滤波",
        "parameters": {"neighbors": (2, 500), "std_ratio": (0.05, 10.0)},
    },
    "radius_outlier": {
        "label": "半径离群点滤波",
        "parameters": {"radius": (0.01, 5.0), "min_points": (1, 500)},
    },
}


class PipelineError(RuntimeError):
    pass


class PipelineCancelled(PipelineError):
    pass


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9_-]+", "-", value.strip().lower()).strip("-_")
    return slug[:64] or f"pipeline-{hashlib.sha1(value.encode('utf-8')).hexdigest()[:10]}"


def validate_pipeline(value: Any, *, require_paths: bool = False) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PipelineError("流水线配置必须是 YAML 对象")
    config = copy.deepcopy(value)
    if config.get("schema") != SCHEMA:
        raise PipelineError(f"流水线 schema 必须是 {SCHEMA}")
    config_id = str(config.get("id") or "").strip()
    if not CONFIG_ID_RE.fullmatch(config_id):
        raise PipelineError("流水线 id 只能包含小写字母、数字、下划线或连字符")
    name = str(config.get("name") or "").strip()
    if not name or len(name) > 80:
        raise PipelineError("流水线名称不能为空且不能超过 80 个字符")
    config["id"] = config_id
    config["name"] = name
    config["description"] = str(config.get("description") or "").strip()[:500]

    input_config = config.setdefault("input", {})
    output_config = config.setdefault("output", {})
    if not isinstance(input_config, dict) or not isinstance(output_config, dict):
        raise PipelineError("input/output 必须是 YAML 对象")
    input_config["format"] = "pcd"
    input_config.setdefault("required_fields", ["x", "y", "z"])
    output_config["format"] = "pcd"
    output_config["storage"] = "binary"
    output_config.setdefault("preserve_intensity", True)
    if require_paths and (not input_config.get("path") or not output_config.get("path")):
        raise PipelineError("可执行流水线必须提供 input.path 和 output.path")

    modules = config.get("modules")
    if not isinstance(modules, list) or not modules:
        raise PipelineError("流水线至少需要一个处理模块")
    seen: set[str] = set()
    normalized_modules = []
    for index, module in enumerate(modules):
        if not isinstance(module, dict):
            raise PipelineError(f"第 {index + 1} 个模块格式非法")
        module_type = str(module.get("type") or "")
        if module_type not in MODULE_SPECS:
            raise PipelineError(f"不支持的处理模块：{module_type or '未填写'}")
        module_id = str(module.get("id") or module_type).strip()
        if not CONFIG_ID_RE.fullmatch(module_id) or module_id in seen:
            raise PipelineError(f"模块 id 非法或重复：{module_id}")
        seen.add(module_id)
        parameters = module.get("parameters") or {}
        if not isinstance(parameters, dict):
            raise PipelineError(f"模块 {module_id} 的 parameters 必须是对象")
        expected = MODULE_SPECS[module_type]["parameters"]
        normalized_parameters: dict[str, float | int] = {}
        for key, limits in expected.items():
            if key not in parameters:
                raise PipelineError(f"模块 {module_id} 缺少参数 {key}")
            raw = parameters[key]
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                raise PipelineError(f"模块 {module_id} 参数 {key} 必须是数字")
            if not limits[0] <= raw <= limits[1]:
                raise PipelineError(f"模块 {module_id} 参数 {key} 超出范围 {limits}")
            normalized_parameters[key] = raw
        if module_type in {"statistical_outlier", "radius_outlier"}:
            integer_key = "neighbors" if module_type == "statistical_outlier" else "min_points"
            normalized_parameters[integer_key] = int(normalized_parameters[integer_key])
        if module_type == "crop_z" and normalized_parameters["min_z"] >= normalized_parameters["max_z"]:
            raise PipelineError(f"模块 {module_id} 的 min_z 必须小于 max_z")
        normalized_modules.append({
            "id": module_id,
            "type": module_type,
            "enabled": bool(module.get("enabled", True)),
            "parameters": normalized_parameters,
        })
    config["modules"] = normalized_modules
    return config


def read_pipeline(path: Path, *, require_paths: bool = False) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise PipelineError(f"无法读取流水线配置 {path}: {exc}") from exc
    return validate_pipeline(value, require_paths=require_paths)


def write_pipeline(path: Path, config: dict[str, Any]) -> None:
    validated = validate_pipeline(config, require_paths=bool(config.get("input", {}).get("path")))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(validated, allow_unicode=True, sort_keys=False), encoding="utf-8")
    os.replace(temporary, path)


def list_pipeline_configs(builtin_root: Path, user_root: Path) -> list[dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for built_in, root in ((True, builtin_root), (False, user_root)):
        if not root.is_dir():
            continue
        for path in sorted(root.glob("*.yaml")):
            try:
                config = read_pipeline(path)
            except PipelineError:
                continue
            config["built_in"] = built_in
            config["config_path"] = str(path)
            result[config["id"]] = config
    return sorted(result.values(), key=lambda item: (not bool(item.get("recommended")), not item["built_in"], item["name"]))


def save_user_pipeline(user_root: Path, value: dict[str, Any]) -> dict[str, Any]:
    config = copy.deepcopy(value)
    config["id"] = slugify(str(config.get("id") or config.get("name") or "custom-pipeline"))
    config.pop("built_in", None)
    config.pop("config_path", None)
    validated = validate_pipeline(config)
    path = user_root / f"{validated['id']}.yaml"
    write_pipeline(path, validated)
    validated.update({"built_in": False, "config_path": str(path)})
    return validated


def resolve_pipeline(template: dict[str, Any], input_path: Path, output_path: Path) -> dict[str, Any]:
    config = validate_pipeline(template)
    config.pop("built_in", None)
    config.pop("config_path", None)
    config["input"]["path"] = str(input_path.resolve())
    config["output"]["path"] = str(output_path.resolve())
    return validate_pipeline(config, require_paths=True)


def _load_cloud(path: Path) -> tuple[Any, dict[str, float] | None]:
    import numpy as np
    import open3d as o3d

    metadata: dict[str, list[str]] = {}
    data_offset = 0
    with path.open("rb") as stream:
        for _ in range(100):
            raw = stream.readline()
            if not raw:
                raise PipelineError("PCD 缺少 DATA 头")
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
        raise PipelineError("PCD 不包含有效的 x/y/z 点")

    offsets: dict[str, tuple[int, str]] = {}
    cursor = 0
    numpy_types = {
        ("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "<i1", ("I", 2): "<i2",
        ("I", 4): "<i4", ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4",
    }
    for field, size, value_type, count in zip(fields, sizes, types, counts):
        dtype = numpy_types.get((value_type.upper(), size))
        if dtype and count == 1:
            offsets[field] = (cursor, dtype)
        cursor += size * count
    if not all(axis in offsets for axis in ("x", "y", "z")):
        raise PipelineError("PCD x/y/z 字段类型不受支持")

    wanted = ["x", "y", "z"] + (["intensity"] if "intensity" in offsets else [])
    if storage == "binary":
        if path.stat().st_size < data_offset + points * cursor:
            raise PipelineError("PCD 二进制数据长度不足")
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
        raise PipelineError(f"暂不支持 DATA {storage or 'unknown'} PCD")

    xyz = np.column_stack([values[axis] for axis in ("x", "y", "z")])
    mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[mask]
    if not len(xyz):
        raise PipelineError("PCD 没有有限坐标点")
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    intensity_meta = None
    if "intensity" in values:
        intensity = np.nan_to_num(values["intensity"][mask], nan=0.0, posinf=0.0, neginf=0.0)
        minimum, maximum = float(intensity.min()), float(intensity.max())
        span = maximum - minimum
        normalized = (intensity - minimum) / span if span > 0 else np.zeros_like(intensity)
        cloud.colors = o3d.utility.Vector3dVector(np.repeat(normalized[:, None], 3, axis=1))
        intensity_meta = {"minimum": minimum, "span": span}
    return cloud, intensity_meta


def _write_cloud(path: Path, cloud: Any, intensity_meta: dict[str, float] | None) -> list[str]:
    import numpy as np

    xyz = np.asarray(cloud.points, dtype=np.float64)
    fields = ["x", "y", "z"]
    columns = [xyz]
    if intensity_meta is not None and cloud.has_colors():
        normalized = np.asarray(cloud.colors, dtype=np.float64)[:, 0]
        columns.append((normalized * intensity_meta["span"] + intensity_meta["minimum"])[:, None])
        fields.append("intensity")
    matrix = np.column_stack(columns).astype("<f4", copy=False)
    field_count = len(fields)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\nSIZE {' '.join(['4'] * field_count)}\n"
        f"TYPE {' '.join(['F'] * field_count)}\nCOUNT {' '.join(['1'] * field_count)}\n"
        f"WIDTH {len(matrix)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(matrix)}\nDATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(matrix.tobytes(order="C"))
    return fields


def _execute_module(cloud: Any, module: dict[str, Any]) -> Any:
    import numpy as np

    parameters = module["parameters"]
    module_type = module["type"]
    if module_type == "crop_z":
        xyz = np.asarray(cloud.points)
        indexes = np.flatnonzero((xyz[:, 2] >= parameters["min_z"]) & (xyz[:, 2] <= parameters["max_z"]))
        return cloud.select_by_index(indexes.tolist())
    if module_type == "voxel_downsample":
        return cloud.voxel_down_sample(parameters["voxel_size"])
    if module_type == "statistical_outlier":
        return cloud.remove_statistical_outlier(
            nb_neighbors=parameters["neighbors"], std_ratio=parameters["std_ratio"],
        )[0]
    if module_type == "radius_outlier":
        return cloud.remove_radius_outlier(
            nb_points=parameters["min_points"], radius=parameters["radius"],
        )[0]
    raise PipelineError(f"没有模块执行器：{module_type}")


def run_pipeline(
    config_or_path: dict[str, Any] | Path,
    *,
    on_event: EventCallback | None = None,
    cancelled: CancelCallback | None = None,
) -> dict[str, Any]:
    config = read_pipeline(config_or_path, require_paths=True) if isinstance(config_or_path, Path) else validate_pipeline(config_or_path, require_paths=True)
    source = Path(config["input"]["path"])
    output = Path(config["output"]["path"])
    if not source.is_file():
        raise PipelineError(f"输入 PCD 不存在：{source}")
    enabled = [module for module in config["modules"] if module["enabled"]]

    def emit(message: str, progress: int, stage: str) -> None:
        if on_event:
            on_event(message, progress, stage)

    def check_cancelled() -> None:
        if cancelled and cancelled():
            raise PipelineCancelled("点云流水线已取消")

    emit("读取原始 PCD（原文件保持只读）", 5, "read")
    cloud, intensity_meta = _load_cloud(source)
    input_points = len(cloud.points)
    stages = [{"stage": "input", "label": "原始输入", "points": input_points}]
    check_cancelled()
    for index, module in enumerate(enabled):
        spec = MODULE_SPECS[module["type"]]
        progress = 10 + round(index * 75 / max(len(enabled), 1))
        emit(f"执行模块 {index + 1}/{len(enabled)}：{spec['label']}", progress, module["id"])
        cloud = _execute_module(cloud, module)
        stages.append({
            "stage": module["id"], "type": module["type"],
            "label": f"{spec['label']}后", "points": len(cloud.points),
        })
        if cloud.is_empty():
            raise PipelineError(f"模块 {module['id']} 执行后没有剩余点")
        check_cancelled()

    emit("写入新的二进制 PCD", 92, "write")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".tmp.pcd")
    fields = _write_cloud(temporary, cloud, intensity_meta if config["output"].get("preserve_intensity", True) else None)
    os.replace(temporary, output)
    emit("流水线执行完成，原始 PCD 未改动", 100, "complete")
    return {
        "input_points": input_points,
        "output_points": len(cloud.points),
        "output_fields": fields,
        "stage_counts": stages,
        "enabled_modules": [module["id"] for module in enabled],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a modular D1 Max point-cloud pipeline YAML.")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--input", type=Path, help="Override input.path")
    parser.add_argument("--output", type=Path, help="Override output.path")
    args = parser.parse_args()
    config = read_pipeline(args.config)
    if args.input:
        config["input"]["path"] = str(args.input.resolve())
    if args.output:
        config["output"]["path"] = str(args.output.resolve())
    result = run_pipeline(config, on_event=lambda message, progress, _: print(f"[{progress:3d}%] {message}"))
    print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))


if __name__ == "__main__":
    main()
