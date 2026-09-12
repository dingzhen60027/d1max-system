"""D1 Max offline 2D workspace, adapted from go2_nav map/version editing.

No ROS imports, robot commands, Go2 data writes or automatic map activation.
Builds publish atomically; source PCD and parent versions stay immutable.
"""
from __future__ import annotations
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
import uuid
from datetime import datetime
from typing import Literal
import numpy as np
import yaml
from PIL import Image, ImageDraw
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator
from .pointcloud_pipeline import run_pipeline, PipelineCancelled, SCHEMA as POINTCLOUD_SCHEMA

ID_RE = re.compile(r"^grid-[a-f0-9]{24}$")
FILES = {"map.pgm", "map.yaml", "localization.pcd", "manifest.json", "pipeline.yaml"}

class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, str_strip_whitespace=True)

class GridParameters(StrictModel):
    filter_enabled: bool = False
    statistical_mean_k: int = Field(default=20, ge=2, le=500)
    statistical_std_dev_mul: float = Field(default=.5, ge=.05, le=10)
    radius: float = Field(default=.3, ge=.01, le=5)
    radius_min_points: int = Field(default=4, ge=1, le=500)
    z_min: float = Field(default=.4, ge=-100, le=100)
    z_max: float = Field(default=1.5, ge=-100, le=100)
    resolution: float = Field(default=.05, ge=.02, le=.5)
    padding: float = Field(default=.5, ge=0, le=5)
    min_points_per_cell: int = Field(default=1, ge=1, le=100)
    background: Literal["unknown", "free"] = "unknown"
    @model_validator(mode="after")
    def ordered_height(self):
        if self.z_min >= self.z_max:
            raise ValueError("Z 下限必须小于上限")
        return self

class GridBuildRequest(StrictModel):
    source_id: str = Field(pattern=r"^[a-f0-9]{16}$")
    name: str = Field(min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)
    parameters: GridParameters = Field(default_factory=GridParameters)

class EditPoint(StrictModel):
    x: float = Field(ge=0, le=100000)
    y: float = Field(ge=0, le=100000)

class EditOperation(StrictModel):
    mode: Literal["occupied", "free", "unknown"]
    shape: Literal["brush", "line"]
    size: int = Field(ge=1, le=100)
    points: list[EditPoint] = Field(min_length=1, max_length=10000)

class GridEditRequest(StrictModel):
    name: str = Field(min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)
    operations: list[EditOperation] = Field(min_length=1, max_length=500)

class MetadataRequest(StrictModel):
    name: str | None = Field(default=None, min_length=1, max_length=80)
    archived: bool | None = None

class SelectionRequest(StrictModel):
    version_id: str

class DeleteRequest(StrictModel):
    version_ids: list[str] = Field(min_length=1, max_length=200)

class ProfileRequest(StrictModel):
    name: str = Field(min_length=1, max_length=80)
    parameters: GridParameters

def now():
    return datetime.now().astimezone().isoformat(timespec="seconds")

def new_id():
    return "grid-" + uuid.uuid4().hex[:24]

def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)

def rasterize(points, parameters: GridParameters):
    """Go2 XY projection, with inclusive max edge and correct ROS image Y flip."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= parameters.z_min) & (points[:, 2] <= parameters.z_max)]
    if not len(points):
        raise ValueError("当前 Z 高度范围没有点；请核对源 PCD 的坐标高度，不是离地高度")
    resolution = parameters.resolution
    lower = np.floor((points[:, :2].min(axis=0) - parameters.padding) / resolution) * resolution
    extent = (points[:, :2].max(axis=0) + parameters.padding - lower) / resolution
    if not np.isfinite(extent).all() or (extent > 16000000).any():
        raise ValueError("点云坐标范围异常，请先裁剪单层地图")
    width, height = np.floor(extent + 1e-9).astype(int) + 1
    width, height = max(2, int(width)), max(2, int(height))
    if width * height > 16000000 or max(width, height) > 8192:
        raise ValueError("栅格超过 1600 万格或单边 8192 格，请增大分辨率或裁剪点云")
    cells = np.floor((points[:, :2] - lower) / resolution + 1e-9).astype(int)
    counts = np.bincount(cells[:, 0] + cells[:, 1] * width, minlength=width * height).reshape(height, width)
    grid = np.full((height, width), 205 if parameters.background == "unknown" else 254, dtype=np.uint8)
    grid[counts >= parameters.min_points_per_cell] = 0
    # 205/255 must stay UNKNOWN on load: free_thresh=.196, NOT Go2's .25.
    metadata = dict(image="map.pgm", mode="trinary", resolution=resolution,
                    origin=[float(lower[0]), float(lower[1]), 0.0], negate=0,
                    occupied_thresh=.65, free_thresh=.196)
    return Image.fromarray(np.flipud(grid)), metadata, len(points)

def draw_operation(draw, operation: EditOperation, width, height):
    """Port of go2_nav _draw_map_operation; use source-image pixel coordinates."""
    if operation.shape == "line" and len(operation.points) != 2:
        raise ValueError("直线必须有起点和终点")
    coordinates = [(int(round(point.x)), int(round(point.y))) for point in operation.points]
    if any(x < 0 or y < 0 or x >= width or y >= height for x, y in coordinates):
        raise ValueError("笔画超出地图边界")
    fill = {"occupied":0, "free":254, "unknown":205}[operation.mode]
    if len(coordinates) > 1:
        draw.line(coordinates, fill=fill, width=operation.size, joint="curve")
    for x, y in (coordinates if len(coordinates) == 1 else (coordinates[0], coordinates[-1])):
        if operation.size == 1:
            draw.point((x,y), fill=fill)
        else:
            radius = operation.size / 2
            draw.ellipse((x-radius,y-radius,x+radius,y+radius), fill=fill)

class GridWorkspace:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.versions = self.root / "versions"
        self.profiles = self.root / "profiles"
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = None
        for path in (self.versions, self.profiles):
            path.mkdir(parents=True, exist_ok=True)
        self.state_path = self.root / "state.json"
        self.job_path = self.root / "job.json"
        self.job = dict(running=False, status="idle", progress=0, logs=[], error=None, result_id=None)
        self.protected_version = lambda: None

    def recover(self):
        with self.lock:
            if self.job_path.is_file():
                try:
                    previous = json.loads(self.job_path.read_text())
                    if previous.get("running"):
                        previous.update(running=False, status="interrupted", error="上次 Web 退出，任务已中断；不会自动重试")
                    self.job = previous
                    atomic_json(self.job_path, self.job)
                except (OSError, ValueError, AttributeError):
                    pass

    def state(self):
        if not self.state_path.exists():
            return {"selected_id":None, "previous_id":None, "archived":[]}
        try:
            value = json.loads(self.state_path.read_text())
            if not isinstance(value, dict) or not isinstance(value.get("archived"), list):
                raise ValueError("非法状态结构")
            identifiers = [*value["archived"], value.get("selected_id"), value.get("previous_id")]
            if any(identifier is not None and (not isinstance(identifier,str) or not ID_RE.fullmatch(identifier)) for identifier in identifiers):
                raise ValueError("非法状态 ID")
            return value
        except (OSError, ValueError, TypeError) as exc:
            raise HTTPException(409, "2D 状态文件损坏；拒绝覆盖，请核对备份") from exc

    def directory(self, version_id):
        if not ID_RE.fullmatch(version_id):
            raise HTTPException(400, "非法地图版本 ID")
        path = self.versions / version_id
        if path.is_symlink() or path.resolve().parent != self.versions:
            raise HTTPException(409, "拒绝操作工作区外的地图")
        if not path.is_dir():
            raise HTTPException(404, "地图版本不存在")
        if any(item.is_symlink() for item in path.iterdir()):
            raise HTTPException(409, "地图版本含符号链接，拒绝操作")
        return path

    def item(self, version_id, state=None):
        path = self.directory(version_id)
        state = state or self.state()
        manifest = json.loads((path / "manifest.json").read_text())
        metadata = yaml.safe_load((path / "map.yaml").read_text())
        issues = [f"缺少 {name}" for name in FILES if not (path / name).is_file()]
        if metadata.get("image") != "map.pgm":
            issues.append("地图图像引用不正确")
        return {**manifest, "id":version_id, "metadata":metadata,
                "archived":version_id in state.get("archived", []),
                "selected":state.get("selected_id") == version_id,
                "complete":not issues, "issues":issues,
                "map_preview_url":f"/api/2d/versions/{version_id}/map.png",
                "map_yaml_path":str(path / "map.yaml"),
                "download_url":f"/api/2d/versions/{version_id}/download",
                "size":sum(f.stat().st_size for f in path.iterdir() if f.is_file())}

    def overview(self):
        with self.lock:
            state = self.state()
            items, invalid = [], []
            for path in sorted(self.versions.iterdir(), reverse=True):
                if not ID_RE.fullmatch(path.name):
                    continue
                try:
                    items.append(self.item(path.name, state))
                except (HTTPException, OSError, ValueError, yaml.YAMLError, AttributeError) as exc:
                    invalid.append({"id":path.name, "error":str(exc)})
            items.sort(key=lambda v:v.get("created_at",""), reverse=True)
            return {"versions":items, "invalid":invalid, "selected_id":state.get("selected_id"),
                    "previous_id":state.get("previous_id"), "job":json.loads(json.dumps(self.job)),
                    "navigation":{"available":False, "localization_available":True, "reason":"已集成双 EKF 点云定位；Nav2 导航与底盘执行尚未接入，不发送导航目标"},
                    "profiles":self.list_profiles()}

    def list_profiles(self):
        config_path = Path(__file__).resolve().parents[1] / "config/navigation2d.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if config.get("schema") != 1:
            raise HTTPException(409,"2D 参数配置 schema 必须为 1")
        values = config["profiles"]
        for value in values:
            value["parameters"] = GridParameters(**value["parameters"]).model_dump()
        for path in sorted(self.profiles.glob("profile-*.yaml")):
            if path.is_symlink():
                continue
            try:
                value = yaml.safe_load(path.read_text())
                value["parameters"] = GridParameters(**value["parameters"]).model_dump()
                values.append(value)
            except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError):
                continue
        return values

    def save_profile(self, request):
        with self.lock:
            profile_id = "profile-" + uuid.uuid4().hex[:16]
            value = {"id":profile_id, "name":request.name.strip(), "parameters":request.parameters.model_dump()}
            (self.profiles / (profile_id+".yaml")).write_text(yaml.safe_dump(value, allow_unicode=True), encoding="utf-8")
            return value

    def update_job(self, **changes):
        with self.lock:
            self.job.update(changes)
            atomic_json(self.job_path, self.job)

    def check_cancel(self):
        if self.cancel.is_set():
            raise PipelineCancelled("2D 地图生成已取消")

    def start(self, request, source):
        with self.lock:
            if self.job["running"]:
                raise HTTPException(409, "已有 2D 生成任务正在运行")
            if source["category"] not in {"maps","processed"} or source.get("archived") or Path(source["path"]).suffix.lower() != ".pcd":
                raise HTTPException(409, "请选择未归档的原始或处理后 PCD")
            version_id = new_id()
            self.cancel.clear()
            self.update_job(running=True, status="running", progress=0, logs=["准备只读源 PCD"], error=None, result_id=None, id=version_id, source_id=source["id"])
            self.thread = threading.Thread(target=self.build, args=(request,source,version_id), daemon=True, name="d1max-grid-build")
            self.thread.start()
            return {"version_id":version_id, "job":dict(self.job)}

    def build(self, request, source, version_id):
        staging = self.versions / ("."+version_id+".building")
        final = self.versions / version_id
        try:
            staging.mkdir(exist_ok=False)
            parameters = request.parameters
            raw = Path(source["path"])
            pcd = staging / "localization.pcd"
            identity = raw.stat()
            pipeline = {"schema":POINTCLOUD_SCHEMA,"id":"grid-localization","name":"2D 定位点云准备","input":{"path":str(raw)},
                        "output":{"path":str(pcd),"preserve_intensity":True},
                        "modules":[
                            {"id":"statistical","type":"statistical_outlier","enabled":parameters.filter_enabled,
                             "parameters":{"neighbors":parameters.statistical_mean_k,"std_ratio":parameters.statistical_std_dev_mul}},
                            {"id":"radius","type":"radius_outlier","enabled":parameters.filter_enabled,
                             "parameters":{"radius":parameters.radius,"min_points":parameters.radius_min_points}}]}
            self.check_cancel()
            if parameters.filter_enabled:
                run_pipeline(pipeline, cancelled=self.cancel.is_set,
                             on_event=lambda message,progress,stage:self.update_job(progress=round(progress*.55),logs=[message]))
            else:
                shutil.copy2(raw, pcd)  # Preserve exact intensity/data encoding when filters are off.
            after = raw.stat()
            if (identity.st_ino,identity.st_size,identity.st_mtime_ns) != (after.st_ino,after.st_size,after.st_mtime_ns):
                raise ValueError("源 PCD 正在变化，请等待建图保存完成后再生成")
            self.check_cancel()
            self.update_job(progress=60, logs=["按 Z 高度切片并投影 XY；定位 PCD 不做高度裁剪"])
            import open3d as o3d
            cloud = o3d.io.read_point_cloud(str(pcd))
            image, metadata, slice_points = rasterize(np.asarray(cloud.points), parameters)
            self.check_cancel()
            image.save(staging / "map.pgm", format="PPM")
            (staging / "map.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
            pipeline["output"]["path"] = str(final / "localization.pcd")
            recipe = {"schema":1, "name":request.name, "source_id":source["id"], "pointcloud":pipeline,
                      "projection":parameters.model_dump(), "outputs":{key:str(final / key) for key in ("map.pgm","map.yaml","localization.pcd")}}
            (staging / "pipeline.yaml").write_text(yaml.safe_dump(recipe,allow_unicode=True,sort_keys=False),encoding="utf-8")
            pixels = np.asarray(image)
            manifest = {"schema":1,"name":request.name.strip(),"note":request.note,"created_at":now(),
                        "source_id":source["id"],"source_name":source["name"],"source_path":source["path"],
                        "origin":"pcd-projection","parent_version":None,"parameters":parameters.model_dump(),
                        "width":image.width,"height":image.height,"slice_points":slice_points,
                        "occupied_cells":int(np.count_nonzero(pixels==0)), "free_cells":int(np.count_nonzero(pixels==254)),
                        "unknown_cells":int(np.count_nonzero(pixels==205)),
                        "warning":"静态 PCD 无观测射线；自由栅格不等于已验证可通行，请检查楼层、墙体和机器人尺寸"}
            atomic_json(staging / "manifest.json", manifest)
            self.check_cancel()
            os.replace(staging, final)
            self.update_job(running=False,status="complete",progress=100,result_id=version_id,logs=["候选版本已保存；未切换选用地图"])
        except Exception as exc:
            if staging.exists() and staging.parent == self.versions and not staging.is_symlink():
                shutil.rmtree(staging)
            cancelled = isinstance(exc, PipelineCancelled) or self.cancel.is_set()
            self.update_job(running=False,status="cancelled" if cancelled else "failed", error=None if cancelled else str(exc), logs=[str(exc)])

    def edit(self, version_id, request):
        with self.lock:
            if self.job["running"]:
                raise HTTPException(409, "请先等待 2D 生成任务完成")
            source = self.directory(version_id)
            if not self.item(version_id)["complete"]:
                raise HTTPException(409,"源版本文件不完整")
            if sum(len(op.points) for op in request.operations) > 100000:
                raise HTTPException(400,"笔画点数过多")
            with Image.open(source / "map.pgm") as opened:
                if opened.width * opened.height > 16000000:
                    raise HTTPException(400,"地图过大")
                original = opened.convert("L")
            edited = original.copy()
            try:
                draw = ImageDraw.Draw(edited)
                for operation in request.operations:
                    draw_operation(draw,operation,*edited.size)
            except ValueError as exc:
                raise HTTPException(400,str(exc)) from exc
            before, after = np.asarray(original), np.asarray(edited)
            changed = int(np.count_nonzero(before != after))
            if not changed:
                raise HTTPException(400,"笔画没有改变地图像素")
            derived = new_id()
            stage = self.versions / ("."+derived+".building")
            stage.mkdir(exist_ok=False)
            try:
                for name in ("map.yaml","localization.pcd"):
                    shutil.copy2(source / name, stage / name)
                edited.save(stage / "map.pgm", format="PPM")
                summary = {"changed_pixels":changed,"operation_count":len(request.operations)}
                manifest = json.loads((source / "manifest.json").read_text())
                manifest.update(name=request.name.strip(),note=request.note,created_at=now(),parent_version=version_id,
                                origin="2d-edited",edit_summary=summary,
                                occupied_cells=int(np.count_nonzero(after==0)), free_cells=int(np.count_nonzero(after==254)),
                                unknown_cells=int(np.count_nonzero(after==205)))
                atomic_json(stage / "manifest.json",manifest)
                recipe = yaml.safe_load((source / "pipeline.yaml").read_text())
                recipe.update(parent_version=version_id, name=request.name.strip())
                recipe.setdefault("edit_history",[]).append({"parent_version":version_id,"operations":[op.model_dump() for op in request.operations]})
                recipe["pointcloud"]["output"]["path"] = str(self.versions / derived / "localization.pcd")
                recipe["outputs"] = {key:str(self.versions / derived / key) for key in ("map.pgm","map.yaml","localization.pcd")}
                (stage / "pipeline.yaml").write_text(yaml.safe_dump(recipe,allow_unicode=True),encoding="utf-8")
                os.replace(stage,self.versions / derived)
            except Exception:
                shutil.rmtree(stage)
                raise
            return {"version_id":derived,"edit_summary":summary}

    def select(self, version_id):
        with self.lock:
            if self.protected_version() and self.protected_version()!=version_id:
                raise HTTPException(409,"定位运行中，必须先停止定位才能切换地图")
            item = self.item(version_id)
            if self.job["running"] or item["archived"] or not item["complete"]:
                raise HTTPException(409,"生成任务运行中，或地图已归档/不完整")
            state = self.state()
            if state.get("selected_id") != version_id:
                state.update(previous_id=state.get("selected_id"),selected_id=version_id)
                atomic_json(self.state_path,state)
            return {"selected_id":version_id, "navigation_started":False}

    def rollback(self):
        with self.lock:
            previous = self.state().get("previous_id")
            if not previous:
                raise HTTPException(409,"没有可回退的上一版本")
            return self.select(previous)

    def patch(self, version_id, request):
        with self.lock:
            if request.archived and self.protected_version()==version_id:
                raise HTTPException(409,"定位正在使用这个版本，不能归档")
            path = self.directory(version_id)
            state = self.state()
            if request.archived and state.get("selected_id") == version_id:
                raise HTTPException(409,"当前选用地图不能归档，请先切换或取消选用")
            if request.name is not None:
                manifest = json.loads((path / "manifest.json").read_text())
                manifest["name"] = request.name.strip()
                atomic_json(path / "manifest.json",manifest)
            if request.archived is not None:
                archived = set(state.get("archived",[]))
                archived.add(version_id) if request.archived else archived.discard(version_id)
                state["archived"] = sorted(archived)
                atomic_json(self.state_path,state)
            return self.item(version_id)

    def delete_archived(self, request):
        with self.lock:
            state = self.state()
            ids = list(dict.fromkeys(request.version_ids))
            targets = []
            for version_id in ids:
                if self.protected_version()==version_id:
                    raise HTTPException(409,"定位正在使用这个版本，不能删除")
                if version_id not in state.get("archived",[]) or version_id == state.get("selected_id"):
                    raise HTTPException(409,"只能永久删除已归档且未选用的 2D 版本")
                targets.append(self.directory(version_id))
            for path in targets:
                shutil.rmtree(path)
            state["archived"] = [value for value in state.get("archived",[]) if value not in ids]
            if state.get("previous_id") in ids:
                state["previous_id"] = None
            atomic_json(self.state_path,state)
            return {"deleted_count":len(ids),"deleted_ids":ids}

    def close(self):
        self.cancel.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5)

def create_router(store: GridWorkspace, source_lookup, shared_lock, other_busy):
    router = APIRouter(prefix="/api/2d")
    @router.get("/overview")
    def overview():
        return store.overview()
    @router.post("/build",status_code=202)
    def build(request:GridBuildRequest):
        with shared_lock:
            if other_busy():
                raise HTTPException(409,"点云处理或定位正在运行，请完成后再生成 2D")
            return store.start(request,source_lookup(request.source_id))
    @router.post("/cancel")
    def cancel():
        with store.lock:
            if not store.job["running"]:
                raise HTTPException(409,"没有进行中的 2D 生成任务")
            store.cancel.set()
            store.update_job(status="cancelling")
            return store.job
    @router.post("/profiles")
    def profile(request:ProfileRequest):
        return store.save_profile(request)
    @router.get("/versions/{version_id}/map.png")
    def preview(version_id:str):
        path = store.directory(version_id)
        with Image.open(path / "map.pgm") as opened:
            data = io.BytesIO()
            opened.save(data,format="PNG")
        return Response(data.getvalue(),media_type="image/png",headers={"Cache-Control":"private, max-age=300"})
    @router.get("/versions/{version_id}/files/{name}")
    def file(version_id:str,name:str):
        if name not in FILES:
            raise HTTPException(404,"文件不存在")
        return FileResponse(store.directory(version_id) / name,filename=name)
    @router.get("/versions/{version_id}/download")
    def download(version_id:str):
        path = store.directory(version_id)
        def chunks():
            import zipfile
            with tempfile.TemporaryFile() as temp:
                with zipfile.ZipFile(temp,"w",zipfile.ZIP_DEFLATED) as archive:
                    for name in FILES:
                        archive.write(path / name,name)
                temp.seek(0)
                while block := temp.read(256*1024):
                    yield block
        return StreamingResponse(chunks(),media_type="application/zip",headers={"Content-Disposition":f'attachment; filename="{version_id}.zip"'})
    @router.post("/versions/{version_id}/edit-2d")
    def edit(version_id:str,request:GridEditRequest):
        return store.edit(version_id,request)
    @router.patch("/versions/{version_id}")
    def metadata(version_id:str,request:MetadataRequest):
        return store.patch(version_id,request)
    @router.post("/selection")
    def select(request:SelectionRequest):
        return store.select(request.version_id)
    @router.delete("/selection")
    def unselect():
        with store.lock:
            if store.protected_version():
                raise HTTPException(409,"请先停止定位，再取消选用地图")
            state=store.state()
            state.update(previous_id=state.get("selected_id"),selected_id=None)
            atomic_json(store.state_path,state)
            return state
    @router.post("/rollback")
    def rollback():
        return store.rollback()
    @router.post("/archive/delete")
    def delete(request:DeleteRequest):
        return store.delete_archived(request)
    return router

def replay_recipe(config_path: Path, output_dir: Path):
    """One YAML replays projection and every edit; never overwrite an existing output."""
    config_path, output_dir = config_path.resolve(), output_dir.resolve()
    if output_dir.exists():
        raise ValueError("输出目录已存在，拒绝覆盖；请选择一个新目录")
    recipe = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if recipe.get("schema") != 1:
        raise ValueError("只支持 schema 1")
    parameters = GridParameters(**recipe["projection"])
    pipeline = recipe["pointcloud"]
    source = Path(pipeline["input"]["path"])
    if not source.is_file():
        raise ValueError("原始输入 PCD 不存在，无法重放")
    output_dir.mkdir(parents=True,exist_ok=False)
    try:
        pipeline["output"]["path"] = str(output_dir / "localization.pcd")
        if any(module["enabled"] for module in pipeline["modules"]):
            run_pipeline(pipeline)
        else:
            shutil.copy2(source,output_dir / "localization.pcd")
        import open3d as o3d
        cloud=o3d.io.read_point_cloud(str(output_dir / "localization.pcd"))
        image,metadata,slice_points=rasterize(np.asarray(cloud.points),parameters)
        history=recipe.get("edit_history",[])
        if sum(len(op.get("points",[])) for entry in history for op in entry["operations"]) > 100000:
            raise ValueError("笔画历史超过 10 万点")
        draw=ImageDraw.Draw(image)
        for entry in history:
            for operation in entry["operations"]:
                draw_operation(draw,EditOperation(**operation),*image.size)
        image.save(output_dir / "map.pgm",format="PPM")
        (output_dir / "map.yaml").write_text(yaml.safe_dump(metadata,sort_keys=False),encoding="utf-8")
        recipe["outputs"]={name:str(output_dir/name) for name in ("map.pgm","map.yaml","localization.pcd")}
        (output_dir / "pipeline.yaml").write_text(yaml.safe_dump(recipe,allow_unicode=True),encoding="utf-8")
        pixels=np.asarray(image)
        atomic_json(output_dir / "manifest.json",{"schema":1,"name":recipe.get("name","YAML 重放"),
                    "created_at":now(),"source_path":str(source),"source_id":recipe.get("source_id"),
                    "origin":"yaml-replay","width":image.width,"height":image.height,"parameters":parameters.model_dump(),
                    "slice_points":slice_points,"occupied_cells":int(np.count_nonzero(pixels==0)),
                    "free_cells":int(np.count_nonzero(pixels==254)),"unknown_cells":int(np.count_nonzero(pixels==205)),
                    "warning":"离线重放版本，尚未验证实机可通行性"})
        return recipe["outputs"]
    except Exception:
        shutil.rmtree(output_dir)
        raise

if __name__ == "__main__":
    import argparse
    parser=argparse.ArgumentParser(description="Replay a D1 Max 2D map pipeline without ROS or robot commands")
    parser.add_argument("--config",required=True,type=Path)
    parser.add_argument("--output-dir",required=True,type=Path)
    args=parser.parse_args()
    print(json.dumps(replay_recipe(args.config,args.output_dir),ensure_ascii=False))
