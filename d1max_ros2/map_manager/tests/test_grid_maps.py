import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from PIL import Image
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient
from backend.grid_maps import GridWorkspace, GridParameters, GridBuildRequest, GridEditRequest, EditOperation, MetadataRequest, DeleteRequest, create_router, rasterize, new_id

class GridTests(unittest.TestCase):
    def test_localization_pins_selected_map(self):
        first=self.build();second=self.build();self.store.select(first)
        self.store.protected_version=lambda:first
        with self.assertRaises(HTTPException):self.store.select(second)
        with self.assertRaises(HTTPException):self.store.patch(first,MetadataRequest(archived=True))
        self.assertEqual(self.store.state()['selected_id'],first)
        self.store.protected_version=lambda:None
        self.store.select(second)
    def test_single_yaml_reproduces_multiple_edit_revisions(self):
        from backend.grid_maps import replay_recipe
        version=self.build()
        for x in (8,18):
            result=self.store.edit(version,GridEditRequest(name="revision",operations=[{"mode":"free","shape":"brush","size":3,"points":[{"x":x,"y":8}]}]))
            version=result["version_id"]
        output=self.root/"replayed"
        replay_recipe(self.store.directory(version)/"pipeline.yaml",output)
        self.assertEqual((output/"map.pgm").read_bytes(),(self.store.directory(version)/"map.pgm").read_bytes())
        self.assertEqual((output/"localization.pcd").read_bytes(),self.raw.read_bytes())
        with self.assertRaises(ValueError):
            replay_recipe(self.store.directory(version)/"pipeline.yaml",output)
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix="d1max-grid-test-")
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.store=GridWorkspace(self.root/"workspace")
        self.raw=self.root/"source.pcd"
        points=[(x*.1,y*.1,1.,float(x+y)) for x in range(20) for y in range(20) if x in (0,19) or y in (0,19)]
        self.raw.write_text("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH "+str(len(points))+"\nHEIGHT 1\nPOINTS "+str(len(points))+"\nDATA ascii\n"+"".join(" ".join(map(str,p))+"\n" for p in points))
        self.source={"id":"a"*16,"name":"test source","path":str(self.raw),"category":"maps","archived":False}
    def build(self,parameters=None):
        request=GridBuildRequest(source_id="a"*16,name="test map",parameters=parameters or GridParameters())
        result=self.store.start(request,self.source)
        self.store.thread.join(20)
        self.assertFalse(self.store.thread.is_alive())
        self.assertEqual(self.store.job["status"],"complete",self.store.job)
        return result["version_id"]
    def test_projection_y_flip_bounds_origin_and_unknown_semantics(self):
        p=GridParameters(resolution=.5,padding=0)
        image,meta,count=rasterize(np.array([[0.,0.,1.],[1.,1.,1.]]),p)
        self.assertEqual(image.size,(3,3))
        pixels=np.asarray(image)
        self.assertEqual(int(pixels[2,0]),0)
        self.assertEqual(int(pixels[0,2]),0)
        self.assertEqual(int(pixels[1,1]),205)
        occupancy=(255-205)/255
        self.assertGreater(occupancy,meta["free_thresh"])
        self.assertLess(occupancy,meta["occupied_thresh"])
        self.assertEqual(meta["origin"],[0.,0.,0.])
    def test_invalid_z_empty_and_excessive_map_rejected(self):
        with self.assertRaises(ValueError):GridParameters(z_min=2,z_max=1)
        with self.assertRaises(ValueError):GridParameters(resolution=float("nan"))
        with self.assertRaises(ValueError):rasterize(np.array([[0,0,5]]),GridParameters())
        with self.assertRaises(ValueError):rasterize(np.array([[0,0,1],[1e9,1e9,1]]),GridParameters())
    def test_corrupt_state_is_not_overwritten(self):
        for payload in ("null", '{"archived": "bad"}', '{"archived": ["../outside"]}'):
            self.store.state_path.write_text(payload)
            with self.assertRaises(HTTPException):self.store.state()
            self.assertEqual(self.store.state_path.read_text(),payload)
    def test_invalid_names_and_boundary_strokes_rejected(self):
        with self.assertRaises(ValueError):GridBuildRequest(source_id="a"*16,name="   ")
        one=self.build()
        with self.assertRaises(HTTPException):
            self.store.edit(one,GridEditRequest(name="outside",operations=[{"mode":"free","shape":"brush","size":3,"points":[{"x":99999,"y":0}]}]))
        self.assertEqual(len(self.store.overview()["versions"]),1)
    def test_build_preserves_source_and_does_not_auto_select(self):
        before=self.raw.read_bytes()
        version=self.build()
        self.assertEqual(self.raw.read_bytes(),before)
        self.assertEqual((self.store.directory(version)/"localization.pcd").read_bytes(),before)
        self.assertIsNone(self.store.state().get("selected_id"))
        item=self.store.item(version)
        self.assertTrue(item["complete"])
        self.assertGreater(item["unknown_cells"],0)
        self.assertEqual(item["free_cells"],0)
    def test_go2_filter_profile_executes_and_recipe_replays(self):
        from backend.grid_maps import replay_recipe
        from backend.pointcloud_pipeline import validate_pipeline
        before=self.raw.read_bytes()
        version=self.build(GridParameters(filter_enabled=True,background="free"))
        directory=self.store.directory(version)
        pipeline=yaml.safe_load((directory/"pipeline.yaml").read_text())
        validate_pipeline(pipeline["pointcloud"],require_paths=True)
        self.assertIn(b"FIELDS x y z intensity",(directory/"localization.pcd").read_bytes())
        self.assertEqual(self.raw.read_bytes(),before)
        self.assertGreater(self.store.item(version)["free_cells"],0)
        replay_recipe(directory/"pipeline.yaml",self.root/"filtered-replay")
        self.assertTrue((self.root/"filtered-replay/map.yaml").exists())
    def test_edit_creates_derived_pgm_preserves_geometry_and_pcd(self):
        version=self.build()
        source=self.store.directory(version)
        before={name:(source/name).read_bytes() for name in ("map.pgm","map.yaml","localization.pcd")}
        operation={"mode":"free","shape":"brush","size":3,"points":[{"x":8,"y":8}]}
        result=self.store.edit(version,GridEditRequest(name="edited",operations=[operation]))
        target=self.store.directory(result["version_id"])
        for name,value in before.items():self.assertEqual((source/name).read_bytes(),value)
        self.assertEqual((target/"map.yaml").read_bytes(),before["map.yaml"])
        self.assertEqual((target/"localization.pcd").read_bytes(),before["localization.pcd"])
        self.assertNotEqual((target/"map.pgm").read_bytes(),before["map.pgm"])
        self.assertEqual(self.store.item(result["version_id"])["parent_version"],version)
        self.assertIsNone(self.store.state().get("selected_id"))
    def test_selection_rollback_archive_delete_and_current_guard(self):
        one=self.build()
        two=self.build(GridParameters(background="free"))
        self.store.select(one);self.store.select(two)
        self.store.rollback()
        self.assertEqual(self.store.state()["selected_id"],one)
        with self.assertRaises(HTTPException):self.store.patch(one,MetadataRequest(archived=True))
        with self.assertRaises(HTTPException):self.store.delete_archived(DeleteRequest(version_ids=[two]))
        self.store.patch(two,MetadataRequest(archived=True))
        self.assertEqual(self.store.delete_archived(DeleteRequest(version_ids=[two]))["deleted_count"],1)
        self.assertTrue(self.raw.exists())
        self.assertTrue(self.store.directory(one).exists())
    def test_invalid_ids_symlinks_and_all_delete_targets_validate_first(self):
        one=self.build()
        with self.assertRaises(HTTPException):self.store.directory("../source.pcd")
        self.store.patch(one,MetadataRequest(archived=True))
        with self.assertRaises(HTTPException):self.store.delete_archived(DeleteRequest(version_ids=[one,new_id()]))
        self.assertTrue(self.store.directory(one).exists())
        (self.store.directory(one)/"escape").symlink_to(self.raw)
        with self.assertRaises(HTTPException):self.store.delete_archived(DeleteRequest(version_ids=[one]))
        self.assertTrue(self.raw.exists())
    def test_build_failure_never_publishes_partial_or_selects(self):
        with patch("backend.grid_maps.rasterize",side_effect=ValueError("fixture failure")):
            self.store.start(GridBuildRequest(source_id="a"*16,name="bad"),self.source)
            self.store.thread.join(20)
        self.assertEqual(self.store.job["status"],"failed")
        self.assertEqual(self.store.overview()["versions"],[])
        self.assertIsNone(self.store.state().get("selected_id"))
    def test_duplicate_build_denied_and_shutdown_cancel(self):
        gate=threading.Event()
        def fake(*_):
            gate.wait(2)
            self.store.update_job(running=False,status="cancelled")
        with patch.object(self.store,"build",fake):
            self.store.start(GridBuildRequest(source_id="a"*16,name="one"),self.source)
            with self.assertRaises(HTTPException):
                self.store.start(GridBuildRequest(source_id="a"*16,name="two"),self.source)
            gate.set()
            self.store.close()
        self.assertTrue(self.store.cancel.is_set())
        self.assertFalse(self.store.thread.is_alive())
    def test_restart_marks_interrupted_without_deleting_source(self):
        self.store.update_job(running=True,status="running")
        fresh=GridWorkspace(self.store.root)
        fresh.recover()
        self.assertEqual(fresh.job["status"],"interrupted")
        self.assertFalse(fresh.job["running"])
        self.assertTrue(self.raw.exists())
    def test_api_no_commands_downloads_and_profile(self):
        app=FastAPI()
        app.include_router(create_router(self.store,lambda _id:self.source,threading.RLock(),lambda:False))
        client=TestClient(app)
        self.assertFalse(client.get("/api/2d/overview").json()["navigation"]["available"])
        self.assertEqual(client.post("/api/2d/navigation/start").status_code,404)
        version=self.build()
        result=client.get(f"/api/2d/versions/{version}/map.png")
        self.assertEqual(result.status_code,200)
        self.assertEqual(client.get(f"/api/2d/versions/{version}/files/secret").status_code,404)
        package=client.get(f"/api/2d/versions/{version}/download")
        self.assertEqual(package.status_code,200)
        self.assertTrue(package.content.startswith(b"PK"))
        result=client.post("/api/2d/profiles",json={"name":"my parameters","parameters":GridParameters().model_dump()})
        self.assertEqual(result.status_code,200)
        self.assertTrue((self.store.profiles/(result.json()["id"]+".yaml")).is_file())
    def test_no_api_build_during_other_cloud_job(self):
        app=FastAPI()
        app.include_router(create_router(self.store,lambda _id:self.source,threading.RLock(),lambda:True))
        client=TestClient(app)
        self.assertEqual(client.post("/api/2d/build",json={"source_id":"a"*16,"name":"blocked"}).status_code,409)
        self.assertFalse(self.store.job["running"])
