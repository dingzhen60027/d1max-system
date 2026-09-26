"""Publish generated read-only artifacts to the existing Web's static directory.

No service restart, new process, API state change, or navigation activation.
"""
import argparse
import json
import shutil
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--result', required=True, type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    result=args.result.resolve(strict=True)
    if result.parent != (root/'data/traversability').resolve():
        raise ValueError('Only generated traversability result directories can be published')
    target=root/'frontend/dist/traversability'/result.name
    target.mkdir(parents=True,exist_ok=True)
    for src in result.iterdir():
        if src.is_file() and src.suffix in {'.json','.yaml','.npz','.bin','.pcd','.ply','.png'}:
            shutil.copy2(src,target/src.name)
    report=json.loads((result/'report.json').read_text())
    viewer=Path(__file__).parent/('ground_viewer' if report.get('kind')=='ground_only' else 'viewer')
    for name in ['index.html','viewer.js']:
        shutil.copy2(viewer/name,target/name)
        shutil.copy2(viewer/name,result/name)
    for dest in [target,result]:
        vendor=dest/'vendor';vendor.mkdir(exist_ok=True)
        three=root/'frontend/node_modules/three'
        for name in ['three.module.js','three.core.js']:
            shutil.copy2(three/'build'/name,vendor/name)
        shutil.copy2(three/'examples/jsm/controls/OrbitControls.js',vendor/'OrbitControls.js')
    print(f'http://127.0.0.1:8766/traversability/{result.name}/')


if __name__=='__main__':main()
