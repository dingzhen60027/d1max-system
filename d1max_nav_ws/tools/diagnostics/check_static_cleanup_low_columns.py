#!/usr/bin/env python3
"""File-only dangerous counterexample for the inherited scene-specific filter.

This records a known limitation; it is not an acceptance test claiming low
objects are safe, and it does not modify a map or relax collision semantics.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.pointcloud_preprocessing.structural_obstacles import filter_structural_obstacles


def check():
    x,y=np.meshgrid(np.arange(0.,4.+1e-9,.04),np.arange(-1.,1.+1e-9,.04),indexing='ij')
    floor=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    cases=[]
    for height in (.25,.35,.65):
        theta,z=np.meshgrid(np.linspace(0.,2*np.pi,8,endpoint=False),
                           np.arange(.04,height+1e-9,.03),indexing='ij')
        pole=np.c_[2.+.025*np.cos(theta.ravel()),.3+.025*np.sin(theta.ravel()),z.ravel()]
        result=filter_structural_obstacles(np.vstack([floor,pole]),
                                          {'enabled':True,'support_mode':'metric_disk'})
        keep=result['keep_mask']
        cases.append(dict(height_m=height,diameter_m=.05,floor_points=len(floor),
            obstacle_points=len(pole),floor_removed=int((~keep[:len(floor)]).sum()),
            obstacle_removed=int((~keep[len(floor):]).sum()),
            obstacle_retained_height_m=float(pole[keep[len(floor):],2].max()),
            statistics=result['statistics']))
    return dict(cases=cases,physical_acceptance=False,
        limitation='The historical scene filter removes most returns of real short thin columns. This is an inherited static-prior limitation, not proof that any particular actual removed record is a human or a real column. The generated static map cannot replace independent live safety perception.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();result=check();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
