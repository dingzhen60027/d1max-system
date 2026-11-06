"""Passive C++ support-domain wiring checks; not a compiled Native verdict.

The C++ gtests cover connector geometry, the support index and LocalFollower
preparation. These checks independently keep all three production predicates
in the same two-floor ordinary-ground domain without starting ROS or SDK.
"""
from pathlib import Path
import re

import pytest


ROOT=Path(__file__).resolve().parents[3]
HEADERS=(
    ('src/scan_planner_vendor/plan_manage/include/plan_manage/support_connection.hpp',True),
    ('src/scan_planner_vendor/plan_manage/include/plan_manage/motion_sweep.hpp',False),
    ('src/d1max_trajectory_tracker/include/d1max_trajectory_tracker/tracker_core.hpp',False),
)


@pytest.mark.parametrize('relative,negative',HEADERS)
@pytest.mark.parametrize('floor,expected',[
    ('floor1',True),('floor2',True),('floor3',False),('',False),
    ('stairs',False),('stair_lower',False),('stair_upper',False),('Floor2',False),
])
def test_actual_floor_predicates_allow_only_the_two_literal_known_floors(relative,negative,floor,expected):
    source=(ROOT/relative).read_text()
    operator='!=' if negative else '=='
    join='&&' if negative else r'\|\|'
    pattern=r'\(\s*s\.floor_id\s*'+operator+r'\s*"([^"]+)"\s*'+join+r'\s*s\.floor_id\s*'+operator+r'\s*"([^"]+)"\s*\)'
    match=re.search(pattern,source)
    assert match,'Missing bounded two-floor production predicate'
    labels=match.groups()
    actual=(not all(floor!=label for label in labels) if negative
            else any(floor==label for label in labels))
    assert actual is expected


@pytest.mark.parametrize('relative,negative',HEADERS)
def test_verified_general_ground_and_original_geometry_limits_remain_required(relative,negative):
    source=re.sub(r'\s+','',(ROOT/relative).read_text())
    if negative:
        assert '!s.verified||' in source
        assert '(s.segment_kind!="floor"&&s.segment_kind!="landing")||s.required_mode!="general"' in source
        assert '(to-from).norm()>.15' in source
        assert 's.max_support_slope_rad>.176' in source and 's.max_support_step_m>.08' in source
    else:
        assert 's.verified&&' in source
        mode='s.mode' if relative.endswith('tracker_core.hpp') else 's.required_mode'
        assert '(s.segment_kind=="floor"||s.segment_kind=="landing")&&'+mode+'=="general"' in source
        if relative.endswith('tracker_core.hpp'):
            assert 's.identity==identity&&s.generation==generation' in source
            assert 's.max_slope<=.176' in source and 's.max_step<=.08' in source
        else:
            assert 's.max_support_slope_rad<=.176' in source and 's.max_support_step_m<=.08' in source
