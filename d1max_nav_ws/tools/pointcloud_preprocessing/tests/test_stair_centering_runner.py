"""Package builder protections; real-map geometry is exercised separately."""
from pathlib import Path
import json
import numpy as np
import pytest

from tools.pointcloud_preprocessing.stair_centering_runner import relocate_package, _record_rejection


@pytest.mark.parametrize('target', ['same', 'inside', 'parent', 'existing'])
def test_relocation_never_overwrites_or_copies_into_its_own_input(tmp_path, target):
    source = tmp_path/'source'
    source.mkdir()
    marker = source/'keep'
    marker.write_text('original')
    destination = {'same': source, 'inside': source/'child', 'parent': tmp_path,
                   'existing': tmp_path/'existing'}[target]
    if target == 'existing': destination.mkdir()
    with pytest.raises(ValueError, match='new_independent_directory'):
        relocate_package(source, destination)
    assert marker.read_text() == 'original'
    assert sorted(p.name for p in source.iterdir()) == ['keep']


def test_rejected_numpy_candidate_keeps_actual_reason(tmp_path):
    result = dict(applied=False, path=np.array([[0., 0., 1.], [1., 0., 1.]]),
        layer_ids=np.array([2, 2]), diagnostics=dict(candidate_rejection='measured_structure_distance_regressed'))
    returned = _record_rejection(tmp_path, result)
    saved = json.loads((tmp_path/'centering_not_applied.json').read_text())
    assert returned['reason'] == result['diagnostics']['candidate_rejection']
    assert saved['path'] == result['path'].tolist()
    assert saved['layer_ids'] == [2, 2]
    assert returned['applied'] is False
