"""Repeated static floor repair preserves original evidence and review history."""
import copy

import numpy as np
import pytest

from pointcloud_preprocessing.repair_floor_support import _validate_evidence_repair


def evidence_fixture():
    # Index 0 is actual lower support; index 1 was already corrected by a
    # reviewed prior; index 2 is newly eligible in a later bounded review.
    return {
        'floor1_support': np.array([True, False, False, False]),
        'floor1_snapped_support': np.array([True, True, False, False]),
        'floor1_metric_support_added': np.zeros(4, dtype=bool),
        'floor1_reviewed_floor_prior': np.array([False, True, False, False]),
        'floor1_source_indices': np.arange(4),
        'floor1_floor_z': np.full(4, -.65),
        'conditioning_owner': np.array([0, 0, 0, -1]),
        'protected_stairs': np.array([False, False, False, True]),
    }


def expanded_review(before):
    after = copy.deepcopy(before)
    after['floor1_reviewed_floor_prior'][2] = True
    after['floor1_snapped_support'][2] = True
    return after


def test_existing_review_can_grow_without_reclassifying_measured_support():
    before = evidence_fixture()
    after = expanded_review(before)
    _validate_evidence_repair(before, after)
    np.testing.assert_array_equal(after['floor1_support'], before['floor1_support'])
    np.testing.assert_array_equal(after['floor1_metric_support_added'], before['floor1_metric_support_added'])


def test_repeating_unchanged_evidence_is_valid():
    before = evidence_fixture()
    _validate_evidence_repair(before, copy.deepcopy(before))


def test_review_cannot_erase_previously_applied_prior():
    before = evidence_fixture()
    after = expanded_review(before)
    after['floor1_reviewed_floor_prior'][1] = False
    with pytest.raises(ValueError, match='reviewed_floor_prior'):
        _validate_evidence_repair(before, after)


def test_new_review_cannot_become_original_measured_support():
    before = evidence_fixture()
    after = expanded_review(before)
    after['floor1_support'][2] = True
    with pytest.raises(ValueError, match='floor1_support'):
        _validate_evidence_repair(before, after)


def test_new_snap_requires_recomputed_metric_or_current_review_evidence():
    before = evidence_fixture()
    after = copy.deepcopy(before)
    after['floor1_snapped_support'][2] = True
    with pytest.raises(ValueError, match='floor1_snapped_support'):
        _validate_evidence_repair(before, after)


@pytest.mark.parametrize('key', ['floor1_source_indices', 'floor1_floor_z',
                                'conditioning_owner', 'protected_stairs'])
def test_review_expansion_does_not_relax_other_original_evidence(key):
    before = evidence_fixture()
    after = expanded_review(before)
    after[key][0] = 1 if key != 'protected_stairs' else True
    with pytest.raises(ValueError, match=key):
        _validate_evidence_repair(before, after)


def test_old_measured_support_and_snapped_membership_cannot_be_removed():
    before = evidence_fixture()
    for key in ('floor1_support', 'floor1_snapped_support'):
        after = expanded_review(before)
        after[key][0] = False
        with pytest.raises(ValueError, match=key):
            _validate_evidence_repair(before, after)


@pytest.mark.parametrize('mutation', ['missing', 'short', 'not_bool', 'unsnapped'])
def test_existing_prior_must_remain_an_aligned_recomputed_snap_mask(mutation):
    before = evidence_fixture()
    after = copy.deepcopy(before)
    key = 'floor1_reviewed_floor_prior'
    if mutation == 'missing':
        del after[key]
    elif mutation == 'short':
        after[key] = after[key][:-1]
    elif mutation == 'not_bool':
        after[key] = after[key].astype(np.uint8)
    else:
        after[key][2] = True
    # Depending on evidence ordering, malformed arrays are rejected either by
    # their earlier snapped-mask use or by the explicit prior contract check.
    with pytest.raises((ValueError, TypeError)):
        _validate_evidence_repair(before, after)
