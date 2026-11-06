"""The map audit measures an incremental review against the actual baseline."""
import copy

import numpy as np
import pytest

from pointcloud_preprocessing.verify_reviewed_corridor import added_reviewed_prior


def fixture():
    return {'floor1_source_indices': np.array([5, 8, 11, 14]),
            'floor1_reviewed_floor_prior': np.array([False, True, False, False])}


def test_repeated_review_counts_only_new_records_not_previous_corrections():
    before = fixture()
    after = copy.deepcopy(before)
    after['floor1_reviewed_floor_prior'][2] = True
    np.testing.assert_array_equal(added_reviewed_prior(after, before, 'floor1'),
                                  [False, False, True, False])
    np.testing.assert_array_equal(before['floor1_reviewed_floor_prior'], [False, True, False, False])


def test_first_review_without_old_prior_remains_supported():
    after = fixture()
    before = {'floor1_source_indices': after['floor1_source_indices'].copy()}
    np.testing.assert_array_equal(added_reviewed_prior(after, before, 'floor1'),
                                  after['floor1_reviewed_floor_prior'])


def test_no_new_review_reports_no_newly_changed_records():
    before = fixture()
    assert not added_reviewed_prior(copy.deepcopy(before), before, 'floor1').any()


def test_cannot_hide_reverted_prior_as_no_new_changes():
    before = fixture()
    after = copy.deepcopy(before)
    after['floor1_reviewed_floor_prior'][1] = False
    with pytest.raises(ValueError, match='previous membership'):
        added_reviewed_prior(after, before, 'floor1')


def test_different_source_record_order_is_not_comparable():
    before = fixture()
    after = copy.deepcopy(before)
    after['floor1_source_indices'] = after['floor1_source_indices'][::-1]
    with pytest.raises(AssertionError):
        added_reviewed_prior(after, before, 'floor1')


@pytest.mark.parametrize('side,mutation', [('before', 'short'), ('after', 'short'),
                                         ('before', 'not_bool'), ('after', 'not_bool')])
def test_prior_masks_keep_original_index_shape_and_boolean_type(side, mutation):
    before = fixture()
    after = copy.deepcopy(before)
    target = before if side == 'before' else after
    key = 'floor1_reviewed_floor_prior'
    target[key] = target[key][:-1] if mutation == 'short' else target[key].astype(np.uint8)
    with pytest.raises(ValueError, match='aligned'):
        added_reviewed_prior(after, before, 'floor1')
