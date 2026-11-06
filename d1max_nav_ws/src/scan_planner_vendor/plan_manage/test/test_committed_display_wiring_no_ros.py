#!/usr/bin/python3
"""Verify display publications cannot share the execution ledger/control lane."""
import argparse
from pathlib import Path
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1]/'include/plan_manage/execution_validator.hpp')
args, remaining = parser.parse_known_args()
SOURCE = args.source.read_text()

class CommittedDisplayWiring(unittest.TestCase):
    def test_all_four_commits_enqueue_no_synchronous_publish(self):
        self.assertEqual(SOURCE.count('queueCommittedDisplayLocked();'), 4)
        self.assertNotIn('committed_pub_->publish(active_->spline)', SOURCE)
        self.assertEqual(SOURCE.count('committed_pub_->publish('), 1)

    def test_independent_timer_and_immutable_single_slot(self):
        self.assertIn('committed_display_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false)', SOURCE)
        self.assertIn('committedDisplayGroup()const{return committed_display_group_;}', SOURCE)
        self.assertIn('create_wall_timer(std::chrono::milliseconds(20)', SOURCE)
        self.assertIn('},committed_display_group_);', SOURCE)
        self.assertIn('std::shared_ptr<const ew::TaggedBspline> pending_committed_display_;', SOURCE)
        self.assertIn('std::make_shared<const ew::TaggedBspline>(active_->spline)', SOURCE)
        self.assertIn('view=std::move(pending_committed_display_);}', SOURCE)
        self.assertIn('if(view)emit(*view);', SOURCE)

    def test_cancel_and_new_domain_drop_unsent_geometry(self):
        for begin,end in [('bool cancelReference(', 'void cancelPending('),
                          ('bool enterTaskDomainLocked(', 'void pinInitialIntentLocked('),
                          ('bool taskLeaseAt(', 'bool taskLease('),
                          ('~ExecutionValidator()', '// Map writer notification')]:
            block=SOURCE.split(begin,1)[1].split(end,1)[0]
            self.assertIn('pending_committed_display_.reset()',block)
        self.assertIn('if(committed_display_timer_)committed_display_timer_->cancel()',SOURCE)

if __name__=='__main__':
    unittest.main(argv=['committed_display_wiring']+remaining)
