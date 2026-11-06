#!/usr/bin/python3
"""Production candidate middleware work must not run on control/map owners."""
from pathlib import Path
import unittest

SOURCE=(Path(__file__).resolve().parents[1]/'include/plan_manage/execution_validator.hpp').read_text()

class CandidateDeliveryWiring(unittest.TestCase):
    def test_only_timer_invokes_consumer(self):
        self.assertEqual(SOURCE.count('publishSelectedCandidate();'),1)
        self.assertEqual(SOURCE.count('selected_candidate_pub_->publish('),1)
        constructor=SOURCE.split('ExecutionValidator(rclcpp::Node*',1)[1].split('~ExecutionValidator()',1)[0]
        self.assertIn('publishSelectedCandidate();',constructor)

    def test_separate_group_bounded_tick_and_cleanup(self):
        self.assertIn('selected_publication_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false)',SOURCE)
        self.assertIn('selectedPublicationGroup()const{return selected_publication_group_;}',SOURCE)
        self.assertIn('create_wall_timer(std::chrono::milliseconds(10)',SOURCE)
        self.assertIn('},selected_publication_group_);',SOURCE)
        self.assertIn('if(selected_publication_timer_)selected_publication_timer_->cancel()',SOURCE)
        self.assertIn('std::optional<ew::TaggedBspline> selected_candidate_publication_;',SOURCE)
        clear=SOURCE.split('void clearCandidatesLocked()',1)[1].split('void expireCandidateOfferLocked(',1)[0]
        self.assertIn('selected_candidate_publication_.reset()',clear)

    def test_publish_has_no_ledger_scope_or_snapshot(self):
        consumer=SOURCE.split('void publishSelectedCandidate() {',1)[1].split('struct MeasuredJoin',1)[0]
        self.assertIn('selected_candidate_publication_.reset();',consumer)
        # Trace capture may also run inside the ledger scope; delivery must not.
        scope=consumer.index('{std::lock_guard<std::mutex> lock(mutex_);')
        depth=0
        for end in range(scope,len(consumer)):
            depth += (consumer[end]=='{')-(consumer[end]=='}')
            if depth==0:
                break
        self.assertGreater(consumer.index('selected_candidate_pub_->publish('),end)
        self.assertNotIn('pool_',consumer)
        self.assertNotIn('emit_command',consumer)
        self.assertNotIn('local_command_pub_',consumer)

    def test_timer_isolates_transport_exception_without_requeue_or_renewal(self):
        timer=SOURCE.split('selected_publication_timer_=',1)[1].split('},selected_publication_group_);',1)[0]
        self.assertIn('try {publishSelectedCandidate();}',timer)
        self.assertIn('catch(const std::bad_alloc&) {throw;}',timer)
        self.assertIn('catch(const std::exception& e)',timer)
        self.assertIn('RCLCPP_ERROR_THROTTLE',timer)
        self.assertNotIn('valid_until',timer)
        self.assertNotIn('selected_candidate_publication_=',timer)

if __name__=='__main__':
    unittest.main()
