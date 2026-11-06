"""Passive source wiring regression; never constructs a ROS node or SDK."""
from pathlib import Path
import os


def node_source():
    return Path(os.environ.get('TRACKER_NODE_SOURCE',
        str(Path(__file__).resolve().parents[1]/'src/local_follower_node.cpp'))).read_text()


def body(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def test_geometry_preparation_is_event_driven_but_activation_shares_control_tick():
    text = node_source()
    for signature in (
        '[this](wire::LocalTrajectoryCommit::ConstSharedPtr result)',
        '[this](wire::ReferenceProposal::ConstSharedPtr p)',
        '[this](wire::SupportReference::ConstSharedPtr s)',
        '[this](wire::TaggedBspline::ConstSharedPtr t)',
        '[this](wire::TrajectoryValidation::ConstSharedPtr v)',
        '[this](wire::LocalNavigationState::ConstSharedPtr state)',
        '[this](wire::RouteDeliveryReceipt::ConstSharedPtr r)',
    ):
        callback = body(text, signature)
        assert 'pumpGeometryExchange()' in callback, signature
        assert 'demand_->publish' not in callback
        assert 'follower_->step' not in callback
    poll = body(text, 'geometry_timer_=create_wall_timer(')
    assert 'milliseconds(10)' in poll and 'pumpGeometryExchange()' in poll
    assert 'tick()' not in poll
    pump = body(text, 'void pumpGeometryExchange(const rclcpp::Time& clock,double r,')
    assert 'commit_dispatch_.forward' in pump and 'follower_->exchangeCommit' in pump
    assert 'follower_->step' not in pump and 'demand_->publish' not in pump
    assert 'trigger=LocalFollower::ExchangeClock::InputDelivery' in pump
    assert 'event_time,trigger' in pump
    control = body(text, '  void tick()')
    assert 'pumpGeometryExchange(clock,r,LocalFollower::ExchangeClock::ControlTick)' in control
    assert control.index('pumpGeometryExchange') < control.index('follower_->step(')
    assert control.count('follower_->step(') == 1
    assert 'double hz=declare_parameter<double>("controller_frequency_hz",20.)' in text
    # Handover derives its runway from the SAME projected session timing,
    # not a second magic lifetime or a fresh timestamp supplied by an ACK.
    assert '1./hz,declare_parameter<double>("command_pipeline_bound_s",.1)' in text
    assert 'declare_parameter<double>("writer_period_s",.05)' in text


def test_writer_facts_only_reconcile_on_existing_control_tick():
    text = node_source()
    callback = body(text, '[this](wire::TaskCommandFeedback::ConstSharedPtr f)')
    assert 'follower_->commandFeedback' in callback
    assert 'step(' not in callback and 'publish(' not in callback
    assert 'pumpGeometryExchange' not in callback
    assert '/d1max/live_planning/execution/task_command_feedback' in text
    declaration = text[text.index('task_feedback_=create_subscription'):text.index('retirement_=create_subscription')]
    assert 'rclcpp::QoS(1).reliable()' in declaration


def test_diagnostic_consumer_is_isolated_without_parallel_controller_callbacks():
    text = node_source()
    constructor = body(text, 'LocalFollowerNode():Node(')
    # Diagnostics cannot change the original single-thread control scheduler.
    assert constructor.count('create_callback_group(') == 0
    assert 'SubscriptionOptions' not in constructor and 'callback_group=' not in constructor
    assert 'startControlTraceWorker();' in constructor
    assert 'trace_timer_' not in text and 'MultiThreadedExecutor' not in text
    assert 'rclcpp::spin(std::make_shared<d1max_trajectory_tracker::LocalFollowerNode>())' in text
    worker = body(text, 'void runControlTraceWorker()')
    assert 'wait_for(lock,std::chrono::seconds(1)' in worker
    assert worker.index('lock.unlock()') < worker.index('flushControlTrace()') < worker.index('lock.lock()')
    assert 'follower_' not in worker and 'catch(...)' in worker
    start = body(text, 'void startControlTraceWorker()')
    assert 'std::thread(' in start and 'catch(...)' in start
    assert 'follower_' not in start
    shutdown = body(text, 'void stopControlTraceWorker()')
    assert shutdown.index('trace_stop_=true') < shutdown.index('notify_all()') < shutdown.index('join()')
    assert '~LocalFollowerNode() override {stopControlTraceWorker();}' in text
    consumer = body(text, 'void flushControlTrace()')
    assert 'follower_' not in consumer and 'state_diagnostics_' not in consumer
    assert 'control_trace_.take()' in consumer and 'trace_file_.write(' in consumer
    assert 'catch(...)' in consumer
    tick = body(text, '  void tick()')
    assert 'flushControlTrace' not in tick and 'trace_file_' not in tick
    assert 'ControlInterruption' not in text and 'LocalStateAtInterruption' not in text
    assert tick.index('demand_->publish(command)') < tick.index('recordControlTrace(')
    producer = body(text, 'void recordControlTrace(')
    assert 'observeTick(sample)' in producer and 'control_trace_.push(' in producer
    assert 'follower_->step(' not in producer and 'catch(...)' in producer
