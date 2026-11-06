"""Passive production wiring checks, not a ROS or physical-stop experiment.

The pure C++ scope/authority behavior is covered by the BT gtests. These
checks locate actual C++ blocks (ignoring comments/string braces), so an
unrelated guard or a comment cannot stand in for guarding an allocation.
"""
from pathlib import Path
import re

import pytest


ROOT = Path(__file__).resolve().parents[3]
NAVIGATOR = ROOT/'src/d1max_navigation_bt/src/navigator_node.cpp'
POLICY = ROOT/'src/d1max_navigation_bt/include/d1max_navigation_bt/runtime_policy.hpp'


def masked_cpp(source):
    token = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
    return token.sub(lambda match: ''.join('\n' if char == '\n' else ' '
                                         for char in match.group()), source)


def closing(code, start, opening='{', closing='}'):
    assert code[start] == opening
    depth = 0
    for index in range(start, len(code)):
        if code[index] == opening:
            depth += 1
        elif code[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    raise AssertionError('Unbalanced production C++ block')


def body(source, signature):
    code = masked_cpp(source)
    match = re.search(signature, code)
    assert match, 'Missing production signature: '+signature
    begin = code.index('{', match.end())
    end = closing(code, begin)
    return source[begin+1:end]


def if_blocks(source):
    code = masked_cpp(source)
    for match in re.finditer(r'\bif\s*\(', code):
        begin_condition = code.index('(', match.start())
        end_condition = closing(code, begin_condition, '(', ')')
        begin = end_condition+1
        while begin < len(code) and code[begin].isspace():
            begin += 1
        if begin < len(code) and code[begin] == '{':
            end = closing(code, begin)
            yield code[begin_condition+1:end_condition], begin+1, end


def compact(source):
    # Only whitespace is removed; real literals in permission predicates stay
    # visible. The selected production bodies contain no explanatory comments.
    return re.sub(r'\s+', '', source)


def test_navigator_defaults_to_navigation_and_validates_actual_scope_parameters():
    source = NAVIGATOR.read_text()
    assert re.search(r'planning_scope_\s*=\s*declare_parameter<std::string>\(\s*'
                     r'"planning_scope"\s*,\s*"navigation"\s*\)', source)
    code = compact(masked_cpp(source))
    assert 'validPlanningScope(planning_scope_,execution_purpose_,preview_only_,transport_mode_)' in code


def test_both_compute_result_owner_allocations_are_inside_their_real_local_scope_guard():
    callback = body(NAVIGATOR.read_text(),
                    r'options\.result_callback\s*=\s*\[this,\s*task\]\s*\([^)]*\)')
    code = masked_cpp(callback)
    allocations = {
        'TaskExecutionAuthority': r'usesLocalExecutionAuthority\s*\(\s*planning_scope_\s*\)\s*&&\s*taskFollower\s*\(\s*\)',
        'exec3::Owner': r'usesLocalExecutionAuthority\s*\(\s*planning_scope_\s*\)\s*&&\s*usesGeometryOwner\s*\(\s*execution_purpose_\s*,\s*preview_only_\s*\)',
    }
    blocks = list(if_blocks(callback))
    for authority, condition in allocations.items():
        positions = [match.start() for match in re.finditer(
            r'std::make_unique\s*<\s*'+re.escape(authority)+r'\s*>', code)]
        assert len(positions) == 1
        matching_blocks = [(begin, end) for guard, begin, end in blocks
                           if re.fullmatch(condition, guard.strip())]
        assert matching_blocks, 'Missing real scope guard for '+authority
        assert all(any(begin <= position < end for begin, end in matching_blocks)
                   for position in positions)
    assert 'snapshot_pub_->publish(route.snapshot)' in compact(callback)
    assert 'BT::NodeStatus::SUCCESS' in callback  # Global result still completes.


@pytest.mark.parametrize('method', ['taskFollowerTick', 'executionTick'])
def test_global_scope_returns_before_any_local_owner_tick_or_lease_publication(method):
    tick = body(NAVIGATOR.read_text(), r'void\s+'+method+r'\s*\(\s*\)')
    assert re.match(r'\s*if\s*\(\s*!\s*usesLocalExecutionAuthority\s*\(\s*planning_scope_\s*\)\s*\)\s*return\s*;',
                    masked_cpp(tick))


def test_public_status_reports_scope_without_relabeling_motion_authority():
    status = compact(body(NAVIGATOR.read_text(), r'void\s+publishStatus\s*\(\s*\)'))
    assert '{"planning_scope",planning_scope_}' in status
    assert '{"motion_enabled",permitsMotionAuthority(execution_purpose_,preview_only_)}' in status
    assert '{"owner","BehaviorTree.CPP"}' in status


def test_new_scope_helpers_preserve_preview_live_only_global_capability():
    source = POLICY.read_text()
    valid = compact(body(source, r'inline\s+bool\s+validPlanningScope\s*\([^)]*\)'))
    assert valid == ('returnscope=="navigation"||(scope=="global_route"&&'
                     'purpose=="planning_only"&&preview&&transport=="live");')
    local = compact(body(source, r'inline\s+bool\s+usesLocalExecutionAuthority\s*\([^)]*\)'))
    assert local == 'returnscope=="navigation";'


def test_execution_permission_and_confirmation_predicates_are_not_relaxed_by_scope():
    source = POLICY.read_text()
    motion = compact(body(source, r'inline\s+bool\s+permitsMotionAuthority\s*\([^)]*\)'))
    assert motion == 'returnpurpose=="execution"&&!preview;'
    confirm = compact(body(source, r'inline\s+std::string\s+executionConfirmationBlocker\s*\([^)]*\)'))
    assert confirm == 'returnpurpose=="planning_only"?"planning_only_execution_disabled":"";'
    navigator = NAVIGATOR.read_text()
    for signature in (
        r'void\s+confirm\s*\([^)]*\)',
        r'void\s+confirmTaskFollower\s*\([^)]*\)',
    ):
        assert 'executionConfirmationBlocker(execution_purpose_)' in compact(body(navigator, signature))
