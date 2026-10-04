# Conditional CAS authorization, not a declaration that candidate is active.
uint8 schema_version
uint8 CONTINUOUS_REPLACE=0
uint8 STATIONARY_REENTRY=1
uint8 transition_mode
string handoff_id
uint64 sequence
uint64 expected_commit_sequence
ExecutionPermit incumbent
ExecutionPermit candidate
# Required only for STATIONARY_REENTRY. This is the original SDK-published
# nonterminal zero-write/MC witness, not BT-local velocity or terminal success.
SDKStationaryEvidence stationary_evidence
builtin_interfaces/Time source_stamp
builtin_interfaces/Time valid_until
# Fixed once for this handoff. Heartbeats cannot renew these two budgets.
builtin_interfaces/Time transition_deadline
builtin_interfaces/Time retain_incumbent_until
bool revoked
string reason
