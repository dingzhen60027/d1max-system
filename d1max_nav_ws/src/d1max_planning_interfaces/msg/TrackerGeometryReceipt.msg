# Software installation fact from the actual tracker, never SDK application,
# collision proof, physical stop confirmation, or permission to actuate.
uint8 schema_version
ExecutionVersion version
string frame_id
string transport_mode
int64 trajectory_id
uint64 sequence
# These installation facts are fixed until a different geometry is installed.
uint64 installation_sequence
uint64 permit_sequence
uint64 admission_sequence
uint64 validation_sequence
builtin_interfaces/Time installed_at
# Original acquisition time of the current body state. Publishing again does
# not refresh installed_at or any of the admission/validation evidence.
builtin_interfaces/Time body_source_stamp
bool installed
# Exact braking_envelope_reentry_required with installed=true fences motion
# for this actual installed geometry. Fixed installation facts stay unchanged;
# only a verified stationary re-entry writer transaction can clear that fence.
string reason
