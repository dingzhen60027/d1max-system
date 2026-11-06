"""Shared route-correspondence bounds; not collision or motion clearance.

ReferenceWindow starts at the measured projection onto the fixed route, not at
the body. Native prepends the actual body and validates that connector later.
Its transport-level 3-D bound must therefore contain the sender's horizontal
detour and vertical correspondence bounds, including after a rigid reanchor.
"""
from math import hypot

PROJECTION_RADIUS_M = .3
LATERAL_RADIUS_M = 1.4
NATIVE_START_DISTANCE_M = hypot(LATERAL_RADIUS_M, PROJECTION_RADIUS_M)
