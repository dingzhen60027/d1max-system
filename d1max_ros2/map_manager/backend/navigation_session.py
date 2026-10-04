"""Generic names for the existing, unique Web navigation lifecycle.

The class alias preserves the implementation, state ownership and service unit.
Callers must bind both API names to one runtime and one shared lock.
"""
from .single_floor import SingleFloorRuntime, _create_session_router


NavigationSessionRuntime = SingleFloorRuntime


def create_navigation_session_router(runtime,shared_lock,other_busy):
    return _create_session_router(runtime,shared_lock,other_busy,prefix='/api/navigation-session')
