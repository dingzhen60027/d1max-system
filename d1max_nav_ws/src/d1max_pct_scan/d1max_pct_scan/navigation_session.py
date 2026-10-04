"""Public session API for the single indoor/outdoor, multi-floor BT mainline.

The legacy module contains the existing versioned implementation, not another
navigator.  Its current execution profile is floor-segment testing; opening
this API does not extend that profile's map, stair or physical acceptance.
Keep the exact same functions, globals, locks and task owner for old callers
and new callers.  Sealed releases continue to load their recorded module.
"""
from .single_floor_session import (
    main,
    prepare,
    run,
    runtime_commands,
    runtime_lock,
    sha,
    verify,
    view,
    view_commands,
)

__all__ = [
    'prepare', 'verify', 'run', 'view', 'main', 'sha',
    'runtime_commands', 'runtime_lock', 'view_commands',
]


if __name__ == '__main__':
    main()
