"""Single-process, sequential PCT map snapshot service (no ROS or SDK).

The parent owns deadlines and cancellation. It kills an active worker when a
goal is replaced, and retains only an idle worker for the next generation.
The planner has at most three native map resources. Only immutable configured
stair-segment curves are cached, with full validation on every request; arbitrary
start/goal floor paths are always solved for the new request.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import time


RESULT_FIELDS = ('path', 'path_xyz', 'layer_ids', 'edge_legs', 'route_type',
                 'floor', 'source_tomogram_sha256', 'execution_authorized',
                 'status', 'direction', 'native_map_cache', 'fixed_stair_cache',
                 'segments', 'anchors', 'layer_transitions')


def serve_requests(connection, build_planner):
    """The injectable builder permits offline protocol/lifecycle regressions."""
    planner = None
    previous_generation = 0
    try:
        while True:
            request = connection.recv()
            if request == {'kind': 'shutdown'}:
                return
            generation = request.get('generation') if isinstance(request, dict) else None
            began = time.monotonic()
            try:
                if isinstance(request, dict) and request.get('kind') == 'warmup':
                    warmup_id = request.get('warmup_id')
                    if (type(warmup_id) is not int or warmup_id < 1
                            or previous_generation != 0 or planner is not None):
                        raise ValueError('one_startup_warmup_required')
                    connection.send(dict(kind='warmup_progress',warmup_id=warmup_id,
                                         phase='initializing_native_resources'))
                    planner = build_planner()
                    metrics = planner.warmup_resources()
                    connection.send(dict(kind='warmed',warmup_id=warmup_id,
                        elapsed_sec=time.monotonic()-began,metrics=metrics))
                    continue  # not a goal; generation remains untouched
                if (not isinstance(request, dict) or request.get('kind') != 'plan'
                        or type(generation) is not int or generation <= previous_generation):
                    raise ValueError('strictly_increasing_plan_generation_required')
                previous_generation = generation
                initialized = planner is not None
                if planner is None:
                    connection.send(dict(kind='progress', generation=generation,
                        phase='initializing_map', worker_reused=False))
                    planner = build_planner()
                planning_began = time.monotonic()
                initialization_sec = planning_began-began
                connection.send(dict(kind='progress', generation=generation,
                    phase='native_search_and_validation', worker_reused=initialized,
                    initialization_sec=initialization_sec))
                result = planner.plan(request['start_xyz'], request['goal_xyz'],
                                      request['start_layer'], request['goal_layer'])
                connection.send(dict(kind='planned', generation=generation,
                    result={key: result[key] for key in RESULT_FIELDS if key in result},
                    worker_reused=initialized, worker_elapsed_sec=time.monotonic()-began,
                    initialization_sec=initialization_sec,
                    native_plan_elapsed_sec=time.monotonic()-planning_began))
            except BaseException as exc:
                connection.send(dict(kind='warmup_failed' if isinstance(request,dict)
                    and request.get('kind')=='warmup' else 'failed', generation=generation,
                    warmup_id=request.get('warmup_id') if isinstance(request,dict) else None,
                    error=str(exc)[:1000], error_code=getattr(exc,'code',type(exc).__name__)))
                # Native A* now resets both successful and failed searches.
                # Keep the outer failure boundary conservative: discard all
                # optimizer/coordinator state, not just A* query state.
                return
    except (EOFError, BrokenPipeError, OSError):
        return
    finally:
        connection.close()


def native_worker(connection, tomogram_path, route_config, map_options,
                  expected_tomogram_sha256, expected_config_sha256):
    def build():
        from d1max_pct_planner.tomogram_map import TomogramMap
        from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute
        if hashlib.sha256(Path(route_config).read_bytes()).hexdigest() != expected_config_sha256:
            raise ValueError('route_configuration_changed_since_parent_snapshot')
        tomogram = TomogramMap(tomogram_path, **map_options)
        if tomogram.sha256 != expected_tomogram_sha256:
            raise ValueError('tomogram_changed_since_parent_snapshot')
        import yaml
        from d1max_pct_planner.paths import expand_tree
        raw = expand_tree(yaml.safe_load(Path(route_config).read_text()))
        if raw.get('schema') == 'd1max.source_identity_route/v1':
            from d1max_pct_planner.singlefloor_route import SinglefloorRoute
            planner = SinglefloorRoute(tomogram,route_config)
        else:
            planner = CrossfloorPreviewRoute(tomogram, route_config)
        if hashlib.sha256(Path(route_config).read_bytes()).hexdigest() != expected_config_sha256:
            raise ValueError('route_configuration_changed_during_worker_initialization')
        return planner
    serve_requests(connection, build)
