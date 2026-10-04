"""One original-coordinate floor, all measured PCT slices; native planning."""
from pathlib import Path
import yaml

from .tomogram_route import TomogramRoute


def load_config(path,tomogram):
    from .paths import expand_tree
    raw = expand_tree(yaml.safe_load(Path(path).read_text()))
    # One declared floor, no stairs; which floor is the map package's choice.
    if (raw.get('schema') != 'd1max.source_identity_route/v1' or raw.get('frame_id')!='d1max_loc_map'
            or not isinstance(raw.get('floor_id'),str) or not raw['floor_id'] or raw.get('stairs_enabled') is not False
            or Path(raw['tomogram_path']).resolve()!=Path(tomogram.source).resolve()
            or raw['unknown_ceiling_policy']!=tomogram.unknown_ceiling_policy
            or raw['limits']['max_ground_step_m']!=tomogram.max_ground_step_m
            or tomogram.provenance.get('geometry_operation')!='source_identity'):
        raise ValueError('single_floor_source_identity_map_binding_invalid')
    tomogram.verify_source(raw['source_pcd'],raw['frame_id'])
    return raw,raw


class SinglefloorRoute:
    def __init__(self,tomogram,config):
        _,self.settings = load_config(config,tomogram)
        self.tomogram = tomogram
        self.route = TomogramRoute(tomogram,self.settings['vendor_root'],
            **self.settings['planning'],defer_map=True)

    def warmup_resources(self):
        self.route._load_native_map()
        return dict(scope='single_floor_all_slices_one_native_map',layers=self.tomogram.layers)

    def plan(self,start,goal,start_layer,goal_layer):
        result = self.route.plan(start,goal,start_layer,goal_layer)
        return dict(result,source_tomogram_sha256=self.tomogram.sha256,
            execution_authorized=False,route_type='same_floor',floor='lower')
