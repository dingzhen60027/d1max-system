import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
import {localizationLayout} from '../scripts/localization-layout.mjs';
test('localization display bridge wrapper is valid shell and retains strict client guards',()=>{
 execFileSync('bash',['-n','scripts/start_monitor_bridge.sh']);
 const source=readFileSync('scripts/start_monitor_bridge.sh','utf8');
 assert(source.includes("'^/d1max/localization/.*$'"));
 assert(source.includes("client_topic_whitelist:=\"['a^']\""));
});
test('localization view replaces only PCD data; exact mosaic and other panels survive',()=>{
 const old=JSON.parse(readFileSync('layouts/D1Max-Monitor.json','utf8'));
 old.configById['3D!d1map'].followTf='d1max_floor1_map';
 old.configById['3D!d1map'].topics={'/d1max/maps/floor1/points':{visible:true}};
 const next=localizationLayout(old);
 assert.deepEqual(next.layout,old.layout);
 assert.deepEqual(next.configById['3D!d1map'].cameraState,old.configById['3D!d1map'].cameraState);
 for(const [id,value] of Object.entries(old.configById))if(!['3D!d1map','d1max-console.D1 状态监控!d1status'].includes(id))assert.deepEqual(next.configById[id],value);
 assert.equal(next.configById['3D!d1map'].followTf,'d1max_loc_map');
 assert.equal(next.configById['3D!d1map'].topics['/d1max/localization/scan_initial_preview'].decayTime,0.5);
 assert.deepEqual(localizationLayout(next),next);
 assert.doesNotMatch(JSON.stringify(next.configById['3D!d1map']),/floor1\/points|building\/points|Urdf|set_pose/);
});
