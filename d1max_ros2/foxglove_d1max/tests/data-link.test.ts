import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {execFileSync} from 'node:child_process';
test('live nodes use the loopback aggregator, distinct from the offline router',()=>{
 const client=readFileSync('config/zenoh-live.json5','utf8');
 const router=readFileSync('config/zenoh-router-live.json5','utf8');
 assert.match(client,/tcp\/127\.0\.0\.1:7448/);
 assert.doesNotMatch(client,/tcp\/192\.168/);
 assert.match(router,/tcp\/192\.168\.168\.100:7447/);
 assert.match(router,/tcp\/127\.0\.0\.1:7448/);
 assert.doesNotMatch(router,/0\.0\.0\.0/);
});
test('monitor owns router before subscribers, rejects both occupied ports and loads custom schemas',()=>{
 for(const f of ['scripts/start_live_monitor.sh','scripts/start_monitor_bridge.sh'])execFileSync('bash',['-n',f]);
 const start=readFileSync('scripts/start_live_monitor.sh','utf8');
 assert.match(start,/for port in \(8769,7448\)/);
 assert(start.indexOf('ros2 run rmw_zenoh_cpp rmw_zenohd')<start.indexOf('scripts/start_sdk_monitor.sh'));
 assert.match(start,/wait -n/);
 assert.match(readFileSync('scripts/start_monitor_bridge.sh','utf8'),/source \/home\/dndx\/d1max_nav_ws\/install\/setup.bash/);
});
