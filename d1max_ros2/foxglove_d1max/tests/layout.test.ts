import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import ts from 'typescript';
const layout=JSON.parse(readFileSync('layouts/D1Max-ANYmal.json','utf8'));
async function script(id:string){
 const code=ts.transpileModule(layout.userNodes[id].sourceCode,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ES2022}}).outputText;
 return import('data:text/javascript;base64,'+Buffer.from(code).toString('base64'));
}
test('all native mosaic references resolve; no legacy panel or map panel',()=>{
 const ids=new Set<string>();
 function walk(tree:any){if(typeof tree==='string'){assert.ok(layout.configById[tree],tree);ids.add(tree);}else if(tree){walk(tree.first);walk(tree.second);}}
 walk(layout.layout);for(const c of Object.values(layout.configById) as any[])for(const tab of c.tabs??[])walk(tab.layout);
 assert.equal(ids.size,Object.keys(layout.configById).length);
 assert.equal([...ids].filter(id=>id.startsWith('d1max-console.')).length,1);
 assert.ok(![...ids].some(id=>id.includes('D1 Max 控制台')||id.startsWith('Map!')));
});
test('SDK script preserves measurements and never invents zero for missing data',async()=>{
 const s=await script('d1-telemetry');const wrap=(data:string)=>s.default({message:{data}});
 assert.equal(wrap('{broken'),undefined);assert.equal(wrap('null'),undefined);
 const x=wrap('{"forward_speed":0.3,"lateral_speed":-0.1,"yaw_speed":0.2,"battery_power_1":86}');
 assert.equal(x.forward_speed,.3);assert.equal(x.lateral_speed,-.1);assert.ok(Number.isNaN(x.battery_power_2));
 assert.ok(Number.isNaN(wrap('{"forward_speed":"2"}').forward_speed));
});
test('display-only TF preserves bag timestamp and known lidar calibration',async()=>{
 const s=await script('d1-calibration'),stamp={sec:1788449291,nanosec:1};
 const out=s.default({message:{header:{stamp}}});
 assert.equal(out.transforms.length,2);
 assert.deepEqual(out.transforms[0].header,{stamp,frame_id:'d1max_lidar'});
 assert.equal(out.transforms[0].child_frame_id,'rslidar_head');
 assert.equal(out.transforms[1].child_frame_id,'rslidar_tail');
 assert.equal(out.transforms[1].transform.translation.z,-.7323);
 for(const t of out.transforms)assert.ok(Math.abs(Math.hypot(...Object.values(t.transform.rotation) as number[])-1)<.00001);
 assert.ok(s.output.startsWith('/d1max/view/'));
});
