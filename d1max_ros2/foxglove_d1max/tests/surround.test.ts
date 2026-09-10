import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import ts from 'typescript';
const live=JSON.parse(readFileSync('layouts/D1Max-Live.json','utf8'));
async function script(id:string){
 const js=ts.transpileModule(live.userNodes[id].sourceCode,{compilerOptions:{target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ES2022}}).outputText;
 return await import('data:text/javascript;base64,'+Buffer.from(js).toString('base64'));
}
test('one lightweight perception tab combines both lidars and image surfaces without robot meshes',()=>{
 const p=live.configById['3D!d1scene'];
 assert.match(p.foxglovePanelTitle,/近似投影/);
 assert.equal(p.layers['d1-urdf'],undefined);
 for(const side of ['front','rear']) {
  assert.equal(p.topics[`/${side}_lidar`].visible,true);
  const image=p.topics[`/${side}_camera/image_compressed`];
  assert.equal(image.visible,true);assert.equal(image.planarProjectionFactor,0);
  assert.equal(image.cameraInfoTopic,`/d1max/view/${side}_camera_nominal_calibration`);
  assert.equal(image.color,'#ffffffb8');
 }
 const ids=new Set<string>();function walk(tree:any){if(typeof tree==='string')ids.add(tree);else{walk(tree.first);walk(tree.second);}}
 for(const tab of live.configById['Tab!d1workspace'].tabs)walk(tab.layout);
 for(const id of ids)assert.ok(live.configById[id],id);
 assert.equal(live.configById['Tab!d1workspace'].tabs.length,3);
 assert.deepEqual(live.configById['Tab!d1workspace'].tabs.map((t:any)=>t.title),['感知总览','状态诊断','雷达与 IMU']);
 assert.equal(live.configById['3D!d1surround'],undefined);
 assert.equal(live.configById['Markdown!d1projectioninfo'],undefined);
});
test('nominal camera intrinsics use actual JPEG dimensions, source stamp and documented FOV',async()=>{
 const s=await script('d1-camera-front');
 // JPEG with APP0 segment, followed by baseline SOF (1920 x 1080).
 const data=new Uint8Array([255,216,255,224,0,4,0,0,255,192,0,8,8,4,56,7,128,0]);
 const header={frame_id:'camera_front',stamp:{sec:123,nsec:456}};
 const result=s.default({message:{header,data}});
 assert.equal(result.width,1920);assert.equal(result.height,1080);
 assert.deepEqual(result.timestamp,header.stamp);
 assert.equal(result.frame_id,header.frame_id);
 assert.ok(Math.abs(result.K[0]-1920/(2*Math.tan(111*Math.PI/360)))<1e-8);
 assert.ok(Math.abs(result.K[4]-1080/(2*Math.tan(70*Math.PI/360)))<1e-8);
 assert.equal(s.default({message:{header,data:new Uint8Array([255,216,255,192,0,100])}}),undefined);
 assert.equal(s.default({message:{header:{...header,frame_id:'wrong'},data}}),undefined);
 assert.deepEqual(data,new Uint8Array([255,216,255,224,0,4,0,0,255,192,0,8,8,4,56,7,128,0]));
});
test('camera optical axes face forward/backward and keep image down aligned with body down',async()=>{
 const s=await script('d1-calibration'),stamp={sec:123,nsec:456};
 const transforms=s.default({topic:'/front_camera/image_compressed',message:{header:{stamp}}}).transforms;
 function rotate(q:any,v:number[]){const{x,y,z,w}=q,[a,b,c]=v,t=[2*(y*c-z*b),2*(z*a-x*c),2*(x*b-y*a)];return[a+w*t[0]+y*t[2]-z*t[1],b+w*t[1]+z*t[0]-x*t[2],c+w*t[2]+x*t[1]-y*t[0]];}
 for(const[frame,forward]of[['camera_front',1],['camera_back',-1]] as const){
  const t=transforms.find((t:any)=>t.child_frame_id===frame);
  assert.equal(t.header.frame_id,'d1max_sensor_rig');
  assert.deepEqual(t.header.stamp,stamp);
  assert.deepEqual(rotate(t.transform.rotation,[0,0,1]),[forward,0,0]);
  assert.deepEqual(rotate(t.transform.rotation,[0,1,0]),[0,0,-1]);
 }
});
test('passive SDK binary source contains no robot command API or inbound ROS command path',()=>{
 const src=readFileSync('../sdk_bridge_ws/src/d1max_sdk_bridge/src/sdk_telemetry_bridge.cpp','utf8');
 const called=[...src.matchAll(/sdk_->(\w+)\(/g)].map(m=>m[1]);
 assert.ok(called.length>0);
 for(const name of called)assert.ok(['SetDataCallback','Connect','Disconnect','IsConnected'].includes(name),name);
 assert.doesNotMatch(src,/create_subscription|create_service|\/cmd_vel|SetControlCallback/);
 assert.match(src,/start_parameter_services\(false\)/);
 assert.match(src,/"ready_for_navigation", nullptr/);
 assert.match(src,/"fault_latched", nullptr/);
});
test('camera scripts pass semantic TypeScript checks including Foxglove fixed-size matrices',()=>{
 const files=new Map<string,string>([
  ['/d1-virtual/types.ts','export type Input<T extends string> = {topic:T;message:{header:{frame_id:string;stamp:{sec:number;nsec:number}};data:Uint8Array}};'],
  ['/d1-virtual/schemas.d.ts',`declare module '@foxglove/schemas' {export type CameraCalibration={timestamp:{sec:number;nsec:number};frame_id:string;width:number;height:number;distortion_model:string;D:number[];K:Float64Array|[number,number,number,number,number,number,number,number,number];R:[number,number,number,number,number,number,number,number,number];P:[number,number,number,number,number,number,number,number,number,number,number,number]};}`]
 ]);
 for(const side of ['front','rear'])files.set(`/d1-virtual/${side}.ts`,live.userNodes['d1-camera-'+side].sourceCode);
 const options:ts.CompilerOptions={strict:true,noEmit:true,skipLibCheck:true,allowImportingTsExtensions:true,target:ts.ScriptTarget.ES2022,module:ts.ModuleKind.ESNext,moduleResolution:ts.ModuleResolutionKind.Bundler};
 const host=ts.createCompilerHost(options),read=host.readFile.bind(host),exists=host.fileExists.bind(host),dir=host.directoryExists?.bind(host);
 host.readFile=p=>files.get(p)??read(p);host.fileExists=p=>files.has(p)||exists(p);
 host.directoryExists=p=>p==='/d1-virtual'||!!dir?.(p);
 host.getSourceFile=(p,lang)=>{const text=host.readFile(p);return text===undefined?undefined:ts.createSourceFile(p,text,lang,true);};
 const program=ts.createProgram([...files.keys()],options,host);
 const errors=ts.getPreEmitDiagnostics(program).map(d=>ts.flattenDiagnosticMessageText(d.messageText,' '));
 assert.deepEqual(errors,[]);
});
