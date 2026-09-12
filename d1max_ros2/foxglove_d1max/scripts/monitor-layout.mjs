// One native mosaic; preserve the user's existing sensor and map cameras.
import {readFileSync} from 'node:fs';
const settings=JSON.parse(readFileSync(new URL('../config/instruments.json',import.meta.url),'utf8'));
const guard=readFileSync(new URL('../config/scripts/instruments.ts',import.meta.url),'utf8');
const speedParser=readFileSync(new URL('../config/scripts/telemetry.ts',import.meta.url),'utf8')
 .replaceAll('__STATE_TOPIC__','/d1max_sdk_bridge/velocity');
// Window presence is independent of static-file loading. No topic means no PCD subscription.
export function emptyMapPanel(previous={}) {
 return {foxglovePanelTitle:'PCD · MAP',
  cameraState:structuredClone(previous.cameraState??{distance:20,perspective:true,phi:40,thetaOffset:135,target:[0,0,0],targetOffset:[0,0,0],targetOrientation:[0,0,0,1],fovy:45,near:.1,far:5000}),
  followMode:'follow-pose',scene:structuredClone(previous.scene??{backgroundColor:'#111218',meshUpAxis:'z_up'}),
  transforms:{},topics:{},layers:{},synchronize:false,imageMode:{}};
}
export function restoreMapWindow(current, previous) {
 const data=structuredClone(current),top=data.layout?.first;
 if(data.layout?.direction!=='column')throw Error('Unexpected layout; refusing to rearrange panels');
 if(top!=='3D!d1scene'&&!(top?.direction==='row'&&top.first==='3D!d1map'&&top.second==='3D!d1scene'))throw Error('Unexpected top row; refusing to rearrange panels');
 const oldTop=previous.layout?.first;
 if(oldTop?.direction!=='row'||oldTop.first!=='3D!d1map'||oldTop.second!=='3D!d1scene')throw Error('Missing original PCD split');
 if(top==='3D!d1scene')data.layout.first=structuredClone(oldTop);
 data.configById['3D!d1map']=emptyMapPanel(data.configById['3D!d1map']??previous.configById['3D!d1map']);
 return data;
}
// Change only the map content and its status source, never the mosaic or other cameras.
export function selectConfiguredMap(current,metadata,panel){
 if(metadata.enabled===false||metadata.maps.length!==1)throw Error('Select exactly one enabled map');
 const data=structuredClone(current),map=metadata.maps[0];
 if(!data.configById['3D!d1map'])throw Error('Restore the PCD window first');
 const seed=structuredClone(current);delete seed.configById['3D!d1map'];
 data.configById['3D!d1map']=monitorLayout(seed,metadata,panel).configById['3D!d1map'];
 data.configById['d1max-console.D1 状态监控!d1status'].config.mapTopic=panel.mapTopic;
 if(data.configById['3D!d1map'].followTf!==map.frame_id)throw Error('Map display frame mismatch');
 return data;
}
export function monitorLayout(original, metadata, panel) {
 const data=JSON.parse(JSON.stringify(original).replaceAll('d1max-console.D1 状态与操作!','d1max-console.D1 状态监控!'));
 const configs=data.configById;
 const row=(first,second,splitPercentage=50)=>({direction:'row',first,second,splitPercentage});
 const col=(first,second,splitPercentage=50)=>({direction:'column',first,second,splitPercentage});
 const statusId='d1max-console.D1 状态监控!d1status';
 configs[statusId]={config:panel,foxglovePanelTitle:'◉'};
 for(const p of Object.values(configs)){
  for(const [id,layer] of Object.entries(p.layers??{}))if(layer.layerId==='foxglove.Urdf'||id==='d1-urdf')delete p.layers[id];
 }
 const map=metadata.enabled===false?undefined:metadata.maps[0];
 if(map){
 const center=map.min.map((v,i)=>(v+map.max[i])/2),extent=map.max.map((v,i)=>v-map.min[i]);
 const previousMap=configs['3D!d1map'];
 configs['3D!d1map']={foxglovePanelTitle:'PCD · MAP',
  cameraState:previousMap?.cameraState??{distance:Math.max(10,Math.hypot(...extent)*1.25),perspective:true,phi:40,thetaOffset:135,target:[0,0,0],targetOffset:center,targetOrientation:[0,0,0,1],fovy:45,near:.1,far:5000},
  followTf:map.frame_id,followMode:'follow-pose',scene:{backgroundColor:'#111218',meshUpAxis:'z_up'},transforms:{},
  topics:Object.fromEntries(metadata.maps.map(m=>[m.topic,{visible:m.frame_id===map.frame_id,colorMode:'colormap',colorField:'z',colorMap:'turbo',pointSize:2,minValue:m.min[2],maxValue:m.max[2],decayTime:0}])),
  layers:{},synchronize:false,imageMode:{}};
 }else{
  configs['3D!d1map']=emptyMapPanel(configs['3D!d1map']);
 }
 configs['3D!d1scene'].foxglovePanelTitle='LiDAR + RGB · LIVE';
 configs['Image!d1front'].foxglovePanelTitle='CAM · F';
 configs['Image!d1rear'].foxglovePanelTitle='CAM · R';
 // A motion measurement appears only once, in its history plot below.
 for(const [id,field] of [['d1b1','battery_power_1'],['d1b2','battery_power_2']]){
  configs['Gauge!'+id]={foxglovePanelTitle:(id==='d1b1'?'B1':'B2')+' · %',path:'/d1max/view/instruments.'+field,
   style:'bar',minValue:0,maxValue:100,showTicks:true,tickInterval:50,
   colorMode:'colormap',colorMap:settings.batteryColorMap,reverse:false,reverseDirection:false};
 }
 const path=(field,label,color)=>({value:'/d1max/view/telemetry.'+field,label,color,enabled:true,timestampMethod:'receiveTime',lineSize:2});
 const plot=(title,paths,range)=>({foxglovePanelTitle:title,paths,showLegend:true,legendDisplay:'top',showPlotValues:false,isSynced:true,xAxisVal:'timestamp',followingViewWidth:settings.historySeconds,minYValue:-range,maxYValue:range,showXAxisLabels:true,showYAxisLabels:true});
 configs['Plot!d1linear']=plot('vₓ / vᵧ · m/s',[path('forward_speed','x',settings.blue),path('lateral_speed','y',settings.gold)],settings.linearRange);
 configs['Plot!d1yaw']=plot('ω · rad/s',[path('yaw_speed','ω',settings.blue)],settings.yawRange);
 // One compact native group, both battery bars visible together; no alternate page.
 configs['Tab!d1batteries']={activeTabIdx:0,tabs:[{title:'🔋',layout:col('Gauge!d1b1','Gauge!d1b2')}]};
 const bottom=row(row('Image!d1front','Image!d1rear'),
  row(row('Plot!d1linear','Plot!d1yaw'),row('Tab!d1batteries',statusId,settings.batteryGroupWidthPercent),settings.bottomPlotsRemainingPercent),
  settings.bottomCamerasWidthPercent);
 data.layout=col(row('3D!d1map','3D!d1scene'),bottom,settings.sceneHeightPercent);
 // Remove every retired tab, prose panel and hidden diagnostic instance.
 const used=new Set();
 function walk(tree){
  if(typeof tree==='string'){used.add(tree);for(const tab of configs[tree]?.tabs??[])walk(tab.layout);}
  else if(tree){walk(tree.first);walk(tree.second);}
 }
 walk(data.layout);data.configById=Object.fromEntries(Object.entries(configs).filter(([id])=>used.has(id)));
 data.userNodes['d1-instruments']={name:'D1 仪表新鲜度（只读）',sourceCode:guard.replaceAll('__STATE_TOPIC__',panel.stateTopic).replaceAll('__MONITOR_TOPIC__',panel.gatewayTopic).replaceAll('__STALE_SECONDS__',String(panel.staleSeconds))};
 // Existing velocity plots consume only true SDK speed callbacks. Batteries
 // stay on the separate RobotState instrument parser; no duplicated panels.
 data.userNodes['d1-telemetry']={name:'D1 SDK 遥测解析（只读）',sourceCode:speedParser};
 return data;
}
