// Reproducible native Foxglove layout. Generated files, never edits app caches.
import {readFile,writeFile,mkdir} from 'node:fs/promises';
const c=JSON.parse(await readFile('config/layout.json','utf8'));
const panel=JSON.parse(await readFile('config/panel.json','utf8'));
const row=(first,second,splitPercentage=50)=>({direction:'row',first,second,splitPercentage});
const col=(first,second,splitPercentage=50)=>({direction:'column',first,second,splitPercentage});
const cameraState={distance:c.distance,perspective:true,phi:55,target:[0,0,0],targetOffset:[0,0,0],targetOrientation:[0,0,0,1],thetaOffset:135,fovy:45,near:.1,far:5000};
const base={cameraState,followMode:'follow-pose',scene:{backgroundColor:'#111218',meshUpAxis:'z_up'},transforms:{},topics:{},layers:{},synchronize:false,imageMode:{}};
const cloud={visible:true,colorMode:'colormap',colorField:'z',colorMap:'turbo',pointSize:2,minValue:-2,maxValue:4,decayTime:0};
const scene={...base,foxglovePanelTitle:'双雷达 · 3D 感知',followTf:c.displayFrame,topics:{[c.frontCloudTopic]:cloud,[c.rearCloudTopic]:cloud},layers:{'d1-grid':{visible:true,frameLocked:true,label:'地面参考 · 1 m',instanceId:'d1-grid',layerId:'foxglove.Grid',size:50,divisions:50,lineWidth:1,color:'#343641',position:[0,0,-.8],rotation:[0,0,0],order:1}}};
const path=(value,label,color)=>({value,label,color,enabled:true,timestampMethod:'receiveTime'});
const plot=(title,paths)=>({foxglovePanelTitle:title,paths,showLegend:true,legendDisplay:'top',isSynced:true,xAxisVal:'timestamp',followingViewWidth:30});
const telemetryTopic='/d1max/view/telemetry';
const script=(await readFile('config/scripts/telemetry.ts','utf8')).replaceAll('__STATE_TOPIC__',panel.stateTopic);
const calibration=(await readFile('config/scripts/calibration.ts','utf8')).replaceAll('__CONFIG__',JSON.stringify(c)).replaceAll('__FRONT_CLOUD__',c.frontCloudTopic).replaceAll('__REAR_CLOUD__',c.rearCloudTopic);
const configById={
 'Tab!d1workspace':{activeTabIdx:0,tabs:[
   {title:'感知总览',layout:row(col('3D!d1scene',row(row('Image!d1front','Image!d1rear'),'Plot!d1speed',68),c.topHeightPercent),'d1max-console.D1 状态监控!d1status',c.sceneWidthPercent)},
   {title:'状态诊断',layout:row(col('Plot!d1linear','Plot!d1yaw'),col('Plot!d1battery','RawMessages!d1state'),64)},
   {title:'雷达与 IMU',layout:row('3D!d1inspect',col('Plot!d1imu','RawMessages!d1imu'),65)}
 ]},
 '3D!d1scene':scene,
 '3D!d1inspect':{...scene,foxglovePanelTitle:'双雷达 · 标定检查'},
 'd1max-console.D1 状态监控!d1status':{config:panel,foxglovePanelTitle:'D1 Max · 状态监控'},
 'Image!d1front':{...base,imageMode:{imageTopic:c.frontImageTopic},foxglovePanelTitle:'前视相机'},
 'Image!d1rear':{...base,imageMode:{imageTopic:c.rearImageTopic},foxglovePanelTitle:'后视相机'},
 'Plot!d1speed':plot('平移速度 · m/s',[path(telemetryTopic+'.forward_speed','纵向','#ab94f7'),path(telemetryTopic+'.lateral_speed','横向','#50c8a4')]),
 'Plot!d1linear':plot('纵向 / 横向速度 · m/s',[path(telemetryTopic+'.forward_speed','纵向','#ab94f7'),path(telemetryTopic+'.lateral_speed','横向','#50c8a4')]),
 'Plot!d1yaw':plot('转向速度 · rad/s',[path(telemetryTopic+'.yaw_speed','转向','#edbc71')]),
 'Plot!d1battery':{...plot('双电池电量 · %',[path(telemetryTopic+'.battery_power_1','电池 1','#ab94f7'),path(telemetryTopic+'.battery_power_2','电池 2','#50c8a4')]),minYValue:0,maxYValue:100},
 'RawMessages!d1state':{topicPath:telemetryTopic,defaultExpanded:true,foxglovePanelTitle:'SDK 遥测 · 只读解析'},
 'Plot!d1imu':plot('双 IMU · 角速度 Z · rad/s',[path(c.frontImuTopic+'.angular_velocity.z','前雷达','#ab94f7'),path(c.rearImuTopic+'.angular_velocity.z','后雷达','#50c8a4')]),
 'RawMessages!d1imu':{topicPath:c.frontImuTopic,defaultExpanded:false,foxglovePanelTitle:'前雷达 IMU'}
};
const layout={configById,globalVariables:{},userNodes:{'d1-telemetry':{name:'D1 SDK 遥测解析（只读）',sourceCode:script},'d1-calibration':{name:'D1 双雷达显示标定（不发布 ROS）',sourceCode:calibration}},playbackConfig:{speed:1},layout:'Tab!d1workspace'};
await mkdir('layouts',{recursive:true});
await writeFile('layouts/D1Max-ANYmal.json',JSON.stringify(layout,null,2)+'\n');
console.log('Generated layouts/D1Max-ANYmal.json');
