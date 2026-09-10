// Lightweight LIVE layout: lidar + image surfaces, no robot meshes or model server.
import {readFile,writeFile} from 'node:fs/promises';
const layout=JSON.parse(await readFile('layouts/D1Max-ANYmal.json','utf8'));
const c=JSON.parse(await readFile('config/layout.json','utf8'));
const cameras=JSON.parse(await readFile('config/cameras.json','utf8'));
// Unit quaternion point rotation; source is the existing mapping calibration.
function rotate(q,v){
 const [x,y,z,w]=q,[a,b,d]=v;
 const tx=2*(y*d-z*b),ty=2*(z*a-x*d),tz=2*(x*b-y*a);
 return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,d+w*tz+x*ty-y*tx];
}
const baseline=rotate(c.frontRotation,c.rearTranslation);
const yaw=Math.atan2(-baseline[1],-baseline[0]);
const bodyRotation=[0,0,Math.sin(yaw/2),Math.cos(yaw/2)];
const bodyTranslation=rotate(bodyRotation,cameras.frontLidarMountXYZ).map(n=>-n);
const transform={header:{frame_id:c.displayFrame},child_frame_id:cameras.rigFrame,
 transform:{translation:Object.fromEntries(['x','y','z'].map((k,i)=>[k,bodyTranslation[i]])),rotation:Object.fromEntries(['x','y','z','w'].map((k,i)=>[k,bodyRotation[i]]))}};
const calibration=layout.userNodes['d1-calibration'];
calibration.name='D1 双雷达与相机显示标定（不发布 ROS）';
calibration.sourceCode=calibration.sourceCode.replace('return {transforms:[',
 'return {transforms:[\n    {...'+JSON.stringify(transform)+',header:{stamp,frame_id:'+JSON.stringify(c.displayFrame)+'}},');
// Camera TF is display-local too. Use camera timestamps even with no lidar visible.
const imageInputs=[cameras.front.imageTopic,cameras.rear.imageTopic];
calibration.sourceCode=calibration.sourceCode.replace(/export const inputs = \[.*?\];/,
 'export const inputs = '+JSON.stringify([c.frontCloudTopic,c.rearCloudTopic,...imageInputs])+';');
calibration.sourceCode=calibration.sourceCode.replace('):Message<',
 imageInputs.map(t=>'|Input<'+JSON.stringify(t)+'>').join('')+'):Message<');
for(const camera of [cameras.front,cameras.rear]){
 const t={child_frame_id:camera.frame,transform:{translation:Object.fromEntries(['x','y','z'].map((k,i)=>[k,camera.mountXYZ[i]])),rotation:Object.fromEntries(['x','y','z','w'].map((k,i)=>[k,camera.opticalRotation[i]]))}};
 calibration.sourceCode=calibration.sourceCode.replace('return {transforms:[',
  'return {transforms:[\n    {...'+JSON.stringify(t)+',header:{stamp,frame_id:'+JSON.stringify(cameras.rigFrame)+'}},');
}
const cameraTemplate=await readFile('config/scripts/camera-calibration.ts','utf8');
for(const key of ['front','rear']){
 const camera=cameras[key];
 layout.userNodes['d1-camera-'+key]={name:(key==='front'?'前':'后')+'相机 · 标称视场近似 / 仅显示',
  sourceCode:cameraTemplate.replaceAll('__IMAGE_TOPIC__',camera.imageTopic)
   .replaceAll('__CALIBRATION_TOPIC__',camera.calibrationTopic)
   .replace('__CAMERA_CONFIG__',JSON.stringify(camera))};
}
for(const id of ['3D!d1scene','3D!d1inspect']){
 const p=layout.configById[id];p.scene.transforms={showLabel:false,axisSize:.12};
 p.cameraState.targetOffset=[...bodyTranslation];
 p.topics[c.frontCloudTopic].opacity=.72;p.topics[c.rearCloudTopic].opacity=.72;
}
// One perception scene keeps lidar and camera surfaces together.
const perception=layout.configById['3D!d1scene'];
perception.foxglovePanelTitle='双雷达 + 前后图像 · 近似投影';
perception.cameraState.distance=cameras.viewDistance;
perception.cameraState.phi=cameras.viewPolarAngleDeg;
perception.cameraState.thetaOffset=cameras.viewAzimuthDeg;
const imageAlpha=Math.round(Math.min(1,Math.max(0,cameras.imageOpacity??.72))*255).toString(16).padStart(2,'0');
for(const camera of [cameras.front,cameras.rear]){
 perception.topics[camera.imageTopic]={visible:true,frameLocked:true,cameraInfoTopic:camera.calibrationTopic,
  distance:cameras.projectionDistance,planarProjectionFactor:cameras.planarProjectionFactor,renderMode:'default',rectifyImage:false,color:'#ffffff'+imageAlpha};
 perception.topics[camera.calibrationTopic]={visible:false,distance:cameras.projectionDistance,planarProjectionFactor:0};
}
const tabs=layout.configById['Tab!d1workspace'];
tabs.activeTabIdx=0;
await writeFile('layouts/D1Max-Live.json',JSON.stringify(layout,null,2)+'\n');
console.log('Generated lightweight live layout: no URDF/STL assets.');
