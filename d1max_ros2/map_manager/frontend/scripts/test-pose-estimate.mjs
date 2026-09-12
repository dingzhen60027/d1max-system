import test from 'node:test';import assert from 'node:assert/strict';
import {imageToMap,mapToImage,dragPose,arrowGeometry} from '../src/lib/pose-estimate.js';
const v={width:1000,height:500,metadata:{resolution:.05,origin:[-20,-10,0]}};
test('all cardinal arrow directions match the actual map yaw, without a 90 degree offset',()=>{
 for(const [end,yaw,dx,dy] of [[{u:.7,v:.5},0,1,0],[{u:.5,v:.2},90,0,-1],[{u:.2,v:.5},180,-1,0],[{u:.5,v:.8},-90,0,1]]){
  const p=dragPose(v,{u:.5,v:.5},end);assert(Math.abs(p.yaw-yaw)<1e-8);
  const a=arrowGeometry(v,p,100);assert(Math.abs(a.endX-a.x-dx*100)<1e-8);assert(Math.abs(a.endY-a.y-dy*100)<1e-8);
 }
});
test('rotated map origins work identically for pick, yaw, and rendering',()=>{
 const rotated={...v,metadata:{...v.metadata,origin:[12,-7,Math.PI/3]}};
 for(const [u,w] of [[.2,.3],[0,1],[1,0]]){const p=imageToMap(rotated,u,w),q=mapToImage(rotated,p.x,p.y);assert(Math.abs(q.u-u)<1e-9);assert(Math.abs(q.v-w)<1e-9)}
 const p=dragPose(rotated,{u:.5,v:.5},{u:.7,v:.5});assert(Math.abs(p.yaw-60)<1e-8);const a=arrowGeometry(rotated,p,50);assert(Math.abs(a.endX-a.x-50)<1e-8);assert(Math.abs(a.endY-a.y)<1e-8);
});
test('non-square image respects metric aspect ratio and a click has no new heading',()=>{
 const p=dragPose(v,{u:.5,v:.5},{u:.6,v:.4});assert(Math.abs(p.yaw-Math.atan(.5)*180/Math.PI)<1e-8);
 assert.equal(dragPose(v,{u:.5,v:.5},{u:.5,v:.5}),null);
});
