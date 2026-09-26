import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const main = document.querySelector('#views');
const data = new Map();
let report, mode = 'compare', views = [];
const labels = { lower: '下层', upper: '上层', stairs: '楼梯连接区域' };

async function binary(file) {
  const response = await fetch(file);
  if (!response.ok) throw Error(`${file}: ${response.status}`);
  return new Float32Array(await response.arrayBuffer());
}
function surfaceGeometry(packed) {
  const n = packed.length/6, pos = new Float32Array(n*18), col = new Float32Array(n*18), r = report.resolution_m*.49;
  const corners = [[-r,-r],[r,-r],[r,r],[-r,-r],[r,r],[-r,r]], rgb=new THREE.Color();
  for(let i=0;i<n;i++) {rgb.setRGB(packed[i*6+3],packed[i*6+4],packed[i*6+5],THREE.SRGBColorSpace);for(let j=0;j<6;j++) {
    const p=i*18+j*3, s=i*6;
    pos[p]=packed[s]+corners[j][0]; pos[p+1]=packed[s+1]+corners[j][1]; pos[p+2]=packed[s+2];
    col[p]=rgb.r; col[p+1]=rgb.g; col[p+2]=rgb.b;
  }}
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setAttribute('color',new THREE.BufferAttribute(col,3));return g;
}
function sourceGeometry(packed, kind) {
  let selected=[];
  for(let i=0;i<packed.length;i+=6) {
    const x=packed[i], y=packed[i+1], z=packed[i+2];
    if(kind==='stairs' && (x<8.5||x>22||y<9||y>17.5)) continue;
    if(kind==='lower' && z>-1.9) continue;
    if(kind==='upper' && z<-1.95) continue;
    selected.push(x,y,z);
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(selected,3));return g;
}
function createView(kind) {
  const container=document.createElement('section');container.className='view'+(mode==='compare'?'':' single');
  const layer=report.layers.find(x=>x.id===kind);
  const area=layer?(layer.counts.free*report.resolution_m**2).toFixed(1):report.geometric_green_area_m2.toFixed(1);
  container.innerHTML=`<div class="view-label"><strong>${labels[kind]||'双层 · 原始三维位置'}</strong><p>${kind==='stairs'?'只显示几何候选；未确认连续通行':`绿色区域 ${area} m²`}</p></div><div class="host"></div><div class="view-foot">${mode==='compare'?'左键平移 · 滚轮缩放':'左键旋转 · 右键平移 · 滚轮缩放'}</div><div class="scale">Z 轴向上 · 单位 m</div>`;
  main.appendChild(container);
  const host=container.querySelector('.host'),scene=new THREE.Scene();scene.background=new THREE.Color('#101722');
  const camera=mode==='compare'?new THREE.OrthographicCamera(-20,20,30,-30,.01,1000):new THREE.PerspectiveCamera(42,1,.02,2000);
  camera.up.set(0,0,1);
  const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio,1.7));renderer.outputColorSpace=THREE.SRGBColorSpace;host.appendChild(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=false;controls.screenSpacePanning=true;
  if(mode==='compare'){controls.enableRotate=false;controls.mouseButtons.LEFT=THREE.MOUSE.PAN;}
  const bounds=new THREE.Box3();
  for(const id of ['lower','upper','stairs']) {
    if(kind!=='all' && id!==kind) continue;
    const g=surfaceGeometry(data.get(id)); const mesh=new THREE.Mesh(g,new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide}));
    scene.add(mesh);g.computeBoundingBox();bounds.union(g.boundingBox);
  }
  const source=new THREE.Points(sourceGeometry(data.get('source'),kind),new THREE.PointsMaterial({color:'#8494ab',size:.055,transparent:true,opacity:.27,depthWrite:false}));
  source.visible=document.querySelector('#raw').checked;scene.add(source);
  const trajectoryData=data.get('trajectory'), tp=[];
  for(let i=0;i<trajectoryData.length;i+=3) {
    const x=trajectoryData[i],y=trajectoryData[i+1],z=trajectoryData[i+2];
    const visible=kind==='all'||(kind==='upper'&&z>-1.6)||(kind==='lower'&&z<-3.8)||(kind==='stairs'&&x>9&&x<22&&y>9&&y<18);
    // Segments only: never connect a removed section with a false straight line.
    if(i+3<trajectoryData.length&&visible) {
      const nx=trajectoryData[i+3],ny=trajectoryData[i+4],nz=trajectoryData[i+5];
      const next=kind==='all'||(kind==='upper'&&nz>-1.6)||(kind==='lower'&&nz<-3.8)||(kind==='stairs'&&nx>9&&nx<22&&ny>9&&ny<18);
      if(next)tp.push(x,y,z,nx,ny,nz);
    }
  }
  const trajG=new THREE.BufferGeometry();trajG.setAttribute('position',new THREE.Float32BufferAttribute(tp,3));
  const trajectory=new THREE.LineSegments(trajG,new THREE.LineBasicMaterial({color:'#c6d8ff',transparent:true,opacity:.9}));
  trajectory.visible=document.querySelector('#trajectory').checked;scene.add(trajectory);
  const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3());
  const grid=new THREE.GridHelper(kind==='stairs'?18:70,kind==='stairs'?18:35,0x28374d,0x1a2638);grid.rotation.x=Math.PI/2;grid.position.set(center.x,center.y,bounds.min.z-.22);scene.add(grid);
  function render(){renderer.render(scene,camera);}
  function fit(){
    const aspect=Math.max(host.clientWidth,1)/Math.max(host.clientHeight,1);controls.target.copy(center);
    if(camera.isOrthographicCamera){const half=Math.max(size.y/2,size.x/2/aspect)*1.10;camera.left=-half*aspect;camera.right=half*aspect;camera.top=half;camera.bottom=-half;camera.zoom=1;camera.up.set(0,1,0);camera.position.set(center.x,center.y,bounds.max.z+80);}
    else{
      camera.aspect=aspect;
      const direction=(kind==='stairs'?new THREE.Vector3(.45,-1,.65):new THREE.Vector3(1.45,-.85,.7)).normalize();
      const right=direction.clone().negate().cross(camera.up).normalize(),up=right.clone().cross(direction.clone().negate()).normalize();
      const tanV=Math.tan(THREE.MathUtils.degToRad(21)),tanH=tanV*aspect;
      let distance=1;
      for(const x of [bounds.min.x,bounds.max.x])for(const y of [bounds.min.y,bounds.max.y])for(const z of [bounds.min.z,bounds.max.z]){
        const p=new THREE.Vector3(x,y,z).sub(center),near=p.dot(direction);
        distance=Math.max(distance,Math.abs(p.dot(right))/tanH+near,Math.abs(p.dot(up))/tanV+near);
      }
      camera.position.copy(center).addScaledVector(direction,distance*1.13);
    }
    camera.updateProjectionMatrix();controls.update();render();
  }
  let initial=true;
  const resize=new ResizeObserver(()=>{renderer.setSize(host.clientWidth,host.clientHeight,false);if(initial){fit();initial=false}else{fit()}});resize.observe(host);
  controls.addEventListener('change',render);
  return {fit,update(){source.visible=document.querySelector('#raw').checked;trajectory.visible=document.querySelector('#trajectory').checked;render();},dispose(){resize.disconnect();controls.dispose();scene.traverse(o=>{o.geometry?.dispose();if(o.material)Array.isArray(o.material)?o.material.forEach(m=>m.dispose()):o.material.dispose()});renderer.dispose();container.remove();}};
}
function setMode(next){
  mode=next;views.forEach(v=>v.dispose());views=[];
  for(const kind of mode==='compare'?['lower','upper']:[mode==='stairs'?'stairs':'all'])views.push(createView(kind));
  document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===mode));
  window.__TRAV_VIEW_MODE__=mode;
}
document.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>setMode(b.dataset.mode)));
document.querySelector('#reset').onclick=()=>views.forEach(v=>v.fit());
for(const id of ['raw','trajectory'])document.getElementById(id).onchange=()=>views.forEach(v=>v.update());
document.querySelector('#detail-toggle').onclick=()=>document.querySelector('#details').classList.toggle('open');
document.querySelector('#detail-close').onclick=()=>document.querySelector('#details').classList.remove('open');
try{
  const response=await fetch('report.json');if(!response.ok)throw Error(`report: ${response.status}`);report=await response.json();
  await Promise.all([...report.layers.map(l=>[l.id,l.file]),['source','source.bin'],['trajectory','recorded_trajectory.bin']].map(async([id,file])=>data.set(id,await binary(file))));
  document.querySelector('#stats').innerHTML=`<span><b>2</b>楼层</span><span><b>${report.resolution_m*100}</b>cm</span>`;
  const connected=(report.graph.final_cross_floor_components||[]).length;
  const oriented=report.footprint_mode==='oriented_rectangle', robot=report.robot;
  if(oriented)document.querySelector('.subtitle').textContent='SC-PGO 优化地图 · 09-04 14:34 · 按机身宽度与朝向评估';
  document.querySelector('#footer').textContent=`${oriented?'绿色需遵守允许朝向 · ':''}${connected?'已发现几何连接，楼梯仍需验证':'楼梯尚未确认连续可通行'} · 灰色与空白不是自由空间 · 无补洞`;
  document.querySelector('#detail-body').innerHTML=`<p>源地图：${report.input_points.toLocaleString()} 点<br>站立包络：930 × 480 × 585 mm<br>水平余量：${Math.round(robot.horizontal_margin_m*1000)} mm / 侧 · 垂直余量：${Math.round(robot.vertical_margin_m*1000)} mm<br>${oriented?`矩形朝向包络：${robot.heading_bins_half_turn} 个方向<br>宽度模型：${Math.round(report.nominal_passage_width_m*100)} cm（含两侧余量，不含栅格误差）<br>坡面候选上限：${robot.candidate_slope_max_deg}° · 台阶候选上限：${Math.round(robot.step_candidate_max_m*100)} cm`:`全方向保守半径：${report.conservative_body_radius_m.toFixed(3)} m`}<br>已接受回环：0</p><p>${report.warnings.join('<br>')}</p><p><a href="traversability.pcd" download>彩色 PCD</a><a href="traversability.ply" download>PLY</a><a href="traversability.npz" download>规划数据 NPZ</a><a href="pipeline.yaml" download>处理配置</a><a href="report.json" download>检查报告</a><a href="two_floors_overview.png" target="_blank">双层总览图</a></p>`;
  setMode(mode);window.__TRAV_READY__=true;window.__TRAV_REPORT__=report;
}catch(error){main.innerHTML=`<div class="error">地图加载失败：${error.message}</div>`;console.error(error);}
