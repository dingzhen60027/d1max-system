import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

// This viewer renders measured points only: no occupancy cells or inferred mesh.
const main = document.querySelector('#views');
const data = new Map();
let report, mode = 'compare', views = [];
const labels = {lower:'下层地面', upper:'上层地面', stairs:'楼梯踏面', all:'双层地面 · 3D'};
const pointSize = () => Number(document.querySelector('#point-size').value);

async function binary(file) {
  const response = await fetch(file);
  if (!response.ok) throw Error(`${file}: ${response.status}`);
  return new Float32Array(await response.arrayBuffer());
}
function nearStairs(x, y) {
  const [lo, hi] = report.stairs_xy_bounds;
  return x >= lo[0]-.6 && x <= hi[0]+.6 && y >= lo[1]-.6 && y <= hi[1]+.6;
}
function planeHeight(plane, x, y) {return -(plane[0]*x+plane[1]*y+plane[3])/plane[2];}
function inRegion(kind, x, y, z) {
  if (report.layers.length === 1) return true;
  if (kind === 'all') return true;
  if (kind === 'stairs') return nearStairs(x,y);
  const midpoint = report.floor_planes.reduce((sum,p)=>sum+planeHeight(p,x,y),0)/2;
  return kind === 'lower' ? z < midpoint : z >= midpoint;
}
function pointGeometry(packed, filter = () => true, colored = true) {
  const pos=[], col=[], rgb=new THREE.Color();
  for(let i=0;i<packed.length;i+=6) {
    if (!filter(packed[i],packed[i+1],packed[i+2])) continue;
    pos.push(packed[i],packed[i+1],packed[i+2]);
    if (colored) {rgb.setRGB(packed[i+3],packed[i+4],packed[i+5],THREE.SRGBColorSpace);col.push(rgb.r,rgb.g,rgb.b);}
  }
  const g=new THREE.BufferGeometry();
  g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));
  if (colored) g.setAttribute('color',new THREE.Float32BufferAttribute(col,3));
  return g;
}
function createView(kind) {
  const container=document.createElement('section');
  container.className='view'+(mode==='compare' && report.layers.length > 1?'':' single');
  const heading=document.createElement('div');heading.className='view-label';
  const title=document.createElement('strong');title.textContent=labels[kind];heading.append(title);
  const count=document.createElement('p');heading.append(count);container.append(heading);
  const host=document.createElement('div');host.className='host';container.append(host);
  const foot=document.createElement('div');foot.className='view-foot';
  foot.textContent=mode==='compare'?'俯视 · 左键平移 · 滚轮缩放':'Z 轴向上 · 左键旋转 · 右键平移 · 滚轮缩放';container.append(foot);main.append(container);
  const scene=new THREE.Scene();scene.background=new THREE.Color('#101722');
  const camera=mode==='compare'?new THREE.OrthographicCamera(-20,20,30,-30,.01,1000):new THREE.PerspectiveCamera(42,1,.02,2000);
  camera.up.set(0,0,1);
  const renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio,1.7));renderer.outputColorSpace=THREE.SRGBColorSpace;host.append(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.screenSpacePanning=true;
  if (mode==='compare') {controls.enableRotate=false;controls.mouseButtons.LEFT=THREE.MOUSE.PAN;}
  const bounds=new THREE.Box3(), materials=[];
  let total=0;
  for(const layer of report.layers) {
    if(kind!=='all' && kind!=='stairs' && kind!==layer.id) continue;
    const g=pointGeometry(data.get(layer.id),kind==='stairs'?nearStairs:undefined);
    if (!g.attributes.position.count) {g.dispose();continue;}
    const material=new THREE.PointsMaterial({vertexColors:true,size:pointSize(),sizeAttenuation:false});materials.push(material);
    scene.add(new THREE.Points(g,material));g.computeBoundingBox();bounds.union(g.boundingBox);total+=g.attributes.position.count;
  }
  count.textContent=`${total.toLocaleString()} 点${kind==='stairs'?' · 含出入口平台':''}`;
  if (bounds.isEmpty()) {bounds.min.set(-1,-1,-1);bounds.max.set(1,1,1);}
  const source=new THREE.Points(pointGeometry(data.get('source'),(x,y,z)=>inRegion(kind,x,y,z),false),new THREE.PointsMaterial({color:'#96a5ba',size:1,sizeAttenuation:false,transparent:true,opacity:.22,depthWrite:false}));scene.add(source);
  const trace=data.get('trajectory'),segments=[];
  for(let i=0;i+5<trace.length;i+=3) {
    if(inRegion(kind,trace[i],trace[i+1],trace[i+2])&&inRegion(kind,trace[i+3],trace[i+4],trace[i+5])) segments.push(...trace.subarray(i,i+6));
  }
  const tg=new THREE.BufferGeometry();tg.setAttribute('position',new THREE.Float32BufferAttribute(segments,3));
  const trajectory=new THREE.LineSegments(tg,new THREE.LineBasicMaterial({color:'#c6d8ff'}));scene.add(trajectory);
  const planned=new THREE.Group();scene.add(planned);
  if(data.has('plan')){
    const packed=data.get('plan'),points=[];
    for(let i=0;i<packed.length;i+=3)points.push(new THREE.Vector3(packed[i],packed[i+1],packed[i+2]+.08));
    class Polyline extends THREE.Curve{getPoint(t,target=new THREE.Vector3()){
      const s=t*(points.length-1),i=Math.min(Math.floor(s),points.length-2);
      return target.copy(points[i]).lerp(points[i+1],s-i);
    }}
    if(points.length>=2){
      planned.add(new THREE.Mesh(new THREE.TubeGeometry(new Polyline(),(points.length-1)*2,.10,6,false),new THREE.MeshBasicMaterial({color:'#ffcd43'})));
      for(const [p,color] of [[points[0],'#54aeff'],[points.at(-1),'#ff6597']]){
        const marker=new THREE.Mesh(new THREE.SphereGeometry(.45,16,12),new THREE.MeshBasicMaterial({color}));marker.position.copy(p);planned.add(marker);
      }
    }
  }
  const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3());
  function render(){renderer.render(scene,camera);}
  function update(){
    source.visible=document.querySelector('#raw').checked;
    trajectory.visible=document.querySelector('#trajectory').checked;
    planned.visible=document.querySelector('#planned-path')?.checked??true;
    materials.forEach(m=>{m.size=pointSize();});render();
  }
  function fit(){
    const aspect=Math.max(host.clientWidth,1)/Math.max(host.clientHeight,1);controls.target.copy(center);
    if(camera.isOrthographicCamera){
      const half=Math.max(size.y/2,size.x/2/aspect)*1.20;
      Object.assign(camera,{left:-half*aspect,right:half*aspect,top:half,bottom:-half,zoom:1});
      camera.up.set(0,1,0);camera.position.set(center.x,center.y,bounds.max.z+80);
    }else{
      camera.aspect=aspect;
      const direction=(kind==='stairs'?new THREE.Vector3(.45,-1,.65):new THREE.Vector3(1.45,-.85,.7)).normalize();
      const right=direction.clone().negate().cross(camera.up).normalize(),up=right.clone().cross(direction.clone().negate()).normalize();
      const tanV=Math.tan(THREE.MathUtils.degToRad(camera.fov/2)),tanH=tanV*aspect;
      let distance=1;
      for(const x of [bounds.min.x,bounds.max.x])for(const y of [bounds.min.y,bounds.max.y])for(const z of [bounds.min.z,bounds.max.z]){
        const p=new THREE.Vector3(x,y,z).sub(center),near=p.dot(direction);
        distance=Math.max(distance,Math.abs(p.dot(right))/tanH+near,Math.abs(p.dot(up))/tanV+near);
      }
      camera.position.copy(center).addScaledVector(direction,distance*1.17);
    }
    camera.updateProjectionMatrix();controls.update();render();
  }
  const resize=new ResizeObserver(()=>{renderer.setSize(host.clientWidth,host.clientHeight,false);fit();});resize.observe(host);
  controls.addEventListener('change',render);update();
  return {fit,update,dispose(){resize.disconnect();controls.dispose();scene.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});renderer.dispose();renderer.forceContextLoss();container.remove();}};
}
function setMode(next){
  mode=next;views.forEach(v=>v.dispose());views=[];main.replaceChildren();
  for(const kind of mode==='compare'?report.layers.filter(l=>l.id!=='stairs').map(l=>l.id):[mode==='stairs'?'stairs':'all']) views.push(createView(kind));
  document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('active',b.dataset.mode===mode);b.setAttribute('aria-pressed',String(b.dataset.mode===mode));});
  window.__TRAV_VIEW_MODE__=mode;
}
document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=()=>{if(report)setMode(b.dataset.mode);});
document.querySelector('#reset').onclick=()=>views.forEach(v=>v.fit());
for(const id of ['raw','trajectory'])document.getElementById(id).onchange=()=>views.forEach(v=>v.update());
document.querySelector('#point-size').oninput=()=>{document.querySelector('#point-size-value').textContent=`${pointSize()} px`;views.forEach(v=>v.update());};
const dialog=document.querySelector('#details');
document.querySelector('#detail-toggle').onclick=()=>dialog.showModal();
document.querySelector('#detail-close').onclick=()=>dialog.close();
try {
  const response=await fetch('report.json');if(!response.ok)throw Error(`report: ${response.status}`);report=await response.json();
  if(report.kind!=='ground_only')throw Error('此页面只接受地面提取结果');
  if(report.layers.length===1){
    labels[report.layers[0].id]='提取地面';labels.all='提取地面 · 3D';
    document.title='D1 Max · 地面点云';document.querySelector('h1').textContent='D1 Max · 地面点云';
    document.querySelector('.subtitle').textContent=report.name;
    document.querySelector('[data-mode="compare"]').textContent='地面俯视';
    document.querySelector('[data-mode="stairs"]').hidden=true;
    document.querySelector('.legend').textContent='绿色：提取地面 · 灰色：回环优化地图';
    mode='all';
  }
  await Promise.all([...report.layers.map(l=>[l.id,l.file]),['source','source.bin'],['trajectory','recorded_trajectory.bin']].map(async([id,file])=>data.set(id,await binary(file))));
  if(report.planning){
    data.set('plan',await binary(report.planning.file));
    const label=document.createElement('label'),toggle=document.createElement('input');toggle.type='checkbox';toggle.id='planned-path';toggle.checked=true;
    label.append(toggle,document.createTextNode('规划路径'));document.querySelector('.toggles').prepend(label);
    toggle.onchange=()=>views.forEach(v=>v.update());
    document.querySelector('.legend').textContent=report.planning.smooth?'黄线：平滑路径 · 蓝点：起点 · 粉点：终点':'黄线：A* 路径 · 蓝点：起点 · 粉点：终点';
  }
  document.querySelector('#point-size').value=report.display_point_size_px;
  document.querySelector('#point-size-value').textContent=`${pointSize()} px`;
  document.querySelector('#footer').textContent=`${report.ground_points.toLocaleString()} 点 · 原始坐标 · 未补洞`;
  if(report.planning)document.querySelector('#footer').textContent=`${report.planning.length_m.toFixed(1)} m · 离线规划测试 · 不控制机器人`;
  const detail=document.querySelector('#detail-body'),description=document.createElement('p');
  description.textContent='仅筛选实测地面点，不计算机器人通行性。点大小只影响显示，不改变导出的 PCD；未拉平、未补洞。';detail.append(description);
  const downloads=document.createElement('div');downloads.className='downloads';detail.append(downloads);
  for(const [file,label] of (report.downloads || [['ground.pcd','完整地面 PCD'],['lower_ground.pcd','下层 PCD'],['upper_ground.pcd','上层 PCD'],['stairs_ground.pcd','楼梯 PCD'],['ground_colored.ply','分层彩色 PLY'],['ground_points.npz','点云数据 NPZ'],['pipeline.yaml','处理配置'],['report.json','检查报告'],['two_floors_overview.png','双层总览图']])){
    const link=document.createElement('a');link.href=file;link.download=file;link.textContent=label;downloads.append(link);
  }
  setMode(mode);window.__TRAV_READY__=true;window.__TRAV_REPORT__=report;
}catch(error){main.replaceChildren();const message=document.createElement('div');message.className='error';message.textContent=`地图加载失败：${error.message}`;main.append(message);console.error(error);}
