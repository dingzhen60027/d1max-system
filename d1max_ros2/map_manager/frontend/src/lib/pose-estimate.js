// One coordinate convention shared by gesture, arrow rendering and API payload.
// Image Y points down; map Y points up. Map origin may include a yaw rotation.
export function normalizeYaw(yaw) { return Math.atan2(Math.sin(yaw), Math.cos(yaw)) }
function geometry(version) {
  const r = Number(version.metadata.resolution)
  const [x, y, angle = 0] = version.metadata.origin.map(Number)
  const width = Number(version.width), height = Number(version.height)
  if (![r,x,y,angle,width,height].every(Number.isFinite) || r<=0 || width<=0 || height<=0) throw Error('地图坐标信息无效')
  return {r,x,y,angle,width,height,c:Math.cos(angle),s:Math.sin(angle)}
}
export function imageToMap(version, u, v) {
  const g=geometry(version), x=u*g.width*g.r, y=(1-v)*g.height*g.r
  return {x:g.x+g.c*x-g.s*y,y:g.y+g.s*x+g.c*y}
}
export function mapToImage(version, x, y) {
  const g=geometry(version), dx=x-g.x, dy=y-g.y
  return {u:(g.c*dx+g.s*dy)/(g.width*g.r),v:1-(-g.s*dx+g.c*dy)/(g.height*g.r)}
}
export function dragPose(version, start, end) {
  const a=imageToMap(version,start.u,start.v),b=imageToMap(version,end.u,end.v)
  if(Math.hypot(b.x-a.x,b.y-a.y)<1e-8)return null
  return {...a,yaw:normalizeYaw(Math.atan2(b.y-a.y,b.x-a.x))*180/Math.PI}
}
export function arrowGeometry(version, pose, length) {
  const start=mapToImage(version,Number(pose.x),Number(pose.y)),g=geometry(version)
  const angle=Number(pose.yaw)*Math.PI/180-g.angle
  const x=start.u*g.width,y=start.v*g.height
  return {x,y,endX:x+Math.cos(angle)*length,endY:y-Math.sin(angle)*length}
}
