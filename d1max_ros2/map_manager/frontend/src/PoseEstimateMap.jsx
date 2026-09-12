import {useEffect,useId,useRef,useState} from 'react'
import {arrowGeometry,dragPose,imageToMap} from './lib/pose-estimate'

// RViz-style gesture. This component never calls an API, publishes ROS or moves a robot.
export default function PoseEstimateMap({version,pose,picked,disabled,onCommit,onError}) {
 const markerId='pose-arrow-'+useId().replaceAll(':','')
 const gesture=useRef(null),surface=useRef(null)
 const [draft,setDraft]=useState(null),[pixelWidth,setPixelWidth]=useState(640)
 useEffect(()=>{const observer=new ResizeObserver(entries=>setPixelWidth(entries[0].contentRect.width));observer.observe(surface.current);return()=>observer.disconnect()},[])
 const eventPoint=e=>{const b=surface.current.getBoundingClientRect();return {u:(e.clientX-b.left)/b.width,v:(e.clientY-b.top)/b.height,x:e.clientX,y:e.clientY,width:b.width}}
 const cancel=()=>{gesture.current=null;setDraft(null)}
 function down(e){
  if(disabled||e.button!==0||e.isPrimary===false)return
  const p=eventPoint(e);if(p.u<0||p.u>1||p.v<0||p.v>1)return
  e.preventDefault();surface.current.focus({preventScroll:true});surface.current.setPointerCapture(e.pointerId)
  gesture.current={...p,id:e.pointerId};setPixelWidth(p.width)
  setDraft({...pose,...imageToMap(version,p.u,p.v)})
 }
 function move(e){
  const start=gesture.current;if(!start||start.id!==e.pointerId||disabled)return
  const next=dragPose(version,start,eventPoint(e));if(next)setDraft({...pose,...next})
 }
 function up(e){
  const start=gesture.current;if(!start||start.id!==e.pointerId)return
  const end=eventPoint(e),next=dragPose(version,start,end),complete=!!next&&Math.hypot(end.x-start.x,end.y-start.y)>=12
  gesture.current=null;setDraft(null)
  if(surface.current.hasPointerCapture(e.pointerId))surface.current.releasePointerCapture(e.pointerId)
  if(!disabled)onCommit({...pose,...(complete?next:imageToMap(version,start.u,start.v))},complete)
 }
 const shown=draft||(picked?pose:null),length=version.width*72/Math.max(1,pixelWidth)
 const arrow=shown?arrowGeometry(version,shown,length):null
 return <div className="pose-estimate-canvas" ref={surface} role="application" aria-label="初始位姿工具：按下选择位置，拖动指定机头方向，松开提交；Esc 取消" aria-disabled={disabled} tabIndex={0}
   onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={cancel} onLostPointerCapture={cancel} onKeyDown={e=>{if(e.key==='Escape')cancel()}}>
  <img alt="初始定位地图" src={version.map_preview_url} draggable={false} onDragStart={e=>e.preventDefault()} onError={onError} onLoad={()=>setPixelWidth(surface.current?.clientWidth||640)}/>
  <svg className="pose-estimate-overlay" viewBox={`0 0 ${version.width} ${version.height}`} aria-hidden="true">
   <defs><marker id={markerId} viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M 0 0 L 10 5 L 0 10 z" fill="currentColor"/></marker></defs>
   {arrow&&<g className={draft?'pose-estimate-arrow is-dragging':'pose-estimate-arrow'} data-testid="initial-pose-arrow" data-yaw={Number(shown.yaw).toFixed(3)}>
    <circle cx={arrow.x} cy={arrow.y} r={length/12}/>
    <line x1={arrow.x} y1={arrow.y} x2={arrow.endX} y2={arrow.endY} markerEnd={`url(#${markerId})`} strokeWidth={length/24}/>
   </g>}
  </svg>
  {draft&&<output className="pose-estimate-readout">{Number(draft.x).toFixed(2)}, {Number(draft.y).toFixed(2)} m · {Number(draft.yaw).toFixed(1)}°</output>}
 </div>
}
