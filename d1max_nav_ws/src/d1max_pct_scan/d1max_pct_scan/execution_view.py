"""Read-only v3 committed-curve presentation; never an admission publisher."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from .atomic_projection_state import stamp_ns


def curve_key(m, *, tagged=False):
    return (m.session_id,m.task_id,m.route_id,m.route_hash,m.map_version_id,
        m.localization_epoch,m.localization_seed_id,
        m.generation if tagged else m.reference_generation,m.segment_id,
        m.anchor_id,m.anchor_revision,m.context_sequence,m.map_geometry_revision)


class CommittedView:
    def __init__(self,session,mode):
        self.session,self.mode=session,mode
        self.permit=None;self.curves={};self.proofs={}

    def permit_received(self,m):
        if (m.version.schema_version!=3 or m.version.session_id!=self.session or m.transport_mode!=self.mode
                or self.permit is not None and m.sequence<=self.permit.sequence):return
        self.permit=deepcopy(m)

    def curve_received(self,m):
        if m.session_id!=self.session or m.schema_version!=2:return
        key=(curve_key(m,tagged=True),m.trajectory.traj_id)
        self.curves[key]=deepcopy(m)
        if len(self.curves)>8:del self.curves[next(iter(self.curves))]

    def proof_received(self,m):
        if m.version.session_id!=self.session or m.transport_mode!=self.mode:return
        key=(curve_key(m.version),m.trajectory_id)
        old=self.proofs.get(key)
        if old is not None and old.sequence>=m.sequence:return
        self.proofs[key]=deepcopy(m)
        if len(self.proofs)>8:del self.proofs[next(iter(self.proofs))]

    def selected(self,now):
        p=self.permit
        if p is None or p.revoked or not p.geometry_committed or p.frame_id!='d1max_loc_odom':return None
        key=(curve_key(p.version),p.trajectory_id)
        curve=self.curves.get(key)
        if curve is None or curve.frame_id!='d1max_loc_odom':return None
        # This layer is explicitly "committed geometry", not a safe-to-drive
        # claim. HOLD keeps the same shape while the task panel reports HOLD;
        # proof expiry still blocks control in the independent safety node.
        until=stamp_ns(p.source_stamp)+350_000_000
        if not 0<=now-stamp_ns(p.source_stamp) or until<=now:return None
        return curve,p,(until-now)*1e-9


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from d1max_planning_interfaces.msg import ExecutionPermit,TrajectoryValidation,TaggedBspline,LocalPlanDebug
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker,MarkerArray
    from .scan_visual_style import official_spline_style
    from .local_debug import local_debug_specs
    parser=argparse.ArgumentParser();parser.add_argument('--session',type=Path,required=True)
    args,rosargs=parser.parse_known_args();s=json.loads((args.session/'session.json').read_text())
    if s.get('pipeline_contract')!='single_floor_v3':raise ValueError('v3_session_required')
    prefix='/d1max/live_planning/'
    class View(Node):
        def __init__(self):
            super().__init__('single_floor_execution_view');self.core=CommittedView(s['id'],s['transport_mode'])
            self.debug=None;self.shown=False
            qos=QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path=self.create_publisher(Marker,prefix+'scan_optimal',qos)
            self.details=self.create_publisher(MarkerArray,prefix+'local_debug',qos)
            self.create_subscription(ExecutionPermit,prefix+'execution/permit',self.core.permit_received,5)
            self.create_subscription(TrajectoryValidation,prefix+'execution/validation',self.core.proof_received,5)
            self.create_subscription(TaggedBspline,prefix+'committed_bspline',self.core.curve_received,qos)
            self.create_subscription(LocalPlanDebug,prefix+'native_local_debug',self.on_debug,5)
            self.create_timer(.1,self.tick)
        def on_debug(self,m):
            if m.session_id==s['id'] and (self.debug is None or stamp_ns(m.header.stamp)>stamp_ns(self.debug.header.stamp)):
                self.debug=m
        def tick(self):
            now=self.get_clock().now().nanoseconds;selected=self.core.selected(now)
            if selected is None:
                if self.shown:
                    marker=Marker(ns='committed_curve',id=0,action=Marker.DELETE)
                    self.path.publish(marker);self.shown=False
                return
            curve,permit,lease=selected;raw=curve.trajectory
            try:style=official_spline_style(order=raw.order,knots=raw.knots,points=[(p.x,p.y,p.z) for p in raw.pos_pts])
            except ValueError:return
            m=Marker(ns='committed_curve',id=0,type=Marker.LINE_STRIP,action=Marker.ADD)
            m.header.frame_id=curve.frame_id;m.header.stamp=permit.source_stamp
            m.pose.orientation.w=1.;m.scale.x=.035;m.color.a=1.
            m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in style['points']]
            m.colors=[ColorRGBA(r=float(r),g=float(g),b=float(b),a=float(a)) for r,g,b,a in style['colors']]
            m.lifetime=Duration(seconds=lease).to_msg();self.path.publish(m);self.shown=True
            d=self.debug
            if (d is None or not d.valid or d.generation!=curve.generation or d.plan_id!=raw.traj_id
                    or d.header.frame_id!=curve.frame_id or not 0<=now-stamp_ns(d.header.stamp)<=400_000_000):return
            try:specs=local_debug_specs(selected_reference=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in d.selected_reference.poses],
                projection=(d.projection.x,d.projection.y,d.projection.z),local_target=(d.local_target.x,d.local_target.y,d.local_target.z))
            except ValueError:return
            markers=[]
            for spec in specs:
                marker=Marker(ns=spec['namespace'],id=0,type=getattr(Marker,spec['kind']),action=Marker.ADD)
                marker.header=d.header;marker.pose.orientation.w=1.;marker.lifetime=m.lifetime
                marker.color=ColorRGBA(**dict(zip(('r','g','b','a'),spec['color'])))
                marker.scale.x=marker.scale.y=marker.scale.z=spec['width']
                if spec['kind']=='SPHERE':marker.pose.position=Point(**dict(zip(('x','y','z'),spec['position'])))
                else:marker.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in spec['points']]
                markers.append(marker)
            self.details.publish(MarkerArray(markers=markers))
    rclpy.init(args=rosargs);node=View()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
