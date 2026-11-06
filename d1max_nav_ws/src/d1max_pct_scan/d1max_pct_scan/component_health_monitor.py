"""Observe real callback progress; the BT remains the sole task owner."""
import json
from pathlib import Path
import time
from .component_health import FunctionalHealth,required_components,advisory_components

PREFIX='/d1max/live_planning/'


def main(args=None):
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.node import Node
    from std_msgs.msg import String
    class Monitor(Node):
        def __init__(self):
            super().__init__('d1max_component_health')
            sid=self.declare_parameter('session_id','').value
            output=self.declare_parameter('output_directory','').value
            scope=self.declare_parameter('planning_scope','navigation').value
            components=required_components(motion_pipeline=self.declare_parameter('motion_pipeline',True).value,
                local_executor=self.declare_parameter('local_executor_enabled',False).value,planning_scope=scope)
            self.health=FunctionalHealth(sid,components,advisory=advisory_components(planning_scope=scope),started=time.monotonic(),
                startup_s=self.declare_parameter('startup_timeout_s',30.).value,
                timeout_s=self.declare_parameter('callback_timeout_s',1.5).value,
                stable_duration_s=self.declare_parameter('stable_duration_s',.6).value,
                minimum_new_samples=self.declare_parameter('minimum_new_samples',3).value)
            self.directory=Path(output).resolve(strict=True)
            self.publisher=self.create_publisher(String,PREFIX+'component_health',1)
            topics=dict(global_='global_status',reference='scan_bridge_status',perception='ray_projector_status',
                map='rays_status',tracker='tracker_status',safety='execution/safety_status')
            # Keep relay diagnostics, but only core process activity contributes
            # to BT readiness. Per-component observations let the BT select its
            # actual phase (global work vs committed-route following); this
            # monitor does not infer task phase or add a recovery wait. Native
            # SCAN evidence expiry remains authoritative.
            for name in self.health.components:
                topic=topics['global_' if name=='global' else name]
                self.create_subscription(String,PREFIX+topic,
                    lambda m,n=name:self.observe(n,m),1)
            self.create_timer(.2,self.tick)
        def observe(self,name,message):
            if len(message.data)>128*1024:return
            try:value=json.loads(message.data)
            except (ValueError,TypeError):return
            self.health.observe(name,value,wall_now=time.time(),monotonic=time.monotonic())
        def tick(self):
            record=self.health.snapshot(wall_now=time.time(),monotonic=time.monotonic())
            encoded=json.dumps(record,allow_nan=False)
            self.publisher.publish(String(data=encoded))
            from .live_view_reload import write_owned_json
            write_owned_json(self.directory/'component_health.json',record)
    rclpy.init(args=args)
    node=Monitor()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError as error:
            # Humble's signal thread can close the context after the internal
            # check. Only that already-completed shutdown is benign.
            if rclpy.ok() or 'rcl_shutdown already called' not in str(error):
                raise


if __name__=='__main__':main()
