"""Observe real callback progress; the BT remains the sole task owner."""
import json
from pathlib import Path
import time
from .component_health import FunctionalHealth, create_functional_heartbeat_timer

PREFIX='/d1max/live_planning/'


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    class Monitor(Node):
        def __init__(self):
            super().__init__('d1max_component_health')
            sid=self.declare_parameter('session_id','').value
            output=self.declare_parameter('output_directory','').value
            components=['global','reference','perception','map','tracker']
            if self.declare_parameter('motion_pipeline',True).value:components.append('safety')
            self.health=FunctionalHealth(sid,components,started=time.monotonic(),
                startup_s=self.declare_parameter('startup_timeout_s',30.).value)
            self.directory=Path(output).resolve(strict=True)
            self.publisher=self.create_publisher(String,PREFIX+'component_health',1)
            topics=dict(global_='global_status',reference='scan_bridge_status',perception='ray_projector_status',
                map='rays_status',tracker='tracker_status',safety='execution/safety_status')
            for name in components:
                topic=topics['global_' if name=='global' else name]
                self.create_subscription(String,PREFIX+topic,
                    lambda m,n=name:self.observe(n,m),1)
            self.heartbeat_timer=create_functional_heartbeat_timer(self,self.tick)
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
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
