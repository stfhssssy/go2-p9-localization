#!/usr/bin/env python3
"""Publish the same explicit camera_init rectangle enforced by the command gate."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker
from guard_core import GuardConfig


class Boundary(Node):
    def __init__(self):
        super().__init__('p9_exploration_boundary')
        values = {}
        for key in ('min_x', 'max_x', 'min_y', 'max_y'):
            self.declare_parameter(key, float('nan'))
            values[key] = self.get_parameter(key).value
        self.cfg = GuardConfig(**values)
        self.pubs = [self.create_publisher(PolygonStamped, t, 10) for t in
                     ['/p9/exploration/navigation_boundary', '/p9/exploration/coverage_boundary']]
        self.vis = self.create_publisher(Marker, '/p9/exploration/boundary_marker', 10)
        self.create_timer(1., self.tick)

    def tick(self):
        c = self.cfg
        msg = PolygonStamped(); msg.header.frame_id = 'camera_init'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in [(c.min_x, c.min_y), (c.max_x, c.min_y), (c.max_x, c.max_y), (c.min_x, c.max_y)]:
            msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.))
        for pub in self.pubs: pub.publish(msg)
        marker = Marker(); marker.header = msg.header; marker.ns = 'task_boundary'; marker.id = 0
        marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD; marker.scale.x = .035
        marker.color.r = 1.; marker.color.g = .85; marker.color.a = .7; marker.pose.orientation.w = 1.
        for p in [*msg.polygon.points, msg.polygon.points[0]]:
            marker.points.append(Point(x=float(p.x), y=float(p.y), z=float(p.z)))
        self.vis.publish(marker)


def main():
    rclpy.init(); node = Boundary()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
