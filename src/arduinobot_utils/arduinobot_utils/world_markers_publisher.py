#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class WorldMarkersPublisher(Node):
    def __init__(self):
        super().__init__('world_markers_publisher')
        self.publisher = self.create_publisher(Marker, 'world_objects', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)

    def publish_markers(self):
        table = Marker()
        table.header.frame_id = 'world'
        table.header.stamp = self.get_clock().now().to_msg()
        table.ns = 'table'
        table.id = 0
        table.type = Marker.CUBE
        table.action = Marker.ADD
        table.pose.position.x = 1.3
        table.pose.position.y = 0.7
        table.pose.position.z = 0.175
        table.pose.orientation.w = 1.0
        table.scale.x = 1.2
        table.scale.y = 0.8
        table.scale.z = 0.35
        table.color.r = 0.6
        table.color.g = 0.4
        table.color.b = 0.2
        table.color.a = 1.0
        self.publisher.publish(table)

        cube = Marker()
        cube.header.frame_id = 'world'
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.ns = 'red_cube'
        cube.id = 1
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = 0.85
        cube.pose.position.y = 0.55
        cube.pose.position.z = 0.45
        cube.pose.orientation.w = 1.0
        cube.scale.x = 0.20
        cube.scale.y = 0.20
        cube.scale.z = 0.20
        cube.color.r = 1.0
        cube.color.g = 0.0
        cube.color.b = 0.0
        cube.color.a = 1.0
        self.publisher.publish(cube)

        cylinder = Marker()
        cylinder.header.frame_id = 'world'
        cylinder.header.stamp = self.get_clock().now().to_msg()
        cylinder.ns = 'blue_cylinder'
        cylinder.id = 2
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose.position.x = 1.25
        cylinder.pose.position.y = 0.85
        cylinder.pose.position.z = 0.45
        cylinder.pose.orientation.w = 1.0
        cylinder.scale.x = 0.16
        cylinder.scale.y = 0.16
        cylinder.scale.z = 0.20
        cylinder.color.r = 0.0
        cylinder.color.g = 0.0
        cylinder.color.b = 1.0
        cylinder.color.a = 1.0
        self.publisher.publish(cylinder)

        sphere = Marker()
        sphere.header.frame_id = 'world'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'green_sphere'
        sphere.id = 3
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = 1.65
        sphere.pose.position.y = 1.0
        sphere.pose.position.z = 0.47
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.24
        sphere.scale.y = 0.24
        sphere.scale.z = 0.24
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        self.publisher.publish(sphere)

        # Red drop zone
        red_drop_zone = Marker()
        red_drop_zone.header.frame_id = 'world'
        red_drop_zone.header.stamp = self.get_clock().now().to_msg()
        red_drop_zone.ns = 'red_drop_zone'
        red_drop_zone.id = 4
        red_drop_zone.type = Marker.CUBE
        red_drop_zone.action = Marker.ADD
        red_drop_zone.pose.position.x = -1.5
        red_drop_zone.pose.position.y = 1.2
        red_drop_zone.pose.position.z = 0.05
        red_drop_zone.pose.orientation.w = 1.0
        red_drop_zone.scale.x = 0.3
        red_drop_zone.scale.y = 0.3
        red_drop_zone.scale.z = 0.1
        red_drop_zone.color.r = 1.0
        red_drop_zone.color.g = 0.0
        red_drop_zone.color.b = 0.0
        red_drop_zone.color.a = 0.7
        self.publisher.publish(red_drop_zone)

        # Blue drop zone
        blue_drop_zone = Marker()
        blue_drop_zone.header.frame_id = 'world'
        blue_drop_zone.header.stamp = self.get_clock().now().to_msg()
        blue_drop_zone.ns = 'blue_drop_zone'
        blue_drop_zone.id = 5
        blue_drop_zone.type = Marker.CUBE
        blue_drop_zone.action = Marker.ADD
        blue_drop_zone.pose.position.x = -1.5
        blue_drop_zone.pose.position.y = 0.6
        blue_drop_zone.pose.position.z = 0.05
        blue_drop_zone.pose.orientation.w = 1.0
        blue_drop_zone.scale.x = 0.3
        blue_drop_zone.scale.y = 0.3
        blue_drop_zone.scale.z = 0.1
        blue_drop_zone.color.r = 0.0
        blue_drop_zone.color.g = 0.0
        blue_drop_zone.color.b = 1.0
        blue_drop_zone.color.a = 0.7
        self.publisher.publish(blue_drop_zone)

        # Green drop zone
        green_drop_zone = Marker()
        green_drop_zone.header.frame_id = 'world'
        green_drop_zone.header.stamp = self.get_clock().now().to_msg()
        green_drop_zone.ns = 'green_drop_zone'
        green_drop_zone.id = 6
        green_drop_zone.type = Marker.CUBE
        green_drop_zone.action = Marker.ADD
        green_drop_zone.pose.position.x = -1.5
        green_drop_zone.pose.position.y = 0.0
        green_drop_zone.pose.position.z = 0.05
        green_drop_zone.pose.orientation.w = 1.0
        green_drop_zone.scale.x = 0.3
        green_drop_zone.scale.y = 0.3
        green_drop_zone.scale.z = 0.1
        green_drop_zone.color.r = 0.0
        green_drop_zone.color.g = 1.0
        green_drop_zone.color.b = 0.0
        green_drop_zone.color.a = 0.7
        self.publisher.publish(green_drop_zone)

def main(args=None):
    rclpy.init(args=args)
    node = WorldMarkersPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()