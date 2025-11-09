#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

class EEMarkersPublisher(Node):
    def __init__(self):
        super().__init__('ee_markers_publisher')
        self.publisher = self.create_publisher(Marker, 'ee_markers', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.get_logger().info("EE Markers Publisher started")
    
    def publish_markers(self):
        try:
            tcp_transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tcp_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            left_transform = self.tf_buffer.lookup_transform(
                'base_link',
                'gripper_left',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            right_transform = self.tf_buffer.lookup_transform(
                'base_link',
                'gripper_right',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            left_x = left_transform.transform.translation.x
            left_y = left_transform.transform.translation.y
            left_z = left_transform.transform.translation.z
            
            right_x = right_transform.transform.translation.x
            right_y = right_transform.transform.translation.y
            right_z = right_transform.transform.translation.z
            
            mid_x = (left_x + right_x) / 2.0
            mid_y = (left_y + right_y) / 2.0
            mid_z = (left_z + right_z) / 2.0
            
            tcp_marker = Marker()
            tcp_marker.header.frame_id = 'base_link'
            tcp_marker.header.stamp = self.get_clock().now().to_msg()
            tcp_marker.ns = 'ee_tcp'
            tcp_marker.id = 0
            tcp_marker.type = Marker.SPHERE
            tcp_marker.action = Marker.ADD
            tcp_marker.pose.position.x = tcp_transform.transform.translation.x
            tcp_marker.pose.position.y = tcp_transform.transform.translation.y
            tcp_marker.pose.position.z = tcp_transform.transform.translation.z
            tcp_marker.pose.orientation.w = 1.0
            tcp_marker.scale.x = 0.03
            tcp_marker.scale.y = 0.03
            tcp_marker.scale.z = 0.03
            tcp_marker.color.r = 0.0
            tcp_marker.color.g = 1.0
            tcp_marker.color.b = 0.0
            tcp_marker.color.a = 0.8
            self.publisher.publish(tcp_marker)
            
            mid_marker = Marker()
            mid_marker.header.frame_id = 'base_link'
            mid_marker.header.stamp = self.get_clock().now().to_msg()
            mid_marker.ns = 'ee_midpoint'
            mid_marker.id = 1
            mid_marker.type = Marker.SPHERE
            mid_marker.action = Marker.ADD
            mid_marker.pose.position.x = mid_x
            mid_marker.pose.position.y = mid_y
            mid_marker.pose.position.z = mid_z
            mid_marker.pose.orientation.w = 1.0
            mid_marker.scale.x = 0.04
            mid_marker.scale.y = 0.04
            mid_marker.scale.z = 0.04
            mid_marker.color.r = 1.0
            mid_marker.color.g = 0.0
            mid_marker.color.b = 0.0
            mid_marker.color.a = 0.9
            self.publisher.publish(mid_marker)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EEMarkersPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
