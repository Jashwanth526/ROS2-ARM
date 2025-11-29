#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String
from rclpy.duration import Duration

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.image_subscriber = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/depth_image', 
            self.depth_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.latest_depth_image = None
        self.camera_info_received = False
        
        self.detection_publisher = self.create_publisher(
            PointStamped, 
            '/detected_objects', 
            10
        )
        
        self.detection_info_publisher = self.create_publisher(
            String,
            '/detection_info',
            10
        )
        
        self.detection_enabled = True
        
        self.camera_fx = 554.25
        self.camera_fy = 554.25
        self.camera_cx = 320.0
        self.camera_cy = 240.0
        self.image_width = 640
        self.image_height = 480
        
        self.get_logger().info("Object Detection Node started")
    
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self.image_width = msg.width
            self.image_height = msg.height
            self.camera_info_received = True
            self.get_logger().info(f"Camera intrinsics updated: fx={self.camera_fx:.2f}, fy={self.camera_fy:.2f}, cx={self.camera_cx:.2f}, cy={self.camera_cy:.2f}")
        
    def image_callback(self, msg):
        if not self.detection_enabled:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detections = self.detect_colored_objects(cv_image)
            
            for detection in detections:
                self.publish_detection(detection)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.get_logger().debug(f"Depth image received: shape={self.latest_depth_image.shape}, dtype={self.latest_depth_image.dtype}")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
            
    def detect_colored_objects(self, image):
        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [(np.array([90, 100, 100]), np.array([130, 255, 255]))],
            'green': [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
            'yellow': [(np.array([26, 120, 120]), np.array([35, 255, 255]))],
            'purple': [(np.array([130, 50, 50]), np.array([160, 255, 255]))],
            'brown': [(np.array([5, 50, 20]), np.array([25, 255, 200]))]
        }
        
        for color_name, ranges in color_ranges.items():
            mask = None
            
            for lower, upper in ranges:
                color_mask = cv2.inRange(hsv, lower, upper)
                mask = color_mask if mask is None else cv2.bitwise_or(mask, color_mask)
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    confidence = min(area / 5000.0, 1.0)
                    
                    detection = {
                        'color': color_name,
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area,
                        'confidence': confidence,
                        'bbox': (x, y, w, h)
                    }
                    
                    detections.append(detection)
                    
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
                    cv2.putText(image, f"{color_name}: {confidence:.2f}", 
                              (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Object Detection", image)
        cv2.waitKey(1)
        
        return detections
    
    def pixel_to_3d_point(self, pixel_x, pixel_y):
        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image available for 3D conversion")
            return None
            
        height, width = self.latest_depth_image.shape[:2]
        pixel_x = int(pixel_x)
        pixel_y = int(pixel_y)
        
        if pixel_x < 0 or pixel_x >= width or pixel_y < 0 or pixel_y >= height:
            self.get_logger().warn(f"Pixel ({pixel_x}, {pixel_y}) outside bounds ({width}x{height})")
            return None
        
        depth = float(self.latest_depth_image[pixel_y, pixel_x])
        
        if depth > 0.0 and not np.isnan(depth) and not np.isinf(depth):
            self.get_logger().info(f"[DEPTH_DEBUG] Detection at depth {depth:.3f}m at pixel ({pixel_x},{pixel_y})")
        
        MIN_DEPTH = 0.4
        if depth <= 0.0 or np.isnan(depth) or np.isinf(depth) or depth > 5.0 or depth < MIN_DEPTH:
            if 0.0 < depth < MIN_DEPTH:
                self.get_logger().info(f"[DEPTH_FILTER] Rejected detection at {depth:.3f}m (gripper/arm, min={MIN_DEPTH}m)")
            elif depth > 0:
                self.get_logger().warn(f"Invalid depth {depth}m at pixel ({pixel_x}, {pixel_y})")
            return None
            
        x = (pixel_x - self.camera_cx) * depth / self.camera_fx
        y = (pixel_y - self.camera_cy) * depth / self.camera_fy
        z = depth
        
        return (x, y, z)
    
    def publish_detection(self, detection):
        point_3d = self.pixel_to_3d_point(detection['center_x'], detection['center_y'])
        
        if point_3d is not None:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "depth_camera_optical"
            
            point_msg.point.x = float(point_3d[0])
            point_msg.point.y = float(point_3d[1])
            point_msg.point.z = float(point_3d[2])
            
            self.get_logger().info(
                f"Camera frame: ({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f})"
            )
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'depth_camera_optical',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
                )
                
                point_in_base = tf2_geometry_msgs.do_transform_point(point_msg, transform)
                point_in_base.point.z += 0.26
                point_in_base.header.frame_id = f"base_link/{detection['color']}"
                
                self.detection_publisher.publish(point_in_base)
                
                self.get_logger().info(
                    f"{detection['color']} object: "
                    f"camera=({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f}), "
                    f"base=({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, {point_in_base.point.z:.3f})"
                )
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"TF transform failed: {str(e)}")
                return
        
        else:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "camera_pixels"
            
            point_msg.point.x = float(detection['center_x'])
            point_msg.point.y = float(detection['center_y'])
            point_msg.point.z = float(detection['confidence'])
            
            self.detection_publisher.publish(point_msg)
            self.get_logger().warn(f"No depth data, publishing pixel coords: ({detection['center_x']}, {detection['center_y']})")
        
        info_msg = String()
        info_msg.data = (
            f"Detected {detection['color']} object at "
            f"({detection['center_x']}, {detection['center_y']}) "
            f"with confidence {detection['confidence']:.2f}"
        )
        self.detection_info_publisher.publish(info_msg)
    
    def enable_detection(self, enable=True):
        self.detection_enabled = enable
        self.get_logger().info(f"Object detection {'enabled' if enable else 'disabled'}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()