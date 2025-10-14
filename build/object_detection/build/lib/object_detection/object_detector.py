#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )
        
        # TODO: Add depth camera subscription for 3D positioning later
        # self.depth_subscriber = self.create_subscription(
        #     Image,
        #     '/depth/image_raw', 
        #     self.depth_callback,
        #     10
        # )
        
        # Store latest depth image
        self.latest_depth_image = None
        
        # Publishers
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
        
        # Detection parameters
        self.detection_enabled = True
        
        # Camera intrinsic parameters (will be loaded from camera_info topic in production)
        # These are approximate values for the simulated camera
        self.camera_fx = 1200.0  # Focal length X
        self.camera_fy = 1200.0  # Focal length Y  
        self.camera_cx = 1152.0  # Principal point X (width/2)
        self.camera_cy = 648.0   # Principal point Y (height/2)
        
        self.get_logger().info("Enhanced Object Detection Node with Depth started")
        
    def image_callback(self, msg):
        if not self.detection_enabled:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect objects
            detections = self.detect_colored_objects(cv_image)
            
            # Publish detections
            for detection in detections:
                self.publish_detection(detection)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def depth_callback(self, msg):
        """Store the latest depth image for 3D coordinate calculation"""
        try:
            # Convert ROS depth image to OpenCV (depth is typically 32FC1 or 16UC1)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
            
    def detect_colored_objects(self, image):
        """
        Detect colored objects (red, blue, green) in the image
        Returns list of detections with color, position, and confidence
        """
        detections = []
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges in HSV
        color_ranges = {
            'red': [
                (np.array([0, 70, 50]), np.array([10, 255, 255])),     # Lower red range
                (np.array([170, 70, 50]), np.array([180, 255, 255]))   # Upper red range
            ],
            'blue': [(np.array([100, 150, 0]), np.array([140, 255, 255]))],
            'green': [(np.array([35, 40, 40]), np.array([85, 255, 255]))]
        }
        
        for color_name, ranges in color_ranges.items():
            mask = None
            
            # Create mask for each color range
            for lower, upper in ranges:
                color_mask = cv2.inRange(hsv, lower, upper)
                mask = color_mask if mask is None else cv2.bitwise_or(mask, color_mask)
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area (adjust based on object size and distance)
                if area > 100:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center point
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Calculate confidence based on area and shape
                    confidence = min(area / 1000.0, 1.0)  # Normalize confidence
                    
                    detection = {
                        'color': color_name,
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area,
                        'confidence': confidence,
                        'bbox': (x, y, w, h)
                    }
                    
                    detections.append(detection)
                    
                    # Draw detection on image for debugging
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(image, f"{color_name}: {confidence:.2f}", 
                              (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Show image for debugging (comment out for production)
        cv2.imshow("Object Detection", image)
        cv2.waitKey(1)
        
        return detections
    
    def pixel_to_3d_point(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates to 3D point using depth data
        Returns (x, y, z) in camera frame coordinates (meters)
        """
        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image available for 3D conversion")
            return None
            
        # Ensure pixel coordinates are within image bounds
        height, width = self.latest_depth_image.shape
        if pixel_x < 0 or pixel_x >= width or pixel_y < 0 or pixel_y >= height:
            self.get_logger().warn(f"Pixel coordinates ({pixel_x}, {pixel_y}) outside image bounds ({width}x{height})")
            return None
        
        # Get depth value at pixel (in meters for simulation, typically)
        depth = self.latest_depth_image[pixel_y, pixel_x]
        
        # Skip invalid depth values
        if depth <= 0 or np.isnan(depth) or np.isinf(depth):
            self.get_logger().warn(f"Invalid depth value {depth} at pixel ({pixel_x}, {pixel_y})")
            return None
            
        # Convert pixel to 3D coordinates using camera intrinsics
        # Standard pinhole camera model: X = (u - cx) * Z / fx
        x = (pixel_x - self.camera_cx) * depth / self.camera_fx
        y = (pixel_y - self.camera_cy) * depth / self.camera_fy  
        z = depth
        
        return (x, y, z)
    
    def publish_detection(self, detection):
        """
        Publish detected object information (back to basic pixel coordinates for now)
        """
        # Create PointStamped message for object position
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "rgb_camera"
        
        # Use pixel coordinates for now (revert to original approach)
        point_msg.point.x = float(detection['center_x'])
        point_msg.point.y = float(detection['center_y'])
        point_msg.point.z = float(detection['confidence'])
        
        self.detection_publisher.publish(point_msg)
        
        # Publish detection info
        info_msg = String()
        info_msg.data = f"Detected {detection['color']} object at ({detection['center_x']}, {detection['center_y']}) with confidence {detection['confidence']:.2f}"
        self.detection_info_publisher.publish(info_msg)
        
        self.get_logger().info(info_msg.data)
    
    def enable_detection(self, enable=True):
        """Enable or disable object detection"""
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
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()