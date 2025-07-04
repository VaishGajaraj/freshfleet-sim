#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = rospy.Publisher('/detected_objects', PoseStamped, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/debug_image', Image, queue_size=10)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pointcloud_callback)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Store latest point cloud for 3D position calculation
        self.latest_pointcloud = None
        
        rospy.loginfo("Object detector initialized")
    
    def image_callback(self, msg):
        """Process RGB image for object detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Simple color-based detection (replace with your ML model)
            detected_objects = self.detect_objects(cv_image)
            
            # Publish detection results
            for obj in detected_objects:
                self.publish_detection(obj, msg.header)
            
            # Publish debug image
            debug_image = self.draw_detections(cv_image, detected_objects)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def pointcloud_callback(self, msg):
        """Store latest point cloud for 3D position calculation"""
        self.latest_pointcloud = msg
    
    def detect_objects(self, image):
        """Simple color-based object detection"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for different objects
        # Red objects (fruits, etc.)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        # Green objects (vegetables, etc.)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # Create masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []
        
        # Process red objects
        for contour in contours_red:
            if cv2.contourArea(contour) > 1000:  # Minimum area threshold
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detected_objects.append({
                        'type': 'red_object',
                        'center': (cx, cy),
                        'contour': contour
                    })
        
        # Process green objects
        for contour in contours_green:
            if cv2.contourArea(contour) > 1000:  # Minimum area threshold
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detected_objects.append({
                        'type': 'green_object',
                        'center': (cx, cy),
                        'contour': contour
                    })
        
        return detected_objects
    
    def draw_detections(self, image, detections):
        """Draw detection results on image for debugging"""
        debug_image = image.copy()
        
        for obj in detections:
            cx, cy = obj['center']
            color = (0, 0, 255) if obj['type'] == 'red_object' else (0, 255, 0)
            
            # Draw circle at detection center
            cv2.circle(debug_image, (cx, cy), 10, color, -1)
            
            # Draw contour
            cv2.drawContours(debug_image, [obj['contour']], -1, color, 2)
            
            # Add label
            cv2.putText(debug_image, obj['type'], (cx-20, cy-20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return debug_image
    
    def publish_detection(self, obj, header):
        """Publish detection result as PoseStamped"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "camera_color_optical_frame"
        
        # Set position (you would calculate 3D position from point cloud here)
        pose_msg.pose.position.x = 0.5  # Example values
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.3
        
        # Set orientation (quaternion for upright object)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Transform to robot base frame if possible
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", 
                header.frame_id, 
                header.stamp, 
                rospy.Duration(1.0)
            )
            pose_msg = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)
        except Exception as e:
            rospy.logwarn(f"Could not transform pose: {e}")
        
        self.detection_pub.publish(pose_msg)
        rospy.loginfo(f"Published detection: {obj['type']} at ({pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}, {pose_msg.pose.position.z:.3f})")

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 