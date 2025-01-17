#!/usr/bin/env python3

from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import rclpy
import cv2

from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class ArucoVisualizer(Node):
    def __init__(self):
        super().__init__('aruco_visualizer')
        
        # Variables 
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cv_bridge = CvBridge()
        self.latest_image = None

        # Subscriptions
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.aruco_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(Image, '/aruco_visualization', 10)
        
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def aruco_callback(self, msg):
        if self.latest_image is None:
            return

        image_with_markers = self.latest_image.copy()

        for i, marker_id in enumerate(msg.marker_ids):
            # Extract pose information
            pose = msg.poses[i]
            position = pose.position
            
            # Project 3D point to 2D image plane (simplified)
            # Note: This is a simplification and may not be accurate for all camera setups
            x = int(position.x * 100 + image_with_markers.shape[1] / 2)
            y = int(-position.y * 100 + image_with_markers.shape[0] / 2)
            
            # Draw marker on image
            cv2.circle(image_with_markers, (x, y), 10, (0, 255, 0), -1)
            cv2.putText(image_with_markers, f"ID: {marker_id}", (x+10, y+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the annotated image
        self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(image_with_markers, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    aruco_visualizer = ArucoVisualizer()
    rclpy.spin(aruco_visualizer)
    aruco_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
