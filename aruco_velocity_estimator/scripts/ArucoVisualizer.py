#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from rclpy.node import Node
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
            '/camera/camera/color/camera_info',
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
        if self.latest_image is None or self.camera_matrix is None:
            return

        image_with_markers = self.latest_image.copy()

        for i, marker_id in enumerate(msg.marker_ids):
            pose = msg.poses[i]
            position = pose.position
            orientation = pose.orientation
            print("Aruco ID: ", marker_id)
            print(f"Position: x={position.x}, y={position.y}, z={position.z}")
            print(f"Orientation: orientation.x={pose.orientation.x}, orientation.y={pose.orientation.y}, orientation.z={pose.orientation.z}, orientation.w={pose.orientation.w}")

            # Create 3D points for the marker corners (assuming a square marker)
            marker_size = 0.05  # Adjust this to your marker size in meters
            object_points = np.array([
                [-marker_size/2, -marker_size/2, 0],
                [marker_size/2, -marker_size/2, 0],
                [marker_size/2, marker_size/2, 0],
                [-marker_size/2, marker_size/2, 0]
            ])

            # Convert quaternion to rotation matrix
            r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
            rvec, _ = cv2.Rodrigues(r.as_matrix())

            tvec = np.array([position.x, position.y, position.z])

            # Project 3D points to 2D image plane
            image_points, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)

            # Draw bounding box
            image_points = image_points.reshape(-1, 2).astype(int)
            cv2.polylines(image_with_markers, [image_points], True, (0, 255, 0), 2)

            # Add marker ID text
            cv2.putText(image_with_markers, f"ID: {marker_id}", tuple(image_points[0]), 
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
