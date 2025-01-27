#!/usr/bin/env python3

import numpy as np
import torch
import os

from scipy.spatial.transform import Rotation as R

import ros2_numpy as rnp
from rclpy.node import Node
from rclpy.time import Time
import rclpy

from ros2_aruco_interfaces.msg import ArucoMarkers
from aruco_velocity_estimator.msg import ArucoMarkersVelocity

from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Twist, Pose, Transform, Vector3, Quaternion, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField

class ArucoPointcloudMatch(Node):
    def __init__(self):
        super().__init__('aruco_pointcloud_match')
        # Variables
        self.point_clouds = None
        self.point_clouds_transformed = None

        self.aruco_detections = None
        self.aruco_detections_transformed = None

        self.transform = None
        self.rotation = None
        self.translation = None 

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.aruco_subscription = self.create_subscription(
            ArucoMarkersVelocity,
            '/aruco_markers_velocities',
            self.aruco_callback,
            10
        )

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/point_cloud2_processed',
            self.pointcloud_callback,
            10
        )

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/point_cloud2_processed_transformed',
            self.pointcloud_callback_02,
            10
        )

        # Publishers
        self.publisher = self.create_publisher(ArucoMarkersVelocity, '/aruco_markers_velocities', 10)

        self.nearby_points_pub = self.create_publisher(PointCloud2, '/nearby_points', 10)
        self.nearby_points_transformed_pub = self.create_publisher(PointCloud2, '/nearby_points_transformed', 10)

    def pointcloud_callback(self, msg):
        self.point_clouds = torch.tensor(self.pointcloud2_to_numpy(msg), device=self.device)
        point_velocities = torch.zeros_like(self.point_clouds)
        if self.aruco_detections is not None:
            for marker_id, position, velocity in self.aruco_detections:
                # Find closest points to aruco marker position
                distances = torch.norm(self.point_clouds - torch.tensor(position, device=self.device), dim=1)
                nearby_points = distances < 0.1  # 10cm threshold
                print(f"Marker: {marker_id} Number of nearby points: {nearby_points.sum()}")
                point_velocities[nearby_points] = torch.tensor(velocity, device=self.device)

                # Create a new PointCloud2 message with only nearby points
                nearby_points_np = self.point_clouds[nearby_points].cpu().numpy()
                nearby_cloud_msg = self.create_point_cloud2(nearby_points_np, msg.header.frame_id)
                # Publish the new point cloud
                self.nearby_points_pub.publish(nearby_cloud_msg)

        print(f"Point cloud 01 number of points {self.point_clouds.shape}")           

    def pointcloud_callback_02(self, msg):
        self.point_clouds_transformed = torch.tensor(self.pointcloud2_to_numpy(msg), device=self.device)
        point_velocities = torch.zeros_like(self.point_clouds_transformed)
        if self.aruco_detections_transformed is not None:
            for marker_id, position, velocity in self.aruco_detections_transformed:
                # Find closest points to aruco marker position
                distances = torch.norm(self.point_clouds_transformed - torch.tensor(position, device=self.device), dim=1)
                nearby_points = distances < 0.1  # 10cm threshold
                print(f"Marker: {marker_id} Number of nearby transformed points: {nearby_points.sum()}")
                point_velocities[nearby_points] = torch.tensor(velocity, device=self.device)

                # Create a new PointCloud2 message with only nearby points
                nearby_points_np = self.point_clouds_transformed[nearby_points].cpu().numpy()
                nearby_cloud_msg = self.create_point_cloud2(nearby_points_np, msg.header.frame_id)
                # Publish the new point cloud
                self.nearby_points_transformed_pub.publish(nearby_cloud_msg)

        print(f"Point cloud 02 number of points {self.point_clouds_transformed.shape}")   

    def create_point_cloud2(self, points, frame_id):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.height = 1
        msg.width = points.shape[0]

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()

        return msg
    
    def aruco_callback(self, msg):
        aruco_detections = []
        aruco_detections_transformed = []

        for i, marker_id in enumerate(msg.marker_ids):
            twist = [msg.twists[i].linear.x, msg.twists[i].linear.y, msg.twists[i].linear.z]
            position = [msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]

            # self.get_logger().info(f"Aruco ID: {marker_id}")
            # self.get_logger().info(f"Velocity: x={twist[0]:.2f}, y={twist[1]:.2f}, z={twist[2]:.2f}")
            # self.get_logger().info(f"Position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

            print(f"Aruco ID: {marker_id}")
            print(f"Velocity Length: {len(twist)}")
            print(f"Position: {len(position)}")
            # print(f"Velocity: {np.array(twist).shape}")
            # print(f"Velocity: {np.array(twist)[:, :3].T}")
            # print(f"Position: {np.array(position)[:, :3].T}")

            # Transform position and velocities to robot base frame
            transformed_positions,transformed_velocities = self.transform_aruco_markers(np.array(position).reshape(1,3), np.array(twist).reshape(1,3))

            aruco_detections.append([marker_id, position, twist])
            aruco_detections_transformed.append([marker_id, transformed_positions, transformed_velocities])

        if aruco_detections:
            self.aruco_detections = aruco_detections
            self.aruco_detections_transformed = aruco_detections_transformed    

    def pointcloud2_to_numpy(self, msg):
        data = rnp.numpify(msg)
        data_new = np.column_stack((data['x'].flatten(), data['y'].flatten(), data['z'].flatten()))
        data_new = data_new.astype(np.float32)
        
        # Vectorized filtering
        mask = ~np.isinf(data_new[:, 0]) & ~np.isinf(data_new[:, 1]) & ~np.isinf(data_new[:, 2])
        filtered_points = data_new[mask]

        return filtered_points   

    def transform_aruco_markers(self, position, velocities):
        while self.transform is None:
            self.load_transform_from_file(os.path.join('src/main/scripts/data/pointcloud_data', 'transform_data_real.txt'))

            if self.transform:
                # Extract translation
                self.translation = np.array([
                    self.transform.transform.translation.x,
                    self.transform.transform.translation.y,
                    self.transform.transform.translation.z
                ])

                # Extract rotation and convert to rotation matrix
                self.rotation = R.from_quat([
                    self.transform.transform.rotation.x,
                    self.transform.transform.rotation.y,
                    self.transform.transform.rotation.z,
                    self.transform.transform.rotation.w
                ]).as_matrix()

        try:
            # Transform Position and Velocities
            transformed_positions = (self.rotation @ position[:, :3].T).T + self.translation
            transformed_velocities = (self.rotation @ velocities[:, :3].T).T 
            return transformed_positions.tolist(), transformed_velocities.tolist()

        except Exception as e:
            self.get_logger().error(f"Failed to transform point cloud: {str(e)}")
            return None, None        

    def load_transform_from_file(self, filename):
        transform_data = {}
        with open(filename, 'r') as f:
            for line in f:
                key, value = line.strip().strip("'").split("','")
                value = value.strip("',")
                transform_data[key] = float(value)

                print(f"Loaded transform data: {key} = {value}")

        # Create a new Transform object
        self.transform = None
        self.transform = TransformStamped()

        # Set the translation
        self.transform.transform.translation = Vector3()
        self.transform.transform.translation.x = transform_data['--x']
        self.transform.transform.translation.y = transform_data['--y']
        self.transform.transform.translation.z = transform_data['--z']

        # Set the rotation (quaternion)
        self.transform.transform.rotation = Quaternion()
        self.transform.transform.rotation.x = transform_data['--qx']
        self.transform.transform.rotation.y = transform_data['--qy']
        self.transform.transform.rotation.z = transform_data['--qz']
        self.transform.transform.rotation.w = transform_data['--qw']

def main(args=None):
    rclpy.init(args=args)
    aruco_pointcloud_match = ArucoPointcloudMatch()
    rclpy.spin(aruco_pointcloud_match)
    aruco_pointcloud_match.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
