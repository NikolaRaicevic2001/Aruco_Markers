#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from rclpy.time import Time

from ros2_aruco_interfaces.msg import ArucoMarkers
class ArucoVelocityEstimator(Node):
    def __init__(self):
        super().__init__('aruco_velocity_estimator')
        # Variables
        self.prev_pose = None
        self.prev_time = None
        self.velocity = Twist()

        # Subscribers
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.pose_callback,
            10)
        
        # Publishers
        self.publisher = self.create_publisher(Twist, '/velcal/obs_vel', 10)

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.prev_pose is None or self.prev_time is None:
            self.prev_pose = msg.poses[0]
            self.prev_time = current_time
            return
        
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        
        for i, marker_id in enumerate(msg.marker_ids):
            pose = msg.poses[i]
            position = pose.position
            print("Aruco ID: ", marker_id)
            print(f"Position: x={position.x}, y={position.y}, z={position.z}")

            # Calculate velocity
            self.velocity.linear.x = (position.x - self.prev_pose.position.x) / dt
            self.velocity.linear.y = (position.y - self.prev_pose.position.y) / dt
            self.velocity.linear.z = (position.z - self.prev_pose.position.z) / dt
            print(f"Velocity: x={self.velocity.linear.x}, y={self.velocity.linear.y}, z={self.velocity.linear.z}")

            # Publish velocity
            self.publisher.publish(self.velocity)
            
        # Update previous pose and time
        self.prev_pose = msg.poses[0]
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    aruco_velocity_estimator = ArucoVelocityEstimator()
    rclpy.spin(aruco_velocity_estimator)
    aruco_velocity_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
