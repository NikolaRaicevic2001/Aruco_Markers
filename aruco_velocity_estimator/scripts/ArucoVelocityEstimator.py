#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist, Pose
from rclpy.time import Time

from ros2_aruco_interfaces.msg import ArucoMarkers
from aruco_velocity_estimator.msg import ArucoMarkersVelocity

class ArucoVelocityEstimator(Node):
    def __init__(self):
        super().__init__('aruco_velocity_estimator')
        # Variables
        self.prev_pose = {}
        self.prev_time = {}

        # Subscribers
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.pose_callback,
            10)
        
        # Publishers
        self.publisher = self.create_publisher(ArucoMarkersVelocity, '/aruco_markers_velocities', 10)

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        velocity_msg = ArucoMarkersVelocity()
        velocity_msg.header = msg.header

        for i, marker_id in enumerate(msg.marker_ids):
            pose = msg.poses[i]
            position = pose.position

            if marker_id not in self.prev_pose or marker_id not in self.prev_time:
                self.prev_pose[marker_id] = pose
                self.prev_time[marker_id] = current_time
                continue
            
            dt = (current_time - self.prev_time[marker_id]).nanoseconds / 1e9
            if dt < 1e-9:
                continue

            if ((position.x - self.prev_pose[marker_id].position.x) < 1e-6 and
               (position.y - self.prev_pose[marker_id].position.y) < 1e-6 and
               (position.z - self.prev_pose[marker_id].position.z) < 1e-3):       
                # Enter position values
                pose = Pose()
                pose.position.x = position.x
                pose.position.y = position.y
                pose.position.z = position.z

                # Calculate velocity
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                    
                velocity_msg.marker_ids.append(marker_id)
                velocity_msg.twists.append(twist)
                velocity_msg.poses.append(pose)

                continue
                
            # Enter position values
            pose = Pose()
            pose.position.x = position.x
            pose.position.y = position.y
            pose.position.z = position.z

            # Calculate velocity
            twist = Twist()
            twist.linear.x = (position.x - self.prev_pose[marker_id].position.x) / dt
            twist.linear.y = (position.y - self.prev_pose[marker_id].position.y) / dt
            twist.linear.z = (position.z - self.prev_pose[marker_id].position.z) / dt

            velocity_msg.marker_ids.append(marker_id)
            velocity_msg.twists.append(twist)
            velocity_msg.poses.append(pose)

            # Update previous pose and time for this marker
            self.prev_pose[marker_id] = pose
            self.prev_time[marker_id] = current_time

            # if (twist.linear.x > 0.1) or (twist.linear.y > 0.1) or (twist.linear.z > 0.1):
            self.get_logger().info(f"Aruco ID: {marker_id}")
            self.get_logger().info(f"Velocity: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, z={twist.linear.z:.2f}")
            self.get_logger().info(f"Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")
            self.get_logger().info(f"Frequency: {1/dt:.2f} hz")
            self.get_logger().info(f"dt: {dt}")

        # Publish velocities for all markers
        if velocity_msg.marker_ids:
            self.publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_velocity_estimator = ArucoVelocityEstimator()
    rclpy.spin(aruco_velocity_estimator)
    aruco_velocity_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
