import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from rclpy.time import Time

class ArucoVelocityEstimator(Node):
    def __init__(self):
        super().__init__('aruco_velocity_estimator')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/velcal/obs_vel', 10)
        
        self.prev_pose = None
        self.prev_time = None
        self.velocity = Twist()

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.prev_pose is None or self.prev_time is None:
            self.prev_pose = msg.poses[0]
            self.prev_time = current_time
            return
        
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate velocity
        self.velocity.linear.x = (msg.poses[0].position.x - self.prev_pose.position.x) / dt
        self.velocity.linear.y = (msg.poses[0].position.y - self.prev_pose.position.y) / dt
        self.velocity.linear.z = (msg.poses[0].position.z - self.prev_pose.position.z) / dt
        
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
