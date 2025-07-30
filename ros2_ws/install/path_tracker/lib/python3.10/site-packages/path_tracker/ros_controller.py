import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from path_tracker.trajectory_generator import generate_trajectory
from path_tracker.path_smoothing import smooth_path
from path_tracker.controller import compute_velocity_commands
import tf_transformations

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Trajectory
        waypoints = [(0, 0), (1, 1), (2, 0), (3, 2)]
        smooth = smooth_path(waypoints)
        self.trajectory = generate_trajectory(smooth)
        self.current_pose = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = (x, y, yaw)

    def timer_callback(self):
        if self.current_pose is None:
            return
        v, w = compute_velocity_commands(self.current_pose, self.trajectory)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
