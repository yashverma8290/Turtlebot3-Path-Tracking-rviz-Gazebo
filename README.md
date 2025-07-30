TurtleBot3 Path Tracking with Trajectory Simulation and Controller (ROS 2 Jazzy/Humble)

Project Overview
This project simulates a TurtleBot3 robot that tracks a smooth path in a 2D environment. It is built in ROS 2 and uses Gazebo and RViz for visualization. It includes:

- A path smoothing algorithm that converts raw waypoints into a nice curved path (like smoothing out a zigzag road).
- A trajectory generator that calculates where the robot should be at every time step (x(t), y(t), θ(t), v(t), ω(t)).
- A trajectory controller that computes the linear and angular velocities (/cmd_vel) to follow the trajectory.
- A full simulation with visualization.

You can showcase this project for ~3 minutes, including full launch of Gazebo + RViz + robot moving along the generated trajectory.

Folder Structure
```
ros2_ws/
└── src/
    └── path_tracker/
        ├── launch/
        │   ├── gazebo_control.launch.py      # Launches simulation with controller
        │   └── simulation.launch.py          # Full launch: smoothing + trajectory + control
        ├── path_tracker/
        │   ├── controller.py                  # Controller node logic
        │   ├── main.py                        # Main script for testing simulation
        │   ├── path_smoothing.py             # Makes the path smooth
        │   ├── ros_controller.py              # ROS controller integration
        │   ├── simulator.py                   # Simulator for trajectory following
        │   └── trajectory_generator.py       # Converts smooth path to time-based trajectory
        ├── setup.py                          # Python package setup
        └── package.xml                       # ROS 2 package manifest




Concept Breakdown (High-Level Idea)

Step-by-Step Flow:
1. Path Smoothing: Given some waypoints, we apply cubic spline interpolation to generate a smooth and continuous path.

2. Trajectory Generation: Convert the smooth path into a time-based trajectory, assigning a time t to each (x, y, theta), and compute desired linear and angular velocities at each point.

3. Trajectory Controller: A Python node runs at real-time frequency (10–20 Hz) and computes the velocity command (Twist) to send to /cmd_vel, using proportional control (kp) to follow the trajectory.

4. Gazebo + RViz: These are launched to visualize the robot moving on the path in simulation.

Launch Files:
gazebo_control.launch.py:
-------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_tracker',
            executable='controller_node',
            output='screen'
        )
    ])

simulation.launch.py:
--------------------
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_tracker',
            executable='controller_node',
            name='trajectory_follower',
            output='screen'
        )
    ])

controller.py:
--------------
import numpy as np

def find_closest_point(trajectory, position):
    dists = [np.linalg.norm(np.array([x, y]) - np.array(position)) for x, y, _ in trajectory]
    return np.argmin(dists)

def compute_velocity_commands(current_pose, trajectory, lookahead=5):
    x, y, theta = current_pose
    idx = find_closest_point(trajectory, (x, y))
    target_idx = min(idx + lookahead, len(trajectory)-1)
    tx, ty, _ = trajectory[target_idx]

    angle_to_target = np.arctan2(ty - y, tx - x)
    angle_diff = angle_to_target - theta

    # Normalize angle
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

    linear_vel = 0.15
    angular_vel = 2.0 * angle_diff
    return linear_vel, angular_vel

main.py:
--------
from path_tracker.path_smoothing import smooth_path
from path_tracker.trajectory_generator import generate_trajectory
from path_tracker.controller import compute_velocity_commands
from path_tracker.simulator import simulate
import matplotlib.pyplot as plt

def main():
    waypoints = [(0, 0), (1, 2), (3, 4), (5, 3), (7, 5)]
    smooth = smooth_path(waypoints)
    trajectory = generate_trajectory(smooth)

    path = simulate(trajectory, compute_velocity_commands)

    # Plot
    x_traj = [p[0] for p in trajectory]
    y_traj = [p[1] for p in trajectory]
    x_sim = [p[0] for p in path]
    y_sim = [p[1] for p in path]

    plt.plot(x_traj, y_traj, label='Trajectory')
    plt.plot(x_sim, y_sim, label='Robot Path')
    plt.scatter(*zip(*waypoints), color='red', label='Waypoints')
    plt.legend()
    plt.title("Trajectory Tracking")
    plt.axis('equal')
    plt.grid()
    plt.show()

path_smoothing.py:
------------------
import numpy as np
from scipy import interpolate

def smooth_path(waypoints, num_points=200):
    waypoints = np.array(waypoints)
    x, y = waypoints[:, 0], waypoints[:, 1]
    t = np.linspace(0, 1, len(waypoints))
    t_new = np.linspace(0, 1, num_points)

    spl_x = interpolate.CubicSpline(t, x)
    spl_y = interpolate.CubicSpline(t, y)

    x_smooth = spl_x(t_new)
    y_smooth = spl_y(t_new)

    return np.vstack((x_smooth, y_smooth)).T

ros_controller.py:
------------------
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

simulator.py:
-------------
import matplotlib.pyplot as plt
import numpy as np

def simulate(trajectory, controller_func, total_time=20.0, dt=0.1):
    x, y, theta = trajectory[0][0], trajectory[0][1], 0.0
    path = [(x, y)]

    for _ in np.arange(0, total_time, dt):
        v, w = controller_func((x, y, theta), trajectory)
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt
        path.append((x, y))

    return path

trajectory_generator.py:
------------------------
def generate_trajectory(smooth_path, velocity=0.1, dt=0.1):
    trajectory = []
    time = 0.0
    for i in range(len(smooth_path)):
        x, y = smooth_path[i]
        trajectory.append((x, y, time))
        time += dt
    return trajectory

How to Run (For 3-Minute Demo)
------------------------------
Step-by-step Terminal Commands

1. Open Terminal 1 (Launch Simulation + Path + Trajectory + Controller)

cd ~/turtlebot3_path_tracking/ros2_ws
source install/setup.bash
ros2 launch path_tracker simulation.launch.py

This will do everything:

- Spawn TurtleBot3 in Gazebo
- Launch RViz
- Run smoothing node
- Run trajectory generator
- Run controller node

2. (Optional) Terminal 2: Monitor robot velocity

ros2 topic echo /cmd_vel

You’ll see messages like:

linear:
  x: 0.2
angular:
  z: -0.05

Which means robot is moving forward and rotating slightly to adjust heading.

3. (Optional) Terminal 3: List active nodes

ros2 node list

Useful to debug if something is not running.

Notes and Limitations
---------------------
- Simulation runs smoothly only for 3–4 minutes on low-end laptop.
- Trajectory assumes constant velocity; no acceleration model is used.
- You can extend the controller with PID or MPC later.

Tools and Dependencies
----------------------
- ROS 2 Humble or Jazzy
- Python 3.10+
- Packages: rclpy, geometry_msgs, scipy, numpy, gazebo_ros, rviz2

Future Improvements
-------------------
- Add obstacle avoidance (e.g., DWA or MPC)
- Use localization (AMCL or EKF) instead of simulated TF
- Record bag files for demo purposes
