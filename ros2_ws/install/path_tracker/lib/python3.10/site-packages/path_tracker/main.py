# main.py

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
