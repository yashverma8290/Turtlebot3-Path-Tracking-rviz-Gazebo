# controller.py
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
