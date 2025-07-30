# trajectory_generator.py
def generate_trajectory(smooth_path, velocity=0.1, dt=0.1):
    trajectory = []
    time = 0.0
    for i in range(len(smooth_path)):
        x, y = smooth_path[i]
        trajectory.append((x, y, time))
        time += dt
    return trajectory
