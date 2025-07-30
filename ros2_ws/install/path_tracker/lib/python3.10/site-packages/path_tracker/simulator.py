# simulator.py
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
