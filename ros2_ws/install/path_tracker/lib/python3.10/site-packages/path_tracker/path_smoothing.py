# path_smoothing.py
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
