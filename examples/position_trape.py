import numpy as np
import matplotlib.pyplot as plt
from trapezoidal_motion import MultiDimensionalTrapezoidalMotion
'''
Position only
'''
def random_point_in_sphere(radius=1.0, offset=None):
    direction = np.random.normal(size=3)
    direction /= np.linalg.norm(direction)
    r = radius * np.cbrt(np.random.rand())
    if offset is None:
        offset = np.zeros(3)
    return np.asarray(offset) + r * direction

# Generate 10 random waypoints in a unit sphere
np.random.seed(1)
waypoints = [random_point_in_sphere() for _ in range(10)]

# Velocity and acceleration limits per axis
v_max_vec = np.array([1.0, 1.0, 1.0])
a_max_vec = np.array([2.0, 2.0, 2.0])

# Create trajectory segments
segments = [
    MultiDimensionalTrapezoidalMotion(waypoints[i], waypoints[i+1], v_max_vec, a_max_vec)
    for i in range(len(waypoints) - 1)
]

# Sample each segment
positions, velocities, accelerations = [], [], []
times_global, knot_times = [], [0.0]
time_cursor = 0.0
N = 100  # number of time samples per segment

for i, seg in enumerate(segments):
    ts_local = np.linspace(0, seg.scalar_profile.T, N, endpoint=(i == len(segments) - 1))
    for tloc in ts_local:
        positions.append(seg.get_position(tloc))
        velocities.append(seg.get_velocity(tloc))
        accelerations.append(seg.get_acceleration(tloc))
        times_global.append(time_cursor + tloc)
    time_cursor += seg.scalar_profile.T
    knot_times.append(time_cursor)

positions = np.vstack(positions)
velocities = np.vstack(velocities)
accelerations = np.vstack(accelerations)
times_global = np.array(times_global)

# Plot position, velocity, acceleration for x, y, z
fig, axes = plt.subplots(3, 3, figsize=(14, 8), sharex=True)
labels = ['x', 'y', 'z']
datas = [positions, velocities, accelerations]
titles = ['Position', 'Velocity', 'Acceleration']

for i in range(3):  # x, y, z axes
    for j in range(3):  # pos, vel, acc
        ax = axes[j][i]
        ax.plot(times_global, datas[j][:, i])
        ax.set_title(f"{titles[j]} - {labels[i]}")
        ax.grid(True)
        if j == 2:
            ax.set_xlabel("Time (s)")
        if i == 0:
            ax.set_ylabel(titles[j])

plt.tight_layout()
plt.show()
