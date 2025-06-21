'''
Orientation only
'''
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from trapezoidal_motion import ScalarTrapezoidalMotion

# Step 1: Generate random orientations
np.random.seed(42)
rotations = [R.random() for _ in range(10)]

# Step 2: Decompose each relative rotation into axis + angle
segments = []
for i in range(len(rotations) - 1):
    r1 = rotations[i]
    r2 = rotations[i + 1]
    r_rel = r2 * r1.inv()
    rvec = r_rel.as_rotvec()
    angle = np.linalg.norm(rvec)
    axis = rvec / angle if angle > 1e-6 else np.array([1.0, 0.0, 0.0])
    segments.append({
        "r_start": r1,
        "axis": axis,
        "angle": angle
    })

# Step 3: Interpolate each segment with independent trapezoidal profile
angular_velocities = []
frame_x_out, frame_y_out, frame_z_out = [], [], []
times = []
time_cursor = 0.0
N_per_segment = 100

for seg in segments:
    profile = ScalarTrapezoidalMotion(seg["angle"], 1.0, 2.0)
    ts = np.linspace(0, profile.T, N_per_segment)
    for t in ts:
        theta = profile.position(t)
        w = profile.velocity(t)
        axis = seg["axis"]
        r_interp = R.from_rotvec(theta * axis) * seg["r_start"]
        Rmat = r_interp.as_matrix()
        frame_x_out.append(Rmat[:, 0])
        frame_y_out.append(Rmat[:, 1])
        frame_z_out.append(Rmat[:, 2])
        angular_velocities.append(w * axis)
        times.append(time_cursor + t)
    time_cursor += profile.T

angular_velocities = np.array(angular_velocities)
frame_x_out = np.array(frame_x_out)
frame_y_out = np.array(frame_y_out)
frame_z_out = np.array(frame_z_out)
times = np.array(times)

# Plot angular velocity
fig1, axes1 = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
labels = ['ω_x', 'ω_y', 'ω_z']
for i in range(3):
    axes1[i].plot(times, angular_velocities[:, i])
    axes1[i].set_ylabel(labels[i])
    axes1[i].grid(True)
axes1[2].set_xlabel("Time (s)")
fig1.suptitle("Angular Velocity (ω) Components Over Time")
plt.tight_layout()

# Plot frame axes over time
fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
frame_labels = ['x', 'y', 'z']
colors = ['r', 'g', 'b']
for i in range(3):  # Axis components x, y, z
    axes2[i].plot(times, frame_x_out[:, i], label='Frame X', color='r')
    axes2[i].plot(times, frame_y_out[:, i], label='Frame Y', color='g')
    axes2[i].plot(times, frame_z_out[:, i], label='Frame Z', color='b')
    axes2[i].set_ylabel(f"{frame_labels[i]} component")
    axes2[i].grid(True)
    axes2[i].legend()
axes2[2].set_xlabel("Time (s)")
fig2.suptitle("Reference Frame Axes (Rotation Matrix Columns)")
plt.tight_layout()

plt.show()
