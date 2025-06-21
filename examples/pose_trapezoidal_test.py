import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib import animation
from pose_trapezoidal import PoseTrapezoidalInterpolator

# --- Generate random poses ----------------------------------------------------
np.random.seed(42)
num_poses = 10
positions = [np.random.uniform(-1, 1, size=3) for _ in range(num_poses)]
rotations = [R.random() for _ in range(num_poses)]
poses = list(zip(positions, rotations))

# --- Create interpolation object ---------------------------------------------
v_max = 1.0
a_max = 2.0
interp = PoseTrapezoidalInterpolator(poses, v_max, a_max)

# --- Sample trajectory -------------------------------------------------------
t_all = np.linspace(0, interp.total_time(), 600)
positions_out = []
velocities_out = []
accelerations_out = []
frame_x_out = []
frame_y_out = []
frame_z_out = []
angular_velocities = []

for t in t_all:
    pos, quat = interp.get_pose(t)
    vel, omega = interp.get_velocity(t)
    acc, _ = interp.get_acceleration(t)
    Rmat = R.from_quat(quat).as_matrix()

    positions_out.append(pos)
    velocities_out.append(vel)
    accelerations_out.append(acc)
    angular_velocities.append(omega)
    frame_x_out.append(Rmat[:, 0])
    frame_y_out.append(Rmat[:, 1])
    frame_z_out.append(Rmat[:, 2])

positions_out = np.vstack(positions_out)
velocities_out = np.vstack(velocities_out)
accelerations_out = np.vstack(accelerations_out)
angular_velocities = np.vstack(angular_velocities)
frame_x_out = np.vstack(frame_x_out)
frame_y_out = np.vstack(frame_y_out)
frame_z_out = np.vstack(frame_z_out)

# --- Plot 1: Translation -----------------------------------------------------
fig1, axs1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
labels = ['x', 'y', 'z']
for i in range(3):
    axs1[0].plot(t_all, positions_out[:, i], label=labels[i])
    axs1[1].plot(t_all, velocities_out[:, i], label=labels[i])
    axs1[2].plot(t_all, accelerations_out[:, i], label=labels[i])
axs1[0].set_ylabel('Position')
axs1[1].set_ylabel('Velocity')
axs1[2].set_ylabel('Acceleration')
axs1[2].set_xlabel('Time (s)')
for ax in axs1:
    ax.grid(True)
    ax.legend()
fig1.suptitle("Translation Interpolation")

# --- Plot 2: Orientation Axes and Angular Velocity ---------------------------
fig2, axs2 = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
for i in range(3):
    axs2[0].plot(t_all, frame_x_out[:, i], label=labels[i])
    axs2[1].plot(t_all, frame_y_out[:, i], label=labels[i])
    axs2[2].plot(t_all, frame_z_out[:, i], label=labels[i])
    axs2[3].plot(t_all, angular_velocities[:, i], label=labels[i])
axs2[0].set_ylabel('Frame X')
axs2[1].set_ylabel('Frame Y')
axs2[2].set_ylabel('Frame Z')
axs2[3].set_ylabel('Angular Velocity')
axs2[3].set_xlabel('Time (s)')
for ax in axs2:
    ax.grid(True)
    ax.legend()
fig2.suptitle("Orientation Frame Vectors and Angular Velocity")

# --- Plot 3: Velocity Comparison ---------------------------------------------
fig3, axs3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for i in range(3):
    axs3[i].plot(t_all, velocities_out[:, i], label=f'Linear {labels[i]}', linestyle='-')
    axs3[i].plot(t_all, angular_velocities[:, i], label=f'Angular {labels[i]}', linestyle='--')
    axs3[i].set_ylabel(f"Vel {labels[i]}")
    axs3[i].legend()
    axs3[i].grid(True)
axs3[-1].set_xlabel("Time (s)")
fig3.suptitle("Linear vs Angular Velocity")

# --- Plot 4: Animated 3D Frame with Position Trace ---------------------------
fig4 = plt.figure(figsize=(8, 8))
ax = fig4.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Animated Pose Interpolation")

trace, = ax.plot([], [], [], 'k--', linewidth=1)
frame_quivers = []

def init_anim():
    for color in ['r', 'g', 'b']:
        q = ax.quiver(0, 0, 0, 0, 0, 0, color=color)
        frame_quivers.append(q)
    trace.set_data([], [])
    trace.set_3d_properties([])
    return frame_quivers + [trace]

def update_anim(i):
    for q in frame_quivers:
        q.remove()
    frame_quivers.clear()

    R_frame = np.column_stack([frame_x_out[i], frame_y_out[i], frame_z_out[i]])
    origin = positions_out[i]
    for j, color in enumerate(['r', 'g', 'b']):
        vec = R_frame[:, j] * 0.3
        q = ax.quiver(*origin, *vec, color=color)
        frame_quivers.append(q)

    trace.set_data(positions_out[:i+1, 0], positions_out[:i+1, 1])
    trace.set_3d_properties(positions_out[:i+1, 2])
    return frame_quivers + [trace]

ani = animation.FuncAnimation(fig4, update_anim, frames=len(t_all),
                              init_func=init_anim, blit=False, interval=30, repeat=True)

# --- Save animation as MP4 ---------------------------------------------------
print("Saving file...")
ani.save("pose_interpolation.mp4", fps=30, dpi=150)
print("Video saved as pose_interpolation.mp4")

# --- Show all plots ----------------------------------------------------------
plt.tight_layout()
plt.show()
