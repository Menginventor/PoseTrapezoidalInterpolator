import numpy as np
from scipy.spatial.transform import Rotation as R
from pose_trapezoidal import PoseTrapezoidalInterpolator

# --- Create random poses -----------------------------------------------------
np.random.seed(42)
num_poses = 5
positions = [np.random.uniform(-1, 1, 3) for _ in range(num_poses)]
rotations = [R.random() for _ in range(num_poses)]
poses = list(zip(positions, rotations))

# --- Create interpolator -----------------------------------------------------
v_max = 1.0
a_max = 2.0
interp = PoseTrapezoidalInterpolator(poses, v_max, a_max)

# --- Sample and print interpolated values ------------------------------------
t_all = np.linspace(0, interp.total_time(), 10)  # 10 sample points

print(f"{'Time':>6} | {'Position (x,y,z)':^35} | {'RotVec (x,y,z)':^35} | {'Quaternion (x,y,z,w)':^40}")
print("-" * 125)
for t in t_all:
    pos, quat = interp.get_pose(t)
    rvec = R.from_quat(quat).as_rotvec()
    print(f"{t:6.2f} | "
          f"{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f} | "
          f"{rvec[0]:7.3f}, {rvec[1]:7.3f}, {rvec[2]:7.3f} | "
          f"{quat[0]:7.3f}, {quat[1]:7.3f}, {quat[2]:7.3f}, {quat[3]:7.3f}")
