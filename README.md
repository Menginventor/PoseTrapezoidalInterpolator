# PoseTrapezoidalInterpolator

This repository provides a unified **pose interpolation** library using **trapezoidal motion profiles** for both translation and rotation. The core idea is to interpolate over a **scalar path** combining position and orientation distance into a single trajectory, guaranteeing **synchronized acceleration, cruise, and deceleration** for both motion components.

**Importantly, the motion stops at each pose** (i.e., zero velocity at keyframes), making it suitable for time-optimal and stop-at-waypoint trajectory planning. 

## 📦 Features

- Unified scalar interpolation for full 6-DOF pose (position + orientation)
- Each segment uses a **trapezoidal velocity profile** with:
  - Zero start and stop velocity
  - Optional cruising phase
- Supports:
  - **Position** interpolation (3D vector)
  - **Orientation** interpolation (quaternion or frame)
- Outputs per time step:
  - Position, Velocity, Acceleration
  - Orientation (Quaternion or Frame Axes)
  - Angular Velocity
- Optionally **customize `v_max` and `a_max`** for each segment
- Visualization tools:
  - Time plots of translation and rotation
  - Linear vs angular velocity comparison
  - Animated 3D frame motion with position trace
  - MP4 export of animation

## 📁 File Structure

```
pose_trapezoidal.py         # Unified pose interpolation class
trapezoidal_motion.py       # Scalar trapezoidal motion profile
examples/
  ├── simple_pose.py        # Minimal example: print interpolated poses
  ├── pose_trapezoidal_test.py  # Full interpolation + visualization
  ├── position_trape.py     # Linear interpolation test
  ├── orientation_trape.py  # Orientation-only test
  └── pose_interpolation.mp4    # Sample exported animation
```

## 🚀 Usage Example

```python
from pose_trapezoidal import PoseTrapezoidalInterpolator
from scipy.spatial.transform import Rotation as R
import numpy as np

# Define poses as (position, rotation) tuples
poses = [
    (np.array([0, 0, 0]), R.from_euler('xyz', [0, 0, 0])),
    (np.array([1, 1, 1]), R.from_euler('xyz', [90, 0, 0], degrees=True))
]

# Create interpolator (v_max and a_max can be per segment if needed)
interp = PoseTrapezoidalInterpolator(poses, v_max=1.0, a_max=2.0)

# Sample interpolated state at time t
t = 0.5
position, quaternion = interp.get_pose(t)
velocity, angular_velocity = interp.get_velocity(t)
```

## 🛠 Requirements

- Python 3.7+
- `numpy`
- `scipy`
- `matplotlib` (for plotting and animation)

Install all requirements:
```bash
pip install -r requirements.txt
```

## 📽 Demo

Run this script to generate a full trajectory visualization and export an animated MP4:

```bash
python examples/pose_trapezoidal_test.py
```

## 📜 License

MIT License — feel free to use, modify, and share with attribution.
