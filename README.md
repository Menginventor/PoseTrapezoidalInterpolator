# Trapezoidal Pose-Interpolator

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

## 📐 Mathematical Foundation

This library interpolates between 6-DOF poses using a unified scalar trajectory governed by trapezoidal motion profiles.

### 1D Motion Profile

The scalar motion profile is defined with a maximum velocity $v_{\text{max}}$ and acceleration $a_{\text{max}}$, resulting in either:

- **Trapezoidal profile** if distance $S$ is long enough to cruise.
- **Triangular profile** if distance is too short to reach $v_{\text{max}}$.

The trajectory satisfies:

$$
\begin{aligned}
v(t) &=
\begin{cases}
a_{\text{max}} t & 0 \leq t < t_1 \\\\
v_{\text{max}} & t_1 \leq t < t_2 \\\\
a_{\text{max}} (T - t) & t_2 \leq t \leq T
\end{cases} \\\\
x(t) &= \int_0^t v(\tau)\, d\tau
\end{aligned}
$$

---

### 3D Translation

Each segment is interpolated by reducing vector displacement to a scalar path of length $S = \| \mathbf{p}_1 - \mathbf{p}_0 \|$, applying the 1D profile along the unit direction.

---

### Rotation as Axis + Angle

Quaternion interpolation is performed using the exponential map of the relative rotation:

$$
\mathbf{q}_{\text{rel}} = \mathbf{q}_1 \cdot \mathbf{q}_0^{-1}
$$

Convert to axis-angle:

$$
\theta = \| \mathbf{r} \|, \quad \mathbf{u} = \frac{\mathbf{r}}{\theta}, \quad \text{where } \mathbf{r} = \log(\mathbf{q}_{\text{rel}})
$$

Then use the **same scalar profile** for $\theta(t)$ to compute rotation:

$$
\mathbf{R}(t) = \exp\left( \theta(t)\, \hat{\mathbf{u}} \right) \cdot \mathbf{R}_0
$$

---

### Unified Pose Trajectory

Each pose segment is parameterized by:

$$
S_i = \sqrt{ \| \Delta \mathbf{p}_i \|^2 + \theta_i^2 }
$$

And a **shared scalar trajectory** $s(t) \in [0, S_i]$ is used to synchronize translation and rotation.

```python
p(t) = (1 - a(t)) * p0 + a(t) * p1
q(t) = slerp(q0, q1, a(t))
```

where $a(t) = \frac{s(t)}{S_i}$ is the scalar fraction from trapezoidal profile.


## 📜 License

MIT License — feel free to use, modify, and share with attribution.
