# Trapezoidal-Profile Pose-Interpolator

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

The **minimum distance required** to reach the maximum velocity is derived by computing the distance needed to accelerate from zero to $v_{\text{max}}$ and then decelerate back to zero without a cruise phase. This forms a triangular velocity profile.

The time to accelerate to $v_{\text{max}}$ is:

$$
t_a = \frac{v_{\text{max}}}{a_{\text{max}}}
$$

The distance covered during acceleration is:

$$
S_a = \frac{1}{2} a_{\text{max}} t_a^2 = \frac{1}{2} a_{\text{max}} \left( \frac{v_{\text{max}}}{a_{\text{max}}} \right)^2 = \frac{v_{\text{max}}^2}{2a_{\text{max}}}
$$

Since the deceleration phase mirrors the acceleration phase, the **minimum total distance** required to form a trapezoidal profile is:

$$
S_{\text{min}} = 2S_a = \frac{v_{\text{max}}^2}{a_{\text{max}}}
$$

If the desired travel distance $S < S_{\text{min}}$, the profile must be **triangular**, reaching a peak velocity lower than $v_{\text{max}}$.

#### Triangular Profile (When $S < S_{\text{min}}$)

When the total distance $S$ is less than $S_{\text{min}}$, the motion cannot reach $v_{\text{max}}$. Instead, it follows a **triangular profile** with a lower peak velocity $v_{\text{peak}}$.

We assume symmetric acceleration and deceleration, and solve for $v_{\text{peak}}$ such that:

$$
S = 2 \cdot \frac{1}{2} \cdot \frac{v_{\text{peak}}^2}{a_{\text{max}}} = \frac{v_{\text{peak}}^2}{a_{\text{max}}}
$$

Solving for $v_{\text{peak}}$ gives:

$$
v_{\text{peak}} = \sqrt{a_{\text{max}} \cdot S}
$$

This ensures the profile accelerates to $v_{\text{peak}}$ and then decelerates back to zero over the total distance $S$.


$$
t_a = \frac{v_{\text{peak}}}{a_{\text{max}}} = \frac{\sqrt{a_{\text{max}} \cdot S}}{a_{\text{max}}} = \sqrt{\frac{S}{a_{\text{max}}}}
$$

- Deceleration time is the same due to symmetry:

$$
t_d = t_a = \sqrt{\frac{S}{a_{\text{max}}}}
$$

- Total motion duration:

$$
T = t_a + t_d = 2 \sqrt{\frac{S}{a_{\text{max}}}}
$$

If trapeziod case, it include crusing phase with $v(t) = v_max$

$$
T_C = (S-S_{min})/v_max
$$

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

Each segment is interpolated by reducing the vector displacement to a scalar path of length

$$
S = \| \mathbf{p}_1 - \mathbf{p}_0 \|,
$$

and applying the 1D trapezoidal/triangular motion profile along this scalar path.

As we interpolate $s(t)$ along the path $S$, the vector position can be computed as:

$$
\mathbf{y}(t) = (1 - a(t)) \mathbf{p}_0 + a(t) \mathbf{p}_1,
$$

where the normalized scalar progress is defined as:

$$
a(t) = \frac{s(t)}{S}.
$$

To ensure the per-axis velocity and acceleration limits ($v_{\text{max}}$, $a_{\text{max}}$) are respected across all vector elements, we define the **unit direction vector**:

$$
\hat{\mathbf{d}} = \frac{\mathbf{p}_1 - \mathbf{p}_0}{\|\mathbf{p}_1 - \mathbf{p}_0\|}.
$$

Then, the scalar velocity and acceleration limits used in the 1D profile are computed by scaling down component-wise:


$$
v_{\text{scalar}} = \min_i \left( \frac{v_{\text{max},i}}{|\hat{d}_i|} \right)
$$

$$
a_{\text{scalar}} = \min_i \left( \frac{a_{\text{max},i}}{|\hat{d}_i|} \right),
$$

for all $i$ where $\hat{d}_i \neq 0$, and treating divisions by zero as $\infty$.

This guarantees that the interpolated vector respects all per-axis constraints during the motion.


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

Then use the **trapezoidal scalar profile** for $\theta(t)$ to compute rotation:

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
