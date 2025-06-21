import numpy as np
from scipy.spatial.transform import Rotation as R
from trapezoidal_motion import ScalarTrapezoidalMotion

class PoseTrapezoidalInterpolator:
    def __init__(self, poses, v_max, a_max):
        self.segments = []
        self.t_knots = [0.0]
        self.num_segments = len(poses) - 1

        for i in range(self.num_segments):
            p0, r0 = poses[i]
            p1, r1 = poses[i + 1]

            delta_p = np.linalg.norm(p1 - p0)
            r_rel = r1 * r0.inv()
            angle = np.linalg.norm(r_rel.as_rotvec())

            motion_distance = np.sqrt(delta_p ** 2 + angle ** 2)
            scalar_interp = ScalarTrapezoidalMotion(motion_distance, v_max, a_max)

            self.t_knots.append(self.t_knots[-1] + scalar_interp.T)

            self.segments.append({
                "p0": p0,
                "p1": p1,
                "r0": r0,
                "r1": r1,
                "angle": angle,
                "scalar_interp": scalar_interp,
                "distance": delta_p
            })

    def total_time(self):
        return self.t_knots[-1]

    def _find_segment(self, t):
        for i in range(self.num_segments):
            if t < self.t_knots[i + 1]:
                return i, t - self.t_knots[i]
        return self.num_segments - 1, t - self.t_knots[-2]

    def get_pose(self, t):
        i, t_local = self._find_segment(t)
        seg = self.segments[i]
        s = seg["scalar_interp"].position(t_local)
        alpha = s / np.sqrt(seg["distance"] ** 2 + seg["angle"] ** 2) if (seg["distance"] ** 2 + seg["angle"] ** 2) > 1e-6 else 0
        pos = seg["p0"] + alpha * (seg["p1"] - seg["p0"])
        theta = alpha * seg["angle"]
        axis = (seg["r1"] * seg["r0"].inv()).as_rotvec()
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-6 else np.array([1.0, 0.0, 0.0])
        quat = (R.from_rotvec(axis * theta) * seg["r0"]).as_quat()
        return pos, quat

    def get_velocity(self, t):
        i, t_local = self._find_segment(t)
        seg = self.segments[i]
        v = seg["scalar_interp"].velocity(t_local)
        axis = (seg["r1"] * seg["r0"].inv()).as_rotvec()
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-6 else np.array([1.0, 0.0, 0.0])
        total = np.sqrt(seg["distance"] ** 2 + seg["angle"] ** 2)
        dir_vec = (seg["p1"] - seg["p0"]) / seg["distance"] if seg["distance"] > 1e-9 else np.zeros(3)
        return v * dir_vec, v * axis / total

    def get_acceleration(self, t):
        i, t_local = self._find_segment(t)
        seg = self.segments[i]
        a = seg["scalar_interp"].acceleration(t_local)
        axis = (seg["r1"] * seg["r0"].inv()).as_rotvec()
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-6 else np.array([1.0, 0.0, 0.0])
        total = np.sqrt(seg["distance"] ** 2 + seg["angle"] ** 2)
        dir_vec = (seg["p1"] - seg["p0"]) / seg["distance"] if seg["distance"] > 1e-9 else np.zeros(3)
        return a * dir_vec, a * axis / total
