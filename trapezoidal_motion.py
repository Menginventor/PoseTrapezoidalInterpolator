import numpy as np

def compute_scalar_limits(direction_unit_vec, v_max_vec, a_max_vec, epsilon=1e-9):
    """
    Project the multidimensional velocity and acceleration limits onto the direction unit vector.
    This gives the effective scalar limits along the trajectory direction.
    """
    v_scalar = np.min([
        abs(v_max_vec[i]) / abs(direction_unit_vec[i]) if abs(direction_unit_vec[i]) > epsilon else np.inf
        for i in range(len(v_max_vec))
    ])
    a_scalar = np.min([
        abs(a_max_vec[i]) / abs(direction_unit_vec[i]) if abs(direction_unit_vec[i]) > epsilon else np.inf
        for i in range(len(a_max_vec))
    ])
    return v_scalar, a_scalar

class ScalarTrapezoidalMotion:
    """
    1D Trapezoidal or Triangular motion planner for a scalar path length.
    It automatically adjusts to triangle profile if v_max cannot be reached.
    """
    def __init__(self, total_distance, max_velocity, max_acceleration):
        self.S = total_distance
        self.v_max_input = max_velocity
        self.a_max = max_acceleration

        if self.S < 1e-9:
            self.v_max = 0
            self.ta = self.tc = self.T = 0
            self.profile_type = "zero"
            return

        # Determine profile type based on achievable peak velocity
        v_peak = np.sqrt(self.a_max * self.S)
        if v_peak < self.v_max_input:
            self.v_max = v_peak
            self.ta = self.v_max / self.a_max
            self.tc = 0
            self.T = 2 * self.ta
            self.profile_type = "triangle"
        else:
            self.v_max = self.v_max_input
            self.ta = self.v_max / self.a_max
            Sa = 0.5 * self.a_max * self.ta**2
            self.tc = (self.S - 2 * Sa) / self.v_max
            self.T = 2 * self.ta + self.tc
            self.profile_type = "trapezoid"

    def position(self, t):
        if t < 0:
            return 0
        elif t < self.ta:
            return 0.5 * self.a_max * t**2
        elif t < self.ta + self.tc:
            return 0.5 * self.a_max * self.ta**2 + self.v_max * (t - self.ta)
        elif t <= self.T:
            td = t - self.ta - self.tc
            return (
                0.5 * self.a_max * self.ta**2 +
                self.v_max * self.tc +
                self.v_max * td -
                0.5 * self.a_max * td**2
            )
        else:
            return self.S

    def velocity(self, t):
        if t < 0:
            return 0
        elif t < self.ta:
            return self.a_max * t
        elif t < self.ta + self.tc:
            return self.v_max
        elif t <= self.T:
            td = t - self.ta - self.tc
            return self.v_max - self.a_max * td
        else:
            return 0

    def acceleration(self, t):
        if t < 0 or t > self.T:
            return 0
        elif t < self.ta:
            return self.a_max
        elif t < self.ta + self.tc:
            return 0
        else:
            return -self.a_max

class MultiDimensionalTrapezoidalMotion:
    """
    Multi-dimensional trapezoidal motion planner using a scalar profile along a direction vector.
    """
    def __init__(self, start_pos, end_pos, v_max_vec, a_max_vec):
        self.start = np.array(start_pos)
        self.end = np.array(end_pos)
        self.delta = self.end - self.start
        self.distance = np.linalg.norm(self.delta)
        self.direction = self.delta / self.distance if self.distance > 1e-9 else np.zeros_like(self.delta)

        # Project multidimensional limits onto direction vector
        self.v_scalar, self.a_scalar = compute_scalar_limits(self.direction, v_max_vec, a_max_vec)
        self.scalar_profile = ScalarTrapezoidalMotion(self.distance, self.v_scalar, self.a_scalar)

    def get_position(self, t):
        scalar_pos = self.scalar_profile.position(t)
        return self.start + (scalar_pos / self.distance) * self.delta

    def get_velocity(self, t):
        scalar_vel = self.scalar_profile.velocity(t)
        return scalar_vel * self.direction

    def get_acceleration(self, t):
        scalar_acc = self.scalar_profile.acceleration(t)
        return scalar_acc * self.direction