# copyright 2025 Dr. Hamid Ossareh, Univ. of Vermont
# Grade field (position-based)

import random, math

class GradeField:
    def __init__(self, spacing=1.0, amp=6.0, seed=None):
        """
        spacing: knot spacing in meters
        amp:     max grade in percent
        """
        self.spacing, self.amp = spacing, amp
        self.ctrl = {}
        self.rng = random.Random(seed)

    def _val(self, k):
        # Control point value (grade, in %) at integer index k
        if k not in self.ctrl:
            self.ctrl[k] = self.rng.uniform(-self.amp, self.amp)
        return self.ctrl[k]

    def grade_at(self, x_m):
        """
        Grade (percent) as a function of longitudinal position x_m (meters).
        Cubic Hermite between control points every 'spacing' meters.
        """
        s = self.spacing
        k0 = math.floor(x_m / s)
        k1 = k0 + 1
        x0, x1 = k0 * s, (k0 + 1) * s

        # Neighbor control points for tangents
        y_1 = self._val(k0 - 1)
        y0  = self._val(k0)
        y1  = self._val(k1)
        y2  = self._val(k1 + 1)

        # Finite-difference tangents (per meter)
        m0 = (y1 - y_1) / (2 * s)
        m1 = (y2 - y0) / (2 * s)

        # Hermite basis on [x0, x1]
        u = (x_m - x0) / (x1 - x0)
        h00 =  2*u**3 - 3*u**2 + 1
        h10 =      u**3 - 2*u**2 + u
        h01 = -2*u**3 + 3*u**2
        h11 =      u**3 -     u**2

        return h00*y0 + h10*(x1 - x0)*m0 + h01*y1 + h11*(x1 - x0)*m1

    # backward compatible
    def grade(self, t_or_x):
        return self.grade_at(t_or_x)
