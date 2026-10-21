import numpy as np


class HandMapper:
    # image u, v (0..1) -> world axes. active_u/v region gets stretched over the fence

    def __init__(self, fence, horizontal="x", vertical="z", invert_horizontal=False,
                 active_u=(0.15, 0.85), active_v=(0.15, 0.85), margin=0.1):
        self.h_axis = horizontal
        self.v_axis = vertical
        self.invert_h = invert_horizontal
        self.u_lo, self.u_hi = active_u
        self.v_lo, self.v_hi = active_v
        self.h_range = fence.range(horizontal, margin)
        self.v_range = fence.range(vertical, margin)

    def _s(self, u):
        s = np.clip((u - self.u_lo) / (self.u_hi - self.u_lo), 0.0, 1.0)
        return 1.0 - s if self.invert_h else s

    def _t(self, v):
        # top of image = high end
        return 1.0 - np.clip((v - self.v_lo) / (self.v_hi - self.v_lo), 0.0, 1.0)

    def to_world(self, u, v):
        # returns {axis: value}
        h_lo, h_hi = self.h_range
        v_lo, v_hi = self.v_range
        return {
            self.h_axis: h_lo + self._s(u) * (h_hi - h_lo),
            self.v_axis: v_lo + self._t(v) * (v_hi - v_lo),
        }

    def velocity_to_world(self, du, dv):
        # image vel -> m/s (ignores clamping)
        h_scale = (self.h_range[1] - self.h_range[0]) / (self.u_hi - self.u_lo)
        v_scale = (self.v_range[1] - self.v_range[0]) / (self.v_hi - self.v_lo)
        return {
            self.h_axis: (-du if self.invert_h else du) * h_scale,
            self.v_axis: -dv * v_scale,
        }

    def h_to_u(self, value):
        # world -> image u, for drawing
        h_lo, h_hi = self.h_range
        s = (value - h_lo) / (h_hi - h_lo)
        if self.invert_h:
            s = 1.0 - s
        return self.u_lo + s * (self.u_hi - self.u_lo)

    def v_to_v(self, value):
        # world -> image v, for drawing
        v_lo, v_hi = self.v_range
        t = (value - v_lo) / (v_hi - v_lo)
        return self.v_lo + (1.0 - t) * (self.v_hi - self.v_lo)
