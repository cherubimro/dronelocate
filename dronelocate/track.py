"""Track filter. Lives outside supernode.py so the math is testable with no
transport in the way, same as tdoa.py."""

import collections

import numpy as np

from .geo import C_LIGHT

# 99% chi-square gate by measurement count. Generous on purpose: this gates
# starved-burst updates against dragging a track, not routine measurements.
_CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345, 4: 13.277, 5: 15.086}


class Tracker:
    """Constant-velocity Kalman filter in ENU.

    Deliberately simple. A particle filter is the right answer for initial
    acquisition because the TDOA cost surface is multimodal, but the grid
    seed inside solve_tdoa already handles that, so by the time a fix
    reaches here it is unimodal and a linear filter is appropriate.
    """

    def __init__(self, tid, fix, t):
        self.tid = tid
        self.x = np.concatenate([fix.enu, np.zeros(3)])
        self.P = np.eye(6) * 1e4
        self.P[:3, :3] = fix.cov
        self.P[3:, 3:] = np.eye(3) * 400.0   # 20 m/s 1-sigma initial speed
        self.t = t
        self.hits = 1
        self.misses = 0
        self.tight_hits = 0
        self.history = collections.deque(maxlen=400)
        self.history.append((t, fix.enu.copy()))

    def predict(self, t):
        dt = max(t - self.t, 1e-3)
        f = np.eye(6)
        f[0, 3] = f[1, 4] = f[2, 5] = dt
        # Anisotropic process noise. A quadrotor manoeuvres hard laterally but
        # climbs slowly, and the vertical is the axis the geometry constrains
        # worst -- isotropic Q lets a weakly observed altitude random-walk
        # until the filtered track is worse than the raw fixes feeding it.
        acc_h, acc_v = 6.0 ** 2, 1.5 ** 2
        g2 = np.array([dt ** 2 / 2] * 3 + [dt] * 3) ** 2
        q = np.diag(g2 * np.array([acc_h, acc_h, acc_v, acc_h, acc_h, acc_v]))
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.t = t

    def update(self, fix, t, alt_prior=None, alt_sigma=60.0):
        self.predict(t)
        h = np.zeros((3, 6))
        h[:3, :3] = np.eye(3)
        y = fix.enu - h @ self.x
        s = h @ self.P @ h.T + fix.cov
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(6) - k @ h) @ self.P

        # Same altitude prior the solver uses, reapplied here as a scalar
        # pseudo-measurement. Without it the filter integrates a long run of
        # weak vertical measurements into a confident drift.
        if alt_prior is not None:
            hz = np.zeros((1, 6)); hz[0, 2] = 1.0
            sz = hz @ self.P @ hz.T + np.array([[alt_sigma ** 2]])
            kz = self.P @ hz.T @ np.linalg.inv(sz)
            self.x = self.x + (kz @ np.array([[alt_prior - self.x[2]]])).ravel()
            self.P = (np.eye(6) - kz @ hz) @ self.P

        self.hits += 1
        self.misses = 0
        self.history.append((t, self.x[:3].copy()))

    def update_tdoa(self, pos_a, pos_b, tdoa_s, R, t):
        """Tight coupling: a burst too starved to solve still measures.

        Fewer than three surviving pairs cannot produce a fix, but each pair
        is still a hyperboloid constraint, and an existing track carries
        enough prior to use it -- fix-then-filter throws exactly this
        information away. EKF update with the measurement linearised at the
        predicted position, innovation-gated on its own chi-square so one
        bad pair cannot drag the track. Returns True if applied.
        """
        pos_a = np.atleast_2d(np.asarray(pos_a, dtype=float))
        pos_b = np.atleast_2d(np.asarray(pos_b, dtype=float))
        tdoa_s = np.asarray(tdoa_s, dtype=float)
        k = len(tdoa_s)
        self.predict(t)

        p = self.x[:3]
        da = np.linalg.norm(p[None, :] - pos_a, axis=1)
        db = np.linalg.norm(p[None, :] - pos_b, axis=1)
        y = tdoa_s - (da - db) / C_LIGHT

        h = np.zeros((k, 6))
        h[:, :3] = ((p[None, :] - pos_a) / np.clip(da[:, None], 1e-6, None)
                    - (p[None, :] - pos_b) / np.clip(db[:, None], 1e-6, None)
                    ) / C_LIGHT

        s = h @ self.P @ h.T + np.asarray(R, dtype=float)
        try:
            sinv = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            return False
        nis = float(y @ sinv @ y)
        gate = _CHI2_99.get(k, _CHI2_99[5] + 2.0 * (k - 5))
        if nis > gate:
            return False

        kk = self.P @ h.T @ sinv
        self.x = self.x + kk @ y
        self.P = (np.eye(6) - kk @ h) @ self.P
        self.tight_hits += 1
        self.history.append((t, self.x[:3].copy()))
        return True

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]
