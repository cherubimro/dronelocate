"""Track filter. Lives outside supernode.py so the math is testable with no
transport in the way, same as tdoa.py."""

import collections

import numpy as np

from .geo import C_LIGHT

# 99% chi-square gate by measurement count. Generous on purpose: this gates
# starved-burst updates against dragging a track, not routine measurements.
_CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345, 4: 13.277, 5: 15.086, 6: 16.812}


def wrap_pi(a):
    """Angle to (-pi, pi]. An azimuth innovation that is not wrapped turns a
    plot at 359.9 deg against a track at 0.1 deg into a 6 rad residual, which
    the gate rejects and the update, if it ran, would fling across the
    sky."""
    a = float(a)
    w = (a + np.pi) % (2.0 * np.pi) - np.pi
    return np.pi if w == -np.pi else w


def radar_measurement(x, radar_enu, psi):
    """h(x) and H for a radar plot: slant range and antenna-frame azimuth.

    Slant range is |p - o| in 3D, so the track's altitude enters the range
    directly -- the radar measures the hypotenuse, and treating it as
    horizontal distance would bias every fused position outward by
    r(1 - cos el). Azimuth is the true bearing atan2(dE, dN) minus the
    mounting offset psi (true bearing of the antenna's spoke 0).
    """
    d = np.asarray(x[:3], dtype=float) - np.asarray(radar_enu, dtype=float)
    r = float(np.linalg.norm(d))
    rho2 = float(d[0] ** 2 + d[1] ** 2)
    r = max(r, 1e-6)
    rho2 = max(rho2, 1e-6)
    h = np.array([r, wrap_pi(np.arctan2(d[0], d[1]) - psi)])
    H = np.zeros((2, 6))
    H[0, :3] = d / r
    H[1, 0] = d[1] / rho2
    H[1, 1] = -d[0] / rho2
    return h, H


def radar_rates(x, radar_enu):
    """d/dt of (range, azimuth) along the track's own velocity. A timing
    error dt on the plot moves the measurement by rates * dt, which is how a
    latency uncertainty becomes a measurement covariance."""
    d = np.asarray(x[:3], dtype=float) - np.asarray(radar_enu, dtype=float)
    v = np.asarray(x[3:6], dtype=float)
    r = max(float(np.linalg.norm(d)), 1e-6)
    rho2 = max(float(d[0] ** 2 + d[1] ** 2), 1e-6)
    return np.array([float(d @ v) / r,
                     float(d[1] * v[0] - d[0] * v[1]) / rho2])


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
        self.late = 0
        # None: one transition per interval, however long (the original
        # behaviour, kept bit-for-bit for TDOA-only tracks). A number:
        # propagate in steps no longer than this. Q is piecewise-constant
        # acceleration per step, so one 20 s step assumes 6 m/s^2 held for
        # 20 s -- 1.3 km of sigma -- where twenty 1 s steps give ~330 m.
        # Matters only for a track that coasts, i.e. one the radar loses in
        # its blind cone; fusion turns it on for radar-supported tracks.
        self.max_step_s = None
        self.last_innov = None          # last radar innovation (range, az)
        self.history = collections.deque(maxlen=400)
        self.history.append((t, fix.enu.copy()))

    @classmethod
    def from_state(cls, tid, x, P, t):
        """A track seeded from something other than a fix -- a radar plot
        converted to ENU, whose altitude is a prior rather than a
        measurement."""
        class _Seed:
            pass
        seed = _Seed()
        seed.enu = np.asarray(x[:3], dtype=float).copy()
        seed.cov = np.asarray(P, dtype=float)[:3, :3].copy()
        trk = cls(tid, seed, t)
        trk.x = np.asarray(x, dtype=float).copy()
        trk.P = np.asarray(P, dtype=float).copy()
        return trk

    @staticmethod
    def _transition(dt):
        f = np.eye(6)
        f[0, 3] = f[1, 4] = f[2, 5] = dt
        # Anisotropic process noise. A quadrotor manoeuvres hard laterally but
        # climbs slowly, and the vertical is the axis the geometry constrains
        # worst -- isotropic Q lets a weakly observed altitude random-walk
        # until the filtered track is worse than the raw fixes feeding it.
        acc_h, acc_v = 6.0 ** 2, 1.5 ** 2
        g2 = np.array([dt ** 2 / 2] * 3 + [dt] * 3) ** 2
        q = np.diag(g2 * np.array([acc_h, acc_h, acc_v, acc_h, acc_h, acc_v]))
        return f, q

    def predicted(self, t):
        """(x, P) at time t without touching the track: gating and
        association must look before they leap. A time at or before the
        track's own returns the current state, the same rule predict()
        applies to a late measurement."""
        if t <= self.t:
            return self.x.copy(), self.P.copy()
        return self._propagate(self.x, self.P, t - self.t)

    def state_at(self, t):
        """(x, P) as seen from a measurement at time t: predicted forward,
        or for a late one carried back along the velocity (no process
        noise removed) -- the state update_radar compares a late plot with."""
        if t >= self.t:
            return self.predicted(t)
        A = np.eye(6)
        A[0, 3] = A[1, 4] = A[2, 5] = -(self.t - t)
        return A @ self.x, A @ self.P @ A.T

    def _propagate(self, x, P, dt):
        n = 1
        if self.max_step_s and dt > self.max_step_s:
            n = int(np.ceil(dt / self.max_step_s))
        f, q = self._transition(dt / n)
        for _ in range(n):
            x = f @ x
            P = f @ P @ f.T + q
        return x, P

    def predict(self, t):
        # t is the measurement's capture time. Bursts are solved on several
        # workers, so one can finish after a later burst has already updated
        # the track. Never run the clock backwards: clamping dt to a positive
        # floor used to fuse the late measurement as if it were current AND
        # rewind self.t, so the next update integrated over the wrong
        # interval. A late one is applied at the track's own time instead --
        # it is at most one solve old -- and counted.
        if t <= self.t:
            self.late += 1
            return
        if self.max_step_s:
            self.x, self.P = self._propagate(self.x, self.P, t - self.t)
        else:
            f, q = self._transition(t - self.t)
            self.x = f @ self.x
            self.P = f @ self.P @ f.T + q
        self.t = t

    def update(self, fix, t, alt_prior=None, alt_sigma=60.0, retrodict=False):
        """retrodict=True: a late fix (t before the track's time) is compared
        with the track carried back to t along its velocity, H = [I, -dt I],
        instead of with the current position -- at 31 m/s a fix 0.2 s late
        is otherwise 6 m of fictitious innovation, enough to push the next
        good fix out of a radar-supported track's gate. Off by default: the
        TDOA-only track layer keeps its pre-fusion arithmetic exactly."""
        late = retrodict and t < self.t
        dt_back = self.t - t
        self.predict(t)
        h = np.zeros((3, 6))
        h[:3, :3] = np.eye(3)
        if late:
            h[0, 3] = h[1, 4] = h[2, 5] = -dt_back
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

    def update_radar(self, rng_m, az_rad, R, radar_enu, psi, t, iterations=3):
        """EKF update with one radar plot: slant range and antenna azimuth.

        Modelled on update_tdoa: linearised at the predicted state, the
        azimuth innovation wrapped to (-pi, pi], and gated on its own NIS
        against the chi-square(2) 99% value so a plot that does not belong
        to this track cannot drag it. psi is the radar's mounting offset in
        radians (true bearing = antenna azimuth + psi). R is the 2x2
        measurement covariance in (m, rad). Returns (applied, nis).

        Iterated (Gauss-Newton on the posterior, `iterations` relinearisations
        about the updated state). A single linearisation is fine at 1-2 km;
        a few hundred metres out, with the altitude known only to the prior,
        range and bearing curve too hard across the prior's spread and one
        plot threw a radar-only track 700 m and 400 m up (measured in
        validate_fusion's blind-cone case). The gate uses the first, plain
        EKF innovation, so iterating changes how a plot is applied, never
        whether.
        """
        late = t < self.t
        self.predict(t)
        R = np.asarray(R, dtype=float)
        x0, p0 = self.x.copy(), self.P
        # A late plot (measured before the track's own time -- the earlier
        # plot of a batch when a fix fell between them) is compared with the
        # track carried BACK to the plot's time, x_b = A x with A = [I, -dt
        # I; 0, I], instead of with the current position: at 15 m/s a
        # 0.1 s-old plot is otherwise 1.5 m of fictitious innovation. No
        # process noise is removed for the gap, which errs on the safe side.
        A = np.eye(6)
        if late:
            A[0, 3] = A[1, 4] = A[2, 5] = -(self.t - t)

        def meas(x):
            h, H = radar_measurement(A @ x, radar_enu, psi)
            return h, H @ A

        h, H = meas(x0)
        y = np.array([float(rng_m) - h[0], wrap_pi(float(az_rad) - h[1])])
        s = H @ p0 @ H.T + R
        try:
            sinv = np.linalg.inv(s)
        except np.linalg.LinAlgError:
            return False, float("inf")
        nis = float(y @ sinv @ y)
        self.last_innov = y
        if nis > _CHI2_99[2]:
            return False, nis
        kk = p0 @ H.T @ sinv
        xi = x0 + kk @ y
        for _ in range(max(int(iterations), 1) - 1):
            h, H = meas(xi)
            yi = np.array([float(rng_m) - h[0], wrap_pi(float(az_rad) - h[1])])
            s = H @ p0 @ H.T + R
            try:
                kk = p0 @ H.T @ np.linalg.inv(s)
            except np.linalg.LinAlgError:
                break
            xi = x0 + kk @ (yi - H @ (x0 - xi))
        self.x = xi
        # Joseph form: the azimuth row is ~1e-3 of the range row in scale,
        # and the short form loses symmetry fast enough at that ratio to
        # matter over a few hundred updates.
        ikh = np.eye(6) - kk @ H
        self.P = ikh @ p0 @ ikh.T + kk @ R @ kk.T
        self.history.append((t, self.x[:3].copy()))
        return True, nis

    def apply_alt_prior(self, alt_prior, alt_sigma):
        """The solver's altitude prior as a scalar pseudo-measurement -- the
        step update() applies after every fix, available on its own for a
        radar-only track, whose altitude nothing else measures."""
        hz = np.zeros((1, 6)); hz[0, 2] = 1.0
        sz = hz @ self.P @ hz.T + np.array([[alt_sigma ** 2]])
        kz = self.P @ hz.T @ np.linalg.inv(sz)
        self.x = self.x + (kz @ np.array([[alt_prior - self.x[2]]])).ravel()
        self.P = (np.eye(6) - kz @ hz) @ self.P

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]
