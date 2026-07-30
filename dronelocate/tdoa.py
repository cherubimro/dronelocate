"""Time-difference-of-arrival: correlation front end, 3D solver back end.

Two stages, kept separate because they fail differently. Correlation failure
looks like a low-quality peak and is a signal problem. Solver failure looks
like a large residual or a huge covariance and is a geometry problem. The
console shows both so you can tell them apart.
"""

from dataclasses import dataclass, field

import numpy as np

from .geo import C_LIGHT

try:  # optional GPU path; falls back silently
    import cupy as _cp

    _HAVE_CUPY = True
except Exception:  # pragma: no cover
    _cp = None
    _HAVE_CUPY = False


@dataclass
class CorrResult:
    lag_s: float          # positive => signal arrived at b later than at a
    quality: float        # peak / rms of correlation surface, dimensionless
    peak_mag: float


def _xp(use_gpu):
    return _cp if (use_gpu and _HAVE_CUPY) else np


_SINC_KERNEL = {}


def _sinc_kernel(half, up):
    key = (half, up)
    ker = _SINC_KERNEL.get(key)
    if ker is None:
        offs = np.arange(-half, half + 1)
        taus = np.linspace(-0.5, 0.5, up + 1)
        # Blackman taper on the truncated sinc: without it the abrupt cutoff
        # rings and puts its own ripple on the reconstructed peak.
        ker = (taus, np.sinc(taus[:, None] - offs[None, :])
               * np.blackman(2 * half + 1)[None, :])
        _SINC_KERNEL[key] = ker
    return ker


def _subsample_peak(r, k, half=24, up=64):
    """Locate the peak between samples by band-limited reconstruction.

    The correlation is band-limited -- its spectrum is the cross-spectrum,
    zero outside the signal band -- so the value between samples is a sinc
    sum over neighbours, not a parabola. That distinction is not academic
    here: bandwidth/fs is about 0.83, so the peak is oversampled barely 1.2x
    and its main lobe spans roughly one sample. A three-point curve fit has
    almost nothing to grip on.

    Measured against known fractional delays, the parabola-on-magnitude this
    replaces carries a systematic S-curve bias reaching 0.12 samples (~50 ns,
    ~15 m of range) that reverses sign across the sample. Being systematic it
    does not average away, and the chi-square rescaling cannot absorb it.
    This estimator holds under 0.003 samples.
    """
    n = len(r)
    taus, m = _sinc_kernel(half, up)
    idx = (np.arange(-half, half + 1) + k) % n
    seg = r[idx]
    if hasattr(seg, "get"):        # cupy -> host; only ~50 samples
        seg = seg.get()
    v = np.abs(m @ np.asarray(seg))
    j = int(np.argmax(v))
    # one parabolic step on the fine grid, so precision is not capped by the
    # grid spacing itself
    if 0 < j < len(v) - 1:
        y0, y1, y2 = float(v[j - 1]), float(v[j]), float(v[j + 1])
        d = y0 - 2.0 * y1 + y2
        if abs(d) > 1e-20:
            step = 0.5 * (y0 - y2) / d * (taus[1] - taus[0])
            return float(np.clip(taus[j] + step, -0.5, 0.5))
    return float(taus[j])


def cross_correlate(a, b, fs, max_lag_s=None, use_gpu=False, center_lag_s=0.0):
    """Cross-correlate two complex baseband snippets.

    Restricting max_lag_s to the geometric bound (baseline / c) is what keeps
    this cheap: we still compute the full FFT but we only search the
    physically possible lags, which suppresses the sidelobe peaks that would
    otherwise win at low SNR.

    center_lag_s moves that window instead of widening it. The measured lag
    is geometry *plus* the two nodes' clock offset difference, and only the
    first of those is bounded by the baseline. Clock drift accumulates from
    session start, so after a few minutes the offset difference exceeds the
    geometric bound and the true peak sits outside a window centred on zero
    -- the correlator then locks onto whatever noise peak is inside it and
    returns a confident wrong lag. Widening the window to cover the offset
    would work but throws away the sidelobe rejection; centring on the known
    offset keeps the window tight and the search honest.
    """
    xp = _xp(use_gpu)
    a = xp.asarray(a, dtype=xp.complex64)
    b = xp.asarray(b, dtype=xp.complex64)

    n = int(1 << int(np.ceil(np.log2(len(a) + len(b)))))
    fa = xp.fft.fft(a, n)
    fb = xp.fft.fft(b, n)
    r = xp.fft.ifft(fa * xp.conj(fb))
    mag = xp.abs(r)

    if max_lag_s is None:
        max_lag = n // 2
    else:
        max_lag = max(2, int(np.ceil(max_lag_s * fs)) + 2)
        max_lag = min(max_lag, n // 2 - 2)

    # The correlation is circular with period n, so lag L lives at index
    # L mod n. Gather [c0 - max_lag, c0 + max_lag] directly; with c0 = 0 this
    # is the old head/tail split, and with c0 != 0 it slides the search onto
    # where the peak is actually expected.
    c0 = int(np.rint(float(center_lag_s) * fs))
    idx = (xp.arange(-max_lag, max_lag + 1) + c0) % n
    window = mag[idx]
    lag_offset = c0 - max_lag

    k = int(xp.argmax(window))
    peak = float(window[k])

    # noise floor from the searched window, excluding the peak neighbourhood
    mask = xp.ones(window.shape[0], dtype=bool)
    lo, hi = max(0, k - 3), min(window.shape[0], k + 4)
    mask[lo:hi] = False
    floor = float(xp.sqrt(xp.mean(window[mask] ** 2))) if bool(mask.any()) else 1e-12
    quality = peak / max(floor, 1e-12)

    # Sub-sample peak, by band-limited reconstruction of the complex
    # correlation rather than a parabola through three magnitudes.
    frac = _subsample_peak(r, int((k + lag_offset) % n))

    lag_samples = (k + lag_offset) + frac
    return CorrResult(lag_s=lag_samples / fs, quality=float(quality), peak_mag=peak)


@dataclass
class Fix:
    enu: np.ndarray
    cov: np.ndarray                    # 3x3 position covariance, m^2
    residual_rms_s: float
    n_meas: int
    converged: bool
    hdop: float = 0.0
    vdop: float = 0.0
    used_altitude_prior: bool = False
    detail: dict = field(default_factory=dict)

    @property
    def sigma_h(self):
        return float(np.sqrt(max(self.cov[0, 0] + self.cov[1, 1], 0.0)))

    @property
    def sigma_v(self):
        return float(np.sqrt(max(self.cov[2, 2], 0.0)))


def huber_weights(r, w, k=1.345):
    """IRLS weights: bound an outlier's influence instead of trusting it.

    Scaled by a robust estimate of the residual spread rather than by the
    assumed sigmas, and that detail is load-bearing. timing_sigma() is
    optimistic enough that reduced chi-square sits around 15 in practice, so
    normalised residuals are typically ~4 across the board. A fixed threshold
    against the assumed sigma would then down-weight every measurement by
    roughly the same factor -- which is a rescale, not outlier rejection, and
    leaves the solution exactly where it was. Dividing by the median spread
    makes the test relative to the other measurements in this fix, which is
    the question actually being asked: is this one out of line with its peers?

    k = 1.345 is the standard Huber constant: 95% efficiency against clean
    Gaussian noise, so switching this on costs almost nothing when there are
    no outliers to find.
    """
    u = np.abs(r) * np.sqrt(w)
    s = 1.4826 * float(np.median(u))          # MAD -> sigma, zero-centred
    if not np.isfinite(s) or s <= 0.0:
        return np.ones_like(u)
    t = u / s
    out = np.ones_like(t)
    hit = t > k
    out[hit] = k / t[hit]
    return out


def _residuals(x, nodes, ref, tdoa):
    d = np.linalg.norm(x[None, :] - nodes, axis=1)
    d0 = np.linalg.norm(x - ref)
    return (d - d0) / C_LIGHT - tdoa, d, d0


def solve_tdoa(
    node_enu,
    ref_enu,
    tdoa_s,
    sigma_s=None,
    alt_prior_m=None,
    alt_prior_sigma_m=60.0,
    search_halfwidth_m=6000.0,
    search_alt_range=(0.0, 250.0),
    max_iter=40,
    robust=False,
):
    """Solve for 3D emitter position from TDOAs against a reference node.

    node_enu     (N,3) non-reference node positions, metres ENU
    ref_enu      (3,)  reference node position
    tdoa_s       (N,)  measured (t_i - t_ref) in seconds
    sigma_s      (N,)  per-measurement timing std dev, seconds
    alt_prior_m  soft constraint on altitude. With all sensors near ground
                 level the vertical is weakly observable; the prior stops a
                 badly determined z from corrupting x and y.

    Seeded by a coarse grid search because the TDOA cost surface is
    multimodal -- a Gauss-Newton started at the centroid will happily lock
    onto a ghost intersection and report a confident wrong answer.
    """
    node_enu = np.atleast_2d(np.asarray(node_enu, dtype=float))
    ref_enu = np.asarray(ref_enu, dtype=float)
    tdoa_s = np.asarray(tdoa_s, dtype=float)
    n = len(tdoa_s)
    if n < 3:
        return None

    if sigma_s is None:
        sigma_s = np.full(n, 30e-9)
    sigma_s = np.asarray(sigma_s, dtype=float)
    w = 1.0 / np.clip(sigma_s, 1e-12, None) ** 2

    use_prior = alt_prior_m is not None

    # --- coarse grid seed -------------------------------------------------
    centre = np.vstack([node_enu, ref_enu]).mean(axis=0)
    g = 21
    ex = np.linspace(centre[0] - search_halfwidth_m, centre[0] + search_halfwidth_m, g)
    ny = np.linspace(centre[1] - search_halfwidth_m, centre[1] + search_halfwidth_m, g)
    uz = np.linspace(search_alt_range[0], search_alt_range[1], 9)
    gx, gy, gz = np.meshgrid(ex, ny, uz, indexing="ij")
    pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)

    dref = np.linalg.norm(pts - ref_enu[None, :], axis=1)
    cost = np.zeros(len(pts))
    for i in range(n):
        di = np.linalg.norm(pts - node_enu[i][None, :], axis=1)
        r = (di - dref) / C_LIGHT - tdoa_s[i]
        cost += w[i] * r * r
    if use_prior:
        cost += ((pts[:, 2] - alt_prior_m) / alt_prior_sigma_m) ** 2
    x = pts[int(np.argmin(cost))].copy()

    # --- Gauss-Newton with Levenberg damping ------------------------------
    lam = 1e-3
    converged = False
    rw = np.ones(n)          # robust multipliers, all 1 unless robust=True
    we = w                   # effective weights used this iteration
    for _ in range(max_iter):
        r, d, d0 = _residuals(x, node_enu, ref_enu, tdoa_s)
        if robust:
            # Recomputed from the current residuals each iteration -- that is
            # what makes this IRLS rather than a single reweighting. Held
            # fixed within the iteration so the descent test below compares
            # like with like.
            rw = huber_weights(r, w)
            we = w * rw
        e_i = (x[None, :] - node_enu) / np.clip(d[:, None], 1e-6, None)
        e_0 = (x - ref_enu) / max(d0, 1e-6)
        j = (e_i - e_0[None, :]) / C_LIGHT

        jw = j * we[:, None]
        a = j.T @ jw
        g_ = j.T @ (we * r)

        if use_prior:
            wa = 1.0 / (alt_prior_sigma_m ** 2)
            ja = np.array([0.0, 0.0, 1.0])
            ra = x[2] - alt_prior_m
            a += wa * np.outer(ja, ja)
            g_ += wa * ja * ra

        try:
            step = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), -g_)
        except np.linalg.LinAlgError:
            break

        x_new = x + step
        r_new, _, _ = _residuals(x_new, node_enu, ref_enu, tdoa_s)
        c_old = float(np.sum(we * r * r))
        c_new = float(np.sum(we * r_new * r_new))
        if use_prior:
            c_old += ((x[2] - alt_prior_m) / alt_prior_sigma_m) ** 2
            c_new += ((x_new[2] - alt_prior_m) / alt_prior_sigma_m) ** 2

        if c_new < c_old:
            x = x_new
            lam = max(lam * 0.5, 1e-9)
            if np.linalg.norm(step) < 1e-3:
                converged = True
                break
        else:
            lam *= 4.0
            if lam > 1e9:
                break

    r, d, d0 = _residuals(x, node_enu, ref_enu, tdoa_s)
    if robust:
        rw = huber_weights(r, w)
        we = w * rw
    e_i = (x[None, :] - node_enu) / np.clip(d[:, None], 1e-6, None)
    e_0 = (x - ref_enu) / max(d0, 1e-6)
    j = (e_i - e_0[None, :]) / C_LIGHT
    # Scale by reduced chi-square. Without this the covariance reports how
    # confident the *assumed* sigmas were, not how well the fit actually
    # agrees with the data -- so an uncalibrated clock produces a tight,
    # confident, badly wrong ellipse. Operators trust the ellipse, so it has
    # to degrade when the solution degrades.
    dof = max(n - 3, 1)
    chi2_red = float(np.sum(we * r * r)) / dof
    scale = max(chi2_red, 1.0)

    # Apply that inflation to the MEASUREMENT information, then add the prior.
    # Scaling the finished covariance instead (cov * scale) inflates the
    # prior's contribution too, and the prior is not a measurement: its
    # uncertainty does not grow because the TDOAs disagree with each other.
    # On the vertical axis -- the one the prior dominates, because the
    # geometry barely constrains it -- that turned a 60 m prior into a 160 m
    # reported sigma, and made sigma_v *anti*-correlate with the actual error
    # (-0.40 measured): a high chi-square makes the solver lean harder on the
    # prior, which lands it near the true altitude while claiming to be less
    # sure. The ellipsoid grew tallest exactly when the fix was best.
    a = (j.T @ (j * we[:, None])) / scale
    if use_prior:
        a += (1.0 / alt_prior_sigma_m ** 2) * np.diag([0.0, 0.0, 1.0])
    try:
        cov = np.linalg.inv(a)
    except np.linalg.LinAlgError:
        cov = np.eye(3) * 1e12

    dv = np.clip(np.diag(cov), 0.0, None)
    return Fix(
        enu=x,
        cov=cov,
        residual_rms_s=float(np.sqrt(np.mean(r ** 2))),
        n_meas=n,
        converged=converged,
        hdop=float(np.sqrt(dv[0] + dv[1])),
        vdop=float(np.sqrt(dv[2])),
        used_altitude_prior=use_prior,
        detail={"chi2_red": chi2_red, "cov_scale": scale, "robust": bool(robust),
                # how many measurements the robust loss pulled back, and how
                # hard it pulled the worst one -- if this is never non-zero the
                # loss is doing nothing and the gate is the thing earning its keep
                "n_downweighted": int(np.sum(rw < 0.999)),
                "min_robust_w": float(np.min(rw))},
    )


def timing_sigma(corr: CorrResult, fs, bandwidth_hz=None):
    """Map a correlation peak to a timing standard deviation.

    Theory says sigma ~ 1 / (2*pi*B_rms*sqrt(SNR)). In practice the parabolic
    peak interpolator floors out around a sixteenth of a sample, so we take
    the worse of the two rather than believing the optimistic one.
    """
    b = bandwidth_hz or (fs * 0.8)
    snr = max(corr.quality ** 2 - 1.0, 0.1)
    theoretical = 1.0 / (2.0 * np.pi * b * np.sqrt(snr))
    # Floor set by what the peak estimator can actually resolve. The old
    # parabola-on-magnitude was good to ~1/16 sample, and that floor dominated
    # everything -- it made timing_sigma() return the same 26 ns for any
    # correlation quality above ~4, so the solver could not tell a clean peak
    # from a marginal one. Sinc reconstruction measures to better than 1/500
    # of a sample; 1/256 is the conservative figure kept here.
    interpolator_floor = (1.0 / fs) / 256.0
    return float(max(theoretical, interpolator_floor))


def gpu_available():
    return _HAVE_CUPY
