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
    doppler_hz: float = 0.0   # differential Doppler (a minus b), CAF only


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
    return _search_peak(r, n, fs, max_lag_s, center_lag_s, xp)


def _search_peak(r, n, fs, max_lag_s, center_lag_s, xp=np):
    """Windowed peak search + quality + sub-sample refinement on a computed
    circular correlation. Shared by every correlation front end so they all
    return identical numbers for identical spectra."""
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


def node_spectra(iq_by_id, fs):
    """One forward FFT per node, padded to the common correlation length.

    cross_correlate computes both sides' FFTs per call; in the all-pairs loop
    each node appears in ~n/2 pairs and those FFTs are 85% of the cost of a
    correlation. Cache them once and correlate pairs from the spectra.
    """
    n_time = max(len(v) for v in iq_by_id.values())
    nfft = int(1 << int(np.ceil(np.log2(2 * n_time))))
    return ({k: np.fft.fft(np.asarray(v, dtype=np.complex64), nfft)
             for k, v in iq_by_id.items()}, nfft)


def cross_correlate_fft(fa, fb, nfft, fs, max_lag_s=None, center_lag_s=0.0,
                        dop_shift_bins=0, weight=None):
    """cross_correlate from cached spectra (see node_spectra).

    The CAF's derotation arrives as an integer circular shift of the cached
    spectrum: fft(b * exp(2j*pi*k*m/nfft)) == roll(fft(b), k) exactly on the
    padded grid, so shifting costs a roll instead of a fresh 64k FFT. The
    sub-bin residual is at most half a bin (fs/2nfft, ~18 Hz here), which
    over a gated burst's effective integration is a fraction of a percent of
    amplitude and no lag bias to first order.

    weight is an optional real GCC weight on a coarse fftfreq grid whose
    length divides nfft (both are powers of two, so nearest-neighbour
    upsampling is a repeat). See ht_weight.
    """
    if dop_shift_bins:
        fb = np.roll(fb, dop_shift_bins)
    spec = fa * np.conj(fb)
    if weight is not None:
        # Broadcast over the reshaped view rather than materialising a
        # 64k repeat per pair -- same result, no allocation.
        spec = (spec.reshape(len(weight), -1) * weight[:, None]).reshape(-1)
    r = np.fft.ifft(spec)
    return _search_peak(r, nfft, fs, max_lag_s, center_lag_s, np)


def welch_psd(x, n_seg=32):
    """Per-node smoothed power spectral density, on a coarse pow2 fftfreq
    grid, for the GCC weight. Welch over n_seg segments plus a 5-bin moving
    average: the weight needs the spectrum's shape, not its fine structure,
    and a noisy weight injects more variance than it removes."""
    x = np.asarray(x, dtype=np.complex64)
    length = len(x) // n_seg
    nfft = int(1 << int(np.ceil(np.log2(max(length, 8)))))
    segs = x[: n_seg * length].reshape(n_seg, length)
    psd = (np.abs(np.fft.fft(segs, nfft, axis=1)) ** 2).mean(axis=0)
    k = np.ones(5) / 5.0
    return np.convolve(np.r_[psd[-2:], psd, psd[:2]], k, mode="valid")


def ht_weight(psd_a, psd_b, floor_quantile=0.10):
    """GCC weight from two single-channel PSDs: regularised Hannan-Thomson.

    Textbook HT is |gamma|^2 / (|S_ab| (1 - |gamma|^2)). The 1/(1-g2) factor
    is its ML pedigree and also its failure mode: at the SNRs where a drone
    is close enough to matter, in-band coherence saturates, the factor rails
    against whatever clip regularises it, and the weight degenerates into
    PHAT-style whitening -- measured here, that gave up quality for nothing
    (interfered pair: 4.0 ns vs plain's 7.2). Dropping the factor keeps the
    two parts that pay: coherence^2 to zero the bins only one channel
    trusts, and the SCOT division 1/sqrt(Sa*Sb) that crushes interference
    by its own power. Measured on OFDM pairs, one 500 kHz 20 dB-INR
    interferer per node in distinct bands: plain 7.2 ns RMS, this weight
    2.5 ns; clean case identical to plain (1.3 vs 1.4 ns) with slightly
    *better* peak quality, because the out-of-band bins go to zero.

    Coherence from one snapshot is biased, so gamma^2 is built from
    per-channel SNR spectra: each channel's noise floor is its lower PSD
    quantile (the guard band and inter-line valleys -- a flat noise
    waveform has no valley to measure, which is why this is a no-op
    without signal structure), SNR = psd/floor - 1, and
    g2 = (sa*sb)/((1+sa)(1+sb)) for independent noises.

    Known limit, by construction: an interferer occupying the SAME band at
    BOTH nodes keeps g2 high and survives the division only by its power.
    Separating that case needs true cross-spectral coherence over segments,
    which needs per-pair Doppler derotation first -- TODO.md.
    """
    na = max(float(np.quantile(psd_a, floor_quantile)), 1e-30)
    nb = max(float(np.quantile(psd_b, floor_quantile)), 1e-30)
    sa = np.clip(psd_a / na - 1.0, 0.0, None)
    sb = np.clip(psd_b / nb - 1.0, 0.0, None)
    g2 = (sa * sb) / ((1.0 + sa) * (1.0 + sb))
    w = g2 / np.sqrt(np.clip(psd_a * psd_b, 1e-60, None))
    pos = w > 0
    if not pos.any():
        return np.ones_like(w)          # no signal found: fall back to plain
    w = np.minimum(w, np.quantile(w[pos], 0.98))   # one bin must not rule
    return w / w.mean()


def caf_correlate(a, b, fs, max_lag_s=None, center_lag_s=0.0,
                  max_doppler_hz=0.0, use_gpu=False, n_seg=None):
    """Cross-ambiguity: correlate over lag AND differential frequency.

    The peak of a plain cross-correlation sits at the TDOA only if the two
    channels are at the same frequency. They are not: emitter motion puts a
    different Doppler on each node (fc/c * v * |u_i - u_j|, hundreds of Hz at
    quadrotor speeds and 2.4 GHz), and undisciplined LOs add their own offset.
    A differential df attenuates the correlation peak by sinc(df*T) -- past
    the first null for a 10 ms window at realistic speeds -- and biases the
    lag before it kills it, which is worse: a wrong answer with a quality
    that still clears the gate.

    Method: segment the window, correlate per segment (batched FFTs), then
    FFT across segments -- the peak's segment-to-segment phase rotation IS
    the differential frequency. Refine the frequency by a parabolic step,
    derotate b, and hand the now-coherent pair to cross_correlate so the
    final lag comes from the same sinc-reconstruction machinery as always.
    Zero measured Doppler therefore reproduces cross_correlate exactly.

    The Doppler stage runs on the CPU on purpose: the arrays are small and
    the batched FFTs are microseconds; only the final full-window correlation
    honours use_gpu.
    """
    dop = estimate_doppler(a, b, fs, max_lag_s, center_lag_s,
                           max_doppler_hz, n_seg)
    if dop == 0.0:
        return cross_correlate(a, b, fs, max_lag_s, use_gpu, center_lag_s)

    # Derotate b onto a's frequency and measure the lag coherently.
    n = len(a)
    t_rel = np.arange(n, dtype=np.float64) / fs
    b_rot = (np.asarray(b, dtype=np.complex64)
             * np.exp(2j * np.pi * dop * t_rel)).astype(np.complex64)
    out = cross_correlate(a, b_rot, fs, max_lag_s, use_gpu, center_lag_s)
    out.doppler_hz = dop
    return out


def estimate_doppler(a, b, fs, max_lag_s=None, center_lag_s=0.0,
                     max_doppler_hz=0.0, n_seg=None):
    """Differential frequency between two channels, by segmented CAF.

    Segment the window, correlate per segment, FFT across segments: the
    correlation peak's segment-to-segment phase rotation is the differential
    frequency. Returns 0.0 when the requested span is under one resolvable
    bin. Coarse-fine split: this estimates to a few Hz; the caller derotates
    and re-runs the full-window correlation for the precise lag.
    """
    n = len(a)
    t_total = n / fs
    if max_doppler_hz < 1.0 / t_total:      # less than one Doppler bin: moot
        return 0.0

    a = np.asarray(a, dtype=np.complex64)
    b = np.asarray(b, dtype=np.complex64)

    if max_lag_s is None:
        max_lag = n // 4
    else:
        max_lag = max(2, int(np.ceil(max_lag_s * fs)) + 2)

    # Slide the search onto the expected offset (clock delta), same reasoning
    # as center_lag_s in cross_correlate. Integer shift is enough here; the
    # final sub-sample lag comes from the full-window correlation afterwards.
    c0 = int(np.rint(float(center_lag_s) * fs))
    b_c = np.roll(b, c0)

    # Segment length caps the unambiguous span at +-fs_seg/2 = +-fs/(2L);
    # keep 15% margin over the requested span. Segments shorter than the lag
    # window would wrap the correlation, so clamp there and accept a reduced
    # span rather than a wrong one.
    seg = n_seg or int(np.ceil(2.3 * max_doppler_hz * t_total))
    seg = int(np.clip(seg, 2, max(2, n // max(4 * max_lag, 32))))
    length = n // seg
    t_seg = length / fs

    nfft = int(1 << int(np.ceil(np.log2(length + 2 * max_lag + 1))))
    asg = a[: seg * length].reshape(seg, length)
    bsg = b_c[: seg * length].reshape(seg, length)
    r = np.fft.ifft(np.fft.fft(asg, nfft, axis=1)
                    * np.conj(np.fft.fft(bsg, nfft, axis=1)), axis=1)
    lags = np.arange(-max_lag, max_lag + 1)
    r = r[:, lags % nfft]                    # (seg, n_lags), complex

    # FFT across segments, zero-padded so the frequency grid is dense enough
    # for a parabolic refinement to land within a few Hz.
    pad = int(1 << int(np.ceil(np.log2(max(seg * 4, 32)))))
    z = np.fft.fftshift(np.fft.fft(r, pad, axis=0), axes=0)
    freqs = np.fft.fftshift(np.fft.fftfreq(pad, d=t_seg))
    keep = np.abs(freqs) <= max_doppler_hz
    if not keep.any():
        keep = np.ones_like(keep)
    zk = np.abs(z[keep])
    fk = freqs[keep]

    p, _l = np.unravel_index(int(np.argmax(zk)), zk.shape)
    dop = float(fk[p])
    if 0 < p < len(fk) - 1:                  # parabolic step on the peak bin
        y0, y1, y2 = float(zk[p - 1, _l]), float(zk[p, _l]), float(zk[p + 1, _l])
        d = y0 - 2.0 * y1 + y2
        if abs(d) > 1e-20:
            dop += 0.5 * (y0 - y2) / d * (fk[1] - fk[0])
    return dop


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


def pair_covariance(pairs, var_pairs_s2, n_nodes, floor_s):
    """Full covariance of a set of TDOA measurements sharing nodes.

    A TDOA error is (node i's timing error) minus (node j's), plus
    interpolation noise, so measurements sharing a node are correlated:
    R = A diag(s) A^T + floor^2 I, with A the +1/-1 pair incidence matrix and
    s the per-node timing variances. Star-reference solving ignores the
    off-diagonal block that the shared reference creates -- every measurement
    carries the reference's error, and a diagonal weight treats those errors
    as independent, which over-counts the reference's information.

    The per-node variances are not directly observable, but with all pairs
    measured the system v_pair ~= s_i + s_j is overdetermined and a
    least-squares split recovers them from the measured pair sigmas
    themselves -- no new model, just accounting. With too few pairs to split
    (a star has n-1 equations for n unknowns) fall back to an even split,
    which reduces R to the correlated-reference structure a star implies.
    """
    pairs = np.asarray(pairs, dtype=int)
    v = np.clip(np.asarray(var_pairs_s2, dtype=float) - floor_s ** 2, 0.0, None)
    m = len(pairs)

    a = np.zeros((m, n_nodes))
    a[np.arange(m), pairs[:, 0]] = 1.0
    a[np.arange(m), pairs[:, 1]] = -1.0

    if m >= n_nodes:
        s, *_ = np.linalg.lstsq(np.abs(a), v, rcond=None)
        s = np.clip(s, 0.0, None)
    else:
        s = np.full(n_nodes, float(np.median(v)) / 2.0 if m else floor_s ** 2)

    return a @ np.diag(s) @ a.T + (floor_s ** 2) * np.eye(m)


def _pair_residuals(x, pos_a, pos_b, tdoa):
    da = np.linalg.norm(x[None, :] - pos_a, axis=1)
    db = np.linalg.norm(x[None, :] - pos_b, axis=1)
    return (da - db) / C_LIGHT - tdoa, da, db


def solve_tdoa_gls(
    pos_a,
    pos_b,
    tdoa_s,
    R,
    alt_prior_m=None,
    alt_prior_sigma_m=60.0,
    search_halfwidth_m=6000.0,
    search_alt_range=(0.0, 250.0),
    max_iter=40,
    robust=False,
):
    """Generalised least squares over arbitrary TDOA pairs.

    Measurement k is (arrival at pos_a[k]) - (arrival at pos_b[k]); R is the
    full n x n measurement covariance from pair_covariance(). The solver is
    the same grid-seeded Levenberg/Gauss-Newton as solve_tdoa -- that one is
    kept verbatim for the star-reference path and its bit-for-bit guarantees;
    this one whitens through the Cholesky factor of R so correlated
    measurements are weighted as the ML estimator says, not as if they were
    independent. Huber weights, when enabled, apply to the *whitened*
    residuals for the same reason.
    """
    pos_a = np.atleast_2d(np.asarray(pos_a, dtype=float))
    pos_b = np.atleast_2d(np.asarray(pos_b, dtype=float))
    tdoa_s = np.asarray(tdoa_s, dtype=float)
    n = len(tdoa_s)
    if n < 3:
        return None

    R = np.asarray(R, dtype=float)
    try:
        L = np.linalg.cholesky(R + 1e-22 * np.eye(n))
    except np.linalg.LinAlgError:
        L = np.diag(np.sqrt(np.clip(np.diag(R), 1e-22, None)))
    rinv = np.linalg.inv(R + 1e-22 * np.eye(n))

    use_prior = alt_prior_m is not None

    # --- coarse grid seed, cost r^T R^-1 r -------------------------------
    centre = np.vstack([pos_a, pos_b]).mean(axis=0)
    g = 21
    ex = np.linspace(centre[0] - search_halfwidth_m, centre[0] + search_halfwidth_m, g)
    ny = np.linspace(centre[1] - search_halfwidth_m, centre[1] + search_halfwidth_m, g)
    uz = np.linspace(search_alt_range[0], search_alt_range[1], 9)
    gx, gy, gz = np.meshgrid(ex, ny, uz, indexing="ij")
    pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)

    da = np.linalg.norm(pts[:, None, :] - pos_a[None, :, :], axis=2)
    db = np.linalg.norm(pts[:, None, :] - pos_b[None, :, :], axis=2)
    res = (da - db) / C_LIGHT - tdoa_s[None, :]
    cost = np.einsum('ij,ij->i', res @ rinv, res)
    if use_prior:
        cost += ((pts[:, 2] - alt_prior_m) / alt_prior_sigma_m) ** 2
    x = pts[int(np.argmin(cost))].copy()

    def whiten(v):
        return np.linalg.solve(L, v)

    # --- Levenberg-damped Gauss-Newton on whitened residuals -------------
    lam = 1e-3
    converged = False
    dw = np.ones(n)
    for _ in range(max_iter):
        r, da_, db_ = _pair_residuals(x, pos_a, pos_b, tdoa_s)
        u = whiten(r)
        if robust:
            dw = huber_weights(u, np.ones(n))
        e_a = (x[None, :] - pos_a) / np.clip(da_[:, None], 1e-6, None)
        e_b = (x[None, :] - pos_b) / np.clip(db_[:, None], 1e-6, None)
        j = (e_a - e_b) / C_LIGHT
        jw = whiten(j)

        a_ = jw.T @ (jw * dw[:, None])
        g_ = jw.T @ (dw * u)

        if use_prior:
            wa = 1.0 / (alt_prior_sigma_m ** 2)
            ja = np.array([0.0, 0.0, 1.0])
            a_ += wa * np.outer(ja, ja)
            g_ += wa * ja * (x[2] - alt_prior_m)

        try:
            step = np.linalg.solve(a_ + lam * np.diag(np.diag(a_) + 1e-12), -g_)
        except np.linalg.LinAlgError:
            break

        x_new = x + step
        r_new, _, _ = _pair_residuals(x_new, pos_a, pos_b, tdoa_s)
        u_new = whiten(r_new)
        c_old = float(u @ (dw * u))
        c_new = float(u_new @ (dw * u_new))
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

    r, da_, db_ = _pair_residuals(x, pos_a, pos_b, tdoa_s)
    u = whiten(r)
    if robust:
        dw = huber_weights(u, np.ones(n))
    e_a = (x[None, :] - pos_a) / np.clip(da_[:, None], 1e-6, None)
    e_b = (x[None, :] - pos_b) / np.clip(db_[:, None], 1e-6, None)
    jw = whiten((e_a - e_b) / C_LIGHT)

    dof = max(n - 3, 1)
    chi2_red = float(u @ (dw * u)) / dof
    scale = max(chi2_red, 1.0)

    # Same covariance discipline as solve_tdoa: inflate the MEASUREMENT
    # information by chi-square, then add the prior, which is not a
    # measurement and must not inflate with it.
    a_ = (jw.T @ (jw * dw[:, None])) / scale
    if use_prior:
        a_ += (1.0 / alt_prior_sigma_m ** 2) * np.diag([0.0, 0.0, 1.0])
    try:
        cov = np.linalg.inv(a_)
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
                "gls": True,
                "n_downweighted": int(np.sum(dw < 0.999)),
                "min_robust_w": float(np.min(dw))},
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
    return float(max(theoretical, interpolator_floor(fs)))


def interpolator_floor(fs):
    """Timing resolution floor of the sub-sample peak estimator, seconds.

    Set by what the peak estimator can actually resolve. The old
    parabola-on-magnitude was good to ~1/16 sample, and that floor dominated
    everything -- it made timing_sigma() return the same 26 ns for any
    correlation quality above ~4, so the solver could not tell a clean peak
    from a marginal one. Sinc reconstruction measures to better than 1/500
    of a sample; 1/256 is the conservative figure kept here. Shared with
    pair_covariance(), which needs the uncorrelated-per-pair part of the
    measurement variance separated from the per-node part.
    """
    return (1.0 / fs) / 256.0


def gpu_available():
    return _HAVE_CUPY
