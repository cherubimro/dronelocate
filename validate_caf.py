"""Prove the CAF against a moving emitter, with no transport in the way.

The failure this guards: with the carrier modelled, a drone at quadrotor
speed puts a different Doppler on every node, and plain cross-correlation of
a 10 ms window first BIASES the lag (a wrong answer whose quality still
clears the gate) and then nulls the peak outright. A baseband-only test
cannot see any of this -- validate_math.py keeps passing while the field
deployment fails -- which is the bug-10 lesson wearing a new hat.

Asserts, so CI fails loudly rather than printing a sad table:
  1. static emitter: caf_correlate == cross_correlate to sub-ns (regression
     guard -- zero measured Doppler must reproduce the old path exactly);
  2. moving emitter: CAF recovers every pair's lag to a few ns and the
     measured Doppler to a few Hz of truth;
  3. moving emitter, full 10-node solve: CAF position error stays near the
     static-case error while the plain correlator's is measurably worse.
"""

import numpy as np

from dronelocate.geo import C_LIGHT
from dronelocate.sigsim import NodeChannel
from dronelocate.tdoa import (caf_correlate, cross_correlate, solve_tdoa,
                              timing_sigma, node_spectra, cross_correlate_fft,
                              estimate_doppler)

FS = 2.4e6
BW = 2.0e6
FC = 2.437e9
N = int(FS * 0.010)

NODES = np.array([
    [-2500., -2200.,   8.], [  100., -2800.,  12.], [ 2600., -1900.,   6.],
    [-2900.,   400.,  10.], [    0.,     0.,  45.], [ 2800.,   600.,   9.],
    [-2200.,  2500.,   7.], [  200.,  2900.,  70.], [ 2400.,  2400.,  11.],
    [ 1200.,  1100.,   5.],
])
TRUTH = np.array([850.0, -600.0, 95.0])
VEL = np.array([28.0, -14.0, 1.5])       # ~31 m/s, the demo orbit's speed
MAX_DOP = FC * (2.0 * 60.0 / C_LIGHT + 1.2e-7)


def render_all(vel):
    iq, dop = [], []
    for i, p in enumerate(NODES):
        ch = NodeChannel(p, FS, BW, rng_seed=100 + i)
        s, info = ch.render(TRUTH, burst_id=7, n_samples=N, t_epoch_s=0.0,
                            capture_start_s=0.0, fc_hz=FC,
                            snr_db_at_1km=25.0, emitter_vel=vel)
        iq.append(s)
        dop.append(info["doppler_hz"])
    return iq, np.array(dop)


def solve(iq, use_caf):
    ref = 4
    max_lag = 6000.0 / C_LIGHT + 5e-6
    tdoa, sig = [], []
    for i in range(len(NODES)):
        if i == ref:
            continue
        if use_caf:
            c = caf_correlate(iq[i], iq[ref], FS, max_lag_s=max_lag,
                              max_doppler_hz=MAX_DOP)
        else:
            c = cross_correlate(iq[i], iq[ref], FS, max_lag_s=max_lag)
        tdoa.append(c.lag_s)
        sig.append(timing_sigma(c, FS, BW))
    others = [i for i in range(len(NODES)) if i != ref]
    fix = solve_tdoa(NODES[others], NODES[ref], np.array(tdoa),
                     sigma_s=np.array(sig), alt_prior_m=100.0)
    err = fix.enu - TRUTH
    return float(np.hypot(err[0], err[1])), float(err[2])


print(f"snippet {N/FS*1e3:.0f} ms @ {FS/1e6:.1f} Msps, fc {FC/1e9:.3f} GHz, "
      f"|v| = {np.linalg.norm(VEL):.1f} m/s")

# --- 1. static: CAF must reproduce the plain path exactly ----------------
iq_s, _ = render_all(None)
worst = 0.0
for i in (0, 3, 7):
    c0 = cross_correlate(iq_s[i], iq_s[4], FS, max_lag_s=30e-6)
    c1 = caf_correlate(iq_s[i], iq_s[4], FS, max_lag_s=30e-6,
                       max_doppler_hz=MAX_DOP)
    worst = max(worst, abs(c1.lag_s - c0.lag_s) * 1e9)
print(f"\nstatic, CAF vs plain: worst lag disagreement {worst:.3f} ns")
assert worst < 1.0, "CAF on a static emitter must match cross_correlate"

# --- 2. moving: per-pair lag and Doppler recovery ------------------------
iq_m, dop_true = render_all(VEL)
ref = 4
lag_err_plain, lag_err_caf, dop_err, q_plain, q_caf = [], [], [], [], []
for i in range(len(NODES)):
    if i == ref:
        continue
    truth_lag = (np.linalg.norm(TRUTH - NODES[i])
                 - np.linalg.norm(TRUTH - NODES[ref])) / C_LIGHT
    c0 = cross_correlate(iq_m[i], iq_m[ref], FS, max_lag_s=30e-6)
    c1 = caf_correlate(iq_m[i], iq_m[ref], FS, max_lag_s=30e-6,
                       max_doppler_hz=MAX_DOP)
    lag_err_plain.append((c0.lag_s - truth_lag) * 1e9)
    lag_err_caf.append((c1.lag_s - truth_lag) * 1e9)
    dop_err.append(c1.doppler_hz - (dop_true[i] - dop_true[ref]))
    q_plain.append(c0.quality)
    q_caf.append(c1.quality)

print(f"moving, plain: lag err median {np.median(np.abs(lag_err_plain)):6.1f} ns"
      f"  worst {np.max(np.abs(lag_err_plain)):7.1f} ns"
      f"  quality median {np.median(q_plain):5.1f}")
print(f"moving, CAF  : lag err median {np.median(np.abs(lag_err_caf)):6.1f} ns"
      f"  worst {np.max(np.abs(lag_err_caf)):7.1f} ns"
      f"  quality median {np.median(q_caf):5.1f}"
      f"  doppler err worst {np.max(np.abs(dop_err)):4.1f} Hz")
assert np.max(np.abs(lag_err_caf)) < 6.0, "CAF must recover every pair's lag"
assert np.max(np.abs(dop_err)) < 8.0, "CAF must recover differential Doppler"
assert np.median(q_caf) > 1.5 * np.median(q_plain), \
    "CAF must restore the correlation quality Doppler destroyed"

# The supernode's cached-spectra fast path (one FFT per node, Doppler as an
# integer-bin spectrum shift) must agree with the reference caf_correlate.
spectra, nfft = node_spectra({0: iq_m[0], 4: iq_m[ref]}, FS)
dop = estimate_doppler(iq_m[0], iq_m[ref], FS, 30e-6, 0.0, MAX_DOP)
cf = cross_correlate_fft(spectra[0], spectra[4], nfft, FS, 30e-6, 0.0,
                         int(np.rint(dop * nfft / FS)))
cs = caf_correlate(iq_m[0], iq_m[ref], FS, max_lag_s=30e-6,
                   max_doppler_hz=MAX_DOP)
d_ns = abs(cf.lag_s - cs.lag_s) * 1e9
print(f"fast path vs reference: lag disagreement {d_ns:.3f} ns")
assert d_ns < 1.0, "cached-spectra fast path must match caf_correlate"

# --- 3. full solve -------------------------------------------------------
h_static, v_static = solve(iq_s, use_caf=False)
h_plain, v_plain = solve(iq_m, use_caf=False)
h_caf, v_caf = solve(iq_m, use_caf=True)
print(f"\nsolve, static reference : h_err={h_static:6.2f} m  v_err={v_static:+7.2f} m")
print(f"solve, moving + plain   : h_err={h_plain:6.2f} m  v_err={v_plain:+7.2f} m")
print(f"solve, moving + CAF     : h_err={h_caf:6.2f} m  v_err={v_caf:+7.2f} m")
assert h_caf < 2.0, "CAF solve must stay near the static-case accuracy"
assert h_plain > 2.0 * h_caf, (
    "plain correlation should be visibly worse on a moving emitter -- if it "
    "is not, the carrier model has stopped modelling the carrier")

print("\nPASS: carrier modelled, plain correlator degrades, CAF recovers it.")
