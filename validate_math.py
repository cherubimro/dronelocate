"""Prove the localization chain against known truth, with no transport in the way."""

import numpy as np

from dronelocate.geo import C_LIGHT, gdop
from dronelocate.sigsim import NodeChannel, quantize, dequantize
from dronelocate.tdoa import (cross_correlate, solve_tdoa, timing_sigma,
                              solve_tdoa_gls, pair_covariance,
                              interpolator_floor)
from dronelocate.track import Tracker

FS = 2.4e6           # RTL-SDR-class rate
BW = 2.0e6
SNIPPET_S = 0.010
N = int(FS * SNIPPET_S)

# 10 nodes over ~6x6 km. Two on masts/high ground to break coplanarity.
NODES = np.array([
    [-2500., -2200.,   8.], [  100., -2800.,  12.], [ 2600., -1900.,   6.],
    [-2900.,   400.,  10.], [    0.,     0.,  45.], [ 2800.,   600.,   9.],
    [-2200.,  2500.,   7.], [  200.,  2900.,  70.], [ 2400.,  2400.,  11.],
    [ 1200.,  1100.,   5.],
])

TRUTH = np.array([850.0, -600.0, 95.0])


def run(clock_sigma_s, fmt, label, alt_prior=True, seed=1):
    rng = np.random.default_rng(seed)
    biases = rng.normal(0.0, clock_sigma_s, len(NODES)) if clock_sigma_s > 0 else np.zeros(len(NODES))

    t_emit, t_cap = 0.0, 0.0
    iq = []
    for i, p in enumerate(NODES):
        ch = NodeChannel(p, FS, BW, clock_bias_s=biases[i], rng_seed=100 + i)
        s, _ = ch.render(TRUTH, burst_id=7, n_samples=N, t_epoch_s=t_emit,
                         capture_start_s=t_cap, snr_db_at_1km=25.0)
        iq.append(dequantize(quantize(s, fmt), fmt, N))   # round-trip the wire format

    baseline = float(np.max(np.linalg.norm(NODES[:, None] - NODES[None, :], axis=2)))
    max_lag = baseline / C_LIGHT + 5e-6

    ref = 4                                  # centre node as reference
    others = [i for i in range(len(NODES)) if i != ref]
    tdoa, sig = [], []
    for i in others:
        c = cross_correlate(iq[i], iq[ref], FS, max_lag_s=max_lag)
        tdoa.append(c.lag_s)
        sig.append(timing_sigma(c, FS, BW))

    fix = solve_tdoa(NODES[others], NODES[ref], np.array(tdoa), sigma_s=np.array(sig),
                     alt_prior_m=100.0 if alt_prior else None, alt_prior_sigma_m=60.0)

    err = fix.enu - TRUTH
    h_err = float(np.hypot(err[0], err[1]))
    print(f"{label:<34} h_err={h_err:7.1f} m  v_err={err[2]:+7.1f} m  "
          f"sigma_h={fix.sigma_h:6.1f}  sigma_v={fix.sigma_v:7.1f}  "
          f"resid={fix.residual_rms_s*1e9:6.1f} ns")
    return h_err, err[2]


print(f"snippet {SNIPPET_S*1e3:.0f} ms @ {FS/1e6:.1f} Msps = {N} samples")
h, v, p = gdop(NODES, TRUTH, ref_index=4)
print(f"geometry: HDOP={h:.2f}  VDOP={v:.2f}  (VDOP >> HDOP is the coplanarity tax)\n")

print("--- perfect clocks (validates geometry + correlator) ---")
run(0.0, "cf32", "cf32, no clock error")
run(0.0, "ci8", "ci8  (HackRF native)")
run(0.0, "ci2", "ci2  (VLBI 2-bit)")

print("\n--- residual clock error after calibration ---")
for s in (10e-9, 50e-9, 200e-9):
    run(s, "ci8", f"ci8, clock sigma {s*1e9:.0f} ns")

print("\n--- altitude prior on/off (VDOP demo) ---")
run(20e-9, "ci8", "with altitude prior", alt_prior=True)
run(20e-9, "ci8", "without altitude prior", alt_prior=False)


# --- all-pairs GLS vs star reference -------------------------------------
# The star hands the reference's timing noise to every measurement and then
# weights the rows as if independent. All pairs + the full covariance is the
# ML combination of the same information. Averaged over bursts because a
# single draw can go either way; the mean must not.
print("\n--- all-pairs GLS vs star reference (clock sigma 20 ns, ci8) ---")
h_star, h_gls = [], []
for trial in range(6):
    rng = np.random.default_rng(50 + trial)
    biases = rng.normal(0.0, 20e-9, len(NODES))
    iq = []
    for i, p in enumerate(NODES):
        ch = NodeChannel(p, FS, BW, clock_bias_s=biases[i],
                         rng_seed=1000 + 17 * trial + i)
        s, _ = ch.render(TRUTH, burst_id=40 + trial, n_samples=N,
                         t_epoch_s=0.0, capture_start_s=0.0,
                         snr_db_at_1km=25.0)
        iq.append(dequantize(quantize(s, "ci8"), "ci8", N))
    max_lag = float(np.max(np.linalg.norm(
        NODES[:, None] - NODES[None, :], axis=2))) / C_LIGHT + 5e-6

    ref = 4
    others = [i for i in range(len(NODES)) if i != ref]
    tdoa, sig = [], []
    for i in others:
        c = cross_correlate(iq[i], iq[ref], FS, max_lag_s=max_lag)
        tdoa.append(c.lag_s)
        sig.append(timing_sigma(c, FS, BW))
    fx = solve_tdoa(NODES[others], NODES[ref], np.array(tdoa),
                    sigma_s=np.array(sig), alt_prior_m=100.0)
    h_star.append(float(np.hypot(*(fx.enu - TRUTH)[:2])))

    pairs, lags, var = [], [], []
    for i in range(len(NODES)):
        for j in range(i + 1, len(NODES)):
            c = cross_correlate(iq[i], iq[j], FS, max_lag_s=max_lag)
            pairs.append([i, j])
            lags.append(c.lag_s)
            var.append(timing_sigma(c, FS, BW) ** 2)
    pairs = np.array(pairs, dtype=int)
    rmat = pair_covariance(pairs, np.array(var), len(NODES),
                           interpolator_floor(FS))
    fg = solve_tdoa_gls(NODES[pairs[:, 0]], NODES[pairs[:, 1]],
                        np.array(lags), rmat, alt_prior_m=100.0)
    h_gls.append(float(np.hypot(*(fg.enu - TRUTH)[:2])))

print(f"star reference, mean h_err over {len(h_star)} bursts: "
      f"{np.mean(h_star):6.2f} m   (per burst: "
      + " ".join(f"{v:.2f}" for v in h_star) + ")")
print(f"all-pairs GLS,  mean h_err over {len(h_gls)} bursts: "
      f"{np.mean(h_gls):6.2f} m   (per burst: "
      + " ".join(f"{v:.2f}" for v in h_gls) + ")")
assert np.mean(h_gls) <= np.mean(h_star) * 1.05 + 0.05, \
    "all-pairs GLS must not be worse than the star on average"

# --- tight coupling: a starved burst still updates the track --------------
# Two surviving pairs cannot produce a fix, but they constrain an existing
# track. Start a track at the last GLS fix, coast it 3 s while the emitter
# moves 30 m east, then hand it just two TDOA rows from the new position.
print("\n--- tight coupling (2 rows, no fix possible) ---")
truth2 = TRUTH + np.array([30.0, 0.0, 0.0])
trk = Tracker(1, fg, t=0.0)
trk.predict(3.0)
coast_err = float(np.linalg.norm(trk.pos - truth2))

tp = np.array([[0, 2], [3, 5]], dtype=int)      # east-west baselines
lag2 = [(np.linalg.norm(truth2 - NODES[i]) - np.linalg.norm(truth2 - NODES[j]))
        / C_LIGHT + 2e-9 for i, j in tp]         # 2 ns of measurement noise
r2 = pair_covariance(tp, np.full(2, (5e-9) ** 2), len(NODES),
                     interpolator_floor(FS))
ok = trk.update_tdoa(NODES[tp[:, 0]], NODES[tp[:, 1]], np.array(lag2), r2, 3.0)
tight_err = float(np.linalg.norm(trk.pos - truth2))
print(f"coast error {coast_err:5.1f} m -> after 2-row tight update "
      f"{tight_err:5.1f} m (applied={ok})")
assert ok, "a consistent starved burst must pass the innovation gate"
assert tight_err < 0.6 * coast_err, \
    "two TDOA rows must pull the coasted track toward the emitter"

bad = trk.update_tdoa(NODES[tp[:1, 0]], NODES[tp[:1, 1]],
                      np.array([lag2[0] + 8e-6]), r2[:1, :1], 3.5)
print(f"8 us outlier row: applied={bad} (must be gated out)")
assert not bad, "the innovation gate must reject a wildly wrong row"

print("\nPASS: all-pairs GLS and tight coupling behave.")
