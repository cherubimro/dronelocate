"""Prove the localization chain against known truth, with no transport in the way."""

import numpy as np

from dronelocate.geo import C_LIGHT, gdop
from dronelocate.sigsim import NodeChannel, quantize, dequantize
from dronelocate.tdoa import cross_correlate, solve_tdoa, timing_sigma

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
