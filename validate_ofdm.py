"""Prove the OFDM chain -- emitter, CP detection, GCC-HT -- with no transport.

What real hardware will eventually calibrate, this pins synthetically:
  1. the CP detector recovers the OFDM geometry exactly (n_useful, n_cp)
     and does NOT claim OFDM on the flat noise waveform;
  2. cyclostationarity detects several dB below the energy gate -- the
     margin that turns "the drone got closer before we saw it" into range;
  3. Hannan-Thomson weighting beats plain correlation under per-node
     interference (distinct bands per node -- the realistic Wi-Fi case and
     the case per-node-PSD HT can actually see) and costs nothing clean;
  4. the demo config's waveform ("ofdm") holds the end-to-end accuracy.

Thresholds here gate the *mechanics*. Real false-alarm rates and the
drone-vs-WiFi margin still need band captures -- TODO.md items 4-5.
"""

import numpy as np

from dronelocate.geo import C_LIGHT
from dronelocate.sigsim import (NodeChannel, master_waveform,
                                OFDM_N_USEFUL, OFDM_N_CP)
from dronelocate.tdoa import (node_spectra, cross_correlate_fft, welch_psd,
                              ht_weight, caf_correlate, solve_tdoa,
                              timing_sigma)
from node import Node

FS = 2.4e6
BW = 2.0e6
FC = 2.437e9
N = int(FS * 0.010)
MAX_DOP = FC * (2.0 * 60.0 / C_LIGHT + 1.2e-7)

rng = np.random.default_rng(7)


def gated(w):
    g = np.zeros(N, dtype=np.float32)
    g[int(N * 0.30): int(N * 0.70)] = 1.0
    return w[:N] * g


def burst(snr_db, kind="ofdm", burst_id=3):
    amp = 10.0 ** (snr_db / 20.0)
    w = gated(master_waveform(burst_id, N, FS, BW, kind=kind)) * amp
    noise = (rng.standard_normal(N) + 1j * rng.standard_normal(N)) / np.sqrt(2)
    return (w + noise).astype(np.complex64)


def blob(centre_hz, width_hz, inr_db):
    """Filtered-noise interferer, independent per call: one node's local
    Wi-Fi neighbour, not a common illuminator."""
    spec = rng.standard_normal(N) + 1j * rng.standard_normal(N)
    f = np.fft.fftfreq(N, 1.0 / FS)
    spec[np.abs(f - centre_hz) > width_hz / 2.0] = 0.0
    x = np.fft.ifft(spec)
    x /= np.sqrt(np.mean(np.abs(x) ** 2)) + 1e-20
    return (x * 10.0 ** (inr_db / 20.0)).astype(np.complex64)


# --- 1. CP detection and classification ----------------------------------
r = Node._cyclo_detect(burst(12.0, "ofdm"))
print(f"ofdm @ 12 dB : detected={r['detected']} metric={r['metric']:.3f} "
      f"n_useful={r.get('n_useful')} n_cp={r.get('n_cp')}")
assert r["detected"], "CP detector must fire on OFDM"
assert r["n_useful"] == OFDM_N_USEFUL, "wrong useful length"
assert abs(r["n_cp"] - OFDM_N_CP) <= 1, "wrong CP length"

for label, x in [("noise waveform", burst(12.0, "noise")),
                 ("pure noise    ", burst(-60.0, "noise"))]:
    r = Node._cyclo_detect(x)
    print(f"{label}: detected={r['detected']} metric={r['metric']:.3f}")
    assert not r["detected"], f"false OFDM claim on {label}"

# --- 2. detection margin below the energy gate ---------------------------
e_min = c_min = None
for snr_db in np.arange(-2.0, 14.1, 0.5):
    x = burst(snr_db, "ofdm", burst_id=int(snr_db * 2) + 100)
    if e_min is None and Node._detect(x)[0] >= 8.0:
        e_min = snr_db
    if c_min is None and Node._cyclo_detect(x)["detected"]:
        c_min = snr_db
assert e_min is not None and c_min is not None, "sweep never detected"
print(f"\nenergy gate first fires at {e_min:+.1f} dB, "
      f"cyclo at {c_min:+.1f} dB -> margin {e_min - c_min:.1f} dB")
assert e_min - c_min >= 3.0, "cyclostationarity should buy >= 3 dB"

# --- 3. GCC-HT vs per-node interference ----------------------------------
P1 = np.array([-2500.0, -2200.0, 8.0])
P2 = np.array([2600.0, -1900.0, 6.0])
TRUTH = np.array([850.0, -600.0, 95.0])
true_lag = (np.linalg.norm(TRUTH - P1) - np.linalg.norm(TRUTH - P2)) / C_LIGHT

def pair_errors(interfere, n_bursts=10):
    ep, eh, qp, qh = [], [], [], []
    for b in range(n_bursts):
        iq = {}
        for k, p in ((0, P1), (1, P2)):
            ch = NodeChannel(p, FS, BW, rng_seed=500 + 10 * b + k,
                             waveform="ofdm")
            s, _ = ch.render(TRUTH, 200 + b, N, 0.0, 0.0, fc_hz=FC,
                             snr_db_at_1km=22.0)
            if interfere:
                s = s + blob(+0.55e6 if k == 0 else -0.65e6, 500e3, 20.0)
            iq[k] = s
        spectra, nfft = node_spectra(iq, FS)
        w = ht_weight(welch_psd(iq[0]), welch_psd(iq[1]))
        cp = cross_correlate_fft(spectra[0], spectra[1], nfft, FS, 30e-6)
        chw = cross_correlate_fft(spectra[0], spectra[1], nfft, FS, 30e-6,
                                  weight=w)
        ep.append((cp.lag_s - true_lag) * 1e9)
        eh.append((chw.lag_s - true_lag) * 1e9)
        qp.append(cp.quality)
        qh.append(chw.quality)
    return (np.sqrt(np.mean(np.square(ep))), np.sqrt(np.mean(np.square(eh))),
            np.median(qp), np.median(qh))

rms_p, rms_h, q_p, q_h = pair_errors(interfere=True)
print(f"\ninterfered : plain rms {rms_p:6.1f} ns (q {q_p:5.1f})   "
      f"HT rms {rms_h:6.1f} ns (q {q_h:5.1f})")
assert rms_h < 0.6 * rms_p, "HT must beat plain under per-node interference"

rms_pc, rms_hc, q_pc, q_hc = pair_errors(interfere=False)
print(f"clean      : plain rms {rms_pc:6.1f} ns (q {q_pc:5.1f})   "
      f"HT rms {rms_hc:6.1f} ns (q {q_hc:5.1f})")
assert rms_hc < 1.5 * rms_pc + 0.5, "HT must cost ~nothing when clean"

# --- 4. end-to-end with the demo waveform --------------------------------
NODES = np.array([
    [-2500., -2200.,   8.], [  100., -2800.,  12.], [ 2600., -1900.,   6.],
    [-2900.,   400.,  10.], [    0.,     0.,  45.], [ 2800.,   600.,   9.],
    [-2200.,  2500.,   7.], [  200.,  2900.,  70.], [ 2400.,  2400.,  11.],
    [ 1200.,  1100.,   5.],
])
VEL = np.array([28.0, -14.0, 1.5])
iq = []
for i, p in enumerate(NODES):
    ch = NodeChannel(p, FS, BW, rng_seed=900 + i, waveform="ofdm")
    s, _ = ch.render(TRUTH, 77, N, 0.0, 0.0, fc_hz=FC,
                     snr_db_at_1km=25.0, emitter_vel=VEL)
    iq.append(s)
ref, max_lag = 4, 6000.0 / C_LIGHT + 5e-6
tdoa, sig = [], []
for i in range(len(NODES)):
    if i == ref:
        continue
    c = caf_correlate(iq[i], iq[ref], FS, max_lag_s=max_lag,
                      max_doppler_hz=MAX_DOP)
    tdoa.append(c.lag_s)
    sig.append(timing_sigma(c, FS, BW))
others = [i for i in range(len(NODES)) if i != ref]
fix = solve_tdoa(NODES[others], NODES[ref], np.array(tdoa),
                 sigma_s=np.array(sig), alt_prior_m=100.0)
err = fix.enu - TRUTH
h = float(np.hypot(err[0], err[1]))
print(f"\nend-to-end, ofdm waveform + moving emitter + CAF: "
      f"h_err={h:.2f} m v_err={err[2]:+.2f} m")
assert h < 2.0, "demo waveform must hold end-to-end accuracy"

# --- 5. the channel's own interference model, full fleet -----------------
# sim.interference gives each node its OWN band-limited neighbour, which is
# the case weighting can delete; a common illuminator is not (see ht_weight).
INT = {"enabled": True, "inr_db": 20.0, "width_hz": 500e3}
iq_i = []
for i, p in enumerate(NODES):
    ch = NodeChannel(p, FS, BW, rng_seed=900 + i, waveform="ofdm",
                     interference=INT)
    s, _ = ch.render(TRUTH, 77, N, 0.0, 0.0, fc_hz=FC,
                     snr_db_at_1km=25.0, emitter_vel=VEL)
    iq_i.append(s)

spectra, nfft = node_spectra({i: v for i, v in enumerate(iq_i)}, FS)
psd = {i: welch_psd(v) for i, v in enumerate(iq_i)}


def fleet_solve(weighted):
    tdoa, sig = [], []
    for i in range(len(NODES)):
        if i == ref:
            continue
        w = ht_weight(psd[i], psd[ref]) if weighted else None
        c = cross_correlate_fft(spectra[i], spectra[ref], nfft, FS, max_lag,
                                weight=w)
        tdoa.append(c.lag_s)
        sig.append(timing_sigma(c, FS, BW))
    f = solve_tdoa(NODES[others], NODES[ref], np.array(tdoa),
                   sigma_s=np.array(sig), alt_prior_m=100.0)
    e = f.enu - TRUTH
    return float(np.hypot(e[0], e[1])), f.detail["chi2_red"]


h_off, chi_off = fleet_solve(False)
h_on, chi_on = fleet_solve(True)
print(f"interfered fleet, plain : h_err={h_off:6.2f} m  chi2={chi_off:7.2f}")
print(f"interfered fleet, GCC   : h_err={h_on:6.2f} m  chi2={chi_on:7.2f}")
assert h_on < h_off, "GCC must help on the fleet's own interference model"

print("\nPASS: OFDM emitter, CP detection margin, GCC-HT, end-to-end.")
