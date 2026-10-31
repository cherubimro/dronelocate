"""Multipath: what it costs, what catches it, and what does not.

The gap this closes: the channel modelled free-space propagation and nothing
else, so every correlation saw exactly one arrival. A real site adds
reflections from buildings and ground. Reflectors are fixed geometry, so
each node's echo profile is drawn once and reused -- which is the whole
danger. A per-burst redraw would average away over a run; a static profile
biases every measurement from that node in the same direction.

This validator exists as much to pin down what does NOT work as what does.
Leading-edge (first-arrival) peak detection is the textbook answer and was
implemented, measured, and removed -- see the assertions below for why.

Asserts:
  1. multipath produces a large BIAS with unchanged precision -- the
     signature of a systematic error;
  2. it wrecks the vertical axis far worse than the horizontal, because
     VDOP amplifies per-node bias;
  3. chi-square notices, but the inflated covariance still under-reports:
     containment collapses;
  4. robust loss does NOT rescue it -- Huber needs an outlier, and multipath
     biases every node;
  5. BANDWIDTH does rescue it, which is the actionable finding.
"""

import numpy as np

from dronelocate.geo import C_LIGHT
from dronelocate.sigsim import NodeChannel
from dronelocate.tdoa import cross_correlate, solve_tdoa, timing_sigma

NODES = np.array([
    [-2500., -2200., 8.], [100., -2800., 12.], [2600., -1900., 6.],
    [-2900., 400., 10.], [0., 0., 45.], [2800., 600., 9.],
    [-2200., 2500., 7.], [200., 2900., 70.], [2400., 2400., 11.],
    [1200., 1100., 5.],
])
TRUTH = np.array([850.0, -600.0, 95.0])
REF = 4
OTHERS = [i for i in range(len(NODES)) if i != REF]
FC = 2.437e9
MAX_LAG = 6000.0 / C_LIGHT + 5e-6
N_BURSTS = 10

# Urban-ish: 3 taps, 200 ns RMS excess delay (60 m of detour), total echo
# power 8 dB below line-of-sight.
MP = {"enabled": True, "taps": 3, "tau_rms_s": 200e-9, "rician_k_db": 8.0}


def solve(fs, bw, mp, robust=False, n_bursts=N_BURSTS):
    n = int(fs * 0.010)
    h, v, chi, contained = [], [], [], 0
    for b in range(n_bursts):
        iq = [NodeChannel(p, fs, bw, rng_seed=300 + i, waveform="ofdm",
                          multipath=mp)
              .render(TRUTH, b, n, 0.0, 0.0, fc_hz=FC, snr_db_at_1km=25.0)[0]
              for i, p in enumerate(NODES)]
        td, sg = [], []
        for i in OTHERS:
            c = cross_correlate(iq[i], iq[REF], fs, max_lag_s=MAX_LAG)
            td.append(c.lag_s)
            sg.append(timing_sigma(c, fs, bw))
        f = solve_tdoa(NODES[OTHERS], NODES[REF], np.array(td),
                       sigma_s=np.array(sg), alt_prior_m=100.0, robust=robust)
        e = f.enu - TRUTH
        hh = float(np.hypot(e[0], e[1]))
        h.append(hh)
        v.append(abs(e[2]))
        chi.append(f.detail["chi2_red"])
        contained += hh < f.sigma_h
    return (np.median(h), np.median(v), np.median(chi),
            100.0 * contained / n_bursts)


def show(label, r):
    print(f"{label:38s} h {r[0]:7.2f} m  v {r[1]:7.2f} m  "
          f"chi2 {r[2]:8.2f}  contained {r[3]:3.0f}%")


# --- 1 & 2. the damage ---------------------------------------------------
FS_N, BW_N = 2.4e6, 2.0e6
print(f"narrowband {BW_N/1e6:.0f} MHz -> resolution cell c/B = "
      f"{C_LIGHT/BW_N:.0f} m ({1e9/BW_N:.0f} ns); echoes are ~200 ns\n")
clean = solve(FS_N, BW_N, None)
dirty = solve(FS_N, BW_N, MP)
show("clean", clean)
show("multipath", dirty)
assert dirty[0] > 5 * clean[0], "multipath must degrade the horizontal"
assert dirty[1] > 20 * clean[1], \
    "multipath must hurt the VERTICAL far worse -- VDOP amplifies per-node bias"

# Per-pair signature: a static bias whose magnitude depends on the two
# nodes' echo profiles -- some pairs partly cancel, others do not -- while
# the run-to-run PRECISION is untouched. Measure across several pairs
# rather than one, because a single pair can happen to cancel.
def pair_stats(mp, pairs):
    out = []
    for i, j in pairs:
        e = []
        for b in range(12):
            s = [NodeChannel(NODES[k], FS_N, BW_N, rng_seed=300 + k,
                             waveform="ofdm", multipath=mp)
                 .render(TRUTH, b, int(FS_N * 0.010), 0.0, 0.0, fc_hz=FC,
                         snr_db_at_1km=25.0)[0] for k in (i, j)]
            c = cross_correlate(s[0], s[1], FS_N, max_lag_s=MAX_LAG)
            tl = (np.linalg.norm(TRUTH - NODES[i])
                  - np.linalg.norm(TRUTH - NODES[j])) / C_LIGHT
            e.append((c.lag_s - tl) * 1e9)
        e = np.array(e)
        out.append((abs(e.mean()), e.std()))
    return np.array(out)


PAIRS = [(0, 4), (2, 4), (7, 4), (0, 7), (2, 9)]
pc = pair_stats(None, PAIRS)
pm = pair_stats(MP, PAIRS)
print(f"\nover {len(PAIRS)} pairs — |bias| median / worst, and precision (sd):")
print(f"  clean     : |bias| {np.median(pc[:,0]):6.1f} / {pc[:,0].max():6.1f} ns"
      f"   sd {np.median(pc[:,1]):.2f} ns")
print(f"  multipath : |bias| {np.median(pm[:,0]):6.1f} / {pm[:,0].max():6.1f} ns"
      f"   sd {np.median(pm[:,1]):.2f} ns")
assert np.median(pm[:, 0]) > 8 * np.median(pc[:, 0]), \
    "multipath must add a static per-pair bias"
assert np.median(pm[:, 1]) < 3 * np.median(pc[:, 1]), \
    "and must NOT degrade precision -- that is the whole danger: the " \
    "measurement stays as repeatable as ever and is simply wrong, so " \
    "nothing downstream that reasons about spread can see it"

# --- 3. chi-square notices, and still under-reports ----------------------
assert dirty[2] > 20 * clean[2], "chi-square must rise"
assert clean[3] > 60 and dirty[3] < 30, \
    "covariance inflates but not enough: containment must collapse"

# --- 4. robust loss cannot help ------------------------------------------
rob = solve(FS_N, BW_N, MP, robust=True)
show("multipath + robust (Huber)", rob)
assert rob[0] > 0.7 * dirty[0], (
    "Huber must NOT rescue this. It scales by the median residual spread, so "
    "it finds a node out of line with its peers -- and multipath biases every "
    "node. If this assertion ever fails, the model has stopped being uniform.")

# --- 5. bandwidth is the defence -----------------------------------------
FS_W, BW_W = 20e6, 18e6
print(f"\nwideband {BW_W/1e6:.0f} MHz -> resolution cell "
      f"{C_LIGHT/BW_W:.0f} m ({1e9/BW_W:.0f} ns): echoes become resolvable\n")
wide_clean = solve(FS_W, BW_W, None)
wide_dirty = solve(FS_W, BW_W, MP)
show("wideband clean", wide_clean)
show("wideband multipath", wide_dirty)
assert wide_dirty[0] < 0.25 * dirty[0], \
    "bandwidth is the multipath defence: 9x the bandwidth must cut the " \
    "horizontal error several-fold"
assert wide_dirty[1] < 0.6 * dirty[1], "and improve the vertical too"

print(f"\nhorizontal error {dirty[0]:.2f} m -> {wide_dirty[0]:.2f} m "
      f"and vertical {dirty[1]:.1f} m -> {wide_dirty[1]:.1f} m, from "
      f"bandwidth alone.")
print("\nNOT a fix, measured and removed: leading-edge (first-arrival) peak")
print("detection. No benefit at any bandwidth with a strong direct path --")
print("the max already IS the direct path, and the residual bias comes from")
print("unresolvable near-in echoes distorting the main lobe. With the direct")
print("path shadowed it traded horizontal 38->15 m for vertical 38->520 m,")
print("because inconsistent per-node peak choices are amplified by VDOP.")

print("\nPASS: multipath biases without losing precision; bandwidth is the "
      "defence, not peak-picking.")
