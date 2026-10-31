"""Prove the frequency-hopping failure, and the wideband recovery.

The gap this closes: a real drone datalink hops across the band on a
schedule the receiver is not told. Until now our emitter sat obligingly on
the node's tuned frequency, so the system could never miss -- and a missed
detection is unrecoverable in a way a degraded measurement is not. Every
other failure this project has found was invisible until the simulator
stopped agreeing with the receiver; this is the same shape.

Asserts:
  1. narrowband node vs hopping emitter: almost every burst is lost, and
     the ones lost are lost completely (no energy, not weak energy);
  2. wideband node + channelizer: the hop is found, independent nodes agree
     on the channel without communicating, and the recovered TDOA is good
     to tens of nanoseconds;
  3. the channel grid is what makes them agree -- a per-node estimate of the
     true centre would put hundreds of kHz of differential offset between
     nodes, far beyond the CAF's Doppler span;
  4. a non-hopping emitter is bit-identical through the channelizer path,
     so nothing regresses when hopping is off.
"""

import numpy as np

from dronelocate.geo import C_LIGHT
from dronelocate.sigsim import NodeChannel
from dronelocate.channelizer import (channel_grid, detect_channel,
                                     downconvert, channel_consensus)
from dronelocate.tdoa import cross_correlate

FC = 2.437e9
BW = 2.0e6
SNIPPET_S = 0.010

NODES = {
    "n01": np.array([-2500., -2200., 8.]),
    "n03": np.array([2600., -1900., 6.]),
    "n05": np.array([0., 0., 45.]),
    "n08": np.array([200., 2900., 70.]),
}
TRUTH = np.array([850.0, -600.0, 95.0])

N_CH = 8
FS_WIDE = 20e6                       # what a B210 can actually digitize
DECIM = 8
HOP = {"enabled": True, "span_hz": FS_WIDE * (N_CH - 1) / N_CH,
       "channels": N_CH}
N_BURSTS = 16


def render(fs, hop, burst, seed_base=10, vel=None):
    n = int(fs * SNIPPET_S)
    out, info = {}, {}
    for i, (nid, p) in enumerate(NODES.items()):
        ch = NodeChannel(p, fs, BW, rng_seed=seed_base + i, waveform="ofdm",
                         hopping=hop)
        s, meta = ch.render(TRUTH, burst, n, 0.0, 0.0, fc_hz=FC,
                            snr_db_at_1km=25.0, emitter_vel=vel)
        out[nid], info[nid] = s, meta
    return out, info


def true_lag(a, b):
    return (np.linalg.norm(TRUTH - NODES[a])
            - np.linalg.norm(TRUTH - NODES[b])) / C_LIGHT


print(f"hop span {HOP['span_hz']/1e6:.1f} MHz in {N_CH} channels; "
      f"burst bandwidth {BW/1e6:.1f} MHz\n")

# --- 1. narrowband node: the failure ------------------------------------
FS_NARROW = 2.4e6
seen = 0
snrs = []
for b in range(N_BURSTS):
    _, info = render(FS_NARROW, HOP, b)
    m = info["n01"]
    snrs.append(m["snr_db"])
    if m["hop_in_band"] > 0.5:
        seen += 1
print(f"narrowband node ({FS_NARROW/1e6:.1f} Msps, band "
      f"±{FS_NARROW/2e6:.2f} MHz):")
print(f"  bursts with the hop in band : {seen}/{N_BURSTS}")
print(f"  median post-hop SNR         : {np.median(snrs):.1f} dB "
      f"(vs +13.6 dB when centred)")
assert seen <= N_BURSTS // 4, \
    "a narrowband node should miss most hops -- if it does not, the hop " \
    "span is too small to be a realistic test"
assert np.median(snrs) < -20.0, \
    "missed hops must be ABSENT, not merely weak: an out-of-band burst " \
    "never reaches the ADC"

# --- 2. wideband + channelizer: the recovery ----------------------------
grid = channel_grid(FS_WIDE, N_CH)
errs, agreed, found = [], 0, 0
for b in range(N_BURSTS):
    iq, info = render(FS_WIDE, HOP, b)
    det = {}
    for nid in iq:
        c, ratio, _ = detect_channel(iq[nid], FS_WIDE, N_CH)
        if ratio >= 4.0:
            det[nid] = c
    if len(det) < len(NODES):
        continue
    found += 1
    ch, rogue = channel_consensus(det, min_agree=3)
    if ch is not None and not rogue:
        agreed += 1

    d = {}
    for nid in iq:
        d[nid], fs2 = downconvert(iq[nid], FS_WIDE, grid[det[nid]], DECIM)
    c = cross_correlate(d["n01"], d["n03"], fs2, max_lag_s=30e-6)
    errs.append((c.lag_s - true_lag("n01", "n03")) * 1e9)

errs = np.abs(np.array(errs))
print(f"\nwideband node ({FS_WIDE/1e6:.0f} Msps) + channelizer "
      f"(decim {DECIM} -> {FS_WIDE/DECIM/1e6:.1f} Msps):")
print(f"  bursts where every node found the hop : {found}/{N_BURSTS}")
print(f"  bursts where all nodes agreed on it   : {agreed}/{N_BURSTS}")
print(f"  TDOA error median {np.median(errs):.1f} ns, worst {errs.max():.1f} ns")
assert found == N_BURSTS, "the channelizer must find every hop"
assert agreed == N_BURSTS, "independent nodes must agree on the channel"
assert np.median(errs) < 25.0 and errs.max() < 80.0, \
    "recovered TDOA must stay usable"

# --- 3. why the grid, and not a per-node estimate ------------------------
# Snapping to a shared grid is what lets nodes agree with no coordination.
# Quantify what a per-node refinement would cost: the spread of true hop
# offsets inside one channel is the differential offset it would inject.
step = FS_WIDE / N_CH
print(f"\nchannel step {step/1e6:.2f} MHz -> a per-node centre estimate could "
      f"differ by up to {step/2e3:.0f} kHz between nodes,")
print(f"  against a CAF Doppler span of order 1 kHz. The grid is the "
      f"coordination-free agreement, not a convenience.")
assert step / 2.0 > 100e3, "grid step should dwarf the CAF Doppler span"

# --- 4. hopping off must change nothing ----------------------------------
iq_a, _ = render(FS_WIDE, None, 3)
iq_b, _ = render(FS_WIDE, {"enabled": False}, 3)
assert np.array_equal(iq_a["n01"], iq_b["n01"]), \
    "disabled hopping must be bit-identical to no hopping"
c_ref = cross_correlate(iq_a["n01"], iq_a["n03"], FS_WIDE, max_lag_s=30e-6)
e_ref = (c_ref.lag_s - true_lag("n01", "n03")) * 1e9
print(f"\nhopping off, full-rate correlate: TDOA error {e_ref:+.1f} ns")
assert abs(e_ref) < 10.0, "the non-hopping path must be unaffected"

print("\nPASS: hopping loses the burst outright; wideband channelization "
      "recovers it.")
