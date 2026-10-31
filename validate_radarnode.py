"""Synthetic end-to-end check of radarnode's detector and timing.

Builds data frames exactly as the HALO sends them (32 spokes, same headers),
at a known rotation rate and with known spoke times. The background is one
real revolution from a recording (--recording, e.g.
testradar/recordings/test.navrec.gz, the HALO 20+ on a bench in a closed
hall) or,
without one, a synthetic hall: dense static echoes near the radar thinning
out to ~60 m. On top: random single-cell flicker and a small target on a
known path. Every revolution
after the clutter map has warmed up is scored:

  - Pd: is there a plot within the gate of the target?
  - range / azimuth / time error of that plot against the truth
  - false plots per revolution (everything else)

Usage: python3 validate_radarnode.py [--recording FILE] [--revs 60] [--rpm 22]
       [--flicker 8e-5] [--value 8] [--doppler] [--r-centre 25 --r-amp 10]
"""

import argparse
import struct
import sys

import numpy as np

from dronelocate import vendor
vendor.add_testradar_path()
import navico      # noqa: E402  testradar toolkit, vendored under testradar/
import radarnode   # noqa: E402

SPOKES, PIXELS = radarnode.SPOKES, radarnode.PIXELS
T0 = 1.8e9                   # epoch-sized start, like the real thing
SMALL_RANGE = 516            # header value the hall recording carries: 129 m edge
EDGE_M = SMALL_RANGE / 4.0
CELL_M = EDGE_M / PIXELS


def synthetic_background(seed=7):
    """angle -> 1024 values shaped like the hall recording: echoes at 13-14
    in most cells near the radar, occupancy falling to a few percent by
    ~40 m and nothing past ~60 m (cell 0.126 m)."""
    rng = np.random.default_rng(seed)
    r_m = (np.arange(PIXELS) + 0.5) * CELL_M
    p_occ = np.clip(np.exp(-r_m / 9.0), 0.0, 1.0) * (r_m < 60)
    static = rng.random((SPOKES, PIXELS)) < p_occ
    vals = rng.choice([13, 14], size=(SPOKES, PIXELS)).astype(np.uint8)
    return {a: np.where(static[a], vals[a], 0).astype(np.uint8) for a in range(SPOKES)}


def background(path):
    """angle -> 1024 pixel values, from the first full revolution."""
    if not path:
        return synthetic_background()
    bg = {}
    for kind, t, d in radarnode.replay_source(path, 0):
        if kind != navico.CHANNEL_DATA:
            continue
        f = radarnode.decode_frame(d)
        if f is None:
            continue
        for a, px in zip(f["angle"], f["pix"]):
            bg.setdefault(int(a), px.copy())
        if len(bg) == SPOKES:
            break
    return bg


R_CENTRE, R_AMP = 75.0, 30.0


def truth_at(t):
    """Target path: in and out between R_CENTRE -/+ R_AMP (default ~45 to
    ~105 m, inside the 129 m edge for any run length, peak radial speed
    ~1.6 m/s), drifting 0.6 deg/s in azimuth."""
    return (R_CENTRE + R_AMP * np.sin(2 * np.pi * t / 120.0),
            (30.0 + 0.6 * t) % 360.0)


CLOCK_OFFSET_MS = 16641.9    # radar ms-stamp vs host clock, as measured


def build_frame(seq0, angles, pix, t_true):
    out = bytearray(b"\x01\x00\x00\x00\x00\x20\x00\x02")
    for i, (a, px) in enumerate(zip(angles, pix)):
        h = bytearray(24)
        h[0], h[1] = 24, 0x02
        struct.pack_into("<H", h, 2, 0x1000 | ((seq0 + i) & 0x0FFF))
        struct.pack_into("<HHHH", h, 6, 0x80, (2 * a) % 4096, 0x8000, SMALL_RANGE)
        stamp = int(round((T0 + t_true[i]) * 1000 + CLOCK_OFFSET_MS)) % 60000
        struct.pack_into("<H", h, 14, stamp)
        out += h
        out += (px[0::2] | (px[1::2] << 4)).astype(np.uint8).tobytes()
    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", help="recording for the background (default: synthetic)")
    ap.add_argument("--revs", type=int, default=60)
    ap.add_argument("--rpm", type=float, default=22.0)
    ap.add_argument("--flicker", type=float, default=8e-5,
                    help="probability per cell per revolution of a spurious echo")
    ap.add_argument("--value", type=int, default=8, help="target echo value 1..13")
    ap.add_argument("--doppler", action="store_true", help="flag the target 0xF")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--r-centre", type=float, default=75.0)
    ap.add_argument("--r-amp", type=float, default=30.0)
    args = ap.parse_args()
    global R_CENTRE, R_AMP
    R_CENTRE, R_AMP = args.r_centre, args.r_amp

    rng = np.random.default_rng(args.seed)
    bg = background(args.recording)
    rate = args.rpm / 60.0 * SPOKES                 # spokes per second
    det = radarnode.Detector(radarnode.DEFAULT_CFG)
    timing = radarnode.TimingModel(latency_s=0.0)
    clock = radarnode.SpokeClock(latency_s=0.0)    # synthetic: no radar latency
    warm = radarnode.DEFAULT_CFG["map"]["warmup_revs"] + 1
    doppler_mode = "approaching" if args.doppler else "off"

    t0 = T0
    total = args.revs * SPOKES
    seq = 0
    plots_by_rev = {}
    hw_spokes = 10                                  # target half-width in azimuth
    for start in range(0, total, 32):
        idx = np.arange(start, min(start + 32, total))
        angles = idx % SPOKES
        t_true = idx / rate                         # seconds since start
        pix = np.stack([bg[int(a)] for a in angles]).copy()
        flick = rng.random(pix.shape) < args.flicker
        pix[flick & (pix == 0)] = rng.integers(1, 4, size=int((flick & (pix == 0)).sum()))
        for k, (a, t) in enumerate(zip(angles, t_true)):
            r, az = truth_at(t)
            d = (a - az / 360.0 * SPOKES + SPOKES / 2) % SPOKES - SPOKES / 2
            if abs(d) <= hw_spokes:
                b = int(r / CELL_M)
                pix[k, max(b - 1, 0):b + 2] = 0x0F if args.doppler else args.value
        data = build_frame(seq, angles, pix, t_true)
        seq += len(idx)
        t_rx = t0 + t_true[-1] + rng.normal(0, 0.0018)   # measured frame jitter
        f = radarnode.decode_frame(data)
        timing.times(t_rx, f["angle"])
        times = clock.times(t_rx, f["ms"])
        for batch in det.add(f, times, doppler_mode):
            plots_by_rev.setdefault(batch["rev"], []).extend(batch["plots"])

    hits, errs_r, errs_az, errs_t, false = 0, [], [], [], []
    scored = range(warm, args.revs - 1)
    for rev in scored:
        plots = plots_by_rev.get(rev, [])
        # truth when the beam crossed the target in this revolution
        best, nfalse = None, 0
        for p in plots:
            t = p["t_utc_ns"] / 1e9 - t0
            r, az = truth_at(t)
            dr = p["rng_m"] - r
            daz = (p["az_deg"] - az + 180) % 360 - 180
            if abs(dr) < 2.0 and abs(daz) < 3.0 and best is None:
                best = (dr, daz, p)
            else:
                nfalse += 1
        false.append(nfalse)
        if best:
            hits += 1
            dr, daz, p = best
            errs_r.append(dr)
            errs_az.append(daz)
            # time the beam centre actually crossed the target
            r, az = truth_at(p["t_utc_ns"] / 1e9 - t0)
            a_true = az / 360.0 * SPOKES
            k = round((rev * SPOKES + a_true) - 0)
            errs_t.append(p["t_utc_ns"] / 1e9 - t0 - k / rate)

    n = len(scored)
    f = lambda v: f"{np.mean(v):+.3f} / {np.std(v):.3f}" if v else "n/a"
    print(f"background {args.recording or 'synthetic'}")
    print(f"revolutions scored {n} (after {warm} warm-up), rpm {args.rpm}, "
          f"flicker {args.flicker:g}, target value {'0xF' if args.doppler else args.value}")
    print(f"Pd per revolution          {hits}/{n} = {hits / max(n, 1):.2f}")
    print(f"range error   mean / sd    {f(errs_r)} m   (cell {CELL_M:.3f} m)")
    print(f"azimuth error mean / sd    {f(errs_az)} deg (spoke {360 / SPOKES:.3f} deg)")
    print(f"time error    mean / sd    {f([e * 1000 for e in errs_t])} ms")
    print(f"false plots per revolution {np.mean(false):.2f} (max {max(false) if false else 0})")
    print(f"timing model rpm           {timing.rpm:.2f}")
    ok = hits / max(n, 1) >= 0.9 and np.mean(false) < 0.5 and errs_r and \
        abs(np.mean(errs_r)) < 0.5 and abs(np.mean(errs_az)) < 0.5 and \
        abs(np.mean(errs_t)) < 0.02
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
