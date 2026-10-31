#!/usr/bin/env python3
"""Emulated HALO recording of the SIM drone, in navico.py's .navrec format.

A real recording shows a real room, so with --radar-replay the radar and the
emulated TDOA nodes see different worlds and Fuse has to stay off. This
renders the drone that scene.py flies (the site file's sim.track) as spokes
the way the HALO sends them -- 32-spoke frames, the same line headers, the
radar's own ms stamp, Doppler codes 0xF / 0xE -- over static ground clutter
and receiver flicker, for range A and range B. Played back, it goes through
radarnode's real path (clutter map, detector, console), not radarsim's plots.

Sync. The recording covers exactly one period L of everything that repeats
(trajectory, minute-wrapping ms stamp, revolution, spoke counter), with
record times 0..L in TRAJECTORY time. A first record on channel 255
(scanner 127: every reader skips it) says so. radarnode then plays from
wall-clock time mod L, and scene.py --wall-clock flies the drone at the
same phase of the same trajectory -- so the echo is where the TDOA nodes
hear the emitter, at the same instant, and Fuse means something.
run_demo.sh --radar-replay does both when it sees the record.

What the radar sees is decided as in radarsim: slant range from the
radar's ENU position, antenna-frame azimuth (true bearing minus
az_offset_deg), vertical fan good to 9 deg and nothing past 12.5, Pd per
revolution, Doppler flag past dop_thresh_mps. Constant rotation rate
(sim.rpm) so the loop closes; rpm_alt is not rendered.

Usage:
  python3 tools/make_radar_rec.py \\
      [--config config/site.json] [--out recordings/sim_orbit.navrec.gz]
  ./run_demo.sh --radar-replay recordings/sim_orbit.navrec.gz
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from fractions import Fraction

import numpy as np

sys.path.insert(0, ".")
from dronelocate import vendor  # noqa: E402
vendor.add_testradar_path()
import navico  # noqa: E402  testradar toolkit, vendored under testradar/
import scene  # noqa: E402
from dronelocate.radarsim import SIM_DEFAULTS  # noqa: E402

SPOKES, PIXELS = navico.SPOKES, navico.SPOKE_PIXELS
PER_FRAME = 32
META_CHANNEL = 255           # scanner 127, kind report: no reader decodes it
CLOCK_OFFSET_MS = 16641.9    # radar ms stamp vs trajectory time, any constant


def frac_lcm(a, b):
    a, b = Fraction(a).limit_denominator(1000), Fraction(b).limit_denominator(1000)
    num = math.lcm(a.numerator * b.denominator, b.numerator * a.denominator)
    return Fraction(num, a.denominator * b.denominator)


def trajectory_period(track):
    kind = track.get("kind", "orbit")
    p = float(track.get("period_s", 180.0 if kind == "orbit" else 240.0))
    # the orbit's climb is sin(th / 2): the full shape repeats every 2 periods
    return 2 * p if kind == "orbit" and float(track.get("climb_m", 30.0)) else p


def report_templates(path):
    """Last report packet of each kind per scanner, from a real recording."""
    out = {}
    for _t, ch, data in navico.read_recording(path):
        s, kind = navico.rec_channel(ch)
        if kind == navico.CHANNEL_REPORT and s in (0, 1) and len(data) >= 2:
            out[(s, data[0], data[1])] = bytes(data)
    return out


def reports_for(templates, scanner, range_m, rpm, dop_thresh):
    """The radar's report packets, patched to say what this recording is:
    transmitting, this range, Doppler 'both', this rotation rate."""
    pk = []
    for (s, sub, cat), d in sorted(templates.items()):
        if s != scanner:
            continue
        d = bytearray(d)
        if cat == 0xC4 and sub == 0x01:
            d[2] = 2                                            # transmit
        elif cat == 0xC4 and sub == 0x02:
            struct.pack_into("<i", d, 2, int(round(range_m * 10)))
            d[7] = 0                                            # custom
        elif cat == 0xC4 and sub == 0x08 and len(d) >= 21:
            struct.pack_into("<H", d, 10, int(round(rpm * 10)))
            d[18] = 1                                           # doppler both
            struct.pack_into("<H", d, 19, int(round(dop_thresh * 100)))
        elif cat == 0xC6:
            d = bytearray(b"\x11\xc6\x00")                      # no errors
        pk.append(bytes(d))
    return pk


def background(rng, cell_m, n_blobs):
    """Static echoes: dense ground clutter within ~250 m, thinning out, plus
    a few fixed 'buildings' further out. The clutter map learns all of it;
    it is there so the console looks like a radar and the map has work."""
    r = (np.arange(PIXELS) + 0.5) * cell_m
    p = np.clip(0.6 * np.exp(-r / 120.0), 0, 1)
    occ = rng.random((SPOKES, PIXELS)) < p
    bg = np.where(occ, rng.integers(5, 14, (SPOKES, PIXELS)), 0).astype(np.uint8)
    for _ in range(n_blobs):
        a = int(rng.integers(0, SPOKES))
        b = int(rng.uniform(300, PIXELS * cell_m * 0.95) / cell_m)
        wa, wb = int(rng.integers(8, 30)), int(rng.integers(2, max(3, int(60 / cell_m))))
        rows = np.arange(a, a + wa) % SPOKES
        # at most 13: 0xE and 0xF are the Doppler codes, not intensities
        bg[np.ix_(rows, np.arange(b, min(b + wb, PIXELS)))] = rng.integers(8, 14)
    return bg


def build_frame(seq0, angles, pix, stamps_ms, small_range):
    out = bytearray(b"\x01\x00\x00\x00\x00\x20\x00\x02")
    for i, a in enumerate(angles):
        h = bytearray(navico.LINE_HEADER_LEN)
        h[0], h[1] = navico.LINE_HEADER_LEN, 0x02
        struct.pack_into("<H", h, 2, 0x1000 | ((seq0 + i) & 0x0FFF))
        struct.pack_into("<HHHH", h, 6, 0x80, (2 * int(a)) % 4096, 0x8000, small_range)
        struct.pack_into("<H", h, 14, int(stamps_ms[i]))
        struct.pack_into("<I", h, 20, 0xA0000000)
        out += h
        px = pix[i]
        out += (px[0::2] | (px[1::2] << 4)).astype(np.uint8).tobytes()
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--radar-id", default=None, help="default: the first radars[] entry")
    ap.add_argument("--out", default="recordings/sim_drone.navrec.gz")
    ap.add_argument("--range-a", type=float, default=None,
                    help="range A scale, m (default radars[].sim.range_scale_m)")
    ap.add_argument("--range-b", type=float, default=None,
                    help="range B scale, m (default 2x range A; 0 = no range B)")
    ap.add_argument("--templates", default=os.path.join(vendor.testradar_dir(), "recordings", "ab_rotation.navrec.gz"),
                    help="real recording whose report packets are reused (patched)")
    ap.add_argument("--flicker", type=float, default=4e-5,
                    help="probability per cell per spoke of a weak spurious echo")
    ap.add_argument("--blobs", type=int, default=25, help="static far echoes")
    ap.add_argument("--echo", type=int, default=11, help="drone echo value 1..13")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=None,
                    help="override the loop length (must keep the sync: "
                         "a multiple of the trajectory period and of 60 s)")
    a = ap.parse_args()

    with open(a.config) as f:
        site = json.load(f)
    radars = site.get("radars", [])
    rd = next((r for r in radars if a.radar_id in (None, r.get("id"))), None)
    if rd is None:
        print("no radar in the site file's radars[] block", file=sys.stderr)
        return 1
    p = dict(SIM_DEFAULTS)
    p.update({k: v for k, v in rd.get("sim", {}).items() if v is not None})
    track = site.get("sim", {}).get("track", {})
    kind = track.get("kind", "orbit")
    fn = {"orbit": scene.orbit, "transit": scene.transit}[kind]
    o = np.asarray(rd["enu"], dtype=float)
    az_off = float(rd.get("az_offset_deg") or 0.0)
    if rd.get("az_offset_deg") is None:
        print("note: az_offset_deg is null -- the supernode will call this radar "
              "'not calibrated' and fuse nothing", file=sys.stderr)

    rpm = float(p["rpm"])
    t_rev = Fraction(60) / Fraction(rpm).limit_denominator(100)
    # spokes per loop must be a multiple of 4096 (counter) and of 32 (frames):
    # two revolutions do both
    L = frac_lcm(frac_lcm(trajectory_period(track), 60), 2 * t_rev)
    if a.seconds:
        L = Fraction(a.seconds).limit_denominator(1000)
    n_spokes = int(L / t_rev * SPOKES)
    rate = SPOKES / float(t_rev)
    ranges = [a.range_a or float(p["range_scale_m"])]
    rb = 2 * ranges[0] if a.range_b is None else a.range_b
    if rb:
        ranges.append(rb)

    rng = np.random.default_rng(a.seed)
    beam_half = p["beam_h_deg"] / 2.0 / 360.0 * SPOKES        # spokes
    templates = report_templates(a.templates) if os.path.exists(a.templates) else {}
    if not templates:
        print(f"note: no report templates ({a.templates}); the console will not "
              "show status/Doppler and radarnode will run Doppler 'off'",
              file=sys.stderr)

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    rec = navico.Recorder(a.out)
    meta = {"kind": "dronelocate-sim", "sync": "wall-clock",
            "sync_period_s": float(L), "track": track, "radar": rd.get("id"),
            "radar_enu": o.tolist(), "az_offset_deg": az_off, "rpm": rpm,
            "ranges_m": ranges, "config": a.config,
            "made": time.strftime("%Y-%m-%d %H:%M:%S")}
    rec(META_CHANNEL, 0.0, json.dumps(meta).encode())

    sc = []
    for s, rmax in enumerate(ranges):
        cell = rmax / PIXELS
        sc.append({"s": s, "rmax": rmax, "cell": cell,
                   "small": int(round(rmax * 4)),
                   "bg": background(rng, cell, a.blobs),
                   "reports": reports_for(templates, s, rmax, rpm, p["dop_thresh_mps"]),
                   "pd_rev": {}})
        if sc[-1]["small"] > 0xFFFF:
            print(f"range {rmax} m does not fit the spoke header", file=sys.stderr)
            return 1

    stats = {s: {"lit_revs": 0, "dop_revs": 0} for s in range(len(ranges))}
    t_report = -1.0
    t_start = time.time()
    for start in range(0, n_spokes, PER_FRAME):
        idx = np.arange(start, min(start + PER_FRAME, n_spokes))
        tau = idx / rate                                  # trajectory time
        angles = idx % SPOKES
        revs = idx // SPOKES
        stamps = np.round(tau * 1000 + CLOCK_OFFSET_MS).astype(np.int64) % 60000
        # where the drone is when each spoke goes out
        pos = np.array([fn(t, track) for t in tau]) - o
        hz = np.hypot(pos[:, 0], pos[:, 1])
        slant = np.linalg.norm(pos, axis=1)
        el = np.degrees(np.arctan2(pos[:, 2], hz))
        az_ant = (np.degrees(np.arctan2(pos[:, 0], pos[:, 1])) - az_off) % 360.0
        vel = np.array([(fn(t + 0.05, track) - fn(t - 0.05, track)) / 0.1 for t in tau])
        vr = np.sum(pos * vel, axis=1) / slant
        d_sp = (angles - az_ant / 360.0 * SPOKES + SPOKES / 2) % SPOKES - SPOKES / 2

        if tau[-1] - t_report >= 1.0:
            t_report = tau[-1]
            for c in sc:
                for pk in c["reports"]:
                    rec(2 * c["s"] + navico.CHANNEL_REPORT, float(tau[-1]), pk)

        for c in sc:
            pix = c["bg"][angles].copy()
            fl = rng.random(pix.shape) < a.flicker
            pix[fl & (pix == 0)] = rng.integers(1, 4, int((fl & (pix == 0)).sum()))
            for k in range(len(idx)):
                rv = int(revs[k])
                if rv not in c["pd_rev"]:
                    # one draw per revolution: detected or not, and where
                    # the echo lands (sim.sig_az_deg, sim.sig_r_cells)
                    pd = p["pd"] if abs(el[k]) <= p["fan_good_deg"] else \
                        p["pd"] * p["pd_edge_factor"]
                    c["pd_rev"][rv] = (
                        rng.random() < pd,
                        rng.normal(0.0, p["sig_az_deg"]) / 360.0 * SPOKES,
                        rng.normal(0.0, p["sig_r_cells"] * c["cell"]))
                det, e_sp, e_r = c["pd_rev"][rv]
                ds = (d_sp[k] - e_sp + SPOKES / 2) % SPOKES - SPOKES / 2
                if not det or abs(ds) > beam_half:
                    continue
                r = slant[k] + e_r
                if r > min(c["rmax"], p["max_det_range_m"]) or r < 2 * c["cell"] \
                        or abs(el[k]) > p["fan_max_deg"]:
                    continue
                # beam shape: weaker at the edges. Three cells centred on the
                # drone's cell: an even depth biases the centroid half a cell
                w = 1.0 - 0.5 * (ds / beam_half) ** 2
                if vr[k] <= -p["dop_thresh_mps"]:
                    val = 0x0F
                elif vr[k] >= p["dop_thresh_mps"] and p["doppler"] == "both":
                    val = 0x0E
                else:
                    val = max(1, int(round(a.echo * w)))
                b = int(r / c["cell"])
                pix[k, max(b - 1, 0):min(b + 2, PIXELS)] = val
                if rv != c.get("last_lit"):
                    stats[c["s"]]["lit_revs"] += 1
                    stats[c["s"]]["dop_revs"] += val >= 0x0E
                    c["last_lit"] = rv
            data = build_frame(int(idx[0]), angles, pix, stamps, c["small"])
            rec(2 * c["s"] + navico.CHANNEL_DATA, float(tau[-1]), data)

        if start % (PER_FRAME * 2048) == 0:
            print(f"\r{100 * start / n_spokes:5.1f} %  ({time.time() - t_start:.0f} s)",
                  end="", file=sys.stderr, flush=True)
    rec.close()

    n_revs = n_spokes // SPOKES
    print(f"\r{a.out}: {float(L):.0f} s loop, {n_revs} revolutions at {rpm:g} rpm, "
          f"{kind} trajectory, {os.path.getsize(a.out) / 1e6:.1f} MB", file=sys.stderr)
    for c in sc:
        st = stats[c["s"]]
        print(f"  range {'AB'[c['s']]} {c['rmax']:.0f} m (cell {c['cell']:.2f} m): "
              f"drone echo in {st['lit_revs']}/{n_revs} revolutions, "
              f"{st['dop_revs']} Doppler-flagged", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
