#!/usr/bin/env python3
"""Radar sensor node for dronelocate: Navico HALO spokes -> plots on Zenoh.

The radar counterpart of dronelocate's node.py. It turns raw spokes into
small detection events ("plots") and publishes them, keeping the bulk data
local -- the same pull-don't-push shape as the TDOA nodes.

    spokes (multicast) -> per-spoke timing -> clutter map -> hit cells
      -> clusters per sector -> plot batch -> dc/{site}/{radar}/evt/plot

Radar protocol knowledge -- spoke frame layout, the Doppler nibble codes,
report decoding -- comes through the testradar toolkit's navico module,
which builds on Mayara (Kees Verruijt and the Mayara contributors,
Apache-2.0) and OpenCPN radar_pi (Copyright (C) 2010-2025 David S. Register,
Dave Cowell, Kees Verruijt, Douwe Fokkema; GPL-2.0-or-later). See the
testradar README for the full attribution.

The radar protocol (navico) comes from the testradar toolkit, vendored as
a git subtree under testradar/ (dronelocate/vendor.py puts it on the path;
TESTRADAR=/path/to/checkout overrides it).

Usage:
  radarnode.py --replay testradar/recordings/test.navrec.gz --no-zenoh --print
  radarnode.py --replay ab_rotation.navrec.gz --scanner 1 --csv plots.csv --no-zenoh
  radarnode.py --site config/site-field.json --hub 10.0.0.5
  radarnode.py --radar 192.168.100.231 --heading 0 ...   live; --heading feeds
                                                         NKOE so Doppler works
  radarnode.py --source sim --site config/site.json      emulated radar from
                                                         dc/{site}/sim/truth

Design notes, all from measurements on the HALO 20+ (see the session notes):
- Spoke times are reconstructed from frame reception time and angle with a
  continuously tracked rotation rate. The rate is NOT a constant: range B
  transmitting moved the shared antenna from 22.1 to 31.5 rpm, and it came
  back at 20.9.
- The 4-bit value is ordinal, not dB, and strong echoes pile up at 13-14, so
  detection is "new relative to this cell's history", never an amplitude
  threshold. One map per range scale: cells mean different distances at
  different scales.
- Doppler mode steals 0xF (approaching) and, in "both", 0xE (receding).
  Flags are evidence, never a requirement: a hovering drone carries none.
- Empty sectors are published too. "Nothing here" is data.
"""

import argparse
import collections
import csv
import json
import math
import os
import queue
import socket
import sys
import threading
import time

import numpy as np

from dronelocate import proto, zconf, vendor

# testradar toolkit, vendored under testradar/ (see dronelocate/vendor.py).
# Needed for a live radar and for replays; --source sim never touches the
# protocol, so it must not require it -- a missing navico used to stop the
# emulated radar in run_demo.sh too.
vendor.add_testradar_path()
try:
    import navico
except ImportError:
    navico = None

if navico is not None:
    SPOKES = navico.SPOKES            # 2048 per revolution
    PIXELS = navico.SPOKE_PIXELS      # 1024 per spoke
    LINE = navico.LINE_LEN            # 24 header + 512 data bytes
    FRAME_HDR = navico.FRAME_HEADER_LEN
    VALID_STATUS = np.array(sorted(navico.VALID_SPOKE_STATUSES), dtype=np.uint8)
else:
    # The same values, so radarsim's sector geometry still lines up. Nothing
    # here decodes a frame without navico: main() refuses live and replay.
    SPOKES, PIXELS, LINE, FRAME_HDR = 2048, 1024, 536, 8
    VALID_STATUS = np.array([0x02, 0x12], dtype=np.uint8)
SECTORS = 16
SECTOR = SPOKES // SECTORS        # 128 spokes = 22.5 deg

NAVICO_MISSING = (
    "radarnode: the navico module (testradar toolkit) is not importable.\n"
    f"  expected testradar/navico.py under {vendor.ROOT} (a git subtree -- is\n"
    "  this a partial copy of the repository?), or TESTRADAR=/path/to/checkout")


# --------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------

def decode_frame(data):
    """One data frame -> dict of arrays for its valid spokes (numpy, no
    per-byte Python: this runs ~750 spokes/s per scanner)."""
    n = (len(data) - FRAME_HDR) // LINE
    if n <= 0:
        return None
    lines = np.frombuffer(data, dtype=np.uint8, count=n * LINE,
                          offset=FRAME_HDR).reshape(n, LINE)
    hdr = lines[:, :navico.LINE_HEADER_LEN]
    ok = (hdr[:, 0] == navico.LINE_HEADER_LEN) & np.isin(hdr[:, 1], VALID_STATUS)
    if not ok.any():
        return None
    hdr, raw = hdr[ok], lines[ok, navico.LINE_HEADER_LEN:]
    u16 = lambda off: hdr[:, off].astype(np.uint32) | (hdr[:, off + 1].astype(np.uint32) << 8)
    large, angle, heading, small = u16(6), u16(8), u16(10), u16(12)
    rng = np.where(large == 0x80,
                   np.where(small == 0xFFFF, 0.0, small / 4.0),
                   large * small / 512.0)
    pix = np.empty((len(raw), PIXELS), dtype=np.uint8)
    pix[:, 0::2] = raw & 0x0F
    pix[:, 1::2] = raw >> 4
    return {"angle": ((angle // 2) % SPOKES).astype(np.int32), "range_m": rng,
            # bytes 2-3: 12-bit spoke counter, high nibble 1 on range A and 0
            # on B here (probably the range id); bytes 14-15: the radar's own
            # millisecond clock, 0..59999 -- per-spoke time without guessing.
            "seq": (u16(2) & 0x0FFF).astype(np.int64), "status": hdr[:, 1].copy(),
            "ms": u16(14).astype(np.int64),
            "heading": heading, "pix": pix, "dropped": int(n - ok.sum())}


# --------------------------------------------------------------------------
# Timing
# --------------------------------------------------------------------------

class SpokeClock:
    """Per-spoke time from the radar's millisecond clock (spoke bytes 14-15).

    The stamp counts 0..59999 ms and wraps every minute. Host reception time
    minus the stamp of a frame's newest spoke is constant to ~0.5 ms within
    a session (measured over 1109 frames, same for both ranges), while frame
    arrival jitters by 2-8 ms. So: estimate that offset robustly (median of
    recent frames), and time each spoke as stamp + offset. What remains
    unknown is the radar's internal latency between measuring and stamping,
    a constant to calibrate against a GPS-logged mover.

    Measured: the offset drifts ~20 ppm against the host (3.7 ms in ~190 s)
    and is unaffected by the epoch in the NKOE packets we send (+30 s made
    no difference), so the clock is the radar's own and must be tracked --
    the median window does that. The offset differs per session.
    """

    def __init__(self, latency_s=0.0, window=64):
        self.latency = latency_s
        self.offs = collections.deque(maxlen=window)
        self.ref = None

    def times(self, t_rx, ms):
        t_ms = t_rx * 1000.0
        o = (t_ms - ms[-1]) % 60000.0
        if self.ref is None:
            self.ref = o
        # keep offsets on one branch of the 60 s wrap
        self.offs.append(self.ref + ((o - self.ref + 30000.0) % 60000.0) - 30000.0)
        off = float(np.median(self.offs))
        # host time of each stamp, choosing the minute closest to arrival
        base = t_ms - off
        k = np.round((base - ms) / 60000.0)
        return (ms + k * 60000.0 + off) / 1000.0 - self.latency

    @property
    def offset_ms(self):
        return None if not self.offs else float(np.median(self.offs))


class TimingModel:
    """Per-spoke time from frame reception time and angle.

    Frames arrive every ~42 ms with ~2 ms jitter and carry 32 consecutive
    spokes, so the last spoke of a frame is the one just sent; earlier ones
    are older by (angle difference) / (angular rate). The rate is an EMA of
    unwrapped angle over reception time, robust to a single late frame.
    latency_s is the radar's internal delay, unknown until calibrated
    against a GPS-logged mover; it is a constant offset, not jitter.
    """

    def __init__(self, latency_s=0.05):
        self.latency = latency_s
        self.rate = None          # spokes per second
        self.last_t = None
        self.last_a = None        # unwrapped angle of the newest spoke
        self.wraps = 0
        self.prev_raw = None

    def unwrap(self, angles):
        out = np.empty(len(angles), dtype=np.int64)
        for i, a in enumerate(angles):
            if self.prev_raw is not None and a < self.prev_raw - SPOKES // 2:
                self.wraps += 1
            self.prev_raw = int(a)
            out[i] = a + self.wraps * SPOKES
        return out

    def times(self, t_rx, angles):
        ua = self.unwrap(angles)
        newest = ua[-1]
        if self.last_t is not None and t_rx > self.last_t:
            inst = (newest - self.last_a) / (t_rx - self.last_t)
            if inst > 0:
                if self.rate is None:
                    self.rate = inst
                elif 0.5 * self.rate < inst < 2.0 * self.rate:
                    self.rate += 0.05 * (inst - self.rate)
                else:
                    # A step change (B started or stopped) moves the rate by
                    # ~40%; follow it quickly instead of rejecting it forever.
                    self.rate += 0.3 * (inst - self.rate)
        self.last_t, self.last_a = t_rx, newest
        rate = self.rate or (22.0 / 60.0 * SPOKES)
        return t_rx - self.latency - (newest - ua) / rate

    @property
    def rpm(self):
        return None if self.rate is None else self.rate / SPOKES * 60.0


# --------------------------------------------------------------------------
# Clutter map and detection
# --------------------------------------------------------------------------

class ClutterMap:
    """Per-cell EMA of value and of occupancy P(value > 0), one revolution
    per update. A cell is a hit when its value stands out from its own
    history: v - mu >= k, or any echo in a cell that is almost never
    occupied. Updated after the decision, so a target is tested against the
    map as it was before the target arrived."""

    def __init__(self, alpha=1 / 16, k=3.0, occ_max=0.1, warmup_revs=16):
        self.alpha, self.k, self.occ_max = alpha, k, occ_max
        self.mu = np.zeros((SPOKES, PIXELS), dtype=np.float32)
        self.occ = np.zeros((SPOKES, PIXELS), dtype=np.float32)
        self.updates = np.zeros(SPOKES, dtype=np.int32)
        self.warmup = warmup_revs

    def warm(self, angle):
        return self.updates[angle] >= self.warmup

    def step(self, angle, v):
        """Returns the hit mask for this spoke and updates the map."""
        mu, occ = self.mu[angle], self.occ[angle]
        if self.updates[angle] == 0:
            mu[:] = v
            occ[:] = v > 0
            hits = np.zeros(PIXELS, dtype=bool)
        else:
            hits = (v > 0) & (((v - mu) >= self.k) | (occ < self.occ_max))
            if not self.warm(angle):
                hits[:] = False
            a = self.alpha
            mu += a * (v - mu)
            occ += a * ((v > 0) - occ)
        self.updates[angle] += 1
        return hits


class Detector:
    """Hit cells -> plots, one batch per sector.

    The current revolution is kept as rasters (value, hit, Doppler, time per
    spoke). When spokes enter sector s, sector s-2 is finalised: clusters are
    labelled over sectors s-3..s-1 so one straddling a boundary is seen
    whole, and those whose centroid lies in s-2 are emitted.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.maps = {}                # range scale (dm) -> ClutterMap
        self.v = np.zeros((SPOKES, PIXELS), dtype=np.uint8)
        self.hit = np.zeros((SPOKES, PIXELS), dtype=bool)
        self.dop = np.zeros((SPOKES, PIXELS), dtype=np.int8)   # +1 app, -1 rec
        self.t = np.zeros(SPOKES, dtype=np.float64)
        self.scale = np.zeros(SPOKES, dtype=np.float64)
        self.mu_snap = np.zeros((SPOKES, PIXELS), dtype=np.float32)
        self.occ_snap = np.zeros((SPOKES, PIXELS), dtype=np.float32)
        self.cur_sector = None
        self.rev = 0
        self.last_angle = None
        self.next_id = 1
        self.hits_this_rev = 0
        self.plots_this_rev = 0
        self.stats_last_rev = {"hit_cells": 0, "plots": 0}

    def map_for(self, range_m):
        key = int(round(range_m * 10))
        m = self.maps.get(key)
        if m is None:
            m = self.maps[key] = ClutterMap(**self.cfg["map"])
        return m

    def add(self, spokes, times, doppler_mode):
        """Feed one decoded frame. Returns finished plot batches."""
        out = []
        for i, a in enumerate(spokes["angle"]):
            a = int(a)
            if self.last_angle is not None and a < self.last_angle - SPOKES // 2:
                self.rev += 1
                self.stats_last_rev = {"hit_cells": self.hits_this_rev,
                                       "plots": self.plots_this_rev}
                self.hits_this_rev = self.plots_this_rev = 0
            self.last_angle = a
            s = a // SECTOR
            if self.cur_sector is not None and s != self.cur_sector:
                out.append(self.finalise((s - 2) % SECTORS))
            self.cur_sector = s

            raw = spokes["pix"][i]
            dop = np.zeros(PIXELS, dtype=np.int8)
            if doppler_mode in ("both", "approaching"):
                dop[raw == 0x0F] = 1
            if doppler_mode == "both":
                dop[raw == 0x0E] = -1
            # Flagged cells lost their intensity to the flag; treat them as
            # strong for the map rather than as their code value.
            v = np.where(dop != 0, 13, raw).astype(np.uint8)
            rng = float(spokes["range_m"][i])
            cmap = self.map_for(rng)
            self.mu_snap[a] = cmap.mu[a]
            self.occ_snap[a] = cmap.occ[a]
            hits = cmap.step(a, v.astype(np.float32))
            if cmap.warm(a):
                # Static clutter never carries a Doppler flag, so a flagged
                # cell is new by definition -- even on top of a wall echo the
                # map has learned as normal. This is what lets a mover be
                # seen where the map alone cannot.
                hits |= dop != 0
            self.v[a], self.hit[a], self.dop[a] = v, hits, dop
            self.t[a], self.scale[a] = times[i], rng
            self.hits_this_rev += int(hits.sum())
        return out

    def finalise(self, sector):
        from scipy import ndimage
        c = self.cfg["cluster"]
        rows = np.arange((sector - 1) * SECTOR, (sector + 2) * SECTOR) % SPOKES
        win = self.hit[rows]
        plots = []
        if win.any():
            lab, n = ndimage.label(win, structure=np.ones((3, 3), dtype=bool))
            for idx, sl in enumerate(ndimage.find_objects(lab), start=1):
                if sl is None:
                    continue
                m = lab[sl] == idx
                r_off, b_off = sl[0].start, sl[1].start
                rr, bb = np.nonzero(m)
                rr, bb = rr + r_off, bb + b_off
                n_spokes = len(np.unique(rr))
                ext_bins = bb.max() - bb.min() + 1
                if not (c["min_spokes"] <= n_spokes <= c["max_spokes"]
                        and len(rr) >= c["min_cells"] and ext_bins <= c["max_bins"]):
                    continue
                ang = rows[rr]
                w = self.v[ang, bb].astype(np.float64) + 1.0
                # centroid in the window's own (unwrapped) row coordinate
                cr = float(np.sum(w * rr) / w.sum())
                centre_row = rows[int(round(cr)) % len(rows)]
                if centre_row // SECTOR != sector:
                    continue
                scale = float(np.median(self.scale[ang]))
                cell = scale / PIXELS if scale > 0 else 0.0
                cb = float(np.sum(w * bb) / w.sum())
                az = ((rows[0] + cr) % SPOKES) * 360.0 / SPOKES
                ext_az = n_spokes * 360.0 / SPOKES
                ext_rng = ext_bins * cell
                t = self.t[ang]
                dop = self.dop[ang, bb]
                n_app, n_rec = int((dop > 0).sum()), int((dop < 0).sum())
                plots.append({
                    "id": self.next_id,
                    "t_utc_ns": int(float(np.sum(w * t) / w.sum()) * 1e9),
                    "t_sigma_ns": int(self.cfg["latency_sigma_s"] * 1e9),
                    "rng_m": (cb + 0.5) * cell,
                    "az_deg": az,
                    "sig_rng_m": max(cell, ext_rng / math.sqrt(12)),
                    "sig_az_deg": max(0.3, ext_az / math.sqrt(12)),
                    "ext_rng_m": ext_rng, "ext_az_deg": ext_az,
                    "n_cells": int(len(rr)), "n_spokes": int(n_spokes),
                    "peak": int(self.v[ang, bb].max()),
                    "sum": int(self.v[ang, bb].sum()),
                    "clutter_mu": float(self.mu_snap[ang, bb].mean()),
                    "clutter_occ": float(self.occ_snap[ang, bb].mean()),
                    "dop": "app" if n_app > n_rec else ("rec" if n_rec else None),
                    "dop_frac": (n_app + n_rec) / len(rr),
                })
                self.next_id += 1
        self.plots_this_rev += len(plots)
        sec_rows = np.arange(sector * SECTOR, (sector + 1) * SECTOR)
        return {"sector": sector, "rev": self.rev,
                "t_utc_ns": int(float(self.t[sec_rows].max()) * 1e9),
                "range_scale_m": float(np.median(self.scale[sec_rows])),
                "plots": plots}


# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------

def replay_source(path, scanner):
    """Recording -> (kind, t, data) for one scanner, as fast as it reads."""
    for t, ch, data in navico.read_recording(path):
        s, kind = navico.rec_channel(ch)
        if s == scanner:
            yield kind, t, data


SYNC_META_CHANNEL = 255      # scanner 127: every recording reader skips it


def sync_meta(path):
    """The sync record tools/make_radar_rec.py writes first, or None. Its
    record times are trajectory time over one period sync_period_s, so
    playing from wall-clock time mod that period puts the echo where
    scene.py --wall-clock puts the drone."""
    try:
        for _t, ch, data in navico.read_recording(path):
            if ch == SYNC_META_CHANNEL:
                meta = json.loads(data)
                return meta if meta.get("sync") == "wall-clock" else None
            return None
    except (OSError, ValueError):
        return None
    return None


if navico is not None:
    class SyncedReplay(navico.Replay):
        """navico.Replay, but each pass starts at wall-clock time mod the
        recording's period instead of at its first record, and every pass
        re-derives that phase -- so the loop never drifts off the scene."""

        def __init__(self, path, period_s, scanner=0):
            super().__init__(path, scanner=scanner)
            self.period_s = float(period_s)

        def describe(self):
            d = super().describe()
            d["source"] += ", synced to the wall clock"
            return d

        def _play(self):
            while not self._stop.is_set():
                # The phase is fixed ONCE per pass. Re-reading the clock while
                # skipping raced the wrap: a pass that ended at 359.97 of 360
                # skipped the whole file, the clock wrapped mid-skip, and the
                # pass anchored wherever the reader had got to -- measured
                # +6 s of lead per loop, the echo where the drone would be.
                base = time.time() % self.period_s
                if self.period_s - base < 0.5:
                    time.sleep(self.period_s - base)
                    base = time.time() % self.period_s
                start = time.monotonic()
                for t, channel, data in navico.read_recording(self.path):
                    if self._stop.is_set():
                        return
                    scanner, kind = navico.rec_channel(channel)
                    if scanner != self.scanner or t < base:
                        continue
                    # Reading up to the phase takes time, and a burst of
                    # stale frames would read as a jump in the radar's
                    # clock: drop what is already late, as a radar would
                    wait = (t - base) - (time.monotonic() - start)
                    if wait < -0.2:
                        continue
                    if wait > 0:
                        time.sleep(wait)
                    self._emit(kind, data)
            self.finished.set()


def live_source(radar):
    q = queue.Queue(maxsize=512)

    def on(kind, t, data):
        try:
            q.put_nowait((kind, t, data))
        except queue.Full:
            pass       # count shows up as sequence drops downstream
    radar.listeners.append(on)
    radar.start()
    while True:
        yield q.get()


def run_sim(args, site, radar_entry):
    """--source sim: emulate the radar from dc/{site}/sim/truth with
    dronelocate.radarsim and publish exactly what the real node would --
    per-sector batches (empty ones too), each one sector after it closes
    (the detector finalises sector s-2 when spokes enter s), and health
    once a second. Runs until killed. The model and what it leaves out are
    in radarsim's docstring."""
    from dronelocate.radarsim import SimRadar, SECTORS as SIM_SECTORS
    assert SIM_SECTORS == SECTORS
    p = dict(radar_entry.get("sim") or {})
    for k in ("rpm", "rpm_alt", "rpm_switch_s", "pd", "false_per_rev",
              "birds", "sig_az_deg", "range_scale_m"):
        v = getattr(args, "sim_" + k, None)
        if v is not None:
            p[k] = v
    p["az_offset_true_deg"] = (args.true_az_offset if args.true_az_offset is not None
                               else p.get("az_offset_true_deg",
                                          radar_entry.get("az_offset_deg") or 0.0))
    p["label_plots"] = bool(args.label_plots)
    p["max_det_range_m"] = p.get("max_det_range_m", p.get("range_scale_m", 3000.0))
    enu = radar_entry.get("enu", [0.0, 0.0, 6.0])
    sim = SimRadar(p, args.radar_id, enu, seed=args.seed)
    p = sim.p
    cfg_off = radar_entry.get("az_offset_deg")

    truth = collections.defaultdict(lambda: collections.deque(maxlen=64))
    tlock = threading.Lock()

    def on_truth(smp):
        try:
            t = proto.decode(smp.payload)
            with tlock:
                truth[t.get("label", "sim-uas-1")].append(
                    (float(t["t_emit_s"]), np.asarray(t["enu"], dtype=float),
                     np.asarray(t.get("vel", [0, 0, 0]), dtype=float)))
        except Exception:
            pass

    def state_fn(hist):
        def f(t):
            # interpolate between bracketing truth samples, extrapolate at
            # constant velocity past the newest (the sector being rendered
            # is at most ~0.3 s old); a truth stream silent for 2 s is a
            # target that is gone
            if not hist or t - hist[-1][0] > 2.0:
                return None
            if t >= hist[-1][0]:
                t0, p0, v0 = hist[-1]
                return p0 + v0 * (t - t0), v0
            for (ta, pa, va), (tb, pb, vb) in zip(hist[:-1], hist[1:]):
                if ta <= t <= tb:
                    w = (t - ta) / max(tb - ta, 1e-9)
                    return pa + w * (pb - pa), va + w * (vb - va)
            t0, p0, v0 = hist[0]
            return p0 + v0 * (t - t0), v0
        return f

    pub = None if args.no_zenoh else Publisher(args, site, args.radar_id)
    if pub is not None:
        pub.session.declare_subscriber(proto.ke_truth(site), on_truth)
    print(f"radarnode: SIM radar {args.radar_id} at enu {enu}, true az offset "
          f"{p['az_offset_true_deg']} deg (site config says {cfg_off}), "
          f"{p['rpm']} rpm"
          + (f" alternating with {p['rpm_alt']} every {p['rpm_switch_s']} s"
             if p.get("rpm_alt") else "")
          + f", scale {p['range_scale_m']} m, Pd {p['pd']}, "
          f"{p['false_per_rev']} false/rev, {p['birds']} birds -> "
          + ("no publish" if pub is None else proto.ke_plot(site, args.radar_id)),
          file=sys.stderr)

    t0 = time.time()
    t = t0                                   # start of the current sector
    sector, rev = 0, 0
    batches = plots_total = 0
    t_health = 0.0
    standby_at = None if args.standby_at is None else t0 + args.standby_at
    # Power from the console, as the live node does it (on = back to
    # standby, as on the HALO). Once a command has arrived it wins over
    # --standby-at.
    commanded = {"power": None}
    if pub is not None:
        def on_cmd(c):
            if c.get("power") in POWER_STATES:
                commanded["power"] = "standby" if c["power"] == "on" else c["power"]
                print(f"radarnode: SIM power -> {c['power']} (from supernode)",
                      file=sys.stderr)
        pub.on_cmd(on_cmd)
    out = collections.deque()                # (publish_time, batch)
    while True:
        now = time.time()
        rpm = sim.rpm_at(t - t0)
        status = commanded["power"] or (
            "standby" if standby_at is not None and now >= standby_at else "transmit")
        dur = 60.0 / rpm / SECTORS
        if status == "transmit":
            while t + dur <= now:
                t_end = t + dur
                sim.step_birds(t_end)
                with tlock:
                    targets = [(lbl, state_fn(list(h)), p["pd"])
                               for lbl, h in truth.items()]
                plots = sim.render(sector, t, t_end, targets)
                out.append((t_end + dur, sim.batch(sector, rev, t_end, plots,
                                                   rpm, cfg_off)))
                t = t_end
                sector += 1
                if sector == SECTORS:
                    sector, rev = 0, rev + 1
        else:
            t = now                           # antenna idle; resume from now
        while out and out[0][0] <= now:
            _, b = out.popleft()
            batches += 1
            plots_total += len(b["plots"])
            if pub:
                pub.put("plot", b)
            if args.print:
                for pl in b["plots"]:
                    print(f"rev {b['rev']:4} sec {b['sector']:2} plot {pl['id']:5} "
                          f"r {pl['rng_m']:7.1f} m az {pl['az_deg']:6.1f} deg "
                          f"dop {pl['dop']}")
        if now - t_health >= 1.0:
            t_health = now
            if pub:
                pub.put("health", sim.health(now, status, rpm, rev, plots_total,
                                             pub.dropped))
        nxt = min([t + dur if status == "transmit" else now + 0.1]
                  + ([out[0][0]] if out else []) + [t_health + 1.0])
        time.sleep(max(0.005, min(0.1, nxt - time.time())))


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------

class Publisher:
    """Zenoh publisher on its own thread with a bounded queue, so a bus
    outage never stalls the spoke loop (BLOCK on a disconnected client
    would). Plots: INTERACTIVE_HIGH/BLOCK like detection events; health:
    DATA_LOW/DROP like node health."""

    def __init__(self, args, site, radar_id):
        import zenoh
        self.proto = proto
        self.session = zenoh.open(zconf.spoke(args.hub, args.port, tls=zconf.tls_from_args(args)))
        self.pub_plot = self.session.declare_publisher(
            proto.ke_plot(site, radar_id),
            priority=zenoh.Priority.INTERACTIVE_HIGH,
            congestion_control=zenoh.CongestionControl.BLOCK)
        self.pub_health = self.session.declare_publisher(
            proto.ke_health(site, radar_id),
            priority=zenoh.Priority.DATA_LOW,
            congestion_control=zenoh.CongestionControl.DROP)
        self.q = queue.Queue(maxsize=256)
        self.dropped = 0
        self._site, self._rid = site, radar_id
        threading.Thread(target=self._run, daemon=True).start()

    def on_cmd(self, fn):
        """Call fn(dict) for every command on dc/{site}/cmd/{radar} -- the
        supernode relays the console's Standby/Transmit here, because only
        this PC is on the radar's LAN."""
        def cb(sample):
            try:
                fn(proto.decode(sample.payload))
            except Exception as e:
                print(f"radarnode: bad command: {e}", file=sys.stderr)
        self._cmd_sub = self.session.declare_subscriber(
            proto.ke_cmd(self._site, self._rid), cb)

    def put(self, kind, msg):
        try:
            self.q.put_nowait((kind, msg))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            kind, msg = self.q.get()
            (self.pub_plot if kind == "plot" else self.pub_health).put(self.proto.encode(msg))
            self.q.task_done()

    def drain(self, timeout=5.0):
        """Wait for queued messages to go out (end of a replay)."""
        end = time.monotonic() + timeout
        while self.q.unfinished_tasks and time.monotonic() < end:
            time.sleep(0.05)
        self.session.close()


# --------------------------------------------------------------------------
# Radar console (testradar's viewer.html), served by this node
# --------------------------------------------------------------------------

POWER_STATES = ("on", "off", "standby", "transmit")


def power_command(state):
    """navico frames for a power command from the console, or None.

    standby / transmit / off are navico.power_frames() itself, the bytes
    tested on the HALO. off is the soft off measured there: 00 C1 00, the
    radio side goes to status 0 while the radar stays on the network and
    keeps reporting. on is its inverse, 00 C1 01, which brings it back to
    standby in under a second -- the first frame of navico's own standby
    sequence, sent alone so that "on" never also starts transmitting."""
    if state not in POWER_STATES:
        return None
    if state == "on":
        return navico.power_frames("standby")[:1]
    return navico.power_frames(state)


def route_ip(host, port=7447):
    """This machine's address on the interface that routes to host. A UDP
    connect sends nothing; it only makes the OS pick the source address."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, int(port)))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


# Range B's console theme: the same viewer.html, recoloured from phosphor
# green to navy as it is served, so the two ranges are told apart at a
# glance (and side by side on the combined page). testradar's viewer has no
# theme switch -- green is written into its CSS and its canvas JS -- so this
# maps those literals. A literal a future viewer no longer has is reported
# and simply stays green; nothing breaks.
NAVY = [
    ("--bg: #06090c", "--bg: #050a17"), ("--panel: #0c1217", "--panel: #0a1428"),
    ("--panel-2: #10191f", "--panel-2: #0e1a33"), ("--line: #1b2730", "--line: #1a2a4a"),
    ("--line-2: #263845", "--line-2: #26385f"), ("--text: #d6e4e1", "--text: #dbe4f3"),
    ("--muted: #78908f", "--muted: #7f8fb0"), ("--dim: #45585c", "--dim: #46557a"),
    ("--accent: #3ee08f", "--accent: #5b9bff"),
    ("rgba(62, 224, 143", "rgba(91, 155, 255"), ("rgba(62,224,143", "rgba(91,155,255"),
    ("rgba(6, 9, 12", "rgba(5, 10, 23"), ("#04130d", "#050c1f"),
    ("rgba(200,255,220", "rgba(205,222,255"),
    # the echo-strength ramp, dark to bright
    ("[[8, 58, 40], [26, 150, 92], [62, 224, 143], [196, 255, 214]]",
     "[[8, 20, 58], [24, 66, 160], [91, 155, 255], [205, 225, 255]]"),
]
THEMES = {"navy": NAVY}


def themed(html, theme):
    """viewer.html with a theme's colour map applied; (html, missing)."""
    missing = []
    for old, new in THEMES.get(theme, []):
        if old in html:
            html = html.replace(old, new)
        else:
            missing.append(old)
    return html, missing


def start_console(radar, bind, port, theme=None):
    """Serve testradar's own viewer.html and its /api on bind:port, against
    this node's live Radar.

    The page and its API come from webserver.py beside navico.py in the
    testradar subtree, loaded by path rather than copied, so the radar
    console here IS testradar's: same design, same controls, nothing to keep
    in step. The node holds the only connection to the radar, so it is the
    only place this can be served from -- the supernode cannot reach the
    radar's LAN. Returns the server, or None (and says why) if the checkout
    has no webserver.py, e.g. navico resolved to a bare pip install."""
    import importlib.util
    from http.server import ThreadingHTTPServer
    path = os.path.join(os.path.dirname(os.path.abspath(navico.__file__)), "webserver.py")
    if not os.path.exists(path):
        print(f"radarnode: no radar console: {path} not found (needs a testradar "
              f"checkout, not just the navico module)", file=sys.stderr)
        return None
    spec = importlib.util.spec_from_file_location("testradar_webserver", path)
    web = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(web)
    hub = web.Hub(radar)                 # listens on radar.listeners
    Base = web.make_handler(hub)
    handler = Base
    if theme:
        with open(os.path.join(os.path.dirname(path), "viewer.html"), encoding="utf-8") as f:
            page, missing = themed(f.read(), theme)
        if missing:
            print(f"radarnode: theme {theme}: {len(missing)} colour(s) not in this "
                  f"viewer.html, left as they are: {missing}", file=sys.stderr)
        body = page.encode("utf-8")

        class handler(Base):             # noqa: N801 -- a class, named for its role
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(200, body, "text/html; charset=utf-8")
                else:
                    super().do_GET()
    srv = ThreadingHTTPServer((bind, int(port)), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# --------------------------------------------------------------------------

DEFAULT_CFG = {
    "map": {"alpha": 1 / 16, "k": 3.0, "occ_max": 0.1, "warmup_revs": 16},
    "cluster": {"min_spokes": 6, "max_spokes": 60, "min_cells": 6, "max_bins": 10},
    "latency_s": 0.0,          # radar internal delay; calibrate, then set
    "latency_sigma_s": 0.05,
}


def load_site(path, radar_id):
    """(site id, radar entry, LocalFrame or None) from a dronelocate site json."""
    with open(path) as f:
        d = json.load(f)
    entry = next((r for r in d.get("radars", []) if r.get("id") == radar_id), {})
    return d["site"], entry, d.get("origin")


def discover_by_serial(serial, iface, timeout=5.0):
    """navico.discover, but picking the radar by its serial number from
    whatever address it answers on: DHCP can move the radar (it came up on
    .249 and .237 instead of .231), the serial stays. A second radar on the
    LAN is ignored rather than taken for this one."""
    rx = navico.multicast_rx(*navico.DISCOVERY_ADDR, iface, timeout=0.5)
    tx = navico.multicast_tx(iface)
    deadline = time.monotonic() + timeout
    next_query = 0.0
    try:
        while time.monotonic() < deadline:
            if time.monotonic() >= next_query:
                tx.sendto(navico.DISCOVERY_QUERY, navico.DISCOVERY_ADDR)
                next_query = time.monotonic() + 1.0
            try:
                data, src = rx.recvfrom(2048)
            except socket.timeout:
                continue
            beacon = navico.parse_beacon(data)
            if beacon and beacon["scanners"] and beacon["serial"] == serial:
                beacon["from"] = src[0]
                return beacon
    finally:
        rx.close()
        tx.close()
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radar", default=None,
                    help="radar IP (default: radars[].ip in --site, else "
                         "192.168.100.231); with a serial it is only a hint "
                         "for which interface to discover on")
    ap.add_argument("--radar-serial", default=None,
                    help="find the radar by serial number at any address "
                         "(default: radars[].serial in --site; unset = by IP)")
    ap.add_argument("--retry-s", type=float, default=5.0,
                    help="live: wait this long between discovery attempts "
                         "until the radar answers (0 = give up after one)")
    ap.add_argument("--http-port", type=int, default=0,
                    help="live: serve the radar console (testradar's "
                         "viewer.html) on this port, e.g. 8082; 0 = off")
    ap.add_argument("--http-bind", default="0.0.0.0")
    ap.add_argument("--theme", choices=["green", "navy"], default=None,
                    help="radar console colours (default: green for range A, "
                         "navy for range B)")
    ap.add_argument("--realtime", action="store_true",
                    help="with --replay: play the recording at its own pace, "
                         "looping, through the live path (implied by "
                         "--http-port)")
    ap.add_argument("--console-url",
                    help="URL announced to the supernode for the console "
                         "link (default http://<address routing to --hub>:"
                         "<http-port>/); set it behind NAT")
    ap.add_argument("--scanner", type=int, default=0, help="0 = range A, 1 = range B")
    ap.add_argument("--replay", metavar="FILE", help="process a navico.py recording")
    ap.add_argument("--site", help="dronelocate site json (site id, radars block)")
    ap.add_argument("--radar-id", default="r01")
    zconf.add_args(ap)
    ap.add_argument("--no-zenoh", action="store_true", help="do not publish")
    ap.add_argument("--print", action="store_true", help="print every plot")
    ap.add_argument("--csv", help="append every plot to this CSV")
    ap.add_argument("--heading", type=float,
                    help="live: feed this fixed heading (deg) so Doppler works")
    ap.add_argument("--k", type=float, help="clutter-map threshold, levels")
    ap.add_argument("--warmup", type=int, help="revolutions before detecting")
    ap.add_argument("--source", choices=["live", "sim"], default="live",
                    help="sim: emulate the radar from dc/{site}/sim/truth")
    sg = ap.add_argument_group("sim source (overrides radars[].sim in the site file)")
    sg.add_argument("--seed", type=int, help="sim RNG seed")
    sg.add_argument("--true-az-offset", type=float,
                    help="mounting offset the sim renders (default: the site "
                         "config's az_offset_deg) -- set it different to "
                         "watch a miscalibrated radar")
    sg.add_argument("--sim-rpm", type=float)
    sg.add_argument("--sim-rpm-alt", type=float,
                    help="alternate with this rate (scan speed / range B)")
    sg.add_argument("--sim-rpm-switch-s", type=float)
    sg.add_argument("--sim-pd", type=float)
    sg.add_argument("--sim-false-per-rev", type=float)
    sg.add_argument("--sim-birds", type=int)
    sg.add_argument("--sim-sig-az-deg", type=float)
    sg.add_argument("--sim-range-scale-m", type=float)
    sg.add_argument("--standby-at", type=float,
                    help="go to standby this many seconds after start")
    sg.add_argument("--label-plots", action="store_true",
                    help="tag plots with the truth label (scoring aid)")
    args = ap.parse_args()

    if args.source == "sim":
        site, radar_entry, _origin = load_site(args.site or "config/site.json",
                                               args.radar_id)
        if not radar_entry:
            print(f"radar {args.radar_id} not in the site's radars[] block",
                  file=sys.stderr)
            return 1
        run_sim(args, site, radar_entry)
        return 0

    if navico is None:
        print(NAVICO_MISSING, file=sys.stderr)
        return 1

    cfg = json.loads(json.dumps(DEFAULT_CFG))
    if args.k is not None:
        cfg["map"]["k"] = args.k
    if args.warmup is not None:
        cfg["map"]["warmup_revs"] = args.warmup

    site, radar_entry = "local", {}
    if args.site:
        site, radar_entry, _origin = load_site(args.site, args.radar_id)
    az_offset = radar_entry.get("az_offset_deg")
    # The site file is where the radar's address lives; --radar overrides it.
    radar_ip = args.radar or radar_entry.get("ip") or "192.168.100.231"
    radar_serial = args.radar_serial or radar_entry.get("serial")

    pub = None if args.no_zenoh else Publisher(args, site, args.radar_id)
    console_url = None

    # range B's console is navy so the two ranges are told apart at a glance
    theme = args.theme if args.theme is not None else ("navy" if args.scanner == 1 else "")
    theme = None if theme in ("", "green") else theme
    realtime = args.replay and (args.realtime or args.http_port)
    if args.replay and not realtime:
        state = {}
        radar = None
        src = replay_source(args.replay, args.scanner)
        origin = f"replay of {os.path.basename(args.replay)} range {'AB'[args.scanner]}"
    elif realtime:
        # A recording through the LIVE path: testradar's Replay has the
        # Radar interface (state, listeners, spokes at the recorded pace,
        # looping), so the console and the publishing behave exactly as
        # with the radar connected. Spoke times are wall-clock, as live.
        meta = sync_meta(args.replay)
        if meta:
            # an emulated recording of the sim drone: play it in step with
            # scene.py --wall-clock (see tools/make_radar_rec.py)
            radar = SyncedReplay(args.replay, meta["sync_period_s"], scanner=args.scanner)
        else:
            radar = navico.Replay(args.replay, scanner=args.scanner)
        state = radar.state
        origin = radar.describe()["source"] + ", real time, looping"
        if args.http_port:
            if start_console(radar, args.http_bind, args.http_port, theme) is not None:
                console_url = args.console_url or (
                    f"http://{route_ip(args.hub, args.port)}:{args.http_port}/")
                print(f"radarnode: radar console on {console_url}", file=sys.stderr)
        if pub:
            pub.on_cmd(lambda c: c.get("power") and print(
                f"radarnode: power -> {c['power']} ignored: this is a recording",
                file=sys.stderr))
        src = live_source(radar)
    else:
        # Wait for the radar instead of exiting: a node started before the
        # radar has powered up (or before its LAN is plugged in) used to die
        # with "No beacon" and needed restarting by hand. Health goes out
        # meanwhile, so the console says "not found" rather than nothing.
        said = False
        while True:
            iface = navico.local_ip_for(radar_ip)
            beacon = (discover_by_serial(radar_serial, iface) if radar_serial
                      else navico.discover(radar_ip, iface))
            if beacon is not None:
                break
            if not said:
                who = f"serial {radar_serial}" if radar_serial else radar_ip
                print(f"radarnode: no beacon from {who} (via {iface}); "
                      + (f"retrying every {args.retry_s:g} s" if args.retry_s > 0
                         else "giving up"), file=sys.stderr)
                said = True
            if args.retry_s <= 0:
                return 1
            if pub:
                pub.put("health", {"v": 3, "node": args.radar_id, "kind": "radar",
                                   "t_utc_ns": int(time.time() * 1e9),
                                   "status": "not found", "source": "halo",
                                   "radar_ip": radar_ip})
            time.sleep(args.retry_s)
        if beacon["from"] != radar_ip:
            print(f"radarnode: serial {radar_serial} found at {beacon['from']} "
                  f"(site file says {radar_ip})", file=sys.stderr)
            radar_ip = beacon["from"]
        radar = navico.Radar(beacon, iface, args.scanner)
        state = radar.state
        if args.heading is not None:
            radar.set_nav(heading=args.heading)
        origin = f"{beacon['from']} serial {beacon['serial']} range {'AB'[args.scanner]}"
        if args.http_port:
            if start_console(radar, args.http_bind, args.http_port, theme) is not None:
                console_url = args.console_url or (
                    f"http://{route_ip(args.hub, args.port)}:{args.http_port}/")
                print(f"radarnode: radar console on {console_url}", file=sys.stderr)
        if pub:
            def on_cmd(c):
                frames = power_command(c.get("power"))
                if frames is not None:
                    radar.send(frames)
                    print(f"radarnode: power -> {c['power']} (from supernode)",
                          file=sys.stderr)
            pub.on_cmd(on_cmd)
        # The radar powers up in standby. An unattended site wants it
        # transmitting again after every power cut, but only when the site
        # file says so: it is a transmitter and must never start by accident.
        # Sent on every connect, by both range nodes; transmit is idempotent.
        if radar_entry.get("auto_transmit"):
            radar.send(navico.power_frames("transmit"))
            print("radarnode: auto_transmit -> transmit", file=sys.stderr)
        src = live_source(radar)       # after the console: start() once, here
    writer = None
    if args.csv:
        new = not os.path.exists(args.csv)
        fcsv = open(args.csv, "a", newline="")
        writer = csv.writer(fcsv)
        if new:
            writer.writerow(["t_utc", "rev", "sector", "id", "rng_m", "az_deg", "n_spokes",
                             "n_cells", "peak", "sum", "clutter_mu", "clutter_occ", "dop",
                             "dop_frac", "sig_rng_m", "sig_az_deg"])

    timing = TimingModel(cfg["latency_s"])     # kept for the rotation rate
    clock = SpokeClock(cfg["latency_s"])
    det = Detector(cfg)
    frames = drops = resets = 0
    last_seq = None
    t_health = 0.0
    batches = plots_total = 0
    proc_s = 0.0
    print(f"radarnode: {origin} -> "
          + ("no publish" if pub is None else proto.ke_plot(site, args.radar_id)),
          file=sys.stderr)

    last_scale, last_angle = 0.0, 0

    def health_msg(t):
        return {"v": 3, "node": args.radar_id, "kind": "radar",
                "scanner": "AB"[args.scanner], "t_utc_ns": int(t * 1e9),
                "status": state.get("mode", {}).get("status"),
                "rpm": timing.rpm, "clock_offset_ms": clock.offset_ms,
                # the radar's own measurement (0xC408), same on both ranges
                "antenna_rpm_reported": state.get("setup_extended", {}).get("antenna_rpm"),
                "frames": frames, "seq_drops": drops,
                "seq_resets": resets,
                "range_scale_m": float(last_scale), "rev": det.rev,
                "maps": len(det.maps),
                "warm": bool(last_scale > 0 and det.map_for(last_scale).warm(last_angle)),
                "last_rev": det.stats_last_rev, "plots_total": plots_total,
                "ms_per_frame": 1000.0 * proc_s / max(frames, 1),
                "publish_dropped": 0 if pub is None else pub.dropped,
                "source": "replay" if args.replay else "halo",
                "radar_ip": None if args.replay else radar_ip,
                "console_url": console_url,
                "nav": None if args.replay or radar.nav is None else {
                    "heading": radar.nav.heading, "yielding": radar.nav.yielding()}}

    for kind, t, data in src:
        if kind == navico.CHANNEL_REPORT:
            r = navico.decode_report(data)
            if r and args.replay:
                state[r[0]] = r[1]
            # In standby or off the radar sends reports but no spokes. Health
            # must still go out, or the supernode can only say "unavailable"
            # where the truth is "standby".
            if t - t_health >= 1.0:
                t_health = t
                if pub:
                    pub.put("health", health_msg(t))
            continue
        dec = decode_frame(data)
        if dec is None:
            continue
        frames += 1
        last_scale, last_angle = float(dec["range_m"][-1]), int(dec["angle"][-1])
        # The u8 spoke counter steps +1. A gap is lost spokes; a big jump or a
        # repeat is the counter restarting (seen when range B starts or stops
        # transmitting), which counting as 255 lost spokes would hide real
        # drops behind.
        seq = dec["seq"]
        steps = np.diff(seq) if last_seq is None else np.diff(np.concatenate(([last_seq], seq)))
        gaps = (steps - 1) % 4096
        drops += int(gaps[gaps < 256].sum())
        resets += int((gaps >= 256).sum())
        last_seq = int(seq[-1])

        t0 = time.perf_counter()
        timing.times(t, dec["angle"])
        times = clock.times(t, dec["ms"])
        doppler = state.get("setup_extended", {}).get("doppler", "off")
        for b in det.add(dec, times, doppler):
            b.update({"v": 3, "node": args.radar_id, "scanner": "AB"[args.scanner],
                      "t_rev_s": None if timing.rpm is None else 60.0 / timing.rpm,
                      "cell_m": b["range_scale_m"] / PIXELS,
                      "az_ref": "antenna", "az_offset_deg": az_offset,
                      "settings": {k: state.get(s, {}).get(k) for s, k in (
                          ("setup", "use_mode"), ("setup", "gain"),
                          ("setup_extended", "doppler"),
                          ("setup_extended", "doppler_speed_m_s"),
                          ("setup_extended", "noise_rejection"),
                          ("setup_extended", "scan_speed"))},
                      "rpm_measured": timing.rpm})
            batches += 1
            plots_total += len(b["plots"])
            if pub:
                pub.put("plot", b)
            for p in b["plots"]:
                if args.print:
                    print(f"rev {b['rev']:4} sec {b['sector']:2} plot {p['id']:5} "
                          f"r {p['rng_m']:7.1f} m az {p['az_deg']:6.1f} deg "
                          f"spokes {p['n_spokes']:3} cells {p['n_cells']:4} "
                          f"peak {p['peak']:2} occ {p['clutter_occ']:.2f} dop {p['dop']}")
                if writer:
                    writer.writerow([p["t_utc_ns"] / 1e9, b["rev"], b["sector"], p["id"],
                                     f"{p['rng_m']:.2f}", f"{p['az_deg']:.2f}",
                                     p["n_spokes"], p["n_cells"], p["peak"], p["sum"],
                                     f"{p['clutter_mu']:.2f}", f"{p['clutter_occ']:.3f}",
                                     p["dop"], f"{p['dop_frac']:.2f}",
                                     f"{p['sig_rng_m']:.2f}", f"{p['sig_az_deg']:.2f}"])
        proc_s += time.perf_counter() - t0

        if t - t_health >= 1.0:
            t_health = t
            if pub:
                pub.put("health", health_msg(t))

    print(f"radarnode: {frames} frames, {det.rev} revolutions, {batches} sector batches, "
          f"{plots_total} plots, {drops} spokes dropped, {resets} counter restarts, "
          f"{1000.0 * proc_s / max(frames, 1):.1f} ms/frame, "
          f"rpm {timing.rpm and round(timing.rpm, 1)}", file=sys.stderr)
    if writer:
        fcsv.close()
    if pub:
        pub.drain()
    return 0


if __name__ == "__main__":
    sys.exit(main())
