"""Radar + TDOA track fusion. Transport-free, like track.py and tdoa.py, so
the whole track layer can be driven offline (validate_fusion.py) with a
simulated clock and nothing but numpy and scipy.

One owner of the track list. Everything that changes a track -- a TDOA fix,
the tight-coupling rows of a starved burst, a radar plot batch -- is
submitted to a heap keyed on *measurement* time and applied by a single
thread in time order, once it is older than now - lag. The lag (~1 s) is
what lets a radar plot measured at t=10.2 s be applied before a TDOA fix
measured at t=10.4 s even though the fix's IQ pull and solve put it on the
bus 0.6-0.8 s later than the plot. Without it the two sources interleave in
arrival order, and the constant-velocity filter would be asked to run its
clock backwards every time a slow solve finished.

The lag is only paid while a radar is actually feeding plots. With no radar
-- none configured, none publishing, or one that is not calibrated -- there
is nothing to reorder against, the lag is zero, and every item is applied
the moment it arrives: exactly the TDOA-only behaviour that existed before
this module, in the same order the old track_lock produced. When a radar
goes quiet mid-run the lag drops to zero as soon as its batches are stale,
so a dead radar can delay fixes by at most its staleness timeout, never
indefinitely.

Design choices, and what they cost (measured numbers in the project notes):

- Association: TDOA fixes against tracks with a 3D Mahalanobis gate
  (chi-square(3) 99%, fix.cov + P); radar plots by GNN in measurement space
  (scipy linear_sum_assignment, chi-square(2) 99%), one plot per track per
  batch.
- Outlier rejection is earned by independent evidence. A fix that fails the
  gate of a track the radar has recently confirmed is rejected rather than
  allowed to drag the track: that is the ~300 m wrong fix a lost correlation
  peak produces with four nodes. A fix that fails the gate of a TDOA-only
  track still updates it, as before -- TDOA alone has no second opinion that
  says the fix rather than the track is wrong.
- Radar-only tracks start tentative, seeded with the solver's altitude prior
  (the radar measures slant range, not height), and confirm on M hits in N
  scan opportunities. Misses are field-of-view aware: no miss is counted
  when the prediction is outside the vertical fan, beyond the range scale,
  or the radar is not usable, so a drone overhead in the blind cone coasts
  instead of being deleted.
"""

import collections
import heapq
import itertools
import math
import threading
import time

import numpy as np

from .geo import C_LIGHT
from .track import Tracker, _CHI2_99, radar_measurement, radar_rates, wrap_pi

#: Fusion defaults. Every key can be overridden by the site config's
#: "fusion" block; the comments say why each number is what it is.
DEFAULTS = {
    # Reorder window. Covers TDOA pull + correlate + solve (~0.6-0.8 s on
    # the 10-node sim) with margin; a radar batch is ready ~2 sectors after
    # its plots were measured (~0.2-0.4 s at 22-48 rpm) and is never the
    # long pole.
    "lag_s": 1.0,
    # A radar whose plot batches are older than this is not waited for.
    # 16 batches per revolution means ~0.08-0.17 s between batches, so 3 s
    # is many missed batches, not one late one.
    "radar_timeout_s": 3.0,
    "gate_tdoa": _CHI2_99[3],          # 11.345
    "gate_radar": _CHI2_99[2],         # 9.210
    "gate_merge": _CHI2_99[6],         # 16.812, full 6-state distance
    "merge_hits": 3,
    # Merging is for two tracks on ONE target, so both must be currently
    # measured (updated within coast_s) and actually close: a coasting
    # track's inflated P passes the chi-square gate against anything, and
    # merged two TDOA drones 100 m apart after 4 s of silence (review A1).
    "merge_dist_m": 30.0,
    "merge_sigma_h_m": 50.0,
    # Radar-only confirmation: M hits in the first N scan opportunities,
    # and the faster rule for a target the radar itself flags as moving.
    "confirm_m": 3, "confirm_n": 4,
    "confirm_m_doppler": 2, "confirm_n_doppler": 3,
    "delete_misses": 4,                # consecutive, field-of-view aware
    # Vertical fan. The HALO's is +/-12.5 deg but sensitivity is only good
    # to ~9 deg; a miss outside the good part is not evidence of absence.
    "fov_elev_deg": 9.0,
    "min_range_m": 30.0,
    # A TDOA update this recent keeps a track alive whatever the radar says:
    # the drone is emitting, the radar just does not see it (range, RCS).
    "tdoa_hold_s": 5.0,
    # No fix seeds or confirms a tentative track below this height (ENU m):
    # the below-ground mirror solution of near-coplanar nodes is not a
    # second drone.
    "min_alt_m": -30.0,
    # A radar-supported track that refuses this many consistent fixes in a
    # row (each within reject_consistent_m + 40 m/s of travel of the last)
    # is re-seeded from the fix: three agreeing fixes outvote one track.
    "reject_reset": 3,
    "reject_consistent_m": 20.0,
    # Radar-only coasting (blind cone, radar down) ends when the prediction
    # is too vague to be worth drawing.
    "max_sigma_h_m": 2000.0,
    # A track with a radar hit this recent is "radar-supported": its gate
    # failures are outliers, not reasons to drag it.
    "radar_support_s": 10.0,
    # ...and has had at least this many radar hits in all. One plot is not
    # evidence the radar sees a target -- it may be a clutter plot that
    # happened to fall in a TDOA track's gate -- so below this a track
    # neither refuses fixes nor is deleted for radar misses.
    "radar_min_hits": 3,
    # The pre-fusion association gate, kept for TDOA-only tracks.
    "legacy_gate_m": 2500.0,
    # A tentative TDOA track (born from a rejected fix) needs a second,
    # consistent fix within this long or it was just an outlier.
    "tdoa_confirm_s": 3.0,
    # ...and needs this many mutually consistent fixes in all. Two was not
    # enough: a lost peak that holds for two bursts published a confirmed
    # track ~300 m off, which then lived forever (follow-up review).
    "tdoa_confirm_n": 3,
    # Fused mode only: a confirmed track the radar has not established
    # (fewer than radar_min_hits plots) is deleted after this long without
    # an update. Before that, once it has gone quiet for coast_s, it is
    # "stale": no single fix updates it any more -- only a confirmed hidden
    # tentative can resume it (it then gets the tentative's state and keeps
    # its id). A stale track's prediction gates everything, and a lone
    # outlier that missed the live track's gate used to be published on it.
    # TDOA-only mode never deletes (pre-fusion parity).
    "tdoa_coast_s": 30.0,
    # Track score for radar-only confirmation (sequential likelihood
    # ratio, Blackman & Popoli): M-of-N alone ignores how cluttered the
    # picture is, and measured here it confirmed 12 false tracks in 10 min
    # at 5 false plots per revolution. confirm_llr = ln(1/alpha), alpha
    # ~1e-5 per tentative (~5000 tentatives in 10 min at 20 false/rev);
    # ln(1e4) = 9.2 still let one through there, 11.5 none, and a real
    # target still confirms on its 3rd plot. The clutter density it is
    # scored against is measured online from unassociated plots, floored
    # so a quiet sky does not become infinitely trusting.
    "confirm_llr": 11.5,
    "radar_pd": 0.8,                   # assumed detection probability in view
    "clutter_per_rev_init": 2.0,
    "clutter_per_rev_min": 0.5,
    "clutter_dop_per_rev_min": 0.05,   # Doppler-flagged false plots are rare
    # Tentative radar tracks that see no scan opportunity at all (radar
    # gone, out of view) are dropped after this.
    "tentative_timeout_s": 15.0,
    # A confirmed track with no update for this long is reported coasting.
    "coast_s": 3.0,
    # Radar seed: altitude unknown, so sigma_U = max(frac * range, min).
    # 0.13 ~ tan(7.5 deg): the target is somewhere in the good part of the
    # fan, not at a known elevation.
    "seed_alt_sigma_frac": 0.13,
    "seed_alt_sigma_min_m": 30.0,
    "seed_vel_sigma_mps": 20.0,
    "seed_vz_sigma_mps": 5.0,
    # Propagation step for radar-supported tracks (Tracker.max_step_s): a
    # coast through the blind cone is tens of seconds, and one long step
    # over-inflates the prediction ~4x at 20 s.
    "coast_step_s": 1.0,
    # Radar latency uncertainty when the plot does not carry t_sigma_ns.
    "latency_sigma_s": 0.05,
    # Calibration monitor: innovations of plots on tracks a fix pinned
    # within this long, with horizontal sigma under cal_sigma_h_m, estimate
    # the radar's azimuth bias and clock offset against TDOA.
    "cal_fix_age_s": 1.0,
    "cal_sigma_h_m": 3.0,
    "cal_sigma_v_m": 10.0,             # range samples also need the height
    "cal_window": 300,
}


def c_gate(cfg):
    return float(cfg["gate_radar"])


class RadarSite:
    """A radar's place in the site frame, from the site config's radars[]."""

    def __init__(self, entry, fov_elev_deg):
        self.id = entry["id"]
        self.enu = np.asarray(entry.get("enu", [0.0, 0.0, 0.0]), dtype=float)
        off = entry.get("az_offset_deg")
        self.az_offset_deg = None if off is None else float(off)
        self.psi = None if off is None else math.radians(float(off))
        self.beam_h_deg = entry.get("beam_h_deg")
        self.fov_elev = math.radians(float(entry.get("fov_elev_deg",
                                                     fov_elev_deg)))

    @property
    def calibrated(self):
        return self.psi is not None

    def elevation(self, p):
        d = np.asarray(p[:3], dtype=float) - self.enu
        return math.atan2(float(d[2]), max(math.hypot(d[0], d[1]), 1e-6))

    def slant(self, p):
        return float(np.linalg.norm(np.asarray(p[:3], dtype=float) - self.enu))


def radar_key(msg):
    """Fusion's key for one radar RANGE: the radar id for range A, id:B for B.

    A HALO's two ranges are one antenna with two range scales, published by
    two radarnode processes. Keyed by id alone they overwrote each other's
    status and mixed their clutter maps. Keyed apart, each range keeps its
    own status, clutter density and miss bookkeeping, and they share the one
    RadarSite (position, mounting azimuth) -- see Fusion.__init__."""
    rid = msg.get("node", "?")
    sc = msg.get("scanner")
    return rid if sc in (None, "A") else f"{rid}:{sc}"


class RadarStatus:
    """Is this radar usable right now, and if not, why not.

    Fed by the supernode's subscribers with wall-clock arrival times. Usable
    for fusion means: configured in the site file, calibrated (az_offset_deg
    known), and plot batches arriving -- health alone is not enough, because
    the fusion lag would then wait for plots that never come.
    """

    # "not found": radarnode is up and publishing health, but the radar has
    # not answered discovery yet (powered off, or its LAN not connected)
    BAD_STATUS = ("standby", "off", "warming up", "not found")

    def __init__(self, rid, site=None, timeout_s=3.0):
        self.id = rid
        self.site = site
        self.timeout = float(timeout_s)
        self.last_health = None
        self.last_batch = None
        self.status = None
        self.rpm = None
        self.t_rev_s = None
        self.range_scale_m = None
        self.batches = 0
        self.plots = 0
        self.batches_unfused = 0
        self.plots_unfused = 0
        self.batches_bad_frame = 0
        self.source = None
        self._rate = collections.deque()
        self._lock = threading.Lock()     # _rate: bus thread writes, owner reads
        self.offset_mismatch = None
        # Operator switch (console "Fuse"). Off is indistinguishable from a
        # radar that stopped publishing: fusable() is false, so the track
        # layer falls back to TDOA-only within the lag, exactly as when a
        # radar is lost mid-run. A plain bool, written by the HTTP thread.
        self.enabled = True
        # Where the radar node serves its own console (viewer.html), as it
        # announced in health. The node knows its address; nothing else does.
        self.console_url = None

    def on_health(self, h, now):
        self.last_health = now
        self.status = h.get("status") or self.status
        self.console_url = h.get("console_url", self.console_url)
        rpm = h.get("antenna_rpm_reported") or h.get("rpm")
        if rpm:
            self.rpm = float(rpm)
        if h.get("range_scale_m"):
            self.range_scale_m = float(h["range_scale_m"])
        self.source = h.get("source", self.source)

    def on_batch(self, b, now):
        self.last_batch = now
        self.batches += 1
        n = len(b.get("plots") or [])
        self.plots += n
        with self._lock:
            self._rate.append((now, n))
            while self._rate and self._rate[0][0] < now - 10.0:
                self._rate.popleft()
        if b.get("rpm_measured"):
            self.rpm = float(b["rpm_measured"])
        if b.get("t_rev_s"):
            self.t_rev_s = float(b["t_rev_s"])
        if b.get("range_scale_m"):
            self.range_scale_m = float(b["range_scale_m"])
        # Batches imply a transmitting radar when no health has said
        # otherwise yet. Health stays the authority on standby/off.
        if self.status is None:
            self.status = "transmit"
        off = b.get("az_offset_deg")
        if (self.site is not None and off is not None
                and self.site.az_offset_deg is not None
                and abs(wrap_pi(math.radians(off - self.site.az_offset_deg))) > 1e-6):
            self.offset_mismatch = float(off)

    def _fresh(self, t, now):
        return t is not None and now - t <= self.timeout

    def available(self, now):
        return (self._fresh(self.last_batch, now)
                and self.status not in self.BAD_STATUS)

    def calibrated(self):
        return self.site is not None and self.site.calibrated

    def fusable(self, now):
        return (self.enabled and self.site is not None and self.calibrated()
                and self.available(now))

    def state(self, now):
        if self.site is None:
            return "unconfigured"
        if self.last_health is None and self.last_batch is None:
            return "unavailable"
        if not self.available(now):
            if self._fresh(self.last_health, now) and self.status in self.BAD_STATUS:
                return self.status
            return "unavailable"
        if not self.calibrated():
            return "not calibrated"
        return "available"

    def plot_rate(self, now):
        """Plots per second over the last 10 s. Read-only: pruning happens
        in on_batch, on the thread that appends."""
        with self._lock:
            return sum(n for t, n in self._rate if t >= now - 10.0) / 10.0

    def summary(self, now):
        age = lambda t: None if t is None else round(now - t, 2)
        return {
            "id": self.id, "state": self.state(now),
            "available": self.available(now),
            "calibrated": self.calibrated(),
            "fused": self.fusable(now),
            "enabled": self.enabled,
            "console_url": self.console_url,
            "configured": self.site is not None,
            "status": self.status, "rpm": self.rpm, "t_rev_s": self.t_rev_s,
            "range_scale_m": self.range_scale_m,
            "batch_age_s": age(self.last_batch),
            "health_age_s": age(self.last_health),
            "batches": self.batches, "plots": self.plots,
            "batches_unfused": self.batches_unfused,
            "plots_unfused": self.plots_unfused,
            "batches_bad_frame": self.batches_bad_frame,
            "plot_rate_hz": round(self.plot_rate(now), 2),
            "enu": None if self.site is None else [float(v) for v in self.site.enu],
            "az_offset_deg": None if self.site is None else self.site.az_offset_deg,
            "az_offset_on_bus_deg": self.offset_mismatch,
            "source": self.source,
        }


class FusedTrack:
    """A Tracker plus the bookkeeping fusion needs around it."""

    def __init__(self, kf, state, src, t, alt_src):
        self.kf = kf
        self.tid = kf.tid
        self.state = state              # tentative | confirmed (coasting derived)
        self.src = set(src)
        self.n_tdoa = 0
        self.n_radar = 0
        self.n_tight = 0
        self.last_tdoa_t = None
        self.last_radar_t = None
        self.last_update_t = t
        self.created_t = t
        self.alt_src = alt_src
        self.last_fix_sigma_v = None
        self.scans = []                 # radar scan outcomes while tentative
        self.dop_hits = 0
        self.llr = 0.0                  # radar track score while tentative
        self.last_plot = None           # (radar id, R) of the last plot hit
        self.misses = 0                 # consecutive radar misses
        self.last_scan = {}             # radar id -> time of last plot hit
        self.dead = False
        self.tentative_tdoa = False
        self.reject_streak = 0          # consecutive fixes this track refused
        self.last_rejected = None       # (t, enu) of the last one

    def radar_supported(self, t, support_s, min_hits=3):
        return (self.state == "confirmed" and self.last_radar_t is not None
                and self.n_radar >= min_hits
                and t - self.last_radar_t <= support_s)

    def snapshot(self, t_now, coast_s):
        st = self.state
        if st == "confirmed" and t_now - self.last_update_t > coast_s:
            st = "coasting"
        kf = self.kf
        return {
            "tid": self.tid, "state": st,
            "src": sorted(self.src),
            "n_tdoa": self.n_tdoa, "n_radar": self.n_radar,
            "n_tight": self.n_tight,
            "last_tdoa_t": self.last_tdoa_t, "last_radar_t": self.last_radar_t,
            "alt_src": self.alt_src,
            "t_state": float(kf.t), "late": kf.late,
            "x": kf.x.copy(), "P": kf.P.copy(),
        }


class Fusion:
    """The track layer. See the module docstring."""

    def __init__(self, cfg=None, radars=(), alt_prior_m=100.0,
                 alt_prior_sigma_m=60.0):
        self.cfg = dict(DEFAULTS)
        self.cfg.update({k: v for k, v in (cfg or {}).items()
                         if not str(k).startswith("_")})
        c = self.cfg
        # The prior update() applies after every fix: None means none, exactly
        # as before fusion (the supernode passes solver.alt_prior_m as is).
        self.alt_prior_m = alt_prior_m
        self.alt_prior_sigma_m = float(alt_prior_sigma_m or 60.0)
        # Where radar-only work puts an unmeasured altitude. Must be a
        # number whatever the site file says -- the solver itself defaults
        # to 100 m -- or every unassociated plot raises (review A3).
        self.seed_alt_m = 100.0 if alt_prior_m is None else float(alt_prior_m)
        self.sites = {}
        self.status = {}
        # range key -> the other range of the same radar (both directions)
        self.range_pair = {}
        for r in radars or []:
            s = RadarSite(r, c["fov_elev_deg"])
            self.sites[s.id] = s
            self.status[s.id] = RadarStatus(s.id, s, c["radar_timeout_s"])
            # Range B is fused only if the site file says the radar runs it:
            # same antenna, so the same RadarSite object, not a copy.
            if "B" in (r.get("scanners") or []):
                kb = f"{s.id}:B"
                self.sites[kb] = s
                self.status[kb] = RadarStatus(kb, s, c["radar_timeout_s"])
                self.range_pair[s.id], self.range_pair[kb] = kb, s.id
        self.tracks = {}
        self.next_tid = 1
        self.clutter = {}               # radar id -> [plain, doppler] per rev
        self.cal = {}                   # radar id -> deque of innovation samples
        self._radar_mode = False        # a radar is fusable for this item
        self._summary = None            # built by the owner, read by anyone
        self.last_error = None
        self.stats = collections.Counter()
        self._merge_count = collections.Counter()
        self._heap = []
        self._seq = itertools.count()
        self._cv = threading.Condition()
        self._t_now = None

    # ---- producers (any thread) ----------------------------------------
    def _push(self, t, kind, payload, now):
        with self._cv:
            heapq.heappush(self._heap, (float(t), next(self._seq), now, kind, payload))
            self._cv.notify()

    def submit_fix(self, t, fix, meta=None, now=None):
        self._push(t, "fix", (fix, meta or {}), time.time() if now is None else now)

    def submit_tight(self, t, pos_a, pos_b, tdoa, R, var=None, now=None):
        """var: the per-pair variances the track choice is normalised by
        (defaults to diag(R))."""
        self._push(t, "tight", (pos_a, pos_b, tdoa, R, var),
                   time.time() if now is None else now)

    def radar_status(self, rid):
        st = self.status.get(rid)
        if st is None:
            # On the bus but not in the site config: reported, never fused.
            st = self.status[rid] = RadarStatus(rid, None, self.cfg["radar_timeout_s"])
        return st

    def on_radar_health(self, h, now=None):
        now = time.time() if now is None else now
        self.radar_status(radar_key(h)).on_health(h, now)

    def inner_owned(self, rid, now):
        """Slant range inside which the OTHER range of this radar owns the
        picture, or 0.

        One echo seen on both ranges is one measurement of one target
        through one antenna at one instant, not two independent ones: fused
        twice, the track would claim twice the confidence it has. So the
        shorter range -- finer cells, better range -- owns everything inside
        its own scale while it is fusable, and the longer range contributes
        only beyond it. Equal scales: range A owns."""
        other = self.range_pair.get(rid)
        if other is None:
            return 0.0
        so, me = self.status.get(other), self.status.get(rid)
        if so is None or not so.fusable(now) or not so.range_scale_m:
            return 0.0
        mine = me.range_scale_m if me is not None else None
        if (mine is None or so.range_scale_m < mine
                or (so.range_scale_m == mine and ":" not in other)):
            return float(so.range_scale_m)
        return 0.0

    def submit_batch(self, batch, now=None):
        """A plot batch from radarnode. Updates the radar's status and, if
        the radar is usable, queues the batch. Returns True if queued."""
        now = time.time() if now is None else now
        rid = radar_key(batch)
        st = self.radar_status(rid)
        st.on_batch(batch, now)
        # Counted on the radar's status, not in self.stats: this runs on the
        # bus thread, and a first-time key in a dict the owner is iterating
        # raised out of build_summary (follow-up review).
        if batch.get("az_ref", "antenna") != "antenna":
            st.batches_bad_frame += 1
            return False
        if not st.fusable(now):
            st.batches_unfused += 1
            st.plots_unfused += len(batch.get("plots") or [])
            return False
        plots = batch.get("plots") or []
        # Keyed on the LATEST plot. A fix measured between two plots of one
        # batch then pops first and is applied on time; the earlier plot is
        # the late one, and update_radar retrodicts the track to the plot's
        # time for it (review A5: keyed on the earliest plot, the precise
        # fix was the one applied late).
        t = max([p["t_utc_ns"] for p in plots], default=batch["t_utc_ns"]) * 1e-9
        self._push(t, "batch", (rid, batch), now)
        return True

    # ---- scheduling ------------------------------------------------------
    def radar_live(self, now):
        return any(s.fusable(now) for s in list(self.status.values()))

    def lag(self, now):
        return float(self.cfg["lag_s"]) if self.radar_live(now) else 0.0

    def pending(self):
        with self._cv:
            return len(self._heap)

    def pop_due(self, now):
        """Items ready at wall time now, oldest measurement first."""
        lag = self.lag(now)
        hold = lag + 0.5
        out = []
        with self._cv:
            while self._heap:
                t, seq, arrived, kind, payload = self._heap[0]
                # lag 0: everything, regardless of t (a node clock slightly
                # ahead of the host must not hold a fix back)...
                if lag > 0.0 and t > now - lag and arrived > now - hold:
                    break
                heapq.heappop(self._heap)
                out.append((seq, t, kind, payload))
        if lag == 0.0:
            # ...and in ARRIVAL order, which is what the old track_lock
            # produced: with no radar the track layer is the old one,
            # ordering included.
            out.sort(key=lambda e: e[0])
        return [(t, kind, payload) for _, t, kind, payload in out]

    def wait(self, now, max_wait=0.25):
        """Block until something may be due (or max_wait)."""
        with self._cv:
            if not self._heap:
                self._cv.wait(timeout=max_wait)
                return
            lag = self.lag(now)
            if lag == 0.0:
                return
            t, _, arrived, _, _ = self._heap[0]
            dt = min(t - (now - lag), arrived - (now - lag - 0.5), max_wait)
            if dt > 0:
                self._cv.wait(timeout=dt)

    def process_due(self, now):
        """Apply everything due; returns output events in order. One bad
        item costs only itself: it is counted and logged, and the items
        popped with it -- fixes included -- are still applied (review A3)."""
        outs = []
        for t, kind, payload in self.pop_due(now):
            try:
                outs.extend(self.process(t, kind, payload, now))
            except Exception as e:          # noqa: BLE001 -- see docstring
                self.stats["item_errors"] += 1
                self.stats["item_errors_" + kind] += 1
                self.last_error = f"{kind} at {t:.3f}: {type(e).__name__}: {e}"
        self.build_summary(now)
        return outs

    def process(self, t, kind, payload, now=None):
        """Apply one item. Single-threaded by contract: only the fusion
        owner calls this."""
        self._t_now = t if self._t_now is None else max(self._t_now, t)
        # Decided per item: is the system fused right now, or TDOA-only?
        # TDOA-only means the pre-fusion track layer, rule for rule.
        self._radar_mode = self.radar_live(time.time() if now is None else now)
        if kind == "fix":
            outs = self._on_fix(t, *payload)
        elif kind == "tight":
            outs = self._on_tight(t, *payload)
        elif kind == "batch":
            outs = self._on_batch(t, *payload, now=now)
        else:
            outs = []
        self._housekeeping(self._t_now, now)
        return outs

    # ---- TDOA ------------------------------------------------------------
    def _new_track(self, kf_factory, state, src, t, alt_src):
        tid = self.next_tid
        self.next_tid += 1
        trk = FusedTrack(kf_factory(tid), state, src, t, alt_src)
        self.tracks[tid] = trk
        return trk

    def _radar_separable(self, trk, p, t):
        """Would the radar have seen position p as a different object from
        track trk? Tested in the radar's own measurement space with the
        covariance of the last plot that hit the track. No radar evidence
        -> treat as separable."""
        lr = getattr(trk, "last_plot", None)
        if lr is None or lr[0] not in self.sites:
            return True
        site, R = self.sites[lr[0]], lr[1]
        x, P = trk.kf.predicted(t)
        hx, H = radar_measurement(x, site.enu, site.psi)
        hp, _ = radar_measurement(np.concatenate([p, x[3:]]), site.enu, site.psi)
        y = np.array([hp[0] - hx[0], wrap_pi(hp[1] - hx[1])])
        S = H @ P @ H.T + 2.0 * R
        try:
            return float(y @ np.linalg.solve(S, y)) > c_gate(self.cfg)
        except np.linalg.LinAlgError:
            return True

    def _alt_src_after_fix(self, trk, fix):
        good_tdoa = fix.sigma_v < 0.5 * self.alt_prior_sigma_m
        trk.last_fix_sigma_v = fix.sigma_v
        if good_tdoa:
            return "tdoa"
        if trk.n_radar and math.sqrt(max(trk.kf.P[2, 2], 0.0)) < 0.5 * self.alt_prior_sigma_m:
            return "radar"
        return "prior"

    def _on_fix(self, t, fix, meta):
        self.stats["tdoa_fixes"] += 1
        if not self._radar_mode:
            return self._on_fix_legacy(t, fix, meta)
        return self._on_fix_fused(t, fix, meta)

    def _on_fix_legacy(self, t, fix, meta):
        """No radar usable: the pre-fusion association, rule for rule --
        nearest track (current state, Euclidean) within legacy_gate_m,
        else a new confirmed track. Kept because with TDOA alone it is the
        intended behaviour, not merely the old one: there is no second
        sensor to say a gate failure is the fix's fault rather than the
        track's, and validate_fusion (f0) checks it bit for bit against a
        copy of the old code, two targets and reordered arrivals included."""
        conf = [k for k in self.tracks.values()
                if not k.dead and k.state == "confirmed"]
        near = min(conf, key=lambda k: float(np.linalg.norm(k.kf.pos - fix.enu))) \
            if conf else None
        if near is not None and \
                float(np.linalg.norm(near.kf.pos - fix.enu)) <= self.cfg["legacy_gate_m"]:
            self.stats["tdoa_assoc_legacy"] += 1
            return self._apply_fix(near, t, fix, meta)
        return self._new_fix_track(t, fix, meta)

    def _new_fix_track(self, t, fix, meta):
        trk = self._new_track(lambda tid: Tracker(tid, fix, t), "confirmed",
                              ["tdoa"], t, "tdoa")
        trk.n_tdoa = 1
        trk.last_tdoa_t = t
        trk.last_update_t = t
        trk.alt_src = self._alt_src_after_fix(trk, fix)
        self.stats["tdoa_new_tracks"] += 1
        return [self._out("tdoa", trk, t, meta=meta, fix=fix)]

    def _apply_fix(self, trk, t, fix, meta):
        late0 = trk.kf.late
        trk.kf.update(fix, t, alt_prior=self.alt_prior_m,
                      alt_sigma=self.alt_prior_sigma_m,
                      retrodict=self._radar_mode)
        trk.reject_streak = 0
        if trk.kf.late > late0:
            self.stats["late_updates"] += 1
        trk.n_tdoa += 1
        trk.last_tdoa_t = t
        trk.last_update_t = max(trk.last_update_t, t)
        trk.src.add("tdoa")
        trk.misses = 0
        if trk.state == "tentative" and (not trk.tentative_tdoa
                                         or trk.n_tdoa >= self.cfg["tdoa_confirm_n"]):
            # Corroborated: a radar-only tentative a fix agrees with, or a
            # hidden TDOA tentative whose tdoa_confirm_n fixes agree.
            resumed = self._resume_stale(trk, t) if trk.tentative_tdoa else None
            if resumed is not None:
                trk = resumed
            else:
                trk.state = "confirmed"
                trk.tentative_tdoa = False
                self.stats["confirmed_by_tdoa"] += 1
        if trk.state == "tentative":
            return []                       # hidden: not published yet
        trk.alt_src = self._alt_src_after_fix(trk, fix)
        outs = [self._out("tdoa", trk, t, meta=meta, fix=fix)]
        self._check_merge(trk, t)
        return outs

    def _stale(self, k, t):
        """Fused mode: a confirmed track the radar has not established that
        has had no update for coast_s. Resumable, not updatable."""
        return (k.state == "confirmed" and k.n_radar < self.cfg["radar_min_hits"]
                and t - k.last_update_t > self.cfg["coast_s"])

    def _resume_stale(self, tent, t):
        """A hidden tentative just reached tdoa_confirm_n consistent fixes.
        If it sits inside the gate of a stale track, that track's drone has
        resumed: hand it the tentative's state and keep the old id."""
        x1, P1 = tent.kf.predicted(t)
        best, best_cost = None, np.inf
        for k in self.tracks.values():
            if k is tent or k.dead or not self._stale(k, t):
                continue
            x2, P2 = k.kf.predicted(t)
            d = x1[:3] - x2[:3]
            s_ = P1[:3, :3] + P2[:3, :3]
            try:
                d2 = float(d @ np.linalg.solve(s_, d))
                cost = d2 + float(np.log(max(np.linalg.det(s_), 1e-300)))
            except np.linalg.LinAlgError:
                continue
            if d2 <= self.cfg["gate_tdoa"] and cost < best_cost:
                best, best_cost = k, cost
        if best is None:
            return None
        best.kf = tent.kf
        best.kf.tid = best.tid
        best.n_tdoa += tent.n_tdoa
        best.last_tdoa_t = tent.last_tdoa_t
        best.last_update_t = max(best.last_update_t, tent.last_update_t)
        best.last_fix_sigma_v = tent.last_fix_sigma_v
        self._kill(tent, "tdoa_resumed")
        return best

    def _hold(self, t, fix, meta, near, kind, stat):
        """Not applied: found (or feed) a hidden tentative unless the fix is
        below the ground. Returns the rejected/held output."""
        c = self.cfg
        self.stats[stat] += 1
        if fix.enu[2] >= c["min_alt_m"]:
            k = self._new_track(lambda tid: Tracker(tid, fix, t),
                                "tentative", ["tdoa"], t, "tdoa")
            k.tentative_tdoa = True
            k.n_tdoa = 1
            k.last_tdoa_t = t
        else:
            self.stats["tdoa_held_below_ground"] += 1
        return [{"kind": kind, "t": t, "meta": meta, "fix": fix,
                 "near_tid": near.tid,
                 "dist_m": float(np.linalg.norm(near.kf.pos - fix.enu))}]

    def _on_fix_fused(self, t, fix, meta):
        c = self.cfg
        live = [k for k in self.tracks.values() if not k.dead]

        def score(k):
            x, P = k.kf.predicted(t)
            s_ = P[:3, :3] + fix.cov
            y = fix.enu - x[:3]
            try:
                d2 = float(y @ np.linalg.solve(s_, y))
                return d2, d2 + float(np.log(max(np.linalg.det(s_), 1e-300)))
            except np.linalg.LinAlgError:
                return np.inf, np.inf

        # Gate on d2; among the tracks that pass, pick by d2 + ln|S| (the
        # negative log-likelihood). Plain d2 favours the vaguest track: a
        # radar-only tentative with 100 m of altitude uncertainty "fits"
        # every fix better than the precise track it duplicates. Confirmed
        # tracks first: a fix that a radar-supported track rejects must not
        # then be handed to a clutter tentative that happens to sit nearby.
        scored = {k.tid: score(k) for k in live}
        conf = [k for k in live
                if k.state == "confirmed" and not self._stale(k, t)
                and scored[k.tid][0] <= c["gate_tdoa"]]
        if conf:
            self.stats["tdoa_assoc_gate"] += 1
            return self._apply_fix(min(conf, key=lambda k: scored[k.tid][1]), t, fix, meta)

        # A fix below the ground never founds or confirms anything: that is
        # the mirror image of a real target, not a second one (review A4).
        above = fix.enu[2] >= c["min_alt_m"]
        tent = [k for k in live if k.state == "tentative" and above
                and scored[k.tid][0] <= c["gate_tdoa"]]
        best_tent = min(tent, key=lambda k: scored[k.tid][1]) if tent else None

        # The confirmed tracks this fix could belong to, and the likeliest
        # one by Mahalanobis distance -- not Euclidean, or a radar-only bird
        # track at the altitude prior can be "nearer" than the drone's own.
        cands = [k for k in live if k.state == "confirmed"
                 and float(np.linalg.norm(k.kf.pos - fix.enu)) <= c["legacy_gate_m"]]
        near = min(cands, key=lambda k: scored[k.tid][0]) if cands else None
        if near is not None and near.radar_supported(t, c["radar_support_s"], c["radar_min_hits"]):
            # Independent evidence says the track is right and the fix is not.
            supported = [k for k in cands if k.radar_supported(t, c["radar_support_s"], c["radar_min_hits"])]
            same = [k for k in supported if not self._radar_separable(k, fix.enu, t)]
            if best_tent is not None and best_tent.tentative_tdoa:
                # a further consistent fix of a hidden tentative: a second
                # emitter, possibly inside the radar's beam (follow-up
                # review: B 50 m cross-range from A at 1 km used to be
                # rejected as "same object" forever)
                self.stats["tdoa_assoc_tentative"] += 1
                return self._apply_fix(best_tent, t, fix, meta)
            # A track that keeps refusing mutually consistent fixes nobody
            # else takes is more likely wrong than they are: after
            # reject_reset of them, re-seed it from the fix, keep its id.
            lr = near.last_rejected
            consistent = (lr is not None and t - lr[0] <= c["tdoa_confirm_s"]
                          and float(np.linalg.norm(fix.enu - lr[1]))
                          <= c["reject_consistent_m"] + 40.0 * (t - lr[0]))
            near.reject_streak = near.reject_streak + 1 if consistent else 1
            near.last_rejected = (t, fix.enu.copy())
            if above and near.reject_streak >= c["reject_reset"]:
                self.stats["tdoa_track_reset"] += 1
                kf_old = near.kf
                near.kf = Tracker(near.tid, fix, t)
                near.kf.max_step_s = kf_old.max_step_s
                near.kf.late = kf_old.late
                near.n_tdoa += 1
                near.last_tdoa_t = t
                near.last_update_t = max(near.last_update_t, t)
                near.reject_streak = 0
                near.alt_src = self._alt_src_after_fix(near, fix)
                return [self._out("tdoa", near, t, meta=meta, fix=fix)]
            self.stats["tdoa_rejected_m"] = round(float(np.linalg.norm(near.kf.pos - fix.enu)), 1)
            if same:
                # Within the radar's resolution of a supported track. Only
                # suppressed outright when below the ground (the mirror
                # image); above it, it may be a second emitter in the beam,
                # so it founds a hidden tentative like any other.
                self.stats["tdoa_rejected_same_object"] += 1
            out = self._hold(t, fix, meta, near, "rejected", "tdoa_rejected")
            out[0]["same_object"] = bool(same)
            return out
        if best_tent is not None:
            self.stats["tdoa_assoc_tentative"] += 1
            return self._apply_fix(best_tent, t, fix, meta)
        if near is not None:
            # Outside the gate of every confirmed track, none of them backed
            # by the radar (or the nearest is stale). The pre-fusion rule
            # would update the nearest anyway; with the radar fused this is
            # the multi-target mode, and there that rule lets one track eat
            # two drones 150 m apart (validate_fusion (h)). So: a hidden
            # tentative, confirmed by tdoa_confirm_n consistent fixes (a
            # second drone, or a paused one resuming its old id) and dead
            # alone in tdoa_confirm_s (an outlier, never applied).
            return self._hold(t, fix, meta, near, "held", "tdoa_gate_fail")
        return self._new_fix_track(t, fix, meta)

    def _on_tight(self, t, pos_a, pos_b, tdoa, R, var=None):
        """Starved burst: the rows go to the confirmed track they fit best
        (smallest normalised TDOA residual), innovation-gated inside
        update_tdoa. Tentative tracks are excluded -- a hyperboloid passes
        near plenty of clutter tracks. No output, as before fusion."""
        live = [k for k in self.tracks.values()
                if not k.dead and k.state == "confirmed"]
        if not live:
            return []
        if var is None:
            var = np.clip(np.diag(np.asarray(R, dtype=float)), 1e-30, None)

        def misfit(k):
            p = k.kf.pos
            pred = (np.linalg.norm(p - pos_a, axis=1)
                    - np.linalg.norm(p - pos_b, axis=1)) / C_LIGHT
            return float(np.sum((tdoa - pred) ** 2 / var))

        trk = min(live, key=misfit)
        ok = trk.kf.update_tdoa(pos_a, pos_b, tdoa, R, t)
        self.stats["tight_updates" if ok else "tight_rejected"] += 1
        if ok:
            trk.n_tight += 1
            trk.last_update_t = max(trk.last_update_t, t)
        return []

    # ---- radar -----------------------------------------------------------
    def _plot_R(self, p, x, site):
        sr = max(float(p.get("sig_rng_m") or 1.0), 0.05)
        sa = math.radians(max(float(p.get("sig_az_deg") or 0.3), 0.3))
        R = np.diag([sr ** 2, sa ** 2])
        st = p.get("t_sigma_ns")
        st = self.cfg["latency_sigma_s"] if st is None else float(st) * 1e-9
        if st > 0:
            g = radar_rates(x, site.enu)
            R = R + np.outer(g, g) * st ** 2
        return R

    def _seed_from_plot(self, p, site, t):
        c = self.cfg
        r = float(p["rng_m"])
        th = math.radians(float(p["az_deg"])) + site.psi
        o = site.enu
        lim = r * math.sin(site.fov_elev)
        du = float(np.clip(self.seed_alt_m - o[2], -lim, lim))
        du = max(du, -o[2])                     # not underground
        du = float(np.clip(du, -0.999 * r, 0.999 * r))
        rho = math.sqrt(max(r * r - du * du, 1e-6))
        pos = o + np.array([rho * math.sin(th), rho * math.cos(th), du])
        sr = max(float(p.get("sig_rng_m") or 1.0), 0.05)
        sa = math.radians(max(float(p.get("sig_az_deg") or 0.3), 0.3))
        su = max(c["seed_alt_sigma_frac"] * r, c["seed_alt_sigma_min_m"])
        J = np.array([
            [r / rho * math.sin(th), rho * math.cos(th), -du / rho * math.sin(th)],
            [r / rho * math.cos(th), -rho * math.sin(th), -du / rho * math.cos(th)],
            [0.0, 0.0, 1.0]])
        cov = J @ np.diag([sr ** 2, sa ** 2, su ** 2]) @ J.T
        P = np.zeros((6, 6))
        P[:3, :3] = cov
        sv, sz = c["seed_vel_sigma_mps"], c["seed_vz_sigma_mps"]
        P[3:, 3:] = np.diag([sv ** 2, sv ** 2, sz ** 2])
        x = np.concatenate([pos, np.zeros(3)])
        return x, P

    def plot_enu(self, p, site, alt=None):
        """Display position of a plot: on the associated track's altitude
        if there is one, else on the seed altitude."""
        r = float(p["rng_m"])
        th = math.radians(float(p["az_deg"])) + site.psi
        o = site.enu
        if alt is None:
            lim = r * math.sin(site.fov_elev)
            du = float(np.clip(self.seed_alt_m - o[2], -lim, lim))
            du = max(du, -o[2])
        else:
            du = float(alt) - o[2]
        du = float(np.clip(du, -0.999 * r, 0.999 * r))
        rho = math.sqrt(max(r * r - du * du, 0.0))
        return [float(o[0] + rho * math.sin(th)), float(o[1] + rho * math.cos(th)),
                float(o[2] + du)]

    def _visible(self, site, x, range_scale, inner_m=0.0):
        c = self.cfg
        r = site.slant(x)
        # inner_m: the other range of this radar owns the picture inside it
        # (inner_owned), so this range's silence there is not a miss
        if r < max(c["min_range_m"], inner_m):
            return False
        if range_scale and r > float(range_scale):
            return False
        return abs(site.elevation(x)) <= site.fov_elev

    def _on_batch(self, t0, rid, batch, now=None):
        c = self.cfg
        site = self.sites[rid]
        psi = site.psi
        plots = sorted(batch.get("plots") or [], key=lambda p: p["t_utc_ns"])
        inner = self.inner_owned(rid, time.time() if now is None else now)
        if inner:
            n0 = len(plots)
            plots = [p for p in plots if float(p["rng_m"]) > inner]
            self.stats["radar_inner_dropped"] += n0 - len(plots)
        rev = int(batch.get("rev", 0))
        sector = int(batch.get("sector", 0))
        n_sec = int(batch.get("n_sectors", 16))
        st = self.status.get(rid)
        t_rev = (batch.get("t_rev_s")
                 or (60.0 / batch["rpm_measured"] if batch.get("rpm_measured") else None)
                 or (st.t_rev_s if st else None) or 2.5)
        # "Hit on this pass": a plot for this track within half a revolution.
        # Time-based on purpose -- the revolution counter on the bus is the
        # one at finalisation, which for the last sectors of a turn is
        # already the next revolution's.
        half = 0.5 * float(t_rev)
        scale = batch.get("range_scale_m")
        # Candidates for this batch's plots. A track the radar has not
        # established (fewer than radar_min_hits plots) takes a plot only
        # while TDOA is holding it (updated within coast_s): a silent TDOA-only track's prediction grows to hundreds
        # of metres in seconds and would take the first clutter plot in it,
        # which then made it "radar-supported" and let it refuse the
        # drone's own fixes when it resumed (validate_fusion (h)).
        t_first = plots[0]["t_utc_ns"] * 1e-9 if plots else 0.0
        live = [k for k in self.tracks.values()
                if not k.dead and not k.tentative_tdoa
                and (k.n_radar >= c["radar_min_hits"] or k.state == "tentative"
                     or t_first - k.last_update_t <= c["coast_s"])]
        self.stats["radar_batches"] += 1
        self.stats["radar_plots"] += len(plots)
        outs = []

        # --- GNN in measurement space ----------------------------------
        assign = {}
        gate_info = {}
        if plots and live:
            big = 1e9
            cost = np.full((len(plots), len(live)), big)
            for j, p in enumerate(plots):
                tj = p["t_utc_ns"] * 1e-9
                z = np.array([float(p["rng_m"]), wrap_pi(math.radians(float(p["az_deg"])))])
                # Clutter-aware gate: the plot goes to a track only if it is
                # likelier to be that track's than clutter,
                # d2 + ln|2 pi S| <= 2 ln(Pd / beta). Binds for a track
                # coasting through the blind cone, whose 9.21 gate has
                # grown to a kilometre and would otherwise take the first
                # false plot it sees; a well-held track never notices it.
                lim = 2.0 * math.log(c["radar_pd"] / self._clutter_beta(
                    rid, p.get("dop") in ("app", "rec"), scale))
                for i, trk in enumerate(live):
                    x, P = trk.kf.state_at(tj)
                    h, H = radar_measurement(x, site.enu, psi)
                    y = np.array([z[0] - h[0], wrap_pi(z[1] - h[1])])
                    S = H @ P @ H.T + self._plot_R(p, x, site)
                    try:
                        d2 = float(y @ np.linalg.solve(S, y))
                        ld = float(np.log(max(np.linalg.det(S), 1e-300)))
                    except np.linalg.LinAlgError:
                        continue
                    if d2 <= c["gate_radar"] and d2 + ld + 2.0 * math.log(2.0 * math.pi) <= lim:
                        cost[j, i] = d2 + ld
                        gate_info[(j, i)] = (d2, S)
                    elif d2 <= c["gate_radar"]:
                        self.stats["radar_gate_clutter"] += 1
            if np.any(cost < big):
                from scipy.optimize import linear_sum_assignment
                rows, cols = linear_sum_assignment(cost)
                for j, i in zip(rows, cols):
                    if cost[j, i] < big:
                        assign[j] = (live[i], gate_info[(j, i)])

        display = []
        updated = set()
        merge_after = []
        n_unassoc = [0, 0]
        for j, p in enumerate(plots):
            tj = p["t_utc_ns"] * 1e-9
            trk, gi = assign.get(j, (None, None))
            flagged = p.get("dop") in ("app", "rec")
            if trk is not None:
                x, _ = trk.kf.state_at(tj)
                R = self._plot_R(p, x, site)
                x_P_v = float(trk.kf.P[2, 2])
                pinned = (trk.last_tdoa_t is not None
                          and abs(tj - trk.last_tdoa_t) <= c["cal_fix_age_s"]
                          and math.sqrt(max(trk.kf.P[0, 0] + trk.kf.P[1, 1], 0.0))
                          <= c["cal_sigma_h_m"])
                late0 = trk.kf.late
                ok, nis = trk.kf.update_radar(float(p["rng_m"]),
                                              math.radians(float(p["az_deg"])),
                                              R, site.enu, psi, tj)
                if trk.kf.late > late0:
                    self.stats["late_updates"] += 1
                if ok:
                    if pinned:
                        self._cal_sample(rid, p, x, site, trk.kf.last_innov,
                                         x_P_v <= c["cal_sigma_v_m"] ** 2)
                    trk.last_plot = (rid, R)
                    trk.kf.max_step_s = c["coast_step_s"]
                    if (trk.n_tdoa == 0
                            and trk.kf.P[2, 2] > self.alt_prior_sigma_m ** 2):
                        # Radar-only: nothing measures the altitude, and the
                        # process noise lets it random-walk until slant
                        # range trades it against horizontal position. Put
                        # the prior back -- but only once the vertical is
                        # vaguer than the prior itself. Re-applying it on
                        # every plot counted the same information again
                        # each time and shrank sigma_U to ~35 m against a
                        # 60 m prior (review); this way it stays between
                        # ~0.7 and 1 prior sigma, which is the truth.
                        trk.kf.apply_alt_prior(self.seed_alt_m, self.alt_prior_sigma_m)
                    self.stats["radar_assoc"] += 1
                    self.stats["radar_nis_sum"] += nis
                    trk.n_radar += 1
                    trk.last_radar_t = tj if trk.last_radar_t is None else max(trk.last_radar_t, tj)
                    trk.last_update_t = max(trk.last_update_t, tj)
                    trk.src.add("radar")
                    trk.misses = 0
                    prev_hit = trk.last_scan.get(rid)
                    if prev_hit is None or tj - prev_hit > half:
                        if trk.state == "tentative":
                            trk.scans.append(True)
                    trk.last_scan[rid] = tj
                    if flagged:
                        trk.dop_hits += 1
                    if trk.state == "tentative":
                        trk.llr += self._llr_hit(rid, gi[0], gi[1], flagged, scale)
                    self._maybe_confirm(trk)
                    if trk.state == "confirmed":
                        trk.alt_src = self._alt_src_radar(trk)
                        outs.append(self._out("radar", trk, tj, plot=p, radar=rid))
                    updated.add(trk.tid)
                    display.append((p, trk.tid, float(trk.kf.pos[2]), None))
                    merge_after.append((trk, tj))
                    continue
                # Gated at the prediction but refused by the update itself:
                # happens for a late plot, which update_radar retrodicts.
                self.stats["radar_update_refused"] += 1
            # unassociated: a tentative radar-only track
            x, P = self._seed_from_plot(p, site, tj)
            k = self._new_track(lambda tid: Tracker.from_state(tid, x, P, tj),
                                "tentative", ["radar"], tj, "prior")
            k.kf.max_step_s = c["coast_step_s"]
            k.n_radar = 1
            k.last_radar_t = tj
            k.scans = [True]
            k.dop_hits = 1 if flagged else 0
            k.last_plot = (rid, self._plot_R(p, x, site))
            n_unassoc[1 if flagged else 0] += 1
            k.last_scan[rid] = tj
            self.stats["radar_new_tentative"] += 1
            display.append((p, None, None, k.tid))

        # Merges only after every plot of the batch has been applied: a merge
        # inside the loop could kill a track a later plot is assigned to.
        for trk, tj in merge_after:
            if not trk.dead:
                self._check_merge(trk, tj)

        # --- scan bookkeeping: the sector the antenna just left ------------
        # A track predicted in sector s-1 has had its chance this revolution
        # (its plot, if any, was in batch s-2, s-1 or this one).
        prev = (sector - 1) % n_sec
        width = 2.0 * math.pi / n_sec
        t_b = batch["t_utc_ns"] * 1e-9
        for trk in list(self.tracks.values()):
            if trk.dead or trk.tentative_tdoa or trk.tid in updated:
                continue
            if trk.n_radar == 0 or (trk.state != "tentative"
                                    and trk.n_radar < c["radar_min_hits"]):
                # Never (or barely) seen by this radar: its silence about the
                # track says nothing (an emitter below the radar's
                # sensitivity), and a TDOA-only track was never deleted
                # before fusion either.
                continue
            x, P = trk.kf.predicted(t_b)
            h, _ = radar_measurement(x, site.enu, psi)
            a = h[1] % (2.0 * math.pi)
            if int(a // width) % n_sec != prev:
                continue
            hit = trk.last_scan.get(rid)
            if hit is not None and t_b - hit < half:
                continue                      # hit on this pass already
            if not self._visible(site, x, scale, inner):
                self.stats["radar_miss_fov"] += 1
                if trk.state == "confirmed" and t_b - trk.last_update_t > c["coast_s"]:
                    outs.append(self._out("coast", trk, t_b, radar=rid))
                continue
            self.stats["radar_miss"] += 1
            trk.misses += 1
            if trk.state == "tentative":
                trk.scans.append(False)
                trk.llr += math.log(max(1.0 - c["radar_pd"], 1e-3))
                if self._tentative_failed(trk):
                    self._kill(trk, "tentative_dropped")
                continue
            if (trk.misses >= c["delete_misses"]
                    and (trk.last_tdoa_t is None
                         or t_b - trk.last_tdoa_t > c["tdoa_hold_s"])):
                self._kill(trk, "deleted_misses")
                continue
            if t_b - trk.last_update_t > c["coast_s"]:
                outs.append(self._out("coast", trk, t_b, radar=rid))

        # clutter density: unassociated plots per revolution, EMA over ~10
        # revolutions (16 batches each)
        cl = self.clutter.setdefault(rid, [c["clutter_per_rev_init"],
                                           c["clutter_dop_per_rev_min"]])
        a = 1.0 / (10.0 * n_sec)
        cl[0] += a * (n_unassoc[0] * n_sec - cl[0])
        cl[1] += a * (n_unassoc[1] * n_sec - cl[1])

        if display:
            outs.append({"kind": "plots", "t": t0, "radar": rid,
                         "sector": sector, "rev": rev,
                         "plots": [{"enu": self.plot_enu(p, site, alt),
                                    "tid": tid, "seed_tid": seed, "dop": p.get("dop"),
                                    "rng_m": float(p["rng_m"]),
                                    "az_true_deg": (float(p["az_deg"])
                                                    + site.az_offset_deg) % 360.0,
                                    "t": p["t_utc_ns"] * 1e-9,
                                    "sim_label": p.get("sim_label")}
                                   for p, tid, alt, seed in display]})
        return outs

    def _alt_src_radar(self, trk):
        if (trk.last_fix_sigma_v is not None and trk.last_tdoa_t is not None
                and trk.last_fix_sigma_v < 0.5 * self.alt_prior_sigma_m):
            return "tdoa"
        if trk.n_tdoa and math.sqrt(max(trk.kf.P[2, 2], 0.0)) < 0.5 * self.alt_prior_sigma_m:
            return "radar"
        return trk.alt_src if trk.alt_src in ("tdoa", "radar") and trk.n_tdoa else "prior"

    def _clutter_beta(self, rid, flagged, scale):
        """False-plot density in (range, azimuth) space, per m per rad."""
        c = self.cfg
        cl = self.clutter.get(rid, [c["clutter_per_rev_init"],
                                    c["clutter_dop_per_rev_min"]])
        if flagged:
            rate = max(cl[1], c["clutter_dop_per_rev_min"])
        else:
            rate = max(cl[0], c["clutter_per_rev_min"])
        area = max(float(scale or 3000.0), 100.0) * 2.0 * math.pi   # m * rad
        return rate / area

    def _llr_hit(self, rid, d2, S, flagged, scale):
        """Score increment of one associated plot: log of (target
        likelihood / clutter density) in (range, azimuth) space."""
        c = self.cfg
        beta = self._clutter_beta(rid, flagged, scale)
        det = max(float(np.linalg.det(2.0 * math.pi * S)), 1e-300)
        return (math.log(c["radar_pd"]) - math.log(beta)
                - 0.5 * math.log(det) - 0.5 * d2)

    def _maybe_confirm(self, trk):
        if trk.state != "tentative":
            return
        c = self.cfg
        if trk.tentative_tdoa:
            return
        hits = sum(trk.scans[:c["confirm_n"]])
        if hits >= c["confirm_m"]:
            ok = True
        elif trk.dop_hits >= c["confirm_m_doppler"]:
            ok = sum(trk.scans[:c["confirm_n_doppler"]]) >= c["confirm_m_doppler"]
        else:
            ok = False
        if ok and trk.llr >= c["confirm_llr"]:
            trk.state = "confirmed"
            self.stats["radar_confirmed"] += 1

    def _tentative_failed(self, trk):
        c = self.cfg
        misses = len(trk.scans) - sum(trk.scans)
        # can it still reach M of N (or the Doppler rule)?
        if misses <= c["confirm_n"] - c["confirm_m"]:
            return False
        return True

    # ---- lifecycle ---------------------------------------------------------
    def _kill(self, trk, why):
        trk.dead = True
        self.stats[why] += 1
        self.tracks.pop(trk.tid, None)
        for key in [k for k in self._merge_count if trk.tid in k]:
            del self._merge_count[key]

    def _housekeeping(self, t, now):
        c = self.cfg
        for trk in list(self.tracks.values()):
            if trk.dead:
                continue
            age = t - trk.last_update_t
            if trk.tentative_tdoa:
                if age > c["tdoa_confirm_s"]:
                    self._kill(trk, "tdoa_tentative_dropped")
                continue
            if trk.state == "tentative":
                if age > c["tentative_timeout_s"]:
                    self._kill(trk, "tentative_timeout")
                elif (now is not None and trk.last_plot is not None
                      and trk.last_plot[0] in self.status
                      and not self.status[trk.last_plot[0]].fusable(now)):
                    # Its radar is gone: nothing can confirm it the way it
                    # was meant to be confirmed, and a stray TDOA fix must
                    # not do it instead (that is how a ghost is born).
                    self._kill(trk, "tentative_radar_lost")
                continue
            if trk.n_radar < c["radar_min_hits"]:
                # Not established by the radar. TDOA-only mode never deletes
                # it (pre-fusion parity); fused mode does after tdoa_coast_s
                # of silence -- a ghost confirmed by a lost peak must not
                # live forever, and a paused drone that resumes within that
                # time gets its id back through _resume_stale.
                if self._radar_mode and age > c["tdoa_coast_s"]:
                    self._kill(trk, "deleted_tdoa_coast")
                continue
            _, P = trk.kf.predicted(t)
            if math.sqrt(max(P[0, 0] + P[1, 1], 0.0)) > c["max_sigma_h_m"]:
                self._kill(trk, "deleted_sigma")

    _TRANSFER = ("kf", "src", "n_tdoa", "n_radar", "n_tight", "last_tdoa_t",
                 "last_radar_t", "last_update_t", "alt_src", "last_fix_sigma_v",
                 "misses", "last_scan", "last_plot")

    def _check_merge(self, trk, t):
        """Two tracks on ONE target: merge once their full-state Mahalanobis
        distance has stayed inside the gate for merge_hits updates running.

        Only while the radar is fused (before fusion nothing merged), only
        between tracks that are both being measured now (updated within
        coast_s), both well determined horizontally (merge_sigma_h_m) and
        actually close (merge_dist_m horizontally). Without those three, a
        coasting track's inflated P passes the gate against anything: two
        TDOA drones 100 m apart merged after 4 s of silence, 5 km apart
        after 21 s, and the survivor then oscillated between them (review
        A1). The older id survives; it takes the better-determined state
        AND that state's bookkeeping, never a union -- otherwise a
        TDOA-only track inherits a bird's radar support."""
        c = self.cfg
        if not self._radar_mode or trk.state != "confirmed" or trk.dead:
            return
        if t - trk.last_update_t > c["coast_s"]:
            return
        x1, P1 = trk.kf.predicted(t)
        if math.sqrt(max(P1[0, 0] + P1[1, 1], 0.0)) > c["merge_sigma_h_m"]:
            return
        for other in list(self.tracks.values()):
            if other is trk or other.dead or other.state != "confirmed":
                continue
            key = (min(trk.tid, other.tid), max(trk.tid, other.tid))
            x2, P2 = other.kf.predicted(t)
            d = x1 - x2
            close = (t - other.last_update_t <= c["coast_s"]
                     and math.sqrt(max(P2[0, 0] + P2[1, 1], 0.0)) <= c["merge_sigma_h_m"]
                     and math.hypot(d[0], d[1]) <= c["merge_dist_m"])
            try:
                d2 = float(d @ np.linalg.solve(P1 + P2, d)) if close else np.inf
            except np.linalg.LinAlgError:
                d2 = np.inf
            if d2 > c["gate_merge"]:
                self._merge_count.pop(key, None)
                continue
            self._merge_count[key] += 1
            if self._merge_count[key] < c["merge_hits"]:
                continue
            keep, drop = (trk, other) if trk.tid < other.tid else (other, trk)
            if np.linalg.det(drop.kf.P[:3, :3]) < np.linalg.det(keep.kf.P[:3, :3]):
                for a in self._TRANSFER:
                    setattr(keep, a, getattr(drop, a))
                keep.kf.tid = keep.tid
            self._kill(drop, "merged")
            return

    # ---- output ------------------------------------------------------------
    def _out(self, kind, trk, t, **kw):
        o = {"kind": kind, "t": t, "track": trk.snapshot(t, self.cfg["coast_s"])}
        if kind == "coast":
            # A coasting report carries the prediction, not a stale update.
            x, P = trk.kf.predicted(t)
            o["track"].update(x=x, P=P, t_state=float(max(t, trk.kf.t)))
        o.update(kw)
        return o

    def _cal_sample(self, rid, p, x, site, y, range_ok=True):
        """One innovation of a plot on a track TDOA has pinned: what the
        radar said minus where TDOA put the target. Their mean, against the
        target's own bearing and range rates, is the radar's azimuth bias and
        its clock offset relative to the TDOA timebase -- neither of which
        anything else in the system can see. Reported, never applied:
        calibration is an operator decision, not a filter's."""
        if y is None:
            return
        c = self.cfg
        g = radar_rates(x, site.enu)
        sr = max(float(p.get("sig_rng_m") or 1.0), 0.05)
        sa = math.radians(max(float(p.get("sig_az_deg") or 0.3), 0.3))
        dq = self.cal.setdefault(rid, collections.deque(maxlen=int(c["cal_window"])))
        # range innovations are only clean when the vertical is pinned too
        # (slant range carries the altitude error times dU/r): otherwise
        # the sample still counts for azimuth, with no weight in range
        dq.append((float(y[0]), float(y[1]), float(g[0]), float(g[1]),
                   sr * sr if range_ok else float("inf"), sa * sa))

    def _cal_estimate(self, rid):
        dq = self.cal.get(rid)
        if not dq or len(dq) < 5:
            return {"n": 0 if not dq else len(dq)}
        a = np.array(dq)
        yr, ya, gr, ga, vr, va = a.T
        # clock offset from range: y_r = rdot * delta + noise
        wr = gr * gr / vr
        if wr.sum() > 0:
            delta = float(np.sum(gr * yr / vr) / wr.sum())
            res = yr - gr * delta
            scale = max(float(np.mean(res * res / vr)), 1.0)
            se_d = math.sqrt(scale / wr.sum())
        else:
            delta, se_d = 0.0, float("inf")
        # azimuth bias, with the clock offset's share of the bearing removed
        yb = ya - ga * delta
        b = float(np.sum(yb / va) / np.sum(1.0 / va))
        scale = max(float(np.mean((yb - b) ** 2 / va)), 1.0)
        se_b = math.sqrt(scale / np.sum(1.0 / va))
        bias_deg, se_bdeg = math.degrees(b), math.degrees(se_b)
        suspect = len(dq) >= 20 and (
            abs(bias_deg) > max(0.5, 3.0 * se_bdeg)
            or abs(delta) > max(0.05, 3.0 * se_d))
        return {"n": len(dq),
                "az_bias_deg": round(bias_deg, 3), "az_bias_se_deg": round(se_bdeg, 3),
                "time_offset_ms": round(1000.0 * delta, 1),
                "time_offset_se_ms": round(1000.0 * se_d, 1),
                "suspect": bool(suspect)}

    def build_summary(self, now):
        """Owner thread only: snapshot everything a reader may want, so the
        HTTP server, the stats printer and the SSE pushes never iterate a
        dict the fusion thread is changing (review A2: "dictionary changed
        size during iteration" propagated out of supernode.run() and closed
        the session). Readers call summary(), which returns this snapshot."""
        live = [k for k in list(self.tracks.values()) if not k.dead]
        stats = dict(self.stats)
        nis_n = stats.get("radar_assoc", 0)
        radars = []
        for st in list(self.status.values()):
            r = st.summary(now)
            r["cal"] = self._cal_estimate(st.id)
            if r["cal"].get("n", 0) == 0 and r["fused"] and st.plots:
                # Silence would read as "fine". Say why there is no estimate.
                r["cal"]["hint"] = ("no plot has landed on a TDOA-pinned track yet: "
                                    "no emitting drone in the radar's view, or every "
                                    "plot gated out (a large az_offset_deg or clock "
                                    "error does that)")
            radars.append(r)
        self._summary = {
            "lag_s": self.lag(now),
            "tracks": {"confirmed": sum(k.state == "confirmed" for k in live),
                       "tentative": sum(k.state == "tentative" for k in live)},
            "radar_nis_mean": (stats.get("radar_nis_sum", 0.0) / nis_n) if nis_n else None,
            "clutter_per_rev": {k: round(v[0], 2) for k, v in list(self.clutter.items())},
            "counters": {k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in stats.items() if k != "radar_nis_sum"},
            "radars": radars,
            "last_error": self.last_error,
        }
        return self._summary

    def summary(self, now=None):
        """The owner's last snapshot (any thread). Before the first one
        exists, a minimal placeholder rather than touching live state."""
        s = self._summary
        if s is None:
            return {"lag_s": 0.0, "tracks": {}, "radar_nis_mean": None,
                    "clutter_per_rev": {}, "counters": {}, "radars": [],
                    "last_error": None}
        return s
