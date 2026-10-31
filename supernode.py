#!/usr/bin/env python3
"""Central brain: correlate, localize, track, serve.

Pipeline per burst:
    events in  ->  group  ->  choose nodes by GDOP  ->  pull IQ  ->
    correlate  ->  solve 3D  ->  track filter  ->  publish + console

The node-selection step is the one that earns its keep. All ten nodes may
hear a burst, but pulling IQ from all ten costs ten transfers to improve a
solution that six well-placed nodes already determine. The supernode knows
the geometry, so it decides.
"""

import argparse
import base64
import collections
import json
import os
import queue
import threading
import time

# Calibration aid: when set, every all-pairs burst appends its per-pair lags
# (ns, arrival at a minus arrival at b) to this JSONL file. Off by default.
MEAS_LOG = os.environ.get("DL_MEAS_LOG")
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import zenoh

from dronelocate import proto, zconf
from dronelocate.geo import C_LIGHT, LocalFrame, gdop
from dronelocate.sigsim import dequantize
from dronelocate.tdoa import (node_spectra, cross_correlate_fft,
                              estimate_doppler, welch_psd, ht_weight,
                              solve_tdoa, solve_tdoa_gls, pair_covariance,
                              interpolator_floor, timing_sigma, gpu_available)
from dronelocate.fusion import Fusion, radar_key


# Console assets are ES modules. A browser refuses to execute a module served
# as text/plain -- console/index.html resolves 'three' through an importmap to
# vendor/three.module.js, and the wrong Content-Type there kills the entire
# module graph. The page still returns 200 and renders its empty shell, so it
# looks like the fleet never arrived rather than like a server bug.
_CONTENT_TYPES = [
    (".html", "text/html; charset=utf-8"),
    (".mjs", "text/javascript; charset=utf-8"),
    (".js", "text/javascript; charset=utf-8"),
    (".css", "text/css; charset=utf-8"),
    (".json", "application/json"),
    (".svg", "image/svg+xml"),
    (".png", "image/png"),
    (".ico", "image/x-icon"),
    (".wasm", "application/wasm"),
]


def content_type(path):
    for ext, ctype in _CONTENT_TYPES:
        if path.endswith(ext):
            return ctype
    return "application/octet-stream"


class Supernode:
    def __init__(self, cfg, correct_clocks=True, http_port=8080, zc=None,
                 spec_port=8081, radar_page_port=8084):
        self.cfg = cfg
        self.frame = LocalFrame.from_dict(cfg.origin)
        self.solver = cfg.__dict__.get("solver") or {}
        try:
            with open(args_config_path[0]) as f:
                self.solver = json.load(f).get("solver", {})
        except Exception:
            pass
        self.correct_clocks = correct_clocks

        # Two independent defences against a bad measurement, both switchable
        # live from the console so you can watch what each one buys.
        #
        #   quality gate  -- refuse a correlation whose peak does not stand out
        #                    from the noise floor. Catches a LOST peak, which
        #                    is wrong by microseconds and which timing_sigma()
        #                    cannot express: it assumes the peak found is the
        #                    right one, merely blurred, so it hands a garbage
        #                    lag almost the same weight as a clean one.
        #   robust loss   -- bound the influence of whatever still gets through.
        #                    Catches outliers a quality gate cannot see, e.g. a
        #                    node with a wrong surveyed position, which
        #                    correlates beautifully and still lies.
        self.quality_gate = bool(self.solver.get("quality_gate", False))
        self.quality_min = float(self.solver.get("quality_min", 8.0))
        self.robust = bool(self.solver.get("robust", False))
        # CAF: search differential Doppler as well as lag. A moving emitter
        # (or an undisciplined LO) puts a different frequency on each node;
        # correlating without compensating biases the lag first and then
        # nulls the peak entirely. vmax bounds the search span.
        self.caf = bool(self.solver.get("caf", True))
        self.caf_vmax = float(self.solver.get("caf_vmax_mps", 60.0))
        # All-pairs GLS: correlate every pair and weight with the full
        # covariance, instead of everyone-vs-reference with diagonal weights
        # that pretend the shared reference's noise is independent per row.
        self.all_pairs = bool(self.solver.get("all_pairs", True))
        # Tight coupling: a starved burst (too few pairs for a fix) still
        # updates an existing track directly with its TDOA rows.
        self.tight = bool(self.solver.get("tight_coupling", True))
        # GCC weighting: "ht" (Hannan-Thomson from per-node PSDs) or "none".
        # A no-op on flat spectra; earns its keep on structured signals and
        # per-node interference.
        self.gcc = str(self.solver.get("gcc", "ht")).lower()

        # Sim-only: which clock regime the emulated nodes are running. Pushed
        # to the nodes over the existing retasking channel rather than being
        # a supernode-side fiction, so what you switch is genuinely the
        # sensors' clocks and the supernode still has to cope with the result.
        self.clock_mode = str((cfg.sim or {}).get("clock_discipline", "gpsdo"))

        self.node_enu = {n["id"]: np.array(n["enu"], dtype=float) for n in cfg.nodes}
        # Measured per-node timing bias (ns, arrival-time bias, from a
        # reference emitter at a known spot). Taken out with the reported
        # clock offset below. Absent = 0, which is the old behaviour.
        self.node_bias_ns = {n["id"]: float(n.get("timing_offset_ns", 0.0))
                             for n in cfg.nodes}
        self.pending = collections.defaultdict(dict)   # burst -> node -> event
        self.pending_t = {}
        self.lock = threading.Lock()

        # The track layer. One owner, one thread: the three solve workers and
        # the radar subscriber submit measurements to it, and only the
        # fusion thread touches a track. This replaces the old track_lock --
        # with a radar in the picture, "whichever worker got the lock first"
        # is not a measurement order the filter can live with.
        self.fusion = Fusion(
            cfg=dict(cfg.__dict__.get("fusion") or {}),
            radars=list(cfg.__dict__.get("radars") or []),
            # passed as is: None means the tracker applies no prior after a
            # fix, exactly as before; the fusion module has its own 100 m
            # fallback (the solver's default) for radar-only seeding
            alt_prior_m=self.solver.get("alt_prior_m"),
            alt_prior_sigma_m=self.solver.get("alt_prior_sigma_m", 60.0))
        # Bursts with fewer than min_nodes events but at least this many go
        # to tight coupling instead of being dropped. Off unless configured:
        # on the 10-node site it would change which bursts count.
        self.tight_min_nodes = self.solver.get("tight_min_nodes")
        self.truth = None
        self.truth_by_burst = {}
        self.health = {}
        self.stats = {"bursts": 0, "fixes": 0, "iq_pulled": 0, "bytes_pulled": 0,
                      "solve_ms": collections.deque(maxlen=50),
                      "pull_ms": collections.deque(maxlen=50),
                      "started": time.time()}
        self.errors = collections.deque(maxlen=100)
        self.subscribers = []

        # Spectrum history per node, for the RF dashboard on the second port.
        # Server-side so a tab opened at any moment gets a populated
        # waterfall instead of staring at an empty canvas for a minute; the
        # browser then extends it from the live stream. 180 frames is ~90 s
        # at 2 bursts/s, and at 256 bytes a frame the whole fleet costs well
        # under a megabyte.
        self.spectra = collections.defaultdict(
            lambda: collections.deque(maxlen=180))
        self.spec_subscribers = []
        self.last_used = []

        self.session = zenoh.open(zc or zenoh.Config())
        site = cfg.site
        self.session.declare_subscriber(proto.ke_detect(site), self._on_event)
        self.session.declare_subscriber(proto.ke_health(site), self._on_health)
        self.session.declare_subscriber(proto.ke_truth(site), self._on_truth)
        self.session.declare_subscriber(proto.ke_spectrum(site), self._on_spectrum)
        self.session.declare_subscriber(proto.ke_plot(site), self._on_plot)
        self.pub_track = self.session.declare_publisher(
            proto.ke_track(site, "live"),
            priority=zenoh.Priority.DATA_HIGH,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )

        # Dwell coordination. The plan lives in config and the schedule is
        # derived from UTC, so nodes agree with or without this publisher;
        # the broadcast exists to change the plan at runtime, which is the
        # part a shared grid cannot do by itself.
        self.dwell_cfg = dict(cfg.__dict__.get("dwell") or {})
        self.dwell_plan = list(self.dwell_cfg.get("plan") or [])
        self.dwell_s = float(self.dwell_cfg.get("dwell_s", 0.5))
        self.pub_dwell = self.session.declare_publisher(
            proto.ke_dwell(site),
            priority=zenoh.Priority.INTERACTIVE_HIGH,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )
        if self.dwell_cfg.get("enabled", False) and self.dwell_plan:
            threading.Thread(target=self._dwell_loop, daemon=True).start()

        self.work = queue.Queue()
        threading.Thread(target=self._fusion_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()
        for _ in range(3):
            threading.Thread(target=self._worker, daemon=True).start()
        threading.Thread(target=self._reaper, daemon=True).start()
        self._serve_http(http_port)
        if spec_port:
            self._serve_spectrum_http(spec_port)
        if radar_page_port:
            self._serve_radar_page(radar_page_port)

    # --- inbound ---------------------------------------------------------
    def _on_event(self, sample):
        try:
            e = proto.decode(sample.payload)
        except Exception:
            return
        b = e["burst"]
        with self.lock:
            self.pending[b][e["node"]] = e
            self.pending_t.setdefault(b, time.time())

    def _on_health(self, sample):
        try:
            h = proto.decode(sample.payload)
            if h.get("kind") == "radar":
                # Radars publish on the same health key as the TDOA nodes.
                # Kept out of self.health, which is the RF fleet: the console
                # and the spectrum dashboard read node fields off it.
                self.fusion.on_radar_health(h)
                return
            self.health[h["node"]] = h
        except Exception:
            pass

    def _on_plot(self, sample):
        """Radar plot batches. Updates the radar's availability whatever its
        state; queued for fusion only if the radar is configured,
        calibrated and live -- otherwise the system is TDOA-only, which is
        exactly what it was before radar existed."""
        try:
            b = proto.decode(sample.payload)
        except Exception:
            self.stats["plot_decode_errors"] = self.stats.get("plot_decode_errors", 0) + 1
            return
        try:
            st = self.fusion.radar_status(radar_key(b))   # one per radar RANGE
            was = st.offset_mismatch
            self.fusion.submit_batch(b)
            if st.offset_mismatch is not None and was is None:
                print(f"[super] radar {st.id}: az_offset_deg on the bus "
                      f"({st.offset_mismatch}) differs from the site config "
                      f"({st.site.az_offset_deg}); using the site config")
        except Exception as e:
            print(f"[super] plot batch failed: {e}")

    def _on_spectrum(self, sample):
        """Display only. Nothing in the solve path reads this, exactly like
        sim/truth -- delete the key expression and the system still
        localizes, it just stops being legible to an operator."""
        try:
            f = proto.decode(sample.payload)
            row = {
                "node": f["node"], "t": f["t_utc_ns"] / 1e9,
                "fc_hz": f["fc_hz"], "fs_sps": f["fs_sps"],
                "n_bins": f["n_bins"],
                "db_ref": f["db_ref"], "db_step": f["db_step"],
                "rssi_dbm": f["rssi_dbm"], "snr_db": f["snr_db"],
                "cls": f.get("cls", "unknown"),
                # Real radio or emulated. Taken from the node's own health
                # rather than the site config's role field, same as the 3D
                # console: config says what a node was meant to be, health
                # says what actually came up, and the gap between them is
                # what you are looking for while bringing hardware online.
                "hw": (self.health.get(f["node"], {}).get("source")
                       not in (None, "sim")),
                "bins": base64.b64encode(bytes(f["bins"])).decode("ascii"),
            }
            self.spectra[f["node"]].append(row)
            self._push_spec({"type": "spec", "row": row,
                             "used": list(self.last_used)})
        except Exception:
            pass

    def _on_truth(self, sample):
        """Scoring only. Nothing in the solve path reads this.

        Kept per burst id, not just latest: a fix is scored against the truth
        of the burst it solved, otherwise solve latency leaks into the error
        metric -- at 31 m/s a fix scored one truth message late reads ~15 m
        of purely fictitious horizontal error.
        """
        try:
            t = proto.decode(sample.payload)
            self.truth = t
            self.truth_by_burst[t["burst"]] = t
            while len(self.truth_by_burst) > 32:
                self.truth_by_burst.pop(next(iter(self.truth_by_burst)))
        except Exception:
            pass

    def _reaper(self):
        window = float(self.solver.get("group_window_ms", 400)) / 1000.0
        min_nodes = int(self.solver.get("min_nodes", 5))
        while True:
            now = time.time()
            ready = []
            with self.lock:
                for b, t0 in list(self.pending_t.items()):
                    if now - t0 >= window:
                        evs = self.pending.pop(b, {})
                        self.pending_t.pop(b, None)
                        if len(evs) >= min_nodes:
                            ready.append((b, evs))
                        elif (self.tight and self.tight_min_nodes
                              and len(evs) >= int(self.tight_min_nodes)):
                            # Too few hearers for a fix, enough pairs to
                            # constrain an existing track (tight coupling).
                            ready.append((b, evs))
                        elif evs:
                            self.stats.setdefault("dropped_bursts", 0)
                            self.stats["dropped_bursts"] += 1
            for b, evs in ready:
                self.work.put((b, evs))
            time.sleep(0.05)

    def set_clock_mode(self, mode):
        """Retask every node's clock discipline. Sim only."""
        mode = str(mode).lower()
        if mode not in ("gpsdo", "holdover", "free"):
            return self.clock_mode
        for n in self.cfg.nodes:
            self.session.put(proto.ke_cmd(self.cfg.site, n["id"]),
                             proto.encode({"clock_mode": mode}),
                             priority=zenoh.Priority.INTERACTIVE_HIGH,
                             congestion_control=zenoh.CongestionControl.BLOCK)
        self.clock_mode = mode
        print(f"[super] node clock discipline -> {mode}")
        return mode

    def _radar_ids(self, which):
        """Radars a /control radar command applies to: one id, or all."""
        known = ({r["id"] for r in (self.cfg.radars or [])}
                 | {s["id"] for s in self._radar_summary()})
        return sorted(known) if which in (None, "", "all") else [which]

    def set_radar_fuse(self, on, which=None):
        """Operator switch: fuse this radar's plots or not. Supernode-side
        only -- the radar keeps transmitting and its node keeps publishing,
        so the plots stay on the console and turning it back on is instant."""
        for rid in self._radar_ids(which):
            self.fusion.radar_status(rid).enabled = bool(on)
            print(f"[super] radar {rid}: fusion {'on' if on else 'off'}")

    def set_radar_power(self, state, which=None):
        """on / off / standby / transmit, relayed to the radar node on its
        command key, which turns it into navico's frames (off is the soft
        off: radio side down, radar still on the network; on brings it back
        to standby). The supernode cannot reach the radar itself: only the
        radar node's PC is on the radar's LAN."""
        state = str(state).lower()
        if state not in ("on", "off", "standby", "transmit"):
            return
        # ids are radar ids, not range keys: both range nodes of r01 listen
        # on dc/{site}/cmd/r01, so one put reaches A and B
        which = which.split(":")[0] if which else which
        for rid in sorted({r.split(":")[0] for r in self._radar_ids(which)}):
            self.session.put(proto.ke_cmd(self.cfg.site, rid),
                             proto.encode({"power": state}),
                             priority=zenoh.Priority.INTERACTIVE_HIGH,
                             congestion_control=zenoh.CongestionControl.BLOCK)
            print(f"[super] radar {rid}: power -> {state}")

    # --- node selection --------------------------------------------------
    def _select(self, events):
        """Greedy PDOP-minimising subset.

        Seeded with an RSSI-weighted centroid: crude, but it only has to be
        good enough to rank geometry, and it costs nothing since RSSI is
        already in the event.
        """
        ids = list(events.keys())
        if len(ids) <= int(self.solver.get("min_nodes", 5)):
            return ids

        p = np.array([self.node_enu[i] for i in ids])
        w = np.array([10 ** (events[i]["rssi_dbm"] / 20.0) for i in ids])
        seed = (p * w[:, None]).sum(axis=0) / max(w.sum(), 1e-9)
        seed[2] = float(self.solver.get("alt_prior_m", 100.0))

        order = sorted(ids, key=lambda i: -events[i]["snr_db"])
        chosen = [order[0]]
        cap = int(self.solver.get("max_nodes", 7))
        while len(chosen) < min(cap, len(ids)):
            best, best_pdop = None, np.inf
            for cand in ids:
                if cand in chosen:
                    continue
                trial = chosen + [cand]
                if len(trial) < 4:
                    best, best_pdop = cand, 0.0
                    break
                _, _, pd = gdop(np.array([self.node_enu[i] for i in trial]), seed, 0)
                if pd < best_pdop:
                    best, best_pdop = cand, pd
            if best is None:
                break
            chosen.append(best)
        return chosen

    # --- solve -----------------------------------------------------------
    def _worker(self):
        while True:
            burst, events = self.work.get()
            try:
                self._process(burst, events)
            except Exception as e:
                print(f"[super] burst {burst} failed: {e}")

    def _pull(self, node_id, cap_id, timeout=4.0):
        stored = getattr(self, "replay_store", None)
        if stored is not None:
            # --replay-iq: the capture comes from a saved burst, not the bus
            return stored.pop((node_id, cap_id), (None, None))
        sel = f"{proto.ke_iq(self.cfg.site, node_id)}?cap_id={cap_id}"
        for reply in self.session.get(sel, timeout=timeout,
                                      priority=zenoh.Priority.BACKGROUND,
                                      congestion_control=zenoh.CongestionControl.BLOCK):
            if reply.ok is None:
                continue
            s = reply.ok
            meta = proto.decode(s.attachment) if s.attachment is not None else {}
            return bytes(s.payload.to_bytes()), meta
        return None, None

    def _save_iq(self, burst, events, raw, meta):
        """--save-iq: keep what this burst was solved from, so it can be
        solved again later -- with surveyed node positions, another config,
        a new solver -- through --replay-iq. Stored as received: the node's
        wire format (ci8/ci2...), its metadata, and every hearer's detection
        event. With --save-iq-all the hearers the selection skipped are
        pulled too (after the solve, so the fix is not delayed), letting a
        replay with corrected positions choose differently."""
        if getattr(self, "save_iq_all", False):
            for nid in events:
                if nid not in raw:
                    payload, m = self._pull(nid, events[nid]["cap_id"])
                    if payload is not None:
                        raw[nid], meta[nid] = payload, m
        rec = {"v": 1, "site": self.cfg.site, "burst": burst, "events": events,
               "iq": raw, "meta": meta, "saved_utc": time.time()}
        path = os.path.join(self.save_iq_dir, f"burst_{int(burst):010d}.cbor")
        with open(path + ".tmp", "wb") as f:
            f.write(proto.encode(rec))
        os.replace(path + ".tmp", path)
        self.stats["iq_saved"] = self.stats.get("iq_saved", 0) + 1

    def replay_iq(self, directory, speed=1.0, wait_s=3.0):
        """--replay-iq: feed saved bursts through the same _process path,
        paced by their capture times (speed > 1 is faster), then stop.
        Waits wait_s first so the console and subscribers are connected
        before the first fix -- without it the first bursts went out to
        nobody."""
        files = sorted(f for f in os.listdir(directory) if f.endswith(".cbor"))
        self.replay_store = {}
        print(f"[super] replaying {len(files)} saved bursts from {directory} "
              f"in {wait_s:g} s")
        time.sleep(wait_s)
        start, first = time.monotonic(), None
        for name in files:
            with open(os.path.join(directory, name), "rb") as f:
                rec = proto.decode(f.read())
            events = rec["events"]
            t = min(e["t_utc_ns"] for e in events.values()) * 1e-9
            first = t if first is None else first
            wait = (t - first) / speed - (time.monotonic() - start)
            if wait > 0:
                time.sleep(wait)
            for nid, payload in rec["iq"].items():
                self.replay_store[(nid, events[nid]["cap_id"])] = (
                    payload, rec.get("meta", {}).get(nid, {}))
            self.work.put((rec["burst"], events))
        print("[super] replay finished")

    def _process(self, burst, events):
        self.stats["bursts"] += 1
        chosen = self._select(events)

        t_pull = time.time()
        iq, meta, raw = {}, {}, {}
        for nid in chosen:
            payload, m = self._pull(nid, events[nid]["cap_id"])
            if payload is None:
                continue
            n = events[nid]["n_samples"]
            iq[nid] = dequantize(payload, events[nid]["fmt"], n)
            meta[nid] = m
            raw[nid] = payload
            self.stats["iq_pulled"] += 1
            self.stats["bytes_pulled"] += len(payload)
        pull_ms = (time.time() - t_pull) * 1000.0
        self.stats["pull_ms"].append(pull_ms)
        if getattr(self, "save_iq_dir", None):
            try:
                self._save_iq(burst, events, raw, meta)
            except Exception as e:          # a full disk must not cost the fix
                print(f"[super] save-iq burst {burst} failed: {e}")

        min_nodes = int(self.solver.get("min_nodes", 5))
        tight_only = False
        if len(iq) < min_nodes:
            if not (self.tight and self.tight_min_nodes
                    and len(iq) >= int(self.tight_min_nodes)):
                return
            tight_only = True

        # reference = strongest of the pulled set
        ref = max(iq.keys(), key=lambda i: events[i]["snr_db"])
        # The burst's capture epoch, not the moment the solve finished: the
        # tracker integrates over the interval between measurements, and
        # solve latency (pull + correlate + solve, varying per burst) is not
        # part of the target's motion.
        t_meas = events[ref]["t_utc_ns"] * 1e-9
        fs = float(events[ref]["fs_sps"])
        bw = float(events[ref]["bw_hz"])

        pts = np.array([self.node_enu[i] for i in iq])
        baseline = float(np.max(np.linalg.norm(pts[:, None] - pts[None, :], axis=2)))
        max_lag = baseline / C_LIGHT + 5e-6

        # Doppler span the CAF must cover: emitter motion contributes up to
        # 2*vmax/c of fractional offset between a pair, undisciplined LOs
        # another ~0.1 ppm. Scaled by the event's actual fc so a 5.8 GHz
        # retask widens the search on its own.
        fc = float(events[ref].get("fc_hz", 2.437e9))
        max_dop = fc * (2.0 * self.caf_vmax / C_LIGHT + 1.2e-7)

        # One forward FFT per node, shared by every pair it appears in --
        # without this the all-pairs loop pays two 64k FFTs per pair and the
        # solve time quadruples. The CAF derotation becomes an integer-bin
        # circular shift of the cached spectrum. Same idea for the GCC
        # weight: one Welch PSD per node, one weight per pair.
        spectra, nfft = node_spectra(iq, fs)
        psd = {i: welch_psd(iq[i]) for i in iq} if self.gcc == "ht" else None
        pair_w = {}

        def correlate(ni, nj, lag_span, center):
            dop = 0.0
            if self.caf:
                dop = estimate_doppler(iq[ni], iq[nj], fs, lag_span, center,
                                       max_dop)
            w = None
            if psd is not None:
                w = pair_w.get((ni, nj))
                if w is None:
                    w = pair_w[(ni, nj)] = ht_weight(psd[ni], psd[nj])
            c = cross_correlate_fft(spectra[ni], spectra[nj], nfft, fs,
                                    lag_span, center,
                                    int(np.rint(dop * nfft / fs)), weight=w)
            c.doppler_hz = dop
            return c

        t_solve = time.time()

        def measure(ni, nj):
            """One pair's TDOA: correlate, centre on the clock delta, gate."""
            # The peak sits at (geometric TDOA + clock offset difference).
            # max_lag only bounds the geometric part, so the search has to be
            # told about the other half or the peak falls outside it once
            # drift has accumulated -- minutes, not hours.
            clk_delta = (events[ni]["clk_off_ns"] + self.node_bias_ns.get(ni, 0.0)
                         - events[nj]["clk_off_ns"] - self.node_bias_ns.get(nj, 0.0)) * 1e-9
            if self.correct_clocks:
                c = correlate(ni, nj, max_lag, clk_delta)
                # remove the per-node clock error the calibration loop
                # recovered; leaving it in is the classic silent failure
                lag = c.lag_s - clk_delta
            else:
                # Uncalibrated demo: we are pretending not to know the offset,
                # so we may not centre on it. Widen instead, otherwise the
                # failure on screen is the correlator losing the peak rather
                # than the solver being fed a biased TDOA, which is the point.
                c = correlate(ni, nj, max_lag + abs(clk_delta), 0.0)
                lag = c.lag_s

            if self.quality_gate and c.quality < self.quality_min:
                # Drop the measurement, not the burst -- node selection already
                # picked a GDOP-redundant subset, so losing one still solves.
                weaker = min(ni, nj, key=lambda k: events[k]["snr_db"])
                self.stats["rejected"] = self.stats.get("rejected", 0) + 1
                self.stats.setdefault("rejected_by_node", collections.Counter())
                self.stats["rejected_by_node"][weaker] += 1
                return None
            return c, lag

        ids = list(iq.keys())
        others, quality, doppler = [], [], []

        if self.all_pairs:
            meas = []                # (i_idx, j_idx, lag, var, quality, dop)
            for xi in range(len(ids)):
                for yj in range(xi + 1, len(ids)):
                    m = measure(ids[xi], ids[yj])
                    if m is None:
                        continue
                    c, lag = m
                    meas.append((xi, yj, lag, timing_sigma(c, fs, bw) ** 2,
                                 c.quality, c.doppler_hz))

            if MEAS_LOG and meas:
                try:
                    with open(MEAS_LOG, "a") as f:
                        f.write(json.dumps({"t": time.time(), "meas": [
                            [ids[a], ids[b], lag * 1e9, v ** 0.5 * 1e9, q]
                            for a, b, lag, v, q, _ in meas]}) + "\n")
                except OSError as e:
                    print(f"[super] meas log: {e}")
            if len(meas) < 3 or tight_only:
                self.stats["starved"] = self.stats.get("starved", 0) + 1
                self._tight_update(ids, meas, fs, t_meas)
                return

            pairs = np.array([[m[0], m[1]] for m in meas], dtype=int)
            node_pos = np.array([self.node_enu[i] for i in ids])
            rmat = pair_covariance(pairs, np.array([m[3] for m in meas]),
                                   len(ids), interpolator_floor(fs))
            fix = solve_tdoa_gls(
                node_pos[pairs[:, 0]], node_pos[pairs[:, 1]],
                np.array([m[2] for m in meas]), rmat,
                alt_prior_m=self.solver.get("alt_prior_m", 100.0),
                alt_prior_sigma_m=self.solver.get("alt_prior_sigma_m", 60.0),
                robust=self.robust,
                ground_alt_m=self.solver.get("ground_alt_m"),
            )

            # Per-node display fields: each contributing node's best pair.
            best = {}
            for xi, yj, lag, v, q, dop in meas:
                for k in (ids[xi], ids[yj]):
                    if k != ref and (k not in best or q > best[k][0]):
                        best[k] = (q, dop)
            others = sorted(best)
            quality = [best[k][0] for k in others]
            doppler = [best[k][1] for k in others]
        else:
            tdoa, sig = [], []
            for nid in ids:
                if nid == ref:
                    continue
                m = measure(nid, ref)
                if m is None:
                    continue
                c, lag = m
                others.append(nid)
                tdoa.append(lag)
                sig.append(timing_sigma(c, fs, bw))
                quality.append(c.quality)
                doppler.append(c.doppler_hz)

            # The min_nodes check upstream ran before correlation, so it
            # cannot know about gate rejections. Re-check here or a burst that
            # lost most of its measurements goes to the solver with too little
            # to constrain three unknowns.
            if len(tdoa) < 3 or tight_only:
                self.stats["starved"] = self.stats.get("starved", 0) + 1
                self._tight_update(
                    ids, [(ids.index(n), ids.index(ref), t, s ** 2, q, d)
                          for n, t, s, q, d in
                          zip(others, tdoa, sig, quality, doppler)], fs, t_meas)
                return

            fix = solve_tdoa(
                np.array([self.node_enu[i] for i in others]),
                self.node_enu[ref],
                np.array(tdoa), sigma_s=np.array(sig),
                alt_prior_m=self.solver.get("alt_prior_m", 100.0),
                alt_prior_sigma_m=self.solver.get("alt_prior_sigma_m", 60.0),
                robust=self.robust,
                ground_alt_m=self.solver.get("ground_alt_m"),
            )
        solve_ms = (time.time() - t_solve) * 1000.0
        self.stats["solve_ms"].append(solve_ms)
        if fix is None:
            return
        self.stats["fixes"] += 1
        # How often a restart beat the grid seed (the mirror sheet), and how
        # often the ground rule overrode the lower cost -- the latter is the
        # genuine-ambiguity rate, and a high value on a hillside site means
        # solver.ground_alt_m is wrong.
        for key in ("restart_won", "floor_forced"):
            if fix.detail.get(key):
                self.stats[key] = self.stats.get(key, 0) + 1

        # Fix accuracy against this burst's own truth -- a solver metric,
        # scored here rather than at emit so that a fix the track layer
        # rejects as an outlier is still counted against the solver.
        err = None
        truth = self.truth_by_burst.get(burst, self.truth)
        if truth is not None:
            d = fix.enu - np.array(truth["enu"], dtype=float)
            err = {"h_m": float(np.hypot(d[0], d[1])), "v_m": float(d[2]),
                   "total_m": float(np.linalg.norm(d))}
            self.errors.append(err)

        self.fusion.submit_fix(t_meas, fix, meta={
            "burst": burst, "chosen": chosen, "others": others, "ref": ref,
            "quality": quality, "doppler": doppler, "pull_ms": pull_ms,
            "solve_ms": solve_ms, "error": err})

    def _tight_update(self, ids, meas, fs, t_meas):
        """Starved burst: too few pairs for a fix, not too few to matter.

        Every surviving pair is still a hyperboloid constraint. Feed them to
        the track they fit best as an EKF measurement update -- the track's
        prior supplies what the burst is missing. No track birth from
        underdetermined data, and the update is innovation-gated inside
        update_tdoa so a bad pair coasts instead of corrupting.
        """
        if not (self.tight and meas):
            return
        node_pos = np.array([self.node_enu[i] for i in ids])
        pairs = np.array([[m[0], m[1]] for m in meas], dtype=int)
        pos_a, pos_b = node_pos[pairs[:, 0]], node_pos[pairs[:, 1]]
        tdoa = np.array([m[2] for m in meas])
        var = np.array([m[3] for m in meas])
        rmat = pair_covariance(pairs, var, len(ids), interpolator_floor(fs))

        # Track choice (smallest normalised TDOA residual -- with two drones
        # up, "the newest track" is as likely to be the wrong one) and the
        # gated update now happen in the fusion thread, in time order.
        self.fusion.submit_tight(t_meas, pos_a, pos_b, tdoa, rmat, var=var)

    # --- track layer -----------------------------------------------------
    def _fusion_loop(self):
        """The single owner of the track list. Wakes when something is
        submitted or when the oldest queued measurement comes due."""
        last_err = None
        while True:
            try:
                self.fusion.wait(time.time())
                outs = self.fusion.process_due(time.time())
            except Exception as e:
                print(f"[super] fusion step failed: {e}")
                time.sleep(0.1)
                continue
            for o in outs:
                try:
                    self._emit(o)
                except Exception as e:
                    print(f"[super] emit {o.get('kind')} failed: {e}")
            if self.fusion.last_error != last_err:
                last_err = self.fusion.last_error
                print(f"[super] fusion item failed (counted, skipped): {last_err}")

    def _status_loop(self):
        """Radar availability for the console, once a second, whether or
        not anything is being tracked -- "the radar is down" matters most
        exactly when nothing is on screen."""
        while True:
            time.sleep(1.0)
            if not self.subscribers:
                continue
            try:
                # node health and stats ride along: until the first fix
                # there is no track message to carry them, and the fleet
                # list stayed blank exactly while the nodes were being
                # brought up
                self._push_raw({"type": "status", "radars": self._radar_summary(),
                                "fusion": self.fusion.summary(),
                                "health": self.health,
                                "stats": self._stat_summary()})
            except Exception as e:
                print(f"[super] status push failed: {e}")

    def _radar_summary(self):
        # the fusion owner's snapshot: never iterate live fusion state from
        # here (HTTP, SSE and the stats printer all run on other threads)
        return self.fusion.summary()["radars"]

    def _emit(self, o):
        """Publish one fusion output. Track updates go on the bus and to the
        console; radar plots go to the console only (display layer)."""
        kind = o["kind"]
        if kind == "plots":
            self._push_raw({"type": "plots", "radar": o["radar"],
                            "sector": o["sector"], "plots": o["plots"]})
            return
        if kind == "rejected":
            m = o["meta"]
            print(f"[super] burst {m.get('burst')}: fix rejected, "
                  f"{o['dist_m']:.0f} m from radar-supported track {o['near_tid']}")
            return
        if kind not in ("tdoa", "radar", "coast"):
            return
        tr = o["track"]
        x, P = tr["x"], tr["P"]
        pos, vel = x[:3], x[3:]
        lat, lon, alt = self.frame.to_geodetic(pos)
        pcov = P[:3, :3]
        fusion_fields = {
            "t_state": tr["t_state"], "state": tr["state"], "src": tr["src"],
            "upd": kind,
            "n_tdoa": tr["n_tdoa"], "n_radar": tr["n_radar"],
            "last_tdoa_t": tr["last_tdoa_t"], "last_radar_t": tr["last_radar_t"],
            "alt_src": tr["alt_src"],
            "track_cov": [[float(c) for c in row] for row in pcov],
        }
        if kind == "tdoa":
            m, fix = o["meta"], o["fix"]
            others, ref, chosen = m["others"], m["ref"], m["chosen"]
            msg = {
                "t": time.time(), "t_meas": o["t"], "burst": m["burst"],
                "tid": tr["tid"],
                "late_updates": tr["late"],
                "enu": [float(v) for v in pos],
                "vel": [float(v) for v in vel],
                "lat": lat, "lon": lon, "alt_m": alt,
                "raw_enu": [float(v) for v in fix.enu],
                "sigma_h": fix.sigma_h, "sigma_v": fix.sigma_v,
                "cov": [[float(c) for c in row] for row in fix.cov],
                "hdop": fix.hdop, "vdop": fix.vdop,
                "resid_ns": fix.residual_rms_s * 1e9,
                "chi2": fix.detail.get("chi2_red", 0.0),
                "n_meas": fix.n_meas, "ref": ref,
                "nodes_heard": len(chosen), "nodes_used": len(others) + 1,
                # Ordered to match corr_quality element for element. Without
                # the ids a consumer gets a bare list of numbers it cannot
                # attribute to anything, which is why the console could not
                # draw these.
                "used": list(others),
                "heard": list(chosen),
                "corr_quality": [float(q) for q in m["quality"]],
                "corr_doppler_hz": [float(d) for d in m["doppler"]],
                "pull_ms": m["pull_ms"], "solve_ms": m["solve_ms"],
                "clock_correction": self.correct_clocks,
                "truth": self.truth, "error": m["error"],
            }
            # Mirrored for the spectrum dashboard: it streams on its own
            # port and never sees this message, but "which nodes did the
            # solver actually use" is exactly what an operator wants marked
            # on the waterfall.
            self.last_used = [ref] + list(others)
        else:
            # Radar update or a coasting report: no fix behind it, so the
            # fix fields carry the track's own state and the TDOA-specific
            # ones are empty rather than invented.
            p = o.get("plot")
            sh = float(np.sqrt(max(pcov[0, 0] + pcov[1, 1], 0.0)))
            sv = float(np.sqrt(max(pcov[2, 2], 0.0)))
            msg = {
                "t": time.time(), "t_meas": o["t"], "burst": None,
                "tid": tr["tid"], "late_updates": tr["late"],
                "enu": [float(v) for v in pos],
                "vel": [float(v) for v in vel],
                "lat": lat, "lon": lon, "alt_m": alt,
                "raw_enu": None,
                "sigma_h": sh, "sigma_v": sv,
                "cov": [[float(c) for c in row] for row in pcov],
                "hdop": None, "vdop": None, "resid_ns": None, "chi2": None,
                "n_meas": 2 if p else 0, "ref": None,
                "nodes_heard": 0, "nodes_used": 0,
                "used": [], "heard": [], "corr_quality": [],
                "corr_doppler_hz": [],
                "pull_ms": None, "solve_ms": None,
                "clock_correction": self.correct_clocks,
                "truth": self.truth, "error": None,
                "radar": o.get("radar"),
                "plot": None if p is None else {
                    "rng_m": float(p["rng_m"]), "az_deg": float(p["az_deg"]),
                    "t": p["t_utc_ns"] * 1e-9, "dop": p.get("dop"),
                    "sig_rng_m": p.get("sig_rng_m"),
                    "sig_az_deg": p.get("sig_az_deg")},
            }
        msg.update(fusion_fields)
        self.pub_track.put(proto.encode(msg))
        self._push(msg)

    # --- console telemetry (SSE) ----------------------------------------
    def _push(self, msg):
        self._push_raw({"type": "track", "data": msg,
                        "health": self.health,
                        "stats": self._stat_summary()})

    def _push_raw(self, obj):
        payload = json.dumps(obj, default=str)
        dead = []
        for q in self.subscribers:
            try:
                q.put_nowait(payload)
            except Exception:
                dead.append(q)
        for q in dead:
            self.subscribers.remove(q)

    def _push_spec(self, msg):
        """Fan out to the RF dashboard's own subscriber list.

        Separate from _push on purpose. The console updates only when a track
        is published, which is correct for it and wrong here: the spectrum
        display must keep scrolling when nothing is being detected at all --
        a flat noise floor is a measurement, and a waterfall that freezes
        whenever the fleet stops solving hides exactly the case an operator
        opened it to investigate.
        """
        payload = json.dumps(msg, default=str)
        dead = []
        for q in self.spec_subscribers:
            try:
                q.put_nowait(payload)
            except Exception:
                dead.append(q)
        for q in dead:
            self.spec_subscribers.remove(q)

    def _stat_summary(self):
        up = time.time() - self.stats["started"]
        errs = list(self.errors)
        return {
            "uptime_s": up, "bursts": self.stats["bursts"],
            "fixes": self.stats["fixes"], "iq_pulled": self.stats["iq_pulled"],
            "mb_pulled": self.stats["bytes_pulled"] / 1e6,
            "ingest_mbps": self.stats["bytes_pulled"] * 8 / 1e6 / max(up, 1e-6),
            "dropped_bursts": self.stats.get("dropped_bursts", 0),
            "solve_ms": float(np.mean(list(self.stats["solve_ms"]))) if self.stats["solve_ms"] else 0,
            "pull_ms": float(np.mean(list(self.stats["pull_ms"]))) if self.stats["pull_ms"] else 0,
            "err_h_p50": float(np.median([e["h_m"] for e in errs])) if errs else None,
            "err_v_p50": float(np.median([abs(e["v_m"]) for e in errs])) if errs else None,
            "gpu": gpu_available(),
            # solver switches, echoed so the console reflects real server
            # state rather than whatever the last click optimistically assumed
            "quality_gate": self.quality_gate,
            "quality_min": self.quality_min,
            "robust": self.robust,
            "caf": self.caf,
            "all_pairs": self.all_pairs,
            "tight_coupling": self.tight,
            "gcc": self.gcc,
            "clock_mode": self.clock_mode,
            "rejected": self.stats.get("rejected", 0),
            "starved": self.stats.get("starved", 0),
            "tight_updates": self.fusion.stats.get("tight_updates", 0),
            "tight_rejected": self.fusion.stats.get("tight_rejected", 0),
            "restart_won": self.stats.get("restart_won", 0),
            "floor_forced": self.stats.get("floor_forced", 0),
            "rejected_by_node": dict(self.stats.get("rejected_by_node", {})),
            # radar + track layer
            "radars": self._radar_summary(),
            "fusion": self.fusion.summary(),
        }

    def _serve_http(self, port):
        sup = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path.startswith("/events"):
                    q = queue.Queue(maxsize=20)
                    sup.subscribers.append(q)
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    try:
                        self.wfile.write(b": connected\n\n")
                        self.wfile.flush()
                        while True:
                            try:
                                data = q.get(timeout=15)
                                self.wfile.write(f"data: {data}\n\n".encode())
                            except queue.Empty:
                                self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                    except Exception:
                        pass
                    finally:
                        if q in sup.subscribers:
                            sup.subscribers.remove(q)
                    return

                if self.path.startswith("/control"):
                    # Live solver switches for the console. GET with query
                    # params because the console is a static page with no
                    # build step; note this server binds 0.0.0.0 and has no
                    # auth, so anyone who can reach the console can flip
                    # these -- fine for a demo, not for a field deployment.
                    q = parse_qs(urlparse(self.path).query)
                    truthy = ("1", "true", "on", "yes")

                    def flag(name, cur):
                        if name not in q:
                            return cur
                        return str(q[name][0]).lower() in truthy

                    sup.quality_gate = flag("gate", sup.quality_gate)
                    sup.robust = flag("robust", sup.robust)
                    sup.caf = flag("caf", sup.caf)
                    sup.all_pairs = flag("pairs", sup.all_pairs)
                    sup.tight = flag("tight", sup.tight)
                    if "gcc" in q:
                        v = str(q["gcc"][0]).lower()
                        sup.gcc = "ht" if v in truthy + ("ht",) else "none"
                    if "quality_min" in q:
                        try:
                            sup.quality_min = float(q["quality_min"][0])
                        except ValueError:
                            pass
                    if "clock" in q:
                        sup.set_clock_mode(q["clock"][0])
                    # radar=<id> picks one radar for the next two; default all
                    rsel = q.get("radar", [None])[0]
                    if "radar_fuse" in q:
                        sup.set_radar_fuse(str(q["radar_fuse"][0]).lower() in truthy, rsel)
                    if "radar_power" in q:
                        sup.set_radar_power(q["radar_power"][0], rsel)
                    body = json.dumps({"quality_gate": sup.quality_gate,
                                       "robust": sup.robust,
                                       "quality_min": sup.quality_min,
                                       "caf": sup.caf,
                                       "gcc": sup.gcc,
                                       "clock_mode": sup.clock_mode}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/health"):
                    # Machine-readable state of everything that is not a
                    # track: radar availability, fusion counters, nodes.
                    body = json.dumps({
                        "radars": sup._radar_summary(),
                        "fusion": sup.fusion.summary(),
                        "nodes": sup.health,
                    }, default=str).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/config"):
                    body = json.dumps({
                        "site": sup.cfg.site, "origin": sup.cfg.origin,
                        "nodes": sup.cfg.nodes, "radio": sup.cfg.radio,
                        "radars": sup.cfg.radars,
                        "radar_status": sup._radar_summary(),
                        "fusion": sup.fusion.cfg,
                    }, default=str).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                # Resolve under console/ and confirm we stayed there. The
                # naive "console" + self.path lets a raw client send
                # GET /../supernode.py and read anything the process can --
                # and this server binds 0.0.0.0, so that is reachable from
                # the network, not just the browser (which would normalise
                # the path before sending it).
                base = os.path.abspath("console")
                rel = self.path.split("?", 1)[0].split("#", 1)[0].lstrip("/")
                path = os.path.abspath(os.path.join(base, rel or "index.html"))
                if path != base and not path.startswith(base + os.sep):
                    self.send_error(403)
                    return
                try:
                    with open(path, "rb") as f:
                        body = f.read()
                except OSError:
                    self.send_error(404)
                    return
                ctype = content_type(path)
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                # The console is edited constantly and served from disk on
                # every request, so a cached copy is always a bug: it hands
                # back JavaScript that was fixed minutes ago and makes the
                # fix look like it did not work. Baked map data under
                # cache/ is immutable by name, so let that one be cached.
                if "/cache/" not in self.path:
                    self.send_header("Cache-Control", "no-store, must-revalidate")
                self.end_headers()
                self.wfile.write(body)

        srv = ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[super] console on http://localhost:{port}")

    def _dwell_loop(self):
        """Announce the schedule the fleet is already deriving.

        Deliberately not the authority on WHEN -- every node computes the
        same dwell index from UTC, so a lost message costs nothing. What
        this publishes is the plan, at a low rate, so a node that joins late
        or a plan changed at runtime propagates without restarting anything.
        """
        lead = float(self.dwell_cfg.get("lead_s", 0.5))
        period = max(float(self.dwell_cfg.get("announce_s", 2.0)), 0.2)
        span = sum(c.get("fs_sps", 0.0) for c in self.dwell_plan) / 1e6
        print(f"[super] dwell coordinator: {len(self.dwell_plan)} channels, "
              f"{self.dwell_s:g} s each, {span:.1f} MHz total span, "
              f"revisit {len(self.dwell_plan) * self.dwell_s:g} s")
        while True:
            now = time.time()
            idx, ch = proto.dwell_at(self.dwell_plan, self.dwell_s, now + lead)
            t_start = (idx + 1) * self.dwell_s
            try:
                self.pub_dwell.put(proto.encode(proto.dwell_message(
                    seq=idx, t_start_utc_ns=int(t_start * 1e9),
                    fc_hz=ch.get("fc_hz", self.cfg.radio.get("fc_hz", 2.437e9)),
                    fs_sps=ch.get("fs_sps", self.cfg.radio.get("fs_sps", 2.4e6)),
                    bw_hz=ch.get("bw_hz", self.cfg.radio.get("bw_hz", 2.0e6)),
                    dwell_s=self.dwell_s, plan=self.dwell_plan)))
            except Exception as e:
                print(f"[super] dwell publish failed: {e}")
            time.sleep(period)

    def _serve_spectrum_http(self, port):
        """The RF dashboard, on its own port.

        A second server rather than another route on 8080, because it is a
        second *screen*: the 3D console and this are meant to sit on two
        monitors at once, and a browser will happily keep both SSE streams
        open. Keeping them on separate ports also means the spectrum feed
        cannot stall the console's stream, and either can be firewalled
        without the other.
        """
        sup = self
        here = os.path.dirname(os.path.abspath(__file__))

        class S(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path.startswith("/spec/events"):
                    q = queue.Queue(maxsize=200)
                    sup.spec_subscribers.append(q)
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    try:
                        # Snapshot first, so a tab opened now shows a filled
                        # waterfall instead of a minute of black. Capped
                        # because the full history for ten nodes is most of a
                        # megabyte and the oldest rows are the least useful.
                        init = {
                            "type": "init",
                            "nodes": [n["id"] for n in sup.cfg.nodes],
                            "used": list(sup.last_used),
                            # The energy gate the nodes themselves apply, so
                            # the display can mark "this sensor is hearing
                            # something" against the same number the fleet
                            # decides on rather than a threshold invented
                            # for the UI.
                            "thresh_db": float((sup.cfg.radio or {})
                                               .get("detect_threshold_db", 8.0)),
                            # so the dashboard can offer the scan's channels
                            # by name and state the span being covered
                            "plan": list(sup.dwell_plan),
                            "dwell_s": sup.dwell_s,
                            "history": {k: list(v)[-120:]
                                        for k, v in sup.spectra.items()},
                        }
                        self.wfile.write(
                            f"data: {json.dumps(init, default=str)}\n\n".encode())
                        self.wfile.flush()
                        while True:
                            try:
                                data = q.get(timeout=15)
                                self.wfile.write(f"data: {data}\n\n".encode())
                            except queue.Empty:
                                self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                    except Exception:
                        pass
                    finally:
                        if q in sup.spec_subscribers:
                            sup.spec_subscribers.remove(q)
                    return

                path = "/spectrum.html" if self.path in ("/", "") else self.path
                rel = urlparse(path).path.lstrip("/")
                fn = os.path.normpath(os.path.join(here, "console", rel))
                if not fn.startswith(os.path.join(here, "console")) \
                        or not os.path.isfile(fn):
                    self.send_response(404)
                    self.end_headers()
                    return
                with open(fn, "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", content_type(fn))
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store, must-revalidate")
                self.end_headers()
                self.wfile.write(body)

        srv = ThreadingHTTPServer(("0.0.0.0", port), S)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[super] spectrum dashboard on http://localhost:{port}")

    def _serve_radar_page(self, port):
        """Every radar range's own console on one page (console/radars.html).

        The consoles themselves are served by the radar nodes, on the radar
        PCs (8082 range A, 8083 range B); each announces its URL in health.
        This is the one place that knows all of them, so it serves the page
        that frames them side by side, plus /radars.json for it to poll.
        Its own port for the same reason as the spectrum dashboard: a
        separate screen."""
        sup = self
        here = os.path.dirname(os.path.abspath(__file__))

        class S(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store, must-revalidate")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/radars.json":
                    rows = [{k: r.get(k) for k in ("id", "state", "status",
                                                    "console_url", "rpm",
                                                    "range_scale_m", "enabled",
                                                    "fused")}
                            for r in sup._radar_summary()]
                    self._send(json.dumps(rows).encode(), "application/json")
                    return
                if path in ("/", "/index.html", "/radars.html"):
                    with open(os.path.join(here, "console", "radars.html"), "rb") as f:
                        self._send(f.read(), "text/html; charset=utf-8")
                    return
                self.send_response(404)
                self.end_headers()

        srv = ThreadingHTTPServer(("0.0.0.0", port), S)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[super] radar consoles, one page, on http://localhost:{port}")

    def run(self):
        print(f"[super] clock correction {'ON' if self.correct_clocks else 'OFF'} | "
              f"GPU {'yes' if gpu_available() else 'no'}")
        try:
            while True:
                time.sleep(5)
                try:
                    self._print_stats()
                except Exception as e:      # never let a report kill the node
                    print(f"[super] stats report failed: {type(e).__name__}: {e}")
        except KeyboardInterrupt:
            pass
        finally:
            self.session.close()

    def _print_stats(self):
        s = self._stat_summary()
        extra = ""
        if s["err_h_p50"] is not None:
            extra = (f" | err_h_p50 {s['err_h_p50']:.1f} m"
                     f" err_v_p50 {s['err_v_p50']:.1f} m")
        if s["starved"] or s["tight_updates"] or s["tight_rejected"]:
            extra += (f" | starved {s['starved']}"
                      f" tight {s['tight_updates']}"
                      f"/{s['tight_rejected']} rej")
        for r in s["radars"]:
            fc = s["fusion"]["counters"]
            extra += (f" | radar {r['id']} {r['state']}"
                      f" {r['plot_rate_hz']:.1f} plots/s"
                      f" assoc {fc.get('radar_assoc', 0)}"
                      f" rej_fix {fc.get('tdoa_rejected', 0)}"
                      f" lag {s['fusion']['lag_s']:.1f} s")
            cal = r.get("cal") or {}
            if cal.get("suspect"):
                # the calibration monitor disagrees with the site file
                extra += (f" CALIBRATION? az bias {cal['az_bias_deg']:+.2f} deg"
                          f" clock {cal['time_offset_ms']:+.0f} ms vs TDOA")
        print(f"[super] bursts {s['bursts']} fixes {s['fixes']} "
              f"| ingest {s['ingest_mbps']:.2f} Mbps "
              f"| pull {s['pull_ms']:.0f} ms solve {s['solve_ms']:.0f} ms{extra}")


args_config_path = ["config/site.json"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--spec-port", type=int, default=8081,
                    help="RF spectrum dashboard; 0 disables it")
    ap.add_argument("--radar-page-port", type=int, default=8084,
                    help="every radar range's console on one page; 0 disables it")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--no-clock-correction", action="store_true",
                    help="show what uncalibrated clocks do to the solution")
    ap.add_argument("--save-iq", metavar="DIR",
                    help="keep every solved burst's IQ + events for --replay-iq")
    ap.add_argument("--save-iq-all", action="store_true",
                    help="with --save-iq: also pull hearers the selection skipped")
    ap.add_argument("--replay-iq", metavar="DIR",
                    help="solve saved bursts instead of listening to nodes")
    ap.add_argument("--replay-speed", type=float, default=1.0)
    ap.add_argument("--replay-wait", type=float, default=3.0,
                    help="seconds to wait for subscribers before replaying")
    zconf.add_args(ap)
    a = ap.parse_args()
    args_config_path[0] = a.config
    cfg = proto.SiteConfig.load(a.config)
    zc = zconf.hub(a.port, a.bind, zconf.tls_from_args(a))
    sup = Supernode(cfg, correct_clocks=not a.no_clock_correction,
                    http_port=a.http_port, zc=zc, spec_port=a.spec_port,
                    radar_page_port=a.radar_page_port)
    if a.save_iq:
        os.makedirs(a.save_iq, exist_ok=True)
        sup.save_iq_dir, sup.save_iq_all = a.save_iq, a.save_iq_all
        print(f"[super] saving solved bursts to {a.save_iq}"
              + (" (all hearers)" if a.save_iq_all else ""))
    if a.replay_iq:
        threading.Thread(target=sup.replay_iq,
                         args=(a.replay_iq, a.replay_speed, a.replay_wait),
                         daemon=True).start()
    sup.run()


if __name__ == "__main__":
    main()
