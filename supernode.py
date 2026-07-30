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
import collections
import json
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import zenoh

from dronelocate import proto, zconf
from dronelocate.geo import C_LIGHT, LocalFrame, gdop
from dronelocate.sigsim import dequantize
from dronelocate.tdoa import cross_correlate, solve_tdoa, timing_sigma, gpu_available


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


class Tracker:
    """Constant-velocity Kalman filter in ENU.

    Deliberately simple. A particle filter is the right answer for initial
    acquisition because the TDOA cost surface is multimodal, but the grid
    seed inside solve_tdoa already handles that, so by the time a fix
    reaches here it is unimodal and a linear filter is appropriate.
    """

    def __init__(self, tid, fix, t):
        self.tid = tid
        self.x = np.concatenate([fix.enu, np.zeros(3)])
        self.P = np.eye(6) * 1e4
        self.P[:3, :3] = fix.cov
        self.P[3:, 3:] = np.eye(3) * 400.0   # 20 m/s 1-sigma initial speed
        self.t = t
        self.hits = 1
        self.misses = 0
        self.history = collections.deque(maxlen=400)
        self.history.append((t, fix.enu.copy()))

    def predict(self, t):
        dt = max(t - self.t, 1e-3)
        f = np.eye(6)
        f[0, 3] = f[1, 4] = f[2, 5] = dt
        # Anisotropic process noise. A quadrotor manoeuvres hard laterally but
        # climbs slowly, and the vertical is the axis the geometry constrains
        # worst -- isotropic Q lets a weakly observed altitude random-walk
        # until the filtered track is worse than the raw fixes feeding it.
        acc_h, acc_v = 6.0 ** 2, 1.5 ** 2
        g2 = np.array([dt ** 2 / 2] * 3 + [dt] * 3) ** 2
        q = np.diag(g2 * np.array([acc_h, acc_h, acc_v, acc_h, acc_h, acc_v]))
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.t = t

    def update(self, fix, t, alt_prior=None, alt_sigma=60.0):
        self.predict(t)
        h = np.zeros((3, 6))
        h[:3, :3] = np.eye(3)
        y = fix.enu - h @ self.x
        s = h @ self.P @ h.T + fix.cov
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(6) - k @ h) @ self.P

        # Same altitude prior the solver uses, reapplied here as a scalar
        # pseudo-measurement. Without it the filter integrates a long run of
        # weak vertical measurements into a confident drift.
        if alt_prior is not None:
            hz = np.zeros((1, 6)); hz[0, 2] = 1.0
            sz = hz @ self.P @ hz.T + np.array([[alt_sigma ** 2]])
            kz = self.P @ hz.T @ np.linalg.inv(sz)
            self.x = self.x + (kz @ np.array([[alt_prior - self.x[2]]])).ravel()
            self.P = (np.eye(6) - kz @ hz) @ self.P

        self.hits += 1
        self.misses = 0
        self.history.append((t, self.x[:3].copy()))

    @property
    def pos(self):
        return self.x[:3]

    @property
    def vel(self):
        return self.x[3:]


class Supernode:
    def __init__(self, cfg, correct_clocks=True, http_port=8080, zc=None):
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

        # Sim-only: which clock regime the emulated nodes are running. Pushed
        # to the nodes over the existing retasking channel rather than being
        # a supernode-side fiction, so what you switch is genuinely the
        # sensors' clocks and the supernode still has to cope with the result.
        self.clock_mode = str((cfg.sim or {}).get("clock_discipline", "gpsdo"))

        self.node_enu = {n["id"]: np.array(n["enu"], dtype=float) for n in cfg.nodes}
        self.pending = collections.defaultdict(dict)   # burst -> node -> event
        self.pending_t = {}
        self.lock = threading.Lock()

        self.tracks = {}
        self.next_tid = 1
        self.truth = None
        self.health = {}
        self.stats = {"bursts": 0, "fixes": 0, "iq_pulled": 0, "bytes_pulled": 0,
                      "solve_ms": collections.deque(maxlen=50),
                      "pull_ms": collections.deque(maxlen=50),
                      "started": time.time()}
        self.errors = collections.deque(maxlen=100)
        self.subscribers = []

        self.session = zenoh.open(zc or zenoh.Config())
        site = cfg.site
        self.session.declare_subscriber(proto.ke_detect(site), self._on_event)
        self.session.declare_subscriber(proto.ke_health(site), self._on_health)
        self.session.declare_subscriber(proto.ke_truth(site), self._on_truth)
        self.pub_track = self.session.declare_publisher(
            proto.ke_track(site, "live"),
            priority=zenoh.Priority.DATA_HIGH,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )

        self.work = queue.Queue()
        for _ in range(3):
            threading.Thread(target=self._worker, daemon=True).start()
        threading.Thread(target=self._reaper, daemon=True).start()
        self._serve_http(http_port)

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
            self.health[h["node"]] = h
        except Exception:
            pass

    def _on_truth(self, sample):
        """Scoring only. Nothing in the solve path reads this."""
        try:
            self.truth = proto.decode(sample.payload)
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

    def _process(self, burst, events):
        self.stats["bursts"] += 1
        chosen = self._select(events)

        t_pull = time.time()
        iq, meta = {}, {}
        for nid in chosen:
            payload, m = self._pull(nid, events[nid]["cap_id"])
            if payload is None:
                continue
            n = events[nid]["n_samples"]
            iq[nid] = dequantize(payload, events[nid]["fmt"], n)
            meta[nid] = m
            self.stats["iq_pulled"] += 1
            self.stats["bytes_pulled"] += len(payload)
        pull_ms = (time.time() - t_pull) * 1000.0
        self.stats["pull_ms"].append(pull_ms)

        if len(iq) < int(self.solver.get("min_nodes", 5)):
            return

        # reference = strongest of the pulled set
        ref = max(iq.keys(), key=lambda i: events[i]["snr_db"])
        fs = float(events[ref]["fs_sps"])
        bw = float(events[ref]["bw_hz"])

        pts = np.array([self.node_enu[i] for i in iq])
        baseline = float(np.max(np.linalg.norm(pts[:, None] - pts[None, :], axis=2)))
        max_lag = baseline / C_LIGHT + 5e-6

        t_solve = time.time()
        others, tdoa, sig, quality = [], [], [], []
        for nid in iq:
            if nid == ref:
                continue
            # The peak sits at (geometric TDOA + clock offset difference).
            # max_lag only bounds the geometric part, so the search has to be
            # told about the other half or the peak falls outside it once
            # drift has accumulated -- minutes, not hours.
            clk_delta = (events[nid]["clk_off_ns"]
                         - events[ref]["clk_off_ns"]) * 1e-9
            if self.correct_clocks:
                c = cross_correlate(iq[nid], iq[ref], fs, max_lag_s=max_lag,
                                    center_lag_s=clk_delta)
                # remove the per-node clock error the calibration loop
                # recovered; leaving it in is the classic silent failure
                lag = c.lag_s - clk_delta
            else:
                # Uncalibrated demo: we are pretending not to know the offset,
                # so we may not centre on it. Widen instead, otherwise the
                # failure on screen is the correlator losing the peak rather
                # than the solver being fed a biased TDOA, which is the point.
                c = cross_correlate(iq[nid], iq[ref], fs,
                                    max_lag_s=max_lag + abs(clk_delta))
                lag = c.lag_s

            if self.quality_gate and c.quality < self.quality_min:
                # Drop the measurement, not the burst -- node selection already
                # picked a GDOP-redundant subset, so losing one still solves.
                self.stats["rejected"] = self.stats.get("rejected", 0) + 1
                self.stats.setdefault("rejected_by_node", collections.Counter())
                self.stats["rejected_by_node"][nid] += 1
                continue

            others.append(nid)
            tdoa.append(lag)
            sig.append(timing_sigma(c, fs, bw))
            quality.append(c.quality)

        # The min_nodes check upstream ran before correlation, so it cannot
        # know about gate rejections. Re-check here or a burst that lost most
        # of its measurements goes to the solver with too little to constrain
        # three unknowns.
        if len(tdoa) < 3:
            self.stats["starved"] = self.stats.get("starved", 0) + 1
            return

        fix = solve_tdoa(
            np.array([self.node_enu[i] for i in others]),
            self.node_enu[ref],
            np.array(tdoa), sigma_s=np.array(sig),
            alt_prior_m=self.solver.get("alt_prior_m", 100.0),
            alt_prior_sigma_m=self.solver.get("alt_prior_sigma_m", 60.0),
            robust=self.robust,
        )
        solve_ms = (time.time() - t_solve) * 1000.0
        self.stats["solve_ms"].append(solve_ms)
        if fix is None:
            return
        self.stats["fixes"] += 1

        now = time.time()
        if not self.tracks:
            self.tracks[self.next_tid] = Tracker(self.next_tid, fix, now)
            self.next_tid += 1
        else:
            tid = min(self.tracks, key=lambda k:
                      np.linalg.norm(self.tracks[k].pos - fix.enu))
            if np.linalg.norm(self.tracks[tid].pos - fix.enu) > 2500.0:
                self.tracks[self.next_tid] = Tracker(self.next_tid, fix, now)
                self.next_tid += 1
            else:
                self.tracks[tid].update(
                    fix, now,
                    alt_prior=self.solver.get("alt_prior_m"),
                    alt_sigma=self.solver.get("alt_prior_sigma_m", 60.0))

        self._emit(burst, fix, chosen, others, ref, quality, pull_ms, solve_ms)

    def _emit(self, burst, fix, chosen, others, ref, quality, pull_ms, solve_ms):
        tid = max(self.tracks)
        trk = self.tracks[tid]
        lat, lon, alt = self.frame.to_geodetic(trk.pos)

        err = None
        if self.truth is not None:
            t = np.array(self.truth["enu"], dtype=float)
            d = fix.enu - t
            err = {"h_m": float(np.hypot(d[0], d[1])), "v_m": float(d[2]),
                   "total_m": float(np.linalg.norm(d))}
            self.errors.append(err)

        msg = {
            "t": time.time(), "burst": burst, "tid": tid,
            "enu": [float(v) for v in trk.pos],
            "vel": [float(v) for v in trk.vel],
            "lat": lat, "lon": lon, "alt_m": alt,
            "raw_enu": [float(v) for v in fix.enu],
            "sigma_h": fix.sigma_h, "sigma_v": fix.sigma_v,
            "cov": [[float(c) for c in row] for row in fix.cov],
            "hdop": fix.hdop, "vdop": fix.vdop,
            "resid_ns": fix.residual_rms_s * 1e9,
            "chi2": fix.detail.get("chi2_red", 0.0),
            "n_meas": fix.n_meas, "ref": ref,
            "nodes_heard": len(chosen), "nodes_used": len(others) + 1,
            # Ordered to match corr_quality element for element. Without the
            # ids a consumer gets a bare list of numbers it cannot attribute
            # to anything, which is why the console could not draw these.
            "used": list(others),
            "heard": list(chosen),
            "corr_quality": [float(q) for q in quality],
            "pull_ms": pull_ms, "solve_ms": solve_ms,
            "clock_correction": self.correct_clocks,
            "truth": self.truth, "error": err,
        }
        self.pub_track.put(proto.encode(msg))
        self._push(msg)

    # --- console telemetry (SSE) ----------------------------------------
    def _push(self, msg):
        payload = json.dumps({"type": "track", "data": msg,
                              "health": self.health,
                              "stats": self._stat_summary()}, default=str)
        dead = []
        for q in self.subscribers:
            try:
                q.put_nowait(payload)
            except Exception:
                dead.append(q)
        for q in dead:
            self.subscribers.remove(q)

    def _stat_summary(self):
        up = time.time() - self.stats["started"]
        errs = list(self.errors)
        return {
            "uptime_s": up, "bursts": self.stats["bursts"],
            "fixes": self.stats["fixes"], "iq_pulled": self.stats["iq_pulled"],
            "mb_pulled": self.stats["bytes_pulled"] / 1e6,
            "ingest_mbps": self.stats["bytes_pulled"] * 8 / 1e6 / max(up, 1e-6),
            "dropped_bursts": self.stats.get("dropped_bursts", 0),
            "solve_ms": float(np.mean(self.stats["solve_ms"])) if self.stats["solve_ms"] else 0,
            "pull_ms": float(np.mean(self.stats["pull_ms"])) if self.stats["pull_ms"] else 0,
            "err_h_p50": float(np.median([e["h_m"] for e in errs])) if errs else None,
            "err_v_p50": float(np.median([abs(e["v_m"]) for e in errs])) if errs else None,
            "gpu": gpu_available(),
            # solver switches, echoed so the console reflects real server
            # state rather than whatever the last click optimistically assumed
            "quality_gate": self.quality_gate,
            "quality_min": self.quality_min,
            "robust": self.robust,
            "clock_mode": self.clock_mode,
            "rejected": self.stats.get("rejected", 0),
            "starved": self.stats.get("starved", 0),
            "rejected_by_node": dict(self.stats.get("rejected_by_node", {})),
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
                    if "quality_min" in q:
                        try:
                            sup.quality_min = float(q["quality_min"][0])
                        except ValueError:
                            pass
                    if "clock" in q:
                        sup.set_clock_mode(q["clock"][0])
                    body = json.dumps({"quality_gate": sup.quality_gate,
                                       "robust": sup.robust,
                                       "quality_min": sup.quality_min,
                                       "clock_mode": sup.clock_mode}).encode()
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
                    }).encode()
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
                self.end_headers()
                self.wfile.write(body)

        srv = ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[super] console on http://localhost:{port}")

    def run(self):
        print(f"[super] clock correction {'ON' if self.correct_clocks else 'OFF'} | "
              f"GPU {'yes' if gpu_available() else 'no'}")
        try:
            while True:
                time.sleep(5)
                s = self._stat_summary()
                extra = ""
                if s["err_h_p50"] is not None:
                    extra = (f" | err_h_p50 {s['err_h_p50']:.1f} m"
                             f" err_v_p50 {s['err_v_p50']:.1f} m")
                print(f"[super] bursts {s['bursts']} fixes {s['fixes']} "
                      f"| ingest {s['ingest_mbps']:.2f} Mbps "
                      f"| pull {s['pull_ms']:.0f} ms solve {s['solve_ms']:.0f} ms{extra}")
        except KeyboardInterrupt:
            pass
        finally:
            self.session.close()


args_config_path = ["config/site.json"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--no-clock-correction", action="store_true",
                    help="show what uncalibrated clocks do to the solution")
    zconf.add_args(ap)
    a = ap.parse_args()
    args_config_path[0] = a.config
    cfg = proto.SiteConfig.load(a.config)
    zc = zconf.hub(a.port, a.bind, zconf.tls_from_args(a))
    Supernode(cfg, correct_clocks=not a.no_clock_correction,
              http_port=a.http_port, zc=zc).run()


if __name__ == "__main__":
    main()
