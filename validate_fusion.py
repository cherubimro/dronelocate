"""Radar + TDOA track fusion against known truth, with no transport in the way.

The offline counterpart of the live sim, like validate_math.py is for the
solver: a synthetic trajectory, TDOA fixes from dronelocate's own all-pairs
GLS solver (real covariance, real altitude prior), radar plots from the
same emulator radarnode.py --source sim uses (dronelocate/radarsim.py), and
dronelocate/fusion.py driven by a simulated wall clock that reproduces the
supernode's arrival order: a fix reaches the track layer 0.6-0.8 s after
capture, a radar batch one sector after its sector closes.

Every scenario is scored against truth at the track's own state time.
Three configurations share one set of measurements per scenario:

    tdoa-only   radar configured but silent -- the supernode with no
                radarnode running
    radar-only  no fixes -- a drone that does not emit
    fused       both

The clock starts at a real epoch (1.79e9 s), not zero: bug 10 is what a
synthetic test that starts at zero cannot see.

Assumptions, stated because the numbers depend on them:
- 10-node site: per-node timing error 0.5 ns + 0.5 ns per pair, which with
  the 2.4 Msps interpolator floor in the reported variance gives the live
  pipeline's reduced chi-square (~0.4) and its 0.2 m horizontal error.
- 4-node (field) site: per-node error 5 ns after calibration,
  reported honestly -- a guess, the TinyB210 PPS path is unmeasured
  (hw_selftest.py). The radar there is given az_offset_deg = 12 as if
  calibrated; the shipped config has null, which fuses nothing.
- Radar: 0.5 deg azimuth noise, 1 cell range noise, plots advertising the
  detector's own (larger) sigmas, Pd 0.9 inside the good fan, rotation
  alternating 24 / 41 rpm.
"""

import json
import math
import sys

import numpy as np

from dronelocate.fusion import Fusion
from dronelocate.geo import C_LIGHT, gdop
from dronelocate.radarsim import SECTORS, SimRadar
from dronelocate.tdoa import interpolator_floor, pair_covariance, solve_tdoa_gls
from dronelocate.track import Tracker

EPOCH = 1.79e9
WARMUP_S = 20.0

SITE10 = json.load(open("config/site.json"))
NODES10 = np.array([n["enu"] for n in SITE10["nodes"]], dtype=float)
RADAR10 = dict(SITE10["radars"][0])
FIELD = json.load(open("config/site-field.json"))
NODES5 = np.array([n["enu"] for n in FIELD["nodes"]], dtype=float)
NODES4 = NODES5[:4]
RADAR4 = dict(FIELD["radars"][0])
RADAR4["az_offset_deg"] = 12.0          # as if calibrated; config has null


# --- trajectories ---------------------------------------------------------

def orbit(c, r, period, climb):
    c = np.asarray(c, dtype=float)

    def pos(t):
        th = 2.0 * math.pi * t / period
        return np.array([c[0] + r * math.cos(th), c[1] + r * math.sin(th),
                         c[2] + climb * math.sin(0.5 * th)])
    return with_vel(pos)


def line(a, b, speed):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    u = (b - a) / np.linalg.norm(b - a)
    return with_vel(lambda t: a + u * speed * t)


def with_vel(pos):
    def f(t):
        return pos(t), (pos(t + 0.05) - pos(t - 0.05)) / 0.1
    return f


ORBIT10 = orbit([600.0, -400.0, 95.0], 900.0, 180.0, 35.0)     # = scene.py's
ORBIT4 = orbit([0.0, 800.0, 60.0], 300.0, 120.0, 10.0)        # in the field box
ORBIT10B = orbit([600.0, -400.0, 110.0], 750.0, 180.0, 20.0)  # 150 m inside ORBIT10


# --- measurement streams ----------------------------------------------------

class Tdoa:
    """Bursts at 2 Hz -> GLS fixes (or tight-coupling rows when only three
    nodes hear), with the solver the supernode runs."""

    def __init__(self, nodes, node_sigma, pair_sigma, fs, alt_prior, alt_sigma,
                 max_nodes=7, min_nodes=4, seed=0):
        self.nodes, self.ns, self.ps, self.fs = nodes, node_sigma, pair_sigma, fs
        self.alt_prior, self.alt_sigma = alt_prior, alt_sigma
        self.max_nodes, self.min_nodes = max_nodes, min_nodes
        self.rng = np.random.default_rng(seed)

    def select(self, p, ids):
        if len(ids) <= self.max_nodes:
            return list(ids)
        seed = np.array([p[0], p[1], self.alt_prior])
        chosen = [ids[0]]
        while len(chosen) < self.max_nodes:
            best, bp = None, np.inf
            for c in ids:
                if c in chosen:
                    continue
                trial = chosen + [c]
                if len(trial) < 4:
                    best = c
                    break
                _, _, pd = gdop(self.nodes[trial], seed, 0)
                if pd < bp:
                    best, bp = c, pd
            chosen.append(best)
        return chosen

    def burst(self, p, avail, corrupt_s=0.0, corrupt_node=3):
        ids = self.select(p, [i for i in range(len(self.nodes)) if avail(i)])
        if len(ids) < 3:
            return None
        e = self.rng.normal(0.0, self.ns, len(self.nodes))
        if corrupt_s:
            # A lost peak at one node: every correlation against it locks
            # onto the same wrong lobe (a reflection, a sidelobe). With four
            # nodes that is still self-consistent -- three unknowns, three
            # independent differences -- so chi-square cannot see it and
            # the fix comes back confident and wrong.
            e[corrupt_node] += corrupt_s
        pairs, lags, var = [], [], []
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                geo = (np.linalg.norm(p - self.nodes[i])
                       - np.linalg.norm(p - self.nodes[j])) / C_LIGHT
                pairs.append([a, b])
                lags.append(geo + e[i] - e[j] + self.rng.normal(0.0, self.ps))
                var.append(2 * self.ns ** 2 + self.ps ** 2
                           + interpolator_floor(self.fs) ** 2)
        lags = np.array(lags)
        pairs = np.array(pairs, dtype=int)
        pos = self.nodes[ids]
        R = pair_covariance(pairs, np.array(var), len(ids), interpolator_floor(self.fs))
        if len(ids) < self.min_nodes:
            return ("tight", (pos[pairs[:, 0]], pos[pairs[:, 1]], lags, R,
                              np.array(var)))
        fix = solve_tdoa_gls(pos[pairs[:, 0]], pos[pairs[:, 1]], lags, R,
                             alt_prior_m=self.alt_prior,
                             alt_prior_sigma_m=self.alt_sigma, robust=True)
        return None if fix is None else ("fix", fix)


def tdoa_events(tdoa, traj, t_end, avail=lambda i, t: True, corrupt=None,
                seed=1, emit=lambda t: True, latency=(0.6, 0.8), label="uas"):
    rng = np.random.default_rng(seed)
    ev = []
    t = 0.0
    while t < t_end:
        tm = t + rng.uniform(0.0, 0.02)
        if emit(tm):
            p, _ = traj(tm)
            c = corrupt(tm) if corrupt else 0.0
            b = tdoa.burst(p, lambda i: avail(i, tm), corrupt_s=c)
            if b is not None:
                arrive = tm + rng.uniform(*latency)
                ev.append((EPOCH + arrive, b[0], EPOCH + tm, b[1], {"t_rel": tm,
                                                                     "corrupt": c,
                                                                     "label": label}))
        t += 0.5
    return ev


def radar_events(entry, params, targets, t_end, off_after=None, seed=2,
                 true_offset=None):
    """Batches + health from the shared emulator. targets: list of
    (label, traj) or (label, traj, pd) in relative time. true_offset: the
    mounting offset the radar really has, when it is not the one the site
    config (entry) claims."""
    sim = SimRadar(dict(params, az_offset_true_deg=entry["az_offset_deg"]
                        if true_offset is None else true_offset),
                   entry["id"], entry["enu"], seed=seed)
    tg = [(tg_[0], (lambda tr: lambda t: tr(t - EPOCH))(tg_[1]),
           tg_[2] if len(tg_) > 2 else sim.p["pd"]) for tg_ in targets]
    ev = []
    t, sector, rev, total = 0.0, 0, 0, 0
    next_h = 0.0
    stop = t_end if off_after is None else off_after
    while t < stop:
        rpm = sim.rpm_at(t)
        dur = 60.0 / rpm / SECTORS
        sim.step_birds(EPOCH + t + dur)
        plots = sim.render(sector, EPOCH + t, EPOCH + t + dur, tg)
        total += len(plots)
        b = sim.batch(sector, rev, EPOCH + t + dur, plots, rpm, entry["az_offset_deg"])
        ev.append((EPOCH + t + 2 * dur + 0.01, "batch", None, b, {}))
        while next_h <= t + dur:
            ev.append((EPOCH + next_h + 0.005, "health", None,
                       sim.health(EPOCH + next_h, "transmit", rpm, rev, total), {}))
            next_h += 1.0
        t += dur
        sector += 1
        if sector == SECTORS:
            sector, rev = 0, rev + 1
    return ev, sim


def legacy_track_layer(events, alt_prior, alt_sigma):
    """The track layer as supernode.py had it before fusion, verbatim in
    logic: nearest track within 2.5 km, Tracker.update, tight rows to the
    smallest normalised residual -- applied in arrival order (what the
    track_lock produced). Returns [(tid, x)] per fix."""
    tracks, nxt, out = {}, 1, []
    for arrive, kind, tm, payload, info in sorted(events, key=lambda e: e[0]):
        if kind == "fix":
            fix = payload
            tid = None
            if tracks:
                tid = min(tracks, key=lambda k: np.linalg.norm(tracks[k].pos - fix.enu))
                if np.linalg.norm(tracks[tid].pos - fix.enu) > 2500.0:
                    tid = None
            if tid is None:
                tid = nxt
                tracks[tid] = Tracker(tid, fix, tm)
                nxt += 1
            else:
                tracks[tid].update(fix, tm, alt_prior=alt_prior, alt_sigma=alt_sigma)
            out.append((tid, tracks[tid].x.copy()))
        elif kind == "tight" and tracks:
            pa, pb, lags, R, var = payload

            def misfit(trk):
                p = trk.pos
                pred = (np.linalg.norm(p - pa, axis=1) - np.linalg.norm(p - pb, axis=1)) / C_LIGHT
                return float(np.sum((lags - pred) ** 2 / var))
            min(tracks.values(), key=misfit).update_tdoa(pa, pb, lags, R, tm)
    return out


def run(entry, events, fusion_cfg, alt_prior, alt_sigma, t_end, tick=0.05,
        wake_on_submit=True):
    """Drive Fusion the way the supernode does: submit on arrival, wake on
    submit and on a short tick, process what is due. Returns the outputs,
    each stamped with the (simulated) wall time it was produced at.
    wake_on_submit=False lets several arrivals queue between wakeups (a
    busy owner thread), which is where processing order can differ."""
    f = Fusion(cfg=fusion_cfg, radars=[entry], alt_prior_m=alt_prior,
               alt_prior_sigma_m=alt_sigma)
    events = sorted(events, key=lambda e: e[0])
    outs = []
    now = EPOCH

    def step(now):
        for o in f.process_due(now):
            o["now"] = now
            outs.append(o)

    for arrive, kind, tm, payload, info in events:
        while now + tick < arrive:
            now += tick
            step(now)
        now = arrive
        if kind == "fix":
            f.submit_fix(tm, payload, meta=dict(info, arrived=arrive), now=arrive)
        elif kind == "tight":
            pa, pb, lags, R, var = payload
            f.submit_tight(tm, pa, pb, lags, R, var=var, now=arrive)
        elif kind == "batch":
            f.submit_batch(payload, now=arrive)
        elif kind == "health":
            f.on_radar_health(payload, now=arrive)
        if wake_on_submit:
            step(now)
    end = EPOCH + t_end + 3.0
    while now < end:
        now += tick
        step(now)
    return f, outs


# --- scoring ----------------------------------------------------------------

def score(outs, traj, kinds=("tdoa", "radar"), t_from=WARMUP_S, t_to=None,
          near_m=300.0):
    """Errors of confirmed track updates against truth at t_state. Updates
    farther than near_m from the target are not its track (scored
    separately as false/other tracks)."""
    eh, ev, evel, tids, far = [], [], [], set(), set()
    for o in outs:
        if o["kind"] not in kinds or o["track"]["state"] == "tentative":
            continue
        tr = o["track"]
        ts = tr["t_state"] - EPOCH
        if ts < t_from or (t_to is not None and ts > t_to):
            continue
        p, v = traj(ts)
        d = tr["x"][:3] - p
        if math.hypot(d[0], d[1]) > near_m:
            far.add(tr["tid"])
            continue
        tids.add(tr["tid"])
        eh.append(math.hypot(d[0], d[1]))
        ev.append(abs(d[2]))
        evel.append(float(np.linalg.norm(tr["x"][3:] - v)))
    q = lambda a, k: float(np.percentile(a, k)) if a else float("nan")
    return {"n": len(eh), "h50": q(eh, 50), "h90": q(eh, 90),
            "v50": q(ev, 50), "v90": q(ev, 90),
            "vel50": q(evel, 50), "vel90": q(evel, 90),
            "tids": tids, "far": far}


def fmt(s):
    return (f"n {s['n']:4d}  h p50 {s['h50']:7.2f} p90 {s['h90']:7.2f} m  "
            f"v p50 {s['v50']:7.2f} p90 {s['v90']:7.2f} m  "
            f"vel p50 {s['vel50']:5.2f} p90 {s['vel90']:5.2f} m/s  "
            f"tracks {sorted(s['tids'])}")


RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


RADAR_P = dict(RADAR10.get("sim") or {})
FUSION_CFG = dict(SITE10.get("fusion") or {})


def three_way(label, entry, tdoa_ev, radar_ev, traj, alt_prior, alt_sigma, t_end):
    res = {}
    for cfg_name, ev in (("tdoa-only", tdoa_ev), ("radar-only", radar_ev),
                         ("fused", tdoa_ev + radar_ev)):
        f, outs = run(entry, ev, FUSION_CFG, alt_prior, alt_sigma, t_end)
        res[cfg_name] = (score(outs, traj), f, outs)
        print(f"  {label:10s} {cfg_name:10s} {fmt(res[cfg_name][0])}")
    # Paired: the fused track at the same instants the TDOA-only one is
    # scored at (its fix updates). "All updates" adds the radar-update
    # outputs between fixes, which are a prediction plus one 1 deg plot and
    # are worse by construction, not because fusion degraded anything.
    s_at = score(res["fused"][2], traj, kinds=("tdoa",))
    res["fused@tdoa"] = (s_at, res["fused"][1], res["fused"][2])
    print(f"  {label:10s} {'fused@fix':10s} {fmt(s_at)}")
    print(f"  {label:10s} {'fused@plot':10s} "
          f"{fmt(score(res['fused'][2], traj, kinds=('radar',)))}")
    return res


def main():
    rng_seed = 7
    print("=== base: 10-node sim site, orbit as scene.py, radar 800 m outside ===")
    T = 180.0
    tdoa10 = Tdoa(NODES10, 0.5e-9, 0.5e-9, 2.4e6, 100.0, 60.0, max_nodes=7,
                  min_nodes=5, seed=rng_seed)
    ev_t10 = tdoa_events(tdoa10, ORBIT10, T)
    ev_r10, _ = radar_events(RADAR10, RADAR_P, [("uas", ORBIT10)], T)
    r10 = three_way("10-node", RADAR10, ev_t10, ev_r10, ORBIT10, 100.0, 60.0, T)
    t_, r_, f_ = (r10[k][0] for k in ("tdoa-only", "radar-only", "fused"))
    fa = r10["fused@tdoa"][0]
    # Ten nodes and the fixed solver give 0.1 m / 0.8 m / 0.6 m/s; a 1 deg,
    # 3 m radar plot at 1-2.5 km carries nothing that can improve on that,
    # so the claim here is only that fusion does NO HARM: same instants,
    # same fixes, within 5% + 2 cm (position) and 10% (velocity p90) --
    # the residue of the 1 s reorder window, nothing more.
    check("10-node fused no worse than TDOA-only (paired, at the fixes)",
          fa["h50"] <= 1.05 * t_["h50"] + 0.02 and fa["v50"] <= 1.05 * t_["v50"] + 0.02
          and fa["vel90"] <= 1.10 * t_["vel90"],
          f"h p50 {fa['h50']:.2f} vs {t_['h50']:.2f} m, v p50 {fa['v50']:.2f} vs "
          f"{t_['v50']:.2f} m, vel p90 {fa['vel90']:.2f} vs {t_['vel90']:.2f} m/s")
    # Radar-only horizontal: ~1 deg (the advertised sigma, rendered as
    # such) at 1-2.5 km is 17-44 m cross-range per plot, filtered over
    # revolutions on a 900 m-radius turn the CV model lags: 30 m p50.
    check("radar-only horizontal p50 < 30 m", r_["h50"] < 30.0,
          f"{r_['h50']:.1f} m p50, {r_['h90']:.1f} m p90")
    nis = r10["fused"][1].build_summary(EPOCH + T)["radar_nis_mean"]
    # chi2(2) mean is 2; the sim now renders the sigma each plot advertises
    check("radar NIS consistent (mean within 2 +/- 0.4)", abs(nis - 2.0) <= 0.4,
          f"mean NIS {nis:.2f}")
    check("10-node one track for the target (fused)", len(f_["tids"]) == 1,
          f"tids {sorted(f_['tids'])}, far {sorted(f_['far'])}")

    print("\n=== (a) four TDOA nodes (field box), radar at the pedestal ===")
    T4 = 240.0
    rp4 = dict(RADAR_P, range_scale_m=1500.0, max_det_range_m=1500.0)
    tdoa4 = Tdoa(NODES4, 5e-9, 0.2e-9, 20e6, 100.0, 100.0, max_nodes=5,
                 min_nodes=4, seed=rng_seed)
    ev_t4 = tdoa_events(tdoa4, ORBIT4, T4)
    ev_r4, _ = radar_events(RADAR4, rp4, [("uas", ORBIT4)], T4)
    r4 = three_way("4-node", RADAR4, ev_t4, ev_r4, ORBIT4, 100.0, 100.0, T4)
    t4, ro4, f4 = (r4[k][0] for k in ("tdoa-only", "radar-only", "fused"))
    # Four ground nodes leave the vertical to the 100 m prior (true altitude
    # ~60 m). Slant range carries altitude: at 500-1100 m and 3-8 deg
    # elevation dU/r ~0.05-0.13, so a 1.5 m range cell pins U to ~10-30 m.
    check("4-node fused vertical at least halves TDOA-only",
          f4["v50"] <= 0.5 * t4["v50"],
          f"fused {f4['v50']:.1f} m vs TDOA-only {t4['v50']:.1f} m p50")
    check("4-node fused horizontal no worse than TDOA-only",
          f4["h50"] <= 1.10 * t4["h50"] + 0.1,
          f"fused {f4['h50']:.2f} m vs TDOA-only {t4['h50']:.2f} m p50")

    print("\n=== (b) one of four nodes drops out for 20 s (t = 100-120 s) ===")
    drop = lambda i, t: not (i == 3 and 100.0 <= t < 120.0)
    ev_tb = tdoa_events(tdoa4, ORBIT4, T4, avail=drop, seed=3)
    n_tight = sum(1 for e in ev_tb if e[1] == "tight")
    for name, ev in (("tdoa-only", ev_tb), ("fused", ev_tb + ev_r4)):
        f, outs = run(RADAR4, ev, FUSION_CFG, 100.0, 100.0, T4)
        s_all = score(outs, ORBIT4)
        s_gap = score(outs, ORBIT4, kinds=("tdoa", "radar", "coast"),
                      t_from=100.0, t_to=125.0)
        print(f"  dropout    {name:10s} {fmt(s_all)}")
        print(f"             during gap  {fmt(s_gap)}  tight updates "
              f"{f.stats['tight_updates']}/{n_tight}")
        if name == "fused":
            check("dropout: one track throughout (fused)",
                  len(s_all["tids"]) == 1 and not s_all["far"],
                  f"tids {sorted(s_all['tids'])}, far {sorted(s_all['far'])}")
            # Tight rows + radar on a 15 m/s target: the worst case is the
            # CV model coasting between radar plots, ~2.5 s at 15 m/s on a
            # 300 m turn -> ~16 m of chord error before the next plot.
            check("dropout: horizontal p90 < 30 m during the gap (fused)",
                  s_gap["h90"] < 30.0, f"{s_gap['h90']:.1f} m p90 over "
                  f"{s_gap['n']} updates")
            check("dropout: tight coupling applied",
                  f.stats["tight_updates"] >= 0.8 * n_tight,
                  f"{f.stats['tight_updates']} of {n_tight} starved bursts")

    print("\n=== (c) non-emitting drone: radar only, no TDOA at all ===")
    f, outs = run(RADAR4, ev_r4, FUSION_CFG, 100.0, 100.0, T4)
    first_plot = min(p["t_utc_ns"] for e in ev_r4 if e[1] == "batch"
                     for p in e[3]["plots"]) * 1e-9 - EPOCH
    conf = [o for o in outs if o["kind"] == "radar" and o["track"]["state"] != "tentative"]
    t_conf = (conf[0]["track"]["t_state"] - EPOCH) if conf else float("inf")
    s_c = score(outs, ORBIT4, kinds=("radar",))
    t_rev = 60.0 / 24.0
    print(f"  first plot at {first_plot:.1f} s, confirmed at {t_conf:.1f} s "
          f"({(t_conf - first_plot) / t_rev:.1f} revolutions at 24 rpm)")
    print(f"  radar-only {fmt(s_c)}  alt_src {conf[-1]['track']['alt_src'] if conf else None}")
    # 3 of 4 (2 of 3 when Doppler-flagged) at 24-41 rpm: confirmation
    # within 4 revolutions = 10 s.
    check("non-emitting: confirmed within 4 revolutions",
          t_conf - first_plot <= 4 * t_rev + 0.1, f"{t_conf - first_plot:.1f} s")
    check("non-emitting: one track, horizontal p50 < 30 m",
          len(s_c["tids"]) == 1 and s_c["h50"] < 30.0,
          f"tids {sorted(s_c['tids'])}, {s_c['h50']:.1f} m p50")

    print("\n=== (d) a lost correlation peak at one node: a ~300 m wrong fix at t = 150 s ===")
    corrupt = lambda t: 1.5e-6 if 150.0 <= t < 150.4 else 0.0
    ev_td = tdoa_events(tdoa4, ORBIT4, T4, corrupt=corrupt, seed=4)
    i_bad = [k for k, e in enumerate(ev_td) if e[4].get("corrupt")]
    e_bad = ev_td[i_bad[0]]
    err_bad = float(np.linalg.norm(e_bad[3].enu[:2] - ORBIT4(e_bad[4]["t_rel"])[0][:2]))
    print(f"  solver output: {err_bad:.0f} m off, sigma_h {e_bad[3].sigma_h:.0f} m, "
          f"chi2 {e_bad[3].detail['chi2_red']:.0f} (the altitude prior is a 4th "
          f"equation, so chi-square sees it and the covariance says so)")
    # The case chi-square cannot see -- e.g. a burst left with exactly
    # three independent differences: same wrong position, the confident
    # covariance of a good fix.
    import copy
    conf_fix = copy.copy(e_bad[3])
    conf_fix.cov = ev_td[i_bad[0] - 1][3].cov.copy()
    ev_tc = list(ev_td)
    ev_tc[i_bad[0]] = e_bad[:3] + (conf_fix,) + e_bad[4:]
    print(f"  confident variant: same position, sigma_h {conf_fix.sigma_h:.1f} m")
    for variant, evt in (("as solved", ev_td), ("confident", ev_tc)):
        for name, ev in (("tdoa-only", evt), ("fused", evt + ev_r4)):
            f, outs = run(RADAR4, ev, FUSION_CFG, 100.0, 100.0, T4)
            s_after = score(outs, ORBIT4, t_from=150.0, t_to=153.0, near_m=1000.0)
            s_before = score(outs, ORBIT4, t_from=146.0, t_to=149.9)
            rej = any(o["kind"] == "rejected" and o["meta"].get("corrupt")
                      for o in outs)
            print(f"  {variant:9s} {name:10s} wrong fix {'REJECTED' if rej else 'applied '}  "
                  f"track h err before p90 {s_before['h90']:5.1f} m, over the 3 s "
                  f"after: p90 {s_after['h90']:6.1f} m  tracks "
                  f"{sorted(s_after['tids'] | s_after['far'])}")
            if name == "fused":
                # 10 m: the fused track's own 4-node horizontal p90 is ~3 m.
                check(f"wrong fix ({variant}) rejected by the radar-supported track",
                      rej and s_after["h90"] < 10.0 and len(s_after["tids"]) == 1,
                      f"{s_after['h90']:.1f} m p90 after")
            elif variant == "confident":
                check("TDOA-only is dragged by the confident one (what rejection buys)",
                      s_after["h90"] > 30.0, f"{s_after['h90']:.1f} m p90 after")

    print("\n=== (e) false plots and birds, 10 minutes ===")
    TE = 600.0
    for lam, birds in ((5.0, 0), (20.0, 0), (5.0, 3)):
        pe = dict(RADAR_P, false_per_rev=lam, birds=birds, pd_bird=0.5,
                  label_plots=True)
        ev_re, sim = radar_events(RADAR10, pe, [("uas", ORBIT10)], TE, seed=11)
        ev_te = tdoa_events(tdoa10, ORBIT10, TE, seed=12)
        f, outs = run(RADAR10, ev_te + ev_re, FUSION_CFG, 100.0, 60.0, TE)
        s = score(outs, ORBIT10, kinds=("tdoa", "radar", "coast"))
        # What each published track really is, from the labels the sim put
        # on the plots it rendered (a scoring aid the fusion never reads).
        lab = {}
        for o in outs:
            if o["kind"] == "plots":
                for p in o["plots"]:
                    tid_ = p["tid"] if p["tid"] is not None else p.get("seed_tid")
                    if tid_ is not None:
                        lab.setdefault(tid_, []).append(p.get("sim_label"))
        published = {o["track"]["tid"] for o in outs
                     if o["kind"] in ("tdoa", "radar", "coast")
                     and o["track"]["state"] != "tentative"}
        kind_of = {}
        for tid in published:
            ls = lab.get(tid, [])
            n_tdoa = max((o["track"]["n_tdoa"] for o in outs
                          if o["kind"] in ("tdoa", "radar", "coast")
                          and o["track"]["tid"] == tid), default=0)
            if n_tdoa or ls.count("uas") * 2 > len(ls):
                kind_of[tid] = "target"
            elif ls.count("false") * 2 <= len(ls):
                kind_of[tid] = "bird"      # mostly a bird's plots (seed included)
            else:
                kind_of[tid] = "false"
        n_k = {k: sorted(t for t, v in kind_of.items() if v == k)
               for k in ("target", "bird", "false")}
        n_plots = sum(1 for e in ev_re if e[1] == "batch" for p in e[3]["plots"])
        print(f"  {lam:4.0f} false/rev, {birds} birds: {n_plots} plots, "
              f"{f.stats['radar_new_tentative']} tentatives; published tracks: "
              f"target {n_k['target']}, bird {len(n_k['bird'])}, false {n_k['false']}; "
              f"clutter estimate {f.build_summary(EPOCH + TE)['clutter_per_rev']}")
        print(f"       target {fmt(s)}")
        if birds == 0:
            check(f"no false track at {lam:.0f} false plots/rev over 10 min",
                  not n_k["false"] and n_k["target"] == [1],
                  f"false {n_k['false']}, target {n_k['target']}")
        else:
            # A bird the radar sees on most revolutions IS a radar target:
            # nothing here can tell it from a non-emitting drone without
            # classification. Reported, not failed; these tracks carry
            # src ['radar'] and alt_src 'prior'.
            print(f"       ({len(n_k['bird'])} bird tracks -- expected: a bird seen "
                  f"on most revolutions is a real radar target)")
            check("birds: no false track, the target keeps one track",
                  not n_k["false"] and n_k["target"] == [1],
                  f"false {n_k['false']}, target {n_k['target']}")

    print("\n=== (f0) no radar at all: the track layer is the pre-fusion one ===")
    # Two targets 150 m apart, one pausing 20 s, with solve latency spread
    # enough to reorder arrivals (0.3-1.5 s against 0.5 s bursts) and an
    # owner thread that only wakes every 0.3 s, so several items queue.
    ev_two = (tdoa_events(tdoa10, ORBIT10, T, seed=31, latency=(0.3, 1.5), label="A")
              + tdoa_events(tdoa10, ORBIT10B, T, seed=32, latency=(0.3, 1.5), label="B",
                            emit=lambda t: not 60.0 <= t < 80.0))
    for label, evs, ap, asg, entry, kw in (
            ("10-node", ev_t10, 100.0, 60.0, RADAR10, {}),
            ("4-node dropout", ev_tb, 100.0, 100.0, RADAR4, {}),
            ("two targets, reordered, batched wakeups", ev_two, 100.0, 60.0, RADAR10,
             {"tick": 0.3, "wake_on_submit": False})):
        ref = legacy_track_layer(evs, ap, asg)
        f, outs = run(entry, evs, FUSION_CFG, ap, asg, T4, **kw)
        new = [(o["track"]["tid"], o["track"]["x"]) for o in outs if o["kind"] == "tdoa"]
        same_ids = [a[0] for a in ref] == [b[0] for b in new]
        dmax = max(float(np.max(np.abs(a[1] - b[1]))) for a, b in zip(ref, new)) \
            if len(ref) == len(new) else float("inf")
        print(f"  {label}: {len(new)} track updates vs {len(ref)} pre-fusion, "
              f"max state difference {dmax:.3g}")
        check(f"TDOA-only ({label}) bit-identical to the pre-fusion track layer",
              same_ids and dmax == 0.0 and not f.stats["merged"],
              f"{len(new)}/{len(ref)} updates, max |dx| {dmax:.3g}, "
              f"merged {f.stats['merged']}, tids {sorted({b[0] for b in new})}")

    print("\n=== (f) radar goes away halfway (t = 90 s): clean TDOA-only fallback ===")
    ev_rf, _ = radar_events(RADAR10, RADAR_P, [("uas", ORBIT10)], T, off_after=90.0)
    f, outs = run(RADAR10, ev_t10 + ev_rf, FUSION_CFG, 100.0, 60.0, T)
    dl_before = [o["now"] - o["meta"]["arrived"] for o in outs
                 if o["kind"] == "tdoa" and 30 < o["t"] - EPOCH < 85]
    dl_after = [o["now"] - o["meta"]["arrived"] for o in outs
                if o["kind"] == "tdoa" and o["t"] - EPOCH > 96]
    n_fix_in = sum(1 for e in ev_t10 if e[1] == "fix")
    n_fix_out = sum(1 for o in outs if o["kind"] in ("tdoa", "rejected"))
    s_after = score(outs, ORBIT10, t_from=96.0)
    s_ref = score(r10["tdoa-only"][2], ORBIT10, t_from=96.0)
    print(f"  hold before {np.mean(dl_before):.2f} s, after {np.mean(dl_after):.3f} s; "
          f"fixes in {n_fix_in}, applied {n_fix_out}")
    print(f"  after loss {fmt(s_after)}")
    print(f"  TDOA-only  {fmt(s_ref)}")
    check("fallback: every fix applied (no stall)", n_fix_out == n_fix_in,
          f"{n_fix_out}/{n_fix_in}")
    check("fallback: no lag once the radar is stale", np.max(dl_after) < 0.06,
          f"max hold {np.max(dl_after) * 1000:.0f} ms after, "
          f"{np.mean(dl_before) * 1000:.0f} ms mean before")
    check("fallback: same track, TDOA-only accuracy",
          len(s_after["tids"]) == 1 and s_after["h50"] <= 1.1 * s_ref["h50"] + 0.05,
          f"tids {sorted(s_after['tids'])}, {s_after['h50']:.2f} vs "
          f"{s_ref['h50']:.2f} m p50")

    print("\n=== (g) non-emitting drone straight over the radar: the blind cone ===")
    TG = 150.0
    over = line([0.0, -1500.0, 60.0], [0.0, 1500.0, 60.0], 20.0)
    ev_rg, _ = radar_events(RADAR4, rp4, [("uas", over)], TG, seed=21)
    f, outs = run(RADAR4, ev_rg, FUSION_CFG, 100.0, 100.0, TG)
    cone = [o for o in outs if o["kind"] in ("radar", "coast")
            and abs(over(o["track"]["t_state"] - EPOCH)[0][1]) < 341.0]
    s_pre = score(outs, over, kinds=("radar",), t_to=55.0)
    s_post = score(outs, over, kinds=("radar",), t_from=95.0, near_m=600.0)
    print(f"  in the cone (|N| < 341 m, el > 9 deg): {sum(o['kind'] == 'coast' for o in cone)} "
          f"coasting reports, {sum(o['kind'] == 'radar' for o in cone)} edge-of-fan plots; "
          f"misses outside view {f.stats['radar_miss_fov']}, counted {f.stats['radar_miss']}")
    print(f"  before {fmt(s_pre)}")
    print(f"  after  {fmt(s_post)}")
    check("blind cone: no deletion, same track before and after",
          f.stats["deleted_misses"] == 0 and len(s_pre["tids"]) == 1
          and s_pre["tids"] == s_post["tids"],
          f"deleted {f.stats['deleted_misses']}, before {sorted(s_pre['tids'])}, "
          f"after {sorted(s_post['tids'])}")

    two_emitters(tdoa10, ev_two)
    calibration_monitor(tdoa10, ev_t10)
    dual_range()
    regressions()

    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} checks passed")
    if n_fail:
        print("FAIL: " + "; ".join(n for n, ok, _ in RESULTS if not ok))
        return 1
    print("PASS: radar + TDOA fusion behaves.")
    return 0


def identities(outs, trajs, t_from=WARMUP_S, near_m=300.0):
    """Which tid served which truth target: every confirmed update is given
    to the nearest truth at its t_state. A target served by two tids lost
    its identity; a tid serving two targets oscillated between them."""
    by_label, by_tid = {}, {}
    for o in outs:
        if o["kind"] not in ("tdoa", "radar") or o["track"]["state"] == "tentative":
            continue
        tr = o["track"]
        ts = tr["t_state"] - EPOCH
        if ts < t_from:
            continue
        d = {lbl: float(np.linalg.norm(tr["x"][:2] - fn(ts)[0][:2]))
             for lbl, fn in trajs.items()}
        lbl = min(d, key=d.get)
        if d[lbl] > near_m:
            lbl = "none"
        by_label.setdefault(lbl, set()).add(tr["tid"])
        by_tid.setdefault(tr["tid"], set()).add(lbl)
    return by_label, by_tid


def two_emitters(tdoa10, ev_two):
    """Review A1: a coasting track's inflated covariance merged it into any
    neighbour within seconds, and the survivor then oscillated."""
    print("\n=== (h) two emitters 150 m apart, B silent 60-80 s ===")
    T = 180.0
    trajs = {"A": ORBIT10, "B": ORBIT10B}
    cases = [("TDOA-only", [], None)]
    for name, pd_b in (("radar sees both", 0.9), ("radar blind to B", 0.0)):
        evr, _ = radar_events(RADAR10, RADAR_P, [("A", ORBIT10), ("B", ORBIT10B, pd_b)],
                              T, seed=41)
        cases.append((name, evr, pd_b))
    for name, evr, pd_b in cases:
        f, outs = run(RADAR10, ev_two + evr, FUSION_CFG, 100.0, 60.0, T)
        by_label, by_tid = identities(outs, trajs)
        resumed = identities(outs, trajs, t_from=80.0)[0]
        print(f"  {name:17s} targets -> tids {dict(sorted((k, sorted(v)) for k, v in by_label.items()))}; "
              f"merged {f.stats['merged']}, rejected {f.stats['tdoa_rejected']}, "
              f"late {f.stats['late_updates']}")
        if name == "TDOA-only":
            # Reported, not checked: this is the pre-fusion nearest-neighbour
            # rule (f0 proves it bit-identical), and its known multi-target
            # limit -- a fix of the resumed B is nearer A's live track than
            # B's 20 s-old position -- is unchanged by design.
            continue
        one_each = (by_label.get("A") and by_label.get("B")
                    and len(by_label["A"]) == 1 and len(by_label["B"]) == 1
                    and by_label["A"] != by_label["B"])
        check(f"two emitters ({name}): one track each, B keeps its id after silence",
              one_each and f.stats["merged"] == 0
              and resumed.get("B") == by_label.get("B"),
              f"A {sorted(by_label.get('A', []))}, B {sorted(by_label.get('B', []))}, "
              f"merged {f.stats['merged']}")


def calibration_monitor(tdoa10, ev_t10):
    """Design gap from review: a wrong configured az_offset or a radar clock
    offset was invisible. The monitor regresses plot innovations on tracks
    TDOA has pinned and reports both; it must flag a wrong one and stay
    quiet on a right one."""
    print("\n=== (i) calibration monitor: wrong az offset, radar clock offset ===")
    T = 180.0
    for name, true_off, lat, expect_bias, expect_dt in (
            ("calibrated", None, 0.0, 0.0, 0.0),
            ("az offset 2 deg wrong", RADAR10["az_offset_deg"] + 2.0, 0.0, -2.0, 0.0),
            ("radar stamps 200 ms late", None, 0.2, 0.0, -200.0)):
        evr, _ = radar_events(RADAR10, dict(RADAR_P, latency_s=lat), [("uas", ORBIT10)],
                              T, seed=51, true_offset=true_off)
        f, outs = run(RADAR10, ev_t10 + evr, FUSION_CFG, 100.0, 60.0, T)
        cal = f.build_summary(EPOCH + T)["radars"][0]["cal"]
        s = score(outs, ORBIT10, kinds=("tdoa", "radar"))
        print(f"  {name:25s} n {cal['n']:3d}  az bias {cal.get('az_bias_deg'):+6.2f} "
              f"+/- {cal.get('az_bias_se_deg'):.2f} deg  clock {cal.get('time_offset_ms'):+7.1f} "
              f"+/- {cal.get('time_offset_se_ms'):5.1f} ms  suspect {cal.get('suspect')}  "
              f"| track h p50 {s['h50']:.2f} m v p50 {s['v50']:.2f} m")
        # 0.5 deg: the same tolerance the suspect flag uses
        ok_b = abs(cal["az_bias_deg"] - expect_bias) <= max(0.5, 3 * cal["az_bias_se_deg"])
        ok_t = abs(cal["time_offset_ms"] - expect_dt) <= max(30.0, 3 * cal["time_offset_se_ms"])
        flag = cal["suspect"] == (expect_bias != 0.0 or expect_dt != 0.0)
        check(f"calibration monitor: {name}", ok_b and ok_t and flag,
              f"bias {cal['az_bias_deg']:+.2f} (expect {expect_bias:+.1f}), clock "
              f"{cal['time_offset_ms']:+.0f} ms (expect {expect_dt:+.0f}), suspect {cal['suspect']}")


class _Fix:
    def __init__(self, enu, s=0.2, sv=None):
        self.enu = np.asarray(enu, float)
        self.cov = np.diag([s * s, s * s, (sv or s) ** 2])
        self.sigma_h = s
        self.sigma_v = sv or s


def dual_range():
    """Range A and range B of one radar: one antenna, two range scales, two
    radarnode processes. One echo seen on both is one measurement, so fused
    from both the track would claim twice the confidence it has. The shorter
    range owns the inside of its scale; the longer one contributes only
    beyond it, and owns everything if the shorter is not live."""
    print("\n=== (d) dual range: A to 1500 m, B to 6000 m, one antenna ===")
    T = 170.0
    entry = dict(RADAR10, scanners=["A", "B"])
    o = np.array(entry["enu"], dtype=float)
    # radially outward, 600 m -> 4000 m at 20 m/s, 80 m up: inside A, then B only
    traj = line(o + [600.0, 0.0, 80.0], o + [4000.0, 0.0, 80.0], 20.0)
    quiet = dict(false_per_rev=0.0, birds=0)
    pa = dict(RADAR_P, range_scale_m=1500.0, max_det_range_m=1500.0, **quiet)
    pb = dict(RADAR_P, range_scale_m=6000.0, max_det_range_m=6000.0, **quiet)
    ev_a, _ = radar_events(entry, pa, [("uas", traj)], T, seed=2)
    ev_b, _ = radar_events(entry, pb, [("uas", traj)], T, seed=3)
    for e in ev_b:
        e[3]["scanner"] = "B"                # what radarnode --scanner 1 publishes

    def b_updates(outs, inside):
        return [x for x in outs if x["kind"] == "radar" and x.get("radar") == "r01:B"
                and (x["plot"]["rng_m"] <= 1500.0) == inside]

    f, outs = run(entry, ev_a + ev_b, FUSION_CFG, 100.0, 60.0, T)
    a_upd = [x for x in outs if x["kind"] == "radar" and x.get("radar") == "r01"]
    check("dual range: no echo inside range A's scale fused from range B too",
          not b_updates(outs, True) and f.stats["radar_inner_dropped"] > 0,
          f"B updates inside 1500 m {len(b_updates(outs, True))}, B plots dropped there "
          f"{f.stats['radar_inner_dropped']}, A updates {len(a_upd)}")
    check("dual range: range B carries the track beyond range A",
          len(b_updates(outs, False)) > 20,
          f"B updates beyond 1500 m {len(b_updates(outs, False))}")
    f2, outs2 = run(entry, ev_b, FUSION_CFG, 100.0, 60.0, T)
    check("dual range: with range A silent, range B owns the inside too",
          len(b_updates(outs2, True)) > 5 and f2.stats["radar_inner_dropped"] == 0,
          f"B updates inside 1500 m {len(b_updates(outs2, True))}")
    plain = {k: v for k, v in entry.items() if k != "scanners"}
    f3, outs3 = run(plain, ev_a + ev_b, FUSION_CFG, 100.0, 60.0, T)
    rb = [r for r in f3.build_summary(EPOCH + T)["radars"] if r["id"] == "r01:B"]
    check("dual range: range B not in the site's scanners is never fused",
          not [x for x in outs3 if x.get("radar") == "r01:B"]
          and rb and not rb[0]["configured"],
          f"B outputs {len([x for x in outs3 if x.get('radar') == 'r01:B'])}, "
          f"status {rb[0]['state'] if rb else 'absent'}")


def regressions():
    """Unit-sized reproductions of the review findings (A2-A7), each the
    failing case before the fix."""
    import math as m
    import threading
    import time as tm
    from dronelocate.track import Tracker as Tk
    print("\n=== (r) review regressions ===")
    T0 = EPOCH
    RADAR = {"id": "r01", "enu": [0.0, 0.0, 8.0], "az_offset_deg": 0.0}

    def plot(t, enu, pid):
        d = np.asarray(enu) - np.array(RADAR["enu"])
        return {"id": pid, "t_utc_ns": int(round(t * 1e9)), "t_sigma_ns": 50_000_000,
                "rng_m": float(np.linalg.norm(d)),
                "az_deg": m.degrees(m.atan2(d[0], d[1])) % 360.0,
                "sig_rng_m": 2.9, "sig_az_deg": 1.0, "dop": None}

    def batch(t_end, plots, sector=4):
        return {"node": "r01", "sector": sector, "rev": 3, "n_sectors": 16,
                "t_utc_ns": int(round(t_end * 1e9)), "range_scale_m": 3000.0,
                "plots": plots, "az_ref": "antenna", "az_offset_deg": 0.0,
                "t_rev_s": 2.5, "rpm_measured": 24.0}

    def live_radar(alt_prior=100.0):
        f = Fusion(cfg={}, radars=[RADAR], alt_prior_m=alt_prior, alt_prior_sigma_m=60.0)
        for i in range(6):
            tb = 0.2 + i * 0.156
            f.on_radar_health({"node": "r01", "kind": "radar", "status": "transmit"},
                              now=T0 + tb)
            f.submit_batch(batch(T0 + tb, [], sector=i), now=T0 + tb + 0.3)
        return f

    def feed(f, b, now):
        """A batch straight to the owner (status + processing), for tests
        that call process() themselves instead of going through the heap."""
        f.radar_status("r01").on_batch(b, now)
        return f.process(b["t_utc_ns"] * 1e-9, "batch", ("r01", b), now)

    # A2: readers never iterate live state
    f = Fusion(cfg={}, radars=[], alt_prior_m=100.0)
    stop, errs, calls = [False], [], [0]

    def owner():
        i = 0
        while not stop[0]:
            i += 1
            k = f._new_track(lambda tid: Tk(tid, _Fix([i, 0, 100]), 0.0), "tentative",
                             ["radar"], 0.0, "prior")
            f.stats[f"k{i % 50}"] += 1
            if i % 3 == 0:
                f._kill(k, "x")
            if len(f.tracks) > 40:
                f._kill(next(iter(f.tracks.values())), "y")
            if i % 20 == 0:
                f.build_summary(tm.time())

    def reader():
        while not stop[0]:
            try:
                f.summary()
                f.status and None
                calls[0] += 1
            except Exception as e:  # noqa: BLE001
                errs.append(repr(e))

    ths = [threading.Thread(target=owner, daemon=True),
           threading.Thread(target=reader, daemon=True)]
    [t_.start() for t_ in ths]
    tm.sleep(1.0)
    stop[0] = True
    tm.sleep(0.1)
    check("A2 summary() safe against the owner thread", not errs,
          f"{calls[0]} reads, {len(errs)} exceptions")

    # A3: no alt_prior_m in the site; a bad item never costs a fix
    f = live_radar(alt_prior=None)
    f.process_due(T0 + 3.0)
    f.submit_batch(batch(T0 + 2.0, [plot(T0 + 1.95, [1000.0, 0.0, 100.0], 7)]), now=T0 + 2.3)
    f.submit_fix(T0 + 2.1, _Fix([1000.0, 0.0, 100.0]), now=T0 + 2.8)
    f.submit_fix(T0 + 2.2, "not a fix", now=T0 + 2.85)          # raises inside
    f.submit_fix(T0 + 2.3, _Fix([1000.0, 0.0, 100.0]), now=T0 + 2.9)
    f.process_due(T0 + 3.4)
    n_fix_tracks = sum(1 for k in f.tracks.values() if k.n_tdoa)
    check("A3 alt_prior_m absent + one bad item: every good item applied",
          f.stats["item_errors"] == 1 and n_fix_tracks == 1
          and f.stats["radar_new_tentative"] + f.stats["confirmed_by_tdoa"] >= 1
          and f.pending() == 0,
          f"item_errors {f.stats['item_errors']} ({f.last_error}), fix tracks "
          f"{n_fix_tracks}, pending {f.pending()}")

    # A5: a fix measured between two plots of one batch is applied on time
    f = live_radar()
    v = np.array([15.0, 0.0, 0.0])
    p0 = np.array([1000.0, 0.0, 100.0])
    pos = lambda t: p0 + v * t
    for i in range(8):
        f.submit_fix(T0 + i * 0.5, _Fix(pos(i * 0.5)), now=T0 + i * 0.5 + 0.7)
    f.process_due(T0 + 5.0)
    trk = f.tracks[1]
    f.submit_batch(batch(T0 + 4.36, [plot(T0 + 4.20, pos(4.20), 1),
                                     plot(T0 + 4.30, [900.0, 400.0, 60.0], 2)]),
                   now=T0 + 4.66)
    f.submit_fix(T0 + 4.25, _Fix(pos(4.25)), now=T0 + 4.95)
    f.process_due(T0 + 5.4)
    err = float(np.linalg.norm(trk.kf.pos - pos(trk.kf.t - T0)))
    check("A5 fix between two plots of a batch not applied late",
          f.stats["late_updates"] <= 1 and trk.kf.late <= 1 and err < 0.5,
          f"late_updates {f.stats['late_updates']} (the earlier plot, retrodicted), "
          f"pos error {err:.2f} m")

    # A4: mirror fixes with a bird nearer by Euclid than the drone's track
    f = live_radar()
    f.process_due(T0 + 1.5)
    R = np.diag([2.9 ** 2, m.radians(1.3) ** 2])

    def mk(x, sig, src, n_tdoa):
        P = np.diag(np.array(sig, float) ** 2)
        k = f._new_track(lambda tid: Tk.from_state(tid, np.array(x, float), P, T0 + 1.5),
                         "confirmed", src, T0 + 1.5, "tdoa")
        k.n_tdoa, k.n_radar = n_tdoa, 20
        k.last_tdoa_t = T0 + 1.5 if n_tdoa else None
        k.last_radar_t = T0 + 1.5
        k.last_plot = ("r01", R)
        return k
    mk([1000, 0, 160, 15, 0, 0], [0.2, 0.2, 0.5, 0.5, 0.5, 0.3], ["tdoa", "radar"], 50)
    mk([1000, 100, 100, 5, 5, 0], [7, 7, 17, 3, 3, 1], ["radar"], 0)
    n0 = len(f.tracks)
    for k_, t in enumerate([2.0, 2.5, 3.0]):
        f.submit_batch(batch(T0 + t, [], sector=k_), now=T0 + t)
        f.process(T0 + t, "fix", (_Fix([1000 + 15 * (t - 1.5), 0.0, -160.0], s=1.0, sv=5.0), {}),
                  now=T0 + t)
    check("A4 below-ground mirror fixes found no ghost (bird nearer than drone)",
          len(f.tracks) == n0 and f.stats["tdoa_rejected"] == 3,
          f"tracks {n0} -> {len(f.tracks)}, rejected {f.stats['tdoa_rejected']} "
          f"(same-object {f.stats['tdoa_rejected_same_object']})")

    # A1 (inheritance): a coasting radar-only bird never merges into a
    # TDOA-only drone track and never lends it radar support
    f = live_radar()
    f.process_due(T0 + 1.5)
    f.process(T0 + 1.5, "fix", (_Fix([1000, 0, 100]), {}), now=T0 + 1.5)
    bird = mk([1000, 300, 100, 0, 0, 0], [7, 7, 17, 3, 3, 1], ["radar"], 0)
    bird.kf.max_step_s = 1.0
    t = 1.5
    while t < 40.0:
        t += 0.5
        if int(t * 2) % 3 == 0:
            f.submit_batch(batch(T0 + t, [], sector=int(t * 2) % 16), now=T0 + t)
        f.process(T0 + t, "fix", (_Fix([1000, 0, 100]), {}), now=T0 + t)
    drone = f.tracks[1]
    check("A1 a coasting bird is not merged into the drone, lends it no radar support",
          not f.stats["merged"] and not drone.radar_supported(T0 + t, 10.0),
          f"merged {f.stats['merged']}, drone src {sorted(drone.src)}")

    # A7 + follow-up 1: a hidden tentative needs tdoa_confirm_n (3)
    # consistent fixes; the ones it absorbs are not counted as rejected
    f = live_radar()
    f.process_due(T0 + 1.5)
    mk([1000, 0, 100, 0, 0, 0], [0.5, 0.5, 2, 0.3, 0.3, 0.3], ["tdoa", "radar"], 50)
    outs = []
    for t in (2.0, 2.5, 3.0):
        outs += f.process(T0 + t, "fix", (_Fix([1500, 0, 100]), {}), now=T0 + t)
    pub = [o["track"]["tid"] for o in outs if o["kind"] == "tdoa"]
    check("A7 second emitter: 1st fix rejected, published only on the 3rd",
          f.stats["tdoa_rejected"] == 1 and f.stats["tdoa_assoc_tentative"] == 2
          and f.stats["confirmed_by_tdoa"] == 1 and len(pub) == 1,
          f"rejected {f.stats['tdoa_rejected']}, tentative "
          f"{f.stats['tdoa_assoc_tentative']}, confirmed {f.stats['confirmed_by_tdoa']}, "
          f"published {len(pub)}")

    # Follow-up 1 (f5_zombie): a lost peak holding for two bursts, then a
    # lone outlier 14 s later. Fused mode must publish neither, and must
    # not keep a track for them.
    def drone_run(radar_on, extra_off, T=60.0, pause=None):
        f = live_radar() if radar_on else Fusion(cfg={}, radars=[RADAR],
                                                 alt_prior_m=100.0, alt_prior_sigma_m=60.0)
        rng = np.random.default_rng(7)
        S = lambda t: np.array([1000.0 + 15 * t, 0.0, 100.0])
        outs, t = [], 1.5
        while t < T:
            if radar_on:
                # radar is up, but never sees this drone: TDOA decides alone
                feed(f, batch(T0 + t, [], sector=int(t * 2) % 16), T0 + t)
            if pause is None or not (pause[0] <= t < pause[1]):
                p = S(t) + rng.normal(0, 0.2, 3) + extra_off(t)
                outs += f.process(T0 + t, "fix", (_Fix(p), {}), now=T0 + t)
            t += 0.5
        return f, outs, S
    off = lambda t: (np.array([0.0, 300.0, 0.0]) if 20.0 <= t < 21.0 else
                     np.array([0.0, 250.0, 0.0]) if 35.0 <= t < 35.5 else np.zeros(3))
    f, outs, S = drone_run(True, off)
    bad = [o for o in outs if o["kind"] == "tdoa"
           and np.linalg.norm(o["track"]["x"][:3] - S(o["t"] - T0)) > 50.0]
    check("follow-up 1: a 2-burst lost peak and a lone outlier publish nothing, leave no track",
          not bad and len(f.tracks) == 1,
          f"{len(bad)} wrong updates published, tracks {sorted(f.tracks)}")
    # ...while a drone that pauses 20 s and resumes keeps its id (resumed
    # through the hidden tentative), and one silent past tdoa_coast_s goes
    f, outs, S = drone_run(True, lambda t: np.zeros(3), pause=(20.0, 40.0))
    tids_ = sorted({o["track"]["tid"] for o in outs if o["kind"] == "tdoa"})
    f2, outs2, _ = drone_run(True, lambda t: np.zeros(3), T=90.0, pause=(20.0, 85.0))
    check("follow-up 1: 20 s pause keeps the id; 65 s pause is deleted after tdoa_coast_s",
          tids_ == [1] and f.stats["tdoa_resumed"] == 1 and f2.stats["deleted_tdoa_coast"] == 1,
          f"tids {tids_}, resumed {f.stats['tdoa_resumed']}, deleted "
          f"{f2.stats['deleted_tdoa_coast']}")
    # TDOA-only mode: the pre-fusion rule, untouched (f0 checks it bit for
    # bit); here just that nothing is ever deleted there
    f3, _, _ = drone_run(False, lambda t: np.zeros(3), T=90.0, pause=(20.0, 85.0))
    check("follow-up 1: TDOA-only mode still never deletes",
          not f3.stats["deleted_tdoa_coast"] and len(f3.tracks) == 1,
          f"deleted {f3.stats['deleted_tdoa_coast']}, tracks {sorted(f3.tracks)}")

    # Follow-up 4 (f3_more): a second emitter 50 m cross-range from a
    # radar-supported drone at 1 km -- inside the beam -- appearing later
    f = live_radar()
    f.process_due(T0 + 1.5)
    A2 = np.array([1000.0, 0.0, 100.0]); B2 = np.array([1000.0, 50.0, 100.0])
    rng = np.random.default_rng(11)
    outs, t = [], 1.5
    while t < 40.0:
        outs += feed(f, batch(T0 + t, [plot(T0 + t - 0.01, A2, int(t * 10))]
                              if int(t * 2) % 5 == 0 else [], sector=int(t * 2) % 16), T0 + t)
        outs += f.process(T0 + t, "fix", (_Fix(A2 + rng.normal(0, 0.2, 3)), {}), now=T0 + t)
        if t >= 15.0:
            outs += f.process(T0 + t + 0.1, "fix", (_Fix(B2 + rng.normal(0, 0.2, 3)), {}),
                              now=T0 + t + 0.1)
        t += 0.5
    onB = {o["track"]["tid"] for o in outs if o["kind"] == "tdoa" and o["t"] - T0 > 20
           and abs(o["track"]["x"][1] - 50.0) < 5.0}
    onA = {o["track"]["tid"] for o in outs if o["kind"] == "tdoa" and o["t"] - T0 > 20
           and abs(o["track"]["x"][1]) < 5.0}
    check("follow-up 4: second emitter inside the beam gets its own track",
          len(onB) == 1 and len(onA) == 1 and onA != onB and not f.stats["merged"],
          f"A tids {sorted(onA)}, B tids {sorted(onB)}, merged {f.stats['merged']}, "
          f"same-object {f.stats['tdoa_rejected_same_object']}")

    # Follow-up 2: bus-thread counters never land in self.stats
    f = live_radar()
    n0 = len(f.stats)
    for i in range(50):
        f.submit_batch(dict(batch(T0 + 1.0, []), az_ref="true"), now=T0 + 1.0)
        f.submit_batch(dict(batch(T0 + 1.0, []), node="r99"), now=T0 + 1.0)
    summ = f.build_summary(T0 + 1.0)
    st = {r["id"]: r for r in summ["radars"]}
    check("follow-up 2: bus-thread counters live on RadarStatus, not in stats",
          len(f.stats) == n0 and st["r01"]["batches_bad_frame"] == 50
          and st["r99"]["batches_unfused"] == 50,
          f"stats keys {n0} -> {len(f.stats)}, bad frame {st['r01']['batches_bad_frame']}, "
          f"unconfigured r99 unfused {st['r99']['batches_unfused']}")

    # Follow-up 5: a fused radar with plots but no calibration samples says why
    f = live_radar()
    f.process_due(T0 + 1.5)
    f.submit_batch(batch(T0 + 2.0, [plot(T0 + 1.9, [900.0, 400.0, 60.0], 1)]), now=T0 + 2.0)
    f.process_due(T0 + 3.5)
    cal = f.build_summary(T0 + 3.5)["radars"][0]["cal"]
    check("follow-up 5: calibration monitor explains n == 0", cal["n"] == 0 and "hint" in cal,
          f"cal {cal}")



if __name__ == "__main__":
    sys.exit(main())
