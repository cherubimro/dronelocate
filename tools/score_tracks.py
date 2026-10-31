#!/usr/bin/env python3
"""Score published tracks against sim truth, live, per update.

Subscribes to dc/{site}/sim/truth and dc/{site}/track/*, records for
--duration seconds, then compares every track update with the truth at the
track's own state time (t_state): for a TDOA update that is its burst's
truth exactly (bug 14: never score against the latest truth), for a radar
update or a coasting report the truth interpolated at t_state. Scoring
only -- the same reason the supernode may hold a sim/truth subscriber.

    tools/score_tracks.py --duration 70 [--split 35] [--out run.jsonl]

--split T reports the first T seconds and the rest separately (e.g. a
radarnode killed T seconds in). Warm-up (--skip, default 10 s) is dropped.
"""

import argparse
import bisect
import json
import math
import sys
import threading
import time

import numpy as np
import zenoh

sys.path.insert(0, ".")
from dronelocate import proto, zconf  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--skip", type=float, default=10.0)
    ap.add_argument("--split", type=float, action="append", default=[])
    ap.add_argument("--out")
    zconf.add_args(ap)
    a = ap.parse_args()
    cfg = proto.SiteConfig.load(a.config)
    truth, tracks = {}, []
    lock = threading.Lock()

    def on_truth(s):
        t = proto.decode(s.payload)
        with lock:
            truth[t["burst"]] = t

    def on_track(s):
        m = proto.decode(s.payload)
        with lock:
            tracks.append(m)

    sess = zenoh.open(zconf.spoke(a.hub, a.port, zconf.tls_from_args(a)))
    sess.declare_subscriber(proto.ke_truth(cfg.site), on_truth)
    sess.declare_subscriber(proto.ke_track(cfg.site), on_track)
    t0 = time.time()
    time.sleep(a.duration)
    sess.close()

    with lock:
        tl = sorted(truth.values(), key=lambda t: t["t_emit_s"])
        msgs = list(tracks)
    ts = [t["t_emit_s"] for t in tl]

    def truth_at(t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(ts):
            return None
        ta, tb = tl[i - 1], tl[i]
        w = (t - ta["t_emit_s"]) / max(tb["t_emit_s"] - ta["t_emit_s"], 1e-9)
        p = np.array(ta["enu"]) + w * (np.array(tb["enu"]) - np.array(ta["enu"]))
        v = np.array(ta["vel"]) + w * (np.array(tb["vel"]) - np.array(ta["vel"]))
        return p, v

    rows = []
    for m in msgs:
        upd = m.get("upd", "tdoa")
        tst = m.get("t_state", m["t_meas"])
        tr = truth.get(m["burst"]) if upd == "tdoa" and m.get("burst") is not None else None
        if tr is not None and abs(tr["t_emit_s"] - tst) < 1e-3:
            p, v = np.array(tr["enu"]), np.array(tr["vel"])
        else:
            pv = truth_at(tst)
            if pv is None:
                continue
            p, v = pv
        d = np.array(m["enu"]) - p
        row = {"t_rel": m["t"] - t0, "upd": upd, "tid": m["tid"],
               "state": m.get("state"), "src": m.get("src"),
               "h": float(math.hypot(d[0], d[1])), "v": float(abs(d[2])),
               "vel": float(np.linalg.norm(np.array(m["vel"]) - v)),
               "fix_h": None, "fix_v": None, "late": m.get("late_updates", 0)}
        if upd == "tdoa" and m.get("raw_enu") and tr is not None:
            df = np.array(m["raw_enu"]) - np.array(tr["enu"])
            row["fix_h"], row["fix_v"] = float(math.hypot(df[0], df[1])), float(abs(df[2]))
        rows.append(row)
    if a.out:
        with open(a.out, "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")

    edges = [a.skip] + sorted(a.split) + [a.duration + 5.0]
    q = lambda xs, k: float(np.percentile(xs, k)) if xs else float("nan")
    for lo, hi in zip(edges[:-1], edges[1:]):
        seg = [r for r in rows if lo <= r["t_rel"] < hi]
        print(f"--- {lo:.0f}-{hi:.0f} s: {len(seg)} track updates, tids "
              f"{sorted({r['tid'] for r in seg})}")
        for kind in ("all", "tdoa", "radar", "coast"):
            sk = seg if kind == "all" else [r for r in seg if r["upd"] == kind]
            if not sk:
                continue
            h, v, vel = [r["h"] for r in sk], [r["v"] for r in sk], [r["vel"] for r in sk]
            print(f"  track {kind:5s} n {len(sk):4d}  h p50 {q(h, 50):6.2f} p90 {q(h, 90):7.2f} m"
                  f"  v p50 {q(v, 50):6.2f} p90 {q(v, 90):7.2f} m"
                  f"  vel p50 {q(vel, 50):5.2f} p90 {q(vel, 90):6.2f} m/s")
        fh = [r["fix_h"] for r in seg if r["fix_h"] is not None]
        fv = [r["fix_v"] for r in seg if r["fix_v"] is not None]
        if fh:
            print(f"  raw fix     n {len(fh):4d}  h p50 {q(fh, 50):6.2f} p90 {q(fh, 90):7.2f} m"
                  f"  v p50 {q(fv, 50):6.2f} p90 {q(fv, 90):7.2f} m")
        print(f"  states {sorted({(r['state'], tuple(r['src'] or [])) for r in seg}, key=str)}")


if __name__ == "__main__":
    main()
