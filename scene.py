#!/usr/bin/env python3
"""Ground truth publisher for the emulated fleet.

Publishes where the drone actually is. Emulated nodes subscribe and render
their own IQ from it -- each applying its own geometric delay, clock error,
path loss and noise. The supernode never subscribes to this key expression;
it only sees IQ. The console does subscribe, purely to score the solver.
"""

import argparse
import math
import time

import numpy as np
import zenoh

from dronelocate import proto, zconf
from dronelocate.geo import LocalFrame


def orbit(t, p):
    """Circular orbit with a slow climb. Exercises both the horizontal
    solution and the weakly observable vertical at the same time."""
    c = np.array(p.get("centre_enu", [0.0, 0.0, 90.0]), dtype=float)
    r = float(p.get("radius_m", 900.0))
    period = float(p.get("period_s", 180.0))
    climb = float(p.get("climb_m", 30.0))
    th = 2.0 * math.pi * t / period
    return np.array([
        c[0] + r * math.cos(th),
        c[1] + r * math.sin(th),
        c[2] + climb * math.sin(th * 0.5),
    ])


def transit(t, p):
    """Straight-line pass across the site at constant altitude."""
    a = np.array(p.get("from_enu", [-4000.0, -1500.0, 110.0]), dtype=float)
    b = np.array(p.get("to_enu", [4000.0, 1500.0, 110.0]), dtype=float)
    period = float(p.get("period_s", 240.0))
    f = (t % period) / period
    return a + (b - a) * f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--kind", choices=["orbit", "transit"], default=None)
    ap.add_argument("--rate", type=float, default=None,
                    help="bursts per second (overrides config)")
    zconf.add_args(ap)
    a = ap.parse_args()

    cfg = proto.SiteConfig.load(a.config)
    frame = LocalFrame.from_dict(cfg.origin)
    track = cfg.sim.get("track", {})
    kind = a.kind or track.get("kind", "orbit")
    rate = a.rate or float(cfg.sim.get("burst_hz", 2.0))
    fn = {"orbit": orbit, "transit": transit}[kind]

    session = zenoh.open(zconf.spoke(a.hub, a.port, zconf.tls_from_args(a)))
    pub = session.declare_publisher(
        proto.ke_truth(cfg.site),
        priority=zenoh.Priority.INTERACTIVE_HIGH,
        congestion_control=zenoh.CongestionControl.BLOCK,
    )

    print(f"[scene] {kind} trajectory, {rate:.1f} bursts/s")
    t0 = time.time()
    burst = 0
    try:
        while True:
            now = time.time()
            enu = fn(now - t0, track)
            lat, lon, alt = frame.to_geodetic(enu)
            pub.put(proto.encode({
                "burst": burst,
                "t_emit_s": now,
                "t_emit_ns": int(now * 1e9),
                "enu": [float(x) for x in enu],
                "lat": lat, "lon": lon, "alt_m": alt,
                "label": "sim-uas-1",
            }))
            burst += 1
            time.sleep(1.0 / rate)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == "__main__":
    main()
