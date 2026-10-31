"""Bench experiments E1-E5 on the HALO 20+ (both ranges), with restore.

Run with a Python that has numpy only if needed; navico is imported from the repo root.
Everything is recorded raw to experiments.navrec (A and B tagged) and the
timeline printed. A finally-block restores modes, ranges, scan speed,
Doppler speed and puts both ranges in standby, whatever happens.
"""
import collections
import statistics
import struct
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import navico  # noqa: E402

T0 = time.time()


def log(msg):
    print(f"{time.time() - T0:7.1f}s  {msg}", flush=True)


iface = navico.local_ip_for("192.168.100.231")
beacon = navico.discover("192.168.100.231", iface)
assert beacon, "no beacon"
A = navico.Radar(beacon, iface, 0)
B = navico.Radar(beacon, iface, 1)
rec = navico.Recorder(os.path.join(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."), "recordings", "experiments.navrec.gz"))

# spoke-clock offset and wrap-measured rpm per scanner, from the data stream
offs = collections.defaultdict(list)       # scanner -> [(t, offset_ms)]
wraps = collections.defaultdict(list)      # scanner -> [t of angle wrap]
last_angle = {}
c415 = []


def data_listener(sc):
    def on(kind, t, d):
        if kind == navico.CHANNEL_REPORT:
            if d[:2] == b"\x15\xc4":
                c415.append((t, d.hex(" ")))
            return
        n = (len(d) - navico.FRAME_HEADER_LEN) // navico.LINE_LEN
        if n < 1:
            return
        o = navico.FRAME_HEADER_LEN + (n - 1) * navico.LINE_LEN
        if d[o + 1] != 0x02:
            return
        ms = struct.unpack_from("<H", d, o + 14)[0]
        offs[sc].append((t, (t * 1000.0 - ms) % 60000.0))
        a = (struct.unpack_from("<H", d, o + 8)[0] // 2) % navico.SPOKES
        if sc in last_angle and a < last_angle[sc] - navico.SPOKES // 2:
            wraps[sc].append(t)
        last_angle[sc] = a
    return on


for r in (A, B):
    r.listeners.append(rec.for_scanner(r.scanner))
    r.listeners.append(data_listener(r.scanner))
    r.start()


def fresh(r, section, timeout=8.0):
    """Drop a cached report section and wait for the radar to resend it."""
    r.state.pop(section, None)
    end = time.time() + timeout
    while time.time() < end:
        if section in r.state:
            return r.state[section]
        time.sleep(0.2)
    return None


def setc(r, control, value):
    r.send(navico.CONTROLS[control][1](str(value)))
    log(f"{'AB'[r.scanner]}: set {control} {value}")


def power(r, state):
    r.send(navico.power_frames(state))
    end = time.time() + 15
    want = "transmit" if state == "transmit" else "standby"
    while time.time() < end:
        st = fresh(r, "mode", 3)
        if st and st["status"] == want:
            break
    log(f"{'AB'[r.scanner]}: {state} -> status {r.state.get('mode', {}).get('status')}")


def rpm_now(sc, window=6.0):
    w = [t for t in wraps[sc] if t > time.time() - window]
    return None if len(w) < 2 else round(60 * (len(w) - 1) / (w[-1] - w[0]), 1)


def offset_stats(sc, t_from, t_to):
    v = [o for t, o in offs[sc] if t_from <= t < t_to]
    if not v:
        return "no data"
    return f"median {statistics.median(v):.1f} ms, sd {statistics.pstdev(v):.2f} ms, n {len(v)}"


def snap(r, tag):
    p = fresh(r, "properties") or {}
    pr = fresh(r, "processing") or {}
    log(f"{'AB'[r.scanner]} [{tag}] transmit_sessions {p.get('transmit_sessions')} "
        f"tx_hours {p.get('transmit_hours')} C407 tx_snapshot {pr.get('tx_snapshot')} "
        f"range_params {pr.get('range_params')}")
    return p.get("transmit_sessions"), pr.get("tx_snapshot")


def rpm_report(tag, scanners=(0, 1)):
    parts = []
    for r in (A, B):
        if r.scanner in scanners:
            se = fresh(r, "setup_extended") or {}
            parts.append(f"{'AB'[r.scanner]}: C408 {se.get('antenna_rpm')} rpm, "
                         f"scan_speed {se.get('scan_speed')}, wraps {rpm_now(r.scanner)} rpm")
    log(f"[{tag}] " + " | ".join(parts))


try:
    time.sleep(4)
    # ------------------------------------------------------------- E1+E3+E2
    log("=== E1/E2/E3/E5 on range A")
    s0, c0 = snap(A, "before")
    power(A, "transmit")
    t1 = time.time()
    time.sleep(20)
    log(f"E1 no NKOE:           offset {offset_stats(0, t1 + 3, time.time())}")
    t2 = time.time()
    A.set_nav(heading=0.0)
    time.sleep(20)
    log(f"E1 NKOE epoch +0 s:   offset {offset_stats(0, t2 + 3, time.time())}")
    A.nav.epoch_offset_s = 30.0
    t3 = time.time()
    time.sleep(20)
    log(f"E1 NKOE epoch +30 s:  offset {offset_stats(0, t3 + 3, time.time())}")
    A.nav.epoch_offset_s = 0.0
    A.set_nav()          # stop sending heading
    power(A, "standby")
    t_stop1 = time.time()
    s1, c1 = snap(A, "after 60 s transmit")
    while time.time() < t_stop1 + 120:
        time.sleep(0.5)
    power(A, "transmit")
    t4 = time.time()
    log(f"E1 after 120 s pause: offset {offset_stats(0, t4 + 3, t4 + 13) if time.sleep(13) is None else ''}")
    # E5: 0D C2 query, then Doppler speed change
    n415 = len(c415)
    A.send([bytes([0x0D, 0xC2])])
    log("E5: sent 0d c2")
    time.sleep(3)
    setc(A, "doppler-speed", 3)
    time.sleep(4)
    setc(A, "doppler-speed", 2)
    time.sleep(4)
    log(f"E5: C415 reports received {len(c415) - n415}")
    for t, h in c415[n415:]:
        log(f"    C415 at {t - T0:.1f}s: {h}")
    while time.time() < t4 + 60:
        time.sleep(0.5)
    power(A, "standby")
    s2, c2 = snap(A, "after second 60 s transmit")
    if c0 and c1 and c2:
        d1 = [b - a for a, b in zip(c0, c1)]
        d2 = [b - a for a, b in zip(c1, c2)]
        log(f"E2: C407 deltas: first cycle {d1}, second cycle {d2} "
            f"(first transmit 60 s; second: 120 s pause + 60 s transmit)")
    log(f"E3: transmit_sessions {s0} -> {s1} -> {s2}")

    # ------------------------------------------------------------- E4 b/c
    log("=== E4 b/c: range A in custom mode, scan speed levels and ranges")
    setc(A, "mode", "custom")
    time.sleep(2)
    power(A, "transmit")
    for lvl in (0, 1, 2, 3):
        setc(A, "scan-speed", lvl)
        time.sleep(9)
        rpm_report(f"custom scan-speed {lvl}, 75 m", (0,))
    for rng in (250, 1000, 3000):
        setc(A, "range", rng)
        time.sleep(9)
        rpm_report(f"custom scan-speed 3, range {rng} m", (0,))
    setc(A, "range", 75)
    setc(A, "scan-speed", 1)       # custom mode's own value before the test
    time.sleep(3)
    power(A, "standby")
    setc(A, "mode", "bird")
    time.sleep(3)

    # ------------------------------------------------------------- E4 a
    log("=== E4 a: who sets the antenna speed with both ranges transmitting")
    setc(A, "mode", "offshore")
    setc(B, "mode", "bird")
    time.sleep(3)
    power(A, "transmit")
    time.sleep(9)
    rpm_report("A offshore alone", (0,))
    power(B, "transmit")
    time.sleep(9)
    rpm_report("A offshore + B bird (B started last)")
    power(A, "standby")
    time.sleep(9)
    rpm_report("B bird alone", (1,))
    power(A, "transmit")
    time.sleep(9)
    rpm_report("A offshore + B bird (A started last)")
finally:
    log("=== restore")
    for r in (A, B):
        power(r, "standby")
    setc(A, "mode", "bird")
    setc(A, "range", 75)
    setc(A, "doppler-speed", 2)
    setc(B, "mode", "offshore")
    setc(B, "range", 115)
    time.sleep(4)
    for r in (A, B):
        r.stop()
    rec.close()
    log(f"recorded {rec.count} packets, {rec.bytes / 1e6:.0f} MB")
