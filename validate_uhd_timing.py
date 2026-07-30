#!/usr/bin/env python3
"""Verify the UHD timing convention and the node.py wiring, without hardware.

    python3 validate_uhd_timing.py

Two things are checked, and they fail in different ways.

1. THE SIGN. UHD gives every node an absolute timestamp for sample zero, so
   node buffers begin at different instants and the correlator's raw lag is
   contaminated by that difference. node.py converts it into the clk_off_ns
   field the supernode already subtracts. Get the sign backwards and nothing
   raises: you get confident positions that are wrong by hundreds of metres,
   and they degrade smoothly as the offsets shrink, so a bench test with a
   good GPSDO looks fine and the field deployment does not. This reproduces
   the arithmetic end to end and also runs it with the sign flipped, so the
   test would notice if someone "tidied" it.

2. THE WIRING. A fake UHD module stands in for the bindings so the
   UhdNodeSource path -- device open, rate adoption, slot grid, dual-channel
   bearing, event fields -- executes here. It cannot tell you whether the
   TinyB210's PPS path is any good; that is hw_selftest.py's job against real
   hardware. It does tell you the plumbing is connected, which is the part
   that has been unexecuted until now.
"""

import sys
import types

import numpy as np

from dronelocate import proto
from dronelocate.geo import C_LIGHT
from dronelocate.sigsim import NodeChannel
from dronelocate.tdoa import cross_correlate, solve_tdoa, timing_sigma

FS = 2.4e6
BW = 2.0e6
N = 24000
T_EMIT = 1_785_000_000.0
T_SLOT = T_EMIT           # the instant every node was told to open its window
EMITTER = np.array([600.0, -400.0, 95.0])
TRUE_BEARING_DEG = 20.0   # phase baked into the fake device's channel 1


def _render_fleet(nodes, t0_errors, seed=7):
    """One capture per node, each window starting at t_slot + its own error."""
    iq, clk_off_ns = {}, {}
    for k, (nid, enu) in enumerate(nodes.items()):
        t0 = T_SLOT + t0_errors[nid]
        ch = NodeChannel(np.array(enu, dtype=float), FS, BW,
                         clock_bias_s=0.0, clock_drift_ppm=0.0, rng_seed=seed + k)
        # capture_start_s is this node's window start -- exactly what UHD
        # reports as t0_actual. clock_bias stays at zero because with UHD the
        # offset is measured, not estimated.
        sig, _ = ch.render(EMITTER, burst_id=1, n_samples=N,
                           t_epoch_s=T_EMIT, capture_start_s=t0,
                           snr_db_at_1km=30.0)
        iq[nid] = sig
        # The convention under test. See UhdNodeSource in node.py.
        clk_off_ns[nid] = -(t0 - T_SLOT) * 1e9
    return iq, clk_off_ns


def _solve(nodes, iq, clk_off_ns, flip_sign=False):
    ref = list(nodes)[0]
    pts = np.array(list(nodes.values()))
    baseline = float(np.max(np.linalg.norm(pts[:, None] - pts[None, :], axis=2)))
    max_lag = baseline / C_LIGHT + 5e-4

    others, tdoa, sig = [], [], []
    for nid in nodes:
        if nid == ref:
            continue
        c = cross_correlate(iq[nid], iq[ref], FS, max_lag_s=max_lag)
        delta = (clk_off_ns[nid] - clk_off_ns[ref]) * 1e-9
        lag = c.lag_s + delta if flip_sign else c.lag_s - delta
        others.append(nid)
        tdoa.append(lag)
        sig.append(timing_sigma(c, FS, BW))

    return solve_tdoa(
        np.array([nodes[i] for i in others]), np.array(nodes[ref]),
        np.array(tdoa), sigma_s=np.array(sig),
        alt_prior_m=100.0, alt_prior_sigma_m=60.0)


def check_sign():
    print("=== 1. clock-offset sign convention ===")
    cfg = proto.SiteConfig.load("config/site.json")
    nodes = {n["id"]: np.array(n["enu"], dtype=float) for n in cfg.nodes}

    rng = np.random.default_rng(11)
    cases = [
        ("scheduled: 30 ns jitter", lambda: rng.normal(0.0, 30e-9)),
        ("scheduled: 1 us jitter", lambda: rng.normal(0.0, 1e-6)),
        ("independent: 100 us spread", lambda: rng.uniform(-100e-6, 100e-6)),
        ("common 5 ms offset (must cancel)", lambda: 5e-3),
    ]

    worst = 0.0
    for label, draw in cases:
        errs = {nid: draw() for nid in nodes}
        iq, clk = _render_fleet(nodes, errs)

        good = _solve(nodes, iq, clk)
        bad = _solve(nodes, iq, clk, flip_sign=True)
        if good is None:
            print(f"  {label:<34} SOLVER RETURNED None")
            return False, np.inf

        d_ok = float(np.linalg.norm(good.enu - EMITTER))
        d_bad = float(np.linalg.norm(bad.enu - EMITTER)) if bad is not None else np.inf
        worst = max(worst, d_ok)
        spread_ns = (max(errs.values()) - min(errs.values())) * 1e9
        print(f"  {label:<34} err {d_ok:7.1f} m   "
              f"(wrong sign: {d_bad:9.1f} m)   t0 spread {spread_ns:.0f} ns")

    return worst < 120.0, worst


# --------------------------------------------------------------------------
# fake UHD bindings: enough of the 4.5.0 surface for the node path to run
# --------------------------------------------------------------------------
def install_fake_uhd(node_enu, t0_error_s=0.0, n_channels=2,
                     fs_actual=2.4e6, fc_actual=2.437e9,
                     true_bearing_deg=20.0, baseline_m=0.061):
    """Inject a stand-in `uhd` module into sys.modules.

    Mirrors the real API shapes verified against UHD 4.5.0: recv() takes the
    whole buffer and returns a count, metadata carries the time_spec of the
    first packet, and StreamCMD has num_samps / stream_now / time_spec.
    """
    m = types.ModuleType("uhd")
    m.types = types.ModuleType("uhd.types")
    m.usrp = types.ModuleType("uhd.usrp")

    class TimeSpec:
        def __init__(self, s):
            self._s = float(s)

        def get_real_secs(self):
            return self._s

    class TuneRequest:
        def __init__(self, f):
            self.f = float(f)

    class _Enum:
        none = "none"; timeout = "timeout"; late = "late"
        overflow = "overflow"; alignment = "alignment"

    class StreamMode:
        num_done = "num_done"; stop_cont = "stop_cont"; start_cont = "start_cont"

    class StreamCMD:
        def __init__(self, mode):
            self.mode = mode
            self.num_samps = 0
            self.stream_now = True
            self.time_spec = None

    class RXMetadata:
        def __init__(self):
            self.error_code = _Enum.none
            self.has_time_spec = False
            self.time_spec = TimeSpec(0.0)

        def strerror(self):
            return "fake: no error"

    class StreamArgs:
        def __init__(self, cpu, otw):
            self.cpu_format, self.otw_format = cpu, otw
            self.channels = [0]

    class _Sensor:
        def __init__(self, v):
            self.value = str(v)
            self._v = v

        def to_bool(self):
            return bool(self._v)

        def to_int(self):
            return int(self._v)

    class _Streamer:
        MAX = 2040

        def __init__(self, dev):
            self.dev = dev
            self._pending = 0
            self._t0 = None
            self._served = 0

        def get_max_num_samps(self):
            return self.MAX

        def issue_stream_cmd(self, cmd):
            if cmd.mode == StreamMode.stop_cont:
                self._pending = 0
                return
            self._pending = int(cmd.num_samps)
            self._served = 0
            start = (cmd.time_spec.get_real_secs() if not cmd.stream_now
                     else self.dev._now())
            # the hardware lands slightly off the requested instant
            self._t0 = start + t0_error_s
            self._buf = self.dev._render(self._t0, self._pending)

        def recv(self, buf, md, timeout=0.1):
            # Contract check: the real binding hands numpy's raw pointer to
            # C++, so a strided view silently loses data. Catch it here rather
            # than on hardware at 3am.
            if not buf.flags["C_CONTIGUOUS"]:
                raise AssertionError(
                    "recv() got a non-contiguous buffer -- this is the bug "
                    "that only shows up with 2 channels")
            if self._pending <= 0:
                md.error_code = _Enum.timeout
                return 0
            take = min(self.MAX, self._pending, buf.shape[1])
            buf[:, :take] = self._buf[:, self._served:self._served + take]
            md.error_code = _Enum.none
            md.has_time_spec = (self._served == 0)
            md.time_spec = TimeSpec(self._t0)
            self._served += take
            self._pending -= take
            return take

    class MultiUSRP:
        def __init__(self, args=""):
            self.args = args
            self._time = T_SLOT - 10.0
            self.rate, self.freq = fs_actual, fc_actual

        def _now(self):
            self._time += 0.001
            return self._time

        def _render(self, t0, n):
            ch = NodeChannel(np.array(node_enu, dtype=float), fs_actual, BW,
                             rng_seed=3)
            sig, _ = ch.render(EMITTER, 1, n, T_EMIT, t0, snr_db_at_1km=30.0)
            out = np.zeros((n_channels, n), dtype=np.complex64)
            out[0] = sig
            # Give channel 1 the phase a real wavefront would impose, so the
            # bearing check measures something instead of correlating a
            # signal with itself.
            if n_channels > 1:
                lam = C_LIGHT / fc_actual
                dphi = (2.0 * np.pi * baseline_m
                        * np.sin(np.radians(true_bearing_deg)) / lam)
                for c in range(1, n_channels):
                    out[c] = sig * np.exp(1j * dphi * c)
            return out

        def get_mboard_sensor_names(self, mb=0):
            return ["gps_locked", "ref_locked", "gps_time", "gps_gpgga"]

        def get_mboard_sensor(self, name, mb=0):
            return {"gps_locked": _Sensor(True), "ref_locked": _Sensor(True),
                    "gps_time": _Sensor(int(T_SLOT)),
                    "gps_gpgga": _Sensor(
                        "$GPGGA,120000,4544.934,N,02112.522,E,1,09,0.9,90.0,M,,,,*00")
                    }[name]

        def set_rx_rate(self, r, ch=0): self.rate = fs_actual
        def set_rx_freq(self, tr, ch=0): self.freq = fc_actual
        def set_rx_gain(self, g, ch=0): pass
        def set_rx_bandwidth(self, b, ch=0): pass
        def set_rx_agc(self, e, ch=0): pass
        def set_clock_source(self, s): self.clk = s
        def set_time_source(self, s): self.tsrc = s
        def set_time_next_pps(self, ts): self._time = ts.get_real_secs()
        def get_time_now(self): return TimeSpec(self._now())
        def get_rx_rate(self, ch=0): return self.rate
        def get_rx_freq(self, ch=0): return self.freq
        def get_rx_stream(self, args): return _Streamer(self)

    m.types.TimeSpec = TimeSpec
    m.types.TuneRequest = TuneRequest
    m.types.RXMetadata = RXMetadata
    m.types.RXMetadataErrorCode = _Enum
    m.types.StreamMode = StreamMode
    m.types.StreamCMD = StreamCMD
    m.usrp.MultiUSRP = MultiUSRP
    m.usrp.StreamArgs = StreamArgs

    for name in ("uhd", "uhd.types", "uhd.usrp"):
        sys.modules.pop(name, None)
    sys.modules["uhd"] = m
    sys.modules["uhd.types"] = m.types
    sys.modules["uhd.usrp"] = m.usrp
    for mod in ("dronelocate.uhd_source", "node"):
        sys.modules.pop(mod, None)
    return m


def check_wiring():
    print("\n=== 2. node.py UHD wiring (fake device) ===")
    ok = True

    install_fake_uhd(node_enu=[0.0, 0.0, 45.0], t0_error_s=2.5e-7)
    import node as node_mod

    cfg = proto.SiteConfig.load("config/site-b210.json")
    # keep the test quick: the wiring does not care about snippet length
    radio = dict(cfg.radio)
    radio["fs_sps"] = FS
    radio["snippet_s"] = N / FS

    if not cfg.hardware or not cfg.capture:
        print("  FAIL  SiteConfig did not expose hardware/capture blocks")
        return False
    print(f"  PASS  config exposes hardware(channels={cfg.hardware['channels']}) "
          f"capture(mode={cfg.capture['mode']})")

    src = node_mod.UhdNodeSource(cfg, cfg.node("n05"), radio)
    src.n_samples = N

    if abs(src.fs - FS) > 1e-6:
        print(f"  FAIL  fs_actual not adopted: {src.fs}")
        ok = False
    else:
        print(f"  PASS  adopts hardware rate {src.fs/1e6:.4f} Msps")

    # slot grid: two nodes, started at different moments, must agree
    t_a, idx_a = src.next_slot(now=T_SLOT + 0.013)
    t_b, idx_b = src.next_slot(now=T_SLOT + 0.181)
    if (t_a, idx_a) != (t_b, idx_b):
        print(f"  FAIL  slot grid disagrees: {t_a}/{idx_a} vs {t_b}/{idx_b}")
        ok = False
    else:
        print(f"  PASS  slot grid agrees across nodes (burst id {idx_a})")

    iq, info, burst, t0, extra = src.capture()
    if iq.ndim != 1 or len(iq) != N:
        print(f"  FAIL  payload is not one channel of {N}: {iq.shape}")
        ok = False
    else:
        print(f"  PASS  publishes channel 0 only ({len(iq)} samples)")

    expect = -(t0 - extra["slot"])
    if abs(info["clock_error_s"] - expect) > 1e-15:
        print(f"  FAIL  clock_error_s {info['clock_error_s']} != {expect}")
        ok = False
    else:
        print(f"  PASS  clk offset {info['clock_error_s']*1e9:+.1f} ns "
              f"= -(t0_actual - slot)")

    missing = [k for k in ("bearing_deg", "coherence", "sched_err_ns",
                           "gps_locked") if k not in extra]
    if missing:
        print(f"  FAIL  event metadata missing {missing}")
        ok = False
    elif abs(extra["bearing_deg"] - TRUE_BEARING_DEG) > 0.5:
        print(f"  FAIL  bearing {extra['bearing_deg']:+.2f} deg, "
              f"expected {TRUE_BEARING_DEG:+.2f}")
        ok = False
    else:
        print(f"  PASS  dual-channel bearing {extra['bearing_deg']:+.2f} deg "
              f"(true {TRUE_BEARING_DEG:+.1f}), "
              f"coherence {extra['coherence']:.3f}, "
              f"unambiguous={extra['bearing_unambiguous']}")

    ev = proto.detection_event(
        node_id="n05", burst_id=burst, t_utc_ns=int(t0 * 1e9), fc_hz=src.fc,
        fs_sps=src.fs, bw_hz=BW, rssi_dbm=-40.0, snr_db=20.0, cap_id="x",
        n_samples=N, fmt="ci8", clock_offset_ns=info["clock_error_s"] * 1e9,
        clock_sigma_ns=50.0, lat=45.0, lon=21.0, alt_m=90.0, extra=extra)
    rt = proto.decode(proto.encode(ev))
    if rt["hw"]["mode"] != "scheduled" or "bearing_deg" not in rt["hw"]:
        print("  FAIL  event does not survive CBOR round trip")
        ok = False
    else:
        print(f"  PASS  event round-trips through CBOR "
              f"({len(proto.encode(ev))} B)")

    # Scheduled mode on an undisciplined clock must refuse, not degrade.
    # Neutering the GPS step leaves time_aligned False, which is exactly the
    # state a board with no sky view comes up in.
    import dronelocate.uhd_source as us
    orig = us.UhdSource._discipline_to_gps
    us.UhdSource._discipline_to_gps = lambda self, t, r: None
    try:
        node_mod.UhdNodeSource(cfg, cfg.node("n05"), radio)
        print("  FAIL  scheduled mode accepted an unaligned clock")
        ok = False
    except RuntimeError as e:
        if "scheduled" in str(e):
            print("  PASS  refuses scheduled mode without PPS alignment")
        else:
            print(f"  FAIL  wrong error: {e}")
            ok = False
    finally:
        us.UhdSource._discipline_to_gps = orig
    return ok


def main():
    sign_ok, worst = check_sign()
    wiring_ok = check_wiring()

    print("\n=== verdict ===")
    print(f"  timing convention : {'PASS' if sign_ok else 'FAIL'} "
          f"(worst {worst:.1f} m)")
    print(f"  node.py wiring    : {'PASS' if wiring_ok else 'FAIL'}")
    if not (sign_ok and wiring_ok):
        return 1
    print("\n  Plumbing verified against a fake device. Schedule jitter on the"
          "\n  real TinyB210 is still unmeasured -- that is hw_selftest.py.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
