#!/usr/bin/env python3
"""Sensor node. One process per node; identical code for emulated and real.

    python3 node.py --node n03                  # emulated
    python3 node.py --node n05 --source rtlsdr  # real dongle via SoapySDR
    python3 node.py --node n05 --source uhd \\
        --config config/site-b210.json          # TinyB210 via UHD

The only difference between the three is where IQ comes from. Detection,
buffering, wire format, QoS and the query interface are shared, so what you
validate with nine emulated nodes is the same code path the real one runs.
"""

import argparse
import collections
import threading
import time
import uuid

import numpy as np
import zenoh

from dronelocate import proto, zconf
from dronelocate.geo import LocalFrame
from dronelocate.sigsim import NodeChannel, bytes_per_sample, quantize


class RingBuffer:
    """Bounded capture store. The supernode pulls from here on demand rather
    than every node pushing IQ on every detection -- that is what keeps the
    aggregate uplink proportional to interesting events, not to node count."""

    def __init__(self, max_captures=256, max_bytes=256 * 1024 * 1024):
        self._d = collections.OrderedDict()
        self._lock = threading.Lock()
        self.max_captures = max_captures
        self.max_bytes = max_bytes
        self.bytes_held = 0

    def put(self, cap_id, payload, meta):
        with self._lock:
            self._d[cap_id] = (payload, meta)
            self.bytes_held += len(payload)
            while len(self._d) > self.max_captures or self.bytes_held > self.max_bytes:
                _, (p, _) = self._d.popitem(last=False)
                self.bytes_held -= len(p)

    def get(self, cap_id):
        with self._lock:
            return self._d.get(cap_id)

    def __len__(self):
        with self._lock:
            return len(self._d)


class SimSource:
    """Renders this node's view of the scene from published ground truth."""

    def __init__(self, cfg, node_cfg, frame, rng):
        s = cfg.sim
        r = cfg.radio
        self.node_enu = np.array(node_cfg["enu"], dtype=float)
        self.fs = float(r["fs_sps"])
        self.bw = float(r["bw_hz"])
        self.snr_at_1km = float(s.get("snr_db_at_1km", 25.0))

        # Per-node clock error. This is the quantity the reference-emitter
        # calibration loop is supposed to estimate; here we know it exactly,
        # which is what lets the console score the calibration.
        self.clock_bias_s = rng.normal(0.0, float(s.get("clock_bias_sigma_s", 2e-8)))
        self.clock_drift_ppm = rng.normal(0.0, float(s.get("clock_drift_ppm_sigma", 0.02)))

        self.channel = NodeChannel(
            self.node_enu, self.fs, self.bw,
            clock_bias_s=self.clock_bias_s,
            clock_drift_ppm=self.clock_drift_ppm,
            rng_seed=int(rng.integers(0, 2 ** 31)),
            discipline=s.get("clock_discipline", "gpsdo"),
            pps_jitter_s=float(s.get("pps_jitter_s", 15e-9)),
            discipline_tau_s=float(s.get("discipline_tau_s", 100.0)),
        )

    @property
    def clock_mode(self):
        return self.channel.discipline

    def set_clock_mode(self, mode):
        """Retaskable at runtime so the console can show each regime live."""
        self.channel.set_discipline(mode, t_s=time.time())

    def render(self, truth, n_samples):
        return self.channel.render(
            emitter_enu=np.array(truth["enu"], dtype=float),
            burst_id=truth["burst"],
            n_samples=n_samples,
            t_epoch_s=truth["t_emit_s"],
            capture_start_s=truth["t_emit_s"],
            snr_db_at_1km=self.snr_at_1km,
        )


class RtlSdrSource:
    """Real capture through SoapySDR.

    SoapySDR rather than librtlsdr directly, so the same code drives a HackRF
    by changing one string. Note the RTL-SDR's R820T2 tops out near 1766 MHz,
    so it cannot see 2.4 GHz drone links -- use 1090 MHz ADS-B for a live
    test, where aircraft broadcast their own position as ground truth.
    """

    def __init__(self, fs, fc, gain_db=32, device="driver=rtlsdr"):
        import SoapySDR
        from SoapySDR import SOAPY_SDR_CF32, SOAPY_SDR_RX

        self._SoapySDR = SoapySDR
        self.dev = SoapySDR.Device(dict(kv.split("=") for kv in device.split(",")))
        self.dev.setSampleRate(SOAPY_SDR_RX, 0, fs)
        self.dev.setFrequency(SOAPY_SDR_RX, 0, fc)
        try:
            self.dev.setGain(SOAPY_SDR_RX, 0, gain_db)
        except Exception:
            pass
        self.stream = self.dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        self.dev.activateStream(self.stream)
        self.fs = fs

    def read(self, n_samples):
        from SoapySDR import errToStr

        buf = np.empty(n_samples, dtype=np.complex64)
        got = 0
        t0 = time.time()
        while got < n_samples:
            chunk = buf[got:]
            sr = self.dev.readStream(self.stream, [chunk], len(chunk), timeoutUs=1000000)
            if sr.ret > 0:
                got += sr.ret
            elif time.time() - t0 > 3.0:
                raise RuntimeError(f"SDR read stalled: {errToStr(sr.ret)}")
        return buf, {"true_range_m": float("nan"), "snr_db": float("nan"),
                     "clock_error_s": 0.0}

    def retune(self, fc):
        from SoapySDR import SOAPY_SDR_RX
        self.dev.setFrequency(SOAPY_SDR_RX, 0, fc)


class UhdNodeSource:
    """B210-class capture, adapted to the node's event model.

    UhdSource is the bare driver. This is the integration layer: it turns a
    hardware capture into the (iq, info) pair the rest of the node already
    understands, and it owns the one piece of arithmetic that decides whether
    TDOA works at all -- the mapping from UHD's absolute timestamps onto the
    clk_off_ns field the correlator subtracts.

    The timing argument, since getting the sign wrong here fails silently
    (plausible positions, wrong by hundreds of metres):

      Node i's buffer begins at absolute time t0_i, reported by the hardware.
      A burst emitted at T_e arrives at node i at T_e + tof_i, so it sits at
      buffer index (T_e + tof_i - t0_i) * fs. The correlator returns
          lag = (tof_i - tof_ref) - (t0_i - t0_ref)
      and the supernode forms  lag - (clk_off_i - clk_off_ref) * 1e-9.
      For that to leave exactly (tof_i - tof_ref) -- the TDOA we want -- we
      need clk_off_i = -(t0_i - t_slot) * 1e9 for any instant t_slot the
      nodes agree on. t_slot cancels in the difference; it exists only to
      keep the number small enough for float64 to carry nanoseconds, which
      raw Unix-epoch timestamps (~1.8e18 ns) cannot.

    Note this works whether or not the capture was scheduled. Scheduling
    makes the windows *overlap*, which is what guarantees the burst is in
    everyone's buffer; the timestamp is what makes them *comparable*. Those
    are two separate jobs and only the second one is arithmetic.
    """

    def __init__(self, cfg, node_cfg, radio, device=None):
        from dronelocate.uhd_source import UhdSource, phase_bearing

        self._phase_bearing = phase_bearing
        hw = dict(cfg.hardware or {})
        cap = dict(cfg.capture or {})

        self.channels = tuple(int(c) for c in hw.get("channels", [0]))
        self.baseline_m = float(hw.get("array_baseline_m", 0.0))
        self.cal_phase_rad = float(hw.get("cal_phase_rad", 0.0))

        self.mode = cap.get("mode", "independent")
        self.rate_hz = float(cap.get("rate_hz", 2.0))
        self.lead_s = float(cap.get("schedule_lead_s", 0.5))
        # Set by Node once the hardware's real sample rate is known, since
        # snippet length depends on it.
        self.n_samples = None

        self.src = UhdSource(
            fs=float(radio["fs_sps"]),
            fc=float(radio["fc_hz"]),
            gain_db=float(node_cfg.get("gain_db", hw.get("gain_db", 40))),
            device=device or hw.get("device_args", "type=b200"),
            channels=self.channels,
            require_gps=bool(hw.get("require_gps", True)),
            gps_timeout_s=float(hw.get("gps_timeout_s", 180.0)),
            bandwidth_hz=radio.get("bw_hz"),
        )

        # The B210 rounds the requested rate to a master-clock divisor. Report
        # what the hardware actually does: the correlator converts lag indices
        # to seconds with this number, so a requested-vs-actual mismatch is a
        # multiplicative bias on every TDOA in the system.
        self.fs = self.src.fs_actual
        self.fc = self.src.fc_actual

        if self.mode == "scheduled" and not self.src.time_aligned:
            # Refusing here rather than degrading quietly: a scheduled capture
            # on an undisciplined clock produces buffers that look fine and
            # are not simultaneous, which is the worst possible failure.
            raise RuntimeError(
                "capture.mode is 'scheduled' but the device clock was never "
                "aligned to a PPS edge (no GPSDO lock). Fix the GPS antenna, "
                "or set capture.mode to 'independent' for bench work.")

    def next_slot(self, now=None):
        """The next instant on the shared UTC grid.

        Every node derives its slots from UTC alone, so ten independently
        started nodes agree on the same instants without exchanging a message.
        This is the placeholder for the supernode-driven coordinator; when
        that lands, the slot comes off the bus instead of off the clock. The
        burst id is the slot index, which is what lets the supernode group
        events from different nodes as one burst.
        """
        now = time.time() if now is None else now
        step = 1.0 / self.rate_hz
        idx = int(np.floor(now / step)) + 1
        # Take the next slot on the grid, and only skip ahead if it is too
        # close for the FPGA to accept the command. Folding lead_s into the
        # floor instead would push every capture a whole extra period out and
        # quietly halve the configured rate.
        while idx * step - now < self.lead_s:
            idx += 1
        return idx * step, idx

    def capture(self):
        """One capture.

        Returns (iq_ch0, info, burst_id, t0_actual, extra).
        """
        t_slot, burst_id = self.next_slot()
        if self.mode == "scheduled":
            iq, t0, meta = self.src.capture_at(t_slot, self.n_samples)
        else:
            iq, t0, meta = self.src.capture_now(self.n_samples)

        info = {
            # sign derived in the class docstring; do not flip without
            # re-deriving, the failure is silent
            "clock_error_s": -(t0 - t_slot),
            "true_range_m": float("nan"),
            "snr_db": float("nan"),
        }
        extra = {
            "t0_actual": t0,
            "slot": t_slot,
            "sched_err_ns": (meta["schedule_error_s"] * 1e9
                             if meta["schedule_error_s"] is not None else None),
            "gps_locked": meta["gps_locked"],
            "ref_locked": meta["ref_locked"],
            "mode": self.mode,
        }

        # Dual-channel bearing. Free at capture time and independent of every
        # other node, so it survives GPS jamming. Carried as advisory metadata:
        # solve_tdoa() does not consume it yet (priority item 4).
        if iq.shape[0] > 1 and self.baseline_m > 0:
            b = self._phase_bearing(iq, self.fs, self.fc, self.baseline_m,
                                    self.cal_phase_rad)
            extra["bearing_deg"] = b["bearing_deg"]
            extra["bearing_candidates_deg"] = b["candidates_deg"]
            extra["bearing_unambiguous"] = b["unambiguous"]
            extra["coherence"] = b["coherence"]

        # Channel 0 is the TDOA payload. Publishing both would double the
        # uplink to buy nothing the correlator can use.
        return np.ascontiguousarray(iq[0]), info, burst_id, t0, extra

    def retune(self, fc):
        self.src.retune(fc)
        self.fc = self.src.fc_actual

    def health(self):
        return self.src.health()

    def close(self):
        self.src.close()


class Node:
    def __init__(self, cfg, node_id, source_kind, device, seed=None, zc=None):
        self.cfg = cfg
        self.node_id = node_id
        self.node_cfg = cfg.node(node_id)
        self.radio = dict(cfg.radio)
        self.frame = LocalFrame.from_dict(cfg.origin)
        self.ring = RingBuffer()
        self.rng = np.random.default_rng(seed if seed is not None
                                         else abs(hash(node_id)) % (2 ** 31))

        enu = np.array(self.node_cfg["enu"], dtype=float)
        self.lat, self.lon, self.alt = self.frame.to_geodetic(enu)

        self.fmt = self.radio["wire_fmt"]

        self.source_kind = source_kind
        if source_kind == "sim":
            self.source = SimSource(cfg, self.node_cfg, self.frame, self.rng)
        elif source_kind == "uhd":
            self.source = UhdNodeSource(cfg, self.node_cfg, self.radio, device)
            # Adopt the rate and frequency the hardware actually settled on,
            # before n_samples is derived from them.
            self.radio["fs_sps"] = self.source.fs
            self.radio["fc_hz"] = self.source.fc
        else:
            self.source = RtlSdrSource(
                self.radio["fs_sps"], self.radio["fc_hz"],
                self.node_cfg.get("gain_db", 32), device)

        self.n_samples = int(self.radio["fs_sps"] * self.radio["snippet_s"])
        if source_kind == "uhd":
            self.source.n_samples = self.n_samples

        # How much this node's timestamps can be trusted, in ns. The sim's
        # 15 ns is a stand-in for a calibrated clock; a GPSDO-disciplined
        # B210 should be set from the schedule jitter hw_selftest.py measures,
        # because that number is what the solver weights measurements by.
        self.clock_sigma_ns = float(
            (cfg.hardware or {}).get("timing_sigma_ns", 15.0)
            if source_kind == "uhd" else 15.0)

        self.stats = {"detections": 0, "iq_served": 0, "bytes_served": 0,
                      "started": time.time()}

        self.session = zenoh.open(zc or zenoh.Config())
        site = cfg.site

        # Detection events must never be dropped and must not queue behind a
        # multi-hundred-KB IQ transfer, hence interactive_high + block.
        self.pub_evt = self.session.declare_publisher(
            proto.ke_detect(site, node_id),
            priority=zenoh.Priority.INTERACTIVE_HIGH,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )
        # Health is a freshness signal; a stale one is worthless, so drop
        # rather than apply backpressure.
        self.pub_health = self.session.declare_publisher(
            proto.ke_health(site, node_id),
            priority=zenoh.Priority.DATA_LOW,
            congestion_control=zenoh.CongestionControl.DROP,
        )
        self.qbl = self.session.declare_queryable(
            proto.ke_iq(site, node_id), self._on_iq_query)
        self.sub_cmd = self.session.declare_subscriber(
            proto.ke_cmd(site, node_id), self._on_cmd)

        if source_kind == "sim":
            self.sub_truth = self.session.declare_subscriber(
                proto.ke_truth(site), self._on_truth)

        self._stop = threading.Event()
        threading.Thread(target=self._health_loop, daemon=True).start()
        if source_kind == "uhd":
            threading.Thread(target=self._uhd_loop, daemon=True).start()
        elif source_kind != "sim":
            threading.Thread(target=self._hw_loop, daemon=True).start()

    # --- inbound ---------------------------------------------------------
    def _on_truth(self, sample):
        self.stats["truth_rx"] = self.stats.get("truth_rx", 0) + 1
        try:
            truth = proto.decode(sample.payload)
            iq, info = self.source.render(truth, self.n_samples)
            self._process(iq, info, burst_id=truth["burst"],
                          t_emit_s=truth["t_emit_s"])
        except Exception as e:  # a node failing must not take down the fleet
            print(f"[{self.node_id}] render error: {e}")

    def _hw_loop(self):
        burst = 0
        while not self._stop.is_set():
            try:
                iq, info = self.source.read(self.n_samples)
                self._process(iq, info, burst_id=burst, t_emit_s=time.time())
                burst += 1
            except Exception as e:
                print(f"[{self.node_id}] capture error: {e}")
                time.sleep(0.5)

    def _uhd_loop(self):
        """Timed captures on the shared UTC grid.

        t_emit_s is the hardware's timestamp for sample zero, not a host
        clock reading. That is the whole reason for using this hardware: the
        host's idea of when a buffer arrived is worth hundreds of
        microseconds, and we need tens of nanoseconds.
        """
        fails = 0
        while not self._stop.is_set():
            try:
                iq, info, burst, t0, extra = self.source.capture()
                self._process(iq, info, burst_id=burst, t_emit_s=t0, extra=extra)
                fails = 0
            except Exception as e:
                fails += 1
                print(f"[{self.node_id}] uhd capture error: {e}")
                # Back off on a persistent fault rather than spinning on a
                # dead USB link and burning a core.
                time.sleep(min(0.2 * fails, 5.0))

    def _on_cmd(self, sample):
        """Downlink retasking. With 20 MHz of instantaneous bandwidth against
        a threat space spanning 433 MHz to 5.8 GHz, centrally coordinated
        dwell is the difference between a set of sensors and a system."""
        try:
            cmd = proto.decode(sample.payload)
            if "fc_hz" in cmd:
                self.radio["fc_hz"] = float(cmd["fc_hz"])
                if hasattr(self.source, "retune"):
                    self.source.retune(self.radio["fc_hz"])
                print(f"[{self.node_id}] retuned to {self.radio['fc_hz']/1e6:.3f} MHz")
            if "wire_fmt" in cmd:
                self.fmt = cmd["wire_fmt"]
                print(f"[{self.node_id}] wire format now {self.fmt}")
            if "clock_mode" in cmd:
                # Only meaningful for emulated nodes -- a real radio's clock
                # discipline is a property of its hardware, not a setting.
                if hasattr(self.source, "set_clock_mode"):
                    self.source.set_clock_mode(str(cmd["clock_mode"]))
                    print(f"[{self.node_id}] clock discipline now "
                          f"{self.source.clock_mode}")
        except Exception as e:
            print(f"[{self.node_id}] bad command: {e}")

    def _on_iq_query(self, query):
        """Serve a capture from the ring buffer.

        Background priority so a burst of these cannot starve the event
        stream, and block so we never silently truncate a capture the
        correlator is waiting on."""
        try:
            params = query.parameters
            cap_id = params.get("cap_id") if hasattr(params, "get") else None
            if cap_id is None:
                cap_id = str(params).split("cap_id=")[-1].split("&")[0]
            hit = self.ring.get(cap_id)
            if hit is None:
                query.reply_err(proto.encode({"err": "no such capture",
                                              "cap_id": cap_id}))
                return
            payload, meta = hit
            query.reply(
                proto.ke_iq(self.cfg.site, self.node_id), payload,
                attachment=proto.encode(meta),
                priority=zenoh.Priority.BACKGROUND,
                congestion_control=zenoh.CongestionControl.BLOCK,
            )
            self.stats["iq_served"] += 1
            self.stats["bytes_served"] += len(payload)
        except Exception as e:
            print(f"[{self.node_id}] query error: {e}")

    # --- processing ------------------------------------------------------
    @staticmethod
    def _detect(iq, n_blocks=64):
        """Block-power detector.

        Peak block against the median block. The median is the noise floor
        estimator rather than the mean because the mean is dragged upward by
        the very burst we are trying to measure -- that mistake produces a
        detector that reports ~0 dB SNR on a strong signal and silently
        detects nothing.
        """
        n = len(iq)
        bs = max(n // n_blocks, 1)
        usable = (n // bs) * bs
        p = (np.abs(iq[:usable].reshape(-1, bs)) ** 2).mean(axis=1)
        noise = float(np.median(p))
        peak = float(np.max(p))
        snr_db = 10.0 * np.log10(max((peak - noise) / max(noise, 1e-20), 1e-12))
        return snr_db, peak, noise

    def _process(self, iq, info, burst_id, t_emit_s, extra=None):
        snr_db, peak, noise = self._detect(iq)
        rssi_dbm = 10.0 * np.log10(max(peak, 1e-20)) - 60.0

        if snr_db < self.radio.get("detect_threshold_db", 8.0) and self.source_kind == "sim":
            self.stats["below_thresh"] = self.stats.get("below_thresh", 0) + 1
            return  # below threshold: this node simply does not see it

        cap_id = uuid.uuid4().hex[:16]
        payload = quantize(iq, self.fmt)
        t0_ns = int(t_emit_s * 1e9)

        clk_off_ns = info.get("clock_error_s", 0.0) * 1e9
        meta = proto.iq_metadata(
            self.node_id, cap_id, burst_id, t0_ns, self.radio["fs_sps"],
            self.radio["fc_hz"], self.fmt, self.n_samples, clk_off_ns)
        self.ring.put(cap_id, payload, meta)

        evt = proto.detection_event(
            node_id=self.node_id, burst_id=burst_id, t_utc_ns=t0_ns,
            fc_hz=self.radio["fc_hz"], fs_sps=self.radio["fs_sps"],
            bw_hz=self.radio["bw_hz"], rssi_dbm=rssi_dbm, snr_db=snr_db,
            cap_id=cap_id, n_samples=self.n_samples, fmt=self.fmt,
            clock_offset_ns=clk_off_ns, clock_sigma_ns=self.clock_sigma_ns,
            lat=self.lat, lon=self.lon, alt_m=self.alt,
            classification="uas_datalink", confidence=0.7, extra=extra)
        self.pub_evt.put(proto.encode(evt))
        self.stats["detections"] += 1

    def _health_loop(self):
        while not self._stop.is_set():
            up = time.time() - self.stats["started"]
            mbps = self.stats["bytes_served"] * 8 / 1e6 / max(up, 1e-6)

            # Real lock state where we have a radio that knows it. Reporting a
            # hardcoded True from a board whose GPSDO has dropped is how an
            # operator ends up trusting a degraded fix.
            gps_lock = True
            hw = None
            if self.source_kind == "uhd":
                try:
                    hw = self.source.health()
                    gps_lock = bool(hw.get("gps_locked", False))
                except Exception as e:
                    hw = {"error": str(e)}
                    gps_lock = False

            self.pub_health.put(proto.encode({
                "node": self.node_id, "t_ns": proto.now_ns(), "up_s": up,
                "detections": self.stats["detections"],
                "truth_rx": self.stats.get("truth_rx", 0),
                "below_thresh": self.stats.get("below_thresh", 0),
                "iq_served": self.stats["iq_served"],
                "uplink_mbps": mbps, "ring": len(self.ring),
                "ring_mb": self.ring.bytes_held / 1e6,
                "fc_hz": self.radio["fc_hz"], "fmt": self.fmt,
                "source": self.source_kind, "gps_lock": gps_lock,
                "hw": hw,
                "clock_mode": getattr(self.source, "clock_mode", None),
                "enu": list(map(float, self.node_cfg["enu"])),
                "lat": self.lat, "lon": self.lon, "alt_m": self.alt,
            }))
            time.sleep(1.0)

    def run(self):
        rate = bytes_per_sample(self.fmt) * self.radio["fs_sps"] * 8 / 1e6
        detail = ""
        if self.source_kind == "uhd":
            detail = (f" | capture {self.source.mode} @ {self.source.rate_hz:g} Hz"
                      f" ch{list(self.source.channels)}")
        print(f"[{self.node_id}] {self.source_kind} @ "
              f"{self.radio['fs_sps']/1e6:.2f} Msps {self.fmt} | "
              f"snippet {self.n_samples} samp = "
              f"{self.n_samples*bytes_per_sample(self.fmt)/1024:.0f} KB | "
              f"continuous-equivalent {rate:.1f} Mbps{detail}")
        try:
            while not self._stop.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            if hasattr(self.source, "close"):
                try:
                    self.source.close()
                except Exception:
                    pass
            self.session.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/site.json")
    ap.add_argument("--node", required=True)
    ap.add_argument("--source", choices=["sim", "rtlsdr", "uhd"], default=None)
    ap.add_argument("--device", default=None,
                    help="SoapySDR or UHD device args; defaults to "
                         "driver=rtlsdr for rtlsdr, or hardware.device_args "
                         "from the site config for uhd")
    ap.add_argument("--capture-mode", choices=["scheduled", "independent"],
                    default=None, help="override capture.mode for a uhd node")
    ap.add_argument("--seed", type=int, default=None)
    zconf.add_args(ap)
    a = ap.parse_args()

    cfg = proto.SiteConfig.load(a.config)
    if a.capture_mode:
        cfg.capture = dict(cfg.capture or {}, mode=a.capture_mode)

    kind = a.source
    if kind is None:
        # A node marked "hw" means real hardware; which driver is a property
        # of the site, not of the node, so it comes from the hardware block.
        if cfg.node(a.node).get("role") == "sim":
            kind = "sim"
        else:
            kind = "uhd" if (cfg.hardware or {}).get("driver") == "uhd" else "rtlsdr"

    device = a.device
    if device is None:
        device = ((cfg.hardware or {}).get("device_args", "type=b200")
                  if kind == "uhd" else "driver=rtlsdr")

    zc = zconf.spoke(a.hub, a.port, zconf.tls_from_args(a))
    Node(cfg, a.node, kind, device, a.seed, zc).run()


if __name__ == "__main__":
    main()
