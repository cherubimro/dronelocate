"""UHD source for B210-class hardware (TinyB210, B205mini, Ettus B2xx).

NOT YET VERIFIED AGAINST HARDWARE. The UHD Python bindings ship with the UHD
build rather than PyPI, so this was written from the API but not executed.
Run `python3 hw_selftest.py` before wiring it into the fleet.

Why this hardware changes the architecture
------------------------------------------
RTL-SDR and HackRF give you no way to know which sample index corresponds to a
UTC instant. You can discipline the sample *rate* with an external reference
and still not know the *phase*, because USB bulk transfer jitter destroys
host-side timestamps at the hundreds-of-microseconds level. Everything in
sigsim.py's clock-error model exists to represent that unknown.

UHD removes it. Two capabilities:

  1. Every buffer arrives with rx_metadata.time_spec -- the hardware's own
     timestamp for sample zero, referenced to the PPS-disciplined clock.

  2. Captures can be *scheduled*: stream_now=False with an absolute
     time_spec means the FPGA opens the window at a named UTC instant.

So instead of inferring alignment after the fact, the supernode names a time
and every node captures at that time. The only remaining differences between
node buffers are propagation delay -- which is the thing we want to measure.
"""

import threading
import time

import numpy as np

try:
    import uhd
    HAVE_UHD = True
except Exception:  # pragma: no cover
    uhd = None
    HAVE_UHD = False


class UhdError(RuntimeError):
    pass


class UhdSource:
    """A B210-class receiver disciplined to GPS, capable of timed captures.

    channels=[0] for single RX. channels=[0, 1] for the coherent pair, which
    share one LO and one ADC clock and are therefore usable as a two-element
    interferometer for bearing -- no inter-node timing required.
    """

    def __init__(self, fs, fc, gain_db=40, device="type=b200",
                 channels=(0,), require_gps=True, gps_timeout_s=180.0,
                 bandwidth_hz=None):
        if not HAVE_UHD:
            raise UhdError(
                "UHD Python bindings not found. They are built with UHD, not "
                "pip-installable, so they land in the site-packages of "
                "whichever Python UHD was compiled against -- check that it "
                "is the one running this process.\n"
                "  Debian/Ubuntu: apt install uhd-host python3-uhd\n"
                "  openSUSE:      zypper install libuhd4_5_0 uhd-utils "
                "uhd-udev uhd-firmware python3-uhd\n"
                "Then run uhd_find_devices to confirm the board enumerates."
            )
        self.channels = list(channels)
        self.fs = float(fs)
        self.fc = float(fc)
        self._lock = threading.Lock()

        self.usrp = uhd.usrp.MultiUSRP(device)
        self.mb_sensors = set(self.usrp.get_mboard_sensor_names(0))

        self.gps_present = "gps_locked" in self.mb_sensors
        self.clock_source = "internal"
        self.time_source = "internal"
        # Set on every path, not just the GPS-locked one. Callers branch on it
        # to decide whether scheduled capture is meaningful, and an attribute
        # that only exists after a successful lock raises AttributeError on
        # exactly the degraded path where you most need to read it.
        self.time_aligned = False

        if self.gps_present:
            self._discipline_to_gps(gps_timeout_s, require_gps)
        elif require_gps:
            raise UhdError(
                "No gps_locked sensor. This board has no GPSDO, so absolute "
                "timing is unavailable and TDOA will need an external PPS on "
                "the PPS input, or reference-emitter calibration. Pass "
                "require_gps=False to proceed without it."
            )

        for ch in self.channels:
            self.usrp.set_rx_rate(self.fs, ch)
            self.usrp.set_rx_freq(uhd.types.TuneRequest(self.fc), ch)
            self.usrp.set_rx_gain(float(gain_db), ch)
            if bandwidth_hz:
                self.usrp.set_rx_bandwidth(float(bandwidth_hz), ch)
            # AGC off: gain must be constant, because a gain change mid-run
            # rescales amplitude and corrupts cross-node RSSI comparison
            try:
                self.usrp.set_rx_agc(False, ch)
            except Exception:
                pass

        self.fs_actual = float(self.usrp.get_rx_rate(self.channels[0]))
        self.fc_actual = float(self.usrp.get_rx_freq(self.channels[0]))

        st = uhd.usrp.StreamArgs("fc32", "sc16")
        st.channels = self.channels
        self.streamer = self.usrp.get_rx_stream(st)
        self.md = uhd.types.RXMetadata()
        self._max_samps = self.streamer.get_max_num_samps()

    # --- timing ----------------------------------------------------------
    def _discipline_to_gps(self, timeout_s, require):
        """Lock to the GPSDO and align the device clock to UTC on a PPS edge.

        set_time_next_pps rather than set_time_now: the whole point is that
        the device's notion of time steps on a PPS edge, so all nodes agree
        to within their PPS accuracy rather than to within host scheduling
        jitter.
        """
        self.usrp.set_clock_source("gpsdo")
        self.usrp.set_time_source("gpsdo")
        self.clock_source = "gpsdo"
        self.time_source = "gpsdo"

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._sensor_bool("ref_locked") and self._sensor_bool("gps_locked"):
                break
            time.sleep(1.0)
        else:
            msg = (f"GPSDO did not lock within {timeout_s:.0f}s "
                   f"(ref_locked={self._sensor_bool('ref_locked')}, "
                   f"gps_locked={self._sensor_bool('gps_locked')})")
            if require:
                raise UhdError(msg)
            print(f"[uhd] WARNING {msg}")
            return

        # step the device clock to the next whole UTC second on a PPS edge
        gps_t = self._sensor_int("gps_time")
        target = (gps_t + 1) if gps_t is not None else int(time.time()) + 1
        self.usrp.set_time_next_pps(uhd.types.TimeSpec(float(target)))
        time.sleep(1.1)
        self.time_aligned = True

    def _strerror(self, err):
        """UHD's own description of an rx error, when it will give us one."""
        try:
            return f"{err} ({self.md.strerror()})"
        except Exception:
            return str(err)

    def _sensor_bool(self, name):
        if name not in self.mb_sensors:
            return False
        try:
            return bool(self.usrp.get_mboard_sensor(name, 0).to_bool())
        except Exception:
            return False

    def _sensor_int(self, name):
        if name not in self.mb_sensors:
            return None
        try:
            return int(self.usrp.get_mboard_sensor(name, 0).to_int())
        except Exception:
            return None

    def device_time(self):
        return float(self.usrp.get_time_now().get_real_secs())

    def gps_position(self):
        """Surveyed position from the onboard GNSS, parsed from GPGGA.

        Use this for a sanity check, not as your surveyed node position. A
        single GNSS fix is 3-5 m, and node position error propagates one-to-one
        into every target fix. Survey properly once and hardcode the result.
        """
        if "gps_gpgga" not in self.mb_sensors:
            return None
        try:
            s = self.usrp.get_mboard_sensor("gps_gpgga", 0).value
        except Exception:
            return None
        f = s.split(",")
        if len(f) < 10 or not f[2]:
            return None

        def dm(v, hemi, deg_digits):
            d = float(v[:deg_digits])
            m = float(v[deg_digits:])
            x = d + m / 60.0
            return -x if hemi in ("S", "W") else x

        try:
            return {"lat": dm(f[2], f[3], 2), "lon": dm(f[4], f[5], 3),
                    "alt_m": float(f[9]) if f[9] else None,
                    "fix_quality": int(f[6]) if f[6] else 0,
                    "n_sats": int(f[7]) if f[7] else 0}
        except (ValueError, IndexError):
            return None

    # --- capture ---------------------------------------------------------
    def capture_at(self, t_utc, n_samples, timeout_pad_s=2.0):
        """Capture n_samples starting at absolute UTC time t_utc.

        Returns (iq, t0_actual, meta). iq is shape (n_channels, n_samples).
        t0_actual is the hardware's timestamp for sample zero -- use that, not
        t_utc, since the device rounds to a sample boundary.

        The supernode issuing one t_utc to all nodes is what makes the
        captures genuinely simultaneous rather than merely close together.
        """
        n = int(n_samples)
        nch = len(self.channels)
        buf = np.zeros((nch, n), dtype=np.complex64)
        # Scratch must stay whole and C-contiguous for the lifetime of the
        # recv. The UHD Python binding hands numpy's raw pointer to the C++
        # streamer, so a strided view -- e.g. chunk[:, :want] on a 2-channel
        # array, whose rows are max_samps apart -- is either rejected or,
        # worse, force-cast to a temporary that receives the samples and is
        # then thrown away. That failure mode is silent and only appears with
        # channels=[0, 1], which is the configuration the B210 profile uses.
        # So: always recv into the full scratch, and clamp on the copy out.
        chunk = np.zeros((nch, self._max_samps), dtype=np.complex64)

        cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        cmd.num_samps = n
        cmd.stream_now = False
        cmd.time_spec = uhd.types.TimeSpec(float(t_utc))

        with self._lock:
            lead = float(t_utc) - self.device_time()
            if lead < 0.05:
                raise UhdError(
                    f"capture time is only {lead*1e3:.1f} ms ahead of device "
                    f"clock; schedule further out")
            self.streamer.issue_stream_cmd(cmd)

            got = 0
            t0_actual = None
            timeout = max(lead, 0.0) + n / self.fs_actual + timeout_pad_s
            while got < n:
                rx = self.streamer.recv(chunk, self.md, timeout)
                err = self.md.error_code
                if err == uhd.types.RXMetadataErrorCode.timeout:
                    raise UhdError("stream timeout waiting for scheduled capture")
                if err == uhd.types.RXMetadataErrorCode.late:
                    # the FPGA was handed a start time that had already passed
                    raise UhdError(
                        "late command: scheduled capture instant had already "
                        "passed at the device; increase schedule_lead_s")
                if err == uhd.types.RXMetadataErrorCode.overflow:
                    # dropped samples break the sample->time mapping, so the
                    # capture is unusable for TDOA rather than merely degraded
                    raise UhdError("overflow during capture; sample timing lost")
                if err != uhd.types.RXMetadataErrorCode.none:
                    raise UhdError(f"UHD rx error: {self._strerror(err)}")
                if rx <= 0:
                    continue
                if t0_actual is None and self.md.has_time_spec:
                    t0_actual = float(self.md.time_spec.get_real_secs())
                take = min(int(rx), n - got)
                buf[:, got:got + take] = chunk[:, :take]
                got += take

        if t0_actual is None:
            raise UhdError("no time_spec in rx metadata; timing unusable")

        return buf, t0_actual, {
            "requested_t0": float(t_utc),
            "actual_t0": t0_actual,
            "schedule_error_s": t0_actual - float(t_utc),
            "fs_actual": self.fs_actual,
            "fc_actual": self.fc_actual,
            "gps_locked": self._sensor_bool("gps_locked"),
            "ref_locked": self._sensor_bool("ref_locked"),
            "n_channels": nch,
        }

    def capture_now(self, n_samples, timeout_pad_s=2.0):
        """Free-running capture, starting as soon as the FPGA can.

        The bench fallback for a board with no GPSDO lock -- an indoor first
        run, or a clone whose PPS path turns out to be unusable. It still
        returns the hardware's own timestamp for sample zero, so bearing work
        and signal checks are fully available; what it cannot give you is a
        start time shared with the other nodes, so TDOA across nodes is not
        meaningful from these captures.
        """
        n = int(n_samples)
        nch = len(self.channels)
        buf = np.zeros((nch, n), dtype=np.complex64)
        chunk = np.zeros((nch, self._max_samps), dtype=np.complex64)

        cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        cmd.num_samps = n

        # A multi-channel stream cannot start "now". UHD's own recv_num_samps
        # gates this the same way -- stream_now only when len(channels) == 1 --
        # because aligning two channels requires the FPGA to start them on a
        # common timed trigger. Ask for stream_now on a 2-channel B210 and the
        # channels are free to begin on different samples, which silently
        # destroys the phase relationship the bearing math depends on.
        with self._lock:
            if nch == 1:
                cmd.stream_now = True
                lead = 0.0
            else:
                lead = 0.05
                cmd.stream_now = False
                cmd.time_spec = uhd.types.TimeSpec(self.device_time() + lead)

            self.streamer.issue_stream_cmd(cmd)
            got = 0
            t0_actual = None
            timeout = lead + n / self.fs_actual + timeout_pad_s
            while got < n:
                rx = self.streamer.recv(chunk, self.md, timeout)
                err = self.md.error_code
                if err == uhd.types.RXMetadataErrorCode.timeout:
                    raise UhdError("stream timeout during free-running capture")
                if err == uhd.types.RXMetadataErrorCode.overflow:
                    raise UhdError("overflow during capture; sample timing lost")
                if err != uhd.types.RXMetadataErrorCode.none:
                    raise UhdError(f"UHD rx error: {self._strerror(err)}")
                if rx <= 0:
                    continue
                if t0_actual is None and self.md.has_time_spec:
                    t0_actual = float(self.md.time_spec.get_real_secs())
                take = min(int(rx), n - got)
                buf[:, got:got + take] = chunk[:, :take]
                got += take

        if t0_actual is None:
            t0_actual = self.device_time()

        return buf, t0_actual, {
            "requested_t0": None,
            "actual_t0": t0_actual,
            # no requested instant, so there is no schedule error to report.
            # None rather than 0.0: zero would read as a perfect capture.
            "schedule_error_s": None,
            "fs_actual": self.fs_actual,
            "fc_actual": self.fc_actual,
            "gps_locked": self._sensor_bool("gps_locked"),
            "ref_locked": self._sensor_bool("ref_locked"),
            "n_channels": nch,
        }

    def retune(self, fc):
        with self._lock:
            for ch in self.channels:
                self.usrp.set_rx_freq(uhd.types.TuneRequest(float(fc)), ch)
            self.fc = float(fc)
            self.fc_actual = float(self.usrp.get_rx_freq(self.channels[0]))

    def health(self):
        return {
            "gps_locked": self._sensor_bool("gps_locked"),
            "ref_locked": self._sensor_bool("ref_locked"),
            "clock_source": self.clock_source,
            "time_source": self.time_source,
            "device_time": self.device_time(),
            "host_offset_s": self.device_time() - time.time(),
            "fs_actual": self.fs_actual,
            "fc_actual": self.fc_actual,
            "n_sats": (self.gps_position() or {}).get("n_sats"),
        }

    def close(self):
        # Tell the FPGA to stop before dropping the streamer. Without this a
        # half-finished num_done command can leave the device streaming into
        # a queue nobody drains, and the next process to open the board
        # inherits an overflow it did not cause.
        try:
            stop = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
            self.streamer.issue_stream_cmd(stop)
        except Exception:
            pass
        self.streamer = None


def phase_bearing(iq_pair, fs, fc, baseline_m, cal_phase_rad=0.0):
    """Bearing from a coherent two-element pair, via phase interferometry.

    Two channels on one B210 share an LO and an ADC clock, so their relative
    phase is meaningful. This needs no inter-node timing at all, which makes
    it the fallback that keeps working when GPS is jammed or spoofed.

    Ambiguity warning: with element spacing above half a wavelength the
    solution is multi-valued. At 2.44 GHz, lambda/2 is 61 mm. Wider spacing
    buys precision and costs uniqueness; two elements also cannot resolve
    front from back. Three or more elements fix both.
    """
    a, b = iq_pair[0], iq_pair[1]
    # vdot(a, b) = sum(conj(a) * b), so the phase is that of b relative to a.
    # Reversing the arguments conjugates the result and mirrors every bearing
    # about the array normal -- a sign error that looks entirely plausible in
    # the output and puts targets on the wrong side.
    xc = np.vdot(a, b)
    dphi = np.angle(xc) - cal_phase_rad
    dphi = (dphi + np.pi) % (2.0 * np.pi) - np.pi

    lam = 299792458.0 / float(fc)
    d = float(baseline_m)

    # A two-element interferometer is unambiguous only when the spacing is at
    # or below lambda/2. Above that, the measured phase wraps and several true
    # angles map to the same reading -- and crucially, they do so while
    # |sin(theta)| stays within 1, so a range check on the result cannot
    # detect it. Uniqueness is a property of the array, not of the answer.
    half_wave = lam / 2.0
    unambiguous = d <= half_wave * 1.001

    # Enumerate every angle consistent with the wrapped phase (grating lobes).
    candidates = []
    kmax = int(np.ceil(2.0 * d / lam)) + 1
    for k in range(-kmax, kmax + 1):
        s = (dphi + 2.0 * np.pi * k) * lam / (2.0 * np.pi * d)
        if abs(s) <= 1.0:
            th = float(np.degrees(np.arcsin(s)))
            if not any(abs(th - c) < 1e-6 for c in candidates):
                candidates.append(th)
    candidates.sort(key=abs)

    primary = candidates[0] if candidates else float("nan")
    return {
        "bearing_deg": primary,
        "bearing_rad": float(np.radians(primary)) if candidates else float("nan"),
        "candidates_deg": candidates,
        "dphi_rad": float(dphi),
        "coherence": float(np.abs(xc) /
                           (np.linalg.norm(a) * np.linalg.norm(b) + 1e-20)),
        "unambiguous": bool(unambiguous),
        "half_wave_spacing_m": half_wave,
        # two elements also cannot separate front from back: this cone
        # ambiguity survives any spacing and needs a third element
        "mirror_deg": float(180.0 - primary) if candidates else float("nan"),
    }
