"""UHD source for B210-class hardware (TinyB210, B205mini, Ettus B2xx).

Run against one TinyB210 (UHD 4.7.0.0, internal "GPSTCXO v3.9" GPSDO) on
2026-10-26: GPSDO lock, PPS alignment, timed captures and timestamps all
work. Not yet run: two boards against one signal, which is the only test
that measures what TDOA actually needs. Run `python3 hw_selftest.py` on
every new board.

Times are carried as (full_secs, frac_secs), never as one float64. At a Unix
epoch of ~1.79e9 s a float64 is spaced 238 ns apart, so get_real_secs() hands
back a timestamp already rounded to ~70 m of range. The first hardware run
measured exactly that grid and reported it as "112 ns schedule jitter".
Same failure family as bug 10.

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

import collections
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


def split_secs(t):
    """A float of seconds as (int full, float frac). t - floor(t) is exact."""
    full = int(np.floor(t))
    return full, float(t) - full


def ts_split(ts):
    """A uhd TimeSpec as (int full, float frac), without going through a float."""
    return int(ts.get_full_secs()), float(ts.get_frac_secs())


def secs_between(a, b):
    """a - b in seconds, both (full, frac), exact to well under a picosecond:
    the integer parts cancel before anything small is added to them."""
    return float(a[0] - b[0]) + (a[1] - b[1])


class UhdSource:
    """A B210-class receiver disciplined to GPS, capable of timed captures.

    channels=[0] for single RX. channels=[0, 1] for the coherent pair, which
    share one LO and one ADC clock and are therefore usable as a two-element
    interferometer for bearing -- no inter-node timing required.
    """

    def __init__(self, fs, fc, gain_db=40, device="type=b200",
                 channels=(0,), require_gps=True, gps_timeout_s=180.0,
                 bandwidth_hz=None, gps_settle_s=90.0, max_host_skew_s=2.0):
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
        self.gps_settle_s = float(gps_settle_s)
        # None disables the host cross-check, for a node with no NTP at all
        self.max_host_skew_s = max_host_skew_s
        # what _discipline_to_gps saw, for the self-test and for health
        self.pps_settle = None

        if not self.gps_present and require_gps:
            raise UhdError(
                "No gps_locked sensor. This board has no GPSDO, so absolute "
                "timing is unavailable and TDOA will need an external PPS on "
                "the PPS input, or reference-emitter calibration. Pass "
                "require_gps=False to proceed without it."
            )

        # Rates BEFORE the clock is aligned. With master_clock_rate on
        # automatic, set_rx_rate() can change it, and a B2xx keeps its time
        # as a tick count: aligned at 16 MHz, then 2.4 Msps moved the master
        # clock to 38.4 MHz and every timestamp became t * 16/38.4 -- a node
        # reporting time_aligned with its clock 1.05e9 s (33 years) behind.
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

        if self.gps_present:
            self._discipline_to_gps(gps_timeout_s, require_gps)

        # PPS phase tracking. On the TinyB210 the GPSDO steers the
        # oscillator's frequency only approximately and nothing steers the
        # device clock's phase: set once on a PPS edge, it then drifts
        # against GPS time, measured 2026-10-28 at about -1 ns/s on four
        # boards (12-15 ns/min between boards, up to 2 us of TDOA offset
        # with uptime). Every PPS edge is timestamped by the device, so the drift is
        # observable: track it and take it out of every capture timestamp.
        self._pps = collections.deque(maxlen=60)    # (device second, phase ns)
        self._pps_lock = threading.Lock()
        self._pps_run = bool(self.gps_present and self.time_aligned)
        if self._pps_run:
            threading.Thread(target=self._pps_track, daemon=True).start()

        st = uhd.usrp.StreamArgs("fc32", "sc16")
        st.channels = self.channels
        self.streamer = self.usrp.get_rx_stream(st)
        self.md = uhd.types.RXMetadata()
        self._max_samps = self.streamer.get_max_num_samps()

        # Throw one capture away. On the TinyB210 the first timed capture
        # after the streamer is created lands +1875 ns from its command and
        # every later one +700 ns, constant to the tick. The constant cancels
        # between nodes; a one-off on whichever capture a node happens to
        # start with does not.
        if self.time_aligned:
            try:
                self.capture_at(self.device_time() + 0.1, 1000)
            except UhdError:
                pass
            # Re-check once everything is configured: anything after the
            # alignment that touches the master clock silently rescales time.
            g = self._sensor_int("gps_time")
            dev = int(self.usrp.get_time_now().get_full_secs())
            if g is not None and abs(dev - g) > 1:
                self.time_aligned = False
                msg = (f"device clock {dev} no longer matches gps_time {g} "
                       f"({dev - g:+d} s) after setup; something changed the "
                       f"master clock rate after alignment")
                if require_gps:
                    raise UhdError(msg)
                print(f"[uhd] WARNING {msg}")

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

        # ref_locked goes true long before the oscillator is on frequency:
        # measured on the TinyB210, the PPS interval read off the device
        # clock starts 625 ns/s out and takes ~40 s to reach zero. Captures
        # taken in that window carry a drifting clock.
        self.pps_settle = self._wait_pps_settled(self.gps_settle_s)
        if not self.pps_settle["settled"]:
            print(f"[uhd] WARNING PPS interval not settled after "
                  f"{self.gps_settle_s:.0f}s (last errors "
                  f"{[round(e * 1e9) for e in self.pps_settle['err_s'][-3:]]} ns)")

        target = self._gps_second()
        if target is None:
            msg = "gps_time never gave a consistent reading; clock not aligned"
            if require:
                raise UhdError(msg)
            print(f"[uhd] WARNING {msg}")
            return

        # step the device clock to the next whole UTC second on a PPS edge
        self.usrp.set_time_next_pps(uhd.types.TimeSpec(target + 1, 0.0))
        time.sleep(1.1)

        # Check the step took: the device must now agree with GPS to the
        # second. Reading it back is the only way to know the PPS edge, and
        # not some other second, latched the time.
        dev = ts_split(self.usrp.get_time_now())
        g = self._sensor_int("gps_time")
        if g is not None and abs(dev[0] - g) > 1:
            msg = (f"device clock {dev[0]} disagrees with gps_time {g} "
                   f"after alignment")
            if require:
                raise UhdError(msg)
            print(f"[uhd] WARNING {msg}")
            return

        # The PPS edge is right; its LABEL can be a whole second off. gps_time
        # is read at no fixed point relative to the edge, and this GPSDO's
        # NMEA does not arrive at a fixed point within the second either, so
        # a read near an edge can name the neighbouring second -- the check
        # above tolerates exactly that. Measured, two TinyB210s side by side:
        # one aligned -19 ms from the host, the other +980.8 ms, and every
        # scheduled capture on it was then half a second in the past. A
        # trusted host clock (NTP, ms-level) resolves the integer second:
        # a whole-second offset is relabelled on the next edge.
        if self.max_host_skew_s is not None:
            for _ in range(3):
                off = self._host_offset()
                k = int(round(off))
                if k == 0 or abs(off - k) > 0.25:
                    break                  # aligned, or not a labelling error
                last = self._next_pps_edge()
                self.usrp.set_time_next_pps(uhd.types.TimeSpec(last[0] + 1 - k, 0.0))
                time.sleep(1.1)
                print(f"[uhd] device clock was {k:+d} s off UTC after alignment "
                      f"(host offset {off*1e3:+.1f} ms); relabelled the PPS edge")
            off = self._host_offset()
            if abs(off) > 0.5:
                msg = (f"device clock {off*1e3:+.1f} ms from the host clock after "
                       f"alignment; refusing it")
                if require:
                    raise UhdError(msg)
                print(f"[uhd] WARNING {msg}")
                return
        self.time_aligned = True

    def _host_offset(self):
        """Device minus host time, seconds. A float is fine here: this only
        has to tell whole seconds apart, against an NTP clock good to ms."""
        return float(self.usrp.get_time_now().get_real_secs()) - time.time()

    def _next_pps_edge(self, timeout_s=3.0):
        """Wait for a PPS edge and return its device time (full, frac). Taken
        right after the edge, so a set_time_next_pps issued now applies to the
        NEXT edge, with almost a second to spare -- issued at an arbitrary
        moment it could race the edge and land one second later."""
        first = ts_split(self.usrp.get_time_last_pps(0))
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            e = ts_split(self.usrp.get_time_last_pps(0))
            if e != first:
                return e
            time.sleep(0.01)
        return first

    def _wait_pps_settled(self, timeout_s, n_good=5):
        """Wait until n_good consecutive PPS intervals, read off the device
        clock, are 1 s to within max(100 ns, 1.5 ticks)."""
        tick = 1.0 / float(self.usrp.get_master_clock_rate())
        tol = max(100e-9, 1.5 * tick)
        edges, errs = [], []
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            e = ts_split(self.usrp.get_time_last_pps(0))
            if not edges or e != edges[-1]:
                if edges:
                    errs.append(secs_between(e, edges[-1]) - 1.0)
                edges.append(e)
                tail = errs[-n_good:]
                if len(tail) == n_good and all(abs(x) <= tol for x in tail):
                    return {"settled": True, "err_s": errs, "tick_s": tick,
                            "waited_s": timeout_s - (t_end - time.time())}
            time.sleep(0.1)
        return {"settled": False, "err_s": errs, "tick_s": tick,
                "waited_s": timeout_s}

    def _gps_second(self, tries=10):
        """The current GPS second, or None if it cannot be trusted.

        Do not believe a single gps_time read. Right after power-up the
        TinyB210's GPSDO emits malformed NMEA, and a read taken then aligned
        the device clock to midnight four days earlier -- every timestamp
        off by 358597 s, and nothing raised. A reading is accepted only when
        a second read 1 s later has advanced by exactly one, and (unless
        max_host_skew_s is None) it agrees with the host's NTP clock.
        """
        prev, warned = None, False
        for _ in range(tries):
            g = self._sensor_int("gps_time")
            ok_host = (self.max_host_skew_s is None or g is None
                       or abs(g - time.time()) <= self.max_host_skew_s)
            if g is not None and prev is not None and g - prev == 1 and ok_host:
                return g
            if g is not None and not ok_host and not warned:
                print(f"[uhd] gps_time {g} is {g - time.time():+.0f} s from the "
                      f"host clock; re-reading (max_host_skew_s="
                      f"{self.max_host_skew_s})")
                warned = True
            prev = g
            time.sleep(1.0)
        return None

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
        """Device time as one float -- fine for scheduling margins and
        timeouts, 238 ns coarse, so never for a measured offset."""
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
        t_utc, since the device rounds to a sample boundary. As a float it is
        only good to 238 ns; anything measured from it must use
        meta["actual_t0_split"] with secs_between(), and
        meta["schedule_error_s"] is already computed that way.

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

        req = split_secs(t_utc)
        cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
        cmd.num_samps = n
        cmd.stream_now = False
        cmd.time_spec = uhd.types.TimeSpec(*req)

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
                    t0_actual = ts_split(self.md.time_spec)
                take = min(int(rx), n - got)
                buf[:, got:got + take] = chunk[:, :take]
                got += take

        if t0_actual is None:
            raise UhdError("no time_spec in rx metadata; timing unusable")

        # The hardware stamp is device time; GPS time = device - drift.
        pps_corr_ns = self.pps_phase_at(t0_actual[0]) if self._pps_run else 0.0
        full, frac = t0_actual[0], t0_actual[1] - pps_corr_ns * 1e-9
        while frac < 0.0:
            full, frac = full - 1, frac + 1.0
        while frac >= 1.0:
            full, frac = full + 1, frac - 1.0
        t0_actual = (full, frac)

        return buf, t0_actual[0] + t0_actual[1], {
            "pps_corr_ns": pps_corr_ns,
            "requested_t0": float(t_utc),
            "actual_t0": t0_actual[0] + t0_actual[1],
            "actual_t0_split": t0_actual,
            "schedule_error_s": secs_between(t0_actual, req),
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
                cmd.time_spec = uhd.types.TimeSpec(
                    *split_secs(self.device_time() + lead))

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
                    t0_actual = ts_split(self.md.time_spec)
                take = min(int(rx), n - got)
                buf[:, got:got + take] = chunk[:, :take]
                got += take

        if t0_actual is None:
            t0_actual = ts_split(self.usrp.get_time_now())

        return buf, t0_actual[0] + t0_actual[1], {
            "requested_t0": None,
            "actual_t0": t0_actual[0] + t0_actual[1],
            "actual_t0_split": t0_actual,
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
            "time_aligned": self.time_aligned,
            "fs_actual": self.fs_actual,
            "fc_actual": self.fc_actual,
            "n_sats": (self.gps_position() or {}).get("n_sats"),
            "pps_phase_ns": self.pps_phase_ns(),
            "pps_phase_fit_ns": (self.pps_phase_at(int(self.device_time()))
                                 if self._pps_run else None),
            "pps_drift_ns_per_s": self.pps_drift_ns_per_s() if self._pps_run else None,
        }

    def _pps_track(self):
        """Record every PPS edge as (whole second, phase in ns). The edge
        register holds the last edge until the next one, so polling at 5 Hz
        sees each edge once."""
        last = None
        while self._pps_run:
            try:
                e = ts_split(self.usrp.get_time_last_pps(0))
            except Exception:
                e = None
            if e is not None and e != last:
                last = e
                wrap = e[1] > 0.5
                with self._pps_lock:
                    self._pps.append((e[0] + (1 if wrap else 0),
                                      ((e[1] - 1.0) if wrap else e[1]) * 1e9))
            time.sleep(0.2)

    def pps_phase_at(self, t_full):
        """Device-clock error (ns) at device second t_full: a straight line
        through the last 60 PPS edges. One edge reads only to the tick (25 ns
        at 40 MHz); the line through 60 of them is well under that, and the
        drift is linear over a minute. Positive = device clock ahead of
        GPS. 0 with no edges yet (no GPSDO, or not aligned)."""
        with self._pps_lock:
            pts = list(self._pps)
        if len(pts) < 5:
            return pts[-1][1] if pts else 0.0
        sec = np.array([p[0] for p in pts], dtype=float)
        ph = np.array([p[1] for p in pts], dtype=float)
        slope, icept = np.polyfit(sec - sec[-1], ph, 1)
        return float(icept + slope * (t_full - sec[-1]))

    def pps_drift_ns_per_s(self):
        with self._pps_lock:
            pts = list(self._pps)
        if len(pts) < 5:
            return None
        sec = np.array([p[0] for p in pts], dtype=float)
        return float(np.polyfit(sec - sec[-1], [p[1] for p in pts], 1)[0])

    def pps_phase_ns(self):
        """Device time at the last PPS edge, minus the nearest whole second.

        The clock is set on a PPS edge once, at alignment. If the GPSDO only
        steers the oscillator's frequency and not its phase, the device clock
        wanders away from GPS time afterwards, and this number shows by how
        much (resolution one tick, 25 ns at 40 MHz). Stays near 0 when the
        clock is phase-locked."""
        try:
            frac = ts_split(self.usrp.get_time_last_pps(0))[1]
        except Exception:
            return None
        return (frac - 1.0 if frac > 0.5 else frac) * 1e9

    def close(self):
        self._pps_run = False
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
