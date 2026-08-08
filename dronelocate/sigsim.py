"""Emitter and channel simulation for the emulated nodes.

Each emulated node runs in its own process and synthesises its own IQ. They
agree on the emitted waveform without exchanging it: the burst id seeds the
RNG, so every node independently generates a bit-identical master waveform.
What differs per node -- and what the solver has to recover -- is the
geometric delay, the clock error, the path loss and the noise.

This matters for the demo's honesty. The supernode is not handed the answer
in any form. It sees only IQ, and has to correlate its way to a position.
"""

import numpy as np

from .geo import C_LIGHT


# CP-OFDM geometry for the synthetic emitter. 128 useful samples at the
# demo's 2.4 Msps is 18.75 kHz subcarrier spacing (LTE-adjacent, which is
# what OcuSync is); CP of 1/8 matches the same family. The CP length is what
# the cyclostationary detector recovers, so keep these importable.
OFDM_N_USEFUL = 128
OFDM_N_CP = 16
_PILOT_SEED = 0x0FD3          # pilots identical across bursts and symbols


def _qpsk(rng, shape):
    return ((rng.integers(0, 2, shape) * 2 - 1)
            + 1j * (rng.integers(0, 2, shape) * 2 - 1)) / np.sqrt(2.0)


def _ofdm_burst(rng, n_samples, fs, bandwidth_hz):
    """CP-OFDM burst: QPSK data, fixed pilots every 8th carrier, DC null.

    Deterministic from rng (which the caller seeds from the burst id), so
    every node still synthesises a bit-identical waveform without exchanging
    it. The pilots reuse a fixed seed on top of that: constant pilots give
    the spectrum its lines and the cyclostationary detector something
    honest to find, exactly like a real downlink.
    """
    n_u, n_cp = OFDM_N_USEFUL, OFDM_N_CP
    n_slen = n_u + n_cp
    n_sym = int(np.ceil(n_samples / n_slen))

    n_active = int(np.clip(round(bandwidth_hz / fs * n_u / 2) * 2, 8, n_u - 4))
    idx = np.r_[1: n_active // 2 + 1, n_u - n_active // 2: n_u]
    pilots = idx[::8]

    x = np.zeros((n_sym, n_u), dtype=np.complex128)
    x[:, idx] = _qpsk(rng, (n_sym, len(idx)))
    # Pilots: fixed positions and magnitudes, but BPSK-scrambled per symbol.
    # The scrambling is not decoration. Repeating an identical pilot vector on
    # every symbol makes the waveform periodic at the symbol rate, which puts
    # autocorrelation sidelobes at multiples of N_u+N_cp at ~17% of the peak.
    # Harmless at 2.4 Msps (144 samples = 60 us, outside the search window)
    # and actively wrong at 20 Msps (7.2 us, well inside it) -- a wideband
    # capture then locks onto a symbol-period sidelobe and returns a
    # confident lag off by an exact multiple of the symbol period. Real OFDM
    # scrambles pilots for the same reason. Power per subcarrier is
    # unchanged, so the PSD keeps its pilot lines for GCC weighting, and the
    # cyclic prefix is untouched, so the CP detector is unaffected.
    pil = _qpsk(np.random.default_rng(_PILOT_SEED), len(pilots))
    scr = np.random.default_rng(_PILOT_SEED + 1).integers(0, 2, n_sym) * 2 - 1
    x[:, pilots] = pil[None, :] * scr[:, None]

    td = np.fft.ifft(x, axis=1)
    td = np.concatenate([td[:, -n_cp:], td], axis=1).ravel()[:n_samples]
    td /= np.sqrt(np.mean(np.abs(td) ** 2)) + 1e-20
    return td.astype(np.complex64)


def master_waveform(burst_id, n_samples, fs, bandwidth_hz, kind="noise"):
    """Emitted waveform, reproducible from burst_id.

    "noise": band-limited Gaussian -- near-ideal thumbtack autocorrelation,
    so correlation quality tracks SNR rather than waveform structure. The
    right choice for isolating solver behaviour, and structurally blind for
    anything that exploits signal structure (GCC weighting, cyclostationary
    detection) -- against a flat spectrum those reduce to no-ops.

    "ofdm": the CP-OFDM burst above, OcuSync-shaped. Correlates slightly
    worse (the CP puts autocorrelation sidelobes at +-OFDM_N_USEFUL samples
    = 53 us here, outside the demo's ~30 us search window but inside a
    clock-widened one -- the quality gate is what catches that), and gives
    detector/estimator structure to work with. The demo config uses this.
    """
    rng = np.random.default_rng(0xD10E + int(burst_id))
    if kind == "ofdm":
        return _ofdm_burst(rng, n_samples, fs, bandwidth_hz)
    n_fft = int(1 << int(np.ceil(np.log2(n_samples))))
    spec = rng.standard_normal(n_fft) + 1j * rng.standard_normal(n_fft)
    freqs = np.fft.fftfreq(n_fft, 1.0 / fs)
    spec[np.abs(freqs) > bandwidth_hz / 2.0] = 0.0
    w = np.fft.ifft(spec)[:n_samples]
    w /= np.sqrt(np.mean(np.abs(w) ** 2)) + 1e-20
    return w.astype(np.complex64)


def apply_fractional_delay(x, delay_samples):
    """Delay by an arbitrary (fractional) number of samples via phase ramp."""
    n = len(x)
    n_fft = int(1 << int(np.ceil(np.log2(n * 2))))
    spec = np.fft.fft(x, n_fft)
    freqs = np.fft.fftfreq(n_fft)
    spec *= np.exp(-2j * np.pi * freqs * delay_samples)
    return np.fft.ifft(spec)[:n].astype(np.complex64)


class NodeChannel:
    """Renders what one node's ADC would see for a given emitter position."""

    DISCIPLINES = ("gpsdo", "holdover", "free")

    def __init__(self, node_enu, fs, bandwidth_hz, clock_bias_s=0.0,
                 clock_drift_ppm=0.0, noise_figure_db=6.0, tx_dbm=20.0,
                 rng_seed=None, discipline="gpsdo", pps_jitter_s=15e-9,
                 discipline_tau_s=100.0, model_carrier=True,
                 waveform="noise", interference=None, hopping=None,
                 multipath=None):
        self.node_enu = np.asarray(node_enu, dtype=float)
        self.fs = float(fs)
        self.bandwidth_hz = float(bandwidth_hz)
        self.clock_bias_s = float(clock_bias_s)
        self.clock_drift_ppm = float(clock_drift_ppm)
        self.noise_figure_db = float(noise_figure_db)
        self.tx_dbm = float(tx_dbm)
        self.rng = np.random.default_rng(rng_seed)
        self._t_ref = None

        self.discipline = str(discipline).lower()
        self.pps_jitter_s = float(pps_jitter_s)
        self.discipline_tau_s = float(discipline_tau_s)
        self.model_carrier = bool(model_carrier)
        self.waveform = str(waveform)

        # Frequency hopping. A real drone datalink does not sit still: it
        # hops across the band on a schedule the receiver does not know.
        # The emitter's hop is derived from the burst id, exactly like the
        # waveform, so every node agrees on where the energy went without
        # exchanging anything -- but the NODE still has to find it, because
        # nothing tells the receiver which channel was used.
        hcfg = dict(hopping or {})
        self.hopping = bool(hcfg.get("enabled", False))
        self.hop_span_hz = float(hcfg.get("span_hz", 60e6))
        self.hop_channels = int(hcfg.get("channels", 16))
        self.hop_seed = int(hcfg.get("seed", 0xB0B))

        # Multipath. Reflectors are buildings and ground -- fixed geometry --
        # so each node's echo profile is drawn ONCE at construction and then
        # reused, not redrawn per burst. That distinction is the whole point:
        # a per-burst redraw would average away over a run, while a static
        # profile biases every measurement from that node in the same
        # direction, which is what a real site does and what a covariance
        # scaled by residual spread cannot see.
        mcfg = dict(multipath or {})
        self.multipath = bool(mcfg.get("enabled", False))
        self._echo_delays_s = np.zeros(0)
        self._echo_gains = np.zeros(0, dtype=complex)
        if self.multipath:
            mp_rng = np.random.default_rng((rng_seed or 0) + 5501)
            n_taps = int(mcfg.get("taps", 3))
            # Exponential power-delay profile: excess delay is the extra path
            # length over line-of-sight, so 100 ns is 30 m of detour.
            tau_rms = float(mcfg.get("tau_rms_s", 200e-9))
            k_db = float(mcfg.get("rician_k_db", 8.0))
            self._echo_delays_s = mp_rng.exponential(tau_rms, n_taps)
            # Total echo power sits K dB below the line-of-sight component.
            total = 10.0 ** (-k_db / 20.0)
            w = mp_rng.dirichlet(np.ones(n_taps))
            self._echo_gains = (total * np.sqrt(w)
                                * np.exp(2j * np.pi * mp_rng.random(n_taps)))

        # A local interferer: this node's own Wi-Fi neighbour, not a common
        # illuminator. 2.4 GHz is the most crowded band there is, and the
        # distinction matters -- an interferer present at ONE node has no
        # cross-channel coherence, which is what GCC weighting exploits to
        # delete it. Centre drawn per node, so no two nodes share a band.
        icfg = dict(interference or {})
        self.interference = bool(icfg.get("enabled", False))
        self.inr_db = float(icfg.get("inr_db", 20.0))
        self.int_width_hz = float(icfg.get("width_hz", 500e3))
        span = max(self.bandwidth_hz * 0.35, 1.0)
        self._int_centre_hz = float(np.random.default_rng(
            (rng_seed or 0) + 977).uniform(-span, span))
        # Receiver LO phase at lock is arbitrary. Drawn from a throwaway
        # generator so enabling the carrier model does not shift self.rng's
        # stream and change every noise draw after it.
        self._lo_phase = float(np.random.default_rng(
            rng_seed if rng_seed is not None else 0).uniform(0.0, 2.0 * np.pi))
        self._disc_e = 0.0       # OU state: the *jitter* about the bias
        self._last_t = None
        self._cur_err = self.clock_bias_s   # last value returned, for holdover
        self._hold_t0 = None
        self._hold_e0 = self.clock_bias_s

    def set_discipline(self, mode, t_s=None):
        """Switch clock regime at runtime.

        Entering holdover freezes wherever the disciplined error currently is
        and free-runs from there, which is what losing sky view actually does
        -- the oscillator does not jump, it simply stops being corrected.
        """
        mode = str(mode).lower()
        if mode not in self.DISCIPLINES:
            raise ValueError(f"unknown clock discipline {mode!r}")
        if mode == "holdover" and self.discipline != "holdover":
            self._hold_t0 = float(t_s) if t_s is not None else self._last_t
            self._hold_e0 = self._cur_err
        self.discipline = mode

    def clock_error_at(self, t_s):
        """Total clock error under the node's current discipline.

        gpsdo    -- the realistic case for hardware with a GPSDO. A
                    disciplined oscillator is a control loop, so its error is
                    *bounded* noise about UTC, not a ramp. Modelled as an
                    Ornstein-Uhlenbeck process: mean-reverting to zero with
                    standard deviation pps_jitter_s and correlation time
                    discipline_tau_s. This is the regime hw_selftest.py
                    measures on a real board.
        holdover -- GPS lock lost. Free-runs from the error it held at the
                    moment of loss, at the oscillator's own drift rate.
        free     -- never disciplined at all. Unbounded linear ramp.

        Drift must accumulate from a session epoch, not from the Unix epoch.
        Multiplying ppm by 1.8e9 seconds gives tens of seconds of "clock
        error", which is not a subtle bug -- it silently turns a 10 ms
        capture into an 80-million-sample allocation.
        """
        t = float(t_s)
        if self._t_ref is None:
            self._t_ref = t
            self._last_t = t

        if self.discipline == "gpsdo":
            dt = max(t - self._last_t, 0.0)
            a = (np.exp(-dt / self.discipline_tau_s)
                 if self.discipline_tau_s > 0 else 0.0)
            self._disc_e = (a * self._disc_e
                            + np.sqrt(max(1.0 - a * a, 0.0))
                            * self.pps_jitter_s * self.rng.standard_normal())
            # Bias survives discipline. GPS corrects phase and rate against
            # UTC, but a per-node static offset -- antenna cable delay,
            # receiver group delay -- is a calibration constant no amount of
            # locking removes. A bias common to every node cancels in the
            # TDOA difference; the per-node part does not, which is exactly
            # what reference-emitter calibration exists to measure.
            e = self.clock_bias_s + float(self._disc_e)
        elif self.discipline == "holdover":
            t0 = self._hold_t0 if self._hold_t0 is not None else self._t_ref
            e = self._hold_e0 + self.clock_drift_ppm * 1e-6 * (t - t0)
        else:  # free
            e = self.clock_bias_s + self.clock_drift_ppm * 1e-6 * (t - self._t_ref)

        self._last_t = t
        self._cur_err = e
        return e

    def clock_rate_error(self):
        """Fractional frequency error of the node's oscillator, dimensionless.

        The LO is synthesized from the same reference as the sample clock, so
        a clock running fast by e also downconverts with an LO high by fc*e.
        gpsdo disciplines rate as well as phase, so ~0 there; holdover and
        free run at the crystal's own drift rate.
        """
        if self.discipline == "gpsdo":
            return 0.0
        return self.clock_drift_ppm * 1e-6

    def hop_offset_hz(self, burst_id):
        """Where this burst actually transmitted, relative to the band centre.

        Deterministic from the burst id so every node renders the same hop
        without communicating -- the same trick as the waveform. The
        receiver is told nothing: finding the energy is the node's job.
        """
        if not self.hopping:
            return 0.0
        rng = np.random.default_rng(self.hop_seed + int(burst_id))
        ch = int(rng.integers(0, self.hop_channels))
        step = self.hop_span_hz / self.hop_channels
        return (ch - (self.hop_channels - 1) / 2.0) * step

    def _interferer(self, n_samples):
        """Band-limited noise blob at this node's own centre frequency."""
        spec = (self.rng.standard_normal(n_samples)
                + 1j * self.rng.standard_normal(n_samples))
        f = np.fft.fftfreq(n_samples, 1.0 / self.fs)
        spec[np.abs(f - self._int_centre_hz) > self.int_width_hz / 2.0] = 0.0
        x = np.fft.ifft(spec)
        x /= np.sqrt(np.mean(np.abs(x) ** 2)) + 1e-20
        return x * 10.0 ** (self.inr_db / 20.0)

    def render(self, emitter_enu, burst_id, n_samples, t_epoch_s,
               capture_start_s, fc_hz=2.437e9, snr_db_at_1km=30.0,
               emitter_vel=None):
        """Return complex64 IQ for one capture window.

        capture_start_s is this node's *intended* window start on the common
        timebase. True arrival lands wherever geometry and clock error put it.
        """
        emitter_enu = np.asarray(emitter_enu, dtype=float)
        r = float(np.linalg.norm(emitter_enu - self.node_enu))
        r = max(r, 1.0)

        tof = r / C_LIGHT
        # Evaluate the clock ONCE per capture and reuse it. Under a
        # disciplined clock this function is stochastic and stateful -- each
        # call advances the loop and draws new jitter -- so calling it again
        # to fill in the metadata reports an error the signal never had. The
        # supernode then subtracts the wrong number, and the difference is
        # pure uncorrectable measurement error that no amount of solver work
        # can recover. It cost ~21 ns here, which swamped everything else.
        clk = self.clock_error_at(t_epoch_s)
        # Form the delay RELATIVE to the capture start before adding anything
        # small. t_epoch_s is an absolute Unix time near 1.785e9, where the
        # float64 spacing is 238 ns -- so building the absolute arrival
        # instant first rounds tof and clk onto a 238 ns grid and discards
        # precisely the precision TDOA lives on (~70 ns RMS per node, ~100 ns
        # between a pair, which is 30 m of range). The two epochs are large
        # but close, so their difference is exact; everything after it is
        # small. Same failure family as anchoring drift to the Unix epoch.
        delay_samples = ((t_epoch_s - capture_start_s) + tof + clk) * self.fs

        # free-space amplitude falls as 1/r; reference SNR quoted at 1 km
        snr_db = snr_db_at_1km - 20.0 * np.log10(r / 1000.0) - (self.noise_figure_db - 6.0)
        amp = 10.0 ** (snr_db / 20.0)

        # Guard: a delay far outside the capture window means the burst is
        # simply not in this window. Return noise rather than allocating a
        # buffer proportional to the error.
        if abs(delay_samples) > n_samples * 4:
            noise = (self.rng.standard_normal(n_samples)
                     + 1j * self.rng.standard_normal(n_samples)) / np.sqrt(2.0)
            return noise.astype(np.complex64), {
                "true_range_m": r, "true_tof_s": tof,
                "clock_error_s": clk,
                "snr_db": -99.0, "missed_window": True,
                "doppler_hz": 0.0,
            }

        # Multipath echoes arrive LATER than line-of-sight, never earlier, so
        # the pad has to cover the longest excess delay or a late tap would
        # be silently truncated -- which would understate exactly the effect
        # under test.
        extra = (float(np.max(self._echo_delays_s)) * self.fs
                 if len(self._echo_delays_s) else 0.0)
        pad = int(abs(delay_samples) + extra) + 64
        w = master_waveform(burst_id, n_samples + 2 * pad, self.fs,
                            self.bandwidth_hz, kind=self.waveform)
        los = apply_fractional_delay(w, delay_samples + pad)
        sig = los[pad: pad + n_samples] * amp

        # Each echo is the same waveform, delayed further and scaled by a
        # complex gain. The gain's phase matters: echoes add coherently with
        # the direct path, so a tap within a fraction of a symbol can pull
        # the composite correlation peak either way, while a well-separated
        # one shows up as a distinct later peak.
        for d_s, g in zip(self._echo_delays_s, self._echo_gains):
            e = apply_fractional_delay(w, delay_samples + pad + d_s * self.fs)
            sig = sig + e[pad: pad + n_samples] * amp * g

        # The carrier. Everything above is baseband, but a real node
        # downconverts from fc, and two things put a frequency offset on what
        # its ADC sees: emitter motion (Doppler, -fc*tau_dot) and its own LO
        # rate error (-fc*e_rate, since the LO comes off the same disciplined
        # reference as the sample clock). Neither moves the envelope
        # measurably -- over 10 ms at 31 m/s the delay drifts ~1 ns and the
        # sample clock skews ~0.05 samples -- but the carrier turns 2-4 full
        # cycles across a window, and cross-correlating two nodes without
        # searching Doppler attenuates the peak by sinc(df*T): past the first
        # null at realistic drone speeds. fc/fs ~ 1000 is why the carrier
        # term matters and the envelope terms do not. A baseband-only sim
        # cannot show this failure, which is exactly the bug-10 lesson again.
        doppler_hz = 0.0
        if self.model_carrier:
            tau_dot = 0.0
            if emitter_vel is not None:
                v = np.asarray(emitter_vel, dtype=float)
                tau_dot = float((emitter_enu - self.node_enu) @ v) / (r * C_LIGHT)
            doppler_hz = -fc_hz * (tau_dot + self.clock_rate_error())
            ph0 = -2.0 * np.pi * fc_hz * (tof + clk) + self._lo_phase
            t_rel = np.arange(n_samples, dtype=np.float64) / self.fs
            sig = sig * np.exp(1j * (2.0 * np.pi * doppler_hz * t_rel + ph0))

        # The hop. The emitter transmits at fc + hop, so in this node's
        # baseband the burst sits at `hop`, not at DC. Energy shifted past
        # Nyquist never reaches the ADC -- the analog front end filters it --
        # so it is discarded rather than aliased back in. A hop that lands
        # wholly outside the digitized band therefore leaves the node with
        # nothing but noise, which is the entire point: a missed detection
        # is unrecoverable, unlike a degraded one.
        hop_hz = self.hop_offset_hz(burst_id)
        in_band = 1.0
        if hop_hz != 0.0:
            spec = np.fft.fft(sig)
            k = int(np.rint(hop_hz * n_samples / self.fs))
            if abs(k) >= n_samples:
                spec[:] = 0.0
            else:
                spec = np.roll(spec, k)
                if k > 0:
                    spec[:k] = 0.0          # wrapped past +Nyquist
                elif k < 0:
                    spec[k:] = 0.0          # wrapped past -Nyquist
            before = float(np.sum(np.abs(np.fft.fft(sig)) ** 2)) + 1e-30
            after = float(np.sum(np.abs(spec) ** 2))
            in_band = after / before
            sig = np.fft.ifft(spec).astype(np.complex64)

        # Gate the burst into the middle of the window. A real capture is
        # mostly noise with a transmission somewhere inside it, and the
        # detector needs that noise floor to measure SNR against. It also
        # keeps the correlation honest: the lag has to be found, not assumed.
        gate = np.zeros(n_samples, dtype=np.float32)
        a0, a1 = int(n_samples * 0.30), int(n_samples * 0.70)
        gate[a0:a1] = 1.0
        edge = max(1, (a1 - a0) // 20)
        ramp = 0.5 * (1.0 - np.cos(np.linspace(0, np.pi, edge)))
        gate[a0:a0 + edge] = ramp
        gate[a1 - edge:a1] = ramp[::-1]
        sig = sig * gate

        noise = (self.rng.standard_normal(n_samples)
                 + 1j * self.rng.standard_normal(n_samples)) / np.sqrt(2.0)
        if self.interference:
            noise = noise + self._interferer(n_samples)
        return (sig + noise).astype(np.complex64), {
            "true_range_m": r,
            "true_tof_s": tof,
            "clock_error_s": clk,
            # Reported after the hop truncation, not before: a burst that
            # landed outside the digitized band did not arrive at 13 dB, it
            # did not arrive at all.
            "snr_db": float(snr_db + 10.0 * np.log10(max(in_band, 1e-12))),
            # Diagnostic only. The supernode must measure Doppler itself via
            # the CAF -- this is not clk_off_ns, it has no calibration stand-in
            # role and nothing on the solve path may read it.
            "doppler_hz": float(doppler_hz),
            # Also diagnostic. A receiver is never told the hop; scoring how
            # often the node found it is the whole experiment, so reading
            # this anywhere but a validator would be cheating.
            "hop_hz": float(hop_hz),
            "hop_in_band": float(in_band),
        }


def quantize(iq, fmt):
    """Pack complex64 to the wire format.

    ci8 mirrors what a HackRF actually delivers. ci2 is the VLBI trick: two
    bits per component costs about 0.55 dB of correlation SNR and cuts the
    payload fourfold, which is the cheapest latency win available when the
    uplink is the bottleneck.
    """
    if fmt == "cf32":
        return iq.astype(np.complex64).tobytes()

    scale = 3.0 * (np.sqrt(np.mean(np.abs(iq) ** 2)) / np.sqrt(2.0) + 1e-20)
    i = np.clip(np.real(iq) / scale * 127.0, -127, 127)
    q = np.clip(np.imag(iq) / scale * 127.0, -127, 127)

    if fmt == "ci8":
        inter = np.empty(len(iq) * 2, dtype=np.int8)
        inter[0::2] = np.round(i).astype(np.int8)
        inter[1::2] = np.round(q).astype(np.int8)
        return inter.tobytes()

    if fmt == "ci2":
        lv = np.array([-96, -32, 32, 96], dtype=np.int8)
        edges = np.array([-64.0, 0.0, 64.0])
        ii = np.digitize(i, edges).astype(np.uint8)
        qq = np.digitize(q, edges).astype(np.uint8)
        packed_vals = (ii << 2) | qq            # 4 bits per complex sample
        n = len(packed_vals)
        if n % 2:
            packed_vals = np.append(packed_vals, 0)
        out = (packed_vals[0::2] << 4) | packed_vals[1::2]
        return out.astype(np.uint8).tobytes()

    raise ValueError(f"unknown format {fmt}")


def dequantize(buf, fmt, n_samples=None):
    if fmt == "cf32":
        return np.frombuffer(buf, dtype=np.complex64)

    if fmt == "ci8":
        a = np.frombuffer(buf, dtype=np.int8).astype(np.float32)
        return (a[0::2] + 1j * a[1::2]).astype(np.complex64)

    if fmt == "ci2":
        lv = np.array([-96.0, -32.0, 32.0, 96.0], dtype=np.float32)
        raw = np.frombuffer(buf, dtype=np.uint8)
        nib = np.empty(len(raw) * 2, dtype=np.uint8)
        nib[0::2] = raw >> 4
        nib[1::2] = raw & 0x0F
        if n_samples is not None:
            nib = nib[:n_samples]
        i = lv[(nib >> 2) & 0x03]
        q = lv[nib & 0x03]
        return (i + 1j * q).astype(np.complex64)

    raise ValueError(f"unknown format {fmt}")


def bytes_per_sample(fmt):
    return {"cf32": 8.0, "ci8": 2.0, "ci2": 0.5}[fmt]
