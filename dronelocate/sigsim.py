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


def master_waveform(burst_id, n_samples, fs, bandwidth_hz):
    """Band-limited complex noise burst, reproducible from burst_id.

    Filtered Gaussian noise is deliberate: it has a near-ideal thumbtack
    autocorrelation, so correlation quality tracks SNR rather than waveform
    structure. Real OcuSync is OFDM and correlates slightly worse; swap this
    for a real capture when you have one.
    """
    rng = np.random.default_rng(0xD10E + int(burst_id))
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
                 discipline_tau_s=100.0):
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

    def render(self, emitter_enu, burst_id, n_samples, t_epoch_s,
               capture_start_s, fc_hz=2.437e9, snr_db_at_1km=30.0):
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
            }

        pad = int(abs(delay_samples)) + 64
        w = master_waveform(burst_id, n_samples + 2 * pad, self.fs, self.bandwidth_hz)
        w = apply_fractional_delay(w, delay_samples + pad)
        sig = w[pad: pad + n_samples] * amp

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
        return (sig + noise).astype(np.complex64), {
            "true_range_m": r,
            "true_tof_s": tof,
            "clock_error_s": clk,
            "snr_db": float(snr_db),
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
