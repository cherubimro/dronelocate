"""Find a frequency-hopping burst in a wide capture and bring it to baseband.

A drone datalink hops. Nothing tells the receiver where, so a node that
digitizes one narrow window sees the burst only when the hop happens to land
in it -- and a missed detection is unrecoverable in a way a degraded one is
not. The answer is to digitize the whole hop span and *find* the energy.

Two steps, both cheap:

  detect_channel()  which sub-band holds the burst, by comparing each
                    channel's in-burst power against its own noise floor.
  downconvert()     shift that channel to DC, low-pass, decimate -- after
                    which every existing stage (CAF, GCC weighting, sinc
                    interpolation, the solver) runs unchanged on a narrowband
                    stream at the reduced rate.

The nodes are not coordinated: each finds the channel independently, and
they agree because they all heard the same emission. Where they disagree,
one of them is wrong and the supernode must not correlate across the
disagreement -- see `channel_consensus`.
"""

import numpy as np


def channel_grid(fs, n_channels):
    """Centre frequency of each channel, in Hz relative to baseband DC."""
    step = fs / n_channels
    return (np.arange(n_channels) - (n_channels - 1) / 2.0) * step


def detect_channel(iq, fs, n_channels, n_blocks=32):
    """Index of the channel holding the burst, plus a confidence ratio.

    Channelise by FFT and integrate power per channel, but score each
    channel against its OWN temporal noise floor rather than against the
    other channels. That distinction matters: a strong narrowband
    interferer parked in one channel beats the drone on absolute power and
    would win a plain argmax, while it loses on burstiness because it is
    always on. This is the same reasoning as the block-power detector --
    median over time, not mean, and peak-versus-floor rather than
    peak-versus-neighbour.

    Returns (channel_index, ratio, per_channel_ratio). The ratio is the
    burst-to-floor power ratio of the winning channel; a value near 1 means
    nothing stood out and the caller should treat the burst as undetected.
    """
    iq = np.asarray(iq, dtype=np.complex64)
    n = len(iq)
    blk = n // n_blocks
    if blk < n_channels * 2:
        n_blocks = max(2, n // (n_channels * 2))
        blk = n // n_blocks
    usable = blk * n_blocks

    # (n_blocks, blk) -> per-block spectrum -> fold into channel bins
    spec = np.fft.fftshift(
        np.fft.fft(iq[:usable].reshape(n_blocks, blk), axis=1), axes=1)
    pw = np.abs(spec) ** 2
    edges = np.linspace(0, blk, n_channels + 1).astype(int)
    per_block = np.stack([pw[:, edges[c]:edges[c + 1]].sum(axis=1)
                          for c in range(n_channels)], axis=1)   # (blocks, ch)

    peak = per_block.max(axis=0)
    floor = np.median(per_block, axis=0)
    ratio = peak / np.clip(floor, 1e-30, None)
    ch = int(np.argmax(ratio))
    return ch, float(ratio[ch]), ratio


def downconvert(iq, fs, offset_hz, decim):
    """Shift `offset_hz` to DC, low-pass to the decimated band, decimate.

    Done in the frequency domain: the shift is a circular roll of the
    spectrum, and the low-pass is truncation to the surviving bins, so the
    filter is brick-wall and costs one FFT pair. Anything outside the
    retained band is discarded rather than folded back, which is what an
    analog front end plus a decimating filter would do.

    Returns (iq_out, fs_out). fs_out is what the correlator must use to turn
    lag indices into seconds -- the project's rule about adopting the rate
    the hardware actually delivered applies here too, because a decimator
    that quietly disagreed with the solver would be a multiplicative bias
    on every TDOA.
    """
    iq = np.asarray(iq, dtype=np.complex64)
    n = len(iq)
    decim = max(1, int(decim))
    keep = n // decim
    if keep < 8:
        raise ValueError("decimation leaves too few samples")

    spec = np.fft.fft(iq)
    k = int(np.rint(-float(offset_hz) * n / fs))       # bring offset -> DC
    spec = np.roll(spec, k)

    half = keep // 2
    out = np.empty(keep, dtype=complex)
    out[:half] = spec[:half]
    out[half:] = spec[n - (keep - half):]
    # 1/decim keeps amplitude, not power, consistent across the transform
    out = np.fft.ifft(out) * (keep / n) * decim
    return out.astype(np.complex64), fs / decim


def channel_consensus(channels, min_agree=3):
    """Majority channel among nodes, and who disagreed.

    Nodes that picked a different channel did not hear the same signal, so
    correlating them against the majority produces a confident wrong lag --
    the same failure class as a lost correlation peak, and equally invisible
    to a per-measurement sigma. Drop them rather than weight them.
    """
    if not channels:
        return None, []
    vals, counts = np.unique(np.array(list(channels.values())), return_counts=True)
    best = int(vals[int(np.argmax(counts))])
    if int(counts.max()) < min_agree:
        return None, list(channels)
    return best, [k for k, v in channels.items() if int(v) != best]
