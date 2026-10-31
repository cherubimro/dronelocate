# Splitter calibration — measuring `cal_phase_rad`

> **TinyB210: read this first.** In the one bearing test so far (2026-10-26/27,
> board 1971XX, two antennas 6 cm apart on a desk) both RX channels
> responded to both antennas: removing either antenna lowered *both*
> channels by 3-6 dB, and a phone's signal arrived at both with coherence
> 0.999 and a phase difference that did not change with direction. **Why is
> not established.** The B210 design has two independent receive chains,
> and this board's receiver noise *is* independent between channels, so do
> not read this as the board having one RX input; mutual coupling between
> antennas half a wavelength apart, leakage along the u.FL cables and a
> strong near source are all candidates. Until it is repeated with the
> antennas isolated from each other, two-antenna bearing -- and so this
> calibration -- is unverified on the TinyB210; see
> docs/tinyb210_bench_2026-10-26.txt Part 3. Separately, on USB 3 the
> board's own link radiates into both receivers (noise coherence 0.5
> between channels; 0.01-0.08 on USB 2).

Every bearing this system produces is offset by an unknown constant until
this is done. `hardware.cal_phase_rad` is `0.0` in every shipped config,
which is not a calibrated value — it is a placeholder.

**An afternoon of work, and it blocks the whole bearing-fusion plan**
(`TODO.md` §5e). Relative comparisons work uncalibrated; absolute direction
does not.

---

## What is actually being measured

`phase_bearing()` turns the phase difference between RX0 and RX1 into an
angle. That difference has two parts:

```
measured_phase  =  direction_of_arrival_phase  +  instrumental_offset
```

The instrumental part comes from unequal cable lengths, board routing to the
two RX ports, and the state of the LO divider. It has nothing to do with
where the target is, and it does not average away.

Feed **one source into both ports through a splitter with equal cables** and
the direction term is zero by construction. Whatever remains is the
instrumental offset. That number is `cal_phase_rad`.

---

## Read this before wiring anything

**1. TX power will destroy an RX front end.** A B210 can transmit around
+10 dBm; its RX damage threshold is far below that. If you use the board's
own TX as the source, put **at least 30 dB** of attenuation in the path and
check it with a power meter before connecting. If you are not certain, use
an external generator at −40 dBm or lower, or an antenna picking up ambient
Wi-Fi.

**2. Cable length mismatch goes straight into the answer.** At 2.437 GHz,
λ = 123.02 mm in air, so λ/2 = 61.51 mm — which is why
`array_baseline_m` is 0.061. Inside coax it is shorter, and a millimetre of
mismatch is not negligible:

| cable | velocity factor | phase error per 1 mm mismatch |
|---|---|---|
| RG-316 / PTFE | 0.66 | **4.43°** |
| foam dielectric | 0.70 | 4.18° |
| semi-rigid | 0.85 | 3.44° |

Use two cables of the same type, same batch, cut or bought to the same
length. The swap step below cancels what remains.

The spacing is for the **highest** frequency the array will see: λ/2 is
28.9 mm at 5.18 GHz and **25.8 mm at 5.8 GHz**. One pair covering both
bands must sit at ≤ 25 mm, which at 2.4 GHz is ~λ/5 -- still unambiguous,
but ~2.4× less precise and more strongly coupled. `array_baseline_m` is
the physical spacing and does not change on retune; `cal_phase_rad` does
(see the retune hazard below), so calibrate per band.

**3. Phase error maps to bearing error, worst near endfire.** With
`d = λ/2`:

| angle off broadside | bearing error per 1° of phase error |
|---|---|
| 0° | 0.32° |
| 30° | 0.37° |
| 45° | 0.45° |
| 60° | 0.64° |
| 75° | **1.23°** |

So a 5° calibration error is under 2° of bearing near broadside and nearly
6° at 75°. Aim for a residual spread of a couple of degrees.

---

## Equipment

- 2-way power splitter/divider covering 2.4 GHz — resistive or Wilkinson.
  It does **not** need to be phase-matched; the swap step removes its
  imbalance along with the cables'.
- Two SMA cables, same type and length.
- Attenuators, if the source is the board's own TX (≥30 dB).
- A source: an external generator on CW at the operating `fc`, the board's
  own TX, or an antenna on ambient Wi-Fi. Ambient works because what matters
  is that **both ports see the same signal**, not that it is clean.

---

## Which port is which

The code never selects an antenna port, so UHD's B210 default applies:
**channel 0 = `RX2` on side A, channel 1 = `RX2` on side B** (read back with
`get_rx_antenna()` on a TinyB210: `RX2`, subdevs `A:A A:B`). An earlier
version of this page said RX0 = `TX/RX`, RX1 = `RX2`; that was wrong.

**On the TinyB210s tested, the `RX2` / `TX/RX` selection made no measurable
difference.** The board brings its ports out on u.FL sockets, through
IPEX-to-SMA cables to the enclosure. Selecting `RX2` or `TX/RX` in UHD
read identically to 0.0-0.1 dB on two boards at 60 dB gain, on every band
from 800 MHz to 2.66 GHz, a strong Wi-Fi router included. Whether that is
how the clone wires its RF switch or something else is not verified -- but
it means **"antenna port vs. the other port" is not a usable test on this
board**. Put the antennas on the connectors whose cables lead to the u.FL
sockets marked RX, and check a working antenna physically: on vs. off
(below).

Which SMA on the enclosure is an RX port depends on how the cables are
routed, so check the cable, not the shell position. The RX LEDs light while
receiving (the snippet below blinks each channel a different number of
times); on this clone the selection `TX/RX` lit nothing:

```python
import time, uhd
u = uhd.usrp.MultiUSRP("type=b200"); fs = 1e6
for ch in (0, 1):
    u.set_rx_rate(fs, ch); u.set_rx_freq(uhd.types.TuneRequest(2.437e9), ch)
seq = [(0, "RX2", 1), (0, "TX/RX", 2), (1, "RX2", 3), (1, "TX/RX", 4)]
for _ in range(10):                       # watch: N blinks, pause, repeat
    for ch, ant, n in seq:
        u.set_rx_antenna(ant, ch)
        for _ in range(n):
            u.recv_num_samps(int(0.35 * fs), 2.437e9, fs, [ch], 30)
            time.sleep(0.35)
        time.sleep(2.0)
```

The LED that blinks **once** is channel 0, the TDOA receive input, which
every node needs an antenna on. **Three** blinks is channel 1. The two- and
four-blink selections (`TX/RX`) stayed dark.

**Use about 60 dB of gain for any receive test.** Measured on two
TinyB210s: the noise floor is flat up to ~40 dB of gain (−65.7 → −60.2
dBFS -- the converter's own noise) and climbs ~0.9 dB per dB above it
(−44 at 60, −29 at 76). At 30-40 dB everything weak sits under that
masking floor, and antennas, ports and directions all look alike. A day of
tests at 30-40 dB concluded "the antennas do nothing"; at 60 the same
antennas plainly work.

**Check that an antenna works** -- physically, at 60 dB, with a **distant,
steady** source: a phone hotspot in another room (beacons ~10/s, no
traffic needed), the laptop's own Wi-Fi off or on another channel. Count
the source's bursts above a threshold with the antennas on, then off.
Measured: vertical antennas on, 408 bursts in 20 s; off, 4. Compare
**counts**, not the mean level of detected bursts -- with antennas off only
the rare strongest bursts clear the threshold, so their levels look higher
and the comparison inverts. A **near** source is no test: a router ~1 m
away reached +18-22 dB with the antennas off and gained only 1-4 dB with
them on, because at that range it couples into the cables and the board.

**Antenna orientation matters as much as the antenna.** Two boards side
by side, same phone in another room: vertical antennas 408 bursts,
horizontal 49 -- Wi-Fi (and a drone's link) is mostly vertically polarised,
and a horizontal element also has a null along its own axis. Mount every
node's antennas vertical, all alike.

**Switch off any Wi-Fi router within a metre or so first.** One under the
desk coupled straight into the bare board and dominated every reading; the
laptop's own Wi-Fi is equally useless as a test source at arm's length.

---

## Procedure

**1. Wire it.** source → attenuators (if TX) → splitter → two equal cables →
the board's two receive inputs, channel 0 and channel 1 (on a TinyB210 the
RX connectors -- follow the cables, see above). No antennas connected.

**2. Measure, with calibration explicitly zero.**

```bash
python3 hw_selftest.py --channels 0,1 --fc 2437e6
```

It prints inter-channel coherence and `phase delta ... deg`. Coherence
should be **high — above 0.9**. With one source split into both ports it
should be near 1.0; anything below ~0.7 means the wiring, not the board, is
the problem. Stop and fix that first: a phase delta measured at low
coherence is noise.

**3. Repeat 5–10 times** without touching anything. Record the spread. A
standard deviation of more than a few degrees means something is loose or
the source is too weak — find out why before trusting a median.

**4. Swap the two cables** at the splitter (cable A to RX1, cable B to RX0)
and repeat. The instrumental offset from the *board* stays put; everything
contributed by cables and splitter imbalance **flips sign**.

```
board_offset  =  (phase_normal + phase_swapped) / 2
cable_mismatch = (phase_normal − phase_swapped) / 2
```

The average is your calibration constant. The difference tells you how well
matched your cables are — if it is large, you have just learned something
useful about the harness rather than baking it into the board constant.

**5. Write it down**, in radians, per board:

```json
"hardware": { "cal_phase_rad": 0.1234 }
```

It is a **per-board** number. Ten nodes need ten calibrations, and swapping
a cable in the field invalidates one of them.

---

## The hazard that decides how the scan is designed

**Verify whether the offset survives a retune.** On AD9361-based boards the
two RX channels share an LO, which is what makes them coherent — but the
phase relationship between them can be re-established differently each time
the synthesiser is retuned. If that is true here, then a calibration taken
at 2437 MHz is **not valid after the dwell coordinator moves the fleet to
2412 MHz and back**.

Test it directly, because the answer changes the design:

1. Calibrate at 2437 MHz. Record the delta.
2. Retune to 2412 MHz, then back to 2437 MHz.
3. Re-measure without touching the wiring.

- **Same within a few degrees** → one calibration per band, and scanning
  costs nothing.
- **Jumps by tens of degrees** → bearings are only trustworthy within a
  dwell, and any of: calibrate per channel and re-apply on retune; keep a
  DF node parked while others scan; or accept bearings as a
  within-dwell-only measurement.

Also re-measure after the board has been running for twenty minutes.
Thermal drift is a smaller effect than retuning but it is not zero.

---

## Validating that it worked

**Zero-degree check.** With `cal_phase_rad` applied and the splitter still
connected, `phase_bearing()` should report **≈ 0°** — the two ports see the
same signal, so broadside is the correct answer. If it does not, the
constant went in with the wrong sign or in degrees instead of radians.

**Known-angle check.** Remove the splitter, fit two antennas at exactly
`array_baseline_m` apart, and put a source at a known bearing at least a few
metres away. Compare. Do this outdoors or in a large room — indoors, a
reflection arriving from a different direction biases the phase, and you
will be measuring your walls.

**Sanity on the ambiguity flags.** With spacing at or below λ/2,
`unambiguous` should be `true` and `candidates_deg` should hold one entry.
If it holds several, the spacing is wider than you think.

---

## What this does not fix

- **No elevation.** A two-element array measures azimuth in its own plane
  and assumes the source is on the horizon. Calibration does not change
  that, and bearings still will not shore up the vertical axis.
- **Front/back stays ambiguous.** Two elements cannot separate θ from its
  mirror (`mirror_deg`). A third element fixes it; so does geometry across
  two or more nodes.
- **Multipath still biases bearings**, and a reflection arriving off-axis
  biases them harder than it biases timing. Calibration removes a constant,
  not a scene.
