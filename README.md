# dronelocate

[![CI](https://github.com/cherubimro/dronelocate/actions/workflows/ci.yml/badge.svg)](https://github.com/cherubimro/dronelocate/actions/workflows/ci.yml)

A working Zenoh-based TDOA localization demo. Ten sensor nodes stream IQ to a
central supernode, which correlates, solves for a 3D position, tracks it, and
renders it on a 3D console with a live uncertainty ellipsoid.

All ten nodes are emulated by default. One can be swapped for a real SDR
through SoapySDR without touching anything else.

This is a research/demo codebase, not a finished product. For a mature,
field-deployable open-source RF direction-finding system, see
[KrakenSDR](https://www.krakenrf.com/) — a 5-channel coherent RTL-SDR array
with its own [DoA software](https://github.com/krakenrf/krakensdr_doa) and an
active user base. dronelocate's TDOA approach and KrakenSDR's phase-coherent
AoA approach are complementary, not competing.

## Quick start

**Docker** — the easiest way to run this on someone else's laptop, and the
only one that works the same on Windows, macOS and Linux:

```bash
docker build -t dronelocate .
docker run --rm --init -p 8080:8080 dronelocate
```

**Native (Linux):**

```bash
pip install -r requirements.txt
./run_demo.sh                 # ten emulated nodes
./run_demo.sh --hw n05        # n05 on a real dongle
./run_demo.sh --no-clock      # show what uncalibrated clocks do
```

`run_demo.sh` probes for an interpreter that can actually import
`zenoh, cbor2, numpy` and prints which one it chose, so it works even where
`python3` is not the right Python. Override with `PYTHON=/path/to/python`.
Stop it with ctrl-C — that fires the cleanup trap that kills all twelve child
processes. It refuses to start if a fleet is already running; `--force`
overrides, but read [Two fleets](#two-fleets-look-like-a-broken-tracker)
first.

**Ubuntu / Debian**, where `pip` is not installed by default and newer
releases refuse to install into the system Python (PEP 668):

```bash
sudo apt install python3-pip python3-venv
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
./run_demo.sh
```

For the sim demo a plain venv is fine. **If you intend to drive a real
B210 it is not** — see [Ubuntu / Debian with UHD](#ubuntu--debian-with-uhd),
because apt's UHD bindings are invisible inside one.

**Windows:** the Python core runs natively (`eclipse-zenoh` publishes a
`win_amd64` wheel and needs Python ≥ 3.8), but the launcher scripts are bash.
In order of least pain: Docker Desktop → WSL2 → a Linux VM → doing it by hand
in PowerShell. WSL2 is a real Linux userspace, so `./run_demo.sh` works
unmodified; it is lighter and better integrated than VirtualBox.

Console: <http://localhost:8080>

### Console controls

- **Ellipsoid / Truth / Rays** — view toggles. Rays run from each node that
  contributed a measurement, brightness by correlation quality; red means the
  peak was too near the noise floor to trust.
- **Quality gate / Robust fit / CAF / GCC-HT** — change how the *supernode
  solves*, live. CAF is the cross-ambiguity search over differential
  Doppler; switch it off and watch what a moving emitter does to a plain
  correlator. GCC-HT is the cross-spectral weighting that suppresses
  interference bins.
- **Node clocks: GPSDO / Holdover / Free** — retasks the emulated nodes' clock
  discipline over the bus. Switch to `Free` and watch the error grow as the
  clocks drift apart; `GPSDO` pulls it back.

---

## What it actually demonstrates

The supernode is never told where the drone is. Emulated nodes subscribe to a
ground-truth channel to render *their own* IQ — each applying its own
geometric delay, clock bias, drift, path loss and noise — and then publish
only detection events. The supernode pulls raw samples, cross-correlates, and
has to find the position itself. Truth reaches the console solely to score the
answer.

Delete the `dc/tm1/sim/truth` key expression and the system still localizes.
That is the point.

### Measured, ten nodes, 2 bursts/second

| | |
|---|---|
| Detection events | ~10/burst, ~1 KB each |
| IQ per snippet | 47 KB (10 ms @ 2.4 Msps, ci8) |
| Aggregate ingest | 3.5–4.7 Mbps |
| Correlate + solve (CPU, 21 CAF pairs) | ~60 ms |
| Reduced chi-square, median | ~0.4 |
| **Horizontal error, median** | **~0.2 m** |
| **Vertical error, median** | **~2 m** |
| Covariance containment (h / v) | 62% / 65% (target 68%) |

The sim models the carrier: at 31 m/s the emitter leaves 250–400 Hz of
differential Doppler between node pairs at 2.437 GHz, which a plain
cross-correlator turns into biased lags and a nulled peak. The correlator is
therefore a CAF (search lag *and* Doppler), every pair is correlated rather
than everyone-vs-reference (GLS with the full covariance), and a burst too
starved to solve still updates the track through an innovation-gated EKF
row. Click **CAF** off in the console to watch the old failure live.

The emitter is CP-OFDM (OcuSync-shaped), not a flat noise burst, because
signal structure is what the last two techniques need to exist at all.
Cross-spectral **GCC weighting** cuts timing error from 8.9 to 3.5 ns RMS
when each node sits next to its own interferer, and costs nothing when the
band is clean. A **cyclic-prefix detector** runs beside the energy gate: the
lag-N_u autocorrelation is immune to carrier offset and Doppler, so it fires
about **10 dB earlier** than energy detection, recovers the OFDM geometry,
and labels the event `uas_ofdm`. Neither is tuned for the real world yet —
the mechanics are proven synthetically, the thresholds need band captures.

Those last three are worth dwelling on. The figures used to be 28–38 m
horizontal and 90–130 m vertical, and almost all of that gap turned out to be
two defects rather than physics: a biased sub-sample peak interpolator, and
float64 losing 238 ns of resolution when time-of-flight was added to an
absolute Unix timestamp. Both are described in `CLAUDE.md`. The uncertainty
ellipsoid is now genuinely a 1-sigma surface — it contains the truth about 68%
of the time, which it never did before.

The horizontal/vertical asymmetry that remains is not a software problem and
no amount of filtering fixes it. With eight of ten nodes near ground level,
the vertical dilution of precision is roughly 16 against a horizontal under 1.

Two levers, both in `config/site.json`:

- **Elevation spread.** Nodes n05 and n08 sit at 45 m and 70 m deliberately.
  Set them to 8 m and watch the vertical error roughly triple.
- **Altitude prior.** `solver.alt_prior_m` constrains the weakly observed
  vertical so it stops corrupting the horizontal. It is applied twice: once
  in the solver, once in the tracker.

---

## Architecture

```
scene.py ──truth──┐
                  ├──> node.py ×10 ──events──> supernode.py ──> console
                  │         ▲                       │
                  │         └────── IQ pull ────────┘
                  │                                 └──> tracks (Zenoh)
```

### Key expressions

| Key | QoS | Purpose |
|---|---|---|
| `dc/{site}/{node}/evt/detect` | `INTERACTIVE_HIGH` / `BLOCK` | detection events |
| `dc/{site}/{node}/iq` | `BACKGROUND` / `BLOCK` | IQ queryable (pull) |
| `dc/{site}/{node}/health` | `DATA_LOW` / `DROP` | telemetry |
| `dc/{site}/cmd/{node}` | `INTERACTIVE_HIGH` / `BLOCK` | retasking downlink |
| `dc/{site}/track/live` | `DATA_HIGH` / `BLOCK` | fused tracks |

The priority split is the reason a 47 KB IQ transfer never delays a detection
event. Health drops rather than blocks, because a stale health message is
worthless and should not apply backpressure.

### Pull, don't push

Nodes publish a ~1 KB event and hold samples in a ring buffer. The supernode
picks a subset by greedy PDOP minimisation (seeded from an RSSI-weighted
centroid) and issues a Zenoh `get()` only to those. Ten nodes hear each burst;
typically six or seven get queried. Halving the transfers costs nothing in
accuracy because the discarded nodes were geometrically redundant.

### Topology and security

Star by default: the supernode listens, nodes connect as clients. With a
direct link and no intermediate router, TLS is genuinely end-to-end. Pass
`--tls-ca/--tls-cert/--tls-key` to enable mTLS. If you later introduce a relay
node, hop-by-hop stops being end-to-end and you want payload AEAD — the IQ
metadata already rides in the Zenoh attachment, which is the natural AAD.

---

## Wire formats

`config/site.json` → `radio.wire_fmt`:

| Format | Bytes/sample | 10 ms snippet | Correlation cost |
|---|---|---|---|
| `cf32` | 8 | 188 KB | reference |
| `ci8` | 2 | 47 KB | negligible — HackRF native |
| `ci2` | 0.5 | 12 KB | ~0.55 dB SNR |

`ci2` is the VLBI trick. When the uplink dominates latency — and in the
measured numbers above it does, at ~90 ms of the ~180 ms budget — this is a
bigger win than a GPU.

---

## Using real hardware

```bash
sudo apt install soapysdr-tools soapysdr-module-rtlsdr python3-soapysdr
echo 'blacklist dvb_usb_rtl28xxu' | sudo tee /etc/modprobe.d/rtlsdr-blacklist.conf
SoapySDRUtil --probe="driver=rtlsdr"
./run_demo.sh --hw n05
```

Note the frequency ceiling: an RTL-SDR's R820T2 tops out near 1766 MHz, so it
cannot see 2.4 GHz drone links. For a live test point it at **1090 MHz ADS-B**
— pulsed, well inside range, and aircraft broadcast their own position, so you
get free ground truth to validate the whole chain against.

Swapping to HackRF is a device-string change (`driver=hackrf`) plus raising
`fs_sps`. Nothing else in the code cares.

---

## Known limits

- **Clock error is a systematic, not noise.** The chi-square covariance
  scaling catches random error honestly but under-reports bias-driven error.
  Run `--no-clock`, or click **Free** in the console, to see a confident,
  stable, wrong solution — the exact failure mode an uncalibrated field
  deployment produces.
- **Accuracy is now bounded by the channel model, not the algorithms.** At
  ~0.4 m the simulator is clean enough that anything you see is coming from
  something real. Real hardware will be limited by things the sim does not
  model: surveyed node position error (3–5 m from a single GNSS fix, and it
  propagates one-to-one), multipath, and the board's actual PPS jitter.
- **No reference-emitter calibration loop.** The sim hands each node its true
  clock error, standing in for what calibration would recover. Building the
  real loop against a broadcast DVB-T or FM signal is the obvious next step.
- **Single target.** Data association is nearest-neighbour with a 2.5 km gate.
  Multiple simultaneous drones need JPDA.
- **GPU path is stubbed.** `pip install cupy` and `tdoa.py` will use it; the
  console reports whether it is active.

## Layout

```
dronelocate/geo.py       WGS84 <-> ENU, GDOP
dronelocate/sigsim.py    emitter (noise | CP-OFDM), channel, quantisers
dronelocate/tdoa.py      CAF correlation, 3D solver, all-pairs GLS
dronelocate/track.py     Kalman tracker + tight-coupling TDOA update
dronelocate/proto.py     key expressions, CBOR schemas
dronelocate/zconf.py     Zenoh session configs, TLS
dronelocate/uhd_source.py  B210 driver (API-checked vs UHD 4.5.0, no hardware)
node.py                  sensor node (sim | rtlsdr | uhd)
supernode.py             correlator, tracker, SSE server
scene.py                 ground-truth emitter (sim only)
console/index.html       3D console (Three.js, vendored)
validate_math.py         solver check with no transport involved
validate_caf.py          moving emitter vs the correlator, CAF recovery
validate_ofdm.py         OFDM emitter, CP detection margin, GCC weighting
theory.tex               the mathematics, in full (pdflatex theory.tex)
validate_uhd_timing.py   UHD timing convention + wiring, via a fake device
hw_selftest.py           B210 acceptance test — run before buying more units
smoke_test.sh            one-shot pipeline test
Dockerfile               reproducible demo, any OS
```

Start with `python3 validate_math.py` — it exercises the solver against known
truth with no Zenoh in the way, which is the right place to debug accuracy.

---

# Real hardware: TinyB210 / B205mini (UHD)

A B210-class board changes the architecture, because UHD exposes what cheap
SDRs cannot: **PPS-referenced hardware timestamps**.

## Why this matters more than bandwidth

`rx_metadata.time_spec` is the device's own timestamp for sample zero,
referenced to the GPSDO-disciplined clock. And captures can be *scheduled*:

```python
cmd.stream_now = False
cmd.time_spec  = uhd.types.TimeSpec(t_utc)   # every node, same instant
```

So the supernode names a UTC instant, all ten nodes open a window there, and
the only remaining differences between their buffers are propagation delay.
Compare that with the RTL-SDR/HackRF path, where sample zero happens at an
unknown time and you must recover the offset via reference-emitter
calibration. The `clock_bias_s` / `clock_drift_ppm` model in `sigsim.py`
exists to represent that unknown — with a GPSDO board it largely goes away.

Set `capture.mode` to `scheduled` in `config/site-b210.json`. Fall back to
`independent` only for boards without a usable PPS path.

## Ubuntu / Debian with UHD

The project was developed on openSUSE, where the packaged UHD bindings target
Python 3.6 and had to be rebuilt from source (`docs/uhd-py311-build.md`).
Ubuntu and Debian are far easier — `python3-uhd` targets the system `python3`
directly — but they have four traps of their own, all found during a real
bring-up.

```bash
sudo apt install uhd-host python3-uhd python3-pip python3-venv
sudo uhd_images_downloader          # B210 firmware + FPGA, ~100 MB
sudo usermod -aG usrp "$USER"       # then log out and back in
uhd_find_devices                    # must list the board before going further
```

**1. Two interpreters, one dependency each.** This is the one that wastes an
afternoon. `pip` puts `zenoh` wherever you pointed it; `apt` puts `uhd` in the
*system* Python's `dist-packages`. A plain virtualenv hides the latter, so the
venv has zenoh without uhd while `/usr/bin/python3` has uhd without zenoh —
and `run_demo.sh` picks the first interpreter that can import zenoh, which is
the one that cannot import uhd. The symptom is maddening because
`apt install python3-uhd` cheerfully answers *"already the newest version"*.

Two ways out — pick one:

```bash
python3 -m venv --system-site-packages .venv   # venv that can see apt's uhd
. .venv/bin/activate && pip install -r requirements.txt

# or skip the venv and put both under the system Python
/usr/bin/python3 -m pip install --break-system-packages -r requirements.txt
```

Settle it in one command before launching anything:

```bash
python3 -c "import zenoh, cbor2, numpy, uhd; print('all four OK')"
```

**2. `--break-system-packages` and missing pip.** Ubuntu 23.04+ marks the
system Python externally managed, so pip refuses without that flag; and
`python3` ships without pip or venv at all until you install them. Older
releases have neither problem and reject the flag as unknown.

**3. The `usrp` group.** UHD's udev rules set `GROUP:="usrp", MODE:="0660"`,
so without membership the radio fails with `USB open failed: insufficient
permissions` — from inside a node log, after the fleet is already up. Group
membership is fixed when a process starts, so an already-open shell will not
have it until you log out and back in; `sg usrp -c '...'` works meanwhile.

**4. `require_gps` defaults to true.** A config with no `hardware` block
gives `require_gps: true` and a 180-second timeout, so a bench board with no
GPS antenna waits three minutes and then dies with a traceback while the rest
of the fleet keeps running. Use the bench profile, which sets `require_gps:
false` and a 5-second timeout:

```bash
CONFIG=config/site-mixed-b210.json ./run_demo.sh --hw n10 --hw-source uhd
```

`did not lock within 5s` as a **warning** rather than a traceback is the
confirmation the right config loaded. `run_demo.sh` preflights the first
three of these and names the fix rather than failing deep in a node log.

Verified against **UHD 4.7.0** on Ubuntu and **4.5.0** on openSUSE.

## Two fleets look like a broken tracker

A run that dies partway leaves the rest alive — a hardware node raising takes
out only that node, and the cleanup trap fires only when `run_demo.sh` itself
exits. Start again and the second supernode cannot bind 7447 and gives up,
but the second `scene.py` and the second set of nodes come up anyway and
attach to the **first** supernode. Several truth publishers, each with its own
burst counter, plus duplicate nodes serving IQ; the tracker sees what look
like several emitters and, with a 2.5 km association gate, visibly catches the
target and loses it again.

Nothing in any log says *"you are running two fleets"*. The tell is in the
rates: measured during one bring-up, 5 bursts/s against a configured 1 and
11.4 Mbps ingest against a ~4 Mbps baseline, with bursts and fixes in perfect
lockstep — a healthy pipeline being fed by too many scenes. `run_demo.sh` now
refuses to start on top of a running fleet for exactly this reason.

## Run the acceptance test first

```bash
python3 hw_selftest.py --channels 0,1
```

The decisive number is **schedule jitter**, not offset. A constant offset is
common to every node and cancels in the TDOA difference; jitter does not.

| Jitter | Position error contribution | Verdict |
|---|---|---|
| < 50 ns | < 15 m | TDOA viable |
| 50–200 ns | 15–60 m | usable, degraded |
| > 200 ns | > 60 m | PPS path unusable; bearing-only |

If a clone fails that test, no amount of software fixes it — but the board is
still useful as a direction-finder, which needs no inter-node timing at all.

## Dual channel is a second sensor, not a spare

Two RX channels on one board share an LO and an ADC clock, so their relative
phase is meaningful — that is an interferometer. `phase_bearing()` in
`uhd_source.py` turns a coherent pair into a bearing.

Measured on synthetic signals: **0.02° std at 20 dB SNR, 0.25° at 0 dB.**

Spacing governs uniqueness, and it is a property of the array rather than of
the answer:

- **61 mm (λ/2 at 2.437 GHz)** — single unambiguous bearing.
- **Wider** — better precision, but the phase wraps and several true angles
  produce the same reading *while |sin θ| stays under 1*, so a range check
  cannot detect it. `phase_bearing()` returns every candidate in
  `candidates_deg` and sets `unambiguous=False`. The true angle is always in
  the set; resolve it with a third element or cross-node consistency.
- Two elements never separate front from back. See `mirror_deg`.

The strategic value: bearings need no inter-node timing, so DF keeps working
when GPS is jammed or spoofed — a realistic scenario in exactly the situations
this system is meant for. Fusing bearings with TDOA also improves the vertical,
which is the axis your ground-level geometry constrains worst.

## Buying advice

Verify on **one** unit before ordering nine more:

1. **Which UHD version.** "Built-in firmware, no file replacement" often means
   a baked FPGA image pinned to one UHD release. Being stuck on UHD 3.15 is a
   long-term tax.
2. **`gps_locked` and `ref_locked` both true**, and how they degrade when sky
   view is lost. A clone GPSDO is likely GPS + TCXO, not OCXO — fine over a
   10 ms window, poor in holdover.
3. **Schedule jitter** (above). The one test that decides the project.
4. **Sustained throughput.** 56 MHz is the AD9361 ceiling, not what two
   channels sustain over USB 3.0. Measure it.
5. 12-bit ADCs give ~72 dB versus HackRF's ~48. Better weak-signal detection
   next to a strong emitter, and it moves passive radar from "not viable" to
   "hard."

No PA on the TinyB210 is an advantage here: lower noise floor, better channel
isolation, and this is a receive-only application.

## Still not built

`uhd_source.py` is **unverified against hardware.** Its API surface has since
been checked symbol by symbol against a real UHD 4.5.0 install (all 38 exist),
which turned up four genuine bugs — including a `recv()` into a non-contiguous
buffer that would have silently lost samples on the two-channel configuration
the B210 profile uses. But no board has run it. `validate_uhd_timing.py`
exercises the whole path against a fake UHD device, so the plumbing is tested
without hardware; schedule jitter is not, and cannot be.

Remaining work for a real deployment:

- ~~Wire `UhdSource` into `node.py`.~~ Done — `node.py --source uhd`.
- Add the scheduled-capture coordinator. Half done: nodes derive capture
  instants from a shared UTC grid, so they agree without exchanging a message.
  What is missing is the supernode *naming* the instant, which is what lets it
  coordinate dwell.
- Feed bearings into `solve_tdoa()` as additional measurement rows. The
  measurement is already on the wire — a dual-channel UHD node ships
  `bearing_deg` and its ambiguity set in the detection event.
- Survey node positions properly. A single GNSS fix is 3–5 m and node position
  error propagates one-to-one into every target fix. Use RTK (ROMPOS for NTRIP
  in Romania) or a 24-hour static survey-in, then hardcode the result.

---

## License

AGPLv3 — see [LICENSE](LICENSE). Copyright (C) 2026 Alin-Adrian Anton,
University Politehnica Timisoara — see [AUTHORS](AUTHORS).

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU Affero General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version.

It is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
details.
