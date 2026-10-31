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

# the three you actually want
CONFIG=config/site-laptop.json    ./run_demo.sh   # smooth on a laptop, both tabs
CONFIG=config/site-scan-lite.json ./run_demo.sh   # COORDINATED SCANNING, 4 channels
./run_demo.sh                                     # ten nodes, the baseline

./run_demo.sh --hw n05                    # n05 on a real dongle (SoapySDR)
./run_demo.sh --hw n10 --hw-source uhd    # n10 on a real B210 (UHD)
./run_demo.sh --no-clock                  # show what uncalibrated clocks do

# radar (Navico HALO) -- see "Radar" below; protocol client, radar console and
# recordings are in testradar/, nothing else to install
./run_demo.sh --radar                     # plus an emulated radar, fused
python3 tools/make_radar_rec.py \
    --out recordings/sim_orbit.navrec.gz  # once: a radar recording of the sim drone
./run_demo.sh --radar-replay recordings/sim_orbit.navrec.gz   # ...played through the real radar path
./run_demo.sh --radar-live                # the real HALO on the LAN
```

Ctrl-C stops everything. Starting a second fleet on top of a running one is
refused — see [Two fleets](#two-fleets-look-like-a-broken-tracker).

Two screens, meant for two monitors: the 3D console on
<http://localhost:8080> and the **RF spectrum dashboard** on
<http://localhost:8081> (`--spec-port`, `0` disables). See
[Spectrum dashboard](#spectrum-dashboard).

Deploying for real — supernode on a Debian server, nodes on Raspberry Pis,
browser on a laptop — is written up step by step in **`debian-server.txt`**.
Short version: there is no separate Zenoh broker to install, because the
supernode *is* the hub.

### Which profile

`CONFIG=` picks the site. They differ by cost, not by capability, and the
difference is large enough that picking the wrong one looks like a bug:

| Config | Fleet | Rate | For |
|---|---|---|---|
| `config/site-laptop.json` | 7 nodes, 2.4 Msps, max_nodes 6 | 1 Hz | **demoing from a laptop with both tabs open** |
| `config/site.json` | 10 nodes, 2.4 Msps | 2 Hz | the documented baseline |
| `config/site-scan-lite.json` | 10 nodes, 4 channels | 2 Hz | coordinated scanning, still laptop-sized |
| `config/site-scan.json` | 10 nodes, 20 Msps, 80 MHz | 2 Hz | full 2.4 GHz sweep — **needs a real machine** |
| `config/site-mixed-b210.json` | 9 sim + 1 B210 | 2 Hz | bench work with real hardware |
| `config/site-b210.json` | 10 × B210 | 2 Hz | deployment profile, requires GPS lock |

```bash
CONFIG=config/site-laptop.json ./run_demo.sh
```

Measured on an i7-1165G7 (four physical cores, Iris Xe on the same die):
`site-laptop` solves in **70 ms** at load 0.8 with 2.4 m median vertical
error, while `site-scan` on the same
box reached load 16 with a **7.5 s** solve — one fix a minute, which looks
exactly like a broken console. Node count was not the difference; 20 Msps is
8× the samples per capture.

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
`win_amd64` wheel and needs Python ≥ 3.8), but `run_demo.sh` is bash.

Note that `CONFIG=...` is **bash syntax** — in PowerShell it is not an
environment assignment and silently does nothing. Pick a path:

```powershell
# WSL2 — a real Linux userspace, so the bash script runs unmodified. Best.
wsl bash -c "cd /path/to/dronelocate && CONFIG=config/site-scan-lite.json ./run_demo.sh"

# Docker Desktop — CONFIG is read from the environment
docker run --rm --init -p 8080:8080 -p 8081:8081 `
  -e CONFIG=config/site-scan-lite.json dronelocate

# Native Windows — run_demo.cmd, and -Config instead of CONFIG=
run_demo.cmd -Config config\site-scan-lite.json
run_demo.cmd -Config config\site-laptop.json
run_demo.cmd -Hw n10 -HwSource uhd
```

**Use `run_demo.cmd`, not `run_demo.ps1` directly.** PowerShell refuses to
run `.ps1` files under its default `Restricted` execution policy, and a
downloaded one is blocked again by the Mark of the Web even under
`RemoteSigned`. Neither applies to `.cmd`, so the wrapper starts the script
with the policy bypassed **for that one invocation** — nothing is changed
permanently and no administrator rights are needed.

If you would rather call the `.ps1` yourself:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_demo.ps1 -Config config\site-laptop.json

# or, per user and permanent (no admin), plus clearing Mark of the Web:
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
Get-ChildItem -Recurse | Unblock-File
```

`run_demo.ps1` (via `run_demo.cmd`) is the PowerShell equivalent of
`run_demo.sh`: same
interpreter probe, same `PYTHONUNBUFFERED`, same refusal to stack a second
fleet, same cleanup on ctrl-C. Prefer WSL2 or Docker where you can — they
run the bash script itself, so they cannot drift from it — the `.ps1` is a
second implementation and can.

**Confirmed working on Windows** (2026-08-11) via `run_demo.cmd`. The
execution-policy wrapper is what makes it work out of the box: invoking the
`.ps1` directly fails on a default install with "running scripts is
disabled on this system", which looks like a broken script rather than a
policy setting.

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

### Spectrum dashboard

<http://localhost:8081> — one waterfall and one min/avg/max periodogram per
node, for reading which sensor is closest and what it actually heard. A
second server on a second port because it is a second *screen*: both tabs
stay open, and neither feed can stall the other.

Nodes publish a 256-bin Welch periodogram per
capture on `dc/{site}/{node}/spectrum`, quantised to a byte per bin — 256
bytes against the ~47 kB of IQ the same capture holds, at `data_low`/`drop`
like health, because a spectrum that arrives late is worthless. It is
published **before** the detection gate on purpose: a clean noise floor is
how you tell a deaf node from a quiet band, and a waterfall that froze
whenever nothing was detected would hide exactly the case you opened it to
investigate.

Reading a panel: **dimmed + `no sig`** = below the fleet's own
`detect_threshold_db` · **`IN FIX`** = the solver used it in the last fix ·
**magenta** = a real radio, taken from the node's own health rather than the
config's `role` · RSSI and SNR are printed, so nothing has to be inferred
from colour.

Two deliberate refusals. **No per-panel autoscale** — every panel shares one
dB range, shown in the header, because autoscaling per panel is what a
single-trace analyser does and it makes the loudest and the deafest sensor
look identical, which is the one question this screen exists to answer. And
the **default colourmap is a single-hue lightness ramp, not a rainbow**:
measured, the linear ramp is monotonic in relative luminance and the classic
one is not, which is why a rainbow invents edges where the spectrum is
smooth. The classic map ships as a toggle — it is what operators have read
for twenty years — but as a choice rather than a default.

The `avg over` selector is wall-clock seconds, not a frame count, and the
header reports the period actually achieved rather than the one requested.
Under a scan, `view` holds one channel (`strongest` by default) instead of
chasing the fleet: panels showing three different bands at once read as an
uncoordinated fleet, which is the opposite of what a coordinator does.

---

## Radar

A Navico HALO 20+ can join the fleet as a second kind of sensor. Its node,
`radarnode.py`, turns spokes into small plot batches on
`dc/{site}/{radar}/evt/plot`, and the supernode fuses them with the TDOA
fixes (`dronelocate/fusion.py`). The radar matters where TDOA is weak:
four ground nodes leave the vertical to the prior, and a drone that does
not transmit is invisible to TDOA. With ten good nodes it adds nothing to
an emitting drone.

**testradar, the radar toolkit, ships in the tree.** The radar protocol
client (`navico.py`), the radar's own console (`viewer.html`,
`webserver.py`), the bench tools and the real HALO 20+ recordings live under
`testradar/`, a git subtree of the separate testradar repository
(<https://dev.cs.upt.ro/alin.anton/testradar>, AGPL-3.0-or-later, same
authors). Its README documents the HALO protocol and the recordings. Nothing
is pip-installed for it: `radarnode.py`, `tools/make_radar_rec.py` and
`validate_radarnode.py` put `testradar/` on their own import path
(`dronelocate/vendor.py`), and `run_demo.sh` serves the radar console from
there. `TESTRADAR=/path/to/checkout` points all of them at another copy.
The demo without a radar and `--radar` (emulated plots) do not touch it.

To update the subtree from upstream:

```bash
git subtree pull --prefix=testradar https://dev.cs.upt.ro/alin.anton/testradar.git main --squash
```

### Three ways to run it

| Command | Radar | Fuse |
|---|---|---|
| `./run_demo.sh --radar` | emulated plots from the sim drone (`radarnode.py --source sim`), no consoles | on |
| `./run_demo.sh --radar-replay FILE` | a `.navrec` recording through the real radar path: clutter map, detector, console | on for a recording made by `tools/make_radar_rec.py`, off for a real one |
| `./run_demo.sh --radar-live` | the real HALO, found on the LAN (`radars[].serial`, else `radars[].ip`); the node waits until it answers | on |

The radar is described by the site file's `radars[]` block (`config/site.json`
has an emulated one, `config/site-field.json` the field one). A radar
whose `az_offset_deg` is null is reported "not calibrated" and fuses
nothing.

A real recording (e.g. testradar's `recordings/ab_rotation.navrec.gz`) shows
a real room, not the simulated sky, so its echoes cannot match the emulated
TDOA nodes: Fuse starts off and the replay is there to show the console.

### Consoles

| Port | What |
|---|---|
| 8082 | range A, phosphor green -- testradar's viewer, served by the radar node |
| 8083 | range B, navy |
| 8084 | every range side by side, each with a full-screen link (supernode, `--radar-page-port`) |

The 3D console's radar row, all through `/control`: **Fuse** turns fusion
on and off at the supernode; **Console ↗** opens :8084; **On / Off** and
**Standby / Transmit** are relayed to the radar node, which sends them to
the radar. **Transmit makes the radar radiate.**

### Generating a radar recording of the sim drone

`tools/make_radar_rec.py` renders the drone that `scene.py` flies (the site
file's `sim.track`) as spoke frames the way the HALO sends them, for range
A and range B, over static clutter and receiver flicker. Played back with
`--radar-replay`, it goes through the same path as a live radar, and it is
in step with the simulation: the recording covers exactly one period of
the trajectory, `run_demo.sh` recognises it and starts `scene.py
--wall-clock`, and each pass of the replay starts at wall-clock time modulo
that period. So the echo is where the TDOA nodes hear the drone, and Fuse
stays on.

```bash
python3 tools/make_radar_rec.py \
    --config config/site.json --out recordings/sim_orbit.navrec.gz
./run_demo.sh --radar-replay recordings/sim_orbit.navrec.gz
```

About 50 s to make; the orbit gives a 360 s loop of about 20 MB.
`recordings/` is gitignored, so a fresh clone has to generate it. Use the
same `--config` for making and for playing: the recording is that site's
radar position and that site's trajectory. Other knobs: `--range-a`,
`--range-b` (0 = no range B), `--flicker`, `--blobs`, `--echo`, `--seed`.
It reuses report packets from a real recording (`--templates`, default
`../testradar/recordings/ab_rotation.navrec.gz`).

### In the field

Only the radar node's PC is on the radar's LAN, so the radar node runs
there and connects to the supernode like any sensor node, one process per
range:

```bash
python3 radarnode.py --site config/site-field.json \
    --radar-id r01 --scanner 0 --http-port 8082 --hub SUPERNODE_ADDRESS
```

`--scanner 1 --http-port 8083` adds range B; it is fused only if the site
lists `"scanners": ["A", "B"]`. `radars[].auto_transmit` sends transmit on
every connect (the HALO powers up in standby), for an unattended site --
opt-in, because the radar is a transmitter.

---

## Coordinated dwell

Ten nodes can already agree on capture *instants* from UTC alone. Nothing
told them which **frequency** to be on, and that gap is not cosmetic: TDOA
correlates IQ between nodes, so a fleet that disagrees about frequency
produces no usable pair at all — not a degraded one. Scanning without
coordination is strictly worse than staring.

Same trick as the capture grid, applied to frequency. The plan lives in
config and every node computes `plan[floor(t/dwell_s) % len(plan)]`, so
independently started nodes are on the same channel in the same window with
no message exchanged. `dc/{site}/dwell` then does what a shared grid cannot:
change the plan at runtime. Losing that broadcast costs nothing.

```bash
CONFIG=config/site-scan-lite.json ./run_demo.sh
```

**`dwell_s` must not be commensurate with the transmit cadence.** Measured
across four channels at 2 bursts/s: 0.2 s and 0.3/0.4/0.5 s give an even 25%
each; 0.35 s gives 28/22/28/21; and **0.25 s samples only two of the four**,
50/50, because `floor(t/0.25)` at `t = k/2` only ever lands on even indices.
The fleet scans diligently, never looks at half the band, and nothing in any
log says so. Faster is not worse and slower is not safer — check the share.

Coverage costs what it costs: with the emitter on one of four channels,
roughly a quarter of bursts reach a fix. What does *not* happen is a split
fleet — nodes are deaf together or hearing together, and split is the
failure this prevents.

### Fibre and LTE in one fleet

Each node picks its own wire format from its own `uplink_mbps`; the event
states the format per node and the correlator already dequantises per node,
so they mix freely. For a 20 Msps dwell at 2 bursts/s:

| link | format | uplink |
|---|---|---|
| fibre | `cf32` | 25.6 Mbps |
| LTE 10 Mbps | `ci8` | 6.4 Mbps |
| LTE 5 Mbps | `ci2` | 1.6 Mbps |

Without that, a wideband dwell simply does not fit a cellular uplink and the
scan is moot.

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
absolute Unix timestamp. Both are described in the project notes. The uncertainty
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
| `dc/{site}/{node}/spectrum` | `DATA_LOW` / `DROP` | PSD frame, display only |
| `dc/{site}/cmd/{node}` | `INTERACTIVE_HIGH` / `BLOCK` | retasking downlink |
| `dc/{site}/dwell` | `INTERACTIVE_HIGH` / `BLOCK` | scan plan |
| `dc/{site}/track/live` | `DATA_HIGH` / `BLOCK` | fused tracks |

The priority split is the reason a 47 KB IQ transfer never delays a detection
event. Health and spectrum drop rather than block, because a stale one of
either is worthless and should not apply backpressure.

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
supernode.py             correlator, tracker, dwell coordinator, SSE servers
scene.py                 ground-truth emitter (sim only)
console/index.html       3D console (Three.js, vendored)
console/spectrum.html    RF spectrum dashboard, second port
console/cache/           baked OSM building footprints, ODbL (see its README)
deploy.sh                push the fleet to Raspberry Pis over ssh
deploy/                  systemd unit + env template for a deployed node
validate_math.py         solver check with no transport involved
validate_caf.py          moving emitter vs the correlator, CAF recovery
validate_ofdm.py         OFDM emitter, CP detection margin, GCC weighting
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

The radar sensor node (`radarnode.py`) uses the Navico HALO protocol client
from the testradar toolkit, vendored under `testradar/` (AGPL-3.0-or-later,
same authors, its own LICENSE and AUTHORS in that directory), whose protocol
knowledge builds on [Mayara](https://github.com/MarineYachtRadar/mayara-server)
(Kees Verruijt and the Mayara contributors, Apache-2.0) and
[OpenCPN radar_pi](https://github.com/opencpn-radar-pi/radar_pi) (Copyright
(C) 2010-2025 David S. Register, Dave Cowell, Kees Verruijt, Douwe Fokkema;
GPL-2.0-or-later). No source file of either project is included.
