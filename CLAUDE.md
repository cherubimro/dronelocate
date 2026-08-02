# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# dronelocate — project context

Passive RF localization of non-cooperative drones. Ten sensor nodes stream IQ
to a central supernode over Zenoh; the supernode cross-correlates, solves TDOA
for a 3D position, tracks it, and renders it in a browser console.

There is no pytest/unittest suite and no linter config (no `pyproject.toml`,
`.flake8`, `ruff.toml`). Correctness is verified by the scripts in "Run it"
below — `validate_math.py` for solver accuracy, `validate_uhd_timing.py` for
the UHD clock-offset convention, `smoke_test.sh` for one-shot pipeline health,
and `hw_selftest.py` for the one hardware acceptance test that matters. Do not
go looking for a test runner or lint command that isn't there.

## Environment

Two machines, and they differ in ways that bite:

- **Deploy target: Debian, Intel NUC.** `python3` is already 3.11 there, so
  every script below works as written and `apt install uhd-host python3-uhd`
  puts the UHD bindings on the interpreter that runs the project.
- **Dev laptop: openSUSE Leap 15.6.** Here the default `python3` is 3.6.15
  and has neither `zenoh` nor `cbor2` — it cannot even import `dataclasses`.
  All project deps (numpy 2.2.4, zenoh 1.9.0, cbor2, scipy 1.15) are
  installed only for 3.11. `run_demo.sh` and `smoke_test.sh` probe for an
  interpreter that can import `zenoh, cbor2, numpy` and print which one they
  picked, so they work unmodified on both machines; override with
  `PYTHON=/path/to/python`. Invoke the `validate_*.py` scripts with
  `python3.11` explicitly.

Python only (Java path was dropped).

- Dev/test: 10 emulated nodes, no hardware needed.
- Target hardware: HamGeek **TinyB210** (B210 clone, AD9361, 2R2T, 70 MHz–6 GHz,
  EXT_CK + PPS input, onboard GPSDO). Driven via UHD.
- **UHD 4.5.0** on the laptop via `zypper install libuhd4_5_0 uhd-utils
  uhd-udev uhd-firmware python3-uhd`. The packaged bindings land in
  `/usr/lib64/python3.6/site-packages/uhd`, so that `import uhd` works **only
  under python3.6** — which cannot run this project. Worked around by building
  UHD 4.5.0.0 from source against 3.11 into `~/opt/uhd-py311`; `source
  ./env-uhd-py311.sh` puts it on the path. Debian's `python3-uhd` targets 3.11
  directly and needs none of this. Recipe: `docs/uhd-py311-build.md`.
- An RTL-SDR is on hand but **cannot see 2.4/5.8 GHz** (R820T2 ceiling ~1766 MHz).
  Use it only for 1090 MHz ADS-B work.
- Backhaul is likely LTE per node, possibly fiber. Uplink budget ~10 Mbps/node.

## Run it

Everything below runs with **no hardware at all** — ten emulated nodes.

```bash
pip install -r requirements.txt
./run_demo.sh                        # 10 emulated nodes → http://localhost:8080
./smoke_test.sh 30                   # one-shot pipeline test with bus diagnostics
python3.11 validate_math.py          # solver only, no transport — accuracy here
python3.11 validate_uhd_timing.py    # UHD timing convention + node wiring, no hw
```

`run_demo.sh` runs until ctrl-c and tails the supernode log; open
<http://localhost:8080> for the 3D console. `--hw n05` swaps one node to a
real SoapySDR dongle, `--no-clock` shows what uncalibrated clocks do.

## Running it on a machine that is not one of these two

The demo is sim-only and needs no radio. The recurring obstacle is never the
code — it is which Python is on PATH and whether `zenoh` is under it.

**Docker is the right answer for a laptop you do not control.** It pins the
interpreter and every dependency, and behaves identically on Windows, macOS
and Linux:

```bash
docker build -t dronelocate .
docker run --rm --init -p 8080:8080 dronelocate   # -> http://localhost:8080
```

Measured: identical to native on the same box (solve ~37 ms, err_h ~0.4 m,
~41% of one core's worth of CPU, 387 MB image). What Docker does *not* fix is
USB hardware — a B210 or RTL dongle needs `--device` passthrough plus udev
rules, and native is simpler for that. For the sim demo there is no downside.

**Windows.** The Python core runs natively — `eclipse-zenoh` ships a
`win_amd64` wheel and requires Python >= 3.8 — but the launcher scripts are
bash, so something Linux-shaped is needed. In order of least pain:
Docker Desktop, then WSL2 (a real Linux userspace, so `./run_demo.sh` works
unmodified, and lighter than a full VM), then VirtualBox/VMware, then
launching twelve processes by hand in PowerShell. Prefer WSL2 over a
traditional VM; prefer Docker over both unless they want to edit and re-run
constantly.

**Beware benchmarking under load.** Twelve Python processes want most of a
machine. Running the host demo and a container at once on 7 cores took solve
time from 37 ms to 2100 ms and horizontal error from 0.4 m to 95 m, which
looks exactly like a code regression and is not. Docker containers also share
the host kernel, so the container's processes show up in the host's `htop` --
easy to mistake for a stray native run. Check `docker stats` and
`/proc/loadavg` before believing a slow number.

**`requirements.txt` is unpinned** (`numpy>=1.24` etc.), so the image will not
be bit-reproducible over time -- the container currently resolves to numpy
2.4.6 / scipy 1.17.1 against the laptop's 2.2.4 / 1.15.2. Both were verified
to give identical results and speed here. Pin them if that ever stops being
true.

## Architecture

```
scene.py ──truth──> node.py ×10 ──events──> supernode.py ──> console (SSE)
                        ▲                        │
                        └──── IQ pull (get) ─────┘
```

Zenoh star topology: supernode listens, nodes connect as clients.

| Key expression | QoS | Purpose |
|---|---|---|
| `dc/{site}/{node}/evt/detect` | INTERACTIVE_HIGH / BLOCK | ~1 KB events |
| `dc/{site}/{node}/iq` | BACKGROUND / BLOCK † | IQ queryable (pull) |
| `dc/{site}/{node}/health` | DATA_LOW / DROP | telemetry |
| `dc/{site}/cmd/{node}` | INTERACTIVE_HIGH / BLOCK | retasking |
| `dc/{site}/track/live` | DATA_HIGH / BLOCK | fused tracks |
| `dc/{site}/sim/truth` | — | **SIM ONLY** |

† Not actually in effect. zenoh 1.9.0 deprecated `priority` and
`congestion_control` on `Query.reply` and **ignores both**, so the IQ reply
path runs at default QoS and warns on every reply. If bulk IQ starts starving
the event stream, this is why — the fix is `zconf.spoke(background_only=True)`
to put bulk on its own link, not the reply arguments.

**Design invariant: the supernode never reads `sim/truth`.** Emulated nodes
subscribe to it to render their own IQ (own geometric delay, clock error, path
loss, noise), then discard it and publish only detection events. Truth reaches
the console solely to score error in metres. Delete that key expression and the
system still localizes. Preserve this — it is what makes the demo honest.

## Files

```
dronelocate/geo.py         WGS84 ↔ ENU, GDOP
dronelocate/sigsim.py      emitter, channel model, ci8/ci2 quantisers
dronelocate/tdoa.py        FFT correlation, grid-seeded Gauss-Newton 3D solver
dronelocate/proto.py       key expressions, CBOR schemas, site config
dronelocate/zconf.py       Zenoh session configs, TLS/mTLS
dronelocate/uhd_source.py  B210 driver — API verified vs UHD 4.5.0, NOT vs hardware
node.py                    sensor node (sim | rtlsdr | uhd)
supernode.py               correlator, GDOP node selection, EKF, SSE server
scene.py                   ground-truth emitter (sim only)
console/index.html         3D console, Three.js vendored for offline use
hw_selftest.py             B210 acceptance test — RUN BEFORE BUYING MORE UNITS
validate_uhd_timing.py     UHD clock-offset sign + node wiring, via a fake device
config/site.json           10-node Timișoara layout, RTL-class radio
config/site-b210.json      B210 profile: 20 Msps, dual channel, scheduled capture
env-uhd-py311.sh           `source` it to use the 3.11 UHD build — laptop only
docs/uhd-py311-build.md    building UHD bindings for 3.11 — openSUSE laptop only
```

## Measured baseline (10 emulated nodes, 2 bursts/s)

Current, after the sub-sample interpolator and epoch-precision fixes:

| | |
|---|---|
| Aggregate ingest | 3.7 Mbps |
| Correlate + solve (CPU) | ~37 ms |
| Correlation residual, median | **1.1 ns** |
| Reduced chi-square, median | **0.4** |
| Horizontal error, median | **0.4 m** |
| Vertical error, median | **5.0 m** |
| Covariance containment (h / v) | **66% / 68%** (target 68%) |

The historical figures were 32 m horizontal / 89 m vertical / 58 ns residual /
chi-square 8.3. Nearly all of that gap was two defects, not physics: the
parabolic peak interpolator (bug 12) and float64 epoch precision (bug 10).
The covariance is now *calibrated* -- a 1-sigma ellipsoid contains the truth
about 68% of the time, which it never did before.

`supernode.py` prints these every 5 s. Caveats before using them as a gate:

- **Vertical error is not reproducible run to run.** `node.py` seeds each
  node's RNG from `abs(hash(node_id))`, and Python randomizes string hashing
  per process, so every run draws different per-node clock biases. Judge
  vertical on a distribution, or pin it with `PYTHONHASHSEED=0` or `--seed`.
- **Latency is hardware-dependent.** Solve time on the dev laptop is ~37 ms;
  on the original 4-core container it was ~72 ms with ~95 ms pull latency.
  Do not "fix" a discrepancy that is just a different box.
- **These are `gpsdo` numbers.** Switch the node clocks to `free` or
  `holdover` and errors grow without bound, by design.

## Bugs already found and fixed — do not reintroduce

1. **Clock drift anchored to Unix epoch.** `drift_ppm × 1.78e9 s` = 35 s of
   error, which turned a 10 ms capture into an 85M-sample allocation and
   wedged the node's callback thread. Drift must accumulate from a session
   epoch. There is now a guard returning noise when delay exceeds the window.
2. **Covariance was lying.** `sigma_v` shrank as real error grew. Now scaled by
   reduced chi-square. Known limit: catches random error honestly, still
   under-reports bias-driven (systematic) error.
3. **SNR via mean/median on an all-signal window** returns ~0 dB and silently
   detects nothing. The burst is now gated into the middle 40% of the window so
   there is a real noise floor. Detector is block-power peak vs median.
4. **`vdot(b, a)` in `phase_bearing`** conjugates the phase and mirrors every
   bearing about the array normal. Must be `vdot(a, b)`.
5. **Ambiguity checked on `|sin θ| > 1`.** Phase wraps before that, so a true
   30° read as −18.7° while still claiming unambiguous. Uniqueness is a
   property of the array (spacing ≤ λ/2), not of the answer. `phase_bearing()`
   now enumerates grating lobes into `candidates_deg`.
6. **`streamer.recv()` handed a non-contiguous view.** `recv(chunk[:, :want])`
   on a 2-channel buffer is strided, and the binding passes numpy's raw
   pointer to C++ — samples land in a discarded temporary. Silent, and only
   with `channels: [0, 1]`, which is what the B210 profile uses. Always recv
   into the whole contiguous scratch and clamp with `min()` on the copy out;
   that is what UHD's own `recv_num_samps` does.
7. **`stream_now=True` on a multi-channel stream.** UHD gates this itself
   (`stream_now = len(channels) == 1`): two channels must start on a common
   timed trigger or they begin on different samples, destroying the phase
   relationship the bearing math depends on.
8. **`time_aligned` set only on the GPS-lock path**, so reading it raised
   `AttributeError` on exactly the degraded path that needs to check it.
9. **The UHD clock-offset sign.** `clk_off_ns = -(t0_actual - t_slot) × 1e9`.
   Derived in `UhdNodeSource`'s docstring and asserted in
   `validate_uhd_timing.py`, which also runs it flipped — the wrong sign puts
   fixes 1.6 km and 4 km out while raising nothing. Do not "simplify" it.
10. **Absolute Unix timestamps in the delay computation.** `render()` built
   `arrival = t_epoch_s + tof + clk` and then subtracted the capture start.
   float64 spacing at t≈1.785e9 is **238 ns**, so tof and the clock error got
   rounded onto a 238 ns grid before the subtraction could recover them --
   ~70 ns RMS per node, ~100 ns between a pair, 30 m of range, thrown away
   for nothing. Form the delay *relative* to the capture start first
   (`(t_epoch_s - capture_start_s) + tof + clk`): the two epochs are large but
   close, so their difference is exact, and everything added after is small.
   This was invisible to `validate_math.py`, which uses t_emit = 0.0 and so
   has no epoch magnitude to lose precision against -- it reported 0.8 ns
   residual while the live pipeline sat at 58 ns. Same failure family as
   bug 1. **A synthetic test that starts the clock at zero cannot see this.**
11. **A stochastic clock model evaluated twice per capture.** Under `gpsdo`
   the discipline is a stateful OU process, so calling `clock_error_at()`
   again to fill in the metadata reported an error the signal never had. The
   supernode subtracted the wrong number and the difference was pure
   uncorrectable error (~21 ns). Evaluate once per capture, reuse the value.
   Harmless under the old deterministic model, which is why it appeared the
   moment discipline was added.
12. **Parabolic peak interpolation on correlation magnitude.** See the
   sub-sample section below -- replaced with sinc reconstruction.
13. **Correlation window bounded by geometry alone.** `max_lag` was
   `baseline/c + 5 µs` (~27 µs), but the correlation peak sits at *geometric
   TDOA + clock offset difference*, and only the first term is bounded by the
   baseline. Clock drift accumulates from session start, so after ~10 minutes
   the offset difference exceeds the window, the true peak falls outside it,
   and the correlator locks onto a noise peak and returns a confident wrong
   lag. Measured live after 26 min: clock deltas to 92 µs against a 27 µs
   window, **104 of 104 fixes** overflowing, χ²_red 1.09e6, horizontal error
   870 m — worse than vertical, which is the tell that it is not a geometry
   problem. `cross_correlate` now takes `center_lag_s` and the supernode
   centres the window on the known offset rather than widening it, which
   would sacrifice the sidelobe rejection the narrow window buys.
   **This is a run-time-dependent failure**: a fresh demo looks perfect and
   degrades as it runs, so a short smoke test cannot see it.

## Outlier defences (console buttons, `solver` config, live-switchable)

Two independent mechanisms, both default-on and both toggleable from the
console via `GET /control?gate=0|1&robust=0|1&quality_min=N`. They catch
different things, which is why both exist.

**Quality gate** (`quality_gate`, `quality_min`, default 8.0) drops a
correlation whose peak does not stand out from the noise floor. This is not
redundant with the per-measurement sigma, and the reason matters:
`timing_sigma()` returns the interpolator floor (~26 ns) for every quality
down to 4, and only 38 ns at quality 2.3. So a correlation that locked onto
pure noise — wrong by microseconds, i.e. kilometres — is handed within a
factor of two of a clean peak's weight. The sigma model assumes the peak
found is the *right* peak, merely blurred; it has no term for "this is a
different peak entirely". Only a gate can express that.

**Robust loss** (`robust`, Huber IRLS, k=1.345) bounds the influence of
whatever gets through. It catches outliers a gate cannot see — a node with a
wrong surveyed position correlates beautifully and still lies. Weights are
scaled by the *median* residual spread, not by the assumed sigmas: with
χ²_red sitting near 15, normalised residuals are ~4 across the board, so a
fixed threshold would down-weight everything equally, which is a rescale
rather than outlier rejection.

Measured, one node corrupted with an 8 µs lag error: plain least squares
330.6 m, robust **8.9 m**. `robust=False` is bit-identical to the pre-robust
code path. Live on the sim fleet, gate 8 + robust took median horizontal
error 26.1 m → 19.8 m.

Do not tighten `quality_min` to "improve" χ². A gate at 45 cut χ²_red from
17.7 to 5.9 and made the error *worse* (26.1 → 33.1 m) while losing a third
of the fixes — χ² is computed over the survivors, so an over-tight gate
flatters its own metric while starving the solve.

## Why vertical error is ~3× horizontal

Not a bug. Eight of ten nodes sit near ground level, so VDOP ≈ 16 against HDOP
< 1. Vertical error ≈ VDOP × timing error × c ≈ 16 × 30 ns × 0.3 m/ns ≈ 145 m.
Nodes n05 and n08 are elevated (45 m, 70 m) deliberately — set them to 8 m in
the config and vertical error roughly triples.

## Priority work

1. **`hw_selftest.py` against one TinyB210.** The decisive metric is schedule
   *jitter*, not offset — a constant offset is common to all nodes and cancels
   in the TDOA difference. <50 ns is viable, >200 ns means the clone's PPS path
   is unusable for TDOA (bearing-only fallback). Still the gate on buying more
   units, and still unrun.
2. ~~UHD bindings on the right interpreter.~~ **Done on both boxes.** The
   Debian NUC gets it from `apt install uhd-host python3-uhd`. The openSUSE
   laptop needed a source build into `~/opt/uhd-py311`
   (`docs/uhd-py311-build.md`); `source ./env-uhd-py311.sh` and
   `import uhd` works under 3.11 with `HAVE_UHD = True`. `node.py --source
   uhd` now runs the real driver through to device discovery. Remaining gap
   is hardware, not software — see item 1.
3. **Scheduled-capture coordinator.** Half done: `UhdNodeSource.next_slot()`
   derives capture instants from a shared UTC grid, so independently started
   nodes agree without exchanging a message, and the slot index doubles as the
   burst id the supernode groups on. What is missing is the supernode *naming*
   the instant on the bus, which is what lets it retask dwell. Note the loop is
   serial — it schedules the next slot only after the current capture returns,
   so with `schedule_lead_s` ≥ the period the effective rate drops. Pipeline it
   when the coordinator lands.
4. **Fuse bearings into `solve_tdoa()`** as extra measurement rows. The
   measurement is already on the wire: a dual-channel UHD node ships
   `bearing_deg`, `candidates_deg`, `unambiguous` and `coherence` in the
   detection event's `hw` block. Nothing consumes it yet. Dual coherent
   channels give a bearing per node with no inter-node timing, so DF survives
   GPS jamming and improves the weak vertical axis.
5. ~~Replace the parabolic peak interpolator.~~ **Done.** `cross_correlate`
   now locates the peak by band-limited (sinc) reconstruction of the complex
   correlation. Benchmarked against known fractional delays: parabolic on
   magnitude was 0.081 samples RMS with a systematic S-curve bias reaching
   0.12 samples (~50 ns, ~15 m); sinc reconstruction plus one refinement step
   is **0.0017 samples (0.7 ns)**, a 46x improvement. Jacobsen's complex
   quadratic was tried and is *worse* here (0.091) — with bandwidth/fs ≈ 0.83
   the peak is oversampled barely 1.2x, so no three-point fit has enough to
   grip. `timing_sigma()`'s interpolator floor dropped from 1/16 to 1/256 of
   a sample accordingly; that floor had been pinning sigma at 26 ns for every
   correlation quality above ~4, hiding the difference between a clean peak
   and a marginal one.
6. **Reference-emitter calibration** (only if the B210 PPS path disappoints).

## Conventions

- Filter and solver math in ENU metres. Convert to WGS84 only at display.
  Least squares in degrees introduces direction-dependent bias.
- Node positions are surveyed and hardcoded. A single GNSS fix is 3–5 m and
  node position error propagates 1:1 into every target fix. Use RTK (ROMPOS
  for NTRIP in Romania) or a 24 h static survey-in.
- Wire formats: `ci8` (HackRF/B210 native, 2 B/sample), `ci2` (VLBI 2-bit,
  0.5 B/sample, ~0.55 dB SNR cost). On a constrained uplink `ci2` beats a GPU —
  uplink is ~90 ms of the ~180 ms budget.
- A real radio reports what it measures. Adopt the hardware's *actual* sample
  rate and centre frequency (`get_rx_rate` / `get_rx_freq`), never the
  requested ones: the correlator turns lag indices into seconds with that
  number, so a requested-vs-actual mismatch is a multiplicative bias on every
  TDOA. Same rule for GPSDO lock in health — never publish a hardcoded `True`.
- Keep artifacts out of the hot path: numpy/CuPy only, no per-sample Python.
- `pip install cupy` and `tdoa.py` uses the GPU automatically.
