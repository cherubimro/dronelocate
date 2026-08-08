# TODO — algorithm upgrades toward field readiness

Outcome of an algorithm review (2026-08-05) against the state of the art for
passive localization of non-cooperative (emitting) drones. Verdict: the
two-step TDOA architecture and most component choices are at or near best
practice; the gaps below are ranked by how badly they bite on real signals.
Items 1–5 landed the same day — measured outcomes inline; item 6 (DPD) and
the smaller items remain. The hardware gate in CLAUDE.md (`hw_selftest.py`
on a real TinyB210) is unaffected and still decides the project.

## 1. CAF + carrier modeling in sigsim — DONE 2026-08-05

The sim is baseband-only: `NodeChannel.render` applies fractional delay,
clock error, path loss and noise, but no carrier term. Real nodes
downconvert from 2.4/5.8 GHz, and a moving emitter leaves a *differential*
Doppler between node pairs of (fc/c)·v·|û_i−û_j|. The demo's own orbit
(2π·900 m/180 s ≈ 31 m/s) gives pair offsets of ~250–400 Hz at 2.437 GHz;
plain cross-correlation of a 10 ms window attenuates by sinc(Δf·T) with
Δf·T ≈ 2.5–4 — past the first null, the peak collapses. LO rate offset does
the same in `free`/`holdover` clock modes (0.02 ppm at 2.4 GHz ≈ 49 Hz).
Same failure family as bugs 10/13: a baseband-only simulator structurally
cannot show it, and a field deployment hits it on day one.

Fix, in two halves:
- **sigsim**: model the carrier — Doppler ramp from emitter radial velocity
  (scene publishes velocity) plus LO offset from the clock discipline's rate
  error. Sample-clock skew stays unmodeled on purpose (0.05 samples over
  10 ms at 0.02 ppm — three orders below the carrier term, since fc/fs ≈ 10³).
- **tdoa**: cross-ambiguity function — segmented correlation, FFT across
  segments for the Doppler axis, peak pick, derotate, then the existing
  sinc-interpolated `cross_correlate` for the final sub-sample lag so none of
  the interpolator accuracy work is lost. Doppler span from a configured
  vmax and the event's fc. Static emitter → bin 0 → identical to today.

Bonus once CAF exists: FDOA becomes a free measurement row (position and
velocity observability) — see item 7. Note FDOA, unlike a bearing, *does*
carry vertical information when the emitter has vertical motion.

**Measured (validate_caf.py + live fleet):** with the carrier modelled, the
plain correlator's per-pair lag error reaches 26.7 ns *while still clearing
the quality gate*; the CAF recovers every pair to <1.5 ns and the Doppler to
<5 Hz. Live A/B on the orbiting fleet: CAF on 0.21 m h / χ² 0.37; CAF off
1.7 m h / χ² 5.0 / horizontal containment 46%. Zero measured Doppler
reproduces the old path to 0.005 ns. The Doppler-estimation stage costs
~35 ms of the ~60 ms solve (21 pairs, serial); batch the per-node segment
FFTs if that ever matters.

## 2. All-pairs correlation + full-covariance GLS — DONE 2026-08-05

Star-reference correlation (everyone vs the strongest node) makes the
reference's timing noise common to all N−1 measurements; the solver's
diagonal-σ weighting ignores that correlation. All pairs gives N(N−1)/2
measurements whose proper weighting is a GLS with
R = A Σ_node Aᵀ + σ_interp² I (A = pair incidence matrix, Σ_node split from
the measured pair sigmas by least squares) — the ML combination, and a free
accuracy gain. Legacy star path kept verbatim (`solve_tdoa` untouched;
`solve_tdoa_gls` is a separate function).

**Measured:** mean horizontal error over 6 deterministic bursts 4.33 m
(star) → 3.54 m (GLS), with the gain concentrated on the bad bursts
(8.6 → 4.8 m) — exactly where correlated-reference noise hurts. The feared
3.5× correlation cost was erased by caching one forward FFT per node
(`node_spectra`): all-pairs at 21 pairs runs ~27 ms vs the old star's 37 ms.

## 3. Tight coupling: TDOA rows straight into the EKF — DONE 2026-08-05

Fix-then-filter discards every burst with <3 surviving correlations — the
`starved` counter is literally counting thrown-away information. A burst with
1–2 good pairs still updates an existing track: EKF measurement
h(x) = (|p−n_i| − |p−n_j|)/c with innovation (chi-square) gating so a single
bad pair cannot drag the track. No track birth from underdetermined bursts.
Main solved path stays fix-level (its covariance calibration is measured and
trusted); tight coupling is the recovery path.

**Measured:** two rows pull a 31 m coasted track to 5.7 m
(validate_math.py); an 8 µs outlier row is gated out; fired live under a
harsh quality gate (`starved 1 tight 1/0 rej` in the 5 s stats line).
Tight updates do not yet emit a track message — console shows the next full
fix (see Smaller, later).

## 3b. Synthetic CP-OFDM emitter — DONE 2026-08-05

Prerequisite for items 4 and 5, and the answer to "can those be built before
hardware": yes, once the emitter stops being spectrally flat.
`master_waveform(kind="ofdm")` is CP-OFDM — 128 useful samples (18.75 kHz
spacing at 2.4 Msps, LTE-adjacent like OcuSync), CP 16, QPSK data, fixed
pilots every 8th carrier, DC null — still deterministic from the burst id,
so nodes agree without exchanging it. `sim.waveform` selects it; the demo
config uses `ofdm`, `noise` stays for isolating solver behaviour.

Known property, not a bug: the CP puts autocorrelation sidelobes at
±128 samples (53 µs). Outside the demo's ~30 µs search window, inside a
clock-widened one; the quality gate is what catches it. Correlation quality
is 45 vs the noise waveform's 47.

## 4. GCC-HT (Hannan-Thomson / Eckart) prewhitening — DONE 2026-08-05

One FFT-domain weight on the cross-spectrum before the inverse transform,
built from per-node Welch PSDs (`welch_psd` → `ht_weight`) and applied on
the cached-spectra fast path. Per-channel SNR spectra give |γ|², floors from
the lower PSD quantile — which is exactly why a flat waveform makes this a
no-op: no valley to measure a floor in.

**Textbook HT lost to its own regularisation.** The ML form's 1/(1−|γ|²)
factor rails once in-band coherence saturates (which it does at any SNR
worth localizing), degenerating into PHAT-style whitening: measured 4.0 ns
RMS against plain correlation's 7.2 on an interfered pair — quality spent
for nothing. Dropping that factor and keeping |γ|²/√(Sa·Sb) — coherence to
zero the bins only one node trusts, SCOT division to crush interference by
its own power — gives **2.5 ns**. Wired as `solver.gcc` = `ht`|`none`,
console button, `/control?gcc=0|1`.

**Measured (validate_ofdm.py):** per-node interference (one 500 kHz,
20 dB-INR blob per node in distinct bands) plain 8.9 ns RMS → HT 3.5 ns.
Clean case identical (1.4 vs 1.4 ns) with *better* peak quality (46 vs 42),
because out-of-band bins go to zero. Costs ~15 ms per burst at 21 pairs.

Across a full 10-node fleet *with the outlier defences disabled* the
per-pair gain compounds through the solve: plain **31.1 m** horizontal,
χ²_red 1.8e4 — a confidently wrong fix — against GCC's **2.3 m**, χ²_red
0.55. `sim.interference` (off by default, so the documented baseline stays
clean) gives each node its own band-limited neighbour; enable it and toggle
the console's GCC-HT button to watch it live.

**Live, on the full system, the margin is much smaller — and that is the
honest number.** Interfered fleet, everything else on (gate + robust +
all-pairs GLS + GDOP selection): GCC on 0.29 m median / 0.56 m p90 /
χ² 0.20; GCC off 0.55 m / **3.19 m p90** / χ² 2.96; back on 0.36 m /
0.95 m / χ² 0.21. The defences already absorb most single-pair damage, so
weighting earns its keep in the *tail* (5.7× at p90) rather than the median.
Two side effects worth knowing: interference costs detections outright
(12 fixes per 22 s window vs 8 with weighting off vs ~44 clean — the
interferer lifts the noise floor under the energy gate), and correlation
*quality* reads slightly lower with GCC on (28 vs 33) while accuracy is
better, because the weight suppresses the sidelobe floor the quality metric
divides by. Do not tune the gate on weighted quality figures.

Known limit by construction: an interferer in the *same* band at *both*
nodes keeps |γ|² high and survives on power alone. Separating that needs
true cross-spectral coherence over segments, which needs per-pair Doppler
derotation first — see Smaller, later.

## 5. Cyclostationary (cyclic-prefix) detection — DONE 2026-08-05

`Node._cyclo_detect`: sliding-window normalised lag-N_u autocorrelation over
candidate useful lengths (64/128/256), then the strongest spectral line of
that product for the symbol period, hence the CP length. Immune to carrier
offset and Doppler by construction — both cancel in x(t)·x*(t+N_u), which is
what makes it the right *detector* for a moving emitter. Sliding rather than
global: a whole-window mean dilutes the statistic by the burst's duty cycle.

**Measured:** recovers N_u=128, N_cp=16 exactly; silent on the flat noise
waveform and on pure noise; fires at **−2 dB where the energy gate needs
+8 dB — 10 dB of margin**. Bursts the energy gate would have dropped are now
kept (`cyclo_saves` stat) and classified `uas_ofdm` with `n_useful`/`n_cp`
in the event's `hw` block. ~0.6 ms per burst per node. `radio.cyclo_detect`.

Still field-calibration work, as predicted: the 6.5σ threshold and the
drone-vs-Wi-Fi margin are set against synthetic data. Real 2.4 GHz occupancy
decides both. The mechanics are proven; the constants are not.

### Which captures can supply those constants (surveyed 2026-08-05)

**DroneRF (Al-Sa'd 2019) cannot — do not spend the 4 GB.** Verified by
downloading it: the distributed CSVs hold **one real integer per sample**,
not complex IQ. The FFT of a segment is Hermitian to 1e-16, and the authors'
own code keeps only half the spectrum. Quadrature is gone irrecoverably, and
with it the sign of the frequency offset. Measured cost to a CP detector on
synthetic CP-OFDM: the complex statistic is flat at CP/symbol regardless of
carrier offset, while the same statistic on Re{z} is cosine-modulated by that
offset and **nulls completely at fs/4N** (0.2005 → 0.0029, a 37 dB collapse).
A threshold fitted there measures the emitter's residual carrier offset, not
its cyclostationarity. Hilbert reconstruction does not repair it — the
analytic signal sums the true +f content with the conjugate-mirrored −f.

Two further blockers even ignoring format: its "no drone" class is a
near-empty lab band (~4% duty cycle, peaks 30 dB below the drone), so it
cannot calibrate a false-alarm rate; and all three of its drones link over
2.4 GHz Wi-Fi — **the drone class *is* Wi-Fi**, so there is no
drone-vs-Wi-Fi boundary in it to measure. Its headline 99.7% detection is
separating −18 dBFS from −49 dBFS, which any energy detector does; and that
figure comes from a 10-fold split shuffled over sub-segments where each
0.25 s recording contributes 100 siblings. Re-run with recording-grouped
folds, its drone-identification macro-F1 falls 0.742 → 0.455, i.e. chance.

Use instead:

- **NIST mds2-2731** — Wi-Fi and Bluetooth I/Q recordings, drone-free *by
  construction*, 30 MS/s at 2437 MHz, HDF5 complex baseband, 900 × 1 s, open
  with no login. This is the H₀ class for a false-alarm rate.
- **DroneDetect V2** — the drone-vs-Wi-Fi margin, because it is the matched
  experiment: 7 drones × 3 modes × **clean / BT / Wi-Fi / both**, complex64.
  No drone-free class, fixed 4 m range.
- **DroneRFb-DIR** — urban, all of 2.4–2.48 GHz with a labelled background
  class and real interference; verify the container before writing a loader.

**But the threshold that will actually hold has to be local.** Band
occupancy is site-specific and none of the above was recorded in Timișoara.
Validate the detector *implementation* against NIST, then capture the real
constant with a B210 on site — UHD already runs under 3.11 on both boxes,
so this is a hardware-availability task, not a software one. Whenever a
classifier margin is quoted from any of these sets, group the folds by
recording; segment-level splits inflate all of them toward chance.

## 5b. Frequency hopping + wideband channelization — DONE 2026-08-06

The emitter used to sit on the node's tuned frequency, so the system could
never miss. Real datalinks hop on a schedule the receiver is not told, and a
missed detection is unrecoverable in a way a degraded measurement is not.
`sim.hopping` derives the hop from the burst id (same coordination-free
trick as the waveform; the receiver is told nothing), places the burst at
that offset in each node's baseband, and discards whatever falls past
Nyquist rather than aliasing it back — an out-of-band burst does not arrive
weak, it does not arrive.

**Measured (validate_hopping.py):** a 2.4 Msps node against a 17.5 MHz hop
span sees **1 of 16** bursts, median post-hop SNR −106 dB. A 20 Msps node
plus `dronelocate/channelizer.py` finds **16 of 16**, with all four nodes
agreeing on the channel every time and TDOA error median 6.1 ns / worst
31 ns. Wire cost falls by `decim` because only the chosen channel is
shipped. Enabled by `radio.channelize` (on in the B210 profile).

**Channelization fights bandwidth — the B210 profile had them both on and
that was wrong.** Decimating to isolate a hop only makes sense when the
target is narrowband; an 18 MHz signal decimated to 2.5 MHz is discarded,
not isolated (measured: 0.26 m → 418 m horizontal). `node.py` now refuses
to decimate below the configured `bw_hz` and prints why, and the B210
profile ships with `channelize` off because it targets a wideband video
downlink. Turn it on only for a narrowband hopping target.

**The channel grid is load-bearing, not a convenience.** Nodes snap to a
shared grid, which is what lets them agree without exchanging a message.
Replacing it with a per-node estimate of the true hop centre would inject
up to half a channel step — 1.25 MHz here — of *differential* offset
between nodes, three orders beyond the CAF's ~1 kHz Doppler span, and the
correlation would simply fail. Same family as the burst-seeded waveform and
the UTC capture grid.

**It also exposed a real defect in our own OFDM emitter.** Pilots were
identical on every symbol, making the waveform periodic at the symbol rate
and putting autocorrelation sidelobes at multiples of N_u+N_cp at ~17% of
the peak. Harmless at 2.4 Msps (144 samples = 60 µs, outside the ~30 µs
search window) and actively wrong at 20 Msps (7.2 µs, well inside it): three
of eight wideband bursts locked onto a sidelobe and returned a confident lag
off by exactly two symbol periods. Pilots are now BPSK-scrambled per symbol,
as real OFDM does; sidelobes fell from 0.168/0.164/0.170 to
0.009/0.005/0.009 and the CP structure the cyclostationary detector uses is
untouched. **A narrowband simulator could not have shown this** — the
sidelobe only enters the search window once the capture is wide.

## 5c. Multipath — MODELLED 2026-08-06, and the fix is bandwidth

`sim.multipath` adds per-node reflection taps (exponential delay profile,
Rician K). Drawn **once per node and reused**, because reflectors are fixed
geometry — a per-burst redraw would average away over a run, while a static
profile biases every measurement from that node the same way. Off by
default; the documented baseline assumes free space.

**Measured, 2 MHz, K = 8 dB, τ_rms 200 ns.** Per pair: |bias| goes 1.1 →
12.3 ns median (worst 60) while run-to-run precision is **unchanged at
0.4 ns**. That is the whole danger — the measurement stays exactly as
repeatable as before and is simply wrong, so nothing that reasons about
spread can see it. Across the fleet: horizontal 0.21 → 4.57 m, vertical
**2.1 → 114 m**. The vertical is hit 25× harder because VDOP ≈ 16
amplifies per-node bias.

**What catches it, partially:** χ²_red rises 0.21 → 94.9, so the system
does know something is wrong, and the covariance inflates ~10×. Not enough
— containment collapses from 100% to 0%. Honest degradation, still
under-reported.

**What does not catch it:** robust loss. 4.57 → 4.50 m, i.e. nothing. Huber
scales by the *median* residual spread, so it finds a node out of line with
its peers; multipath biases every node, so there is no outlier to find.
This is our own documented reasoning about Huber, confirmed the hard way.

**What does fix it: bandwidth.** At 18 MHz the resolution cell is 17 m
(56 ns) instead of 150 m (500 ns), the echoes become resolvable, and the
error falls to **0.26 m horizontal / 35 m vertical** — 17× better
horizontally from bandwidth alone. This is the same lever as pointing the
B210 at the video downlink rather than the control link, and it is now the
strongest argument for doing so.

**Negative result worth keeping: leading-edge peak detection does not
work.** It is the textbook answer, it was implemented, measured, and
removed. Three findings, in order of how much they cost to learn:
- Unbounded, it is catastrophic: the search window spans the array's
  geometric range (tens of µs) while an echo is at most a µs or two late,
  so it accepted a structural peak 21 µs early and put the fix **158 km**
  out. Any first-arrival search must be bounded to a plausible excess
  delay before the maximum.
- Bounded, it is a no-op wherever the direct path is strong: the maximum
  already *is* the direct path, and the residual bias comes from
  unresolvable near-in echoes distorting the main lobe. Nothing to pick.
- With the direct path shadowed — the one case it should win — it traded
  horizontal 38 → 15 m for vertical 38 → **520 m**, because inconsistent
  per-node peak choices are exactly what VDOP amplifies.
So multipath is a bandwidth problem and a survey problem, not a
peak-picking problem. Super-resolution over the correlation is the only
remaining software avenue and is not obviously worth it.

## 6. DPD (direct position determination) for the weak-signal regime

Two-step is near-optimal at high SNR; several dB from optimal exactly where
drones are hard (far, weak, below the quality gate). DPD grid-searches
position and coherently sums correlation energy across all pairs. Pragmatic
form: keep two-step in the hot path, run DPD only on bursts that starve or
fail the gate today. Heavy; do after 1–3 land and only if weak-target
performance matters operationally.

## 7. Smaller, later

- **Cross-spectral coherence weighting** (segment-averaged Sab, not the
  per-node-PSD proxy) to catch a common-band interferer at both nodes —
  needs per-pair Doppler derotation before the segments can be averaged.
- **Wi-Fi vs drone classification** from the recovered CP geometry — the
  measurement (`n_useful`, `n_cp`) already ships in the event's `hw` block;
  nothing consumes it, and the decision boundary needs real captures.
- **FDOA rows** from CAF into solver/EKF (velocity observability, vertical).
- **IMM** (CV + coordinated turn) for multirotor maneuvering; **JPDA** for
  multiple targets (known limit, single-target NN gate today).
- **Multipath-aware peak logic** (first-peak-vs-max, super-resolution on the
  correlation) once real urban captures show the bias.
- Console emission of tight-coupling track updates (today only full fixes
  publish).

## Explicitly endorsed as-is (do not churn)

Sinc sub-sample interpolation (at the theory floor, benchmarked), clock-
centered narrow correlation windows (bug 13), gate + Huber as separate
defences, χ²-scaled covariance with measured containment, GDOP subset
selection, ENU solving, ci2 uplink quantization, grid-seeded Gauss-Newton
(closed forms buy nothing here).

## Boundary (state it in any deployment claim)

All of this localizes *emitters*. Non-cooperative ≠ non-emitting — consumer
drones stream video continuously — but a fully RF-silent autonomous drone is
invisible to TDOA/DF/CAF alike; that regime needs passive radar, acoustics,
or EO/IR. Bearing fusion (CLAUDE.md priority 4) remains the right hedge for
GPS-denied operation — **but not for the vertical axis**; see the correction
there.

**If the passive-radar fallback is ever built, it is a different band, not
just a different mode.** A 10-inch carbon-fibre propeller resonates as a
half-wave dipole near 600 MHz; measured drone RCS peaks there and falls
~6 dB by 1 GHz. Every published passive detection of a small drone uses a
UHF illuminator — DVB-T, DAB or LTE450, roughly 450–770 MHz — never the
2.4 GHz band this system works in. Expect 0.5–3 km against small airframes
(the 5 km outlier used a 50 kW transmitter); median airframe RCS is −20 to
−15 dBsm with 5–9 dB of fluctuation loss on top. Nobody in the open-source
world has passively detected a drone at all.

What would carry over from this codebase: the transport, `geo.py`, the CAF
(passive radar's core computation *is* a reference-vs-surveillance
cross-ambiguity), sub-sample interpolation, the tracker, and the solver's
grid seed / Huber / χ² machinery — only the residual changes from a TDOA
hyperboloid to a bistatic-range ellipsoid. What is genuinely new: two
antennas per node with isolation between them, and direct-path cancellation
(the direct signal runs ~40 dB above the target return), which we have none
of. Treat it as a second sensor sharing a chassis, not a replacement.

## Order of improvements:

Do now, in this order:

1. The two doc corrections. Twenty minutes, no code, and the bearing one is the highest-value item on either list per unit effort — a wrong rationale sitting in the roadmap ("bearings will fix the vertical") will misdirect whoever picks up priority item 4, possibly me in three weeks. Fix that and the UHF band note.

2. Frequency hopping. The single biggest gap. A real OcuSync link hops; our node sits on one frequency and our emitter obligingly transmits there. This is the only item on either list that could be fatal to the concept rather than merely degrading — if we can't detect a hopping link, nothing downstream matters. Testable entirely in sim, and it follows the pattern that has found every real bug here.

3. Multipath. Second, but I want to flag that it's the more insidious of the two. A miss is visible — no fix appears. Multipath produces a systematic range bias where every individual measurement looks internally consistent, so χ� stays healthy and the covariance keeps claiming confidence. That's bug 2's family: confident and wrong. I put it second only because hopping can break the concept outright while multipath degrades it. If you'd rather see the subtler failure first, that's a defensible swap.

4. Hysteresis. Cheap, real, not worth its own session. I'd fold it in as a rider whenever we next touch the tracker — which multipath work probably will.

Defer, with a clear trigger:

5. The angular spectrum likelihood. Still the best idea in the survey, and it dissolves bug 5 rather than working around it. The blocker is that implementing it honestly means first building a synthetic dual-channel array in sigsim — and then we'd be validating a bearing fusion design against a model of an array we don't own, at 2.4 GHz, where the research says a circular array has no elevation information anyway. That's the "optimising against ourselves" trap in its purest form. Trigger: a dual-channel B210 on the bench. At that point it moves to the top of the list.

Skip:

6. Phase-slope delay estimation — equal accuracy by a different route, but it earns that from long high-SNR averaging and ours is measured on the actual one-shot burst. Revisit only if dwell ever lengthens. 7. DPD — heavy, and it only pays in an SNR regime we can't characterise until hardware exists.

So the honest shape is: two corrections and two simulator-honesty items, in that order, then stop until a board arrives.



