# ox-alpha.md — additional ladders

Complements TODO.md; deliberately does not repeat its items (DPD, bearing
fusion, coherence weighting, FDOA rows, JPDA/IMM, hysteresis, reference-emitter
calibration are all already there). Same discipline as everywhere else in this
tree: each rung states what it buys, what gates it, and how to prove it moved.
Nothing here touches accuracy except where it says so; most of it buys latency,
headroom, or robustness — which at 2 bursts/s is headroom, not necessity. That
is exactly why the triggers matter more than the ideas.

**This document's own gate, which supersedes every trigger below.**
`hw_selftest.py` against a TinyB210 has not been run, and the board was on the
bench on 2026-08-11 (TODO.md:350) — so this is not a hardware-availability
problem. The schedule-jitter number decides which half of this list is worth
anything: over 200 ns and most of it is moot (bearing-only fallback, different
project), under 50 ns and several rungs become urgent rather than optional.
Nothing here is worth starting before that number exists. A first pass of this
document was reviewed against the code on 2026-08-24; the corrections it
produced are folded in below, and three rungs did not survive.

---

## Ladder A — correlator throughput, no language rewrite

Read the budget first: the solve runs ~100 ms against a 500 ms burst period.
Every rung in this ladder buys latency the system does not currently need.
None of it starts before A's trigger fires: measured per-burst wall clock
breaching the deadline on the *deployment* box, thermally soaked, not on a
throttling laptop.

**A1. Batch the per-pair Doppler estimate.**
The batchable work is narrower than it first looks, and one earlier claim
about it was simply wrong.

*What is real:* `estimate_doppler(iq[ni], iq[nj], …)` (supernode.py:383)
re-segments and re-FFTs **both raw sides on every pair** — 21 pairs pay 42
segmented transforms over 7 nodes' worth of distinct data. The segment
geometry is identical across pairs, so they stack into one
`(n_pairs, seg, length)` call. That is the ~20 ms stage, and it is the only
genuinely redundant transform left.

*What is not:* `cross_correlate_fft` does **not** re-FFT anything.
`node_spectra` (supernode.py:376, tdoa.py:156) already caches one forward FFT
per node — that optimisation landed — and the per-pair cost is an integer-bin
roll, a multiply, and one irreducible ifft (tdoa.py:169). Batching the 21
iffts is worth a few ms of call overhead, not a transform count.

*What was wrong:* an earlier draft argued "three threads contending over 21
small calls is strictly worse than one batched array op." There is no such
contention. `_reaper` queues whole bursts (supernode.py:259) and each of the
three workers pops a whole burst (supernode.py:171, 316) — the threads
parallelise *bursts*, and because pocketfft releases the GIL they genuinely
scale across cores. The threading is a throughput win. The case for batching
rests on per-call overhead and vectorisation alone, which is a smaller and
more honest claim.

Note also that TODO.md:45 already records "batch the per-node segment FFTs if
that ever matters." This rung is that note with measurements attached, not a
new idea. Prove it with `time.perf_counter` histograms around the pair loop,
before/after, thermally soaked per the project notes.

**A2. Plan-cached FFTs — folded into A1, not a separate rung.**
`scipy.fft(workers=N)` is free (scipy is already a dependency) and worth
trying in the same sitting. pyFFTW is a new dependency for single-digit
milliseconds; do not add it.

**A3. cuFFT — only past a trigger.** The arrays are 512 kB (64k complex64);
PCIe transfer and launch latency eat most of cuFFT's advantage at batch 21.
The cupy path is not merely unused, it is unreachable: `use_gpu` defaults
False on `cross_correlate` (tdoa.py:88) and `caf_correlate` (tdoa.py:260),
**no call site anywhere passes True**, and the live correlator
`cross_correlate_fft` has no GPU parameter at all. The project notes' old claim that
`pip install cupy` makes tdoa.py use the GPU automatically was false and has
been corrected.

Trigger arithmetic, corrected: n_pairs > ~100 is **~15 nodes** at all-pairs
(n(n−1)/2), not 35 — the earlier figure assumed a fleet size rather than a
correlated-set size, and `solver.max_nodes` caps the correlated set at 7
today, so the trigger is a *config* change as much as a fleet change. Until
then, or until the 8x site-scan capture profile runs routinely, GPU is
ceremony. `docs/gpu_and_language_port.md` has the full measured argument.

**A4. Recorded non-goal: Rust/C++ rewrite.** The hot path is pocketfft + BLAS
already; a rewrite recovers glue overhead, not FLOPs, and moves no metric an
operator cares about (error, fix rate, latency budget). Revisit only if
(A1–A3 land and profiling still shows a Python-floor breach of the deadline)
or (deployment to constrained nodes demands a single static binary — an ops
argument, and a legitimate one, but say it plainly when it happens).

---

## Ladder B — spend the tracker's posterior on the front end

The tracker already knows roughly where the emitter is and how fast it moves.
The correlation front end ignores this on every burst and rediscovers it from
scratch.

**B1. Doppler prior from track velocity.** Per pair, the predicted differential
radial velocity bounds the CAF span far tighter than `caf_vmax_mps = 60`.
Innovation-gate it: use the tight span when track covariance is small, fall
back to the full span otherwise (and always for untracked bursts). Buys
cheaper estimates and fewer confident-wrong Doppler locks — the failure mode
`caf_correlate`'s docstring worries about, bias-before-null.

**Know the floor before promising a win.** `max_dop = fc·(2·vmax/c + 1.2e-7)`
(supernode.py:369). The second term is undisciplined-LO offset, ~292 Hz at
2.437 GHz, and no track predicts it. Under `free` or `holdover` it dominates
and the tightening is capped no matter how good the track is. This rung buys
a fraction of 20 ms under `gpsdo` and close to nothing without it.

**B2. Track-tightened lag windows.** Today `max_lag = baseline/c + 5e-6`
(supernode.py:362), sized for acquisition. Once a track is converged, the
predicted range-difference spread per pair is known to tens of metres;
shrinking the window shrinks the sidelobe-capture probability proportionally.
Same mechanism family as bug 13's clock-centred windows — the window is
already *centred* on the pairwise clock delta (supernode.py:401–417), so this
shrinks the half-width and extends that fix rather than replacing it.

Two hazards, both of which must be handled in the same change:

- **Track hijack.** A tight window centred by a wrong track finds only the
  peaks that confirm the wrong track, and the residuals look fine because the
  contradicting peak was never searched. Needs a periodic full-window burst,
  or a covariance/innovation-triggered reset to acquisition width — "once
  converged" is not a sufficient condition, because converged-and-wrong is
  exactly the state this creates.
- **`quality_min` silently recalibrates.** Quality is peak-over-floor computed
  across the *searched* window (tdoa.py:120–147). Shrink the window and the
  floor estimate changes, so the whole quality distribution shifts and the
  gate no longer means what it was tuned to mean. Same trap the project notes already
  document for GCC-weighted quality. Re-derive the gate, or the gate becomes
  a random number.

**B3. Top-K peak hypotheses.** Keep the best K=3 peaks per pair above the
quality gate, not one; enumerate combinations pruned by incremental χ²
(beam search), solve the survivors, keep the joint-best. This attacks the
multipath ghost-peak problem from the association side — TODO 5d's
first-peak-vs-max logic handles one reflector heuristically; this handles the
general case at bounded cost.

This is **not** a reintroduction of leading-edge peak detection: joint-χ²
across pairs is a consistency test, the opposite of a per-pair first-arrival
heuristic, and inconsistent per-node peak choices are precisely what it
rejects. But temper the expectation — TODO 5c's measured verdict is that
multipath is a bandwidth problem and super-resolution is not obviously worth
it. And resolve the earlier draft's contradiction: it said "gate on real
captures" and "validate through `validate_multipath.py`" in consecutive
sentences. Gate on real captures. Validating a multi-hypothesis *association*
scheme against the sim's own tap model is optimising against ourselves;
`validate_multipath.py` is a regression check afterwards, not the evidence.

**B4. Sequential pair selection with early exit — killed.** Kept here as a
recorded refusal, because it sounds efficient and is not. With `min_nodes` 5
and `max_nodes` 7 the trim space is at most a couple of pulls. Worse, it
attacks both outlier defences at once: Huber needs redundant peers to
identify a liar, and a covariance-trace stopping rule is blind to exactly the
bias-driven error bug 2 documents the covariance cannot see — it will stop
confidently on a wrong fix. If uplink is ever *measured* as binding on real
LTE, `ci2` is already built and is 4x smaller than `ci8`; that is the first
lever, not this.

---

## Ladder C — measurements already on the wire, unconsumed

**C1. Remote ID as an independent cross-check, not a solver row.**
`proto.detection_event` carries `rid` (proto.py:175) and it is the only
occurrence in the tree — nothing reads it **and nothing writes it**. The
alt_prior plumbing genuinely is drop-in (`solve_tdoa` tdoa.py:426–427,
`solve_tdoa_gls` tdoa.py:632–633, tracker track.py:52, GDOP seed
supernode.py:291) — mechanically a RID altitude would replace the 60 m soft
prior with a ~10 m constraint tomorrow.

Do not do it. Two reasons, and the first is the important one:

- **It inverts the design ethos.** This system exists for *non-cooperative*
  drones. A drone that broadcasts Remote ID is precisely the one whose claim
  you want to verify independently — and fusing that claim into the solve
  path destroys the independence the verification requires. It is the
  field-side twin of the sim/truth invariant: nothing in the solve path reads
  the answer. "The vertical stops being the weak axis" is misleading; the
  vertical becomes *the target's self-report*.
- **"Zero new hardware" is wrong.** RID rides Wi-Fi beacons and BT
  advertisements. There is no RID receiver in this codebase, so this needs a
  per-site sniffer (cheap, not zero) plus RID-track-to-RF-track association.

What to build instead: decode RID where available, display it beside the RF
fix, and score the disagreement. A cooperative aircraft whose broadcast
position disagrees with its RF fix is the single most interesting event this
system can produce, and fusing the two throws that signal away.

**C2. Spectrum frames as a weight-mode selector — killed, redundant.**
The claims check out: nodes publish a 256-byte PSD every burst at
DATA_LOW/DROP (node.py:585, 760) and `_on_spectrum` is explicitly display-only
(supernode.py:196–199). But the correlator does not need those frames. It
already computes a fresh full-rate Welch PSD per node per burst from the
pulled IQ (supernode.py:377) — strictly more information than 256 quantised
display bins — and `ht_weight` already self-neutralises, returning ones when
it finds no structure (tdoa.py:252). The per-burst adaptivity this rung wants
is already running. The console toggle it wants to retire exists for A/B
demonstration, not for operation.

---

## Ladder D — scale-out path (gated on fleet size, not ambition)

**D1. Shard by burst, not by pair.** The earlier draft had the sharding axis
backwards. Under all-pairs correlation, pairs share nodes — so a *shard of
pairs* needs the IQ of nearly every node in the burst, and pair-sharding
multiplies node uplink by the shard count. "Ships CorrResults (~100 bytes)
instead of IQ" describes the shard→central hop while hiding the node→shard
hop, and uplink (~10 Mbps LTE, ~90 ms of the ~180 ms budget) is the
bottleneck this document names elsewhere.

Shard by burst instead: a second supernode takes alternate bursts, pulls only
from the nodes it selected, and no IQ is duplicated. The existing three-worker
pool already proves burst-parallelism works; this is that, across boxes.
Trigger: per-burst wall clock breaching the deadline after A1–A3, or fix rate
capped by correlation rather than by uplink.

**D2. Deploy the QUIC/background link.** `zconf.spoke(…, background_only=…)`
(zconf.py:46) appends a `?prio=6-7` endpoint (zconf.py:58–59) so bulk IQ
cannot head-of-line block detection events; node.py:921 calls `spoke()`
without it. Written, undeployed — and it is exactly what the project notes' QoS
footnote prescribe as the fix for the dead `Query.reply` QoS arguments.
Measure HOL blocking on a real LTE backhaul before trusting TCP in the field:
the docstring's claim is plausible, unmeasured, and cheap to measure. Trigger
is field hardware, like almost everything worth doing here.

**D3. Multi-site fusion at the geodetic seam.** `geo.LocalFrame` round-trips
exist (geo.py:45); sites exchange tracks/detections (bytes), never IQ.
Anything more is a different project wearing this one's clothes.

---

## Ladder E — the rungs this document was missing

Added after review. Every one of these outranks all of Ladder A.

**E1. Capture and replay.** Nothing in this tree turns a real capture into a
regression test. TODO.md says in three different places that everything
remaining wants real captures; nothing builds the path to having them. Add a
record mode that dumps the node ring buffer to disk with its event metadata,
and an offline driver that replays real IQ through the correlator and the
validators. This single rung supplies the trigger data for B3, the cyclo
thresholds, the multipath peak logic and DPD — and it is buildable today,
with no radio, against the surveyed public datasets in E4.

**E2. A soak harness.** Bug 13 was invisible to a 30 s smoke test *by
construction* — it is a run-time-dependent failure, and a fresh demo looks
perfect. Nothing on either list institutionalises the hour-long run that
would have caught it: clock deltas, χ², window-overflow count and fix rate
trended over time, with a pass/fail on drift rather than on a snapshot. More
valuable than every rung of Ladder A combined.

**E3. Authenticate `/control`.** `GET /control?gate=0&robust=0&clock=free` on
an unauthenticated HTTP server switches off every outlier defence and
un-disciplines every node clock. The Zenoh bus has mTLS (zconf.py); the
console has nothing. Ladder D plans multi-box scale-out over field LTE while
the control plane is open. Fix before any deployment that is not a lab bench.

**E4. Dataset loaders for the surveyed public captures** (NIST mds2-2731,
DroneDetect V2 — TODO item 5's own recommendations). Real RF, no field trip,
and it calibrates the cyclostationary detector's constants, which are
synthetic-only today. Pairs directly with E1.

**E5. Pipeline the scheduled-capture loop.** Project-notes priority 3: the loop
schedules the next slot only after the current capture returns, so with
`schedule_lead_s` ≥ the period the effective rate drops. That is a known,
real throughput defect on the *hardware* path — more field-relevant than any
latency this document's Ladder A recovers, and it was absent from the list.

---

## Validation discipline (applies to every rung)

Each landed rung ships a before/after latency or error histogram from the
existing validators (`validate_math.py`, `validate_caf/ofdm/hopping/
multipath.py`), benchmarked thermally soaked and away from load, per the
thermal post-mortem in the project notes. A rung that cannot show its histogram did not land.

One addition, learned the expensive way: `validate_math.py` does not exercise
the GCC-weighted path at all — it never calls `cross_correlate_fft` with a
`weight`. `validate_ofdm.py` is the covering test for anything touching
weighting. Naming the wrong validator is how a change ships "verified" while
nothing checked it.
