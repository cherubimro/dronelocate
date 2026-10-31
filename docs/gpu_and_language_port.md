# GPU cards and language ports: an engineering assessment

Two hypothetical questions, each answered go/no-go with the conditions stated as
measurable thresholds. Everything after the verdicts exists to justify them.

---

## Verdict 1 — Should GPU acceleration of the supernode be pursued? **No — not now. Conditionally yes later, and the condition is measurable.**

**No today** because the thing a GPU improves — per-burst correlation compute,
~70 ms of a ~100 ms solve at the shipped 2.4 Msps configuration [measured] — is
neither the accuracy constraint nor the latency constraint nor the throughput
constraint:

- Accuracy is floored by clock jitter: 15 ns against ~2.84 ns of correlation
  error — the error a GPU could shave is already **5× smaller** than the error
  the clock injects [measured].
- Latency is dominated by policy and I/O: 400 ms grouping window + ~90 ms
  *serial* IQ pull + ~100 ms solve. The GPU removes ≤ 70 ms of ~590 ms — ~12% —
  while the window alone is 12.4 m of target motion at 31 m/s [computed].
- Throughput has measured headroom at the operating point (2 bursts/s, 3
  workers) [measured], and per-burst compute is **constant in fleet size** under
  the shipped `max_nodes` cap (§3) — more sensors do not change this verdict.

**Yes, when either of two measurable conditions arrives** — and then as a
batched complex64 rewrite of one block of `tdoa.py` (§1.1 shows the "install
CuPy" path is fiction), on current-generation silicon:

1. **Wideband operation**: per-burst correlate+solve exceeding ~250 ms per
   worker at the target burst rate — which the 20 Msps B210 profile is projected
   to hit (~0.6–0.9 s/burst/core [computed]) — *and* experiment 2 in §8 shows
   the batched GPU block beating the CPU by ≥ 3× end-to-end at 512k/complex64.
2. **DPD is built** (TODO item 6): grid-search × coherent sums is the most
   GPU-shaped computation this project will ever contain. Build that on a GPU
   from day one.

Buying a high-FP64 card (Titan V-class, 1:2 FP64) specifically is a **no in all
branches**: the only FP64 in the hot path is an accidental complex64×float64
promotion (§1.2) fixed by one cast, after which FP64 throughput is irrelevant to
this workload — and a naive port that keeps the accident would be crippled on an
FP64-poor (1:24) card anyway. Fix the dtype; do not buy silicon to forgive it.

## Verdict 2 — Should a rewrite in Rust, C, or Ada/SPARK be pursued? **No, for all three — a rewrite does not address what limits scaling.**

The premise "Python is the scaling bottleneck" is contradicted by this project's
own measurements: the GIL holds 1.02% of a burst at 2.4 Msps and 0.11% at
20 Msps (Amdahl ceilings 98× and 952×), and the observed 1.72× process-pool
ceiling on 4 cores was memory bandwidth and package power — a wall no language
moves [measured]. The hot loops already execute in C/C++ (pocketfft, OpenBLAS).
A full port's realistic same-hardware gain is ≤ 2–3× (§5), for weeks-to-months
of effort and the risk of re-introducing any of the 13+ subtle numerical bugs
this codebase has already paid to find. Scaling to many sensors is a
burst-throughput problem (§3), and burst throughput scales across processes and
machines in Python today.

Conditional carve-outs, each with its threshold:

- **Hybrid (PyO3/Cython correlator extension): no until** the cheap pure-NumPy
  steps (§4 rows 0/0b/b: dtype fix, parallel pull, cross-burst process pool,
  batched FFT calls) have been done *and* profiles still attribute ≥ ~20% of a
  burst to interpreter/allocation glue. The measured 1% GIL fraction predicts
  this threshold is never reached.
- **C: no** in every branch — same ceiling as Rust, worse safety, and the
  hazard profile (sign/index arithmetic) is exactly this project's bug history.
- **Ada/SPARK: no for the correlator, unconditionally** — SPARK proves absence
  of runtime error, not numerical correctness, and its numerics/FFT story is C
  bindings that put the proof boundary around unproven code (§5). **Yes for one
  future component**: if the system ever gains an authority-bearing interlock
  (engagement gate, geofenced effector), that small, stable, high-consequence
  state machine is where SPARK is the right tool.

**What to do instead** (highest value per effort, §4): cast the GCC weight to
float32 (removes an accidental complex128 from the dominant stage), parallelize
the IQ pull (~90 → ~15–30 ms), prefer `ci2` on constrained uplinks (at 20 Msps
the wire dominates all compute options), and scale bursts across processes and,
if needed, supernodes sharded by geography.

---

Every number below is tagged: **[measured]** = an established measurement from this
project (the project notes, TODO.md, or the profiling that produced the GIL/Amdahl figures);
**[computed]** = arithmetic from the source code and measured unit costs;
**[doc]** = from vendor/library documentation; **[estimate]** = engineering estimate,
stated as such.

---

## 1. What the supernode actually computes, per the source

Read from `supernode.py` and `dronelocate/tdoa.py` (current tree, 2026-08).

Per burst, with defaults (`all_pairs: true`, `gcc: "ht"`, `caf: true`,
`max_nodes: 7` → K = 7 nodes, m = K(K−1)/2 = 21 pairs; 2.4 Msps, 10 ms snippet →
24,000 samples, `nfft` = 65,536):

| Stage | Where | Count | Measured cost [measured] |
|---|---|---|---|
| Serial IQ pull (`_pull`, one `session.get` per node, **sequential**) | `supernode._process` | 7 | ~90 ms on the reference container; network-dependent |
| Dequantize ci8 → complex64 | `sigsim.dequantize` | 7 | small |
| Per-node forward FFT (`node_spectra`, complex64 in) | `tdoa.py` | 7 | 10 ms |
| Per-node Welch PSD (`welch_psd`, 32 segs, nfft 1024) | `tdoa.py` | 7 | 2 ms |
| Per-pair HT weight (`ht_weight`, 1024 bins) | `tdoa.py` | 21 | 4 ms |
| Per-pair Doppler estimate (`estimate_doppler`, ~29 segs × 1024-pt batched FFTs + cross-segment FFT) | `tdoa.py` | 21 | 20 ms |
| Per-pair weighted correlation (spectrum multiply + weight + 64k IFFT + windowed peak + sinc refinement) | `cross_correlate_fft` / `_search_peak` / `_subsample_peak` | 21 | 33 ms |
| `pair_covariance` + GLS solve (grid seed 21×21×9 = 3,969 points, Levenberg GN ≤ 40 iters) + tracker + glue | `tdoa.py`, `track.py` | 1 | ~31 ms (the 100 ms total minus the above) [computed] |

Concurrency: three worker **threads** consuming a `queue.Queue` of bursts
(`supernode.__init__`), so bursts overlap; within a burst everything is serial.
Established scaling on a 4-core laptop-class machine: thread pool 1.45×, process
pool 1.72×, concluded memory-bandwidth- and power-limited, not GIL-limited
(GIL-held fraction of a burst: 1.02% at 2.4 Msps, 0.11% at 20 Msps; Amdahl ceilings
98× and 952×) [measured]. NumPy releases the GIL inside FFTs.

### 1.1 Correction of record: the GPU path is not wired

The project notes say "`pip install cupy` and `tdoa.py` uses the GPU automatically."
**That is false.** Verified against the source:

- `use_gpu` exists only on `cross_correlate()` and `caf_correlate()`
  (`tdoa.py:88,251`), defaulting `False`. A repo-wide grep finds **zero call sites
  that pass `use_gpu=True`** — not in `supernode.py`, not in any validate script.
- The hot path never reaches those functions. `supernode._process` builds a
  `correlate()` closure that calls `estimate_doppler` + `cross_correlate_fft`
  (`supernode.py:380–394`) — and `cross_correlate_fft`, `node_spectra`,
  `estimate_doppler`, `welch_psd`, `ht_weight`, `pair_covariance`,
  `solve_tdoa_gls` **have no GPU parameter at all**. This is true for both the
  all-pairs branch and the star-reference branch; both go through the same closure.
- `gpu_available()` (`tdoa.py:792`) reports only whether CuPy *imports*. It feeds
  the startup banner and the console stats (`supernode.py:644,901`). So the console
  can say "GPU yes" while not one FLOP runs on the device.

README's phrasing ("GPU path is stubbed") is half right — "stubbed" is accurate,
"`tdoa.py` will use it" is not. The `requirements.txt` comment ("optional:
cupy-cuda12x (GPU correlation)") is misleading the same way. **"Enable the GPU" is
not an install; it is a batched rewrite of the correlation block** — realistically
days of work (batch the 21 pairs into one `(21, nfft)` array, batched FFT/IFFT,
per-row windowed argmax, keep sinc refinement and solver on the host), plus the
dtype fix below, plus removing the per-call sync points (`int(xp.argmax(...))` per
pair) the vestigial path contains.

### 1.2 Dtype audit: where float64/complex128 actually lives

This matters because it decides whether GPU FP64 throughput is relevant at all.

- **IQ and FFTs are complex64 by design.** `node_spectra` and `cross_correlate`
  cast to `complex64`, and NumPy ≥ 2.0 preserves single precision through
  `np.fft` [doc] — so the forward FFTs run and store in complex64
  (64k × 8 B = 512 KB per node spectrum).
- **The default hot path silently promotes to complex128.** `ht_weight` returns a
  float64 array. In `cross_correlate_fft`,
  `spec.reshape(len(weight), -1) * weight[:, None]` multiplies complex64 by
  float64, which **promotes to complex128** under NumPy's rules [doc]. With the
  default `gcc: "ht"`, therefore, the 21 IFFTs — the single largest stage, 33 ms
  [measured] — run in complex128: double the memory traffic and double the
  transfer size of what the algorithm needs. One `astype(np.float32)` on the
  weight (or building it float32) removes every complex128 from the correlation
  hot path. Nothing in the algorithm needs double precision there: the peak is
  refined on the host in float64 over ~49 samples (`_subsample_peak`), which is
  where the precision actually pays.
- float64 legitimately remains in: the solver/GLS/covariance (`solve_tdoa_gls`,
  `pair_covariance`), the tracker, `geo.py` — all tiny at m = 21 and correctly
  double (position arithmetic and epoch-relative times want it; see bug 10).
  `sigsim` uses complex128 in the emitter — node-side, sim-only.

**Consequences for the two GPU classes on hand:**

- **Older consumer (Kepler-class GeForce) cards: FP64 at 1/24 of FP32 rate** [doc]
  (compute-class boards of that generation were 1/3). A *naive* CuPy port of
  today's code — which would faithfully reproduce the complex128 promotion,
  since CuPy follows NumPy's promotion rules — lands its dominant kernel on
  exactly the operation such a card is worst at, and can plausibly lose to the
  host CPU. This is a trap: invisible on CPU, where FP64 costs ~2×, disqualifying
  on a 1:24 card.
- **Titan V-class (Volta GV100): FP64 at 1/2 of FP32** (~6.9–7.4 TFLOPS FP64,
  ~650 GB/s HBM2) [doc]. That ratio is the one property that could make it a
  *different* answer rather than a faster one — but only for code that keeps
  complex128 in the hot path. After the one-line dtype fix, the workload is
  complex64 end to end on the device, and the FP64 advantage buys nothing this
  code needs. What would remain valuable is the memory bandwidth — and current-
  generation mid-range cards match or beat it there, with warranty, half the
  power (a used GV100 board is ~2018 silicon at 250 W, no warranty, and at the
  end of its driver/toolkit support window). **Do not buy a high-FP64 card to
  accommodate a dtype accident; fix the dtype.**

---

## 2. The scaling law: what "many sensors" actually costs

From the source: N nodes hear a burst → `_reaper` groups events (O(N) dict ops,
trivial); `_select` runs greedy PDOP over the hearers, capped at
`solver.max_nodes` (7); IQ is pulled from **only the chosen K ≤ 7**; correlation
runs over m = K(K−1)/2 pairs; the GLS builds and factors an m × m covariance.

So the quadratic term is in **pulled** nodes, not fleet nodes. Under the shipped
policy, per-burst compute is *constant in N*. What grows with fleet size:

| Component | Order | At N = 64 [computed] |
|---|---|---|
| Event ingest, grouping, health, spectra deques | O(N) | ~128 events/s at 2 bursts/s; negligible |
| Greedy PDOP selection (`_select`): (cap−1) rounds × N candidates, each a small `gdop()` | O(cap·N) | ~450 gdop calls/burst; ~10–30 ms, Python-overhead-bound [estimate]; vectorizable if it ever matters |
| IQ pull, correlation, solve | O(1) — capped | unchanged, ~100 ms |
| SSE payload (full health dict per message) | O(N) | tens of KB/message; LAN-trivial |

### Per-burst cost, capped vs uncapped

Unit costs derived from the measured budget [computed]: per node
1.43 ms FFT + 0.29 ms PSD = **1.71 ms**; per pair 0.95 ms Doppler + 0.19 ms HT +
1.57 ms weighted correlation = **2.71 ms**. Solver terms: grid-seed whitening
≈ 2·3969·m² FLOP (fp64 GEMM), R inversion + Cholesky ≈ 3·m³ FLOP, plus ≤ 40 GN
iterations of O(m²). Single laptop-class core, 2.4 Msps:

| N nodes | Capped (max_nodes 7) | Uncapped: m pairs | Per-node stages | Per-pair stages | GLS + seed | Uncapped total | Uncapped seed memory |
|---|---|---|---|---|---|---|---|
| 10 | ~100 ms | 45 | 17 ms | 122 ms | ~5 ms | **~0.15 s** | 4 MB |
| 20 | ~100 ms | 190 | 34 ms | 515 ms | ~20–60 ms | **~0.6 s** | 18 MB |
| 32 | ~100 ms | 496 | 55 ms | 1.34 s | ~0.15–0.5 s | **~1.6–2 s** | 47 MB |
| 64 | ~100 ms | 2,016 | 109 ms | 5.5 s | ~2–6 s (seed GEMM 32 GFLOP + R ops 25 GFLOP fp64) | **~8–12 s** | ~0.5 GB peak (G×m×3 float64 arrays in `solve_tdoa_gls`'s seed) |

All uncapped figures [computed]/[estimate] from measured unit costs; the GLS
column spans a range because fp64 GEMM throughput per core varies ~4× across
machine classes.

Three observations that matter more than the table:

1. **The O(N²) never materializes under the shipped policy**, and the project's
   own geometry measurements say uncapping is pointless: best-7 PDOP 11.08 vs
   all-10 10.85, HDOP 0.83 vs 0.62 [measured]. Timing precision improves at best
   ~√(m) with more pairs, and the clock-jitter floor (§6) is reached long before
   496 pairs pay for themselves. If robustness ever argues for more diversity,
   `max_nodes` 8–10 (28–45 pairs) is the sane ceiling — a 1.3–2.2× compute bump,
   not a scaling problem.
2. **The axis that actually grows is bursts/second** — more emitters, more area,
   higher fix rate. A 64-node fleet covering a city partitions naturally: each
   burst is heard by the nodes near it, grouped by burst id, solved as an
   independent ~100 ms task. Bursts are embarrassingly parallel across worker
   processes and even across supernodes sharded by geography (the Zenoh key
   space already namespaces by site). **Scaling to many sensors is a
   burst-throughput problem, and burst throughput is a process-pool problem, not
   a single-solve-speed problem.**
3. If someone *does* uncap: the memory blow-up in `solve_tdoa_gls`'s vectorized
   grid seed (G×m×3 float64 intermediates, ~0.5 GB at m = 2016) arrives before
   the FLOPs do. The seed would need chunking; the R handling (`inv` **and**
   `cholesky` of the same matrix — it currently computes both) would want a
   single factorization. These are notes for that future, not present defects.

---

## 3. Q1: does the workload shape suit a GPU?

Per-stage analysis at the default 2.4 Msps (nfft 64k) and the B210 profile's
20 Msps (200k samples, nfft 524,288 — `config/site-b210.json`). Arithmetic
intensity computed as FFT FLOPs ≈ 5·n·log₂n over bytes touched [computed].

| Stage | Shape per burst | FLOPs | Bytes (c64) | AI (FLOP/B) | Batchable? | GPU verdict |
|---|---|---|---|---|---|---|
| Node FFTs | 7 × 64k (7 × 512k) | 7 × 5.2 M (7 × 50 M) | 7 × 1 MB (× 8 MB) | ~5 | yes, one batched call | Wins only as part of a whole-block offload; alone, launch+transfer eats it |
| Welch PSD | 7 × 32×1k FFT | ~1 M | ~2 MB | ~1 | yes | Too small to bother; keep on host or fold into block |
| HT weights | 21 × 1k elementwise | ~0.1 M | ~0.5 MB | <1 | yes | Host. Negligible either way |
| Doppler estimates | 21 × (3 batched FFT sets ~29×1024, + 128-pt cross-seg FFT) | 21 × ~4.5 M | ~40 MB | ~2 | **yes — identical shapes across pairs within a burst** (same max_lag ⇒ same seg count/nfft; only the per-pair roll offset differs, a cheap kernel) | Good GPU (or batched-CPU) candidate. Currently 20 ms serial [measured] |
| Weighted correlations | 21 × (64k multiply+weight+IFFT+windowed peak) | 21 × ~5.5 M | ~30 MB (60 MB as accidental c128) | ~4 | yes: one (21, nfft) batched IFFT + per-row gather/argmax | The prize: 33 ms [measured], the largest stage |
| Sinc refinement | 21 × (65×49 matmul on host) | ~0.1 M | 8 KB device→host | — | already tiny | Host, as the existing code correctly does |
| GLS + covariance | m=21 rows, fp64, branchy | ~10 M | small | — | no (iterative, data-dependent) | Host. Only GPU-worthy at m ≥ ~500, which shouldn't exist |
| Kalman / tight coupling | 6-state | ~µs | — | — | — | Host |

**Transfers and latency.** Input per burst: 7 × 24,000 × 8 B = **1.3 MB** at
2.4 Msps; 7 × 200,000 × 8 B = **11 MB** at 20 Msps. Over PCIe 3.0 x16
(~12 GB/s practical [doc]): 0.1 ms and ~1–2 ms. Output: kilobytes (peak
neighborhoods + scalars). Kernel launches for a properly batched block: ~30–60
per burst at ~5–20 µs each [doc] ≈ 0.3–1 ms. **Conclusion: at these sizes the
offload is not PCIe-dominated and not launch-dominated — provided it is shipped
as one block per burst.** Offloading per-pair, per-call (what the vestigial
`use_gpu` path would do) multiplies round trips and syncs and forfeits most of
the win, though even that would likely still beat 70 ms of CPU at 64k sizes.

The round trip **improves** latency rather than costing it: the correlation
block drops from ~70 ms to a device time bounded by memory bandwidth —
~28 large FFTs × ~3 passes over 0.5–4 MB each ≈ 0.1–0.7 GB of device traffic,
i.e. **~2–5 ms on a 250 GB/s-class older card, ~1–2 ms on a modern mid-range
one** [computed, bandwidth model], plus ~1–2 ms PCIe. Per burst: ~100 ms →
~35–40 ms (the solver/glue ~31 ms becomes the floor — Amdahl, again).

**And that is precisely why it is not worth doing now.** The end-to-end latency
budget at 2.4 Msps is ~400 ms grouping window + ~90 ms serial pull + ~100 ms
solve ≈ 590 ms [measured]; the GPU removes ≤ 70 ms of it (~12%), while the
window alone is 12.4 m of target motion at 31 m/s [computed]. Throughput has
"ample headroom" at 2 bursts/s across 3 workers [measured]. A GPU here solves a
problem the system does not have.

**Where the GPU case becomes real** — two regimes, both visible in the repo:

1. **20 Msps (B210 profile).** Scaling the measured unit costs by ~9.5× (8× points
   × log factor): node FFTs ~95 ms, correlations ~315 ms, Doppler ~160 ms —
   **~0.6–0.9 s per burst per core** [computed/estimate]. That no longer fits
   2 bursts/s × 3 workers on a 4-core box. A batched complex64 GPU port brings the
   block to ~5–20 ms + transfers even on an older card [computed] — a 30–100×
   stage win, turning the solve back into a solver-bound ~50 ms. Note, however,
   what else breaks first at 20 Msps: **the pull.** 400 KB/node ci8 over a
   10 Mbps LTE uplink is ~320 ms per node — 2.3 s if pulled serially as the code
   does today, ~320 ms parallelized, ~80 ms with `ci2` [computed]. The README's
   line "on a constrained uplink `ci2` is worth more than a GPU" is verified and,
   at B210 rates, understated.
2. **DPD** (TODO item 6): grid search over position × coherent sums across pairs
   — high arithmetic intensity, massive uniform batches, no branching. That is
   the most GPU-shaped computation this project will ever contain, and the TODO
   already defers it until real captures exist. If DPD is built, build it on the
   GPU from day one.

**Two cards vs one.** Nothing in the codebase is multi-GPU aware. The cheap
mapping is one worker (burst) per device — reasonable, but only pays once burst
throughput is actually the bottleneck; also beware that second PCIe slots on
4-core desktops frequently run at x4. Two older-generation cards do not combine
into one faster correlator.

---

## 4. The honest alternatives, compared

The comparison the decision actually needs — per unit of effort and risk, on the
hardware classes available (4-core workstation with two older CUDA cards;
option on a high-FP64 GPU; a 12-vCPU VM; hypothetically a many-core Xeon):

| Option | Expected gain | Effort | Risk | Latency effect |
|---|---|---|---|---|
| (0) Dtype hygiene: float32 GCC weight (kills the complex128 promotion) | ~1.3–2× on the largest stage; halves its memory traffic [computed] | one line + revalidation | minimal (weight is a shaping function; revalidate with `validate_ofdm.py`) | small direct; enables everything below |
| (0b) Parallel IQ pull (concurrent `get`s) | none on compute | small | low | **pull ÷ K: ~90 → ~15–30 ms at 2.4 Msps; the single largest latency win available at 20 Msps over LTE** [computed] |
| (a) More CPU cores (Xeon-class, more memory channels) | near-linear across bursts until bandwidth; the measured 1.72×-on-4-cores ceiling was the *machine* (bandwidth/power), not Python [measured] | zero code | zero | none per burst; throughput only |
| (b) Process pool with per-burst batching (and batched `scipy.fft`, `workers=`, within a burst) | ~2–3× on existing boxes [estimate] | days | low | modest |
| (c) GPU, batched correlator block | ~10–30× on ~60–70% of per-burst compute ⇒ ~2.5–3× per burst (Amdahl: solver/glue floor); decisive at 20 Msps | days of rewrite (§1.1) + (0) first | medium (new failure modes, device memory mgmt) | −60 ms at 2.4 Msps (~12% of end-to-end); large at 20 Msps |
| (d) Language port (full) | ≤ 2–3× same hardware (§5) | weeks–months | high (re-validating 15 fixed bugs' worth of subtle correctness) | similar to (b)+(0) |

Ranking per effort/risk: **(0) and (0b) first, then (a)/(b) as scale demands,
(c) when the 20 Msps profile or DPD becomes operational, (d) not on performance
grounds at all.**

---

## 5. Q2: the language ports, taken seriously

The premise to state before each option: profiling already established that the
interpreter is ~1% of a burst [measured], NumPy's FFTs release the GIL, and the
hot loops execute inside pocketfft (C/C++) and OpenBLAS. **A rewrite therefore
cannot make the FLOPs much faster; it can only make the glue, the memory
behavior, and the parallelism better.** The measured 1.45×/1.72×
thread/process-pool results on 4 cores were bandwidth/power-limited — a wall a
rewrite does not move.

### Rust

- **Compute:** `rustfft` is mature, SIMD-dispatched (AVX2/NEON), and competitive
  with FFTW-class C for power-of-two complex transforms [doc — rustfft's own
  published benchmarks; treat exact ratios as workload-dependent]. `ndarray` +
  `nalgebra` cover the solver comfortably. Net FFT speed vs pocketfft: parity to
  ~1.5× either way — not the payoff.
- **Where it genuinely wins over a Python process pool:** shared-memory
  parallelism *within* a burst. `rayon` over pairs shares the node spectra
  read-only with zero copies; a Python process pool must ship ~0.5 MB (64k) to
  8 MB (512k at 20 Msps) of spectra per task or plumb shared memory by hand.
  Also: fused multiply-weight-IFFT passes (one traversal instead of three
  allocations), allocator control, no pool warm-up inside the 400 ms window.
  Note the honest ceiling: within-burst parallel span is ~70 of 100 ms; with the
  ~31 ms serial solver/glue, 4 cores cap at ~2× per burst [computed]. Over 496
  pairs rayon would shine — but 496 pairs is the policy this report recommends
  never enabling.
- **Verdict:** the only port worth discussing, and then only in hybrid form (below).

### C

FFTW (with plans and wisdom, threaded), BLAS/LAPACK, OpenMP. Everything said for
Rust applies with a weaker safety story and the same conclusion: NumPy is already
calling this class of library; a C rewrite buys fusion + threading + no glue,
i.e. the same ≤ 2–3×, at higher defect risk in exactly the kind of index/sign
arithmetic this project's bug list (13 entries of it) shows to be the hazard.
No reason to prefer C over Rust unless team skill dictates.

### Ada/SPARK

What SPARK actually proves: absence of runtime errors (overflow, range and index
violations, uninitialized reads, division by zero), contract adherence, and
freedom from data races in tasking. What it does **not** prove: that an FFT is
numerically correct, that a sinc interpolator is unbiased, that a Kalman filter
is stable — floating-point proof support exists but bounding the numerics of a
correlator through it is research-grade work, not engineering. The numerics
ecosystem is thin: `Ada.Numerics.Generic_Real_Arrays` and BLAS/LAPACK/FFTW via C
bindings — at which point the proof boundary stops at the binding and you are
proving the glue around an unproven core, which inverts the risk profile of this
codebase (the bugs that hurt here were algorithmic and numerical, exactly the
part SPARK cannot see). Add the iteration cost: this correlator's algorithms
changed weekly through 2026-08 (CAF, GCC-HT, channelizer, cyclo-detection all
landed within a month per the git history); freezing that churn into a proved
language is the wrong trade at this maturity.

**Where Ada/SPARK would be the right tool:** if this system ever grows a
safety- or authority-bearing component — an engagement/weapons-release
interlock, a geofenced effector gate, an airspace-deconfliction message path —
that component is small, stable, high-consequence, and exactly SPARK's home
turf, and a certification argument would value the proof. Keep such a component
architecturally separate from the DSP either way. For the correlator: wrong tool,
said plainly.

### The hybrid: keep Python orchestration, move the correlator to a native extension

PyO3 (or Cython) around one function: "given K spectra and pair metadata, return
lags/qualities/Dopplers." Captures the fused-loop and shared-memory-thread
benefits — most of a full port's realistic speedup — for perhaps 5–10% of its
cost, and leaves Zenoh, HTTP/SSE, config, and the solver in Python where they
iterate fastest. Honest expected gain: ~1.5–2.5× per burst [estimate]. **But the
pure-NumPy steps in §4 rows (0)/(0b)/(b) reach a similar place without a build
system**, so the hybrid is justified only if, after those, profiles still show
allocation/glue overhead ≥ ~20% of a burst — which the measured GIL/overhead
numbers suggest it will not.

### Deployment argument, checked

The static-binary argument applies to the **node**, not the supernode — confirmed
by `debian-server.txt`: nodes run on Raspberry Pis (venv + PEP 668 friction, the
`--system-site-packages` requirement for apt's `python3-uhd` to be visible), the
supernode on a Debian server where a venv is a solved problem. Two caveats that
weaken even the node-side argument: (1) a "static" node binary cannot absorb
UHD/SoapySDR — C++ shared libraries plus a firmware/FPGA images directory that
ships separately — so deployment is never truly single-file for radio nodes, and
Rust's UHD binding ecosystem is immature [doc/estimate]; (2) the node's compute
(energy gate, cyclo-detect autocorrelation, quantize, ring buffer) is light
enough that a Pi runs it in Python today. A Rust node is an ops nicety, not a
performance need, and it would fork the "identical code for emulated and real"
property `node.py` deliberately maintains.

---

## 6. What actually limits this system, in rank order

Verified against source and configs, at the current 10-node / 2.4 Msps scale:

1. **Clock/PPS jitter.** `pps_jitter_s: 1.5e-8` in every config; nodes report
   `clock_sigma_ns` 15. Correlation error is ~2.84 ns [measured] against a
   1.63 ns interpolator floor at 2.4 Msps [computed from `interpolator_floor`].
   15 ns ≈ 5× the correlation error — `debian-server.txt` states it outright:
   "widening bandwidth buys almost nothing until the clock is fixed." Multiplied
   by VDOP ≈ 16 on the vertical axis. **No compute purchase moves this.**
2. **The unrun hardware gate.** `hw_selftest.py` against a real TinyB210 —
   < 50 ns schedule jitter viable, > 200 ns unusable for TDOA [measured
   criteria, unrun]. This decides the project and costs nothing but bench time.
3. **Node survey error.** 3–5 m from a single GNSS fix, propagating 1:1 into
   every fix while correlating perfectly. RTK or 24 h survey-in.
4. **Geometry.** VDOP ≈ 16 with near-ground nodes; vertical ≈ 10× horizontal by
   construction.
5. **Latency budget.** 400 ms grouping window (= 12.4 m at 31 m/s) + ~90 ms
   *serial* pull + ~100 ms solve. The two biggest levers are the window policy
   and parallelizing the pull — neither is compute.
6. **Uplink.** ~10 Mbps/node; at 20 Msps the wire format dominates everything
   (§3). `ci2` beats any processor purchase on this axis [measured/computed].
7. **Compute.** Last at 10 nodes / 2.4 Msps ("ample headroom" [measured]).
   At 64 nodes under the cap: still last — only O(N) selection/ingest grows.
   At 20 Msps, or with several simultaneous emitters, it climbs to roughly
   fourth, behind clock, survey, and uplink/pull — and §3–4 list its remedies in
   order: dtype fix, batching/process pool, then GPU.

---

## 7. Benchmarking in a VM with a masked CPU model — a warning that generalizes

One available test platform is a 12-vCPU KVM/QEMU guest whose CPU model is the
hypervisor's generic masked model rather than the host CPU. The specifics
generalize to any VM used for performance work on a NumPy-dominated code:

- **What a baseline-only vCPU costs.** NumPy's manylinux wheels compile with a
  runtime SIMD dispatch (baseline SSE/SSE2/SSE3; AVX2/AVX-512 paths selected at
  import by CPUID) [doc]. A masked CPU model hides AVX/AVX2/FMA (often
  unconditionally — without XSAVE the guest cannot enable AVX at all), so every
  *dispatched* kernel — the elementwise ufuncs behind derotation, magnitude,
  weighting, clipping, quantization — silently drops to its SSE2 path: a
  plausible 2–8× loss on those stages [estimate]. OpenBLAS (bundled in NumPy
  wheels, DYNAMIC_ARCH) likewise selects kernels by CPUID and falls back to
  generic ones — which hits the GLS at large m and any BLAS-heavy experiment.
- **The FFT is the different case, and it matters because the budget is
  FFT-dominated.** pocketfft's vectorization (both the numpy.fft and scipy.fft
  builds in manylinux wheels) is *compile-time*, built at the wheel's baseline —
  not runtime-dispatched [doc/source-level; verify with the canary below]. So
  wheel-installed FFTs run baseline-width code on every machine, and a masked VM
  loses comparatively little on the FFT stages specifically. Net effect: a VM
  benchmark of this workload is *unevenly* pessimistic — ufunc stages crushed,
  FFT stages roughly representative — which **distorts the stage mix and
  therefore the apparent scaling behavior**, not just the absolute numbers. An
  SSE2-starved core looks compute-bound and scales flatteringly well across
  workers; the same code on real AVX2 silicon is nearer the memory-bandwidth
  wall and stops scaling earlier. A "Python scales to 12 cores fine" conclusion
  from such a VM is a measurement of a masked feature set, not of Python.
- **This is almost certainly a free fix.** Host passthrough
  (`<cpu mode='host-passthrough'/>` in libvirt, `-cpu host` in QEMU) exposes the
  host ISA. Verify inside the guest afterwards (`lscpu` flags: avx2, fma;
  NumPy's `np.show_runtime()` "found" list) before trusting any number.
- **Honest VM benchmarking rules,** given that a guest typically has no PMU
  (no `perf`), no thermal sensors, and no view of host throttling — the same
  trap as the project notes' thermal-throttling section in a new form, except the VM
  looks cool and idle *by construction*:
  1. Fix the CPU model first; confirm the flags.
  2. Watch **steal time** (`%st` in `vmstat`/`top`) — the one signal of host
     oversubscription available. Discard any run with sustained steal above
     ~1–2%; 12 vCPUs are a promise, not 12 cores.
  3. Run a fixed canary (e.g., a single-core 64k-FFT loop) before and after
     every measurement; if the canary drifts, the host changed, not the code —
     the same discipline the project notes already mandate for thermal drift.
  4. Use the VM for **scaling shape** (speedup vs worker count), never for
     absolute numbers, and only post-passthrough.
- **Where such a VM fits:** if it is the largest core count available, it is the
  right place for the process-pool scaling experiment (§8, experiment 3) — after
  the CPU model fix, with steal monitored, results read as shape only.

---

## 8. Recommendation and test plan

**Recommendation.** Buy nothing and port nothing yet. The binding constraints
are clock hardware (unmeasured), survey, and latency policy — not compute. Do
the dtype fix and the parallel pull when next touching the code; keep
`max_nodes` capped and scale across bursts. Revisit the GPU exactly when the
20 Msps profile or DPD becomes operational — and then as a batched complex64
block port, on current-generation silicon, not on a high-FP64 card bought to
forgive a promotion accident. Decline the full rewrite; keep the hybrid
native-correlator option in reserve behind a profiling threshold. Reserve
Ada/SPARK for a future authority-bearing interlock, where it is the right tool.

**Experiments, generic, each with a decision threshold.** Ordered by cost;
1–3 fit in an afternoon on hardware already owned.

1. **Establish the 20 Msps baseline (decides whether compute is a problem at
   all).** Run the sim fleet with the B210 profile (`config/site-b210.json`,
   512k FFTs) on the intended supernode-class machine; read the per-stage and
   per-burst numbers the supernode already prints. *Threshold:* if
   correlate+solve stays under ~250 ms/burst/worker at the target burst rate,
   compute is not the constraint at B210 rates either — stop here and file the
   GPU under DPD.
2. **The one-hour GPU go/no-go (on any working CUDA box, older cards
   included).** Benchmark the correlation block's shape in CuPy vs NumPy on the
   same machine: batch of 7 forward + 21 inverse FFTs at 64k *and* 512k, in
   complex64 *and* complex128, host→device and device→host transfers included,
   plus the elementwise multiply-weight pass. This costs a page of throwaway
   script outside the repo and answers three questions at once: whether the
   block wins end-to-end at each size, how badly the FP64 cliff punishes the
   complex128 variant on an FP64-poor card (expect roughly an order of
   magnitude on that stage — seeing it once inoculates against the naive port),
   and what the real transfer/launch overhead is on that box. *Threshold:* GPU
   end-to-end < ⅓ of CPU at the 512k complex64 size ⇒ a batched port of the
   block pays at B210 rates; anything less ⇒ the GPU waits for DPD.
3. **Process-pool scaling shape (on the largest honest core count available —
   a VM only per §7's rules).** Fixed 20 Msps burst workload, burst-parallel
   workers 1→12, throughput and per-burst latency vs worker count, canary
   before/after, steal logged. *Threshold:* ≥ 8× at 12 workers ⇒ a mid-range
   many-core CPU covers 64-node fleets under the capped policy with no code
   beyond the pool — the boring Xeon answer wins. Plateau ≤ 4× ⇒ memory
   bandwidth is the wall; the GPU (or the hybrid correlator with shared-memory
   threads) moves up the priority list.
4. **Latency, not throughput.** Measure end-to-end burst-to-fix latency with the
   pull serialized (as shipped) vs parallelized, at 2.4 and 20 Msps wire rates,
   including a constrained-uplink emulation at ~10 Mbps/node. *Expected:* pull
   parallelization and `ci2` dominate every compute option at 20 Msps; if they
   do, that ordering — wire format, pull concurrency, then compute — is the
   scaling roadmap, and it contains no GPU and no rewrite in its first two
   entries.
5. **Only if (2) passed and (1) failed:** implement the batched GPU correlator
   block (complex64, one round trip per burst, sinc refinement and solver on
   host) and A/B it live with the console's existing toggles and metrics.
   *Threshold to keep it:* per-burst solve ≤ 40% of CPU baseline at equal
   accuracy (χ², containment unchanged) — accuracy regression here means a
   dtype or sync bug, not a tuning problem.

---

## 9. Documentation corrections this review surfaced

- **Project notes** ("Conventions"): "`pip install cupy` and `tdoa.py` uses the GPU
  automatically" — false; no hot-path call site passes `use_gpu`, and the
  default all-pairs path has no GPU parameter to pass (§1.1). Both mirrored
  copies carry the line.
- **README.md** ("Known limits"): "GPU path is stubbed. `pip install cupy` and
  `tdoa.py` will use it" — first half right, second half wrong; the console
  reports CuPy *importability*, not use.
- **requirements.txt**: "# optional: cupy-cuda12x (GPU correlation)" — same
  correction; installing it changes a status string only.
- **Project notes / TODO.md** (unrecorded finding): with the default `gcc: "ht"`,
  the weighted correlation stage runs in complex128 via NumPy promotion of the
  float64 weight (§1.2) — worth recording alongside the dtype conventions, since
  it silently doubles the dominant stage's memory traffic on CPU and would
  cripple a naive port on an FP64-poor GPU.
- **TODO.md** item 1's note "batch the per-node segment FFTs if that ever
  matters" — flagged constraint: `estimate_doppler` rolls `b` by the pair's
  clock delta *before* segmenting, so per-node segment-FFT caching is not a
  drop-in (the roll changes segment boundaries per pair). Cross-pair batching
  within a burst (identical shapes) is the clean form of that optimization.
