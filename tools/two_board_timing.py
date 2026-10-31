#!/usr/bin/env python3
"""Board-to-board timing agreement: the measurement TDOA actually consumes.

    tools/two_board_timing.py --a serial=AAAA --b serial=BBBB
    tools/two_board_timing.py --boards 1970XX,1974XX,1972XX,1971XX
    tools/two_board_timing.py --a serial=AAAA --b serial=BBBB --source tx-a

hw_selftest.py can only compare one board's PPS with its own clock. What a
TDOA fix needs is boards, each disciplined by its own GPSDO, agreeing on
when a sample was taken. So: one signal into all of them, all capture at the
same UTC instant, cross-correlate every pair, and remove each board's
reported clock offset exactly as the supernode does. What is left per pair is

    tdoa = (path_j - path_i) + (fixed board offsets) + relative jitter

The first two are constant and calibrate out. **The scatter is the number**:
it is the relative timing error between two nodes, per capture, and it goes
into every pair the correlator forms. < 50 ns is viable, > 200 ns is not.
Drift (ns/min) matters too: two GPSDOs wandering apart slowly is invisible
in one board's PPS check.

With three or more boards every pair comes from the SAME captures, so the
pair means must close around every triangle (i->j + j->k = i->k). A loop
that does not close is a measurement fault, not a clock one -- which is what
makes the per-board offsets it then estimates (relative to the first board)
trustworthy enough to calibrate with.

Wiring, either way:
  - each board has its OWN GPS antenna (or a GPS splitter feeding them);
    one board disciplining all would hide exactly what is being measured
  - --source external: a generator, a fixed transmitter (a phone hotspot
    kept busy), or one antenna on ambient Wi-Fi into a splitter with equal
    cables to RX2 A of each board (the outer port whose RX LED blinks once
    in the LED test -- see splitter_calibration.md). Over the air, nothing
    may move during the run: geometry is part of the constant
  - --source tx-a: the first board transmits noise from its TX/RX A port.
    Through a splitter, put at least 30 dB of attenuation in front of it:
    +10 dBm into an RX port destroys it.

All boards can sit on one host on USB 3: captures are short timed bursts,
not continuous streams. Turn off any Wi-Fi router within a metre or so of
the boards first -- one under the desk coupled straight into a bare board
and swamped every port. A freshly powered board's oscillator starts ~15 ppm
off and pulls in within ~2 min; one that stays there (seen once, cleared by
a power-cycle) makes every correlation fail on the carrier offset.
"""

import argparse
import csv
import itertools
import statistics
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dronelocate.tdoa import cross_correlate, timing_sigma  # noqa: E402
from dronelocate.uhd_source import (  # noqa: E402
    UhdError, UhdSource, secs_between, split_secs)

C_LIGHT = 299792458.0


def dev_args(spec, mcr):
    """'1970XX' or 'serial=1970XX,...' -> UHD args with the master clock."""
    args = spec if "=" in spec else f"serial={spec}"
    return args if "master_clock_rate" in args else f"{args},master_clock_rate={mcr:g}"


def open_board(label, spec, a, out, tries=2):
    """One board, retried once if its GPSDO is not found. UHD detects the
    GPSDO over a slow serial link at open time with a short timeout; four
    boards opened in parallel made one of them -- a board that finds its
    GPSDO every time alone -- report "GPS invalid reply" and no GPSDO."""
    for k in range(tries):
        try:
            out[label] = UhdSource(a.fs, a.fc, a.gain, dev_args(spec, a.mcr), channels=(0,),
                                   require_gps=True, gps_timeout_s=a.gps_timeout)
            return
        except Exception as e:  # reported by the caller, which knows every board
            out[label] = e
            if "No gps_locked sensor" not in str(e) or k == tries - 1:
                return
            print(f"  board {label}: GPSDO not detected, retrying", flush=True)
            time.sleep(2.0)


class NoiseTx:
    """Continuous noise from one board's TX/RX port, for --source tx-a."""

    def __init__(self, src, fs, fc, gain):
        import uhd
        u = src.usrp
        u.set_tx_rate(fs, 0)
        u.set_tx_freq(uhd.types.TuneRequest(fc), 0)
        u.set_tx_gain(gain, 0)
        u.set_tx_antenna("TX/RX", 0)
        st = uhd.usrp.StreamArgs("fc32", "sc16")
        st.channels = [0]
        self.tx = u.get_tx_stream(st)
        self.md = uhd.types.TXMetadata()
        rng = np.random.default_rng(1)
        n = 1 << 16
        self.buf = (0.2 * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
                    / np.sqrt(2)).astype(np.complex64).reshape(1, -1)
        self.stop = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self.stop.is_set():
            self.tx.send(self.buf, self.md)
        self.md.end_of_burst = True
        self.tx.send(np.zeros((1, 0), np.complex64), self.md)

    def __enter__(self):
        self.th.start()
        time.sleep(0.3)
        return self

    def __exit__(self, *exc):
        self.stop.set()
        self.th.join(2.0)


def capture_all(srcs, t_slot, n):
    res = {}

    def one(k):
        try:
            res[k] = srcs[k].capture_at(t_slot, n)
        except UhdError as e:
            res[k] = e
    ths = [threading.Thread(target=one, args=(k,)) for k in srcs]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    return res


def mad(x):
    x = np.asarray(x, dtype=float)
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def pair_stats(t, td):
    """Scatter, short-term scatter and drift for one pair's tdoa series (ns)."""
    t, td = np.asarray(t), np.asarray(td)
    blocks = [mad(td[(t >= s) & (t < s + 30.0)]) for s in np.arange(0.0, t.max() + 1e-9, 30.0)
              if np.sum((t >= s) & (t < s + 30.0)) >= 5]
    return {"n": len(td), "mean": float(np.mean(td)), "median": float(np.median(td)),
            "std": float(np.std(td)), "mad": mad(td),
            "mad30": float(np.median(blocks)) if blocks else float("nan"),
            "drift": (float(np.polyfit(t / 60.0, td, 1)[0]) if np.ptp(t) >= 60.0
                      else float("nan")),
            "ptp": float(np.ptp(td))}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--a", help="UHD args (or serial) for board A")
    ap.add_argument("--b", help="UHD args (or serial) for board B")
    ap.add_argument("--boards", help="comma-separated serials (or UHD args), two or more; "
                                     "the first is the offset reference")
    ap.add_argument("--source", choices=("external", "tx-a"), default="external")
    ap.add_argument("--tx-gain", type=float, default=20.0,
                    help="first board's TX gain for --source tx-a (attenuate >= 30 dB!)")
    ap.add_argument("--fs", type=float, default=20e6)
    ap.add_argument("--fc", type=float, default=2.437e9)
    # 60: below ~45 dB a TinyB210 hears only its converter's noise floor
    ap.add_argument("--gain", type=float, default=60.0)
    ap.add_argument("--mcr", type=float, default=40e6,
                    help="master clock rate: 40 MHz gives 25 ns timestamp ticks")
    ap.add_argument("--captures", type=int, default=100)
    ap.add_argument("--rate-hz", type=float, default=2.0)
    ap.add_argument("--snippet-ms", type=float, default=1.0)
    ap.add_argument("--max-lag-us", type=float, default=5.0,
                    help="search half-width around the reported clock difference")
    ap.add_argument("--quality-min", type=float, default=8.0)
    ap.add_argument("--gps-timeout", type=float, default=180.0)
    ap.add_argument("--out", help="CSV of every capture, one column per pair")
    ap.add_argument("--start-at", metavar="HH:MM:SS",
                    help="local wall-clock time to start capturing (after the boards are "
                         "open and aligned), so an operator moving the source can follow "
                         "a schedule; the CSV's t_s counts from it")
    a = ap.parse_args()

    specs = ([s.strip() for s in a.boards.split(",") if s.strip()] if a.boards
             else [s for s in (a.a, a.b) if s])
    if len(specs) < 2:
        ap.error("give two or more boards: --boards S1,S2[,...] or --a ... --b ...")
    labels = [s.split("serial=")[-1].split(",")[0] for s in specs]
    pairs = list(itertools.combinations(range(len(labels)), 2))

    print(f"=== opening {len(labels)} boards (GPS lock, PPS settle, alignment) ===")
    # One after another, not in parallel: see open_board. Already-disciplined
    # boards settle in seconds, so this costs little.
    srcs = {}
    for lab, spec in zip(labels, specs):
        open_board(lab, spec, a, srcs)
    bad = {k: v for k, v in srcs.items() if isinstance(v, Exception)}
    if bad:
        for k, e in bad.items():
            print(f"  FAIL  board {k}: {e}")
        for s in srcs.values():
            if not isinstance(s, Exception):
                s.close()
        return 1
    srcs = {lab: srcs[lab] for lab in labels}          # keep the given order
    offs = {}
    for k in labels:
        h = srcs[k].health()
        ps = srcs[k].pps_settle or {}
        offs[k] = h["host_offset_s"]
        print(f"  board {k}: gps_locked={h['gps_locked']} ref_locked={h['ref_locked']} "
              f"aligned={h['time_aligned']} sats={h['n_sats']} "
              f"PPS settled={ps.get('settled')} host offset {h['host_offset_s']*1e3:+.1f} ms")
    # Every clock must name the same second, or every slot is in the past for
    # one of them -- seen once as a board +980.8 ms from the host, 300 of 300
    # captures refused. Say so once instead.
    if max(offs.values()) - min(offs.values()) > 0.5:
        print("  FAIL  the boards' clocks name different seconds "
              f"({', '.join(f'{k} {v:+.3f} s' for k, v in offs.items())}). Re-run.")
        return 1
    fs = srcs[labels[0]].fs_actual
    if any(abs(srcs[k].fs_actual - fs) > 1e-6 for k in labels):
        print(f"  FAIL  boards run different rates: "
              f"{[srcs[k].fs_actual for k in labels]}")
        return 1

    n = int(fs * a.snippet_ms / 1000.0)
    period = 1.0 / a.rate_hz
    series = {p: ([], [], []) for p in pairs}         # t, tdoa ns, sigma ns
    rows, low_q, failed = [], 0, 0
    print(f"\n=== {a.captures} simultaneous captures, {n} samples each, "
          f"{len(pairs)} pairs, source {a.source} ===")

    def run():
        nonlocal low_q, failed
        t_start, last = None, -np.inf
        ref = srcs[labels[0]]
        if a.start_at:
            import datetime as dt
            hh, mm, ss = (int(x) for x in a.start_at.split(":"))
            now = dt.datetime.now()
            when = now.replace(hour=hh, minute=mm, second=ss, microsecond=0)
            wait = (when - now).total_seconds()
            if wait < 0:
                print(f"  --start-at {a.start_at} is already past; starting now", flush=True)
            else:
                print(f"  waiting {wait:.0f} s for {a.start_at}", flush=True)
                time.sleep(wait)
            # t_s counts from the announced instant, not from the first slot
            t_start = time.time() + ref.device_time() - time.time()
        for i in range(a.captures):
            now = ref.device_time()
            # never reuse a slot, even if a capture returned early
            t_slot = max((np.floor((now + 0.3) / period) + 1) * period, last + period)
            last = t_slot
            res = capture_all(srcs, t_slot, n)
            errs = {k: r for k, r in res.items() if isinstance(r, Exception)}
            if errs:
                failed += 1
                print(f"  capture {i}: " + "; ".join(f"{k} {e}" for k, e in errs.items()))
                continue
            slot = split_secs(t_slot)
            # clk = -(t0 - slot), exact, as node.py publishes it
            clk = {k: -secs_between(res[k][2]["actual_t0_split"], slot) for k in res}
            t_start = t_slot if t_start is None else t_start
            row = {"i": i, "t_s": t_slot - t_start}
            for (pi, pj) in pairs:
                ki, kj = labels[pi], labels[pj]
                delta = clk[kj] - clk[ki]
                # same arithmetic as the supernode: correlate j against i with
                # the window centred on the known clock difference, subtract it
                c = cross_correlate(res[kj][0][0], res[ki][0][0], fs,
                                    max_lag_s=a.max_lag_us * 1e-6, center_lag_s=delta)
                key = f"{ki}>{kj}"
                if c.quality < a.quality_min:
                    low_q += 1
                    row[key] = ""
                    continue
                td = (c.lag_s - delta) * 1e9
                series[(pi, pj)][0].append(row["t_s"])
                series[(pi, pj)][1].append(td)
                series[(pi, pj)][2].append(timing_sigma(c, fs) * 1e9)
                row[key] = round(td, 3)
            rows.append(row)
            if i % 20 == 0:
                done = sum(1 for p in pairs if row.get(f"{labels[p[0]]}>{labels[p[1]]}") != "")
                print(f"  capture {i:4d}: {done}/{len(pairs)} pairs usable", flush=True)

    try:
        if a.source == "tx-a":
            with NoiseTx(srcs[labels[0]], fs, a.fc, a.tx_gain):
                run()
        else:
            run()
    finally:
        for s in srcs.values():
            s.close()

    if a.out and rows:
        keys = ["i", "t_s"] + [f"{labels[i]}>{labels[j]}" for i, j in pairs]
        with open(a.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)

    print(f"\n=== result: {len(rows)} captures, {failed} failed, "
          f"{low_q} pair correlations below quality {a.quality_min:g} ===")
    stats = {}
    print(f"  {'pair':<16}{'n':>5}{'mean':>10}{'std':>8}{'MAD':>8}{'MAD/30s':>9}"
          f"{'drift':>11}   (ns; drift ns/min)")
    for p in pairs:
        t, td, sig = series[p]
        name = f"{labels[p[0]]}>{labels[p[1]]}"
        if len(td) < 10:
            print(f"  {name:<16}{len(td):>5}   too few usable captures")
            continue
        s = pair_stats(t, td)
        s["sigma"] = statistics.median(sig)
        stats[p] = s
        print(f"  {name:<16}{s['n']:>5}{s['mean']:>+10.1f}{s['std']:>8.1f}{s['mad']:>8.1f}"
              f"{s['mad30']:>9.1f}{s['drift']:>+11.1f}")
    if not stats:
        print("  No pair had enough usable captures. Check the source power, that every\n"
              "  board hears it, and that every board's PPS settled (raise --gain).")
        return 1

    # Per-board offsets relative to the first board, from the pair medians:
    # median_ij ~ o_j - o_i, solved by least squares with o_0 = 0. With three
    # or more boards the system is over-determined, and its residuals ARE the
    # loop closure: every triangle's i->j + j->k - i->k.
    if len(labels) >= 3 and len(stats) >= 2:
        A, y = [], []
        for (pi, pj), s in stats.items():
            row = np.zeros(len(labels))
            row[pj], row[pi] = 1.0, -1.0
            A.append(row[1:])
            y.append(s["median"])
        o, *_ = np.linalg.lstsq(np.array(A), np.array(y), rcond=None)
        o = np.r_[0.0, o]
        print(f"\n  per-board offset relative to {labels[0]} (includes each board's path "
              f"to the source; constant -> calibrates out):")
        for k, v in zip(labels, o):
            print(f"    {k:<10}{v:+9.1f} ns")
        closures = []
        for i, j, k in itertools.combinations(range(len(labels)), 3):
            if all(p in stats for p in ((i, j), (j, k), (i, k))):
                c = stats[(i, j)]["median"] + stats[(j, k)]["median"] - stats[(i, k)]["median"]
                closures.append((f"{labels[i]}>{labels[j]}>{labels[k]}", c))
        if closures:
            print("  loop closure (i>j + j>k - i>k, should be ~0):")
            for name, c in closures:
                print(f"    {name:<28}{c:+8.1f} ns")
            worst = max(abs(c) for _, c in closures)
            ok = worst <= max(10.0, 2.0 * statistics.median(s["mad"] for s in stats.values())
                              / np.sqrt(min(s["n"] for s in stats.values())) * 3)
            print(f"  -> worst {worst:.1f} ns: "
                  + ("loops close, the offsets are consistent" if ok else
                     "LOOPS DO NOT CLOSE -- a measurement fault (a pair locking onto "
                     "different frames or a wrong peak), not a clock one; do not "
                     "calibrate from this run"))

    worst_std = max(s["std"] for s in stats.values())
    worst_mad = max(s["mad"] for s in stats.values())
    print(f"\n  worst pair scatter: std {worst_std:.1f} ns ({worst_std*1e-9*C_LIGHT:.1f} m), "
          f"MAD {worst_mad:.1f} ns")
    if worst_std < 50:
        print("  VIABLE: every pair agrees to < 50 ns. TDOA works on this hardware.")
    elif worst_mad < 50:
        print("  VIABLE IN THE BULK: robust scatter < 50 ns on every pair, the std is "
              "inflated by outlier captures (mixed sources, weak frames).")
    elif worst_std < 200:
        print("  MARGINAL: 50-200 ns. Usable, with a proportionally worse fix floor.")
    else:
        print("  NOT VIABLE for TDOA: > 200 ns between boards. Bearing-only fallback.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
