#!/usr/bin/env python3
"""Acceptance test for one B210-class board. Run this before buying nine more.

    python3 hw_selftest.py                    # single board
    python3 hw_selftest.py --channels 0,1     # also test the coherent pair

Checks, in the order they matter:

  1. UHD talks to the board at all, and which UHD version does it.
  2. The GPSDO locks, both ref_locked and gps_locked go true, the PPS
     interval settles, and the device clock aligns to UTC on a PPS edge.
  3. PPS stability: the device clock latched on every PPS edge for a minute.
     THIS IS THE SINGLE-BOARD GATE. Interval scatter is PPS jitter against
     the disciplined clock; if it is hundreds of ns the clone's PPS path is
     not usable and no amount of software fixes it.
  4. Scheduled captures: the FPGA honours timed commands, with a constant
     offset. This cannot see PPS jitter -- the command and the timestamp
     come from the same counter -- so it is a plumbing check, not a timing
     one. (The first version of this test called it decisive.)
  5. Timestamp monotonicity across back-to-back captures.
  6. Channel coherence, if you asked for two channels.

What no single board can measure is its PPS against true UTC, which is what
TDOA actually consumes. That needs two boards on one signal.

All timing arithmetic keeps UHD's (full_secs, frac_secs) apart. As one
float64 at today's epoch a timestamp is 238 ns coarse, and the first run on
hardware reported that grid as "112 ns of jitter".
"""

import argparse
import statistics
import subprocess
import sys
import time

import numpy as np

C_LIGHT = 299792458.0

OK, WARN, FAIL = "PASS", "WARN", "FAIL"


def line(status, label, detail=""):
    colour = {"PASS": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m"}[status]
    print(f"  {colour}{status}\033[0m  {label:<38} {detail}")
    return status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="type=b200")
    ap.add_argument("--fs", type=float, default=20e6)
    ap.add_argument("--fc", type=float, default=2.437e9)
    # 60: below ~45 dB a TinyB210's floor is its converter's noise, not the
    # antenna's, and weak signals are invisible (measured 2026-10-26)
    ap.add_argument("--gain", type=float, default=60.0)
    ap.add_argument("--channels", default="0")
    ap.add_argument("--captures", type=int, default=12)
    ap.add_argument("--snippet-ms", type=float, default=10.0)
    ap.add_argument("--baseline-m", type=float, default=0.061,
                    help="element spacing for the coherence/bearing check")
    ap.add_argument("--pps-s", type=float, default=60.0,
                    help="how long to watch PPS edges for the stability check")
    a = ap.parse_args()

    channels = tuple(int(x) for x in a.channels.split(","))
    results = []

    print("\n=== 1. UHD and device ===")
    try:
        import uhd
    except Exception as e:
        line(FAIL, "UHD Python bindings", str(e)[:60])
        print("\n  apt install uhd-host python3-uhd && uhd_images_downloader\n")
        return 1
    # The Python bindings expose no version attribute at all (checked against
    # 4.5.0), so getattr here always fell through to "unknown" -- which is
    # useless in a report whose first job is to say which UHD you are running.
    # The C++ side does know, via uhd_config_info.
    ver = getattr(uhd, "__version__", None)
    if ver is None:
        try:
            ver = subprocess.run(["uhd_config_info", "--version"],
                                 capture_output=True, text=True,
                                 timeout=10).stdout.strip() or "unknown"
        except Exception:
            ver = "unknown (no uhd_config_info on PATH)"
    results.append(line(OK, "UHD bindings import", str(ver)))

    from dronelocate.uhd_source import (UhdSource, UhdError, phase_bearing,
                                        secs_between, ts_split)

    print("\n=== 2. GPSDO ===")
    try:
        src = UhdSource(a.fs, a.fc, a.gain, a.device, channels=channels,
                        require_gps=False, gps_timeout_s=180.0)
    except UhdError as e:
        line(FAIL, "open device", str(e)[:70])
        return 1

    h = src.health()
    results.append(line(OK if h["gps_locked"] else FAIL, "gps_locked",
                        str(h["gps_locked"])))
    results.append(line(OK if h["ref_locked"] else FAIL, "ref_locked",
                        str(h["ref_locked"])))
    results.append(line(OK if h["clock_source"] == "gpsdo" else WARN,
                        "clock/time source", f"{h['clock_source']}/{h['time_source']}"))
    ps = src.pps_settle
    if ps is not None:
        results.append(line(OK if ps["settled"] else WARN, "PPS interval settled",
                            f"after {ps['waited_s']:.0f} s"
                            if ps["settled"] else "not within gps_settle_s"))
    results.append(line(OK if h["time_aligned"] else FAIL, "device clock aligned to UTC",
                        str(h["time_aligned"])))
    pos = src.gps_position()
    if pos:
        line(OK, "GNSS position",
             f"{pos['lat']:.6f}, {pos['lon']:.6f}  {pos['n_sats']} sats")
    else:
        line(WARN, "GNSS position", "no GPGGA (fine if antenna indoors)")

    off = h["host_offset_s"]
    results.append(line(OK if abs(off) < 1.0 else WARN, "device vs host clock",
                        f"{off*1e3:+.1f} ms"))
    print(f"        actual rate {h['fs_actual']/1e6:.6f} Msps, "
          f"freq {h['fc_actual']/1e6:.4f} MHz")

    print("\n=== 3. PPS stability (the single-board gate) ===")
    usrp = src.usrp
    mcr = float(usrp.get_master_clock_rate())
    tick = 1.0 / mcr
    print(f"        master clock {mcr/1e6:.2f} MHz: device timestamps are "
          f"{tick*1e9:.1f} ns ticks")
    edges = []
    t_end = time.time() + a.pps_s
    while time.time() < t_end:
        e = ts_split(usrp.get_time_last_pps(0))
        if not edges or e != edges[-1]:
            edges.append(e)
        time.sleep(0.05)
    iv = [secs_between(b, e) - 1.0 for e, b in zip(edges, edges[1:])]
    if len(iv) < 10:
        results.append(line(FAIL, "PPS edges", f"only {len(edges)} in {a.pps_s:.0f} s"))
        pps_jit = None
    else:
        pps_jit = statistics.pstdev(iv)
        # where each edge falls within the device second; with the clock
        # disciplined this stays put, and wander here is PPS jitter too
        ph = [(e[1] if e[1] < 0.5 else e[1] - 1.0) for e in edges]
        st = OK if pps_jit < 50e-9 else WARN if pps_jit < 200e-9 else FAIL
        results.append(line(st, "PPS interval jitter (std dev)",
                            f"{pps_jit*1e9:.1f} ns -> {pps_jit*C_LIGHT:.1f} m, "
                            f"{len(iv)} intervals"))
        print(f"        interval error min/max {min(iv)*1e9:+.1f} / "
              f"{max(iv)*1e9:+.1f} ns; PPS phase in device second "
              f"{min(ph)*1e9:+.1f} .. {max(ph)*1e9:+.1f} ns")
        # one tick of quantisation alone gives ~tick/sqrt(6) of scatter
        print(f"        resolution floor ~{tick/6**0.5*1e9:.0f} ns from the tick; "
              f"a reading near it means 'at least this good'")
        if tick > 30e-9:
            line(WARN, "timestamp resolution",
                 f"{tick*1e9:.0f} ns ticks; --device type=b200,master_clock_rate=40e6")

    print("\n=== 4. Scheduled capture (timed-command check) ===")
    n = int(h["fs_actual"] * a.snippet_ms / 1000.0)
    errs, gaps, last_t0 = [], [], None
    for i in range(a.captures):
        t_target = src.device_time() + 0.5
        try:
            iq, t0, meta = src.capture_at(t_target, n)
        except UhdError as e:
            line(FAIL, f"capture {i}", str(e)[:60])
            continue
        errs.append(meta["schedule_error_s"])
        if last_t0 is not None:
            gaps.append(secs_between(meta["actual_t0_split"], last_t0))
        last_t0 = meta["actual_t0_split"]
        time.sleep(0.05)

    if not errs:
        line(FAIL, "scheduled captures", "none succeeded")
        return 1

    med = statistics.median(errs)
    spread = (max(errs) - min(errs))
    jitter = statistics.pstdev(errs) if len(errs) > 1 else 0.0

    print(f"        {len(errs)}/{a.captures} captures succeeded, "
          f"{n} samples each")
    print(f"        per capture (ns): {[round(e * 1e9, 1) for e in errs]}")
    line(OK, "median schedule offset", f"{med*1e9:+.1f} ns "
         f"(constant pipeline delay; common to identical boards, cancels)")

    # Command and timestamp share one counter, so a healthy FPGA lands every
    # capture on the same offset to the tick. Scatter here is a plumbing
    # fault (a lost command, a first-capture transient), not PPS jitter --
    # section 3 is where PPS jitter shows.
    st = OK if spread <= 1.5 * tick else WARN
    results.append(line(st, "schedule offset constant",
                        f"p-p {spread*1e9:.1f} ns, std {jitter*1e9:.1f} ns "
                        f"({tick*1e9:.1f} ns tick)"))

    if gaps:
        bad = [g for g in gaps if g <= 0]
        results.append(line(OK if not bad else FAIL, "timestamps monotonic",
                            f"{len(gaps)} intervals, {len(bad)} non-increasing"))

    print("\n=== 5. Signal sanity ===")
    p = float(np.mean(np.abs(iq[0]) ** 2))
    pdb = 10 * np.log10(max(p, 1e-20))
    clip = float(np.mean(np.abs(np.real(iq[0])) > 0.98))
    results.append(line(OK if -60 < pdb < -3 else WARN, "mean power",
                        f"{pdb:.1f} dBFS"))
    results.append(line(OK if clip < 0.001 else FAIL, "clipping",
                        f"{clip*100:.3f}% of samples"))
    dc = abs(complex(np.mean(iq[0])))
    line(OK if dc < 0.05 else WARN, "DC offset", f"{dc:.4f}")

    if len(channels) > 1:
        print("\n=== 6. Channel coherence (dual-channel DF viability) ===")
        b = phase_bearing(iq, h["fs_actual"], h["fc_actual"], a.baseline_m)
        results.append(line(OK if b["coherence"] > 0.5 else WARN,
                            "inter-channel coherence", f"{b['coherence']:.3f}"))
        print(f"        phase delta {np.degrees(b['dphi_rad']):+.2f} deg")
        print("        With both inputs on the same source this is your")
        print("        calibration constant -- pass it as cal_phase_rad.")

    src.close()

    print("\n=== Verdict ===")
    if FAIL in results:
        print("  \033[31mDo not buy nine more yet.\033[0m Something above failed.")
        print("  If it was PPS jitter, the PPS path into the FPGA is not")
        print("  usable for TDOA and this board can only do bearing-based DF.")
        return 1
    if WARN in results:
        print("  \033[33mUsable with caveats.\033[0m Review the warnings above.")
        return 0
    print("  \033[32mAll checks passed.\033[0m This board's PPS path is good "
          "enough for TDOA.")
    if pps_jit is not None:
        print(f"  Timing floor from PPS jitter alone: ~{pps_jit*C_LIGHT:.1f} m "
              f"per node, before geometry (GDOP) is applied.")
    print("  Not measured here: this PPS against true UTC. That needs a second")
    print("  board and one signal into both -- do it before buying the fleet.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
