#!/usr/bin/env python3
"""Acceptance test for one B210-class board. Run this before buying nine more.

    python3 hw_selftest.py                    # single board
    python3 hw_selftest.py --channels 0,1     # also test the coherent pair

Checks, in the order they matter:

  1. UHD talks to the board at all, and which UHD version does it.
  2. The GPSDO locks, and both ref_locked and gps_locked go true.
  3. The device clock aligns to UTC on a PPS edge.
  4. Scheduled captures land where they were asked to. THIS IS THE ONE THAT
     DECIDES WHETHER TDOA IS POSSIBLE. If schedule error is tens of
     microseconds, the clone's PPS path into the FPGA is not usable and no
     amount of software fixes it.
  5. Timestamp monotonicity across back-to-back captures.
  6. Channel coherence, if you asked for two channels.
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
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--channels", default="0")
    ap.add_argument("--captures", type=int, default=12)
    ap.add_argument("--snippet-ms", type=float, default=10.0)
    ap.add_argument("--baseline-m", type=float, default=0.061,
                    help="element spacing for the coherence/bearing check")
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

    from dronelocate.uhd_source import UhdSource, UhdError, phase_bearing

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

    print("\n=== 3. Scheduled capture accuracy (the decisive test) ===")
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
            gaps.append(t0 - last_t0)
        last_t0 = t0
        time.sleep(0.05)

    if not errs:
        line(FAIL, "scheduled captures", "none succeeded")
        return 1

    med = statistics.median(errs)
    spread = (max(errs) - min(errs))
    jitter = statistics.pstdev(errs) if len(errs) > 1 else 0.0

    print(f"        {len(errs)}/{a.captures} captures succeeded, "
          f"{n} samples each")
    line(OK, "median schedule offset", f"{med*1e6:+.3f} us "
         f"({med*C_LIGHT:+.1f} m equivalent)")

    # A constant offset is harmless -- it is common to all nodes and cancels in
    # the TDOA difference. Jitter does NOT cancel and is the real figure of
    # merit. 100 ns of jitter is 30 m of position error.
    if jitter < 50e-9:
        st = OK
    elif jitter < 200e-9:
        st = WARN
    else:
        st = FAIL
    results.append(line(st, "schedule JITTER (std dev)",
                        f"{jitter*1e9:.1f} ns -> {jitter*C_LIGHT:.1f} m"))
    print(f"        peak-to-peak {spread*1e9:.1f} ns "
          f"({spread*C_LIGHT:.1f} m)")
    print("        Constant offset cancels between nodes; jitter does not.")

    if gaps:
        bad = [g for g in gaps if g <= 0]
        results.append(line(OK if not bad else FAIL, "timestamps monotonic",
                            f"{len(gaps)} intervals, {len(bad)} non-increasing"))

    print("\n=== 4. Signal sanity ===")
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
        print("\n=== 5. Channel coherence (dual-channel DF viability) ===")
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
        print("  If it was schedule jitter, the PPS path into the FPGA is not")
        print("  usable for TDOA and this board can only do bearing-based DF.")
        return 1
    if WARN in results:
        print("  \033[33mUsable with caveats.\033[0m Review the warnings above.")
        return 0
    print("  \033[32mAll checks passed.\033[0m Timing is good enough for TDOA.")
    print(f"  Expected position error floor from timing alone: "
          f"~{jitter*C_LIGHT*1.5:.1f} m before geometry (GDOP) is applied.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
