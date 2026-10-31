# TODO — Navico HALO toolkit

What is still open on the radar side. The fusion with TDOA and the field
plan live in dronelocate's TODO.md (section 8); this file is about the radar
itself and the protocol.

## Protocol still undecoded

The authoritative way to close these is a capture of a real Navico MFD
(Simrad / Lowrance / B&G) talking to *our* radar while every setting is
changed one at a time, with the time of each change noted. Capture from a
port mirror or a TAP: a normal switch port does not see the MFD-radar
multicast. See Mayara's docs/capturing-traffic.md.

- **The "D" message family** on each range's type 0x0012 service (send
  236.6.7.20:6690 / 236.6.7.15:6005, report 236.6.7.19:6689 /
  236.6.7.13:6006): a 4 Hz D480 heartbeat, and D401/D403/D405 replies to MFD
  queries 02/03/05 D2 and to 17 D1. Probably tracking / guard-zone
  configuration. Nothing in Mayara or radar_pi decodes it.
- **0xC409 types 13-55**: we know each one's level mask, max and
  availability flag, not which control it is.
- **0xC407** beyond the mode-dependent parameters at 128-147; **0xC40A**
  bytes 14-23; **0xC402**'s range table (seen only on a HALO 24).
- **0xC415**, the reply to query 0D C2: all zeros here, non-zero in a
  third-party capture.
- **Bit 7 of the "max level" byte** in 0xC409 level entries (has-auto?
  has-off?).
- **Spoke header bytes 20-23**: 0xA0000000 on almost every spoke,
  0x80000000 on ~3 spokes/s.
- **Timed transmit**: 0xC404 u32@26/30 read (30, 300) here, (20, 180) on other
  HALOs -- fits transmit/standby seconds; no command known.
- Radar-sent NKOE on 239.238.55.72:7526 every ~10 s announcing the
  navigation kinds it consumes: could be listened to instead of assumed.

## Measurements still to make on the radar

- **First detection with radarnode on a real mover**: a person walking to
  and from the radar in the hall, A transmitting with `--heading 0` so
  Doppler works. Also measures the real Doppler false-flag rate, which the
  recordings so far cannot (no heading was fed, so no flags exist in them).
- **Outdoors, first day at the field site**: clutter per range scale
  (10-minute recordings), whether custom mode + gain exposes a noise floor
  (decides CFAR vs map-only), and the first drone detection range.
- **Radar latency and clock offset** (measurement to spoke stamp): only
  against a GPS-logged mover (car or drone). The spoke ms clock drifts
  ~20 ppm and differs per session, so radarnode tracks it continuously; the
  constant latency is still assumed 0 +- 50 ms.
- **Mounting azimuth** (`radars[].az_offset_deg` in the site file): compass
  on day one, then a solver against a GPS log (`radar_cal.py`, not written).
- **Dual range**: whether B on a faster scan speed costs A sensitivity (the
  antenna follows the faster range); wall-echo intensity on A with B on/off.
- **Level-0 scan speed** varied 20.9-23.6 rpm between runs and settled at
  20.9 after B stopped: unexplained.

## Tools

- `radar_cal.py`: azimuth offset, range offset, latency from radar plots
  against a GPS track (validate first in the hall with 3 reflectors at
  tape-measured bearings).
- The web console shows range A only: `serve --scanner 1` works, a side-by-
  side A+B page does not exist.
