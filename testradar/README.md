# testradar — Navico HALO radar toolkit

Python toolkit for Navico HALO radars on the LAN: a protocol client and CLI
(`navico.py`, stdlib only), a live web display with every control, raw
recordings, and the bench experiments that established what the protocol
fields mean. Built and verified against a **HALO 20+** (firmware 7.1, Sep
2023), dual range A/B.

The protocol knowledge started from [Mayara](https://github.com/MarineYachtRadar/mayara-server)
(Apache-2.0) and [OpenCPN radar_pi](https://github.com/opencpn-radar-pi/radar_pi)
(GPL-2.0-or-later); several of their assumptions turned out wrong on this
radar and are corrected here (see *Verified on hardware* below).

[dronelocate](https://dev.cs.upt.ro/alin.anton/dronelocate) uses this
package for its radar sensor node (`radarnode.py` there).

## Install

```bash
pip install -e .                 # from a checkout; installs the navico module + `navico` command
```

`navico.py` needs nothing beyond the standard library. The bench tools below
run from the checkout.

## CLI

```bash
navico discover                          # serial, multicast groups of A, B and the extra services
navico status [--raw]                    # every decoded report (+ raw bytes)
navico monitor [--spokes]                # live hex of every changed report / spoke header
navico power standby|transmit|off
navico set CONTROL VALUE                 # `navico set --help` lists all controls
navico set raw "04 c2"                   # any bytes, sent as-is
navico nav --heading 0                   # feed heading/COG/SOG (needed for Doppler)
navico record -o run.navrec.gz [--both]  # raw spokes + reports, A and B tagged
navico rotation run.navrec.gz            # antenna rpm per range over time
navico spokes -s 5 -o ppi.png            # rate, drops, one revolution as an image
navico export run.navrec.gz -o spokes.csv
navico --replay run.navrec.gz status|spokes|monitor|serve
navico --scanner 1 ...                   # range B
```

Without installing: `python3 navico.py ...`.

## Web display

```bash
python3 navico.py serve [--host 0.0.0.0] [--port 8090]   # http://127.0.0.1:8090/
```

Port 8090 by default so it can run next to dronelocate's supernode, which
serves its 3D console on 8080 and the spectrum dashboard on 8081.

Live PPI with a sweep, bearing scale, no-transmit sectors and an EBL/VRM
cursor readout (Doppler approaching/receding coloured); live instruments
from the spoke headers (antenna rpm measured vs reported, spokes/s, dropped
spokes from the 12-bit counter, radar ms-clock offset and jitter, echo fill);
per-revolution charts (range profile, echo-level histogram, bearing rose,
trends); every control and installation setting with the radar's
availability flags, heading feed, recording, raw command box, all decoded
and raw reports. Self-contained, no internet needed. Files: `webserver.py`,
`viewer.html`.

## Tests

```bash
tests/setup_browser.sh            # once: Playwright + headless Chromium, no root needed
tests/run_uitest.sh --read-only   # page vs radar, sends nothing
tests/run_uitest.sh               # also drives rain/Doppler/gain through the page and restores
```

## Recordings (`recordings/`)

Format: magic `NAVREC1\n`, then records `<f64 unix time><u8 channel><u32 len><payload>`;
channel = 2 × scanner + kind (kind 0 data, 1 report), so range A is 0/1 and
B is 2/3. `.gz` files are read and written transparently.

| file | what |
|---|---|
| `test.navrec.gz` | 8 s, range A, bird mode, 75 m, radar on a bench in a closed hall |
| `ab_rotation.navrec.gz` | 40 s, A and B; B transmits from ~11 s to ~28 s (antenna speeds up) |
| `experiments.navrec.gz` | the E1–E5 experiment run below, A and B, ~6.5 min |

All three are indoor: the echoes are the hall's walls, not outdoor clutter.

## Experiments (`experiments/2026-10-24/`)

Scripts as run, with `experiments.log`. Every run snapshotted the radar's
state first and restored it afterwards.

## Verified on hardware (HALO 20+)

| finding | evidence |
|---|---|
| NKOE heading/COG field is radians × 10000, not 0..0xF800 = 360° (Mayara and radar_pi both use the latter) | 90°/200°/359° come back exact in spoke headers; 0xF800 scale gave 90.9° and rejected 359° |
| Spoke header bytes 14–15 are the radar's ms clock (0..59999, wraps each minute) | host−stamp offset constant to 0.35–0.5 ms over 1000+ frames; drifts ~20 ppm; NKOE epoch +30 s has no effect |
| Spoke bytes 2–3: 12-bit spoke counter, top nibble 1 on A, 0 on B | +1 per spoke, no drops |
| Spoke status 0xC2 is a transmit start/stop marker (angle 0, range 0), not data | seen only on B start/stop |
| 0xC408 bytes 10–11 = measured antenna rpm × 10, same on both ranges | 221/315/209 = 22.1/31.5/20.9 rpm, equal to spoke-wrap rate |
| Scan speed 0/1/2/3 → ~22/32/41/47 rpm (custom, 75 m); level 3 gives ~38 rpm at 3 km | E4 |
| With both ranges transmitting the antenna runs at the faster range's speed | E4: start order did not matter |
| 0xC408 byte 2 is sea state (Mayara: "STC curve"); byte 14 manual sea level | set and read back |
| 0xC409: 5-byte level entries `[levels mask][0][0][max\|bit7][avail]`; avail 0x80 adjustable, 0x00 radar-controlled (commands still obeyed in custom), 0x20 not available (target boost, local IR) | tests on both ranges |
| 0xC40A: antenna 510 mm, horizontal beamwidth 4.60°, probably 25 W | constant on A and B |
| 0xC403 u32@18 = mains power-up counter | 8→9 and 9→10 across two power cycles; unchanged by transmit cycles and soft off |
| 0xC403 has no build number at 16 (Mayara reads 0 there); the build is the tail of the time string ("279") | |
| 0xC407 @136/140/144 are mode-dependent parameters, resent at transmit transitions — not time counters | timed transmit cycles left them unchanged; they follow bird/offshore |
| `00 C1 00` puts the radar in state "off" (status 0) while it stays on the network; `00 C1 01` returns to standby in < 1 s | E-soft-off |
| Bird mode locks noise, expansion, separation, local IR, scan speed | commands ignored in bird, obeyed in custom |
| The beacon also announces per-range type 0x0012 services; their send addresses are where the speed packet goes | `navico discover` |
| 0D C2 query → 0xC415 report (all zeros here) | E5 |

Still unknown: most 0xC409 entries beyond type 12 (no public names), the
0x0012 "D" message family, parts of 0xC407/0xC40A/0xC402, the meaning of
0xC415. A capture of a Navico MFD changing every setting is the way to
decode them.

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

### Third-party protocol knowledge

`navico.py` is an independent Python implementation, but its protocol
constants, packet layouts and several decoding choices come from:

- **Mayara** (mayara-server, `src/lib/brand/navico`) — Kees Verruijt and the
  Mayara contributors; licensed under the Apache License 2.0 —
  https://github.com/MarineYachtRadar/mayara-server
- **OpenCPN radar_pi** (`src/navico`) — Copyright (C) 2010 David S. Register,
  (C) 2012-2013 Dave Cowell, (C) 2012-2016 Kees Verruijt, (C) 2013-2025
  Douwe Fokkema; licensed under the GNU GPL version 2 or (at your option) any
  later version — https://github.com/opencpn-radar-pi/radar_pi

No source file of either project is copied here. Apache-2.0 and
GPL-2.0-or-later material can both be incorporated into an
AGPL-3.0-or-later work. The same attribution heads `navico.py`.

