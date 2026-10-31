#!/usr/bin/env python3
"""Minimal Navico HALO radar client (stdlib only).

Provenance: an independent Python implementation, but its protocol constants,
packet layouts and several decoding choices come from
  - Mayara (github.com/MarineYachtRadar/mayara-server, src/lib/brand/navico),
    Kees Verruijt and the Mayara contributors, Apache License 2.0
  - OpenCPN radar_pi (github.com/opencpn-radar-pi/radar_pi, src/navico),
    Copyright (C) 2010 David S. Register, 2012-2013 Dave Cowell,
    2012-2016 Kees Verruijt, 2013-2025 Douwe Fokkema; GNU GPL v2 or later
Where they and our HALO 20+ disagree, the radar's behaviour is what is coded,
and the comment says so.

Copyright (C) 2026 Alin-Adrian Anton, University Politehnica Timisoara.
License: GNU AGPL v3 or later -- see LICENSE.

All multi-byte fields are little-endian except IP addresses/ports in the
beacon, which are network order.

Usage:
  navico.py discover            find radars and print their multicast addresses
  navico.py status              print state reports (mode, range, gain, model...)
  navico.py spokes [-s SECS] [-o image.png]
                                receive spoke data, report rotation rate, and
                                optionally render one revolution as a PNG
  navico.py power standby|transmit|off   (off = soft off; radar stays on the network)
  navico.py set CONTROL VALUE   e.g. `set range 1500`, `set gain auto`,
                                `set mode harbor` (`set --help` lists all)
  navico.py record -o FILE [-s SECS] [--both]
                                record raw spoke and report packets (--both:
                                range A and B together, tagged per range)
  navico.py rotation FILE       rotation rate per range over time, with the
                                status/mode/scan-speed changes each range reported
  navico.py serve [--port 8090] live web display with controls (8090: dronelocate
                                uses 8080 for its console, 8081 for spectra)

  --replay FILE makes status, spokes and serve read a recording instead of
  the radar.
"""

import argparse
import base64
import gzip
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
import zlib

DISCOVERY_ADDR = ("236.6.7.5", 6878)
DISCOVERY_QUERY = bytes([0x01, 0xB1])
BEACON_HEADER = bytes([0x01, 0xB2])

RADAR_SERVICE_TYPE = 0x0010
SUBTYPE_DATA, SUBTYPE_SEND, SUBTYPE_REPORT = 0x0010, 0x0011, 0x0012

# Queries / keep-alive sent every few seconds, as Mayara does.
# 01/04 as Mayara; 03/05/0A as radar_pi (its keep-alive cycle).
REPORT_REQUESTS = [bytes([q, 0xC2]) for q in (0x01, 0x03, 0x04, 0x05, 0x0A)]
STAY_ON_A = bytes([0xA0, 0xC1])
REPORT_REQUEST_INTERVAL = 2.0

FRAME_HEADER_LEN = 8
LINE_HEADER_LEN = 24
SPOKE_BYTES = 512  # 1024 pixels, 4 bits each
LINE_LEN = LINE_HEADER_LEN + SPOKE_BYTES
SPOKES_RAW = 4096
SPOKES = 2048
SPOKE_PIXELS = 1024
# 0x02 HALO/BR24 data, 0x12 4G data. 0xC2 is NOT data: single-spoke frames with
# angle 0, range 0 and an empty payload, sent only when transmission starts or
# stops (seen on range B here). Accepting it put a bogus angle-0 spoke into
# the image and into wrap counting.
VALID_SPOKE_STATUSES = {0x02, 0x12}
MARKER_SPOKE_STATUS = 0xC2

STATUS_NAMES = {0: "off", 1: "standby", 2: "transmit", 5: "warming up"}
MODEL_NAMES = {
    10: "BR24", 12: "3G", 13: "4G", 14: "HALO", 16: "HALO 24", 17: "HALO 20",
    18: "HALO 20+", 19: "HALO 2000", 20: "HALO 3000", 21: "HALO 5000",
    22: "HALO 4000", 23: "HALO 6000",
}
HALO_MODES = {0: "custom", 1: "harbor", 2: "offshore", 3: "buoy", 4: "weather", 5: "bird",
              6: "bird+"}
DOPPLER_MODES = {0: "off", 1: "both", 2: "approaching"}
ERROR_NAMES = {
    0x00001: "Persistence corrupt", 0x10001: "Zero bearing fault",
    0x10002: "Bearing pulse fault", 0x10003: "Motor not running",
    0x10004: "Comms not active", 0x10005: "Magnetron heater voltage",
    0x10006: "Modulation voltage", 0x10007: "Trigger fault",
    0x10008: "Video fault", 0x10009: "Fan fault", 0x1000A: "Scanner config fault",
    0x1000B: "Power supply transient", 0x1000C: "Scanner detect fail",
    0x1000D: "PA soft overheat", 0x1000E: "PA hard overheat",
    0x1000F: "Gateway datapath error", 0x10010: "PSU overheat",
    0x10011: "PSU voltage", 0x10012: "PSU power", 0x10013: "PSU hardware fault",
    0x10014: "PSU comms fault", 0x10015: "Mechanical fault", 0x10016: "LED fault",
    0x10017: "Scanner fail", 0x10018: "Radar interface fault",
    0x10019: "Low battery", 0x1001A: "Motor stall", 0x1001B: "Safety switch",
    0x1001C: "Doppler fail", 0x1001D: "Install cache mismatch",
    0x1001E: "Tracking unavailable",
}

CHANNEL_DATA, CHANNEL_REPORT = 0, 1


# --------------------------------------------------------------------------
# Sockets
# --------------------------------------------------------------------------

def local_ip_for(radar_ip):
    """The address of the local interface that routes to the radar."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((radar_ip, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def multicast_rx(group, port, iface_ip, timeout=1.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    # Binding to the group (not 0.0.0.0) keeps other groups on the same port out.
    s.bind((group, port))
    mreq = socket.inet_aton(group) + socket.inet_aton(iface_ip)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    s.settimeout(timeout)
    return s


def multicast_tx(iface_ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(iface_ip))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    s.bind((iface_ip, 0))
    return s


# --------------------------------------------------------------------------
# Discovery
# --------------------------------------------------------------------------

def parse_beacon(data):
    """Returns dict with serial, radar address and a list of scanners (A, B)."""
    if len(data) < 26 or data[:2] != BEACON_HEADER:
        return None
    serial = data[2:18].split(b"\0", 1)[0].decode("ascii", "replace")
    radar_ip = socket.inet_ntoa(data[18:22])
    radar_port, = struct.unpack(">H", data[22:24])
    num_devices, = struct.unpack("<H", data[24:26])

    off = 26
    scanners = []
    services = []      # every group, for display and the 0x0012 service
    for _ in range(num_devices):
        if off + 6 > len(data):
            break
        service_type, _, sub, num_services = struct.unpack("<HBBH", data[off:off + 6])
        off += 6
        if off + num_services * 10 > len(data):
            break
        addrs = {}
        for _ in range(num_services):
            subtype, = struct.unpack("<H", data[off:off + 2])
            ip = socket.inet_ntoa(data[off + 4:off + 8])
            port, = struct.unpack(">H", data[off + 8:off + 10])
            addrs[subtype] = (ip, port)
            off += 10
        services.append({"type": service_type, "sub": sub, "addrs": addrs})
        if service_type == RADAR_SERVICE_TYPE and all(
                k in addrs for k in (SUBTYPE_DATA, SUBTYPE_SEND, SUBTYPE_REPORT)):
            scanners.append((sub, {
                "data": addrs[SUBTYPE_DATA],
                "send": addrs[SUBTYPE_SEND],
                "report": addrs[SUBTYPE_REPORT],
            }))
    scanners.sort(key=lambda x: x[0])
    return {
        "serial": serial,
        "addr": (radar_ip, radar_port),
        "scanners": [s for _, s in scanners],
        # Our HALO 20+ also announces, per range, a type 0x0012 service with
        # its own data/send/report groups (A: 236.6.7.18:6688 / .20:6690 /
        # .19:6689, B: .13:6004 / .15:6005 / .13:6006), plus 0xfffd and
        # 0x001f groups. The 0x0012 send addresses are where the speed packet
        # goes (HALO_SPEED_ADDRS). In third-party captures the radar sends a
        # 4 Hz "D480" heartbeat on the 0x0012 report groups and answers MFD
        # queries 02/03/05 D2 there; contents undecoded (tracking or guard
        # zone configuration, probably).
        "services": services,
    }


def discover(radar_ip, iface_ip, timeout=5.0):
    rx = multicast_rx(*DISCOVERY_ADDR, iface_ip, timeout=0.5)
    tx = multicast_tx(iface_ip)
    deadline = time.monotonic() + timeout
    next_query = 0.0
    try:
        while time.monotonic() < deadline:
            if time.monotonic() >= next_query:
                tx.sendto(DISCOVERY_QUERY, DISCOVERY_ADDR)
                next_query = time.monotonic() + 1.0
            try:
                data, src = rx.recvfrom(2048)
            except socket.timeout:
                continue
            if radar_ip and src[0] != radar_ip:
                continue
            beacon = parse_beacon(data)
            if beacon and beacon["scanners"]:
                beacon["from"] = src[0]
                return beacon
    finally:
        rx.close()
        tx.close()
    return None


def fmt_addr(a):
    return f"{a[0]}:{a[1]}"


# --------------------------------------------------------------------------
# State reports
# --------------------------------------------------------------------------

def utf16z(b):
    return b.decode("utf-16-le", "replace").split("\0", 1)[0]


def pct(byte):
    """0..255 wire value -> 0..100 as shown on an MFD."""
    return round(byte / 2.55)


def decode_report(data):
    """Decode one report packet into (name, dict) or None if unknown."""
    if len(data) < 2:
        return None
    sub, cat = data[0], data[1]
    if cat == 0xC6 and sub == 0x11 and len(data) >= 3:
        n = data[2]
        codes = [struct.unpack_from("<I", data, 3 + 4 * i)[0]
                 for i in range(n) if 3 + 4 * i + 4 <= len(data)]
        return "errors", {"active": [f"0x{c:05X} {ERROR_NAMES.get(c, '?')}" for c in codes]
                          or ["none"]}
    if cat != 0xC4:
        return None
    if sub == 0x01 and len(data) == 18:
        return "mode", {"status": STATUS_NAMES.get(data[2], f"unknown ({data[2]})")}
    if sub == 0x02 and len(data) == 99:
        rng, = struct.unpack_from("<i", data, 2)
        sea, = struct.unpack_from("<i", data, 17)
        return "setup", {
            "range_m": rng / 10,
            "use_mode": HALO_MODES.get(data[7], data[7]),
            "gain": f"{pct(data[12])}{' (auto)' if data[8] else ''}",
            "sea": f"{sea} auto={data[13]}",
            "rain": pct(data[22]),
            "interference_rejection": data[34],
            "target_expansion": data[38],
            "target_boost": data[42],
        }
    if sub == 0x03 and len(data) == 129:
        scanner_type, tx_hours, warmup, max_range = struct.unpack_from("<IIII", data, 30)
        major, minor = struct.unpack_from("<II", data, 50)
        return "properties", {
            "model": MODEL_NAMES.get(scanner_type, f"unknown type {scanner_type}"),
            # Mayara reads a "build" u16 at 16; it is 0 here and the number
            # that follows the time string ("... 03:17:35 279") is the build.
            "firmware": f"{major}.{minor} {utf16z(data[58:90])} {utf16z(data[90:122])}",
            # Power-ups from mains: 8 -> 9 and 9 -> 10 across two power cycles
            # (2026-10-24); unchanged by transmit cycles and by the soft
            # off/on of 00 C1 00 / 00 C1 01.
            "power_ups": struct.unpack_from("<I", data, 18)[0],
            "feature_flags": f"0x{data[14]:02x}",
            "transmit_hours": tx_hours,
            "warmup_s": warmup,
            "max_range_m": max_range / 10,
            "protocol_version": struct.unpack_from("<I", data, 122)[0],
        }
    if sub == 0x04 and len(data) == 66:
        bearing, = struct.unpack_from("<h", data, 6)
        height, = struct.unpack_from("<i", data, 10)
        fwd, = struct.unpack_from("<h", data, 20)
        stbd, = struct.unpack_from("<h", data, 23)
        sectors = []
        for i in range(4):
            en, start, end = struct.unpack_from("<Bhh", data, 34 + 5 * i)
            if en:
                sectors.append(f"#{i} {start / 10:.1f}..{end / 10:.1f} deg")
        return "config", {
            "bearing_alignment_deg": bearing / 10,
            # radar_pi: parking angle, u16 decidegrees, > 180 shown negative.
            "parking_angle_deg": (lambda a: a - 360 if a > 180 else a)(
                struct.unpack_from("<H", data, 8)[0] / 10),
            "antenna_height_m": height / 1000,
            "accent_light": data[19],
            "antenna_forward_mm": fwd,
            "antenna_starboard_mm": stbd,
            "no_transmit_sectors": sectors or ["none"],
            # Paired values (30, 300) here, (20, 180) on other HALOs: fits a
            # timed-transmit setting in seconds. Unverified; no command known.
            "timed_tx_s?": list(struct.unpack_from("<II", data, 26)),
        }
    if sub == 0x08 and len(data) >= 18:
        c = data[2:18]
        # Mayara calls c[0] the STC curve; on a HALO 20+ it is what the sea
        # state command (0xC10B) sets.
        out = {
            "sea_state": c[0], "local_interference_rejection": c[1],
            "scan_speed": c[2], "sidelobe": "auto" if c[3] else pct(c[7]),
            "noise_rejection": c[10], "target_separation": c[11],
            "sea_clutter": c[12],
            "auto_sea_offset": struct.unpack("b", bytes([c[13]]))[0],
            # Measured antenna speed in 0.1 rpm, same on both ranges: read
            # 221 / 315 / 209 while spoke wraps gave 22.1 / 31.5 / 20.9 rpm.
            "antenna_rpm": struct.unpack_from("<H", c, 8)[0] / 10,
        }
        if len(data) >= 21:
            out["doppler"] = DOPPLER_MODES.get(data[18], data[18])
            out["doppler_speed_m_s"] = struct.unpack_from("<H", data, 19)[0] / 100
        if len(data) >= 24:
            mode, variant = data[22], data[23]
            out["use_mode"] = "bird+" if (mode, variant) == (5, 1) else HALO_MODES.get(mode, mode)
        return "setup_extended", out
    if sub == 0x06:
        return "installation", decode_installation(data)
    if sub == 0x07 and len(data) >= 164:
        i32 = lambda off: struct.unpack_from("<i", data, off)[0]
        return "processing", {
            # Not time counters (timed transmit cycles left them unchanged).
            # They follow the use mode -- bird 411561/183229/-37081, offshore
            # 420777/200091/-20219 -- and the radar resends them only at a
            # transmit transition, so after a mode change they lag until then.
            "mode_params": [i32(136), i32(140), i32(144)],
            # Depend on range and mode (bird 75 m: 0x6000/0x14000, offshore
            # 115 m: 0x9000/0x12000); resent right after a range command.
            "range_params": [f"0x{i32(128):x}", f"0x{i32(132):x}"],
            "other": {"b69": data[69], "i118": i32(118), "i126": i32(126),
                      "i148": i32(148), "i152": i32(152), "i156": i32(156),
                      "i160": i32(160)},
        }
    if sub == 0x0A and len(data) >= 18:
        u16 = lambda off: struct.unpack_from("<H", data, off)[0]
        return "antenna", {
            "antenna_length_mm": u16(6),          # 510 on this 20" dome
            "beamwidth_h_deg": u16(8) / 100,      # 4.60 here (datasheet 4.9)
            "power_mw?": struct.unpack_from("<I", data, 2)[0],
            "flags": f"0x{struct.unpack_from('<I', data, 10)[0]:x}",
            "u14": u16(14), "u16": u16(16),
        }
    if sub == 0x12 and len(data) >= 14:
        return "serial", {"serial": data[2:14].split(b"\0", 1)[0].decode("ascii", "replace"),
                          "hex": data[2:].hex(" ")}
    if sub == 0x09:
        return "features", decode_features(data[2:])
    if sub == 0x15:
        # Sent in reply to the query 0D C2 (which neither Mayara nor radar_pi
        # sends). All zeros on our radar; a third-party capture has
        # "00 00 00 01 00 00 00 00 00 04 00 00 00 00 40 00 00". Meaning unknown.
        return "0xC415", {"length": len(data), "hex": data[2:].hex(" ")}
    return f"0xC4{sub:02X}", {"length": len(data), "hex": data[2:].hex(" ")}


def decode_installation(data):
    """0xC406: TLV entries of u16 tag, u16 length, payload."""
    out = {}
    off = 2
    while off + 4 <= len(data):
        tag, n = struct.unpack_from("<HH", data, off)
        off += 4
        p = data[off:off + n]
        off += n
        if tag == 0x0000:
            out["name"] = p.split(b"\0", 1)[0].decode("ascii", "replace").strip().rstrip(";")
        elif tag == 0x0001 and len(p) >= 12:
            # Offsets confirmed by changing each installation setting on this
            # radar and watching which payload bytes moved.
            out["bearing_alignment_deg"] = struct.unpack_from("<h", p, 0)[0] / 10
            out["antenna_height_m"] = struct.unpack_from("<H", p, 2)[0] / 1000
            out["antenna_forward_mm"] = struct.unpack_from("<h", p, 6)[0]
            out["antenna_starboard_mm"] = struct.unpack_from("<h", p, 10)[0]
        elif tag == 0x0003 and p:
            sectors = []
            for i in range(p[0]):
                if 1 + 5 * i + 5 > len(p):
                    break
                en, start, end = struct.unpack_from("<Bhh", p, 1 + 5 * i)
                sectors.append(f"#{i} {'on' if en else 'off'} {start / 10:.1f}..{end / 10:.1f}")
            out["sectors"] = sectors
        elif tag == 0x0004 and len(p) >= 8:
            out["cable_length_mm"] = struct.unpack_from("<i", p, 4)[0]
        else:
            out[f"tag_{tag}"] = p.hex(" ")
    return out


FEATURE_LEVELS = {3: "interference_rejection", 4: "noise_rejection", 5: "target_boost",
                  6: "stc_curve", 7: "target_separation", 8: "scan_speed",
                  12: "local_interference_rejection"}
# Last byte of a level/range entry. 0x00 is not "locked by the current mode":
# the table is byte-identical for range A in bird mode and B in offshore, and
# scan speed (flagged 0x00) obeyed its command in custom mode.
FEATURE_AVAIL = {0x80: "adjustable", 0x00: "radar-controlled", 0x20: "not available"}


def decode_features(data):
    """0xC409: TLV entries of u8 type, u8 reserved, u8 length, payload."""
    out = {}
    off = 0
    while off + 3 <= len(data):
        typ, n = data[off], data[off + 2]
        p = data[off + 3:off + 3 + n]
        off += 3 + n
        if typ == 2 and len(p) >= 4:
            mask, = struct.unpack_from("<I", p)
            # Bit n = use mode n (radar_pi); Mayara's bit order differs.
            out["use_modes"] = [nm for i, nm in HALO_MODES.items() if i < 6 and mask >> i & 1]
        elif len(p) == 5 and typ not in (15, 19) and (
                typ in FEATURE_LEVELS or p[2] == 0 and p[1] == 0):
            # [bitmask of levels][0][0][max level | bit 7][availability].
            # Controls controlled only by these 5-byte entries (types beyond 12
            # included) have no public name; they are shown by number.
            # 0x20 confirmed on this radar: target boost and local IR ignore
            # their commands.
            levels = [i for i in range(8) if p[0] >> i & 1]
            name = FEATURE_LEVELS.get(typ, f"type_{typ}")
            out[name + "_levels"] = (f"{levels} max {p[3] & 0x7F}"
                                     + (" +bit7" if p[3] & 0x80 else "")
                                     + f" {FEATURE_AVAIL.get(p[4], f'0x{p[4]:02x}')}")
        elif typ == 9 and len(p) >= 4:
            out["sidelobe_range"] = f"{p[0]}..{p[1]} {FEATURE_AVAIL.get(p[3], hex(p[3]))}"
        elif typ == 15 and len(p) >= 5:
            out["sea_range"] = (f"manual {p[0]}..{p[1]}, auto offset "
                                f"{struct.unpack('b', p[2:3])[0]}..{struct.unpack('b', p[3:4])[0]} "
                                f"{FEATURE_AVAIL.get(p[4], hex(p[4]))}")
        elif typ == 19 and len(p) >= 5:
            out["doppler_speed_range_cm_s"] = (f"{struct.unpack_from('<H', p, 1)[0]}.."
                                               f"{struct.unpack_from('<H', p, 3)[0]} "
                                               f"{FEATURE_AVAIL.get(p[0], hex(p[0]))}")
        elif typ == 10:
            if p[:1] == b"\0":
                out["antenna"] = "dome"
            elif p:
                # count, then 3 bytes per entry (byte, u16 size mm) per a
                # HALO 3006 capture; unverified on any radar we have.
                sizes = [struct.unpack_from("<H", p, 2 + 3 * i)[0]
                         for i in range(p[0]) if 4 + 3 * i <= len(p)]
                out["antenna"] = "open array " + ", ".join(f"{s} mm" for s in sizes)
        elif typ == 11 and len(p) >= 8:
            lo, hi = struct.unpack_from("<II", p)
            out["instrumented_range_m"] = f"{lo / 10}..{hi / 10}"
        else:
            out[f"type_{typ}"] = p.hex(" ")
    return out


# --------------------------------------------------------------------------
# Control commands
# --------------------------------------------------------------------------

def _level(lo, hi):
    def parse(v):
        n = int(v)
        if not lo <= n <= hi:
            raise ValueError(f"must be {lo}..{hi}")
        return n
    return parse


def _percent_byte(v):
    n = float(v)
    if not 0 <= n <= 100:
        raise ValueError("must be 0..100")
    return round(n * 2.55)


def _choice(names):
    inv = {v: k for k, v in names.items()}

    def parse(v):
        if v not in inv:
            raise ValueError(f"must be one of {', '.join(inv)}")
        return inv[v]
    return parse


def _cmd(sub):
    return bytes([sub, 0xC1])


def cmd_frames_range(v):
    m = float(v)
    if not 20 <= m <= 100000:
        raise ValueError("must be 20..100000 metres")
    return [_cmd(0x03) + struct.pack("<i", round(m * 10))]


def cmd_frames_gain(v):
    if v == "auto":
        return [_cmd(0x06) + bytes([0x00, 0, 0, 0]) + struct.pack("<I", 1) + bytes([0])]
    return [_cmd(0x06) + bytes([0x00, 0, 0, 0]) + struct.pack("<I", 0)
            + bytes([_percent_byte(v)])]


def cmd_frames_sea(v):
    """HALO sea clutter: `auto`, `auto+N`/`auto-N` (offset -50..50) or 0..100.
    The mode frame always goes first; the value frame does not carry the mode."""
    def frame(auto, setting, kind):
        return _cmd(0x11) + bytes([auto, max(setting, 0), setting & 0xFF, kind])
    if v.startswith("auto"):
        frames = [frame(1, 0, 0x01)]
        if len(v) > 4:
            off = int(v[4:])
            if not -50 <= off <= 50:
                raise ValueError("auto offset must be -50..50")
            frames.append(frame(1, off, 0x04))
        return frames
    n = int(v)
    if not 0 <= n <= 100:
        raise ValueError("must be 0..100, auto or auto±N")
    return [frame(0, 0, 0x01), frame(0, n, 0x02)]


def cmd_frames_rain(v):
    return [_cmd(0x06) + bytes([0x04, 0, 0, 0, 0, 0, 0, 0, _percent_byte(v)])]


def cmd_frames_sidelobe(v):
    if v == "auto":
        return [_cmd(0x06) + bytes([0x05, 0, 0, 0, 1, 0, 0, 0, 0])]
    return [_cmd(0x06) + bytes([0x05, 0, 0, 0, 0, 0, 0, 0, _percent_byte(v)])]


def cmd_frames_mode(v):
    mode = _choice(HALO_MODES)(v)
    # Bird+ is bird with variant 1.
    return [_cmd(0x10) + (bytes([5, 1]) if mode == 6 else bytes([mode, 0]))]


def cmd_frames_doppler_speed(v):
    cms = round(float(v) * 100)
    if not 0 <= cms <= 1594:
        raise ValueError("must be 0..15.94 m/s")
    return [_cmd(0x24) + struct.pack("<H", cms)]


def _simple(sub, parse):
    return lambda v: [_cmd(sub) + bytes([parse(v)])]


def _deci_degrees(v):
    return (round(float(v) * 10) + 7200) % 3600


def cmd_frames_bearing(v):
    return [_cmd(0x05) + struct.pack("<h", _deci_degrees(v))]


def cmd_frames_antenna_height(v):
    m = float(v)
    if not 0 <= m <= 100:
        raise ValueError("must be 0..100 metres")
    return [_cmd(0x30) + struct.pack("<Ii", 0x01, round(m * 1000))]


def cmd_frames_antenna_offset(v):
    try:
        fwd, stbd = (float(x) for x in v.split(","))
    except ValueError:
        raise ValueError("must be FORWARD,STARBOARD in metres, e.g. 1.5,-0.3")
    return [_cmd(0x30) + struct.pack("<Iii", 0x04, round(fwd * 1000), round(stbd * 1000))]


def cmd_frames_parking(v):
    a = float(v)
    if not -179 <= a <= 180:
        raise ValueError("must be -179..180 degrees")
    return [_cmd(0x40) + struct.pack("<H", _deci_degrees(a)) + bytes(2)]


def cmd_frames_antenna_size(v):
    return [_cmd(0x30) + struct.pack("<I", 0x03) + bytes([_level(0, 2)(v), 0, 0, 0])]


def cmd_frames_sector(v):
    """N:START:END (degrees, enabled), N:START:END:off (disabled, angles kept)
    or N:off, N = 1..4. Two frames: enable, then range."""
    parts = v.split(":")
    try:
        n = int(parts[0]) - 1
        if not 0 <= n <= 3 or len(parts) not in (2, 3, 4):
            raise ValueError
        if parts[1:] == ["off"]:
            enabled, start, end = 0, 0, 0
        else:
            enabled = 0 if parts[3:] == ["off"] else 1
            if len(parts) == 4 and parts[3] != "off":
                raise ValueError
            start, end = _deci_degrees(parts[1]), _deci_degrees(parts[2])
    except (ValueError, IndexError):
        raise ValueError("must be N:START:END, N:START:END:off or N:off with N 1..4, "
                         "e.g. 1:170:190")
    head = bytes([n, 0, 0, 0, enabled])
    return [_cmd(0x0D) + head, _cmd(0xC0) + head + struct.pack("<hh", start, end)]


def cmd_frames_raw(v):
    """Any hex string, sent as-is (e.g. `set raw "0a c1 01"`)."""
    return [bytes.fromhex(v.replace(":", " "))]


# name -> (help, frames builder)
CONTROLS = {
    "range": ("metres", cmd_frames_range),
    "gain": ("0..100 or auto", cmd_frames_gain),
    "sea": ("0..100, auto, or auto±N (offset -50..50)", cmd_frames_sea),
    "rain": ("0..100", cmd_frames_rain),
    "sidelobe": ("0..100 or auto", cmd_frames_sidelobe),
    "mode": ("|".join(HALO_MODES.values()), cmd_frames_mode),
    "doppler": ("|".join(DOPPLER_MODES.values()), _simple(0x23, _choice(DOPPLER_MODES))),
    "doppler-speed": ("m/s threshold 0..15.94", cmd_frames_doppler_speed),
    "interference": ("0..3 (off/low/medium/high)", _simple(0x08, _level(0, 3))),
    "local-interference": ("0..3", _simple(0x0E, _level(0, 3))),
    "noise": ("0..3", _simple(0x21, _level(0, 3))),
    "target-boost": ("0..2 (off/low/high)", _simple(0x0A, _level(0, 2))),
    "target-expansion": ("0..3 (off/low/medium/high)", _simple(0x12, _level(0, 3))),
    "target-separation": ("0..3", _simple(0x22, _level(0, 3))),
    # Measured on our HALO 20+ in custom mode at 75 m: 0 -> 22.0, 1 -> 31.9,
    # 2 -> 40.8, 3 -> 47.4 rpm (level 0 varied 20.9-23.6 between runs).
    # At level 3: 46.7 rpm at 250 m, 48.8 at 1 km, 37.8 at 3 km. Locked in
    # bird mode. With both ranges transmitting the antenna runs at the
    # faster range's speed, whichever started last. Read the real speed from
    # 0xC408 antenna_rpm, never assume it.
    "scan-speed": ("0..3 (~22/32/41/47 rpm at short range)", _simple(0x0F, _level(0, 3))),
    "sea-state": ("0..2 (calm/moderate/rough)", _simple(0x0B, _level(0, 2))),
    "accent-light": ("0..3", _simple(0x31, _level(0, 3))),
    # Installation
    "bearing-alignment": ("degrees -180..180", cmd_frames_bearing),
    "antenna-height": ("metres", cmd_frames_antenna_height),
    "antenna-offset": ("FORWARD,STARBOARD metres from GPS", cmd_frames_antenna_offset),
    "parking-angle": ("degrees -179..180 (open array; a HALO 20+ dome ignores it)", cmd_frames_parking),
    "antenna-size": ("open array only: 0 = 3 ft, 1 = 4 ft, 2 = 6 ft", cmd_frames_antenna_size),
    "no-transmit-sector": ("N:START:END degrees, N:START:END:off or N:off (N 1..4)",
                           cmd_frames_sector),
    "raw": ("hex bytes sent unchanged to the command address", cmd_frames_raw),
}

INSTALLATION_CONTROLS = {"parking-angle", "antenna-size", "bearing-alignment", "antenna-height", "antenna-offset",
                         "no-transmit-sector"}


def power_frames(state):
    """standby | transmit | off.

    00 C1 01 then 01 C1 00/01 is what Mayara and radar_pi send. 00 C1 00 is
    sent by neither; on our HALO 20+ it puts the radar in state "off"
    (status 0) while it stays on the network, answers ping and keeps
    reporting -- a soft off of the radio side. 00 C1 01 brings it back to
    standby in under a second. It is not a reboot: the power-up counter in
    0xC403 does not move (a mains power cycle does).
    """
    if state == "off":
        return [bytes([0x00, 0xC1, 0x00])]
    return [bytes([0x00, 0xC1, 0x01]), bytes([0x01, 0xC1, 1 if state == "transmit" else 0])]


# --------------------------------------------------------------------------
# Radar connection and recording
# --------------------------------------------------------------------------

HALO_HEADING_ADDR = ("239.238.55.73", 7527)
HALO_SPEED_ADDRS = [("236.6.7.20", 6690), ("236.6.7.15", 6005)]
NKOE_U01 = bytes([0, 0, 0x10, 0, 0, 0x14, 0, 0, 4, 0, 0, 0, 0, 0, 5, 0x3C, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0x20])
# Angles in NKOE packets are radians * 10000 (0..62831). Mayara assumes
# 0..0xF800 = 360 deg, but a HALO 20+ reads 15872 as 90.9 deg and rejects
# 63311 (> 2 pi), which only fits radians * 10000.
def nkoe_angle(deg):
    return int(math.radians(deg % 360) * 10000)


class NavSender:
    """Feeds heading (every 100 ms) and COG/SOG (every 250 ms) to a HALO, as an
    MFD does. HALO uses it for Doppler and to stamp true heading on spokes.
    Packet layouts from Mayara info.rs / radar_pi."""

    OTHER_SENDER_TIMEOUT = 10.0

    def __init__(self, iface_ip):
        self.iface_ip = iface_ip
        self.tx = multicast_tx(iface_ip)
        # Like radar_pi, stay quiet while another MFD feeds heading.
        self.rx = multicast_rx(*HALO_HEADING_ADDR, iface_ip, timeout=0)
        self.heading = self.cog = self.sog = None  # degrees, degrees, m/s
        # Added to the epoch field of every NKOE packet. Tested: +30 s left
        # the radar's spoke clock (spoke bytes 14-15) unchanged to 0.4 ms, so
        # that clock does not follow the time we send. Normally 0.
        self.epoch_offset_s = 0.0
        self.counter = 0
        self.sent = 0
        self.other_sender = None
        self._other_seen = 0.0

    def _listen(self):
        while True:
            try:
                _, src = self.rx.recvfrom(2048)
            except (BlockingIOError, socket.timeout):
                return
            if src[0] != self.iface_ip:
                self.other_sender = src[0]
                self._other_seen = time.monotonic()

    def yielding(self):
        """True while another device is sending heading (we don't)."""
        return time.monotonic() - self._other_seen < self.OTHER_SENDER_TIMEOUT

    def _nkoe(self, kind, tail):
        pkt = (b"NKOE" + bytes([0, 1, 0x90, 0x02]) + struct.pack(">H", self.counter)
               + NKOE_U01 + kind
               + struct.pack("<q", int((time.time() + self.epoch_offset_s) * 1000))
               + bytes([2, 0, 0, 0, 0, 0, 0, 0]) + bytes(8) + b"\xff" + tail)
        self.counter = (self.counter + 1) & 0xFFFF
        return pkt

    def run(self, stop):
        last_nav = 0.0
        while not stop.is_set():
            self._listen()
            if self.yielding():
                stop.wait(0.1)
                continue
            if self.heading is not None:
                h = nkoe_angle(self.heading)
                self.tx.sendto(self._nkoe(bytes([0x12, 0xF1, 1, 0]), struct.pack("<H", h)
                                          + bytes([0xFF, 0x7F, 0x79, 0xF8, 0xFC])),
                               HALO_HEADING_ADDR)
                self.sent += 1
            if self.cog is not None and self.sog is not None and \
                    time.monotonic() - last_nav >= 0.25:
                last_nav = time.monotonic()
                c = nkoe_angle(self.cog)
                self.tx.sendto(self._nkoe(bytes([0x02, 0xF8, 1, 0]), b"\xfc"
                                          + struct.pack("<HH", c, int(self.sog * 100))
                                          + b"\xff\xff"), HALO_HEADING_ADDR)
                speed = (bytes([1, 0xD3, 1, 0, 0, 0]) + struct.pack("<H", int(self.sog * 10))
                         + bytes([0, 0, 1, 0, 0, 0]) + struct.pack("<H", int(self.cog % 360 * 10))
                         + bytes([0, 0, 1, 0x33, 0, 0, 0]))
                for addr in HALO_SPEED_ADDRS:
                    self.tx.sendto(speed, addr)
                self.sent += 1
            stop.wait(0.1)
        self.tx.close()
        self.rx.close()


class Radar:
    """Live connection: receives reports (and optionally spokes) on background
    threads, keeps the latest decoded state, and sends commands."""

    def __init__(self, beacon, iface_ip, scanner=0):
        self.beacon = beacon
        self.iface_ip = iface_ip
        self.scanner = scanner
        self.sc = beacon["scanners"][scanner]
        self.state = {}
        self.raw = {}
        self.listeners = []
        self.nav = None
        self._tx = multicast_tx(iface_ip)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []

    def describe(self):
        return {"source": f"{self.beacon['from']} serial {self.beacon['serial']}",
                "data": fmt_addr(self.sc["data"]), "live": True}

    def start(self, spokes=True):
        self._spawn(self._report_loop)
        if spokes:
            self._spawn(self._data_loop)
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self._tx.close()

    def send(self, frames):
        with self._lock:
            for f in frames:
                self._tx.sendto(f, self.sc["send"])
                time.sleep(0.05)

    def set_nav(self, heading=None, cog=None, sog=None):
        """Start (or update) sending heading/COG/SOG to the radar."""
        if self.nav is None:
            self.nav = NavSender(self.iface_ip)
            self._spawn(lambda: self.nav.run(self._stop))
        self.nav.heading, self.nav.cog, self.nav.sog = heading, cog, sog

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    def _emit(self, channel, data):
        t = time.time()
        if channel == CHANNEL_REPORT:
            r = decode_report(data)
            if r:
                self.state[r[0]] = r[1]
                self.raw[r[0]] = data.hex(" ")
            else:
                self.raw[f"{data[1]:02X}{data[0]:02X}" if len(data) > 1 else "short"] = \
                    data.hex(" ")
        for fn in list(self.listeners):
            fn(channel, t, data)

    def _report_loop(self):
        rx = multicast_rx(*self.sc["report"], self.iface_ip, timeout=0.5)
        next_req = 0.0
        try:
            while not self._stop.is_set():
                if time.monotonic() >= next_req:
                    # Also the stay-alive that keeps us registered as a client.
                    self.send(REPORT_REQUESTS + [STAY_ON_A])
                    next_req = time.monotonic() + REPORT_REQUEST_INTERVAL
                try:
                    data, _ = rx.recvfrom(4096)
                except socket.timeout:
                    continue
                self._emit(CHANNEL_REPORT, data)
        finally:
            rx.close()

    def _data_loop(self):
        rx = multicast_rx(*self.sc["data"], self.iface_ip, timeout=0.5)
        try:
            while not self._stop.is_set():
                try:
                    data, _ = rx.recvfrom(65536)
                except socket.timeout:
                    continue
                self._emit(CHANNEL_DATA, data)
        finally:
            rx.close()


# Recording file: magic, then records of <double unix time><u8 channel><u32 len><payload>.
# channel = 2 * scanner + kind (kind: 0 data, 1 report), so range A is 0/1 as in
# recordings made before range B could be recorded, and range B is 2/3.
REC_MAGIC = b"NAVREC1\n"
REC_HEADER = struct.Struct("<dBI")


class Recorder:
    def __init__(self, path):
        # .gz paths are compressed on the fly: spokes are 4-bit and mostly
        # zero, so a recording shrinks ~25x (133.7 MB -> 5.4 MB measured).
        self.f = gzip.open(path, "wb") if path.endswith(".gz") else open(path, "wb")
        self.f.write(REC_MAGIC)
        self.count = 0
        self.bytes = 0
        self._lock = threading.Lock()

    def for_scanner(self, scanner):
        """Listener for one Radar: tags its records with the scanner index."""
        return lambda channel, t, data: self(2 * scanner + channel, t, data)

    def __call__(self, channel, t, data):
        with self._lock:
            self.f.write(REC_HEADER.pack(t, channel, len(data)))
            self.f.write(data)
            self.count += 1
            self.bytes += len(data)

    def close(self):
        with self._lock:
            self.f.close()


def rec_channel(channel):
    """Recording channel -> (scanner index, CHANNEL_DATA or CHANNEL_REPORT)."""
    return channel // 2, channel % 2


def read_recording(path):
    with (gzip.open(path, "rb") if path.endswith(".gz") else open(path, "rb")) as f:
        if f.read(len(REC_MAGIC)) != REC_MAGIC:
            raise ValueError(f"{path} is not a navico.py recording")
        while True:
            h = f.read(REC_HEADER.size)
            if len(h) < REC_HEADER.size:
                return
            t, channel, n = REC_HEADER.unpack(h)
            data = f.read(n)
            if len(data) < n:
                return
            yield t, channel, data


class Replay(Radar):
    """Plays a recording back in real time (looping), with the same interface
    as Radar so status, spokes and serve work unchanged."""

    def __init__(self, path, speed=1.0, loop=True, scanner=0):
        self.path = path
        self.scanner = scanner
        self.speed = speed
        self.loop = loop
        self.state = {}
        self.raw = {}
        self.listeners = []
        self.nav = None
        self._stop = threading.Event()
        self._threads = []
        self.finished = threading.Event()

    def describe(self):
        return {"source": f"replay of {os.path.basename(self.path)} "
                          f"range {'AB'[self.scanner] if self.scanner < 2 else self.scanner}",
                "data": "-",
                "live": False}

    def start(self, spokes=True):
        self._spawn(self._play)
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)

    def send(self, frames):
        raise RuntimeError("cannot send commands to a recording")

    def set_nav(self, heading=None, cog=None, sog=None):
        raise RuntimeError("cannot send navigation data to a recording")

    def _play(self):
        while not self._stop.is_set():
            first = None
            start = time.monotonic()
            for t, channel, data in read_recording(self.path):
                if self._stop.is_set():
                    return
                scanner, kind = rec_channel(channel)
                if scanner != self.scanner:
                    continue
                first = t if first is None else first
                wait = (t - first) / self.speed - (time.monotonic() - start)
                if wait > 0:
                    time.sleep(wait)
                self._emit(kind, data)
            if not self.loop:
                break
        self.finished.set()


# --------------------------------------------------------------------------
# Spokes
# --------------------------------------------------------------------------

def decode_line_header(h):
    """Gen3+/HALO spoke header -> (angle 0..2047, range_m, heading or None)."""
    if h[0] != LINE_HEADER_LEN or h[1] not in VALID_SPOKE_STATUSES:
        return None
    large_range, angle, heading, small_range = struct.unpack_from("<HHHH", h, 6)
    if large_range == 0x80:
        rng = 0.0 if small_range == 0xFFFF else small_range / 4
    else:
        rng = large_range * small_range / 512
    hdg = None
    if heading & 0x4000 and not heading & ~(0x4000 | (SPOKES_RAW - 1)):
        hdg = (heading & (SPOKES_RAW - 1)) * 360.0 / SPOKES_RAW
    return (angle // 2) % SPOKES, rng, hdg


def spoke_meta(h):
    """The spoke-header fields decode_line_header leaves out.

    status   0x02 data (0x12 on 4G); 0xC2 = transmit start/stop marker
    counter  bytes 2-3 & 0x0FFF: spoke counter, +1 per spoke, wraps at 4096;
             the top nibble (flags) was 1 on range A and 0 on B
    ms       bytes 14-15: the radar's own clock, ms 0..59999 wrapping each
             minute. Host time minus stamp is constant to ~0.5 ms within a
             session, so it times each spoke far better than packet arrival
             (2-8 ms jitter); it drifts ~20 ppm against the host and ignores
             the NKOE epoch, so track the offset rather than fix it once.
    tail     bytes 20-23: 0xA0000000 on almost every spoke, 0x80000000 on ~3
             spokes/s; unknown
    """
    word = struct.unpack_from("<H", h, 2)[0]
    return {"status": h[1], "counter": word & 0x0FFF, "flags": word >> 12,
            "ms": struct.unpack_from("<H", h, 14)[0],
            "tail": struct.unpack_from("<I", h, 20)[0]}


def unpack_pixels(raw):
    """512 wire bytes -> 1024 values 0..15, low nibble first."""
    out = bytearray(SPOKE_PIXELS)
    out[0::2] = bytes(b & 0x0F for b in raw)
    out[1::2] = bytes(b >> 4 for b in raw)
    return out


def write_png(path, width, height, rgb_rows):
    def chunk(tag, body):
        c = struct.pack(">I", len(body)) + tag + body
        return c + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
    raw = b"".join(b"\0" + bytes(row) for row in rgb_rows)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def palette(doppler):
    """16 RGB colours for wire pixel values. In Doppler mode 0x0F is
    approaching (and 0x0E receding when showing both)."""
    pal = [(0, 0, 0)] + [(round(30 + 225 * i / 14), round(200 * (1 - i / 14) + 40), 0)
                         for i in range(15)]
    if doppler in ("both", "approaching"):
        pal[15] = (255, 0, 255)
    if doppler == "both":
        pal[14] = (0, 200, 255)
    return pal


def render_ppi(spokes, size, pal):
    """Polar spokes (angle -> 1024 pixels) to a size x size RGB image,
    angle 0 (bow) at the top, clockwise."""
    half = size / 2
    scale = SPOKE_PIXELS / half
    rows = []
    for y in range(size):
        row = bytearray(size * 3)
        dy = half - y - 0.5
        for x in range(size):
            dx = x + 0.5 - half
            r = int(math.hypot(dx, dy) * scale)
            if r >= SPOKE_PIXELS:
                continue
            a = int((math.atan2(dx, dy) / (2 * math.pi)) * SPOKES) % SPOKES
            line = spokes.get(a)
            if line is not None:
                row[3 * x:3 * x + 3] = bytes(pal[line[r]])
        rows.append(row)
    return rows


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def print_state(state):
    for name in sorted(state):
        print(f"[{name}]")
        for k, v in state[name].items():
            if isinstance(v, list):
                v = ", ".join(map(str, v))
            print(f"  {k:30} {v}")


def cmd_status(args, radar):
    radar.start(spokes=False)
    time.sleep(args.seconds)
    radar.stop()
    if not radar.state:
        print("No reports received (check firewall / multicast on this interface).")
        return 1
    print_state(radar.state)
    if args.raw:
        print("\n[raw report packets, latest of each]")
        for k in sorted(radar.raw):
            print(f"  {k:16} {radar.raw[k]}")
    return 0


def cmd_monitor(args, radar):
    """Print every report packet whose bytes changed, and optionally every
    spoke header, until Ctrl+C."""
    last = {}

    def on_packet(ch, t, data):
        ts = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"
        if ch == CHANNEL_REPORT:
            key = data[:2]
            if last.get(key) == data:
                return
            last[key] = data
            r = decode_report(data)
            print(f"{ts} report {data[1]:02X}{data[0]:02X} len {len(data)}: {data.hex(' ')}")
            if r:
                print(f"{'':13}-> {r[0]} {r[1]}")
        elif args.spokes:
            n = (len(data) - FRAME_HEADER_LEN) // LINE_LEN
            print(f"{ts} frame len {len(data)} header {data[:FRAME_HEADER_LEN].hex(' ')}, "
                  f"{n} spokes")
            for i in range(n):
                off = FRAME_HEADER_LEN + i * LINE_LEN
                h = data[off:off + LINE_HEADER_LEN]
                print(f"{'':13}{h.hex(' ')} -> {decode_line_header(h)} {spoke_meta(h)}")
        sys.stdout.flush()

    radar.listeners.append(on_packet)
    radar.start(spokes=args.spokes)
    try:
        # a replay ends with its data; a live radar runs until Ctrl+C
        while not (getattr(radar, "finished", None) and radar.finished.is_set()):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    radar.stop()
    return 0


def cmd_nav(args, radar):
    radar.start(spokes=False)
    radar.set_nav(args.heading, args.cog, args.sog)
    print(f"sending heading {args.heading} cog {args.cog} sog {args.sog} m/s "
          "(Ctrl+C to stop)")
    was = False
    try:
        while True:
            time.sleep(1)
            if radar.nav.yielding() != was:
                was = radar.nav.yielding()
                print(f"paused: {radar.nav.other_sender} is sending heading" if was
                      else "resumed sending")
    except KeyboardInterrupt:
        pass
    radar.stop()
    return 0


def cmd_export(args, _radar):
    """Recording -> CSV: one row per spoke with header fields and 1024 pixels."""
    rows = 0
    with open(args.output, "w") as out:
        out.write("time,scanner,angle,range_m,heading_deg,status,header_hex,"
                  + ",".join(f"p{i}" for i in range(SPOKE_PIXELS)) + "\n")
        for t, ch, data in read_recording(args.recording):
            scanner, kind = rec_channel(ch)
            if kind != CHANNEL_DATA:
                continue
            n = (len(data) - FRAME_HEADER_LEN) // LINE_LEN
            for i in range(n):
                off = FRAME_HEADER_LEN + i * LINE_LEN
                h = data[off:off + LINE_HEADER_LEN]
                hdr = decode_line_header(h)
                if hdr is None:
                    continue
                angle, rng, hdg = hdr
                px = unpack_pixels(data[off + LINE_HEADER_LEN:off + LINE_LEN])
                out.write(f"{t:.3f},{'AB'[scanner] if scanner < 2 else scanner},{angle},{rng},{'' if hdg is None else f'{hdg:.2f}'},"
                          f"{h[1]:#04x},{h.hex()}," + ",".join(map(str, px)) + "\n")
                rows += 1
    print(f"{rows} spokes -> {args.output}")
    return 0


def cmd_spokes(args, radar):
    q = queue.Queue()
    radar.listeners.append(lambda ch, t, d: ch == CHANNEL_DATA and q.put(d))
    radar.start()
    spokes = {}
    frames = lines = bad = dropped = 0
    statuses = {}
    last_angle = last_seq = None
    rotations = []
    rng = hdg = None
    start = time.monotonic()
    deadline = start + args.seconds
    while time.monotonic() < deadline:
        try:
            data = q.get(timeout=0.2)
        except queue.Empty:
            continue
        frames += 1
        n = (len(data) - FRAME_HEADER_LEN) // LINE_LEN
        for i in range(n):
            off = FRAME_HEADER_LEN + i * LINE_LEN
            h = data[off:off + LINE_HEADER_LEN]
            statuses[h[1]] = statuses.get(h[1], 0) + 1
            if h[1] == MARKER_SPOKE_STATUS:
                continue        # transmit start/stop marker, not a spoke
            # Bytes 2-3: 12-bit spoke counter (top nibble is a range id). A
            # big jump is the counter restarting, not lost spokes.
            seq = struct.unpack_from("<H", h, 2)[0] & 0x0FFF
            if last_seq is not None:
                gap = (seq - last_seq - 1) % 4096
                dropped += gap if gap < 256 else 0
            last_seq = seq
            hdr = decode_line_header(h)
            if hdr is None:
                bad += 1
                continue
            angle, rng, hdg = hdr
            lines += 1
            if last_angle is not None and angle < last_angle - SPOKES // 2:
                rotations.append(time.monotonic())
            last_angle = angle
            if args.output:
                spokes[angle] = unpack_pixels(data[off + LINE_HEADER_LEN:off + LINE_LEN])
    radar.stop()

    elapsed = time.monotonic() - start
    print(f"source         {radar.describe()['source']}")
    print(f"frames         {frames} ({frames / elapsed:.1f}/s)")
    print(f"spokes         {lines} valid, {bad} invalid, {dropped} dropped")
    print(f"status bytes   " + ", ".join(f"0x{k:02x}: {v}" for k, v in sorted(statuses.items())))
    if frames == 0:
        print("No spoke data. The radar only sends spokes while transmitting "
              "(see `status`), or multicast is being blocked.")
        return 1
    print(f"range          {rng} m")
    print(f"heading        {f'{hdg:.1f} deg' if hdg is not None else 'not supplied'}")
    if len(rotations) >= 2:
        period = (rotations[-1] - rotations[0]) / (len(rotations) - 1)
        print(f"rotation       {60 / period:.1f} rpm ({len(rotations) - 1} revolutions timed)")
    if args.output:
        doppler = radar.state.get("setup_extended", {}).get("doppler", "off")
        print(f"coverage       {len(spokes)}/{SPOKES} angles")
        write_png(args.output, args.size, args.size,
                  render_ppi(spokes, args.size, palette(doppler)))
        print(f"image          {args.output} ({args.size}x{args.size}, "
              f"edge = {rng} m, doppler {doppler})")
    return 0


def send_and_confirm(radar, frames, sections):
    radar.start(spokes=False)
    time.sleep(1.0)  # let the first report requests go out
    radar.send(frames)
    for f in frames:
        print(f"sent {f.hex(' ')}")
    for s in sections:
        radar.state.pop(s, None)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not all(s in radar.state for s in sections):
        time.sleep(0.2)
    time.sleep(0.5)
    radar.stop()
    print_state({s: radar.state[s] for s in sections if s in radar.state})


def cmd_power(args, radar):
    send_and_confirm(radar, power_frames(args.state), ["mode"])
    return 0


def cmd_set(args, radar):
    try:
        frames = CONTROLS[args.control][1](args.value)
    except ValueError as e:
        print(f"{args.control}: {e}", file=sys.stderr)
        return 2
    if args.control in INSTALLATION_CONTROLS:
        sections = ["config", "installation"]
    elif args.control == "raw":
        sections = ["mode", "setup", "setup_extended", "config", "installation"]
    else:
        sections = ["setup", "setup_extended"]
    send_and_confirm(radar, frames, sections)
    return 0


def cmd_record(args, radar):
    rec = Recorder(args.output)
    radars = [radar]
    if args.both:
        if len(radar.beacon["scanners"]) < 2:
            print("This radar has no range B.", file=sys.stderr)
            return 1
        other = 1 - radar.scanner
        radars.append(Radar(radar.beacon, radar.iface_ip, other))
    for r in radars:
        r.listeners.append(rec.for_scanner(r.scanner))
        r.start()
    names = "+".join("AB"[r.scanner] for r in sorted(radars, key=lambda r: r.scanner))
    print(f"recording range {names} to {args.output}"
          + (f" for {args.seconds}s" if args.seconds else " (Ctrl+C to stop)"))
    try:
        start = time.monotonic()
        while not args.seconds or time.monotonic() - start < args.seconds:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    for r in radars:
        r.stop()
    rec.close()
    print(f"{rec.count} packets, {rec.bytes / 1e6:.1f} MB")
    return 0


def cmd_rotation(args, _radar):
    """Rotation rate per scanner in time windows, from a recording, plus the
    status / mode / scan speed each scanner reported. For checking whether one
    range's settings change the shared antenna's speed."""
    windows = {}   # (scanner, window index) -> list of (t, unwrapped angle)
    unwrap = {}    # scanner -> (last angle, offset)
    events = []    # (t, scanner, text)
    last = {}      # (scanner, field) -> value
    t0 = None
    for t, ch, data in read_recording(args.recording):
        t0 = t if t0 is None else t0
        scanner, kind = rec_channel(ch)
        if kind == CHANNEL_REPORT:
            r = decode_report(data)
            if not r:
                continue
            name, fields = r
            for key in {"mode": ["status"], "setup": ["use_mode", "range_m"],
                        "setup_extended": ["scan_speed"]}.get(name, []):
                if key in fields and last.get((scanner, key)) != fields[key]:
                    last[(scanner, key)] = fields[key]
                    events.append((t, scanner, f"{key} = {fields[key]}"))
            continue
        if len(data) < FRAME_HEADER_LEN + LINE_HEADER_LEN:
            continue
        hdr = decode_line_header(data[FRAME_HEADER_LEN:FRAME_HEADER_LEN + LINE_HEADER_LEN])
        if hdr is None:
            continue
        angle = hdr[0]
        prev, offset = unwrap.get(scanner, (angle, 0))
        if angle < prev - SPOKES // 2:
            offset += SPOKES
        unwrap[scanner] = (angle, offset)
        w = int((t - t0) // args.window)
        windows.setdefault((scanner, w), []).append((t, angle + offset))

    def rpm(points):
        if len(points) < 3:
            return None
        n = len(points)
        mt = sum(p[0] for p in points) / n
        ma = sum(p[1] for p in points) / n
        num = sum((p[0] - mt) * (p[1] - ma) for p in points)
        den = sum((p[0] - mt) ** 2 for p in points)
        return None if den == 0 else num / den / SPOKES * 60

    scanners = sorted({k[0] for k in windows} | {e[1] for e in events})
    nwin = max([k[1] for k in windows] + [0]) + 1
    label = lambda i: "AB"[i] if i < 2 else str(i)
    print(f"{'t (s)':>7} " + " ".join(f"{'rpm ' + label(i):>8}" for i in scanners) + "  events")
    ev = sorted(events)
    for w in range(nwin):
        cols = []
        for i in scanners:
            v = rpm(windows.get((i, w), []))
            cols.append(f"{v:8.1f}" if v is not None else f"{'-':>8}")
        lo, hi = t0 + w * args.window, t0 + (w + 1) * args.window
        notes = "; ".join(f"{label(e[1])}: {e[2]}" for e in ev if lo <= e[0] < hi)
        print(f"{w * args.window:7.1f} " + " ".join(cols) + ("  " + notes if notes else ""))
    return 0


def cmd_serve(args, radar):
    import webserver
    return webserver.serve(radar, args.host, args.port)


# --------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--radar", default="192.168.100.231", help="radar IP")
    p.add_argument("--scanner", type=int, default=0,
                   help="0 = range A, 1 = range B (dual-range radars)")
    p.add_argument("--replay", metavar="FILE",
                   help="read a recording instead of the radar")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("discover")
    s = sub.add_parser("status")
    s.add_argument("-s", "--seconds", type=float, default=6)
    s.add_argument("--raw", action="store_true", help="also print raw report bytes")
    s = sub.add_parser("monitor", help="print every changed report (and spoke headers)")
    s.add_argument("--spokes", action="store_true", help="also print every spoke header")
    s = sub.add_parser("nav", help="send heading / COG / SOG to the radar")
    s.add_argument("--heading", type=float, help="true heading, degrees")
    s.add_argument("--cog", type=float, help="course over ground, degrees")
    s.add_argument("--sog", type=float, help="speed over ground, m/s")
    s = sub.add_parser("export", help="convert a recording's spokes to CSV")
    s.add_argument("recording")
    s.add_argument("-o", "--output", required=True)
    s = sub.add_parser("rotation", help="rotation rate per range over time, from a recording")
    s.add_argument("recording")
    s.add_argument("-w", "--window", type=float, default=1.0, help="window length, s")
    s = sub.add_parser("spokes")
    s.add_argument("-s", "--seconds", type=float, default=5)
    s.add_argument("-o", "--output", help="write one revolution to this PNG")
    s.add_argument("--size", type=int, default=800)
    s = sub.add_parser("power")
    s.add_argument("state", choices=["standby", "transmit", "off"])
    s = sub.add_parser("set", formatter_class=argparse.RawDescriptionHelpFormatter,
                       epilog="controls:\n" + "\n".join(
                           f"  {k:20} {v[0]}" for k, v in CONTROLS.items()))
    s.add_argument("control", choices=list(CONTROLS), metavar="CONTROL")
    s.add_argument("value")
    s = sub.add_parser("record")
    s.add_argument("-o", "--output", required=True)
    s.add_argument("--both", action="store_true",
                   help="record range A and range B together (tagged per range)")
    s.add_argument("-s", "--seconds", type=float, default=0, help="0 = until Ctrl+C")
    s = sub.add_parser("serve")
    s.add_argument("--host", default="127.0.0.1",
                   help="listen address (0.0.0.0 to allow other machines)")
    s.add_argument("--port", type=int, default=8090)
    args = p.parse_args()

    handlers = {"status": cmd_status, "spokes": cmd_spokes, "power": cmd_power,
                "set": cmd_set, "record": cmd_record, "serve": cmd_serve,
                "monitor": cmd_monitor, "nav": cmd_nav, "export": cmd_export,
                "rotation": cmd_rotation}

    if args.cmd in ("export", "rotation"):
        return handlers[args.cmd](args, None)
    if args.replay:
        if args.cmd not in ("status", "spokes", "serve", "monitor"):
            print("--replay works with status, spokes, monitor and serve.", file=sys.stderr)
            return 2
        return handlers[args.cmd](args, Replay(args.replay, loop=args.cmd == "serve",
                                               scanner=args.scanner))

    iface_ip = local_ip_for(args.radar)
    beacon = discover(args.radar, iface_ip)
    if beacon is None:
        print(f"No beacon from {args.radar} via {iface_ip}.", file=sys.stderr)
        return 1

    if args.cmd == "discover":
        print(f"radar          {beacon['from']} (serial {beacon['serial']})")
        print(f"local iface    {iface_ip}")
        for i, sc in enumerate(beacon["scanners"]):
            print(f"scanner {'AB'[i] if i < 2 else i}      data {fmt_addr(sc['data'])}  "
                  f"report {fmt_addr(sc['report'])}  send {fmt_addr(sc['send'])}")
        names = {SUBTYPE_DATA: "data", SUBTYPE_SEND: "send", SUBTYPE_REPORT: "report"}
        for g in beacon["services"]:
            if g["type"] != RADAR_SERVICE_TYPE:
                print(f"service {g['type']:#06x}/{g['sub']}  " + "  ".join(
                    f"{names.get(k, hex(k))} {fmt_addr(v)}" for k, v in g["addrs"].items()))
        return 0
    if args.scanner >= len(beacon["scanners"]):
        print(f"Radar has only {len(beacon['scanners'])} scanner(s).", file=sys.stderr)
        return 1
    return handlers[args.cmd](args, Radar(beacon, iface_ip, args.scanner))


if __name__ == "__main__":
    sys.exit(main())
