#!/usr/bin/env python3
"""Record and replay the Zenoh bus, so a field session can be run again.

Why this exists. A test flight happens once; what the supernode did with it
should be reproducible afterwards -- with a new tracker, new gates, the radar
fused or not. Everything the system says to itself in pub/sub goes through
dc/{site}/**, so recording that, with reception times, and putting it back
on a bus at the same pace replays the session to any subscriber: the
supernode's fusion, the console, a scoring script.

    python3 tools/busrec.py record -o flight1.dcbus.gz [--key 'dc/field1/**'] [--hub 10.0.0.5]
    python3 tools/busrec.py info flight1.dcbus.gz
    python3 tools/busrec.py replay flight1.dcbus.gz [--speed 2] [--only plot,track]
    python3 tools/busrec.py replay flight1.dcbus.gz --listen 7448   # be the hub

What it cannot replay: the IQ pull. Bulk IQ moves as query/reply
(dc/{site}/{node}/iq), not as publications, so it never crosses a
subscriber and is not recorded; a replayed detection event therefore cannot
be re-solved by a live supernode. Replay the solver's *outputs* (tracks) and
the radar's plots, or record IQ with the nodes' own ring buffers.

File: magic b"DCBUS1\\n", then per sample
  <f64 reception time><u16 key length><key><u32 payload length><payload>
  <u32 attachment length><attachment>      (length 0xFFFFFFFF = no attachment)
gzip when the name ends in .gz. Payloads are stored as received (CBOR).
"""

import argparse
import collections
import gzip
import os
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import zenoh  # noqa: E402

from dronelocate import zconf  # noqa: E402

MAGIC = b"DCBUS1\n"
NO_ATTACHMENT = 0xFFFFFFFF


def _open(path, mode):
    return gzip.open(path, mode) if path.endswith(".gz") else open(path, mode)


def read_samples(path):
    """Yield (t, key, payload, attachment or None) from a bus recording."""
    with _open(path, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise ValueError(f"{path} is not a busrec recording")
        while True:
            h = f.read(10)
            if len(h) < 10:
                return
            t, klen = struct.unpack("<dH", h)
            key = f.read(klen).decode()
            plen, = struct.unpack("<I", f.read(4))
            payload = f.read(plen)
            alen, = struct.unpack("<I", f.read(4))
            att = None if alen == NO_ATTACHMENT else f.read(alen)
            if len(payload) < plen:
                return
            yield t, key, payload, att


def session_config(a):
    if a.listen:
        return zconf.hub(port=a.listen, bind=a.bind, tls=zconf.tls_from_args(a))
    return zconf.spoke(a.hub, a.port, tls=zconf.tls_from_args(a))


def cmd_record(a):
    s = zenoh.open(session_config(a))
    lock = threading.Lock()
    f = _open(a.output, "wb")
    f.write(MAGIC)
    counts = collections.Counter()

    def on(sample):
        t = time.time()
        key = str(sample.key_expr).encode()
        payload = bytes(sample.payload.to_bytes())
        att = sample.attachment
        att = None if att is None else bytes(att.to_bytes())
        rec = (struct.pack("<dH", t, len(key)) + key + struct.pack("<I", len(payload))
               + payload + struct.pack("<I", NO_ATTACHMENT if att is None else len(att))
               + (att or b""))
        with lock:
            f.write(rec)
            counts[key.decode()] += 1

    s.declare_subscriber(a.key, on)
    print(f"recording {a.key} -> {a.output}"
          + (f" for {a.seconds:g} s" if a.seconds else " (Ctrl+C to stop)"), file=sys.stderr)
    t0 = time.monotonic()
    try:
        while not a.seconds or time.monotonic() - t0 < a.seconds:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    s.close()
    with lock:
        f.close()
    print(f"{sum(counts.values())} samples on {len(counts)} keys", file=sys.stderr)
    return 0


def _kind(key):
    """Last meaningful key segment, for --only: detect, plot, health, track..."""
    parts = key.split("/")
    return parts[-1] if parts[-1] not in ("live",) else parts[-2]


def cmd_info(a):
    counts, first, last, size = collections.Counter(), {}, {}, collections.Counter()
    t0 = t1 = None
    for t, key, payload, att in read_samples(a.recording):
        counts[key] += 1
        size[key] += len(payload) + (len(att) if att else 0)
        first.setdefault(key, t)
        last[key] = t
        t0 = t if t0 is None else t0
        t1 = t
    if t0 is None:
        print("empty recording")
        return 1
    print(f"{sum(counts.values())} samples, {t1 - t0:.1f} s, "
          f"from {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t0))}")
    for key in sorted(counts):
        dur = last[key] - first[key]
        rate = (counts[key] - 1) / dur if dur > 0 else 0.0
        print(f"  {counts[key]:7}  {rate:7.2f}/s  {size[key] / 1e3:9.1f} kB  {key}")
    return 0


def cmd_replay(a):
    only = set(a.only.split(",")) if a.only else None
    s = zenoh.open(session_config(a))
    pubs = {}
    sent = 0
    if a.listen:
        time.sleep(a.wait)        # let subscribers connect to us first
    while True:
        start, first = time.monotonic(), None
        for t, key, payload, att in read_samples(a.recording):
            if only and _kind(key) not in only:
                continue
            if a.site_from and a.site_to:
                key = key.replace(f"dc/{a.site_from}/", f"dc/{a.site_to}/", 1)
            first = t if first is None else first
            wait = (t - first) / a.speed - (time.monotonic() - start)
            if wait > 0:
                time.sleep(wait)
            pub = pubs.get(key)
            if pub is None:
                pub = pubs[key] = s.declare_publisher(key)
            pub.put(payload, attachment=att)
            sent += 1
        if not a.loop:
            break
    time.sleep(0.5)
    s.close()
    print(f"replayed {sent} samples", file=sys.stderr)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def bus_args(p):
        zconf.add_args(p)
        p.add_argument("--listen", type=int, metavar="PORT",
                       help="act as the hub on PORT instead of connecting to --hub")
        p.add_argument("--bind", default="0.0.0.0")

    p = sub.add_parser("record")
    bus_args(p)
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--key", default="dc/**")
    p.add_argument("-s", "--seconds", type=float, default=0, help="0 = until Ctrl+C")
    p = sub.add_parser("info")
    p.add_argument("recording")
    p = sub.add_parser("replay")
    bus_args(p)
    p.add_argument("recording")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--only", help="comma list of key kinds: detect,plot,health,track,truth,...")
    p.add_argument("--site-from"), p.add_argument("--site-to")
    p.add_argument("--wait", type=float, default=2.0,
                   help="with --listen: seconds to wait for subscribers before replaying")
    a = ap.parse_args()
    return {"record": cmd_record, "info": cmd_info, "replay": cmd_replay}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
