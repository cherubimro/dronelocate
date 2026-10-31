"""Exercise navico.py controls against the real radar, one change at a time,
checking the radar's own report and printing which raw report bytes changed.

usage: hwtest.py STEPFILE   (lines: control value [section.field=expected])
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import navico  # noqa: E402

RADAR = "192.168.100.231"


def raw_bytes(radar):
    return {k: bytes.fromhex(v) for k, v in radar.raw.items()}


def diff(a, b):
    out = []
    for k in sorted(set(a) | set(b)):
        x, y = a.get(k, b""), b.get(k, b"")
        if x == y:
            continue
        if len(x) != len(y):
            out.append(f"{k}: length {len(x)} -> {len(y)}")
            continue
        ch = [f"[{i}] {x[i]:02x}->{y[i]:02x}" for i in range(len(x)) if x[i] != y[i]]
        out.append(f"{k}: " + ", ".join(ch))
    return out


def lookup(state, path):
    sec, field = path.split(".")
    return state.get(sec, {}).get(field)


def main():
    iface = navico.local_ip_for(RADAR)
    beacon = navico.discover(RADAR, iface)
    radar = navico.Radar(beacon, iface, 0).start(spokes=False)
    time.sleep(4)
    ok = fail = 0
    for line in open(sys.argv[1]):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        control, value = parts[0], parts[1]
        expect = parts[2] if len(parts) > 2 else None
        before = raw_bytes(radar)
        frames = navico.CONTROLS[control][1](value)
        radar.send(frames)
        got = None
        if expect:
            path, want = expect.split("=", 1)
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                got = lookup(radar.state, path)
                if str(got) == want:
                    break
                time.sleep(0.2)
            time.sleep(1.0)
        else:
            time.sleep(3.0)
        d = diff(before, raw_bytes(radar))
        if expect:
            res = "PASS" if str(got) == want else "FAIL"
            ok += res == "PASS"
            fail += res == "FAIL"
            print(f"{res} set {control} {value}: {path} = {got!r} (want {want})")
        else:
            print(f"---- set {control} {value} (no check)")
        print(f"     sent {' | '.join(f.hex(' ') for f in frames)}")
        for x in d:
            if not x.startswith(("0xC407", "0xC40A")):  # counters, change constantly
                print(f"     changed {x}")
    radar.stop()
    print(f"\n{ok} passed, {fail} failed")


main()
