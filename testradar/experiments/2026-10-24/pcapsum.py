import gzip, struct, sys, collections
f = gzip.open(sys.argv[1], "rb")
gh = f.read(24)
le = gh[:4] in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1")
E = "<" if le else ">"
linktype = struct.unpack(E + "I", gh[20:24])[0]
flows = collections.Counter(); ops = collections.Counter(); n = 0; t0 = t1 = None
while True:
    h = f.read(16)
    if len(h) < 16: break
    ts, tu, incl, orig = struct.unpack(E + "IIII", h)
    d = f.read(incl); n += 1
    t = ts + tu / 1e6; t0 = t if t0 is None else t0; t1 = t
    off = 14 if linktype == 1 else (16 if linktype == 113 else 0)
    if linktype == 1 and d[12:14] == b"\x81\x00": off = 18
    ip = d[off:]
    if len(ip) < 28 or ip[9] != 17: continue
    ihl = (ip[0] & 15) * 4
    src = ".".join(map(str, ip[12:16])); dst = ".".join(map(str, ip[16:20]))
    sp, dp = struct.unpack(">HH", ip[ihl:ihl + 4]); pl = ip[ihl + 8:]
    flows[(src, f"{dst}:{dp}")] += 1
    if len(pl) >= 2 and pl[1] in (0xC1, 0xC2, 0xB1): ops[(src, f"{dst}:{dp}", f"{pl[1]:02X}{pl[0]:02X}")] += 1
print(f"packets {n}, duration {t1 - t0:.1f} s, linktype {linktype}")
print("flows (src -> dst:port):")
for k, v in flows.most_common(15): print(f"  {v:6} {k[0]:16} -> {k[1]}")
print("command/query opcodes seen (src, dst, opcode):")
for k, v in sorted(ops.items()): print(f"  {v:5} {k[0]:16} -> {k[1]:22} {k[2]}")
