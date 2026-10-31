import os
import sys, time, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import navico
iface = navico.local_ip_for("192.168.100.231")
b = navico.discover("192.168.100.231", iface)
r = navico.Radar(b, iface, 0)
seen = collections.Counter()
phase = ["before"]
def on(ch, t, d):
    if ch != navico.CHANNEL_DATA: return
    h = navico.decode_line_header(d[8:32])
    if h: seen[(phase[0], None if h[2] is None else h[2])] += 1
r.listeners.append(on)
r.start()
time.sleep(3)
phase[0] = "s0"; r.set_nav(heading=0.0); time.sleep(4); phase[0] = "s90"; r.set_nav(heading=90.0, cog=88.0, sog=2.5); time.sleep(5)
phase[0] = "s200"; r.set_nav(heading=200.0, cog=198.0, sog=2.5); time.sleep(5); phase[0] = "s359"; r.set_nav(heading=359.0); time.sleep(5)
phase[0] = "stopped"; r.set_nav(); time.sleep(6)
print(f"nav packets sent: {r.nav.sent}")
r.stop()
for k, v in sorted(seen.items(), key=lambda kv: str(kv[0])): print(k, v)
