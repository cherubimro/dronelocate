#!/usr/bin/env bash
# Single-shot pipeline test: launch, observe, diagnose, tear down.
set -u
cd "$(dirname "$0")"

# Same interpreter probe as run_demo.sh -- see the comment there.
PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  for _c in python3 python3.11 python3.12 python3.13; do
    command -v "$_c" >/dev/null 2>&1 || continue
    if "$_c" -c 'import zenoh, cbor2, numpy' >/dev/null 2>&1; then PY="$_c"; break; fi
  done
fi
if [ -z "$PY" ]; then
  echo "No Python with zenoh + cbor2 + numpy found." >&2
  echo "Try: pip install -r requirements.txt   (or PYTHON=/path/to/python)" >&2
  exit 1
fi
echo "using $PY ($("$PY" -V 2>&1))"

mkdir -p logs; rm -f logs/*.log
PIDS=()
cleanup(){ for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

DUR=${1:-30}

PYTHONUNBUFFERED=1 "$PY" supernode.py --config config/site.json > logs/supernode.log 2>&1 & PIDS+=($!)
sleep 4
for n in n01 n02 n03 n04 n05 n06 n07 n08 n09 n10; do
  PYTHONUNBUFFERED=1 "$PY" node.py --node "$n" --source sim > "logs/$n.log" 2>&1 & PIDS+=($!)
done
sleep 4
PYTHONUNBUFFERED=1 "$PY" scene.py > logs/scene.log 2>&1 & PIDS+=($!)
sleep 3

# bus spy
timeout $((DUR-6)) "$PY" - <<'EOF' > logs/spy.log 2>&1 &
import time, zenoh
from dronelocate import zconf, proto
s = zenoh.open(zconf.spoke("127.0.0.1", 7447))
seen, first = {}, {}
def mk(tag):
    def cb(sample):
        seen[tag] = seen.get(tag, 0) + 1
        if tag not in first:
            try: first[tag] = str(proto.decode(sample.payload))[:180]
            except Exception as e: first[tag] = f"decode fail: {e}"
    return cb
for ke, tag in [("dc/tm1/sim/truth","truth"), ("dc/tm1/*/evt/detect","detect"),
                ("dc/tm1/*/health","health"), ("dc/tm1/track/*","track")]:
    s.declare_subscriber(ke, mk(tag))
time.sleep(18)
for k,v in first.items(): print(f"first {k}: {v}")
print("COUNTS:", seen)
EOF
SPY=$!

sleep "$DUR"
echo "===== SPY ====="; cat logs/spy.log 2>/dev/null
echo "===== SUPERNODE ====="; tail -5 logs/supernode.log
echo "===== NODE n03 ====="; tail -5 logs/n03.log
echo "===== SCENE ====="; tail -2 logs/scene.log
