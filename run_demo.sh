#!/usr/bin/env bash
# Bring up the whole site: supernode, ten sensor nodes, and the scene.
#
#   ./run_demo.sh                 all ten nodes emulated
#   ./run_demo.sh --hw n05        n05 uses the real dongle via SoapySDR
#   ./run_demo.sh --no-clock      show what uncalibrated clocks do
#
# Logs land in ./logs. Console at http://localhost:8080
set -u

# Pick an interpreter that can actually import the deps, rather than assuming
# `python3` is the right one. On the Debian NUC it is (3.11) and the first
# candidate wins. On the openSUSE laptop `python3` is 3.6.15 and cannot import
# zenoh, so we fall through to python3.11. Override with PYTHON=... if needed.
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

# Python block-buffers stdout when it is a pipe or a file, so the supernode's
# 5-second stats line sat in a 4 KB buffer and logs/supernode.log stayed empty
# for minutes -- and `tail -f` on it showed nothing. Invisible when running
# straight in a terminal, fatal under `docker logs`, which is the only view a
# container gives you. smoke_test.sh always set this; run_demo.sh did not.
export PYTHONUNBUFFERED=1

CONFIG=${CONFIG:-config/site.json}
LOGDIR=${LOGDIR:-logs}
HW_NODE=""
EXTRA=""

while [ $# -gt 0 ]; do
  case "$1" in
    --hw) HW_NODE="$2"; shift 2 ;;
    --no-clock) EXTRA="--no-clock-correction"; shift ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
done

mkdir -p "$LOGDIR"
rm -f "$LOGDIR"/*.log
PIDS=()

cleanup() {
  echo
  echo "stopping ${#PIDS[@]} processes"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "starting supernode"
"$PY" supernode.py --config "$CONFIG" $EXTRA > "$LOGDIR/supernode.log" 2>&1 &
PIDS+=($!)
sleep 3

NODES=$("$PY" -c "import json;print(' '.join(n['id'] for n in json.load(open('$CONFIG'))['nodes']))")
for n in $NODES; do
  if [ "$n" = "$HW_NODE" ]; then
    echo "starting $n (real SDR)"
    "$PY" node.py --config "$CONFIG" --node "$n" --source rtlsdr > "$LOGDIR/$n.log" 2>&1 &
  else
    echo "starting $n (emulated)"
    "$PY" node.py --config "$CONFIG" --node "$n" --source sim > "$LOGDIR/$n.log" 2>&1 &
  fi
  PIDS+=($!)
done
sleep 3

echo "starting scene"
"$PY" scene.py --config "$CONFIG" > "$LOGDIR/scene.log" 2>&1 &
PIDS+=($!)

echo
echo "console:  http://localhost:8080"
echo "logs:     $LOGDIR/"
echo "ctrl-c to stop"
echo
sleep 2
tail -f "$LOGDIR/supernode.log"
