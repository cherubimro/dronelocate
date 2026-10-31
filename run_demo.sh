#!/usr/bin/env bash
# Bring up the whole site: supernode, ten sensor nodes, and the scene.
#
#   ./run_demo.sh                 all ten nodes emulated
#   ./run_demo.sh --hw n05        n05 uses the real dongle via SoapySDR
#   ./run_demo.sh --hw n10 --hw-source uhd     n10 is a real B210 via UHD
#   ./run_demo.sh --no-clock      show what uncalibrated clocks do
#   ./run_demo.sh --radar         also an emulated radar (radarnode.py --source
#                                 sim, from the site's radars[] block), fused
#   ./run_demo.sh --radar-live    the REAL radar instead: radarnode.py live,
#                                 radar IP from radars[].ip (else 192.168.100.231),
#                                 waits until it answers, radar console
#                                 (testradar's viewer.html) on :8082. navico
#                                 and the console come from the testradar/
#                                 subtree ($TESTRADAR overrides)
#   ./run_demo.sh --radar-replay FILE.navrec
#                                 a recorded HALO session through the live
#                                 path, looping, radar console on :8082 --
#                                 the console without the radar. Fuse starts
#                                 OFF: recorded echoes are a real room, not
#                                 the simulated sky -- unless the recording
#                                 came from tools/make_radar_rec.py: that is
#                                 the sim drone, played in step with the
#                                 scene, and Fuse stays ON
#
# A hardware node is a live sensor shown in the fleet, not a contributor to
# the fix: it never even joins a burst. The supernode groups detections by
# burst id, and the emulated nodes take theirs from scene.py's counter while
# a UHD node takes its UTC slot index, so the hardware node's detections form
# groups of one, below solver.min_nodes, and its IQ is never pulled. (Were
# they grouped, it hears the real band and they the simulated emitter, so the
# correlations could not match anyway.) That is the point -- it exercises the
# whole node path, transport included, against real silicon. Its spectrum is
# on the dashboard (port 8081), ~70 dB below the emulated nodes: a real radio
# reports fractions of ADC full scale, the sim arbitrary units.
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
HW_SOURCE="rtlsdr"        # default keeps `--hw n05` behaving as it always has
HW_SOURCE_GIVEN=0
FORCE=0
EXTRA=""
RADAR=0

while [ $# -gt 0 ]; do
  case "$1" in
    --hw) HW_NODE="$2"; shift 2 ;;
    --hw-source) HW_SOURCE="$2"; HW_SOURCE_GIVEN=1; shift 2 ;;
    --no-clock) EXTRA="--no-clock-correction"; shift ;;
    --force) FORCE=1; shift ;;
    --radar) RADAR=1; shift ;;
    --radar-live) RADAR=live; shift ;;
    --radar-replay) RADAR=replay; RADAR_REC="$2"; shift 2 ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
done

case "$HW_SOURCE" in
  rtlsdr|uhd) ;;
  *) echo "--hw-source must be 'rtlsdr' or 'uhd' (got '$HW_SOURCE')" >&2; exit 1 ;;
esac

# Refuse the combination that silently does nothing, rather than starting ten
# emulated nodes and letting the operator conclude the radio is broken.
if [ "$HW_SOURCE_GIVEN" = 1 ] && [ -z "$HW_NODE" ]; then
  echo "--hw-source needs --hw <node>; on its own it selects nothing." >&2
  exit 1
fi

# Preflight the UHD path. Both failures below are otherwise reported deep in
# the node log as an import error or "USB open failed: insufficient
# permissions", by which point the fleet is already up and the operator is
# reading the wrong log.
if [ -n "$HW_NODE" ] && [ "$HW_SOURCE" = "uhd" ]; then
  # The UHD bindings are a system package, not a pip dependency, so the
  # interpreter chosen above may not have them. On a box where UHD was built
  # against 3.11 by hand, env-uhd-py311.sh is what puts it on the path; it is
  # gitignored because its paths are machine specific, so absence is normal.
  if ! "$PY" -c 'import uhd' >/dev/null 2>&1 && [ -f ./env-uhd-py311.sh ]; then
    echo "uhd not importable under $PY; sourcing ./env-uhd-py311.sh"
    # shellcheck disable=SC1091
    . ./env-uhd-py311.sh
  fi
  if ! "$PY" -c 'import uhd' >/dev/null 2>&1; then
    echo "No 'uhd' module for $PY ($(command -v "$PY"))." >&2
    echo "  Debian/Ubuntu: sudo apt install uhd-host python3-uhd" >&2
    echo "                 sudo uhd_images_downloader   # B210 firmware+FPGA" >&2
    echo "  built from source: source ./env-uhd-py311.sh first" >&2
    echo "  (see docs/uhd-py311-build.md)" >&2
    # The common Debian/Ubuntu failure is not a missing package but a split
    # between two interpreters: pip put zenoh in a venv (so the probe above
    # picked the venv python), while apt put uhd in the SYSTEM python's
    # dist-packages, which a venv hides by default. Each interpreter then has
    # half of what is needed. Say so, because "apt install python3-uhd"
    # reports "already the newest version" and the script still refuses.
    if "$PY" -c 'import sys; sys.exit(0 if sys.prefix != sys.base_prefix else 1)' 2>/dev/null; then
      echo >&2
      echo "  NOTE: $PY is a virtualenv, which hides apt's python3-uhd." >&2
      echo "  Either recreate it with --system-site-packages, or install the" >&2
      echo "  pip deps into the system python and use that:" >&2
      echo "    pip install --break-system-packages -r requirements.txt" >&2
      echo "    PYTHON=/usr/bin/python3 $0 $*" >&2
    elif command -v /usr/bin/python3 >/dev/null 2>&1 \
         && /usr/bin/python3 -c 'import uhd' >/dev/null 2>&1; then
      echo >&2
      echo "  NOTE: /usr/bin/python3 CAN import uhd but was not picked, so it" >&2
      echo "  is missing zenoh/cbor2/numpy. Install those there and re-run:" >&2
      echo "    pip install --break-system-packages -r requirements.txt" >&2
      echo "    PYTHON=/usr/bin/python3 $0 $*" >&2
    fi
    exit 1
  fi
  # A USRP's device node is owned by a group, not by the invoking user. Warn
  # rather than fail: the group name is distribution-specific and the node
  # may legitimately be run as root.
  if [ "$(id -u)" != 0 ] && getent group usrp >/dev/null 2>&1 \
     && ! id -nG | tr ' ' '\n' | grep -qx usrp; then
    echo "warning: $(id -un) is not in group 'usrp'; the radio will fail to open." >&2
    echo "         fix: sudo usermod -aG usrp $(id -un)   (then log out and back in)" >&2
    echo "         or run this script under: sg usrp -c '...'" >&2
  fi
fi

# Refuse to stack a second fleet on a running one.
#
# A run that dies partway leaves the rest alive: a hardware node raising
# takes out only that node, and the cleanup trap below fires only when THIS
# script exits, so closing the terminal leaves everything running. Start
# again and the second supernode cannot bind 7447 and gives up -- but the
# second scene.py and the second set of sensor nodes come up anyway and
# attach to the FIRST supernode. You then have several truth publishers,
# each with its own burst counter, plus duplicate nodes serving IQ. The
# tracker sees what look like several emitters and, with a 2.5 km
# association gate, visibly catches the target and loses it again.
#
# Measured during a real bring-up: 5 bursts/s against a configured 1, and
# 11.4 Mbps ingest against a ~4 Mbps baseline, with bursts and fixes in
# perfect lockstep -- i.e. the pipeline was healthy and simply had too many
# scenes feeding it. Nothing in the logs says "you are running two fleets",
# which is what makes it worth refusing up front.
#
# This check must run BEFORE the log wipe below, or starting a second fleet
# would delete the running one's logs -- destroying the only evidence.
#
# Patterns are precise on purpose: "node.py" is a substring of
# "supernode.py", so the node pattern demands a separator in front of it.
# That exact trap cost real debugging time.
port_busy() {
  if command -v ss >/dev/null 2>&1; then
    ss -tln 2>/dev/null | grep -qE "[:.]$1[[:space:]]"
  elif command -v netstat >/dev/null 2>&1; then
    netstat -tln 2>/dev/null | grep -qE "[:.]$1[[:space:]]"
  else
    return 1
  fi
}
n_super=$(pgrep -cf "[p]ython.*supernode\.py" 2>/dev/null || true)
n_node=$(pgrep -cf "[p]ython.*[ /]node\.py" 2>/dev/null || true)
n_scene=$(pgrep -cf "[p]ython.*scene\.py" 2>/dev/null || true)
: "${n_super:=0}" "${n_node:=0}" "${n_scene:=0}"
if [ "$FORCE" != 1 ] \
   && { [ "$n_super" -gt 0 ] || [ "$n_node" -gt 0 ] || [ "$n_scene" -gt 0 ] \
        || port_busy 7447; }; then
  echo "A fleet is already running -- refusing to start a second one." >&2
  echo "  supernode.py : $n_super" >&2
  echo "  node.py      : $n_node" >&2
  echo "  scene.py     : $n_scene" >&2
  port_busy 7447 && echo "  zenoh port 7447 is in use" >&2
  echo >&2
  echo "Two fleets share one supernode and produce duplicate truth streams," >&2
  echo "which looks like the tracker losing the target at random." >&2
  echo >&2
  echo "Stop the old one first:" >&2
  echo "  pkill -f scene.py; pkill -f supernode.py; pkill -f '[ /]node.py'" >&2
  echo "Or, if you really do want a second site (different port and config):" >&2
  echo "  $0 --force ..." >&2
  exit 1
fi

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
    echo "starting $n (real SDR via $HW_SOURCE)"
    "$PY" node.py --config "$CONFIG" --node "$n" --source "$HW_SOURCE" > "$LOGDIR/$n.log" 2>&1 &
  else
    echo "starting $n (emulated)"
    "$PY" node.py --config "$CONFIG" --node "$n" --source sim > "$LOGDIR/$n.log" 2>&1 &
  fi
  PIDS+=($!)
done
sleep 3

# A recording made by tools/make_radar_rec.py is the sim drone itself, not a
# real room: its first record says so. The scene then flies on the wall clock
# and the replay plays in step with it, so Fuse stays on.
SIM_REC=0
if [ "$RADAR" = replay ] && [ -f "$RADAR_REC" ] && "$PY" - "$RADAR_REC" <<'EOF'
import gzip, json, struct, sys
p = sys.argv[1]
f = gzip.open(p) if p.endswith(".gz") else open(p, "rb")
ok = f.read(8) == b"NAVREC1\n"
if ok:
    t, ch, n = struct.unpack("<dBI", f.read(13))
    ok = ch == 255 and json.loads(f.read(n)).get("sync") == "wall-clock"
sys.exit(0 if ok else 1)
EOF
then
  SIM_REC=1
fi

echo "starting scene"
"$PY" scene.py --config "$CONFIG" $([ "$SIM_REC" = 1 ] && echo --wall-clock) \
  > "$LOGDIR/scene.log" 2>&1 &
PIDS+=($!)

if [ "$RADAR" != 0 ]; then
  RADARS=$("$PY" -c "import json;print(' '.join(r['id'] for r in json.load(open('$CONFIG')).get('radars', [])))")
  [ -z "$RADARS" ] && echo "--radar: $CONFIG has no radars[] block" >&2
fi
if [ "$RADAR" = 1 ]; then
  for r in $RADARS; do
    echo "starting $r (emulated radar)"
    "$PY" radarnode.py --source sim --site "$CONFIG" --radar-id "$r" > "$LOGDIR/$r.log" 2>&1 &
    PIDS+=($!)
  done
elif [ "$RADAR" = live ] || [ "$RADAR" = replay ]; then
  # navico + the radar console (webserver.py + viewer.html) come from the
  # testradar/ subtree in this repository; radarnode.py puts it on sys.path
  # itself (dronelocate/vendor.py). $TESTRADAR names another checkout
  # instead. This block only reports what will be used.
  RPATH=""
  if [ "$RADAR" = replay ] && [ ! -f "$RADAR_REC" ]; then
    echo "--radar-replay: no such recording: $RADAR_REC" >&2
  fi
  for d in "${TESTRADAR:-}" ./testradar; do
    [ -n "$d" ] && [ -f "$d/navico.py" ] && [ -f "$d/viewer.html" ] \
      && { RPATH=$(cd "$d" && pwd); break; }
  done
  if [ -n "$RPATH" ]; then
    echo "testradar from $RPATH (navico + radar console)"
  else
    echo "--radar-$RADAR: no testradar/navico.py + viewer.html here (partial copy" >&2
    echo "  of the repository?) and \$TESTRADAR is not set to a checkout" >&2
  fi
  # One radar per host. Range A's console on :8082 (green), range B's on
  # :8083 (navy), both on one page at the supernode's :8084. Live, range B
  # runs only if the site file lists it (radars[].scanners); a replay runs
  # both, since a dual-range recording carries both.
  r=$(echo $RADARS | cut -d' ' -f1)
  SCANNERS=$("$PY" -c "import json;r=[x for x in json.load(open('$CONFIG')).get('radars',[]) if x['id']=='$r'];print(' '.join((r[0].get('scanners') if r else None) or ['A']))")
  [ "$RADAR" = replay ] && SCANNERS="A B"
  for sc in $SCANNERS; do
    case "$sc" in A) idx=0; port=8082 ;; B) idx=1; port=8083 ;; *) continue ;; esac
    if [ "$RADAR" = replay ]; then
      echo "starting $r range $sc (REPLAY of $RADAR_REC; console on :$port)"
      "$PY" radarnode.py --site "$CONFIG" --radar-id "$r" --scanner $idx \
        --replay "$RADAR_REC" --http-port $port > "$LOGDIR/$r$sc.log" 2>&1 &
    else
      echo "starting $r range $sc (REAL radar; console on :$port)"
      "$PY" radarnode.py --site "$CONFIG" --radar-id "$r" --scanner $idx \
        --http-port $port > "$LOGDIR/$r$sc.log" 2>&1 &
    fi
    PIDS+=($!)
  done
  if [ "$RADAR" = replay ] && [ "$SIM_REC" = 1 ]; then
    echo "radar fusion ON (emulated recording of the sim drone, played in step with the scene)"
  elif [ "$RADAR" = replay ]; then
    curl -s -o /dev/null "http://localhost:8080/control?radar_fuse=0" \
      && echo "radar fusion OFF (recorded echoes; the console's Fuse button turns it on)"
  fi
fi

echo
echo "console:  http://localhost:8080"
echo "spectrum: http://localhost:8081"
case "$RADAR" in live|replay)
  echo "radar:    http://localhost:8082 range A, :8083 range B, :8084 both on one page"
  [ "$RADAR" = live ] && echo "          (consoles appear once the radar answers)" ;;
esac
echo "logs:     $LOGDIR/"
echo "ctrl-c to stop"
echo
sleep 2
tail -f "$LOGDIR/supernode.log"
