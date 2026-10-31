#!/usr/bin/env bash
# Push this repository to a set of Raspberry Pis (or any Linux hosts) and run
# one sensor node on each under systemd.
#
#   ./deploy.sh --hub 192.168.1.10 pi-a:n01 pi-b:n03 pi-c:n10:uhd
#   ./deploy.sh --status pi-a:n01 pi-b:n03
#   ./deploy.sh --stop   pi-a:n01
#
# Each argument is host:node[:source]. The node id must exist in the site
# config; source defaults to sim.
#
# Nothing here is required to distribute the fleet -- node.py already takes
# --hub and --port, and the supernode already binds 0.0.0.0. This only
# removes the tedium: shipping the same tree everywhere, keeping one config
# byte-identical across hosts, and restarting nodes that die.
#
# The supernode and scene.py are NOT deployed. Exactly one of each must run,
# somewhere reachable, and putting that decision in a loop over hosts is how
# you end up with two scene.py instances publishing rival truth streams --
# which looks like the tracker randomly losing the target and says nothing
# useful in any log.
set -euo pipefail

HUB=""
PORT=7447
CONFIG="config/site.json"
DIR="/opt/dronelocate"
ENVDIR="/etc/default"
SSH_USER=""
PYTHON="/usr/bin/python3"
MODE="deploy"
DRYRUN=0

die() { echo "deploy: $*" >&2; exit 1; }

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --hub)     HUB="$2"; shift 2 ;;
    --port)    PORT="$2"; shift 2 ;;
    --config)  CONFIG="$2"; shift 2 ;;
    --dir)     DIR="$2"; shift 2 ;;
    --user)    SSH_USER="$2"; shift 2 ;;
    --python)  PYTHON="$2"; shift 2 ;;
    --status)  MODE="status"; shift ;;
    --stop)    MODE="stop"; shift ;;
    --logs)    MODE="logs"; shift ;;
    --dry-run) DRYRUN=1; shift ;;
    -h|--help) usage 0 ;;
    -*)        die "unknown option $1" ;;
    *)         TARGETS+=("$1"); shift ;;
  esac
done

[ "${#TARGETS[@]}" -gt 0 ] || usage 1
[ "$MODE" != "deploy" ] || [ -n "$HUB" ] || die "--hub is required to deploy"

# Two ways to ship. From a git clone we send exactly the committed tree,
# which is reproducible and lets us warn when your edits are not in it. From
# an unpacked release tarball there is no .git at all -- and that is the
# normal case for whoever received the tarball, so refusing there would make
# this script useless in the form it is actually distributed.
if command -v git >/dev/null 2>&1 && git rev-parse --git-dir >/dev/null 2>&1; then
  SHIP=git
else
  SHIP=dir
  echo "note: not a git checkout -- shipping this directory as-is." >&2
  echo "      Every host gets a byte-identical copy, which is what matters;" >&2
  echo "      you just lose the record of which commit it came from." >&2
  echo >&2
fi

# Uncommitted work is a silent way for two hosts to disagree, and hosts that
# disagree about node positions are the hardest failure this system has: the
# solver trusts surveyed positions 1:1, so a stale config correlates
# beautifully and still lies.
if [ "$SHIP" = git ] && [ "$MODE" = "deploy" ] && ! git diff-index --quiet HEAD --; then
  echo "warning: working tree is dirty; deploying committed HEAD, not your edits" >&2
  git status --short >&2
  echo >&2
fi

[ -f "$CONFIG" ] || die "config not found: $CONFIG"
if [ "$SHIP" = git ] && ! git ls-files --error-unmatch "$CONFIG" >/dev/null 2>&1; then
  die "$CONFIG is not committed, so it would not be in the shipped tree"
fi

# One tar stream on stdout, whichever source we are shipping from.
ship_tree() {
  if [ "$SHIP" = git ]; then
    git archive --format=tar HEAD
  else
    tar c --exclude=.git --exclude=logs --exclude='*.pyc' \
          --exclude=__pycache__ --exclude='.venv' --exclude='*.log' .
  fi
}

run() {  # run <host> <command...>
  local host="$1"; shift
  if [ "$DRYRUN" = 1 ]; then
    echo "  [dry-run] ssh $host $*"
    return 0
  fi
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$host" "$@"
}

for target in "${TARGETS[@]}"; do
  IFS=: read -r host node source <<<"$target"
  [ -n "${host:-}" ] && [ -n "${node:-}" ] || die "bad target '$target' (want host:node[:source])"
  source="${source:-sim}"
  case "$source" in sim|uhd|rtlsdr) ;; *) die "bad source '$source' in '$target'" ;; esac
  sshhost="$host"
  [ -z "$SSH_USER" ] || sshhost="$SSH_USER@$host"

  case "$MODE" in
    status)
      echo "== $host ($node)"
      run "$sshhost" "systemctl is-active dronelocate-node@$node 2>/dev/null || true; \
                      systemctl show -p NRestarts --value dronelocate-node@$node 2>/dev/null \
                        | sed 's/^/  restarts: /' || true"
      continue ;;
    stop)
      echo "== $host ($node): stopping"
      run "$sshhost" "sudo systemctl disable --now dronelocate-node@$node 2>/dev/null || true"
      continue ;;
    logs)
      echo "== $host ($node)"
      run "$sshhost" "journalctl -u dronelocate-node@$node -n 20 --no-pager 2>/dev/null || true"
      continue ;;
  esac

  echo "== $host -> node $node (source $source)"

  # Sanity-check the node id against the config we are about to ship, before
  # touching the remote host. node.py raises KeyError for an unknown id, but
  # it does so after systemd has already enabled a unit that can never start.
  python3 - "$CONFIG" "$node" <<'PY' || die "node id not in config"
import json, sys
cfg, want = sys.argv[1], sys.argv[2]
ids = [n["id"] for n in json.load(open(cfg))["nodes"]]
if want not in ids:
    print(f"  node '{want}' is not in {cfg}; have: {', '.join(ids)}", file=sys.stderr)
    raise SystemExit(1)
PY

  if [ "$DRYRUN" = 1 ]; then
    if [ "$SHIP" = git ]; then
      echo "  [dry-run] would ship $(git rev-parse --short HEAD) to $host:$DIR"
    else
      echo "  [dry-run] would ship this directory to $host:$DIR"
    fi
  else
    run "$sshhost" "sudo mkdir -p '$DIR' && sudo chown \$(id -un):\$(id -gn) '$DIR'"
    ship_tree | ssh -o BatchMode=yes "$sshhost" "tar x -C '$DIR'"
  fi

  # Shared env file, then the per-node override carrying only what differs.
  run "$sshhost" "sudo tee '$ENVDIR/dronelocate' >/dev/null <<'EOF'
HUB=$HUB
PORT=$PORT
CONFIG=$CONFIG
PYTHON=$PYTHON
SOURCE=sim
EOF"
  if [ "$source" != "sim" ]; then
    run "$sshhost" "sudo tee '$ENVDIR/dronelocate-$node' >/dev/null <<'EOF'
SOURCE=$source
EOF"
  else
    run "$sshhost" "sudo rm -f '$ENVDIR/dronelocate-$node'"
  fi

  # Fill the unit template. User is the account we logged in as, so the node
  # runs unprivileged; the usrp supplementary group in the unit is what gets
  # it to the radio.
  if [ "$DRYRUN" = 1 ]; then
    echo "  [dry-run] would install dronelocate-node@.service and start $node"
    continue
  fi
  remote_user=$(run "$sshhost" "id -un")
  sed -e "s|@@DIR@@|$DIR|g" -e "s|@@USER@@|$remote_user|g" -e "s|@@ENVDIR@@|$ENVDIR|g" \
      deploy/dronelocate-node@.service \
    | ssh -o BatchMode=yes "$sshhost" "sudo tee /etc/systemd/system/dronelocate-node@.service >/dev/null"

  run "$sshhost" "sudo systemctl daemon-reload \
                  && sudo systemctl enable --now dronelocate-node@$node \
                  && sleep 2 \
                  && systemctl is-active dronelocate-node@$node"
done

if [ "$MODE" = "deploy" ]; then
  cat <<EOF

Nodes started. The supernode and scene.py are not deployed -- run exactly one
of each, on a host the nodes can reach:

  python3 supernode.py --config $CONFIG --bind 0.0.0.0
  python3 scene.py     --config $CONFIG --hub $HUB     # simulation only

  ./deploy.sh --status ${TARGETS[*]}
  ./deploy.sh --logs   ${TARGETS[*]}
EOF
fi
