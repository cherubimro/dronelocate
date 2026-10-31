#!/usr/bin/env bash
# Run the headless browser test against a running `navico.py serve`.
#   tests/run_uitest.sh                 compare, then drive rain/doppler/gain and restore
#   tests/run_uitest.sh --read-only     compare only, send nothing to the radar
#   tests/run_uitest.sh --url http://127.0.0.1:8090/ --screenshot /tmp/ui.png
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ENV="$HERE/.env"
if [ ! -x "$ENV/venv/bin/python" ] || [ ! -f "$ENV/fonts.conf" ]; then
    echo "Run tests/setup_browser.sh first." >&2
    exit 1
fi
export LD_LIBRARY_PATH="$(find "$ENV/libroot" -name '*.so*' -printf '%h\n' | sort -u | tr '\n' ':')"
export FONTCONFIG_FILE="$ENV/fonts.conf"
exec "$ENV/venv/bin/python" -I "$HERE/uitest.py" "$@"
