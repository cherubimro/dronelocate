#!/usr/bin/env bash
# One-time setup for tests/uitest.py, without root:
#   - Python venv with Playwright (pip bootstrapped with get-pip.py, since
#     this machine's Python has no ensurepip)
#   - Playwright's headless Chromium (goes to ~/.cache/ms-playwright)
#   - the shared libraries and fonts Chromium needs, downloaded with
#     `apt-get download` and unpacked under tests/.env/libroot
# Re-running is safe; finished steps are skipped.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ENV="$HERE/.env"
mkdir -p "$ENV/dl" "$ENV/debs" "$ENV/libroot" "$ENV/fc-cache"

if [ ! -x "$ENV/venv/bin/pip" ]; then
    python3 -m venv --without-pip "$ENV/venv"
    curl -sSL -o "$ENV/dl/get-pip.py" https://bootstrap.pypa.io/get-pip.py
    "$ENV/venv/bin/python" -I "$ENV/dl/get-pip.py" -q
fi
"$ENV/venv/bin/pip" install -q playwright
"$ENV/venv/bin/playwright" install chromium-headless-shell

# Libraries missing from a server install, plus one font family.
PKGS="libatk1.0-0t64 libatk-bridge2.0-0t64 libatspi2.0-0t64 libxcomposite1 libxdamage1
      libxfixes3 libxrandr2 libxrender1 libxi6 libxres1 libgbm1 libasound2t64
      fonts-dejavu-core"
cd "$ENV/debs"
for p in $PKGS; do
    ls "$p"_*.deb >/dev/null 2>&1 || apt-get download "$p"
done
for f in *.deb; do dpkg-deb -x "$f" "$ENV/libroot"; done

cat > "$ENV/fonts.conf" <<EOF
<?xml version="1.0"?>
<!DOCTYPE fontconfig SYSTEM "fonts.dtd">
<fontconfig>
  <dir>$ENV/libroot/usr/share/fonts</dir>
  <cachedir>$ENV/fc-cache</cachedir>
</fontconfig>
EOF

SHELL_BIN=$(ls -d ~/.cache/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-linux64/chrome-headless-shell | tail -1)
LIBS=$(find "$ENV/libroot" -name '*.so*' -printf '%h\n' | sort -u | tr '\n' ':')
if LD_LIBRARY_PATH="$LIBS" ldd "$SHELL_BIN" | grep "not found"; then
    echo "Chromium still misses the libraries above; add their packages to PKGS." >&2
    exit 1
fi
echo "Browser test environment ready in $ENV"
