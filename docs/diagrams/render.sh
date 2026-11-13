#!/usr/bin/env bash
# Render docs/diagrams/src/*.html to docs/diagrams/*.png at 2x with headless Chrome.
# Usage: docs/diagrams/render.sh [name ...]    (CHROME=/path/to/chrome to override)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chrome="${CHROME:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"
[[ -x "$chrome" ]] || chrome="$(command -v google-chrome || command -v chromium || true)"
[[ -n "$chrome" ]] || { echo "Chrome/Chromium not found; set CHROME=" >&2; exit 1; }
flags=(--headless=new --disable-gpu --hide-scrollbars --no-first-run --virtual-time-budget=3000)
names=("$@"); [[ ${#names[@]} -gt 0 ]] || names=($(cd "$here/src" && ls *.html | sed 's/\.html$//'))
for name in "${names[@]}"; do
  url="file://$here/src/$name.html"
  height="$("$chrome" "${flags[@]}" --window-size=1800,1200 --dump-dom "$url" 2>/dev/null |
    sed -n 's/.*data-h="\([0-9]*\)".*/\1/p' | head -1)"
  [[ -n "$height" ]] || { echo "could not measure $name" >&2; exit 1; }
  "$chrome" "${flags[@]}" --window-size="1800,$height" --force-device-scale-factor=2 \
    --screenshot="$here/$name.png" "$url" >/dev/null 2>&1
  echo "$name.png  1800x$height (@2x)"
done
