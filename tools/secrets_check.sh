#!/usr/bin/env bash
# Fail if WiFi / credential files are tracked by git.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

bad="$(git ls-files | rg -i 'wifi_credentials\.h$|credential|\.pem$|\.p12$' || true)"
# Allow example templates only
filtered=""
while IFS= read -r f; do
  [[ -z "$f" ]] && continue
  case "$f" in
    *.example.h|*.example|*.md) ;;
    *) filtered+="$f"$'\n' ;;
  esac
done <<< "$bad"

if [[ -n "${filtered//[$'\n']/}" ]]; then
  echo "ERROR: secret-like paths are tracked by git:" >&2
  printf '%s' "$filtered" >&2
  echo "Untrack with: git rm --cached <path>  (keep local file)" >&2
  exit 1
fi

# Also catch real SSID/pass literals re-introduced under tools (heuristic)
if git grep -nE 'WIFI_PASS\s+"[^"]{4,}"' -- '*.h' '*.ino' '*.cpp' 2>/dev/null \
  | rg -v 'your-password|example|minha_senha' >/tmp/openems-secrets-hit.$$ 2>/dev/null; then
  if [[ -s /tmp/openems-secrets-hit.$$ ]]; then
    echo "ERROR: possible real WIFI_PASS literals in tracked sources:" >&2
    cat /tmp/openems-secrets-hit.$$ >&2
    rm -f /tmp/openems-secrets-hit.$$
    exit 1
  fi
fi
rm -f /tmp/openems-secrets-hit.$$

echo "secrets-check: OK"
