#!/usr/bin/env bash
# Startet die G1 Control App (nach ./install.sh).
set -euo pipefail
cd "$(dirname "$0")"
if [ -d .venv ]; then
    # shellcheck disable=SC1091
    source .venv/bin/activate
fi
exec python3 -m g1control "$@"
