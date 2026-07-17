#!/usr/bin/env bash
# Installation der G1 Control App auf Ubuntu 22.04.
set -euo pipefail
cd "$(dirname "$0")"

echo "== Systempakete (Qt/OpenGL-Laufzeit) =="
sudo apt-get update
sudo apt-get install -y \
    python3 python3-pip python3-venv \
    libgl1 libglu1-mesa libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-icccm4 libxcb-keysyms1 libxcb-shape0 libegl1

echo "== Virtuelle Umgebung =="
if [ ! -d .venv ]; then
    python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip

echo "== Python-Abhängigkeiten =="
pip install -r requirements.txt

echo "== Unitree SDK (unitree_sdk2_python, offiziell) =="
# Nicht auf PyPI — Installation direkt aus dem offiziellen Repository.
pip install "git+https://github.com/unitreerobotics/unitree_sdk2_python.git"

echo
echo "Fertig. Start mit:  ./run.sh"
