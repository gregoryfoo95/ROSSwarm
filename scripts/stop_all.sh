#!/usr/bin/env bash
set -euo pipefail
echo "[kill] Killing common sim processes …"
pkill -f arducopter     2>/dev/null || true
pkill -f mavproxy.py    2>/dev/null || true
pkill -f "gz sim"       2>/dev/null || true
echo "[kill] Done."
