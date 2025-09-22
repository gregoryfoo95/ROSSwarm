#!/usr/bin/env bash
set -euo pipefail

echo "[MAVProxy] Attaching to tcp:127.0.0.1:5760 → udp:127.0.0.1:14550"
exec mavproxy.py \
  --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550
