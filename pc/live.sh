#!/bin/sh
# Launch the live radar server with the RIGHT interpreter and the RIGHT range grid.
#   ./live.sh            -> live mode
#   ./live.sh <args...>  -> passed through to radar_server.py
# WHY this exists: plain `python` on this Mac is system 3.14 (no pyserial) -> the reader
# thread dies with ModuleNotFoundError while the process stays up looking alive. And
# RANGE_STEP must match the FLASHED cfg (firmware dR = 0.1065 m/bin); the server default
# 0.085 aims the cube ~1.1 m past the body at 4-5 m (live 0723: bin 52 = 5.54 m for a body
# at 4.39 m, and z40 came back NEGATIVE on the mis-aimed bin).
cd "$(dirname "$0")" || exit 1
exec env RANGE_STEP="${RANGE_STEP:-0.107}" ./.venv/bin/python web/radar_server.py "${@:-live}"
