#!/bin/bash
# Start APE UI (the RWR station dashboard), then open http://localhost:8765
cd "${RWR_WS:-$HOME/rwr_humble_ws}" && exec pixi run python "$(dirname "$(readlink -f "$0")")/server.py" "$@"
