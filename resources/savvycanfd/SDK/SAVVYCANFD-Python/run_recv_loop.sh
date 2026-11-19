#!/usr/bin/env bash
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"
PYTHONPATH="$DIR:${PYTHONPATH:-}" python3 examples/recv_loop.py "$@"
