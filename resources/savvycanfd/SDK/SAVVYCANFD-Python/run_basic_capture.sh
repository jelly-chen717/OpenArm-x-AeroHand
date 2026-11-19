#!/usr/bin/env bash
# Run examples/basic_capture.py using bundled libraries.
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"
PYTHONPATH="$DIR:${PYTHONPATH:-}" python3 examples/basic_capture.py "$@"
