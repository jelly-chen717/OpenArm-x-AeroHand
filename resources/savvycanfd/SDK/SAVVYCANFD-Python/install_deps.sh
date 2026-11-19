#!/usr/bin/env bash
# Optional: install Python dev tools for the examples. The core usb_can
# package has zero required dependencies — pure ctypes.
set -e

PIP="${PIP:-python3 -m pip}"

echo "==> usb_can has no required runtime dependencies."
echo "    Optional tooling: pytest (run tests), ipython (interactive)."
echo
read -r -p "Install pytest + ipython? (y/N): " choice
case "${choice,,}" in
    y|yes) $PIP install --upgrade pip; $PIP install pytest ipython ;;
    *) echo "Skipped." ;;
esac
echo "Done."
