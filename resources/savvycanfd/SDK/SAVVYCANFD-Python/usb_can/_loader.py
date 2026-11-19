"""Locate and load the usb_can shared library on Windows / Linux / macOS."""
from __future__ import annotations

import ctypes
import os
import platform
import sys
from pathlib import Path
from typing import List


_LOADED: "ctypes.CDLL | None" = None


def _platform_subdir() -> str:
    """Map (sys.platform, arch) onto a subdir inside ``usb_can/_libs/``."""
    arch = platform.machine().lower()
    if sys.platform == "win32":
        return "windows-x64"
    if sys.platform == "darwin":
        return "macos-arm64" if arch in ("arm64", "aarch64") else "macos-x64"
    if arch in ("aarch64", "arm64"):
        return "linux-arm64"
    return "linux-x64"


def _candidate_paths() -> List[Path]:
    """Search order for the shared library."""
    here = Path(__file__).resolve().parent
    pkg_root = here.parent           # python/
    sdk_root = pkg_root.parent       # repo root

    if sys.platform == "win32":
        names = ["usb_can.dll"]
        sub = ["bin"]
        env_var = "USB_CAN_LIB"
    elif sys.platform == "darwin":
        names = ["libusb_can.dylib", "libusb_can.5.dylib", "libusb_can.5.0.0.dylib"]
        sub = ["lib"]
        env_var = "USB_CAN_LIB"
    else:
        names = ["libusb_can.so.5", "libusb_can.so", "libusb_can.so.5.0.0"]
        sub = ["lib"]
        env_var = "USB_CAN_LIB"

    paths: List[Path] = []

    override = os.environ.get(env_var)
    if override:
        paths.append(Path(override))

    bundled = here / "_libs" / _platform_subdir()
    search_dirs = [
        bundled,
        here,
        pkg_root,
        sdk_root,
        *(sdk_root / s for s in sub),
        *(pkg_root / s for s in sub),
    ]
    for d in search_dirs:
        for n in names:
            paths.append(d / n)

    return paths


def _preload_dependencies() -> None:
    """Pull libusb-1.0.dll into the loader cache on Windows so usb_can.dll
    finds it. No-op elsewhere."""
    if sys.platform != "win32":
        return
    here = Path(__file__).resolve().parent
    candidates = [
        here / "_libs" / _platform_subdir(),
        here,
        here.parent,
        here.parent.parent / "bin",
    ]
    for d in candidates:
        cand = d / "libusb-1.0.dll"
        if cand.is_file():
            try:
                ctypes.WinDLL(str(cand))
            except OSError:
                pass
            if hasattr(os, "add_dll_directory"):
                try:
                    os.add_dll_directory(str(d))
                except OSError:
                    pass
            return


def load_library() -> ctypes.CDLL:
    """Return the loaded shared library (cached). Raises OSError on failure."""
    global _LOADED
    if _LOADED is not None:
        return _LOADED

    _preload_dependencies()

    last_error: "Exception | None" = None
    for p in _candidate_paths():
        if not p.is_file():
            continue
        try:
            _LOADED = ctypes.CDLL(str(p))
            return _LOADED
        except OSError as e:
            last_error = e

    arch = platform.machine()
    searched = "\n  ".join(str(p) for p in _candidate_paths()[:6])
    raise OSError(
        f"Failed to locate usb_can shared library ({sys.platform}/{arch}).\n"
        f"Set USB_CAN_LIB env var, or place the library next to one of:\n  {searched}\n"
        f"Last error: {last_error}"
    )
