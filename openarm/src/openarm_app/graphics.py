"""Record the launch environment and a same-process GLFW OpenGL probe."""
import ctypes
import getpass
import json
import os
from pathlib import Path
import sys
import time
import re


def configure_windows_gpu_preference(directory="logs"):
    """Set only this interpreter's two paths in the invoking user's registry."""
    if sys.platform != "win32":
        return
    import winreg
    registry_path = r"Software\Microsoft\DirectX\UserGpuPreferences"
    paths = sorted({sys.executable, sys._base_executable})
    changes = []
    with winreg.CreateKeyEx(winreg.HKEY_CURRENT_USER, registry_path, 0,
                            winreg.KEY_READ | winreg.KEY_WRITE) as key:
        for path in paths:
            try:
                old, kind = winreg.QueryValueEx(key, path)
            except FileNotFoundError:
                old, kind = None, winreg.REG_SZ
            if old is not None and (kind != winreg.REG_SZ or not isinstance(old, str)):
                raise RuntimeError(f"Unexpected GPU preference type for {path}")
            parts = [p for p in (old or "").split(";") if p and not re.match(r"^GpuPreference=", p, re.I)]
            new = ";".join(parts + ["GpuPreference=2"]) + ";"
            if old != new:
                changes.append({"path": path, "previous": old, "new": new})
        if changes:
            folder = Path(directory)
            folder.mkdir(parents=True, exist_ok=True)
            backup = folder/f"gpu-preferences-user-{time.time_ns()}.json"
            backup.write_text(json.dumps(changes, indent=2), encoding="utf-8")
            for change in changes:
                winreg.SetValueEx(key, change["path"], 0, winreg.REG_SZ, change["new"])
            print(f"GPU preference backup: {backup.resolve()}", flush=True)
        for path in paths:
            value = winreg.QueryValueEx(key, path)[0]
            print(f"High-performance GPU preference: {path} -> {value}", flush=True)


def graphics_report():
    import glfw
    from OpenGL.GL import glGetString, GL_VENDOR, GL_RENDERER, GL_VERSION
    report = {"pid": os.getpid(), "parent_pid": os.getppid(), "user": getpass.getuser(),
              "python": sys.executable, "base_python": sys._base_executable,
              "cwd": os.getcwd(), "probe_scope": "hidden GLFW context in the viewer process; not a frame-rate measurement",
              "environment": {name: os.environ.get(name) for name in
                              ("SESSIONNAME", "MUJOCO_GL", "PYOPENGL_PLATFORM", "__COMPAT_LAYER")}}
    if sys.platform == "win32":
        import winreg
        executable = ctypes.create_unicode_buffer(32768)
        ctypes.windll.kernel32.GetModuleFileNameW(None, executable, len(executable))
        report["process_executable"] = executable.value
        report["administrator_token"] = bool(ctypes.windll.shell32.IsUserAnAdmin())
        report["gpu_preferences"] = {}
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\DirectX\UserGpuPreferences") as key:
                for path in {sys.executable, sys._base_executable, executable.value}:
                    try:
                        report["gpu_preferences"][path] = winreg.QueryValueEx(key, path)[0]
                    except FileNotFoundError:
                        report["gpu_preferences"][path] = None
        except FileNotFoundError:
            report["gpu_preferences_key_missing"] = True
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed")
    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(64, 64, "OpenArm graphics probe", None, None)
        if not window:
            raise RuntimeError("OpenGL context creation failed")
        glfw.make_context_current(window)
        for key, value in (("vendor", GL_VENDOR), ("renderer", GL_RENDERER), ("opengl_version", GL_VERSION)):
            report[key] = glGetString(value).decode()
    finally:
        if window:
            glfw.destroy_window(window)
        glfw.default_window_hints()
        glfw.terminate()
    return report


def record_graphics(directory):
    report = graphics_report()
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/"graphics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"OpenGL renderer: {report['renderer']} | PID: {report['pid']}", flush=True)
    print(f"Graphics log: {(directory/'graphics.json').resolve()}", flush=True)
    return report


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Configure or inspect OpenArm graphics in the current user's environment")
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("--output", default="logs/graphics-check")
    args = parser.parse_args()
    if args.configure:
        configure_windows_gpu_preference()
    else:
        record_graphics(args.output)
