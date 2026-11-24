"""Start one simulation scene and its matching local web controller."""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from urllib.request import ProxyHandler, build_opener
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
HTTP = build_opener(ProxyHandler({}))


def bind_windows_service_lifetime():
    """Use a job so children exit even if Windows stops the launcher abruptly."""
    import ctypes
    from ctypes import wintypes

    class BasicLimitInformation(ctypes.Structure):
        _fields_ = [
            ('PerProcessUserTimeLimit', ctypes.c_int64),
            ('PerJobUserTimeLimit', ctypes.c_int64),
            ('LimitFlags', wintypes.DWORD),
            ('MinimumWorkingSetSize', ctypes.c_size_t),
            ('MaximumWorkingSetSize', ctypes.c_size_t),
            ('ActiveProcessLimit', wintypes.DWORD),
            ('Affinity', ctypes.c_size_t),
            ('PriorityClass', wintypes.DWORD),
            ('SchedulingClass', wintypes.DWORD),
        ]

    class ExtendedLimitInformation(ctypes.Structure):
        _fields_ = [
            ('BasicLimitInformation', BasicLimitInformation),
            ('IoInfo', ctypes.c_uint64 * 6),  # Win32 IO_COUNTERS
            ('ProcessMemoryLimit', ctypes.c_size_t),
            ('JobMemoryLimit', ctypes.c_size_t),
            ('PeakProcessMemoryUsed', ctypes.c_size_t),
            ('PeakJobMemoryUsed', ctypes.c_size_t),
        ]

    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateJobObjectW.argtypes = (ctypes.c_void_p, wintypes.LPCWSTR)
    kernel.CreateJobObjectW.restype = wintypes.HANDLE
    kernel.SetInformationJobObject.argtypes = (
        wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD,
    )
    kernel.SetInformationJobObject.restype = wintypes.BOOL
    kernel.GetCurrentProcess.argtypes = ()
    kernel.GetCurrentProcess.restype = wintypes.HANDLE
    kernel.AssignProcessToJobObject.argtypes = (wintypes.HANDLE, wintypes.HANDLE)
    kernel.AssignProcessToJobObject.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
    kernel.CloseHandle.restype = wintypes.BOOL

    job = kernel.CreateJobObjectW(None, None)
    if not job:
        raise ctypes.WinError(ctypes.get_last_error())
    extended_limits_class = 9  # JobObjectExtendedLimitInformation
    kill_on_close = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    allow_breakaway = 0x0800  # JOB_OBJECT_LIMIT_BREAKAWAY_OK
    limits = ExtendedLimitInformation()
    limits.BasicLimitInformation.LimitFlags = kill_on_close | allow_breakaway
    configured = kernel.SetInformationJobObject(
        job, extended_limits_class, ctypes.byref(limits), ctypes.sizeof(limits),
    )
    if not (configured and kernel.AssignProcessToJobObject(job, kernel.GetCurrentProcess())):
        error = ctypes.WinError(ctypes.get_last_error())
        kernel.CloseHandle(job)
        raise error
    # The launcher owns the non-inheritable handle until process exit.
    # Children inherit the job itself, including the real Python behind a venv launcher.


def open_browser(url):
    if sys.platform == 'win32':
        # Keep the user's browser independent of the simulation process tree.
        subprocess.Popen(
            [sys.executable, '-m', 'webbrowser', '-t', url], stdout=subprocess.DEVNULL,
            creationflags=subprocess.CREATE_BREAKAWAY_FROM_JOB | subprocess.CREATE_NO_WINDOW,
        )
    else:
        webbrowser.open(url)


def stop_process(process):
    if process.poll() is not None:
        return
    if sys.platform == 'win32':
        # A Windows venv python.exe is a launcher with a real Python child.
        # terminate() kills only the launcher and leaves HTTP ports occupied.
        result = subprocess.run(
            ['taskkill', '/PID', str(process.pid), '/T', '/F'],
            capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW,
        )
        if result.returncode and process.poll() is None:
            raise RuntimeError(f'Could not stop service process tree {process.pid}.')
    else:
        process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def wait_ready(process, url, scene=None):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Process exited with code {process.returncode}. See startup logs.')
        try:
            with HTTP.open(url, timeout=2) as response:
                content = response.read()
            if scene is not None:
                state = json.loads(content)['result']
                if state.get('scene') != scene or state.get('api_version') != 2:
                    raise RuntimeError('Unexpected simulation scene or API version.')
            return
        except OSError:
            time.sleep(.2)
    raise TimeoutError(f'Service startup timed out: {url}')


def running_piano(sim_port, web_port):
    web_url = f'http://127.0.0.1:{web_port}'
    try:
        with HTTP.open(f'{web_url}/api/music', timeout=1) as response:
            if not isinstance(json.load(response).get('songs'), list):
                return False
        with HTTP.open(f'{web_url}/api/piano', timeout=2) as response:
            session = json.load(response).get('session')
        with HTTP.open(f'http://127.0.0.1:{sim_port}/piano/state', timeout=1) as response:
            simulation = json.load(response).get('result', {})
    except (OSError, ValueError, AttributeError):
        return False
    return isinstance(simulation, dict) and bool(session) and simulation.get('session') == session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('default', 'piano'), default='default')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--check', action='store_true', help='Check startup and close both services.')
    parser.add_argument('--reuse-running', action='store_true', help='Reopen the running piano session if available.')
    parser.add_argument('--sim-port', type=int, default=8769)
    parser.add_argument('--web-port', type=int)
    args = parser.parse_args()
    port = args.web_port or (8090 if args.scene == 'piano' else 8088)
    if not 1 <= args.sim_port <= 65535 or not 1 <= port <= 65535 or port == args.sim_port:
        parser.error('Use two distinct ports in the range 1..65535.')
    if args.reuse_running and args.scene == 'piano' and not args.check and running_piano(args.sim_port, port):
        url = f'http://127.0.0.1:{port}/'
        print(f'Piano and MuJoCo are already running: {url}', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        return 0
    # Check both ports before opening either process.
    try:
        with ExitStack() as stack:
            for number in (args.sim_port, port):
                # Windows keeps closed HTTP connections in TIME_WAIT. A live
                # listener must still be rejected before probing with reuse.
                with socket.socket() as connection:
                    connection.settimeout(.3)
                    if connection.connect_ex(('127.0.0.1', number)) == 0:
                        raise OSError(f'Port {number} already has a running service.')
                probe = stack.enter_context(socket.socket())
                if sys.platform == 'win32':
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(('127.0.0.1', number))
    except OSError as exc:
        print(f'Port unavailable. Close the previous simulation or select other ports. {exc}', flush=True)
        return 1

    logdir = ROOT / 'logs' / f'startup-{time.strftime("%Y%m%d-%H%M%S")}-{args.scene}'
    logdir.mkdir(parents=True, exist_ok=True)
    processes = []
    print(f'Logs: {logdir}', flush=True)
    try:
        if sys.platform == 'win32':
            bind_windows_service_lifetime()
        with ExitStack() as stack:
            def start(module, options, filename):
                log = stack.enter_context((logdir / filename).open('w', encoding='utf-8'))
                process = subprocess.Popen([sys.executable, '-m', module, *options], cwd=ROOT,
                                           stdout=log, stderr=subprocess.STDOUT,
                                           creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0)
                processes.append(process)
                return process

            sim_args = ['--scene', args.scene, '--port', str(args.sim_port)]
            if args.headless:
                sim_args.append('--headless')
            sim = start('openarm_app.sim_server', sim_args, 'simulation.txt')
            wait_ready(sim, f'http://127.0.0.1:{args.sim_port}/state', args.scene)
            module = 'paino_skill.score_controller' if args.scene == 'piano' else 'openarm_app.web_controller'
            web = start(module, ['--port', str(port), '--sim-port', str(args.sim_port), '--no-browser'], 'web.txt')
            url = f'http://127.0.0.1:{port}/'
            wait_ready(web, url)
            print(f'READY {url}', flush=True)
            print('Close the MuJoCo window or press Ctrl+C here to finish.', flush=True)
            if args.check:
                time.sleep(2)
                if sim.poll() is not None or web.poll() is not None:
                    raise RuntimeError('A service exited during the startup check.')
                return 0
            if not args.no_browser:
                open_browser(url)
            while sim.poll() is None and web.poll() is None:
                time.sleep(.2)
            if sim.poll() is not None and sim.returncode:
                raise RuntimeError(f'Simulation exited with code {sim.returncode}.')
            if web.poll() is not None:
                raise RuntimeError(f'Web controller exited with code {web.returncode}.')
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f'Startup failed: {exc}\nLogs: {logdir}', flush=True)
        return 1
    finally:
        for process in reversed(processes):
            stop_process(process)
    return 0


if __name__ == '__main__':
    sys.exit(main())
