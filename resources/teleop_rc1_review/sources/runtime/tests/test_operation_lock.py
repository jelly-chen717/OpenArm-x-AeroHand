"""Linux-only file-lease regressions; no hardware or service dependencies."""
import importlib.util
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / "ops" / "operation_lock.py"
THIS_FILE = Path(__file__).resolve()
ENV_FD = "OPENARM_OPERATION_LEASE_FD"


def load_lock(source=SOURCE):
    spec = importlib.util.spec_from_file_location("tested_operation_lock", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(mode, root, *args):
    return [sys.executable, "-I", str(THIS_FILE), "--lease-worker", mode,
            str(SOURCE), str(root), *(str(arg) for arg in args)]


def worker(mode, source, root, *args):
    """Subprocess entry points use only this module and the file-lease source."""
    lease = load_lock(source)
    if mode == "probe":
        try:
            with lease.exclusive(root):
                result = {"acquired": True}
        except RuntimeError as error:
            result = {"acquired": False, "error": str(error)}
        print(json.dumps(result), flush=True)
    elif mode == "reenter":
        inherited = int(os.environ[ENV_FD])
        with lease.exclusive(root) as outer:
            with lease.exclusive(root) as inner:
                assert outer == inner == inherited
                assert lease.child_fds() == (inherited,)
        os.fstat(inherited)  # An inherited context must not close its caller's FD.
        print(json.dumps({"fd": inherited, "still_open": True,
                          "environment": os.environ[ENV_FD]}), flush=True)
    elif mode == "spawn_keeper":
        ready_write, release_read = map(int, args)
        with lease.exclusive(root) as fd:
            child = subprocess.Popen(
                command("keeper", root, ready_write, release_read),
                env=os.environ.copy(),
                pass_fds=(*lease.child_fds(), ready_write, release_read),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, close_fds=True,
            )
            print(json.dumps({"pid": child.pid, "fd": fd}), flush=True)
        # Deliberately exit without waiting: the inherited child must keep the
        # same open-file-description lease after this parent closes its FD.
    elif mode == "keeper":
        ready_write, release_read = map(int, args)
        with lease.exclusive(root) as fd:
            os.write(ready_write, b"ready\n")
            os.close(ready_write)
            readable, _, _ = select.select([release_read], [], [], 10)
            if not readable or os.read(release_read, 1) != b"x":
                raise RuntimeError("Test keeper did not receive its bounded release")
        os.close(fd)
        os.close(release_read)
    else:
        raise ValueError(mode)


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux flock/pass_fds semantics required")
class OperationLockTests(unittest.TestCase):
    def setUp(self):
        # Do not accidentally reuse a production lease inherited by the test
        # runner. Each test owns only its TemporaryDirectory lock.
        self.environment = patch.dict(os.environ)
        self.environment.start()
        self.addCleanup(self.environment.stop)
        os.environ.pop(ENV_FD, None)
        self.temp = tempfile.TemporaryDirectory(prefix="openarm-lease-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.lease = load_lock()

    def run_worker(self, mode="probe", *, inherited=False, pass_fds=(), env=None):
        child_env = os.environ.copy() if env is None else dict(env)
        if not inherited:
            child_env.pop(ENV_FD, None)
        result = subprocess.run(command(mode, self.root), env=child_env,
                                pass_fds=pass_fds, capture_output=True, text=True,
                                check=False, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_independent_process_conflicts_until_owner_releases(self):
        with self.lease.exclusive(self.root):
            denied = self.run_worker()
            self.assertFalse(denied["acquired"])
            self.assertIn("Another hardware operation", denied["error"])
        self.assertTrue(self.run_worker()["acquired"])

    def test_parent_fd_is_reentrant_in_child_and_still_excludes_other_processes(self):
        with self.lease.exclusive(self.root) as fd:
            result = self.run_worker("reenter", inherited=True,
                                     pass_fds=self.lease.child_fds())
            self.assertEqual(result["fd"], fd)
            self.assertEqual(result["environment"], str(fd))
            self.assertTrue(result["still_open"])
            os.fstat(fd)
            self.assertFalse(self.run_worker()["acquired"])
            payload = json.loads((self.root / "state/hardware_operation.lock").read_text())
            self.assertEqual(payload["pid"], os.getpid())
        self.assertTrue(self.run_worker()["acquired"])

    def test_inherited_fd_for_a_different_inode_is_rejected(self):
        with self.lease.exclusive(self.root):
            wrong = os.open(self.root / "unrelated.lock", os.O_RDWR | os.O_CREAT, 0o600)
            try:
                env = dict(os.environ, **{ENV_FD: str(wrong)})
                denied = self.run_worker(inherited=True, pass_fds=(wrong,), env=env)
                self.assertFalse(denied["acquired"])
                self.assertIn("not this project lock", denied["error"])
                os.fstat(wrong)
                self.assertFalse(self.run_worker()["acquired"])
            finally:
                os.close(wrong)
        self.assertTrue(self.run_worker()["acquired"])

    def test_child_keeps_lease_after_owning_parent_has_exited(self):
        ready_read, ready_write = os.pipe()
        release_read, release_write = os.pipe()
        child_env = os.environ.copy()
        child_env.pop(ENV_FD, None)
        parent = None
        try:
            parent = subprocess.Popen(
                command("spawn_keeper", self.root, ready_write, release_read),
                env=child_env, pass_fds=(ready_write, release_read),
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, close_fds=True,
            )
            os.close(ready_write)
            ready_write = None
            os.close(release_read)
            release_read = None
            output, error = parent.communicate(timeout=5)
            self.assertEqual(parent.returncode, 0, error)
            self.assertGreater(json.loads(output)["pid"], 0)
            readable, _, _ = select.select([ready_read], [], [], 5)
            self.assertTrue(readable, "Inherited child never acquired its lease")
            self.assertEqual(os.read(ready_read, 6), b"ready\n")
            # The original owning process has exited, but the inherited FD is
            # still open in its child. A brand-new open must remain excluded.
            self.assertIsNotNone(parent.poll())
            self.assertFalse(self.run_worker()["acquired"])
            os.write(release_write, b"x")
            os.close(release_write)
            release_write = None
            deadline = time.monotonic() + 5
            while True:
                if self.run_worker()["acquired"]:
                    break
                if time.monotonic() >= deadline:
                    self.fail("Child did not release its lease")
                time.sleep(0.01)
        finally:
            if release_write is not None:
                try:
                    os.write(release_write, b"x")
                except BrokenPipeError:
                    pass
            for fd in (ready_read, ready_write, release_read, release_write):
                if fd is not None:
                    os.close(fd)
            if parent is not None:
                if parent.poll() is None:
                    parent.kill()
                parent.communicate(timeout=5)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--lease-worker":
        worker(*sys.argv[2:])
    else:
        unittest.main()
