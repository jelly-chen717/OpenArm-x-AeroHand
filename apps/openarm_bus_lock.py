"""Process-lifetime channel leases shared by this workspace's PCAN entry points."""
from pathlib import Path
import tempfile


class ChannelLease:
    def __init__(self, channel):
        self.path = Path(tempfile.gettempdir())/f'openarm_pcan_{channel:x}.lock'
        self.stream = None

    def acquire(self):
        import msvcrt
        stream = self.path.open('a+b')
        try:
            if self.path.stat().st_size == 0:
                stream.write(b'0')
                stream.flush()
            stream.seek(0)
            msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
        except BaseException:
            stream.close()
            raise
        self.stream = stream

    def release(self):
        if self.stream is not None:
            # Closing releases the Windows byte-range lock, also after process death.
            self.stream.close()
            self.stream = None
