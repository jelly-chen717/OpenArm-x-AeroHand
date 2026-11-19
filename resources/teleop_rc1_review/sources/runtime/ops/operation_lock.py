# 项目中文注释（2026-10-29）：跨进程操作锁。
# 调用与职责：网页和显式命令共用文件锁；通过文件描述符向子进程继承租约。
# 运行边界：只修改锁文件和环境变量，不操作电机。
# 本批只增加注释；标准 IK、原桥和底层驱动的原字节在文件旁注中解释。
"""One host lease shared by the panel and explicit hardware/transport CLIs."""
from contextlib import contextmanager
import json
import os
from pathlib import Path

ENV_FD = 'OPENARM_OPERATION_LEASE_FD'


# 注释：获取或继承本项目唯一操作租约，阻止并发硬件事务。
@contextmanager
def exclusive(root):
    import fcntl
    root = Path(root)
    path = root/'state/hardware_operation.lock'
    path.parent.mkdir(parents=True,exist_ok=True)
    inherited = os.environ.get(ENV_FD)
    owned = inherited is None
    if owned:
        fd = os.open(path,os.O_RDWR|os.O_CREAT,0o600)
    else:
        fd = int(inherited)
        actual,expected = os.fstat(fd),path.stat()
        if (actual.st_dev,actual.st_ino)!=(expected.st_dev,expected.st_ino):
            raise RuntimeError('Inherited hardware lease is not this project lock')
    previous = os.environ.get(ENV_FD)
    try:
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Another hardware operation holds the project lease') from error
        if owned:
            payload=json.dumps({'pid':os.getpid(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip()}).encode()
            os.ftruncate(fd,0)
            os.write(fd,payload)
            os.fsync(fd)
        os.environ[ENV_FD]=str(fd)
        yield fd
    finally:
        if previous is None:
            os.environ.pop(ENV_FD,None)
        else:
            os.environ[ENV_FD]=previous
        if owned:
            os.close(fd)


# 注释：返回需传给子进程的操作锁文件描述符。
def child_fds():
    value=os.environ.get(ENV_FD)
    return (int(value),) if value is not None else ()
