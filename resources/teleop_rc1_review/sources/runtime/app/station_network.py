"""Workstation LAN addresses only; no ROS, CAN or motor operations.

Read the target's physical IPv4 interfaces so the hotspot DHCP lease can change
without weakening same-origin checks or editing motion code.
"""
import ipaddress
from pathlib import Path
import socket
import struct
import time

PORT = 4446
_cache = (0.0, ())


def local_addresses():
    global _cache
    now = time.monotonic()
    if now < _cache[0]:
        return _cache[1]
    import fcntl
    addresses = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _, name in socket.if_nameindex():
            if not (Path('/sys/class/net') / name / 'device').exists():
                continue
            try:
                value = fcntl.ioctl(sock.fileno(), 0x8915,
                                    struct.pack('256s', name.encode()[:15]))
                address = socket.inet_ntoa(value[20:24])
                ip = ipaddress.IPv4Address(address)
                if not (ip.is_loopback or ip.is_link_local or ip.is_unspecified):
                    addresses.append((name, address))
            except OSError:
                continue
    _cache = (now + 2.0, tuple(addresses))
    return _cache[1]


def preferred_origin():
    addresses = local_addresses()
    # Prefer the operator's Windows hotspot when connected; retain wired LAN.
    hotspot = [ip for _, ip in addresses if ip.startswith('192.168.137.')]
    address = hotspot[0] if hotspot else (addresses[0][1] if addresses else '127.0.0.1')
    return f'https://{address}:{PORT}'


def allowed_origins():
    return {f'https://{ip}:{PORT}' for _, ip in local_addresses()} | {
        f'https://127.0.0.1:{PORT}'}


def same_origin(request, optional=False):
    """Accept one exact origin on a current local IP, never arbitrary Host data."""
    if request.headers.get('sec-fetch-site') == 'cross-site':
        return False
    origins = request.headers.getlist('origin')
    if not origins:
        return optional
    return (len(origins) == 1 and origins[0] in allowed_origins()
            and origins[0] == str(request.base_url).rstrip('/'))
