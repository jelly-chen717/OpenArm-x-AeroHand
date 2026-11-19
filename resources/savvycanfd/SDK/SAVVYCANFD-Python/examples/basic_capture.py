#!/usr/bin/env python3
"""Basic example - mirrors the C ``examples/basic_can.c`` flow.

Discovers the first CAN device, opens it at 500 kbps, sends one frame,
prints anything received within 2 seconds, then shuts down cleanly.
"""
import sys
import usb_can


def main() -> int:
    if usb_can.can_sdk_init() != usb_can.CAN_OK:
        print("SDK init failed", file=sys.stderr)
        return 1
    print("[OK] SDK initialized")

    devices = usb_can.discover()
    if not devices:
        print("No CAN devices found")
        usb_can.can_sdk_shutdown()
        return 1

    print(f"[OK] Found {len(devices)} device(s):")
    for d in devices:
        print(f"  [{d.index}] {d.name.decode()} - {d.description.decode()} "
              f"(Driver: {d.driver.decode()}, Caps: 0x{d.capabilities:02X})")

    handle = usb_can.open_device(devices[0].index)
    print(f"\n[OK] Opened: {usb_can.name(handle)}")

    usb_can.configure(handle, bitrate=500000, sample_point=875)
    print("[OK] Configured: 500000 bps")

    if usb_can.can_start(handle) != usb_can.CAN_OK:
        print("Start failed")
        usb_can.can_close(handle)
        usb_can.can_sdk_shutdown()
        return 1
    print("[OK] CAN bus started")

    usb_can.send(handle, 0x123, bytes([1, 2, 3, 4, 5, 6, 7, 8]))
    print("[OK] Sent: ID=0x123 DLC=8")

    print("\n[...] Waiting for messages (2 seconds)...")
    msgs = usb_can.receive(handle, max_count=16, timeout_ms=2000)
    if msgs:
        print(f"[OK] Received {len(msgs)} message(s):")
        for can_id, dlc, flags, data, ts_us in msgs:
            print(f"  ID=0x{can_id:03X} DLC={dlc} Flags=0x{flags:02X} "
                  f"Data={data.hex(' ')} ts={ts_us}us")
    else:
        print("[--] No messages received")

    s = usb_can.stats(handle)
    print(f"\n[STATS] Rx={s.rx_frames} Tx={s.tx_frames} "
          f"RxErr={s.rx_errors} TxErr={s.tx_errors} State={s.bus_state}")

    usb_can.can_stop(handle)
    usb_can.can_close(handle)
    usb_can.can_sdk_shutdown()
    print("\n[OK] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
