#!/usr/bin/env python3
"""Continuous receive loop using the context-manager Device class.

Stops on Ctrl-C and prints aggregate stats.
"""
import sys
import usb_can


def main() -> int:
    try:
        with usb_can.Device(0) as d:
            print(f"Opened: {d.name()}  caps=0x{d.capabilities():02X}")
            d.configure(bitrate=500000)
            d.start()
            print("Listening (Ctrl-C to stop)...")
            try:
                while True:
                    for can_id, dlc, flags, data, ts_us in d.receive(timeout_ms=200):
                        ext = "X" if flags & usb_can.CAN_FLAG_EXTENDED else "S"
                        fd  = "FD" if flags & usb_can.CAN_FLAG_FD else "  "
                        print(f"{ts_us:>14}us  {ext}{fd}  0x{can_id:08X}  [{dlc:2}]  {data.hex(' ')}")
            except KeyboardInterrupt:
                print("\nInterrupted.")
            s = d.stats()
            print(f"Rx={s.rx_frames} Tx={s.tx_frames} state={s.bus_state} overruns={s.rx_overruns}")
    except OSError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
