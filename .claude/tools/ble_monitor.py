# /// script
# dependencies = [
#   "bleak~=0.22",
# ]
# ///
"""Headless, bounded BLE monitor for the microCam firmware (insect_cam).

Non-interactive counterpart to save_to_file_bt.py: connects, captures the
image stream for a fixed duration, prints throughput stats and decoded
IMG INFO notifications, then exits on its own. Intended to be run by
agents/scripts, so it never opens a UI and never loops forever.

Usage examples:
    # List nearby BLE devices and whether insect_cam is visible
    uv run ble_monitor.py scan

    # Capture the stream for 10 s, save raw frame bytes to a file
    uv run ble_monitor.py capture --duration 10 --out /tmp/frames.bin

    # Request a single image instead of a stream
    uv run ble_monitor.py capture --mode single --duration 5 --out /tmp/frame.bin

    # Connect and listen without sending any command
    uv run ble_monitor.py capture --mode none --duration 5
"""

import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "insect_cam"  # firmware CONFIG_BT_DEVICE_NAME

IMG_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
IMG_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"  # write commands here
IMG_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"  # image chunks come from here
IMG_INFO_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"  # status info comes from here

# Wire format: 80x59 @ 4 bpp, two pixels per byte (see ble_manager.c).
FRAME_SIZE = 80 * 59 // 2  # 2360 bytes

COMMANDS = {"single": bytes([1]), "stream": bytes([2]), "none": None}
STOP_STREAM = bytes([3])


async def cmd_scan(args: argparse.Namespace) -> int:
    print(f"Scanning for {args.timeout:.0f} s...", file=sys.stderr)
    devices = await BleakScanner.discover(timeout=args.timeout)
    found = None
    for d in sorted(devices, key=lambda d: (d.name or "~")):
        marker = ""
        if d.name == args.name:
            found = d
            marker = "   <-- target"
        print(f"{d.address}  {d.name or '(no name)'}{marker}")
    if found:
        print(f"\nFOUND {args.name} at {found.address}")
        return 0
    print(f"\nNOT FOUND: no device named '{args.name}' ({len(devices)} devices seen)")
    return 1


async def cmd_capture(args: argparse.Namespace) -> int:
    print(f"Scanning for '{args.name}' ({args.timeout:.0f} s)...", file=sys.stderr)
    device = await BleakScanner.find_device_by_name(args.name, timeout=args.timeout)
    if device is None:
        print(f"NOT FOUND: '{args.name}' not advertising. Run the scan subcommand to see nearby devices.")
        return 1
    print(f"Found {device.name} at {device.address}", file=sys.stderr)

    buffer = bytearray()
    second = {"bytes": 0}
    disconnected = asyncio.Event()
    frame_times: list[float] = []

    def on_disconnect(_client):
        disconnected.set()

    def on_info(_sender, data: bytearray):
        if len(data) < 7:
            print(f"IMG INFO (short): {data.hex()}")
            return
        mtu = data[1] | (data[2] << 8)
        width = data[3] | (data[4] << 8)
        print(f"IMG INFO | tx_state={data[0]} mtu={mtu} width={width} bpp={data[5]} mode={data[6]}")

    def on_chunk(_sender, data: bytearray):
        buffer.extend(data)
        second["bytes"] += len(data)
        while len(frame_times) < len(buffer) // FRAME_SIZE:
            frame_times.append(time.monotonic())

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        print(f"Connected | negotiated MTU: {client.mtu_size}")
        await client.start_notify(IMG_INFO_UUID, on_info)
        await client.start_notify(IMG_TX_UUID, on_chunk)

        command = COMMANDS[args.mode]
        if command is not None:
            await asyncio.sleep(0.5)
            await client.write_gatt_char(IMG_RX_UUID, command, response=False)
            print(f"Sent command: {args.mode}")

        start = time.monotonic()
        try:
            while time.monotonic() - start < args.duration and not disconnected.is_set():
                await asyncio.sleep(1.0)
                nbytes = second["bytes"]
                second["bytes"] = 0
                t = time.monotonic() - start
                print(f"[{t:5.1f}s] {nbytes / 1024:6.1f} KB/s ({nbytes * 8 / 1000:6.0f} kbps)"
                      f"  total frames: {len(buffer) // FRAME_SIZE}")
        finally:
            if command is not None and client.is_connected:
                try:
                    await client.write_gatt_char(IMG_RX_UUID, STOP_STREAM, response=False)
                except Exception:
                    pass

    elapsed = time.monotonic() - start
    complete = len(buffer) // FRAME_SIZE
    partial = len(buffer) % FRAME_SIZE
    fps = 0.0
    if len(frame_times) > 1:
        fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])

    if args.out:
        with open(args.out, "wb") as fp:
            fp.write(buffer)
        print(f"Wrote {len(buffer)} bytes to {args.out}")

    print("--- SUMMARY ---")
    print(f"window: {elapsed:.1f} s | bytes: {len(buffer)} | complete frames: {complete}"
          f" ({FRAME_SIZE} B each) | partial leftover: {partial} B | avg fps: {fps:.2f}")
    if disconnected.is_set() and elapsed < args.duration:
        print(f"WARNING: link dropped after {elapsed:.1f} s (before the requested window ended)")
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bounded, headless BLE monitor for insect_cam.")
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument("--name", default=DEVICE_NAME, help=f"Device name (default: {DEVICE_NAME})")
    shared.add_argument("--timeout", type=float, default=6.0, help="Scan timeout in seconds (default: 6)")

    sub = parser.add_subparsers(dest="command", required=True)

    scan_p = sub.add_parser("scan", parents=[shared], help="List nearby BLE devices")
    scan_p.set_defaults(func=cmd_scan)

    cap_p = sub.add_parser("capture", parents=[shared], help="Connect and capture the image stream")
    cap_p.add_argument("--duration", type=float, default=10.0, help="Capture window in seconds (default: 10)")
    cap_p.add_argument("--mode", choices=sorted(COMMANDS), default="stream",
                       help="Command to send after subscribing (default: stream; 'none' just listens)")
    cap_p.add_argument("--out", default=None, help="File to write the raw captured bytes to")
    cap_p.set_defaults(func=cmd_capture)

    return parser


if __name__ == "__main__":
    args = build_parser().parse_args()
    sys.exit(asyncio.run(args.func(args)))
