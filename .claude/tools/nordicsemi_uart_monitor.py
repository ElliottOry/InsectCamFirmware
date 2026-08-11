# /// script
# dependencies = [
#   "pyserial~=3.5",
# ]
# ///
"""Portable UART read/write utility.

Usage examples:
    # Read for 10 seconds then exit
    python nordicsemi_uart_monitor.py read --port /dev/ttyUSB0 --duration 10

    # Read continuously until Ctrl+C
    python nordicsemi_uart_monitor.py read --port COM3 --baud 115200

    # Write a single string
    python nordicsemi_uart_monitor.py write --port /dev/ttyUSB0 --message "AT+RST"

    # Interactive monitor: read logs + type to send
    python nordicsemi_uart_monitor.py monitor --port /dev/ttyUSB0

Requirements:
    pip install pyserial

license: "SPDX-License-Identifier: LicenseRef-Nordic-5-Clause"
metadata:
    version: 1.0.0
    Copyright: "(c) 2026 Nordic Semiconductor ASA"
"""

import argparse
import signal
import sys
import threading

try:
    import serial  # pyright: ignore[reportMissingModuleSource]
except ImportError:
    sys.exit("pyserial not found. Install it with: pip install pyserial")


def _open_port(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    """Open a serial port with the given baud rate and timeout.

    :param port: Serial port path or name, e.g. ``/dev/ttyUSB0`` or ``COM3``.
    :param baud: Baud rate for communication.
    :param timeout: Read timeout in seconds.
    :returns: An open :class:`serial.Serial` instance ready for I/O.
    :raises SystemExit: If the port cannot be opened.
    """
    try:
        return serial.Serial(port, baudrate=baud, timeout=timeout)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open port '{port}': {exc}")


def _read_loop(ser: serial.Serial, stop_event: threading.Event) -> None:
    """Read lines from a serial port until *stop_event* is set.

    Decoded lines are printed to stdout as they arrive.  On a
    :exc:`serial.SerialException` or :exc:`TypeError` the stop event is
    set so callers can clean up gracefully.

    :param ser: An open serial port to read from.
    :param stop_event: Event used to signal the loop to terminate.
    """
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if line:
                print(line.decode(errors="replace"), end="", flush=True)
        except serial.SerialException as exc:
            print(f"\n[UART error] {exc}", file=sys.stderr)
            stop_event.set()
        except TypeError:
            stop_event.set()


def cmd_read(args: argparse.Namespace) -> None:
    """Read from UART, optionally for a fixed duration.

    Starts a background reader thread and blocks until the duration
    elapses or a termination signal (SIGINT / SIGTERM) is received.

    :param args: Parsed CLI arguments with attributes ``port`` (str),
        ``baud`` (int), and ``duration`` (float | None).
    """
    ser = _open_port(args.port, args.baud)
    stop = threading.Event()

    def _on_signal(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)

    reader = threading.Thread(target=_read_loop, args=(ser, stop), daemon=True)
    reader.start()

    print(
        f"[UART] Reading {args.port} @ {args.baud} baud"
        + (f" for {args.duration}s" if args.duration else " (Ctrl+C to stop)"),
        file=sys.stderr,
    )

    try:
        if args.duration:
            stop.wait(timeout=args.duration)
            stop.set()
        else:
            stop.wait()
    finally:
        ser.close()
        reader.join(timeout=2)
        print("\n[UART] Done.", file=sys.stderr)


def cmd_monitor(args: argparse.Namespace) -> None:
    """Read from UART continuously while sending lines typed on stdin.

    Starts a background reader thread and enters an interactive loop
    that encodes each stdin line and writes it to the serial port.
    Terminates on SIGINT, SIGTERM, or EOF.

    :param args: Parsed CLI arguments with attributes ``port`` (str),
        ``baud`` (int), and ``newline`` (str).
    """
    ser = _open_port(args.port, args.baud)
    stop = threading.Event()

    def _on_signal(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)

    reader = threading.Thread(target=_read_loop, args=(ser, stop), daemon=True)
    reader.start()

    print(f"[UART] Monitor on {args.port} @ {args.baud} baud — type to send, Ctrl+C to quit.", file=sys.stderr)

    try:
        while not stop.is_set():
            try:
                line = input()
            except EOFError:
                break
            if stop.is_set():
                break
            try:
                ser.write((line + args.newline).encode())
                ser.flush()
            except serial.SerialException as exc:
                print(f"\n[UART error] {exc}", file=sys.stderr)
                stop.set()
    finally:
        stop.set()
        ser.close()
        reader.join(timeout=2)
        print("\n[UART] Done.", file=sys.stderr)


def cmd_write(args: argparse.Namespace) -> None:
    """Write a single string to UART and exit.

    Appends the configured line-ending to ``args.message``, encodes it,
    and flushes the port before closing.

    :param args: Parsed CLI arguments with attributes ``port`` (str),
        ``baud`` (int), ``message`` (str), and ``newline`` (str).
    :raises SystemExit: If the write operation fails.
    """
    ser = _open_port(args.port, args.baud)
    try:
        payload = (args.message + args.newline).encode()
        ser.write(payload)
        ser.flush()
        print(f"[UART] Sent {len(payload)} bytes to {args.port}.", file=sys.stderr)
    except serial.SerialException as exc:
        sys.exit(f"Write failed: {exc}")
    finally:
        ser.close()


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser with all subcommands.

    :returns: Configured :class:`argparse.ArgumentParser` with ``read``,
        ``write``, and ``monitor`` subcommands registered.
    """
    parser = argparse.ArgumentParser(description="Simple portable UART read/write utility.")
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0 or COM3")
    shared.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")

    sub = parser.add_subparsers(dest="command", required=True)

    read_p = sub.add_parser("read", parents=[shared], help="Read from UART")
    read_p.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SECONDS",
        help="Stop after N seconds (omit to run until Ctrl+C)",
    )
    read_p.set_defaults(func=cmd_read)

    write_p = sub.add_parser("write", parents=[shared], help="Write a string to UART")
    write_p.add_argument("--message", required=True, help="String to send")
    write_p.add_argument(
        "--newline",
        default="\r\n",
        help=r"Line ending appended to message (default: \r\n, use '' for none)",
    )
    write_p.set_defaults(func=cmd_write)

    monitor_p = sub.add_parser(
        "monitor",
        parents=[shared],
        help="Read UART continuously and send lines typed on stdin",
    )
    monitor_p.add_argument(
        "--newline",
        default="\r\n",
        help=r"Line ending appended to each sent line (default: \r\n)",
    )
    monitor_p.set_defaults(func=cmd_monitor)

    return parser


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)
