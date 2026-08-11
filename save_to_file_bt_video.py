import asyncio
import csv
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "insect_cam"   # change if your CONFIG_BT_DEVICE_NAME is different

IMG_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
IMG_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"  # write commands here
IMG_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"  # image chunks come from here
IMG_INFO_UUID    = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"  # status info comes from here

# The firmware sends RAW 8-bit grayscale frames over the TX characteristic at the
# full 160x119 sensor resolution — no JPEG, no downsampling (see ble_manager.c).
# Each frame on the wire is an 8-byte header (b"KRAW" magic + width u16 LE +
# height u16 LE) followed by width*height pixel bytes in row order; the receiver
# resyncs on the magic if a frame is dropped mid-transfer.
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 119
RAW_MAGIC = b"KRAW"
JPG_MAGIC = b"KJPG"   # JPEG frames: same 8-byte header, body ends at FF D9
RAW_HDR_LEN = 8

# Interleaved between frames, the firmware sends batched IMU packets from the
# ICM-42686: b"KIMU" magic, sample count u8 (1..16), format u8 (=1), base
# timestamp u32 LE (device uptime ms of the oldest sample), temp s16 LE raw,
# then per sample: dt u16 LE from the base timestamp, accel X/Y/Z and gyro
# X/Y/Z as s16 LE raw counts.  Samples land in a per-session CSV.
KIMU_MAGIC = b"KIMU"
KIMU_HDR_LEN = 12
KIMU_SAMPLE_LEN = 14
KIMU_MAX_SAMPLES = 16
KIMU_FORMAT = 1

ACCEL_LSB_PER_G = 2048.0      # +/-16 g full scale
GYRO_LSB_PER_DPS = 16.4       # +/-2000 dps full scale
TEMP_LSB_PER_C = 132.48
TEMP_OFFSET_C = 25.0

SHOW_REALTIME = True
RECORD_VIDEO = True
# Written at the real measured fps so playback matches wall-clock time.
# 2x nearest-neighbor upscale: pixel-exact, and 320x238 is even in both
# dimensions (yuv420 codecs reject the native 160x119).
VIDEO_SCALE = 2

# Veiling glare (lens haze) adds a near-uniform gray veil: the black point rises
# and contrast compresses into a narrow band.  Counteract it by remapping black/
# white points (percentiles below) to the full 0..255 range.  The live view uses
# rolling per-frame levels (smoothed across frames); the saved .mp4 uses one set
# of levels computed over the whole clip so the video doesn't flicker.
DEVEIL = True
DEVEIL_PCT_LO = 1.0
DEVEIL_PCT_HI = 99.5
DEVEIL_MIN_SPAN = 24        # don't blow up noise on near-flat scenes
DEVEIL_SMOOTH = 0.3         # EMA weight of the newest frame's levels (live view)
DEVEIL_SAT = 250            # pixels at/above this count as the light source itself
                            # and are excluded from level estimation — otherwise a
                            # lamp/bright sky in frame pins the white point at 255
                            # and the stretch does nothing
DEVEIL_CLAHE = True         # local contrast equalization (needs OpenCV): counters
                            # the glare gradient near a light source, which no
                            # single global stretch can
DEVEIL_CLAHE_CLIP = 2.5

_deveil_state = {"lo": None, "hi": None}


def _deveil_span(lo: float, hi: float) -> tuple[float, float]:
    if hi - lo < DEVEIL_MIN_SPAN:
        mid = (lo + hi) / 2.0
        lo = mid - DEVEIL_MIN_SPAN / 2.0
        hi = mid + DEVEIL_MIN_SPAN / 2.0
    return lo, hi


def _deveil(img):
    import numpy as np

    body = img[img < DEVEIL_SAT]
    if body.size < img.size // 20:      # essentially the whole frame is blown out
        return img

    lo = float(np.percentile(body, DEVEIL_PCT_LO))
    hi = float(np.percentile(body, DEVEIL_PCT_HI))

    st = _deveil_state
    if st["lo"] is None:
        st["lo"], st["hi"] = lo, hi
    else:
        st["lo"] += DEVEIL_SMOOTH * (lo - st["lo"])
        st["hi"] += DEVEIL_SMOOTH * (hi - st["hi"])

    lo, hi = _deveil_span(st["lo"], st["hi"])
    out = (img.astype(np.float32) - lo) * (255.0 / (hi - lo))
    out = np.clip(out, 0.0, 255.0).astype(np.uint8)

    if DEVEIL_CLAHE:
        try:
            import cv2
            out = cv2.createCLAHE(clipLimit=DEVEIL_CLAHE_CLIP,
                                  tileGridSize=(4, 4)).apply(out)
        except ImportError:
            pass

    return out

START_SINGLE_IMAGE = bytes([1])
START_STREAM = bytes([2])
STOP_STREAM = bytes([3])

frame_buffer = bytearray()
frame_counter = 0

fps_times = deque(maxlen=30)
current_fps = 0.0

# Rolling throughput stats, reset every report interval by the session loop.
stats = {"bytes": 0, "frames": 0, "imu": 0}

_rt_state = {"im": None, "initialized": False, "title": None}

# Per-session IMU CSV, opened lazily on the first KIMU packet so a camera-only
# firmware (IMU absent) never leaves an empty file behind.
_imu_state = {"file": None, "writer": None}

# Console view of the IMU stream, throttled so 100 Hz samples don't flood the
# terminal: one line per period showing the newest sample in physical units.
IMU_PRINT_PERIOD_S = 0.25
_imu_print = {"t": 0.0, "count": 0}


def _write_imu_samples(temp_raw: int, samples: list) -> None:
    if _imu_state["writer"] is None:
        name = f"imu_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        _imu_state["file"] = open(name, "w", newline="")
        _imu_state["writer"] = csv.writer(_imu_state["file"])
        _imu_state["writer"].writerow(
            ["dev_ts_ms", "host_time",
             "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
             "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_c"])
        print(f"IMU log: {name}")

    host_t = time.time()
    temp_c = temp_raw / TEMP_LSB_PER_C + TEMP_OFFSET_C
    for ts, ax, ay, az, gx, gy, gz in samples:
        _imu_state["writer"].writerow(
            [ts, f"{host_t:.3f}", ax, ay, az, gx, gy, gz,
             f"{ax / ACCEL_LSB_PER_G:.4f}", f"{ay / ACCEL_LSB_PER_G:.4f}",
             f"{az / ACCEL_LSB_PER_G:.4f}", f"{gx / GYRO_LSB_PER_DPS:.3f}",
             f"{gy / GYRO_LSB_PER_DPS:.3f}", f"{gz / GYRO_LSB_PER_DPS:.3f}",
             f"{temp_c:.2f}"])
    _imu_state["file"].flush()

    _imu_print["count"] += len(samples)
    now = time.time()
    if _imu_print["t"] == 0.0:
        _imu_print["t"] = now
    elif now - _imu_print["t"] >= IMU_PRINT_PERIOD_S:
        rate = _imu_print["count"] / (now - _imu_print["t"])
        ts, ax, ay, az, gx, gy, gz = samples[-1]
        print(f"IMU  | a[g] {ax / ACCEL_LSB_PER_G:+6.2f} {ay / ACCEL_LSB_PER_G:+6.2f}"
              f" {az / ACCEL_LSB_PER_G:+6.2f}"
              f" | g[dps] {gx / GYRO_LSB_PER_DPS:+7.1f} {gy / GYRO_LSB_PER_DPS:+7.1f}"
              f" {gz / GYRO_LSB_PER_DPS:+7.1f}"
              f" | {temp_c:5.1f} C | {rate:5.1f} Hz | dev_ts {ts} ms")
        _imu_print["t"] = now
        _imu_print["count"] = 0


def _close_imu_log() -> None:
    if _imu_state["file"] is not None:
        _imu_state["file"].close()
    _imu_state["file"] = None
    _imu_state["writer"] = None


def _show_frame_realtime(frame: bytes, fps: float | None = None) -> None:
    if not SHOW_REALTIME:
        return

    try:
        import numpy as np
        from matplotlib import pyplot as plt
    except Exception as e:
        if not _rt_state["initialized"]:
            print(f"Realtime view unavailable: {e}")
            _rt_state["initialized"] = True
        return

    img = np.frombuffer(frame, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
    if DEVEIL:
        img = _deveil(img)

    if _rt_state["im"] is None:
        plt.ion()
        plt.figure("HM01B0 - BLE Realtime")
        _rt_state["title"] = plt.title("FPS: --")
        _rt_state["im"] = plt.imshow(img, cmap="gray", vmin=0, vmax=255)
        plt.axis("off")
        plt.tight_layout()
        plt.show()
        _rt_state["initialized"] = True
    else:
        _rt_state["im"].set_data(img)

    if fps is not None and _rt_state["title"] is not None:
        _rt_state["title"].set_text(f"FPS: {fps:.2f}")

    plt.pause(0.001)


# Video recording: frames accumulate here and are written to an .mp4 when the
# program quits (Ctrl+C) or when the user types "save" + Enter.  Guarded by a
# lock because frames arrive on the asyncio/BLE thread while "save" runs on the
# stdin watcher thread.
_rec_lock = threading.Lock()
_rec_frames: list[bytes] = []
_rec_times: list[float] = []


def record_frame(frame: bytes) -> None:
    if not RECORD_VIDEO:
        return
    with _rec_lock:
        _rec_frames.append(frame)
        _rec_times.append(time.time())


def save_video() -> None:
    """Write all recorded frames to an .mp4 and clear the buffer."""
    with _rec_lock:
        frames = _rec_frames[:]
        times = _rec_times[:]
        _rec_frames.clear()
        _rec_times.clear()

    if not frames:
        print("No frames recorded; nothing to save.")
        return

    try:
        import cv2
        import numpy as np
    except Exception as e:
        print(f"Cannot write video ({e}); dropping {len(frames)} frames.")
        return

    # Play back at the real measured rate; fall back to 10 fps for tiny clips.
    if len(times) > 1 and times[-1] > times[0]:
        fps = (len(times) - 1) / (times[-1] - times[0])
        fps = min(max(fps, 1.0), 60.0)
    else:
        fps = 10.0

    filename = datetime.now().strftime("insect_cam_%Y%m%d_%H%M%S.mp4")
    size = (IMAGE_WIDTH * VIDEO_SCALE, IMAGE_HEIGHT * VIDEO_SCALE)
    writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        print(f"Failed to open video writer for {filename}; "
              f"dropping {len(frames)} frames.")
        return

    # One clip-wide de-veil level set (excluding light-source pixels), applied as
    # a LUT to every frame so the video's brightness doesn't flicker; CLAHE then
    # adds the local contrast the global LUT can't.
    lut = None
    clahe = None
    if DEVEIL:
        allpix = np.frombuffer(b"".join(frames), dtype=np.uint8)
        body = allpix[allpix < DEVEIL_SAT]
        if body.size >= allpix.size // 20:
            lo, hi = _deveil_span(float(np.percentile(body, DEVEIL_PCT_LO)),
                                  float(np.percentile(body, DEVEIL_PCT_HI)))
            ramp = (np.arange(256, dtype=np.float32) - lo) * (255.0 / (hi - lo))
            lut = np.clip(ramp, 0.0, 255.0).astype(np.uint8)
        if DEVEIL_CLAHE:
            clahe = cv2.createCLAHE(clipLimit=DEVEIL_CLAHE_CLIP,
                                    tileGridSize=(4, 4))

    for frame in frames:
        img = np.frombuffer(frame, dtype=np.uint8).reshape((IMAGE_HEIGHT, IMAGE_WIDTH))
        if lut is not None:
            img = lut[img]
        if clahe is not None:
            img = clahe.apply(img)
        img = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        writer.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    writer.release()

    print(f"Saved {filename}: {len(frames)} frames, "
          f"{len(frames) / fps:.1f} s @ {fps:.2f} fps")


def _stdin_watcher() -> None:
    """Type 'save' + Enter to write the video immediately (backstop in case
    save-on-quit never runs, e.g. the process gets killed hard)."""
    for line in sys.stdin:
        if line.strip().lower() == "save":
            save_video()


def handle_img_info(sender, data: bytearray):
    if len(data) < 7:
        print(f"IMG info too short: {data.hex()}")
        return

    tx_state = data[0]
    mtu = data[1] | (data[2] << 8)
    width = data[3] | (data[4] << 8)
    bpp = data[5]
    mode = data[6]
    imu = data[7] if len(data) >= 8 else "n/a"

    print(f"IMG INFO | tx_state={tx_state} mtu={mtu} width={width} bpp={bpp} mode={mode} imu={imu}")


def _pop_packet() -> tuple | None:
    """Demux the TX stream: returns ("frame", pixels) or ("imu", temp_raw,
    [(ts_ms, ax, ay, az, gx, gy, gz), ...]), or None if no complete packet."""
    global frame_buffer

    while True:
        raw_at = frame_buffer.find(RAW_MAGIC)
        jpg_at = frame_buffer.find(JPG_MAGIC)
        imu_at = frame_buffer.find(KIMU_MAGIC)
        starts = [s for s in (raw_at, jpg_at, imu_at) if s >= 0]
        if not starts:
            # Keep the tail in case a magic straddles the chunk boundary.
            if len(frame_buffer) >= len(RAW_MAGIC):
                del frame_buffer[: -(len(RAW_MAGIC) - 1)]
            return None

        # Whichever packet the device sent first.
        start = min(starts)
        if start > 0:
            del frame_buffer[:start]

        if frame_buffer[:4] == RAW_MAGIC:
            if len(frame_buffer) < RAW_HDR_LEN:
                return None

            width = frame_buffer[4] | (frame_buffer[5] << 8)
            height = frame_buffer[6] | (frame_buffer[7] << 8)

            if width != IMAGE_WIDTH or height != IMAGE_HEIGHT:
                # Magic bytes appeared inside pixel data — skip them and resync.
                del frame_buffer[: len(RAW_MAGIC)]
                continue

            if len(frame_buffer) < RAW_HDR_LEN + width * height:
                return None

            frame = bytes(frame_buffer[RAW_HDR_LEN : RAW_HDR_LEN + width * height])
            del frame_buffer[: RAW_HDR_LEN + width * height]
            return ("frame", frame)

        if frame_buffer[:4] == JPG_MAGIC:
            if len(frame_buffer) < RAW_HDR_LEN:
                return None

            width = frame_buffer[4] | (frame_buffer[5] << 8)
            height = frame_buffer[6] | (frame_buffer[7] << 8)

            if width != IMAGE_WIDTH or height != IMAGE_HEIGHT:
                # Magic bytes appeared inside compressed data — skip and resync.
                del frame_buffer[: len(JPG_MAGIC)]
                continue

            # A JPEG frame ends at the EOI marker; FF-stuffing in the entropy
            # stream guarantees FF D9 cannot appear before the real end.
            eoi = frame_buffer.find(b"\xff\xd9", RAW_HDR_LEN)
            if eoi < 0:
                return None                       # frame still in flight

            jpg = bytes(frame_buffer[RAW_HDR_LEN : eoi + 2])
            del frame_buffer[: eoi + 2]

            import io
            from PIL import Image
            try:
                img = Image.open(io.BytesIO(jpg)).convert("L")
            except Exception as e:
                print(f"JPEG decode failed ({e}), dropping frame")
                continue
            if img.size != (IMAGE_WIDTH, IMAGE_HEIGHT):
                print(f"JPEG frame has unexpected size {img.size}, dropping")
                continue
            # Hand decoded pixels downstream so display/recording code is
            # format-agnostic.
            return ("frame", img.tobytes())

        # KIMU packet
        if len(frame_buffer) < KIMU_HDR_LEN:
            return None

        count = frame_buffer[4]
        fmt = frame_buffer[5]
        if not (1 <= count <= KIMU_MAX_SAMPLES and fmt == KIMU_FORMAT):
            # Magic bytes appeared inside pixel data — skip them and resync.
            del frame_buffer[: len(KIMU_MAGIC)]
            continue

        need = KIMU_HDR_LEN + count * KIMU_SAMPLE_LEN
        if len(frame_buffer) < need:
            return None

        base_ts, temp_raw = struct.unpack_from("<Ih", frame_buffer, 6)
        samples = []
        for i in range(count):
            off = KIMU_HDR_LEN + i * KIMU_SAMPLE_LEN
            dt, ax, ay, az, gx, gy, gz = struct.unpack_from("<H6h", frame_buffer, off)
            samples.append((base_ts + dt, ax, ay, az, gx, gy, gz))
        del frame_buffer[:need]
        return ("imu", temp_raw, samples)


def handle_image_chunk(sender, data: bytearray):
    global frame_buffer
    global frame_counter
    global current_fps

    stats["bytes"] += len(data)          # raw stream bytes received (throughput meter)
    frame_buffer.extend(data)

    while True:
        pkt = _pop_packet()
        if pkt is None:
            break

        if pkt[0] == "imu":
            _, temp_raw, samples = pkt
            stats["imu"] += len(samples)
            _write_imu_samples(temp_raw, samples)
            continue

        frame = pkt[1]
        frame_counter += 1
        stats["frames"] += 1

        now = time.time()
        fps_times.append(now)

        if len(fps_times) > 1:
            current_fps = (len(fps_times) - 1) / (fps_times[-1] - fps_times[0])

        _show_frame_realtime(frame, current_fps)
        record_frame(frame)

        print(f"Displayed frame {frame_counter} ({len(frame)} bytes) | FPS: {current_fps:.2f}")


async def find_device():
    print("Scanning for BLE device...")

    devices = await BleakScanner.discover(timeout=5.0)

    for device in devices:
        if device.name == DEVICE_NAME:
            print(f"Found {device.name}: {device.address}")
            return device

    print("Could not find device by name. Nearby devices:")
    for device in devices:
        print(f"  {device.name} | {device.address}")

    return None


async def run_session(device) -> None:
    """One connect → stream → disconnect cycle. Returns when the link drops."""
    global frame_buffer

    disconnected = asyncio.Event()

    def on_disconnect(_client):
        print("Disconnected.")
        disconnected.set()

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        print("Connected")

        # Fresh start: drop any partial frame and zero the throughput counters.
        frame_buffer = bytearray()
        stats["bytes"] = 0
        stats["frames"] = 0
        stats["imu"] = 0

        await client.start_notify(IMG_INFO_UUID, handle_img_info)
        await client.start_notify(IMG_TX_UUID, handle_image_chunk)
        print("Notifications enabled")

        await asyncio.sleep(0.5)
        print("Starting stream...")
        await client.write_gatt_char(IMG_RX_UUID, START_STREAM, response=False)

        # Report measured throughput once per second until the link drops.
        last = time.time()
        try:
            while not disconnected.is_set() and client.is_connected:
                await asyncio.sleep(1.0)
                now = time.time()
                dt = now - last
                last = now
                nbytes = stats["bytes"]
                nframes = stats["frames"]
                nimu = stats["imu"]
                stats["bytes"] = 0
                stats["frames"] = 0
                stats["imu"] = 0
                if dt > 0:
                    kbps = nbytes * 8 / 1000.0 / dt
                    kBps = nbytes / 1024.0 / dt
                    print(f"[throughput] {kBps:6.1f} KB/s  ({kbps:6.0f} kbps)  "
                          f"{nframes / dt:4.2f} fps  {nimu / dt:5.0f} imu/s")
        finally:
            _close_imu_log()
            # Best-effort stop; ignore errors if the link is already gone.
            try:
                await client.write_gatt_char(IMG_RX_UUID, STOP_STREAM, response=False)
            except Exception:
                pass


async def main():
    """Scan/connect with automatic reconnect so the throughput test survives drops."""
    while True:
        try:
            device = await find_device()
            if device is None:
                print("Device not found; retrying in 3 s...")
                await asyncio.sleep(3)
                continue
            await run_session(device)
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Session error: {e!r}")
        print("Reconnecting in 2 s...  (Ctrl+C to quit)")
        await asyncio.sleep(2)


if __name__ == "__main__":
    if RECORD_VIDEO:
        print("Recording video: Ctrl+C saves on quit, or type 'save' + Enter to save now.")
        threading.Thread(target=_stdin_watcher, daemon=True).start()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        save_video()
