import asyncio
import csv
import struct
import sys
import threading
import time
from collections import deque

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "insect_cam"   # change if your CONFIG_BT_DEVICE_NAME is different

IMG_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca3e"
IMG_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca3e"  # write commands here
IMG_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca3e"  # image chunks come from here
IMG_INFO_UUID    = "6e400004-b5a3-f393-e0a9-e50e24dcca3e"  # status info comes from here

# Each frame on the wire is an 8-byte header (4-byte magic + width u16 LE +
# height u16 LE); the receiver resyncs on the magic if a frame is dropped
# mid-transfer.  Two body formats are decoded, selected by BLE_STREAM_RAW in the
# firmware's ble_manager.c and reported in the IMG INFO packet:
#   KRAW - width*height pixel bytes in row order, 8-bit grayscale, no encoding
#          at all.  Lossless, maximum quality, ~1.1 fps.
#   KJPG - q90 grayscale JPEG ending at the EOI marker.  Lossy, ~3 fps, ~5x less
#          data per frame.  (current firmware)  Decoding it needs Pillow.
# Nominal geometry of the current firmware.  The DECODER does not use these -- it
# takes each frame's width/height from that frame's header, so a ChangeResolution
# command is followed without restarting the script.  They stay because the dead
# column, de-veil and focus tuning further down was all measured at this geometry,
# and the comments there refer back to it.
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 119
# Header dimensions outside this range mean the magic turned up inside pixel data
# rather than at a real frame start -- see _pop_packet, which resyncs on that.
# The largest resolution the app offers is 200x160, so this leaves plenty of room.
DIM_MAX = 320
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
#0.876
#0.867
SHOW_REALTIME = True
SAVE_TO_DISK = False

# Commands are written to IMG_RX_UUID as [opcode][payload], one ATT write each.
# The opcodes are the ordinals of BleCommand in the reference Android client
# (insect_cam-Insect-Cam-Android, MainActivity.java) and the payload tables below
# are its spinner arrays from res/values/strings.xml -- keep both in sync with it,
# since the firmware is shared between the two.
#
# NOTE: the firmware in this repo decodes only 1/2/3 (on_rx_received in
# src/ble_manager.c).  Everything else is logged there and dropped, so opcodes
# 4-10 go out correctly but do nothing until that switch grows the cases.  The
# device's RTT line "RX received N bytes --> <hex>" is how you confirm one landed.
CMD_NO_COMMAND          = 0
CMD_START_SINGLE_IMAGE  = 1
CMD_START_STREAM        = 2
CMD_STOP_STREAM         = 3
CMD_CHANGE_RESOLUTION   = 4
CMD_CHANGE_PHY          = 5
CMD_GET_BLE_PARAMS      = 6
CMD_CHANGE_ANGLE        = 7
CMD_JPEG                = 8
CMD_STREAM_ACCEL        = 9
CMD_WRITE_REG           = 10

CMD_NAMES = {
    CMD_NO_COMMAND:         "NoCommand",
    CMD_START_SINGLE_IMAGE: "StartSingleCapture",
    CMD_START_STREAM:       "StartStreaming",
    CMD_STOP_STREAM:        "StopStreaming",
    CMD_CHANGE_RESOLUTION:  "ChangeResolution",
    CMD_CHANGE_PHY:         "ChangePhy",
    CMD_GET_BLE_PARAMS:     "GetBleParams",
    CMD_CHANGE_ANGLE:       "ChangeAngle",
    CMD_JPEG:               "JPEG",
    CMD_STREAM_ACCEL:       "StreamAccel",
    CMD_WRITE_REG:          "WriteReg",
}

# Payload value tables -- the commands carry the INDEX into these, not the value.
RESOLUTIONS = [(100, 100), (160, 120), (160, 160), (200, 160)]
PHYS = ["1Mbps", "2Mbps"]
ANGLES = [-45, -30, -15, 0, 15, 30, 45]        # degrees; index 3 (0 deg) is the app's default
REG_DEVICES = {"cam": 0, "camera": 0, "accel": 1, "imu": 1}

# The app auto-starts nothing; this script has always begun streaming on connect,
# so that stays the default.  Set False to connect and drive it purely by hand.
AUTO_START_STREAM = True

frame_buffer = bytearray()
frame_counter = 0
display_queue = None

# Rolling throughput stats, reset every report interval by the session loop.
stats = {"bytes": 0, "frames": 0, "imu": 0}

# Per-session IMU CSV, opened lazily on the first KIMU packet so a camera-only
# firmware (IMU absent) never leaves an empty file behind.
_imu_state = {"file": None, "writer": None}

# Console view of the IMU stream, throttled so 100 Hz samples don't flood the
# terminal: one line per period showing the newest sample in physical units.
IMU_PRINT_PERIOD_S = 0.25
_imu_print = {"t": 0.0, "count": 0}


# --- Link signal strength (RSSI) ------------------------------------------------
# Two different numbers get reported here, and they are NOT interchangeable:
#
#   adv  - RSSI of the ADVERTISING packet seen during the scan, before any
#          connection exists.  This is the same quantity nRF Connect / LightBlue
#          show in their scan list, so it is the one to compare against a phone.
#   live - RSSI of the CONNECTED link, polled while streaming.  Comparable to the
#          iOS app's stats row, which reads connected RSSI every 2 s.
#
# Both are measured by the LOCAL radio (this Mac), so they say how well this
# machine hears the device.  A Mac's antenna and front end are worth roughly
# 10-15 dB over a phone's, so expect this readout to sit ABOVE the phone's for
# the same device at the same distance -- that gap is normal and is not a fault.
# What matters for diagnosis is the gap between insect_cam and a reference
# peripheral measured by the SAME receiver (see SHOW_SCAN_REFERENCE below).
RSSI_POLL_S = 1.0             # how often to poll the connected link
RSSI_WINDOW = 30              # samples kept for the rolling avg/min/max

# RSSI is genuinely noisy -- multipath alone swings it several dB frame to frame,
# so a single instantaneous reading is a poor thing to compare against a phone.
# The rolling window makes the comparison meaningful.
rssi_state = {
    "adv": None,              # advertising RSSI captured at scan time
    "last": None,             # newest connected reading, dBm
    "hist": deque(maxlen=RSSI_WINDOW),
    "note": None,             # why live polling is unavailable, reported once
}

# Print the strongest few other peripherals at scan time.  This is the A/B the RF
# brief asks for: an absolute dBm number means little on its own, but insect_cam
# measured against a normal peripheral BY THE SAME RECEIVER is diagnostic.
SHOW_SCAN_REFERENCE = True
SCAN_REFERENCE_COUNT = 3


def _rssi_brief() -> str:
    """Compact 'rssi -63 dBm' for the per-frame line."""
    last = rssi_state["last"]
    return "rssi   ?  dBm" if last is None else f"rssi {last:4d} dBm"


def _rssi_detail() -> str:
    """Rolling window for the 1 Hz line -- this is the number to compare."""
    hist = rssi_state["hist"]
    if not hist:
        adv = rssi_state["adv"]
        return f"rssi   ?     (adv {adv} dBm)" if adv is not None else "rssi   ?"
    avg = sum(hist) / len(hist)
    return (f"rssi {rssi_state['last']:4d} dBm "
            f"(avg {avg:6.1f}  min {min(hist):4d}  max {max(hist):4d}  n={len(hist):2d})")


async def rssi_monitor(client) -> None:
    """Poll the connected link's RSSI until cancelled.

    bleak 3.x dropped the public BleakClient.get_rssi(); on macOS the method
    still exists on the CoreBluetooth backend object, so reach through _backend.
    On a backend without it (BlueZ/WinRT) this degrades to advertising-only
    rather than failing the session."""
    getter = getattr(getattr(client, "_backend", None), "get_rssi", None)
    if getter is None:
        rssi_state["note"] = "live RSSI not supported by this bleak backend"
        print(f"  NOTE: {rssi_state['note']}; scan RSSI only.")
        return

    while True:
        try:
            value = int(await getter())
        except Exception as e:
            # One line, once: a failing poll must not drown the frame log.
            if rssi_state["note"] is None:
                rssi_state["note"] = repr(e)
                print(f"  NOTE: RSSI poll failed ({e!r}); continuing without it.")
        else:
            rssi_state["last"] = value
            rssi_state["hist"].append(value)
        await asyncio.sleep(RSSI_POLL_S)


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

_rt_state = {"im": None, "ax": None, "initialized": False, "dims": None}

# The firmware's SPIS capture deliberately skips the first and last byte of every
# line (arm_next_spis_transfer in HM01B0_CAPTURE.c), so column 0 and column
# IMAGE_WIDTH-1 are hard 0 in every frame.  That is 2/160 = 1.25% of the pixels --
# MORE than DEVEIL_PCT_LO -- so the black point below used to land on those dead
# pixels and come out as 0 every single time, cutting the stretch from ~2.5x to
# ~1.2x and never subtracting the veil.  That, not the sensor exposure, is what
# made frames look washed out.  Replicate the neighbouring column for DISPLAY so
# the levels are measured on real pixels; frames handed to save_frame() stay
# untouched, so what lands on disk is still exactly what the sensor produced.
DEAD_COLS = 1


def _fill_dead_cols(img):
    if DEAD_COLS <= 0:
        return img
    img = img.copy()
    img[:, :DEAD_COLS] = img[:, DEAD_COLS:DEAD_COLS + 1]
    img[:, -DEAD_COLS:] = img[:, -DEAD_COLS - 1:-DEAD_COLS]
    return img

# Veiling glare (lens haze) adds a near-uniform gray veil: the black point rises
# and contrast compresses into a narrow band.  Counteract it for DISPLAY by
# remapping the frame's rolling black/white points (percentiles below, smoothed
# across frames to avoid flicker) to the full 0..255 range.  Saved PNGs stay raw.
DEVEIL = True
DEVEIL_PCT_LO = 1.0
DEVEIL_PCT_HI = 99.5
# Don't blow up noise on near-flat scenes.  The measured temporal noise floor is
# ~2.5 counts, so this span caps the stretch at 255/64 = 4x and keeps amplified
# noise near 10 counts; at the old 24 a blank wall stretched 7.2x and the frame
# turned into visible grain.  Real scenes span far more than this and are unaffected.
DEVEIL_MIN_SPAN = 64
DEVEIL_SMOOTH = 0.3         # EMA weight of the newest frame's levels
DEVEIL_SAT = 250            # pixels at/above this count as the light source itself
                            # and are excluded from level estimation — otherwise a
                            # lamp/bright sky in frame pins the white point at 255
                            # and the stretch does nothing
DEVEIL_CLAHE = True         # local contrast equalization (needs OpenCV): counters
                            # the glare gradient near a light source, which no
                            # single global stretch can
DEVEIL_CLAHE_CLIP = 2.5

_deveil_state = {"lo": None, "hi": None}

# Black/white points are measured on this sub-window only; the FULL frame is still
# displayed and saved.  The left edge has to be excluded because the sensor's first
# valid pixels of every row are a readout-settling artifact, not scene content:
# columns 3-9 measure 183 -> 134 counts against an interior level of 126, settling
# by column 10.  At ~4% of the frame they sit above DEVEIL_PCT_HI, so they were
# setting the white point all by themselves -- measured stretch 2.7x with them in,
# 7.2x with them excluded.  Column IMAGE_WIDTH-1 is excluded as a dead column.
DEVEIL_WIN_L = 10
DEVEIL_WIN_R = 1


def _deveil(img):
    import numpy as np

    win = img[:, DEVEIL_WIN_L:img.shape[1] - DEVEIL_WIN_R]
    if win.size == 0:
        win = img
    body = win[win < DEVEIL_SAT]
    if body.size < win.size // 20:      # essentially the whole frame is blown out
        return img

    lo = float(np.percentile(body, DEVEIL_PCT_LO))
    hi = float(np.percentile(body, DEVEIL_PCT_HI))

    st = _deveil_state
    if st["lo"] is None:
        st["lo"], st["hi"] = lo, hi
    else:
        st["lo"] += DEVEIL_SMOOTH * (lo - st["lo"])
        st["hi"] += DEVEIL_SMOOTH * (hi - st["hi"])

    lo, hi = st["lo"], st["hi"]
    if hi - lo < DEVEIL_MIN_SPAN:
        mid = (lo + hi) / 2.0
        lo = mid - DEVEIL_MIN_SPAN / 2.0
        hi = mid + DEVEIL_MIN_SPAN / 2.0

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


def _show_frame_realtime(frame: bytes, width: int, height: int) -> bool:
    """Render one frame.  Returns whether it actually reached a window, so the
    caller never claims a frame was displayed when the view is a no-op."""
    if not SHOW_REALTIME:
        return False

    try:
        import numpy as np
        from matplotlib import pyplot as plt
    except Exception as e:
        if not _rt_state["initialized"]:
            print(f"Realtime view unavailable: {e}")
            print("  -> frames ARE arriving but nothing will be shown."
                  "  Fix: .venv/bin/pip install matplotlib numpy")
            _rt_state["initialized"] = True
        return False

    img = np.frombuffer(frame, dtype=np.uint8).reshape((height, width))
    img = _fill_dead_cols(img)
    if DEVEIL:
        img = _deveil(img)

    # A resolution change has to rebuild the image artist: set_data with a
    # different shape leaves the axes scaled to the old frame.
    if _rt_state["im"] is None or _rt_state["dims"] != (width, height):
        plt.ion()
        plt.figure("HM01B0 - BLE Realtime", clear=True)
        _rt_state["im"] = plt.imshow(img, cmap="gray", vmin=0, vmax=255)
        _rt_state["ax"] = plt.gca()
        _rt_state["dims"] = (width, height)
        plt.axis("off")
        # Set the title before tight_layout so the layout reserves room for it.
        _rt_state["ax"].set_title(_rssi_brief(), fontsize=9, family="monospace")
        plt.tight_layout()
        plt.show()
        _rt_state["initialized"] = True
    else:
        _rt_state["im"].set_data(img)
        # Live dB over the picture: the readout updates with every frame, so the
        # signal can be watched while the board or antenna is moved by hand.
        if _rt_state["ax"] is not None:
            _rt_state["ax"].set_title(_rssi_brief(), fontsize=9, family="monospace")

    plt.pause(0.001)
    return True


# Live focus aid.  Turn the lens barrel (or peel the protective film) and watch
# this number PEAK -- it is the whole point of the readout, not an absolute score.
#
# It reports how SHARP the edges are, not how strong -- deliberately, because a
# raw gradient score swings 4.4x with lighting alone at constant focus, which makes
# it useless for the one job it has.  Scene contrast is cancelled by dividing the
# 1-pixel gradient by the 4-pixel gradient: a resolved edge is just as steep across
# 1 px as across 4 (ratio -> 100), while a blurred one only builds up over the
# longer baseline (ratio -> 0).  Multiplying the whole scene by k scales both, so
# the ratio does not move.  Sensor noise is white and therefore equally strong at
# both scales, which would read as perfect sharpness, so an Immerkaer noise
# estimate is subtracted from each in quadrature first.
#
# Validated on a blur sweep against a synthetic ceiling-line target carrying this
# sensor's real ~2.5-count noise:
#     0   no detail above the noise floor (guard tripped)
#    35   very blurred  (~4 px)
#    39   blurred       (~2 px)
#    47   soft          (~1 px)
#    85   nearly sharp  (~0.5 px)
#    98   sharp, single-pixel detail resolved
# Note the scale is compressed at the blurred end (4 px and 2 px are only 4 points
# apart) and opens up near focus -- expect little movement until you get close,
# then a steep climb.  Contrast invariance over an 8x lighting range at fixed
# focus: 84.6 to 85.1, i.e. 0.6%.  Frame-to-frame std on a real static scene is
# ~2 points.  This module currently sits at ~43-45, i.e. roughly 1.5-2 px of blur.
# For scale, the sensor's own row-start artifact steps 45 counts between adjacent
# columns, so the capture path can carry a hard edge -- if this score stays low
# while pointed at a detailed subject, the problem is the optics.
SHOW_FOCUS = True


_focus_state = {"ema": None}
FOCUS_SMOOTH = 0.4            # EMA weight of the newest frame; at ~1 fps this
                              # settles in ~2 s, steady enough to peak by hand


def _focus_score(frame: bytes, width: int, height: int) -> float:
    try:
        import numpy as np
        from numpy.lib.stride_tricks import sliding_window_view
    except Exception:
        return float("nan")

    img = np.frombuffer(frame, dtype=np.uint8).reshape(
        (height, width)).astype(np.float32)
    # Interior only: skips the dead columns and the row-start settling band, both
    # of which are sensor artifacts that would score as permanent "sharpness".
    roi = img[8:height - 8, DEVEIL_WIN_L:width - DEVEIL_WIN_R]
    if roi.shape[0] < 8 or roi.shape[1] < 8:
        return 0.0

    # Mean-square gradient energy at both scales.  RMS over ~15k samples, not an
    # extreme percentile: percentiles of a max are high-variance and made this
    # swing 18-43 on identical scenes.  Variances also add, so the noise term
    # below subtracts exactly.
    d1 = np.concatenate([np.diff(roi, axis=1).ravel(),
                         np.diff(roi, axis=0).ravel()])
    d4 = np.concatenate([(roi[:, 4:] - roi[:, :-4]).ravel(),
                         (roi[4:, :] - roi[:-4, :]).ravel()])

    # Immerkaer sigma: a Laplacian-style kernel that cancels smooth image
    # structure, so this tracks sensor noise rather than scene detail.
    lap = np.array([[1., -2., 1.], [-2., 4., -2.], [1., -2., 1.]])
    win = sliding_window_view(roi, (3, 3))
    sigma = float(np.abs((win * lap).sum((-2, -1))).mean() * np.sqrt(np.pi / 2) / 6)
    nvar = 2.0 * sigma * sigma    # variance of a difference of two noisy pixels

    e1 = max(float((d1 ** 2).mean()) - nvar, 0.0)
    e4 = max(float((d4 ** 2).mean()) - nvar, 0.0)

    # Below this there is no structure above the noise floor and the ratio is
    # meaningless.  Measured separation: pure noise peaks at 0.50, the weakest
    # real scene here sits at 2.61.
    if e4 < 1.5:
        _focus_state["ema"] = None
        return 0.0

    score = 100.0 * (e1 / e4) ** 0.5
    prev = _focus_state["ema"]
    score = score if prev is None else prev + FOCUS_SMOOTH * (score - prev)
    _focus_state["ema"] = score
    return float(score)


def save_frame(frame: bytes, idx: int, width: int, height: int) -> None:
    if not SAVE_TO_DISK:
        return

    # PNG keeps the raw frame lossless (a .jpg here would re-add compression).
    filename = f"image_{idx:03d}.png"

    try:
        from PIL import Image
    except Exception as e:
        print(f"Cannot save frame ({e})")
        return

    Image.frombytes("L", (width, height), frame).save(filename)

    print(f"Saved {filename}")


async def display_frames() -> None:
    while True:
        frame, idx, width, height = await display_queue.get()
        shown = _show_frame_realtime(frame, width, height)
        save_frame(frame, idx, width, height)
        focus = (f"  focus {_focus_score(frame, width, height):5.1f}"
                 if SHOW_FOCUS else "")
        print(f"Frame {idx} ({len(frame)} bytes){focus}  {_rssi_brief()}"
              f"{'' if shown else ' [decoded, NOT displayed]'}")


def _queue_display_frame(frame: bytes, idx: int, width: int, height: int) -> None:
    if display_queue is None:
        _show_frame_realtime(frame, width, height)
        save_frame(frame, idx, width, height)
        return

    try:
        display_queue.put_nowait((frame, idx, width, height))
    except asyncio.QueueFull:
        try:
            display_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass
        display_queue.put_nowait((frame, idx, width, height))


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

    # Matches BLE_IMAGE_MODE_RAW / _JPEG in ble_manager.c -- worth spelling out,
    # since it is the only way to confirm from the host which wire format the
    # firmware is actually built for.
    mode_name = {1: "RAW (lossless)", 2: "JPEG (lossy)"}.get(mode, "unknown")

    print(f"IMG INFO | tx_state={tx_state} mtu={mtu} width={width} bpp={bpp} "
          f"mode={mode} {mode_name} imu={imu}")

    # The Android client answers a small MTU with requestMtu(247).  Bleak has no
    # equivalent -- CoreBluetooth negotiates the MTU itself and exposes no way to
    # ask again -- so all that is left is to say so, since 20-byte payloads cut the
    # frame rate by roughly an order of magnitude.
    if 0 < mtu < 64:
        print(f"  WARNING: MTU {mtu} is tiny; expect very low fps."
              " Reconnect or re-pair to renegotiate.")


def _pop_packet() -> tuple | None:
    """Demux the TX stream: returns ("frame", pixels, width, height) or
    ("imu", temp_raw, [(ts_ms, ax, ay, az, gx, gy, gz), ...]), or None if no
    complete packet.  Frame dimensions come from the header, so a resolution
    change on the device is followed without restarting."""
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

            if not (1 <= width <= DIM_MAX and 1 <= height <= DIM_MAX):
                # Magic bytes appeared inside pixel data — skip them and resync.
                del frame_buffer[: len(RAW_MAGIC)]
                continue

            if len(frame_buffer) < RAW_HDR_LEN + width * height:
                return None

            frame = bytes(frame_buffer[RAW_HDR_LEN : RAW_HDR_LEN + width * height])
            del frame_buffer[: RAW_HDR_LEN + width * height]
            return ("frame", frame, width, height)

        if frame_buffer[:4] == JPG_MAGIC:
            if len(frame_buffer) < RAW_HDR_LEN:
                return None

            width = frame_buffer[4] | (frame_buffer[5] << 8)
            height = frame_buffer[6] | (frame_buffer[7] << 8)

            if not (1 <= width <= DIM_MAX and 1 <= height <= DIM_MAX):
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
            if img.size != (width, height):
                print(f"JPEG frame is {img.size}, header said "
                      f"{(width, height)}, dropping")
                continue
            # Hand decoded pixels downstream so display/recording code is
            # format-agnostic.
            return ("frame", img.tobytes(), width, height)

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

        _, frame, width, height = pkt
        frame_counter += 1
        stats["frames"] += 1

        _queue_display_frame(frame, frame_counter, width, height)

        print(f"Received frame {frame_counter} "
              f"({len(frame)} bytes, {width}x{height})")


async def send_command(client, opcode: int, payload: bytes = b"", note: str = "") -> bool:
    """Write one [opcode][payload] command, exactly as the Android client frames it
    (ImageTransferService.sendCommand).  Returns whether it reached the device."""
    if client is None or not client.is_connected:
        print("  not connected")
        return False

    pkt = bytes([opcode]) + payload
    name = CMD_NAMES.get(opcode, "Unknown")
    hexed = " ".join(f"{b:02x}" for b in payload) if payload else "-"

    try:
        # Acknowledged write, matching the app: Android's writeCharacteristic uses
        # WRITE_TYPE_DEFAULT, and the firmware characteristic permits both.  One ack
        # per hand-typed command costs nothing against a 19 kB frame.
        await client.write_gatt_char(IMG_RX_UUID, pkt, response=True)
    except Exception as e:
        print(f"  {name}({opcode}) failed: {e!r}")
        return False

    print(f"  -> {name}({opcode}) [{hexed}]{'  ' + note if note else ''}")
    return True


CONSOLE_HELP = """\
Commands (opcode in parentheses; 4-10 are accepted by this script and sent on the
wire, but the current firmware only acts on 1/2/3):
  single                       (1)  capture one frame
  stream                       (2)  start streaming
  stop                         (3)  stop streaming
  res <0-3>                    (4)  100x100 / 160x120 / 160x160 / 200x160
  phy <0-1>                    (5)  1Mbps / 2Mbps
  params                       (6)  request BLE params (also forces a fresh IMG INFO)
  angle <0-6>                  (7)  -45 -30 -15 0 +15 +30 +45 deg
  jpeg <0|1>                   (8)  off / on
  accel                        (9)  accelerometer-triggered capture
  reg <cam|accel> <AAAA> <VV>  (10) write register, hex: 2-byte addr, 1-byte value
  raw <hex> [hex...]           send arbitrary opcode + payload bytes
  help                         this list
  quit                         stop the stream and exit\
"""


def _parse_index(arg: str, count: int, what: str):
    """Shared parse for the index-payload commands.  Returns None and complains on
    anything the device would reject, so a typo never reaches the wire."""
    try:
        idx = int(arg, 0)
    except ValueError:
        print(f"  {what}: '{arg}' is not a number (expected 0-{count - 1})")
        return None
    if not 0 <= idx < count:
        print(f"  {what}: {idx} out of range (expected 0-{count - 1})")
        return None
    return idx


async def run_console_command(client, line: str) -> bool:
    """Execute one typed command line.  Returns False to end the session (quit)."""
    parts = line.split()
    if not parts:
        return True

    cmd, args = parts[0].lower(), parts[1:]

    def need(n: int) -> bool:
        if len(args) != n:
            print(f"  {cmd}: expected {n} argument{'' if n == 1 else 's'}, got {len(args)}")
            return False
        return True

    if cmd in ("help", "?"):
        print(CONSOLE_HELP)

    elif cmd == "single":
        await send_command(client, CMD_START_SINGLE_IMAGE)

    elif cmd == "stream":
        await send_command(client, CMD_START_STREAM)

    elif cmd == "stop":
        await send_command(client, CMD_STOP_STREAM)

    elif cmd == "params":
        await send_command(client, CMD_GET_BLE_PARAMS)

    elif cmd == "accel":
        await send_command(client, CMD_STREAM_ACCEL)

    elif cmd == "res":
        if need(1):
            idx = _parse_index(args[0], len(RESOLUTIONS), "res")
            if idx is not None:
                w, h = RESOLUTIONS[idx]
                await send_command(client, CMD_CHANGE_RESOLUTION, bytes([idx]), f"{w}x{h}")

    elif cmd == "phy":
        if need(1):
            idx = _parse_index(args[0], len(PHYS), "phy")
            if idx is not None:
                await send_command(client, CMD_CHANGE_PHY, bytes([idx]), PHYS[idx])

    elif cmd == "angle":
        if need(1):
            # Index, not degrees -- the app sends the spinner position, and taking
            # degrees would make "angle 0" ambiguous (index 0 is -45 deg).
            idx = _parse_index(args[0], len(ANGLES), "angle")
            if idx is not None:
                await send_command(client, CMD_CHANGE_ANGLE, bytes([idx]),
                                   f"{ANGLES[idx]:+d} deg")

    elif cmd == "jpeg":
        if need(1):
            idx = _parse_index(args[0], 2, "jpeg")
            if idx is not None:
                await send_command(client, CMD_JPEG, bytes([idx]),
                                   "on" if idx else "off")

    elif cmd == "reg":
        if need(3):
            dev = REG_DEVICES.get(args[0].lower())
            if dev is None:
                print(f"  reg: unknown device '{args[0]}' (expected cam or accel)")
                return True
            try:
                # Address is 2 bytes big-endian as typed and the value 1 byte,
                # matching MainActivity.hexStringToByteArray + the app's cmdData.
                addr = int(args[1], 16)
                val = int(args[2], 16)
            except ValueError:
                print(f"  reg: '{args[1]}' / '{args[2]}' are not hex")
                return True
            if not 0 <= addr <= 0xFFFF or not 0 <= val <= 0xFF:
                print("  reg: address must be 16-bit and value 8-bit")
                return True
            payload = bytes([dev, (addr >> 8) & 0xFF, addr & 0xFF, val])
            await send_command(client, CMD_WRITE_REG, payload,
                               f"{args[0].lower()} [0x{addr:04X}] = 0x{val:02X}")

    elif cmd == "raw":
        if not args:
            print("  raw: expected at least one hex byte (the opcode)")
            return True
        try:
            data = bytes(int(a, 16) for a in args)
        except ValueError:
            print("  raw: arguments must be hex bytes, e.g. 'raw 04 01'")
            return True
        if any(b > 0xFF for b in data):
            print("  raw: each argument must fit in one byte")
            return True
        await send_command(client, data[0], data[1:], "raw")

    elif cmd in ("quit", "exit"):
        await send_command(client, CMD_STOP_STREAM)
        return False

    else:
        print(f"  unknown command '{cmd}' -- type 'help'")

    return True


def start_stdin_reader(loop) -> asyncio.Queue:
    """Feed typed lines to the event loop from a daemon thread.  A thread rather
    than an async reader because stdin has to keep working across reconnects and a
    daemon thread needs no cancellation handling when the script exits."""
    queue = asyncio.Queue()

    def reader():
        for line in sys.stdin:
            loop.call_soon_threadsafe(queue.put_nowait, line)

    threading.Thread(target=reader, daemon=True, name="stdin").start()
    return queue


async def console_task(client, queue: asyncio.Queue, done: asyncio.Event) -> None:
    """Drain typed commands for as long as this session lasts."""
    # Anything typed while the link was down was aimed at a device that wasn't
    # listening; replaying it here would fire a burst of commands at connect.
    stale = 0
    while not queue.empty():
        queue.get_nowait()
        stale += 1
    if stale:
        print(f"  (discarded {stale} line(s) typed while disconnected)")

    while True:
        line = await queue.get()
        try:
            if not await run_console_command(client, line.strip()):
                done.set()
                return
        except Exception as e:
            print(f"  command error: {e!r}")


async def find_device():
    print("Scanning for BLE device...")

    # return_adv gives the AdvertisementData alongside each device, which is the
    # only place bleak 3.x carries RSSI -- BLEDevice.rssi was removed.
    found = await BleakScanner.discover(timeout=5.0, return_adv=True)

    # Strongest first: makes the reference list below directly readable.
    seen = sorted(found.values(), key=lambda pair: pair[1].rssi, reverse=True)

    target = None
    for device, adv in seen:
        if device.name == DEVICE_NAME:
            target = device
            rssi_state["adv"] = adv.rssi
            print(f"Found {device.name}: {device.address}  adv rssi {adv.rssi} dBm")
            break

    if target is None:
        print("Could not find device by name. Nearby devices:")
        for device, adv in seen:
            print(f"  {adv.rssi:4d} dBm | {device.name} | {device.address}")
        return None

    # The comparison that actually diagnoses an RF fault: same receiver, same
    # moment, insect_cam vs whatever else is in the room.
    if SHOW_SCAN_REFERENCE:
        others = [(d, a) for d, a in seen if d.address != target.address
                  and d.name][:SCAN_REFERENCE_COUNT]
        if others:
            print("  reference peripherals seen by this same radio:")
            for device, adv in others:
                delta = adv.rssi - rssi_state["adv"]
                print(f"    {adv.rssi:4d} dBm  ({delta:+3d} dB vs insect_cam)  {device.name}")

    return target


async def run_session(device, stdin_queue, quit_event) -> None:
    """One connect → stream → disconnect cycle. Returns when the link drops."""
    global frame_buffer
    global display_queue

    disconnected = asyncio.Event()

    def on_disconnect(_client):
        print("Disconnected.")
        disconnected.set()

    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        print("Connected")

        # Fresh start: drop any partial frame and zero the throughput counters.
        frame_buffer = bytearray()
        display_queue = asyncio.Queue(maxsize=2)
        display_task = asyncio.create_task(display_frames())
        console = asyncio.create_task(console_task(client, stdin_queue, quit_event))
        # Per-session: the previous link's readings say nothing about this one.
        rssi_state["last"] = None
        rssi_state["hist"].clear()
        rssi_state["note"] = None
        rssi_task = asyncio.create_task(rssi_monitor(client))
        stats["bytes"] = 0
        stats["frames"] = 0
        stats["imu"] = 0

        await client.start_notify(IMG_INFO_UUID, handle_img_info)
        await client.start_notify(IMG_TX_UUID, handle_image_chunk)
        print("Notifications enabled")

        await asyncio.sleep(0.5)

        # The app asks for the link parameters as soon as discovery finishes
        # (MainActivity, ACTION_GATT_SERVICES_DISCOVERED); do the same.  On this
        # firmware the opcode itself is ignored, but any RX write makes it push a
        # fresh IMG INFO, so this still reports the negotiated MTU right away.
        await send_command(client, CMD_GET_BLE_PARAMS)

        if AUTO_START_STREAM:
            print("Starting stream...")
            await send_command(client, CMD_START_STREAM)
        print("Type 'help' for commands.")

        # Report measured throughput once per second until the link drops.
        last = time.time()
        try:
            while not disconnected.is_set() and client.is_connected:
                # Sleeping on the quit event instead of the clock keeps the 1 s
                # throughput cadence but exits the moment 'quit' is typed.
                try:
                    await asyncio.wait_for(quit_event.wait(), timeout=1.0)
                    break
                except asyncio.TimeoutError:
                    pass
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
                          f"{nframes / dt:4.2f} fps  {nimu / dt:5.0f} imu/s  "
                          f"{_rssi_detail()}")
        finally:
            for task in (display_task, console, rssi_task):
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            display_queue = None
            _close_imu_log()
            # Best-effort stop; ignore errors if the link is already gone.
            try:
                await client.write_gatt_char(IMG_RX_UUID,
                                             bytes([CMD_STOP_STREAM]), response=False)
            except Exception:
                pass


async def main():
    """Scan/connect with automatic reconnect so the throughput test survives drops."""
    # One reader for the whole run: it has to outlive individual sessions so typing
    # still works while the script is between reconnect attempts.
    stdin_queue = start_stdin_reader(asyncio.get_running_loop())
    quit_event = asyncio.Event()

    while not quit_event.is_set():
        try:
            device = await find_device()
            if device is None:
                print("Device not found; retrying in 3 s...")
                await asyncio.sleep(3)
                continue
            await run_session(device, stdin_queue, quit_event)
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Session error: {e!r}")
        if quit_event.is_set():
            break
        print("Reconnecting in 2 s...  (Ctrl+C to quit)")
        await asyncio.sleep(2)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
