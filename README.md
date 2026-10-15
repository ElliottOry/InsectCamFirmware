# Kagafei Insect Camera

Zephyr / nRF Connect SDK firmware for the **Kagafei ES2832AA2** module (Nordic **nRF52832**). It drives a Himax
**HM01B0** camera and a TDK **ICM-42686** IMU. Each frame is JPEG-encoded on the device and streamed over
BLE as `KJPG` frames, with IMU samples (`KIMU`) sent between frames. A Python receiver on the host
connects, shows the frames live, and logs the IMU data to CSV.

## Requirements

### Hardware

- Kagafei ES2832AA2 board with the HM01B0 camera and ICM-42686 IMU fitted
- A **SEGGER J-Link** (a J-Link EDU Mini works) wired to the board's SWD pins. It is used for flashing
  and for the console: the console uses RTT because `uart0` is disabled, as its pins carry the camera's HSYNC/VSYNC
- A host with Bluetooth LE (developed on macOS)

### Firmware toolchain

| Tool | Version / notes |
|---|---|
| nRF Connect SDK | **v3.4.0**, installed under `/opt/nordic/ncs/v3.4.0` (install via nRF Connect for Desktop → Toolchain Manager, or `nrfutil sdk-manager`) |
| NCS toolchain bundle | `ccc010f809` (the bundle that matches v3.4.0), under `/opt/nordic/ncs/toolchains/` |
| SEGGER J-Link Software | `JLinkExe` on `PATH` (the Makefile adds `/usr/local/bin`) |
| GNU make | the macOS default `/usr/bin/make` is fine |

`nrfutil` is **not** required. The board's `runners.yaml` defaults to it, but the Makefile flashes
with `-r jlink`.

If your install lives somewhere else, override the paths:

```sh
make build NCS_ROOT=$HOME/ncs NCS_VERSION=v3.4.0 TOOLCHAIN_ID=<bundle id>
```

### Python receiver

- Python **3.11+** (developed on 3.14)
- Packages in [`requirements.txt`](requirements.txt): `bleak`, `numpy`, `matplotlib`, `pillow`, `opencv-python`

`make venv` creates `.venv/` and installs these packages. The receivers must run from that venv. If
`matplotlib` is missing, the receiver still connects and decodes frames but never opens a window. When
that happens, each frame's log line ends with `[decoded, NOT displayed]`.

## Quick start

```sh
make check     # confirm NCS, toolchain, west and J-Link are found
make flash     # build + flash over J-Link
make rtt       # watch the device console (Ctrl+C to quit)
make receive   # in another terminal: connect over BLE and show frames live
```

Run `make` with no target to list every target.

## Make targets

| Target | What it does |
|---|---|
| `build` | Incremental build into `build/` (`west build -p auto`) |
| `pristine` | Clean rebuild (`-p always`). Use it after editing board files, the DTS or `CMakeLists.txt` |
| `flash` | Build, then `west flash -r jlink` |
| `rtt` | RTT console over J-Link |
| `menuconfig` | Kconfig editor for the app image |
| `ram` / `rom` | Memory usage reports. RAM is tight, at about 90% of 64 KB |
| `clean` | Remove `build/` |
| `venv` | Create `.venv` and install `requirements.txt` |
| `receive` | `save_to_file_bt.py`: live image view and IMU CSV |
| `video` | `save_to_file_bt_video.py`: records the stream to a video file |
| `distclean` | Remove `build/`, `.venv/` and `__pycache__/` |

Useful overrides: `BOARD`, `BUILD_DIR`, `RUNNER`, `NCS_ROOT`, `NCS_VERSION`, `TOOLCHAIN_ID`, `PYTHON`.

### Why the Makefile `cd`s into the NCS tree

When `west build` runs from this directory, it fails with *"unknown command; do you need to run
this inside a workspace?"*. The Makefile exports `ZEPHYR_BASE`, `ZEPHYR_SDK_INSTALL_DIR` and the
toolchain `PATH`. It then runs west from `$(NCS_DIR)` and points west back at this project with
`-d build/`.

## Project layout

```
boards/Kagafei/Kagafei_ES2832AA2/   custom board definition (DTS, pinctrl, defconfig)
dts/bindings/                       bindings for the HM01B0 and the SPIS capture device
src/main.c                          entry point
src/ble_manager.c                   BLE peripheral "insect_cam", command handling, frame/IMU streaming
src/HM01B0/                         camera: MCLK (TIMER→PPI→GPIOTE), I2C regs, SPIS pixel capture
src/icm42686/, src/imu_stream.c     IMU driver and batching
JPEGEncoder4Cortex-M/               on-device JPEG encoder (fixed-point DCT)
prj.conf                            Kconfig. Stack sizes in it were measured, see the comments
save_to_file_bt*.py                 host-side BLE receivers
```

`main_old.c`, `main_bringup.c`, `HM01B0/other_cap.c` and `HM01B0_UART.c` are kept for reference and
are intentionally left out of the build.

## Troubleshooting

- **`VTref=0.000V` / "Target voltage too low" when flashing.** The board is unpowered or the SWD lead
  is loose. This is a hardware problem, not a tooling problem.
- **The receiver can't find the device.** It scans for the name `insect_cam`
  (`CONFIG_BT_DEVICE_NAME`). If you change that name, update `DEVICE_NAME` in the receiver scripts too.
- **Frames arrive but no window opens.** Run `.venv/bin/pip list` and check that `matplotlib` is
  installed.
- **An MPU fault after you add a `printk`.** Logging runs in immediate mode, so a `printk` uses the
  calling thread's stack. That costs 400 B or more per message. Keep `printk` calls in BLE callbacks
  short, or move them to a work item.
