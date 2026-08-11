cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(kagafei_config)

# Deliberately excluded: src/HM01B0/other_cap.c (legacy capture path,
# duplicate symbols), HM01B0_UART.c (uart0 is disabled), and main_old.c /
# main_bringup.c (kept for reference).  JPEGEncoder4Cortex-M encodes each
# frame on-device (with a rewritten fixed-point DCT); BLE streams KJPG frames.
target_sources(app PRIVATE
  src/main.c
  src/ble_manager.c
  src/i2c_diag.c
  src/spis_preinit.c
  src/imu_stream.c
  src/icm42686/icm42686.c
  src/HM01B0/HM01B0_CAPTURE.c
  src/HM01B0/HM01B0_SPI.c
  src/HM01B0/HM01B0_I2C.c
  src/HM01B0/HM01B0_CLK.c
  src/HM01B0/HM01B0_BLE.c
  JPEGEncoder4Cortex-M/jpegencoder.c
)

target_include_directories(app PRIVATE src src/HM01B0 src/icm42686 JPEGEncoder4Cortex-M)
