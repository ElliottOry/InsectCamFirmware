/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Author: tekritesh@github.com
 */

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include "HM01B0/HM01B0_def_values.h"
#include "imu_stream.h"
#include "jpegencoder.h"

#define BT_UUID_IMG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_RX_VAL \
    BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_TX_VAL \
    BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_INFO_VAL \
    BT_UUID_128_ENCODE(0x6e400004, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)

/* Overhead: opcode (u8) + handle (u16) */
#define ATT_NTF_SIZE(payload_len) (1 + 2 + payload_len)

// #define IMG_WIDTH  160
// #define IMG_HEIGHT 119
// extern uint8_t image[];

#define CHUNK_SIZE CONFIG_BT_L2CAP_TX_MTU

/* Wire format for the BLE live view.  Both formats share the same 8-byte frame
 * header the receiver syncs on: a 4-byte magic, then width and height as u16 LE.
 *
 *   RAW (1): every captured pixel byte verbatim, 8-bit grayscale, no encoding of
 *            any kind -- 8 + 160*119 = 19048 B/frame.  Lossless: what the sensor
 *            produced is exactly what lands on disk.  ~1.1 fps on this link.
 *   JPEG(0): q90 grayscale, ~3.7 KB/frame, ~3 fps.  Body ends at the EOI marker
 *            (FF D9), which entropy byte-stuffing guarantees cannot appear inside
 *            the compressed data.  Lossy.  (current)
 *
 * JPEG is selected because the link, not the sensor, is the bottleneck: ~5x less
 * data per frame buys ~3x the frame rate.  The encoder never holds a whole JPEG in
 * RAM -- jpg_write() feeds the notification stager as bits are produced -- so the
 * only cost over RAW is the 1344 B encoder state, which RAW compiles out.
 * save_to_file_bt.py decodes both and needs no change to follow this switch. */
#define BLE_STREAM_RAW 0

#define BLE_OUT_W IMG_WIDTH
#define BLE_OUT_H IMG_HEIGHT
#define BLE_IMAGE_MODE_RAW  1
#define BLE_IMAGE_MODE_JPEG 2

#if BLE_STREAM_RAW
#define BLE_IMAGE_MODE BLE_IMAGE_MODE_RAW
#else
#define BLE_IMAGE_MODE BLE_IMAGE_MODE_JPEG
#endif

static const uint8_t frame_hdr[8] = {
#if BLE_STREAM_RAW
    'K', 'R', 'A', 'W',
#else
    'K', 'J', 'P', 'G',
#endif
    IMG_WIDTH & 0xFF, (IMG_WIDTH >> 8) & 0xFF,
    IMG_HEIGHT & 0xFF, (IMG_HEIGHT >> 8) & 0xFF,
};

#if !BLE_STREAM_RAW
/* Encoder state (1344 B) + per-frame source context for the callbacks.
 * Static is safe: only the BLE streaming thread encodes, one frame at a time. */
static jpeg_data jpg;
static const uint8_t *jpg_src;
static atomic_t *jpg_lines_done;
#endif

#define BLE_STAGE_MAX 244
static uint8_t ble_stage[BLE_STAGE_MAX];
static uint16_t ble_stage_len;
static uint16_t ble_max_payload;
static uint32_t tx_bytes_sent;
static uint32_t tx_notify_wait_ms;
static uint16_t tx_notify_chunks;
static struct bt_conn *tx_notify_conn;
static const struct bt_gatt_attr *tx_notify_attr;
static uint32_t tx_frame_count;




// BLE Characteristics
static const struct bt_uuid_128 insect_cam_service = BT_UUID_INIT_128(BT_UUID_IMG_SERVICE_VAL);

static const struct bt_uuid_128 tx_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_IMG_TX_VAL);

static const struct bt_uuid_128 rx_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_IMG_RX_VAL);

static const struct bt_uuid_128 img_info_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_IMG_INFO_VAL);

static const struct bt_data adv_ad_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMG_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};


// BLE Variables
struct bt_conn *default_conn;
static struct k_work advertise_work;
static struct k_work link_info_work;

/* Connection gate for the whole pipeline: the bit is set while a central is
 * connected.  The capture loop, BLE streaming thread and IMU sampler block on
 * this (instead of polling) while disconnected, so the CPU actually idles and
 * only the advertiser draws power between connections. */
#define CONN_EVT_CONNECTED BIT(0)
static struct k_event conn_evt;

bool ble_is_connected(void)
{
	return default_conn != NULL;
}

void ble_wait_for_connection(void)
{
	k_event_wait(&conn_evt, CONN_EVT_CONNECTED, false, K_FOREVER);
}

/* macOS gives each BLE connection only a few ms of radio time per connection event
 * (Wi-Fi coexistence), so throughput scales with EVENTS PER SECOND: the interval must
 * come down from macOS's default 30 ms to 15 ms.  Measured on this link: 30 ms events
 * carry only ~460 B (~2 DLE packets) => ~15 KB/s => 0.8 fps.
 *
 * macOS also picks the MAXIMUM of any offered interval range (offered 15-30 ms, it
 * granted 30.00 ms), so the request must offer EXACTLY 15 ms.  Apple's accessory
 * guidelines ask for max >= min + 15 ms, but in practice Apple accepts an exact
 * 15 ms request — if this central refuses it, later attempts fall back to the
 * guideline-compliant 15-30 ms range so we at least keep a valid negotiation.
 *
 * Sent from delayed work, not the connected() callback (Apple centrals commonly
 * ignore requests before discovery/subscription settle), and re-checked/retried on a
 * timer, Nordic ble_conn_params-style.  The handler reads the ACTUAL interval from
 * bt_conn_get_info() so a granted update stops the retries. */
#define CONN_PARAM_MAX_ATTEMPTS 4
static struct k_work_delayable conn_param_work;
static int conn_param_attempts;

static void conn_param_work_handler(struct k_work *work)
{
	struct bt_conn_info info;

	if (default_conn == NULL) {
		return;
	}
	if (bt_conn_get_info(default_conn, &info) == 0 && info.le.interval <= 12) {
		printk("Conn interval goal reached (%u units)\n", info.le.interval);
		return;
	}
	if (conn_param_attempts >= CONN_PARAM_MAX_ATTEMPTS) {
		printk("Conn param: central kept the long interval; giving up\n");
		return;
	}
	conn_param_attempts++;
	bool exact = (conn_param_attempts <= 2);   /* try exact 15 ms first */
	int err = bt_conn_le_param_update(default_conn,
			exact ? BT_LE_CONN_PARAM(12, 12, 0, 400)
			      : BT_LE_CONN_PARAM(12, 24, 0, 400));
	printk("Conn param request #%d (%s): %d\n", conn_param_attempts,
	       exact ? "15 ms exact" : "15-30 ms", err);
	k_work_schedule(&conn_param_work, K_SECONDS(4));
}


// Stream-specific state
uint16_t ble_mtu = 0;
uint8_t tx_state = 0;

bool char_tx = false;
bool notif_state[2] = {false, false};

static ssize_t on_rx_received(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf,
                              uint16_t len,
                              uint16_t offset,
                              uint8_t flags);

/* Implemented in this file; referenced by BLE streamer */
void ble_stream_frame(const uint8_t *image, atomic_t *lines_done);




void advertising_work_handler(struct k_work *work) {
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, adv_ad_data, ARRAY_SIZE(adv_ad_data), NULL, 0);
}

/* What the link actually negotiated (interval / PHY / data length / MTU), logged
 * once per stream start.  Submitted from the ATT write callback but run here: at
 * seven arguments this is the single most stack-hungry printk in the app, and the
 * BT RX thread cannot afford it under LOG_MODE_IMMEDIATE. */
static void link_info_work_handler(struct k_work *work)
{
	struct bt_conn *conn = default_conn;
	struct bt_conn_info info;

	if (!conn || bt_conn_get_info(conn, &info) != 0) {
		return;
	}
	printk("Link: interval=%u.%02u ms, PHY tx/rx=%u/%u, dlen tx/rx=%u/%u, mtu=%u\n",
	       (info.le.interval * 5) / 4, ((info.le.interval * 5) % 4) * 25,
	       info.le.phy ? info.le.phy->tx_phy : 0,
	       info.le.phy ? info.le.phy->rx_phy : 0,
	       info.le.data_len ? info.le.data_len->tx_max_len : 0,
	       info.le.data_len ? info.le.data_len->rx_max_len : 0,
	       bt_gatt_get_uatt_mtu(conn));
}

static void ccc_cfg_changed_tx(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_state[1] = notif_enabled;
    printk("Tx Notifications %s\n", notif_enabled ? "enabled" : "disabled");

}

static void ccc_cfg_changed_img_info(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_state[0] = notif_enabled;
    printk("Img Info Notifications %s\n", notif_enabled ? "enabled" : "disabled");

}
         
BT_GATT_SERVICE_DEFINE(
    insect_cam,
    BT_GATT_PRIMARY_SERVICE(&insect_cam_service),
    BT_GATT_CHARACTERISTIC(&tx_characteristic_uuid.uuid,(BT_GATT_CHRC_NOTIFY), (BT_GATT_PERM_NONE), NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_tx, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&img_info_characteristic_uuid.uuid,(BT_GATT_CHRC_NOTIFY), (BT_GATT_PERM_NONE), NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_img_info, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&rx_characteristic_uuid.uuid,(BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_WRITE), BT_GATT_PERM_WRITE, NULL, on_rx_received, NULL)
    );

/* Push one notification, retrying while the controller's TX queue is full.  Returns
 * 0 on success, -1 if the link is wedged or dropped (caller abandons the frame). */
static int ble_notify_chunk(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const uint8_t *data, uint16_t len)
{
    int64_t deadline = k_uptime_get() + 2000;

    for (;;) {
        int err = bt_gatt_notify(conn, attr, data, len);
        if (err == 0) {
            tx_notify_chunks++;
            return 0;
        }
        if (err == -ENOMEM || err == -EAGAIN) {
            /* TX buffers full: SLEEP briefly (not k_yield) so lower-priority threads
             * can drain the controller's TX queue — k_yield only schedules equal/
             * higher priority and can busy-spin forever here.  Retry the same chunk. */
            if (k_uptime_get() > deadline) {
                printk("BLE Tx stalled, dropping rest of frame\n");
                return -1;
            }
            k_msleep(1);
            tx_notify_wait_ms++;
            continue;
        }
        printk("Img Tx Notify failed: %d\n", err);   /* e.g. disconnect — stop */
        tx_state = 0;
        return -1;
    }
}

static int raw_wait_for_rows(atomic_t *lines_done, uint32_t rows_needed)
{
    if (!lines_done) {
        return 0;
    }

    int64_t deadline = k_uptime_get() + 2000;

    while ((uint32_t)atomic_get(lines_done) < rows_needed) {
        if (k_uptime_get() > deadline) {
            printk("BLE raw stream stalled waiting on capture, dropping frame\n");
            return -1;
        }
        k_msleep(1);
    }

    return 0;
}

static int ble_flush_stage(void)
{
    if (ble_stage_len == 0) {
        return 0;
    }
    if (ble_notify_chunk(tx_notify_conn, tx_notify_attr, ble_stage, ble_stage_len) != 0) {
        return -1;
    }
    ble_stage_len = 0;
    return 0;
}

static int ble_write_output(const uint8_t *data, uint32_t len)
{
    uint32_t pos = 0;

    while (pos < len) {
        uint16_t room = ble_max_payload - ble_stage_len;
        uint16_t copy_len = MIN((uint32_t)room, len - pos);

        memcpy(&ble_stage[ble_stage_len], &data[pos], copy_len);
        ble_stage_len += copy_len;
        pos += copy_len;

        if (ble_stage_len >= ble_max_payload && ble_flush_stage() != 0) {
            return -1;
        }
    }

    tx_bytes_sent += len;
    return (int)len;
}

#if !BLE_STREAM_RAW
/* Encoder callbacks.  Reads are in MCU order (8-byte row segments, top band
 * to bottom), so pacing on the capture's line counter works exactly like the
 * raw path: each read waits only until the capture has produced that row.
 * Writes go straight into the notification stager — the JPEG is never held in
 * RAM as a whole. */
static int jpg_read(uint_t pos, uint8_t *dst, uint_t len)
{
    if (raw_wait_for_rows(jpg_lines_done, pos / IMG_WIDTH + 1) != 0) {
        return -1;
    }
    memcpy(dst, &jpg_src[pos], len);
    return (int)len;
}

static int jpg_write(uint8_t *d, uint_t len)
{
    return (ble_write_output(d, len) < 0) ? -1 : (int)len;
}
#endif /* !BLE_STREAM_RAW */

/* Stream one capture out the TX characteristic: the 8-byte sync header, then
 * the frame body (see BLE_STREAM_RAW).  The capture thread enqueues the frame
 * at capture START and holds buffer 0 (NUM_BUFS == 1) until we release it, so
 * the body must be paced on the capture's line counter — raw_wait_for_rows()
 * blocks until the row we are about to send has actually landed.  On any
 * mid-frame drop the receiver resyncs on the next frame's header. */
void ble_stream_frame(const uint8_t *image, atomic_t *lines_done)
{
    struct bt_conn *conn = default_conn;
    if (!conn) {
        printk("No active connection\n");
        return;
    }
    /* The streaming thread runs the moment a client connects, but the central may
     * not have enabled TX notifications yet (CCC unwritten) — notifying then just
     * floods the log with -EINVAL "not subscribed".  notif_state[1] is set by the
     * TX CCC callback; skip the frame quietly until it's true. */
    if (!notif_state[1]) {
        return;
    }
    const struct bt_gatt_attr *attr =
        bt_gatt_find_by_uuid(insect_cam.attrs, 0xffff, &tx_characteristic_uuid.uuid);
    if (!attr) {
        printk("Tx Notify attribute not found!\n");
        return;
    }

    uint16_t mtu = bt_gatt_get_uatt_mtu(conn);
    uint16_t max_payload = (mtu > 3) ? (mtu - 3) : 20;   /* ATT notification overhead */
    if (max_payload > BLE_STAGE_MAX) {
        max_payload = BLE_STAGE_MAX;
    }

    tx_notify_conn = conn;
    tx_notify_attr = attr;
    ble_stage_len = 0;
    ble_max_payload = max_payload;
    tx_bytes_sent = 0;
    tx_notify_wait_ms = 0;
    tx_notify_chunks = 0;

    int64_t send_start = k_uptime_get();

    if (ble_write_output(frame_hdr, sizeof(frame_hdr)) < 0) {
        return;
    }

#if BLE_STREAM_RAW
    /* One row at a time, each gated on the capture having produced it.  The
     * stager packs rows across notification boundaries, so no payload is
     * wasted on the 160-byte row length. */
    for (uint32_t row = 0; row < IMG_HEIGHT; row++) {
        if (raw_wait_for_rows(lines_done, row + 1) != 0) {
            return;                    /* capture stalled: receiver resyncs */
        }
        if (ble_write_output(&image[row * IMG_WIDTH], IMG_WIDTH) < 0) {
            return;                    /* link wedged/dropped */
        }
    }
    int body_len = IMG_WIDTH * IMG_HEIGHT;
#else
    /* Re-init the encoder state every frame (the library requires it: DC
     * predictor, bitstream remainder and data_len all carry over otherwise;
     * mcu_*_max == 0 makes encode_image recompute the geometry). */
    memset(&jpg, 0, sizeof(jpg));
    jpg.width   = IMG_WIDTH;
    jpg.height  = IMG_HEIGHT;
    jpg.c_info  = color_info_gray_scale;
    jpg.quality = normal;              /* q90 tables: good quality, ~4-6x smaller */
    jpg_src = image;
    jpg_lines_done = lines_done;

    int body_len = encode_image(&jpg, jpg_read, jpg_write);
    if (body_len < 0) {
        return;                        /* link wedged/dropped: receiver resyncs */
    }
#endif

    if (ble_flush_stage() != 0) {
        return;
    }
    int32_t send_ms = (int32_t)(k_uptime_get() - send_start);

    if ((++tx_frame_count % 10) == 0) {
        printk("%s frame %u: %d bytes, %d ms, %u chunks, %u ms tx-wait\n",
               BLE_STREAM_RAW ? "RAW" : "JPG",
               tx_frame_count, body_len, send_ms,
               tx_notify_chunks, tx_notify_wait_ms);
    }
}

/* KIMU packet: 12-byte header ("KIMU" magic, count u8, format u8, base_ts_ms
 * u32 LE, temp_raw s16 LE) + count * 14-byte samples (dt_ms u16 offset from
 * base_ts, then accel X/Y/Z and gyro X/Y/Z as s16 LE raw counts).  Sent ONLY
 * between image frames: pixel bytes are arbitrary, so a mid-frame insert would
 * corrupt the receiver's byte counting.  Max packet 12+16*14=236 B, always
 * within one notification at the negotiated 247 MTU. */
#define KIMU_FORMAT      0x01
#define KIMU_MAX_SAMPLES 16
#define KIMU_HDR_LEN     12
#define KIMU_SAMPLE_LEN  14

/* Drain buffered IMU samples out the TX characteristic.  Same single-writer
 * rule as ble_stream_frame: only the BLE streaming thread may call this. */
void ble_stream_imu(void)
{
    if (!imu_stream_active() || imu_stream_pending() == 0) {
        return;
    }
    struct bt_conn *conn = default_conn;
    if (!conn || !notif_state[1]) {
        return;
    }
    const struct bt_gatt_attr *attr =
        bt_gatt_find_by_uuid(insect_cam.attrs, 0xffff, &tx_characteristic_uuid.uuid);
    if (!attr) {
        return;
    }

    uint16_t mtu = bt_gatt_get_uatt_mtu(conn);
    uint16_t max_payload = (mtu > 3) ? (mtu - 3) : 20;   /* ATT notification overhead */
    if (max_payload > BLE_STAGE_MAX) {
        max_payload = BLE_STAGE_MAX;
    }

    tx_notify_conn = conn;
    tx_notify_attr = attr;
    ble_stage_len = 0;
    ble_max_payload = max_payload;

    /* Static: the streaming thread's 2 KB stack has no room to spare. */
    static uint8_t pkt[KIMU_HDR_LEN + KIMU_MAX_SAMPLES * KIMU_SAMPLE_LEN];
    struct imu_sample s;

    while (imu_stream_pop(&s)) {
        uint32_t base_ts = s.ts_ms;
        uint16_t off = KIMU_HDR_LEN;
        uint8_t n = 0;

        do {
            sys_put_le16((uint16_t)(s.ts_ms - base_ts), &pkt[off]);
            sys_put_le16((uint16_t)s.accel[0], &pkt[off + 2]);
            sys_put_le16((uint16_t)s.accel[1], &pkt[off + 4]);
            sys_put_le16((uint16_t)s.accel[2], &pkt[off + 6]);
            sys_put_le16((uint16_t)s.gyro[0],  &pkt[off + 8]);
            sys_put_le16((uint16_t)s.gyro[1],  &pkt[off + 10]);
            sys_put_le16((uint16_t)s.gyro[2],  &pkt[off + 12]);
            off += KIMU_SAMPLE_LEN;
            n++;
        } while (n < KIMU_MAX_SAMPLES && imu_stream_pop(&s));

        pkt[0] = 'K'; pkt[1] = 'I'; pkt[2] = 'M'; pkt[3] = 'U';
        pkt[4] = n;
        pkt[5] = KIMU_FORMAT;
        sys_put_le32(base_ts, &pkt[6]);
        sys_put_le16((uint16_t)imu_stream_last_temp_raw(), &pkt[10]);

        if (ble_write_output(pkt, off) < 0) {
            return;   /* link wedged/dropped: abandon, receiver resyncs on next magic */
        }
    }

    /* Must not leave bytes staged: ble_stream_frame resets ble_stage_len. */
    ble_flush_stage();
}


/* Anything logged from here runs on the BT host RX thread, and under
 * LOG_MODE_IMMEDIATE each printk formats + writes RTT on that thread's stack.
 * Keep this callback to ONE short printk: the seven-argument link snapshot
 * used to live inline and blew the RX stack straight into the MPU guard.  It
 * now runs from link_info_work on the system workqueue (2 KB, prj.conf). */
static ssize_t on_rx_received(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf,
                              uint16_t len,
                              uint16_t offset,
                              uint8_t flags) {
    const uint8_t *rx_data = buf;
    char hex[3 * 8 + 4] = {0};
    uint16_t shown = MIN(len, 8);
    int pos = 0;

    for (uint16_t i = 0; i < shown; i++) {
        pos += snprintk(&hex[pos], sizeof(hex) - pos, "%02x ", rx_data[i]);
    }
    if (shown < len) {
        snprintk(&hex[pos], sizeof(hex) - pos, "...");
    }
    printk("RX received %u bytes --> %s\n", len, hex);

    char_tx = true;


    if (len == 1 && rx_data[0] == 1){
        tx_state = 1;
        printk("Enabling Single Image..\n");
        // send_large_data(conn);
    }

    if (len == 1 && rx_data[0] == 2){
        printk("Enabling Stream..\n");
        tx_state = 2;
        /* Snapshot the negotiated link right when streaming starts so every test run
         * logs what actually got applied -- deferred to the workqueue, see above. */
        k_work_submit(&link_info_work);
    }

    if(len == 1 && rx_data[0] == 3){ 
        printk("Disabling Stream..\n");
        tx_state = 0;

    }

    return len;
}



void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated,
};

static void connected(struct bt_conn *conn, uint8_t err) {
	if (err != 0) {
		printk("BLE connect failed (err 0x%02x)\n", err);
		return;
	}

	printk("BLE connected\n");
	default_conn = bt_conn_ref(conn);

	/* Ask for a 15 ms interval (see CONN_PARAM_APPLE) once the central has settled.
	 * 2M PHY + data length stay AUTO-negotiated; do NOT force the PHY (it crashes this
	 * SoftDevice Controller — HCI LE Set PHY 0x2032 timeout). */
	conn_param_attempts = 0;
	k_work_schedule(&conn_param_work, K_SECONDS(2));

	k_event_post(&conn_evt, CONN_EVT_CONNECTED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	/* reason 0x08 = supervision timeout (link lost), 0x13 = remote user terminated,
	 * 0x16 = local terminated, 0x3e = failed to establish. */
	printk("BLE disconnected, reason 0x%02x\n", reason);
	k_work_cancel_delayable(&conn_param_work);
	bt_conn_unref(conn);
	default_conn = NULL;
	k_event_clear(&conn_evt, CONN_EVT_CONNECTED);
	k_work_submit(&advertise_work);
}

/* --- Link diagnostics: report what the connection actually negotiated --------- */
static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
	/* interval is in 1.25 ms units; timeout in 10 ms units. */
	printk("LE interval=%u.%02u ms, latency=%u, timeout=%u ms\n",
	       (interval * 5) / 4, ((interval * 5) % 4) * 25, latency, timeout * 10);
	/* Retry pacing lives in conn_param_work_handler (it reads the real interval). */
}

/* PHY / data-length REPORTING.  CONFIG_BT_USER_PHY_UPDATE / _DATA_LEN_UPDATE are
 * enabled only so these callbacks exist and bt_conn_get_info() exposes the values;
 * CONFIG_BT_AUTO_* stay explicitly ON so the host still negotiates by itself.
 * NEVER call bt_conn_le_phy_update() here: commanding the PHY crashed this
 * SoftDevice Controller (HCI LE Set PHY 0x2032 timeout).  If these ever report
 * 1M PHY or 27-byte data length, that alone is a 2-4x throughput loss. */
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	printk("PHY updated: TX %u RX %u (1=1M, 2=2M)\n", param->tx_phy, param->rx_phy);
}

static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
	printk("Data len updated: TX %u RX %u bytes\n",
	       info->tx_max_len, info->rx_max_len);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_len_updated,
};

void init_ble(){

    int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	k_event_init(&conn_evt);
	bt_gatt_cb_register(&gatt_callbacks);

	k_work_init(&advertise_work, advertising_work_handler);
	k_work_init(&link_info_work, link_info_work_handler);
	k_work_init_delayable(&conn_param_work, conn_param_work_handler);

    
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, adv_ad_data, ARRAY_SIZE(adv_ad_data), NULL, 0);
	

    

}

/* Manager loop: keeps the img_info characteristic in sync with the stream state
 * and the negotiated MTU.  Runs forever on the BLE manager thread; `seconds` is
 * kept for the existing call signature and ignored.  Streaming itself is handled
 * by the BLE streaming worker thread. */
void run_peripheral_step(uint16_t seconds) {

    ARG_UNUSED(seconds);

    struct bt_gatt_attr *img_info_notify_attr_global = bt_gatt_find_by_uuid(insect_cam.attrs, 0xffff, &img_info_characteristic_uuid.uuid);

    if (!img_info_notify_attr_global) {
        printk("Img Info notify attribute not found!\n");
        return;
    }

    while (1) {
        if (default_conn == NULL) {
            printk("Idle,Ready to Pair....\n");
            ble_mtu = 0;
            tx_state = 0;
            /* Same connection gate the capture and streaming threads block on
             * (cleared in disconnected()), so this thread costs nothing between
             * connections.  conn_sem's binary give/take does not survive a
             * reconnect cleanly. */
            ble_wait_for_connection();
            continue;
        }

        /* Notifying before the central writes the img_info CCC returns -EINVAL
         * (CONFIG_BT_GATT_ENFORCE_SUBSCRIPTION).  notif_state[0] is set by
         * ccc_cfg_changed_img_info; ble_mtu starts at 0, so the first notify
         * still fires the moment the subscription lands. */
        if (notif_state[0]) {
            uint16_t current_mtu = bt_gatt_get_uatt_mtu(default_conn);

            if ((char_tx == true) || (current_mtu != ble_mtu)) {
                ble_mtu = current_mtu;
                char_tx = false;
                uint8_t img_info[8] = {tx_state,current_mtu & 0xFF,(current_mtu >> 8) & 0xFF,BLE_OUT_W & 0xFF,(BLE_OUT_W >> 8) & 0xFF,8,BLE_IMAGE_MODE,imu_stream_active() ? 1 : 0};
                int err = bt_gatt_notify(default_conn, img_info_notify_attr_global, &img_info, sizeof(img_info));
                if (err) {
                    printk("Img Information Notify failed: %d\n", err);
                } else {
                    printk("Tx pkt:tx[%d] mtu[%d]\n",tx_state,current_mtu);
                }
            }
        }

        /* Only ticks while connected -- matches the streaming thread's existing
         * 100 ms FIFO poll, so no new wakeup class while advertising. */
        k_msleep(100);
    }
}
