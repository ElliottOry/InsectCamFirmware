#include "HM01B0_BLE.h"

extern struct bt_conn *default_conn;


/* The streaming thread carries the deepest call chain in the app: pack +
 * bt_gatt_notify -> ATT -> L2CAP -> HCI.  At 768 B it OVERFLOWED (silently, no MPU
 * guard then) and trampled neighboring RAM — the idle thread died of a UsageFault
 * ~10 s in.  2048 B gives the chain real headroom; the manager thread also calls
 * bt_gatt_notify (img_info) so it gets 1024. */
#define BLE_MGR_STACK_SZ    1024
#define BLE_STREAM_STACK_SZ 2048

static struct k_fifo *the_fifo;

/* BLE manager thread (handles advertising/connection state) */
K_THREAD_STACK_DEFINE(ble_mgr_stack, BLE_MGR_STACK_SZ);
static struct k_thread ble_mgr_thread_data;

/* BLE streaming thread (drains FIFO and notifies over BLE) */
K_THREAD_STACK_DEFINE(ble_stream_stack, BLE_STREAM_STACK_SZ);
static struct k_thread ble_stream_thread_data;

/* Forward for manager entry */
static void ble_manager_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    /* Run BLE peripheral state machine forever */
    run_peripheral_step(0);
}

void hm_ble_init(struct k_fifo *fifo)
{
    the_fifo = fifo;

    /* Initialize BLE stack and start advertising */
    init_ble();

    /* Start BLE core (advertising/connection/MTU updates) */
    k_thread_create(&ble_mgr_thread_data, ble_mgr_stack, BLE_MGR_STACK_SZ,
                    ble_manager_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);

    /* Start BLE streaming worker (pulls frames from FIFO and sends) */
    k_thread_create(&ble_stream_thread_data, ble_stream_stack, BLE_STREAM_STACK_SZ,
                    streaming_photos_ble_fifo, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
}

void streaming_photos_ble_fifo(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        /* No central: return any in-flight frame's buffer credit so the capture
         * producer can park cleanly, then sleep until the next connect — the
         * 100 ms IMU-flush poll below would otherwise wake the CPU forever for
         * an empty FIFO. */
        if (!ble_is_connected()) {
            frame_item_t *stale;
            while ((stale = k_fifo_get(the_fifo, K_NO_WAIT)) != NULL) {
                if (stale->consumed_sem) {
                    k_sem_give(stale->consumed_sem);
                }
                k_free(stale);
            }
            ble_wait_for_connection();
            continue;
        }

        /* Short timeout instead of K_FOREVER so buffered IMU samples still go
         * out while no frames arrive (AE converge window, capture stalls). */
        frame_item_t *item = k_fifo_get(the_fifo, K_MSEC(100));
        if (item == NULL) {
            ble_stream_imu();
            continue;
        }

        /* IMU samples accumulated during the previous frame's transmit go out
         * first -- KIMU packets may only sit BETWEEN KRAW frames. */
        ble_stream_imu();

        /* Downsample + transmit straight from the capture buffer, pacing behind the
         * in-progress capture.  The buffer is held for the whole transmit (released
         * only after): transmit is the bottleneck and the capture hides inside it, so
         * this stays transmit-bound (~3 fps) without needing a frame-size copy. */
        ble_stream_frame(item->data, item->lines_done);

        if (item->consumed_sem) {
            k_sem_give(item->consumed_sem);
        }
        /* Always free the FIFO wrapper */
        k_free(item);
    }
}
