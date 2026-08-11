#include "HM01B0_UART.h"

#define UART_THREAD_STACK_SZ 768

/* 2x2-average binning before UART TX.  Full-res frames (19040 B) take ~207 ms at
 * 921600 baud => ~4.2 fps, wire-limited.  Binned 80x59 frames (4720 B) take ~51 ms,
 * which fits inside the sensor's ~79 ms frame period — each output row needs two
 * captured source rows (1.29 ms) but only 0.87 ms of wire time, so TX runs in
 * lock-step just behind capture and finishes ~1 ms after the last line.  The stream
 * then locks to the sensor cadence: ~12.6 fps, the camera's ceiling at this MCLK.
 * Receiver must match ("save_to_file copy3.py" BIN2 flag).  0 = full 160x119. */
#define UART_BIN2 1

#if UART_BIN2
#define UART_OUT_W (IMG_WIDTH / 2)    /* 80 */
#define UART_OUT_H (IMG_HEIGHT / 2)   /* 59 — odd source row 118 is dropped */
#endif

static struct k_fifo *the_new_fifo;

K_THREAD_STACK_DEFINE(uart_stack, UART_THREAD_STACK_SZ);
static struct k_thread uart_thread_data;

void send_frame_over_uart_binary(uint8_t *image, atomic_t *lines_done)
{
static const char hdr[]  = "<FRAME>\n";
static const char tail[] = "</FRAME>\n";

for (int i = 0; i < sizeof hdr - 1; i++)  uart_poll_out(uart_dev, hdr[i]);
#if UART_BIN2
for (int y = 0; y < UART_OUT_H; y++)
{
    /* Output row y averages source rows 2y and 2y+1: wait until both are behind
     * the capture write pointer, bailing out if capture wedges so we never hang. */
    int64_t wait_deadline = k_uptime_get() + 2000;
    while (lines_done && atomic_get(lines_done) < 2 * (y + 1)) {
        if (k_uptime_get() > wait_deadline) {
            printk("UART Tx stalled waiting on capture, dropping rest of frame\n");
            goto tail_out;
        }
        k_msleep(1);
    }

    const uint8_t *r0 = &image[(2 * y) * IMG_WIDTH];
    const uint8_t *r1 = r0 + IMG_WIDTH;
    uint8_t line[UART_OUT_W];
    for (int x = 0; x < UART_OUT_W; x++) {
        line[x] = (uint8_t)((r0[2 * x] + r0[2 * x + 1] +
                             r1[2 * x] + r1[2 * x + 1] + 2) >> 2);
    }
    for (int x = 0; x < UART_OUT_W; x++) {
        uart_poll_out(uart_dev, line[x]);
    }
    /* Let lower-priority threads (e.g. logger) run between rows */
    k_yield();
}
#else
for (size_t i = 0; i < IMAGE_SIZE; i++)
{
    /* Frames are enqueued at capture start (see frame_item_t): stay behind the
     * capture write pointer, bailing out if capture wedges so we never hang. */
    int64_t wait_deadline = k_uptime_get() + 2000;
    while (lines_done && i >= (size_t)atomic_get(lines_done) * IMG_WIDTH) {
        if (k_uptime_get() > wait_deadline) {
            printk("UART Tx stalled waiting on capture, dropping rest of frame\n");
            goto tail_out;
        }
        k_msleep(1);
    }
   uart_poll_out(uart_dev, image[i]);
   if ((i & 0x7F) == 0x7F) {
       /* Periodically yield to let lower-priority threads (e.g. logger) run */
       k_yield();
   }
}
#endif
tail_out:
for (int i = 0; i < sizeof tail - 1; i++) uart_poll_out(uart_dev, tail[i]);
}


void setup_cam_capture_fifo(struct k_fifo *fifo)
{
    the_new_fifo = fifo;

    k_thread_create(&uart_thread_data, uart_stack, UART_THREAD_STACK_SZ,
                    streaming_photos_fifo, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(3), 0, K_NO_WAIT);
}

void streaming_photos_fifo(void *p1, void *p2, void *p3)
{
    while (1) {
        frame_item_t *item = k_fifo_get(the_new_fifo, K_FOREVER);
        if (item == NULL) {
            continue;
        }
        send_frame_over_uart_binary(item->data, item->lines_done);
        if (item->consumed_sem) {
            k_sem_give(item->consumed_sem);
        }
        /* Always free the FIFO wrapper */
        k_free(item);
    }
}
