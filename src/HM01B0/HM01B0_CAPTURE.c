#include "HM01B0_CAPTURE.h"
#include <string.h>
#include <hal/nrf_power.h>

/* Connection gate (ble_manager.c) pausing the pipeline between centrals. */
extern bool ble_is_connected(void);
extern void ble_wait_for_connection(void);

#define LINE_THREAD_STACK_SZ 768

/* Measured 2026-07-21: parking with MCLK running averaged HIGHER (~3.0+ mA)
 * than parking with MCLK stopped (~2.7-2.8 mA) -- the sensor does not return
 * to its ~0.15 mA spec standby even clocked, so the nRF's clock machinery
 * (~1.3 mA) is pure loss.  Keep 0.  The sensor's ~2.4 mA residual draw at
 * this board's 3.3 V rails (0.3 V above recommended max) is state-independent
 * and not reachable from firmware. */
#define PARK_KEEP_MCLK 0

/* AE convergence, paid once per BLE connection (cam_stream_resume) — not per
 * frame.  At MCLK 2.67 MHz, vt_pix_clk = MCLK/8 ≈
 * 333 kHz, so frame time = 128 x 215 / 333kHz ≈ 82.6 ms (~12 fps sensor cadence —
 * AE integration is capped at frame_length-2 in HM01B0_I2C.c so frames never
 * stretch); ~2200 ms is ~26 frames, enough for AE to settle across the day/night
 * range.  (Scale with MCLK if you change the divider in HM01B0_CLK.c.)
 */
#define AE_CONVERGE_MS 2200


static atomic_t capturing;
static atomic_t line_idx;
static uint8_t *current_dst;

/* Arms capture for exactly ONE frame.  The VSYNC ISR starts a frame only when this
 * is set; the line thread clears it once the frame completes and is enqueued.  While
 * it is clear the sensor keeps streaming (so AE stays converged) but VSYNC frames are
 * ignored, leaving the target buffer untouched while consumers drain it.
 */
static atomic_t capture_armed;

/* Multi-buffering: capture frame N+1 into one buffer while consumers drain frame N
 * from the other, so capture and transmit overlap instead of running back-to-back.
 * write_idx is the buffer the producer asks for next; active_idx is latched by the
 * VSYNC ISR for the frame currently being captured.
 *
 * NUM_BUFS = 1 -> single 8-bit buffer (no overlap).  Two full 19 KB frames don't fit
 * in the nRF52832's 64 KB RAM alongside the BLE stack, so this stays at 1.
 */
#define NUM_BUFS 1
static atomic_t write_idx;
static int      active_idx;

static struct k_fifo *cap_fifo1;
static struct k_fifo *cap_fifo2;
/* Per-buffer "consumers finished, buffer free" counter (one give per consumer). */
static struct k_sem  buf_done_sem[NUM_BUFS];
static int consumers_expected;

static struct k_sem  frame_sem;          /* “frame is ready” semaphore       */
static struct k_sem  line_sem;           /* given by SPIS END callback       */

/* Set once the in-progress frame has been handed to the consumer FIFOs.  Frames are
 * enqueued at frame START (first line) so BLE/UART transmit overlaps the capture —
 * consumers pace themselves on line_idx via item->lines_done and never read past the
 * capture write pointer.  Cleared when the frame completes.
 */
static bool frame_handed_off;

extern void run_peripheral_step( uint16_t seconds);

//extern void run_peripheral_step( uint16_t seconds, uint8_t *image);


extern struct spi_dt_spec spis_dev;


K_THREAD_STACK_DEFINE(line_stack, LINE_THREAD_STACK_SZ);
static struct k_thread line_thread_data;

__aligned(4)  /* EasyDMA friendly                                             */
static uint8_t image_buf[NUM_BUFS][IMAGE_SIZE];   /* 8-bit grayscale frames */

static struct gpio_callback vsync_cb;

void setup_cam_capture(struct k_fifo *fifo1, struct k_fifo *fifo2)
{
    cap_fifo1 = fifo1;
    cap_fifo2 = fifo2;
    consumers_expected = 0;
    if (cap_fifo1) consumers_expected++;
    if (cap_fifo2) consumers_expected++;
    /* Each buffer starts free with full credits, so the first use of either buffer
     * doesn't block.  Consumers give one credit per frame; the producer takes
     * consumers_expected credits before reusing a buffer. */
    {
        int max_credits = consumers_expected ? consumers_expected : 1;
        for (int b = 0; b < NUM_BUFS; b++) {
            k_sem_init(&buf_done_sem[b], max_credits, max_credits);
        }
    }
    // Initialize the clock output for the camera
    hm_clk_out();
    k_msleep(100);
    /* Make sure the sensor actually answers before configuring it: a reset that
     * interrupted an I2C transaction leaves the HM01B0 holding SDA low, and then
     * every init_cam() write fails silently -> no VSYNC -> capture stalls forever. */
    if (hm_cam_probe() != 0) {
        printk("HM01B0 not responding on I2C even after bus recovery -- "
               "power-cycle the board if this persists\n");
    }
    // Initialize the camera
    init_cam();
    // Initialize the SPI interface for camera data transfer
    hm_spi_init();


    gpio_pin_configure(gpio0_dev, PIN_VSYNC, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_init_callback(&vsync_cb, vsync_isr, BIT(PIN_VSYNC));
    gpio_add_callback(gpio0_dev, &vsync_cb);
    gpio_pin_interrupt_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_EDGE_RISING);
    // Start the camera capture process
    //arm_next_spis_transfer();

    k_sem_init(&frame_sem, 0, 1);
    k_sem_init(&line_sem,  0, 1);

    /* Spawn dedicated line-capture thread */
    k_thread_create(&line_thread_data, line_stack, LINE_THREAD_STACK_SZ,
                    line_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    
}




/* Arms SPIS to receive one 8-bit line directly into the frame buffer. */
static void arm_next_spis_transfer(void)
{
    /* Skip the first and last byte of each SPIS transfer by:
     * - clearing the first and last positions in the destination line buffer
     * - programming the RX DMA to write only the middle bytes
     */
    if (IMG_WIDTH >= 2) {
        current_dst[0] = 0x00;
        current_dst[IMG_WIDTH - 1] = 0x00;
    }

    struct spi_buf rx = {
        .buf = (IMG_WIDTH > 2) ? (current_dst + 1) : current_dst,
        .len = (IMG_WIDTH > 2) ? (IMG_WIDTH - 2) : IMG_WIDTH
    };
    struct spi_buf_set rxset = { .buffers = &rx, .count = 1 };
    /* Use Zephyr SPI API properly: device first, then config, then TX/RX */
    spi_transceive(spis_dev.bus, &spis_dev.config, NULL, &rxset);

    k_sem_give(&line_sem);
}

/* Hands the frame being captured into image_buf[active_idx] to every consumer FIFO.
 * Called at frame START so consumers transmit while lines are still arriving; each
 * item carries &line_idx so the consumer can pace on capture progress.  The producer
 * takes consumers_expected credits before reusing the buffer and each consumer
 * returns one when done — if a malloc fails, return that consumer's credit right away
 * so the producer never blocks on a consumer that never got the frame (drops a frame
 * instead of stalling the whole stream). */
static void enqueue_frame_to_consumers(void)
{
    uint8_t *frame = image_buf[active_idx];
    struct k_sem *done = &buf_done_sem[active_idx];
    struct k_fifo *fifos[2] = { cap_fifo1, cap_fifo2 };

    for (int i = 0; i < 2; i++) {
        if (!fifos[i]) {
            continue;
        }
        frame_item_t *item = k_malloc(sizeof(frame_item_t));
        if (item) {
            item->data = frame;
            item->len = IMAGE_SIZE;
            item->consumed_sem = done;
            item->lines_done = &line_idx;
            k_fifo_put(fifos[i], item);
        } else {
            k_sem_give(done);
        }
    }
    frame_handed_off = true;
}

static void line_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (true) {
        k_sem_take(&line_sem, K_FOREVER);     /* unblocked by SPIS END ISR   */

        if (!atomic_get(&capturing)) {
        continue;                         /* ignore stray events         */
    }

    /* New frame just armed (VSYNC reset line_idx): hand the buffer to consumers
     * BEFORE the first line lands so transmit overlaps capture. */
    if (atomic_get(&line_idx) == 0 && !frame_handed_off) {
        enqueue_frame_to_consumers();
    }

    /* Capture one 8-bit line directly into the frame buffer */
    arm_next_spis_transfer();

    /* Advance line index and write pointer for next line */
    atomic_inc(&line_idx);
    current_dst += IMG_WIDTH;

    /* If we've captured the last line, end frame */
    if (atomic_get(&line_idx) >= IMG_HEIGHT) {
        /* Consumers already hold the frame (enqueued at frame start) and see
         * line_idx == IMG_HEIGHT, i.e. the whole buffer is valid.  They give the
         * buffer credit back when done; the producer waits on that before rearming,
         * so line_idx cannot be reset out from under a draining consumer. */
        frame_handed_off = false;
        atomic_clear(&capturing);
        atomic_clear(&capture_armed);   /* one frame per arm; producer re-arms for next buffer */
        k_sem_give(&frame_sem);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* VSYNC ISR – starts a new frame                                             */
/* -------------------------------------------------------------------------- */


static void vsync_isr(const struct device *dev,
    struct gpio_callback *cb,
    uint32_t pins)
    {
    if (gpio_pin_get(dev, PIN_VSYNC)) {          /* rising edge = frame start */
    if (!atomic_get(&capture_armed)) {
        return;                              /* disarmed: ignore frame, keep buffers stable */
    }
    active_idx = atomic_get(&write_idx);     /* latch which buffer this frame fills */
    atomic_set(&capturing, 1);
    atomic_set(&line_idx, 0);
    current_dst = image_buf[active_idx];


    k_sem_reset(&line_sem);                  /* flush stale grants        */
    k_sem_give(&line_sem);                   /* wake line thread directly  */
    }
}


/* Park the capture front end while no BLE central is connected: sensor to
 * standby (I2C first, while MCLK still runs — the sensor's register block is
 * clocked from MCLK), then MCLK timer off, then the SoC back to low-power idle
 * (hm_spi_init switched it to constant latency for SPIS, which keeps HFCLK — and
 * milliamps — burning even in sleep).  The sensor retains its register file with
 * MCLK gated, so resume needs no re-init; if it ever comes back wedged, the
 * stall watchdog in stream_photos() recovers with a full re-init anyway. */
static void cam_stream_pause(void)
{
    atomic_clear(&capture_armed);
    atomic_clear(&capturing);
    hm_i2c_write(REG_MODE_SELECT, MODE_STAND_BY);
    /* PARK_KEEP_MCLK experiment: every no-MCLK state measured ~2.5 mA at the
     * supply while the boot phase (MCLK running, sensor in standby) fit under
     * ~1 mA of sensor draw -- suggesting the HM01B0 cannot hold a clean
     * standby without its clock (its register/housekeeping logic is clocked
     * from MCLK).  Keeping MCLK running while parked costs the nRF the timer/
     * GPIOTE/HFCLK machinery (~1.2-1.5 mA) but may drop the sensor from
     * ~2.4 mA to its spec standby (~0.15 mA).  Measure both builds and keep
     * the cheaper one.  (0x3061 LDO power-down was tried: no effect.) */
#if !PARK_KEEP_MCLK
    hm_clk_enable(false);
#endif
    /* Every GPIOTE channel left in Task/Event mode holds the 16 MHz clock on
     * through sleep (~1 mA each way on the nRF52832), so release them all:
     * the VSYNC edge interrupt's channel, then the scope/SPIS-gate channels. */
    gpio_pin_interrupt_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_DISABLE);
    hm_spis_gpiote_park();
    /* LVLD idles LOW (the sensor drives it push-pull; it only goes high during
     * a pixel line), so the internal pull-up gate_trigger_init put on it burns
     * ~0.25 mA (3.3 V / 13 k) the whole time the camera is parked.  The line
     * stays driven by the powered sensor, so no pull is needed while parked. */
    NRF_GPIO->PIN_CNF[PIN_GATE] =
        (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
        (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
        (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos);
    nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_LOWPWR);
}

static void cam_stream_resume(void)
{
    nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
    /* Restore gate_trigger_init's exact pin config before re-enabling gating. */
    NRF_GPIO->PIN_CNF[PIN_GATE] =
        (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
        (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
        (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos);
    hm_spis_gpiote_unpark();
    gpio_pin_interrupt_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_EDGE_RISING);
    hm_clk_enable(true);
    k_msleep(5);                               /* MCLK stable before I2C */
    /* MODE_STREAMING (0x01) = Streaming 1 = CONTINUOUS until standby.  Do NOT use 0x03
     * here: that is Streaming 2, which outputs a fixed number of frames and then
     * auto-enters standby (clearing the mode bits) — the sensor would stop after N
     * frames and spi_transceive would block forever waiting for lines. */
    hm_i2c_write(REG_MODE_SELECT, MODE_STREAMING);
    k_msleep(AE_CONVERGE_MS);                  /* AE convergence, once per connection */
}

void stream_photos(void)
{
    /* While connected the sensor streams continuously (AE converges once per
     * connection, not per frame); no frames land in a buffer until armed. */
    atomic_set(&capture_armed, 0);

    /* setup_cam_capture()/hm_spi_init() left MCLK and constant latency on, so
     * start "running": the first disconnected iteration then actually parks the
     * front end instead of assuming it is already parked. */
    bool front_end_running = true;

    int w = 0;
    uint32_t frame_count = 0;
    int consecutive_stalls = 0;
    while(1)
    {
        /* Battery gate: with no central connected nothing consumes frames, so
         * park everything and sleep until the next connect — this thread, the
         * line thread and SPIS all go quiet and the CPU can actually idle. */
        if (!ble_is_connected()) {
            if (front_end_running) {
                cam_stream_pause();
                front_end_running = false;
                printk("No BLE central: camera parked, CPU idling until connect\n");
            }
            ble_wait_for_connection();
            continue;
        }
        if (!front_end_running) {
            printk("BLE central connected: restarting camera\n");
            cam_stream_resume();
            front_end_running = true;
        }

        /* Wait until this buffer's previous consumers are done (it's free).  Use a
         * watchdog timeout instead of K_FOREVER so a lost/late consumer credit can
         * never hang the stream — it logs and proceeds (worst case a torn frame). */
        for (int i = 0; i < consumers_expected; i++) {
            if (k_sem_take(&buf_done_sem[w], K_MSEC(3000)) != 0) {
                printk("STALL: buffer %d not released by consumer in 3s (BLE side); recovering\n", w);
                break;
            }
        }

        atomic_set(&write_idx, w);
        k_sem_reset(&frame_sem);
        atomic_set(&capture_armed, 1);         /* capture exactly the next frame into buffer w */

        /* Watchdog the capture: a frame is ~83 ms, so ~500 ms means the capture/SPIS
         * path is stuck (e.g. a line transfer that never completed). Keep this short —
         * every stall costs this long before we re-kick, so a long timeout tanks fps.
         * Log, disarm, re-kick, retry rather than blocking. */
        if (k_sem_take(&frame_sem, K_MSEC(500)) != 0) {
            printk("STALL: capture #%u did not complete in 500ms; re-kicking stream\n", frame_count);
            atomic_clear(&capture_armed);
            /* Escalate after repeated stalls: one MODE_SELECT rewrite can't help if
             * the I2C bus is wedged (sensor holding SDA after an interrupted
             * transaction) or the sensor lost its whole register config.  Recover
             * the bus and redo the full sensor init, then resume streaming. */
            if (++consecutive_stalls >= 4) {
                printk("Capture stalled %d times; recovering I2C + full sensor re-init\n",
                       consecutive_stalls);
                consecutive_stalls = 0;
                if (hm_cam_probe() != 0) {
                    printk("HM01B0 still not responding -- power-cycle the board\n");
                }
                init_cam();
            }
            /* Safety net: if the sensor ever dropped out of streaming, restart it.
             * Resuming line output also lets a stuck spi_transceive complete. */
            hm_i2c_write(REG_MODE_SELECT, MODE_STREAMING);
            w = (w + 1) % NUM_BUFS;
            continue;
        }

        consecutive_stalls = 0;

        if ((++frame_count % 10) == 0) {
            printk("cap %u\n", frame_count);   /* heartbeat: capture is alive */
        }

        /* Frame in buffer w is now enqueued; consumers drain it while we immediately
         * loop to capture the next buffer — capture and transmit overlap (NUM_BUFS>1).
         * With NUM_BUFS==1 this stays on buffer 0, so the producer waits for consumers
         * via buf_done_sem[0] before reusing it (single-buffer behavior). */
        w = (w + 1) % NUM_BUFS;
    }

}

void take_single_photo(void)
{
    /* One-shot: converge AE (sensor may be cold), capture one frame, then standby.
     * Use continuous streaming (0x01) during convergence; 0x03 (Streaming 2) would
     * auto-standby after N frames mid-convergence. */
    atomic_set(&capture_armed, 0);
    atomic_set(&write_idx, 0);             /* one-shot always uses buffer 0 */
    hm_i2c_write(REG_MODE_SELECT, MODE_STREAMING);
    k_msleep(AE_CONVERGE_MS);              /* let AE settle */
    k_sem_reset(&frame_sem);
    atomic_set(&capture_armed, 1);         /* capture + enqueue exactly the next frame */
    k_sem_take(&frame_sem, K_FOREVER);     /* line thread disarms once it completes */
    hm_i2c_write(REG_MODE_SELECT, 0x00);
    /* Frame is already enqueued to FIFO by the line thread */
}

void turn_off_cam(void)
{
    hm_i2c_write(REG_MODE_SELECT, 0x00); // go to standby mode
    hm_clk_enable(false); // disable the clock output
    atomic_clear(&capturing);
    atomic_clear(&capture_armed);
    k_sem_reset(&frame_sem);
    k_sem_reset(&line_sem);
    k_thread_abort(&line_thread_data);
    gpio_remove_callback(gpio0_dev, &vsync_cb);
}

void cam_standby(void)
{
    gpio_pin_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_DISABLE);
    hm_i2c_write(REG_MODE_SELECT, 0x00); // go to standby mode
}
