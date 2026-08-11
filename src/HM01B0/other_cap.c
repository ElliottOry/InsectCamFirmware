#include "HM01B0_CAPTURE.h"
#include <string.h>

#define LINE_THREAD_STACK_SZ 768

static atomic_t capturing;
static atomic_t line_idx;
static uint8_t *current_dst;

static struct k_fifo *cap_fifo;

static struct k_sem  frame_sem;          /* “frame is ready” semaphore       */
static struct k_sem  line_sem;           /* given by SPIS END callback       */
static struct k_work start_capture_work;

extern void run_peripheral_step( uint16_t seconds);


K_THREAD_STACK_DEFINE(line_stack, LINE_THREAD_STACK_SZ);
static struct k_thread line_thread_data;

__aligned(4)  /* EasyDMA friendly                                             */
static uint8_t image[IMAGE_SIZE];

static struct gpio_callback vsync_cb;

void setup_cam_capture(struct k_fifo *fifo)
{
    cap_fifo = fifo;
    // Initialize the clock output for the camera
    hm_clk_out();
    k_msleep(100);
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
    hm_spi_set_line_sem(&line_sem);
    k_work_init(&start_capture_work, start_capture_fn);

    /* Spawn dedicated line-capture thread */
    k_thread_create(&line_thread_data, line_stack, LINE_THREAD_STACK_SZ,
                    line_thread, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
    
}




static void start_capture_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    k_sem_give(&line_sem);
}


/* Arms SPIS for the next 240-byte DMA reception */
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

    uint8_t *rx_ptr = (IMG_WIDTH > 2) ? (current_dst + 1) : current_dst;
    size_t   rx_len = (IMG_WIDTH > 2) ? (IMG_WIDTH - 2) : IMG_WIDTH;
    hm_spis_arm_rx(rx_ptr, rx_len);
}

static void line_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    
    while (true) {
        k_sem_take(&line_sem, K_FOREVER);     /* unblocked by SPIS END ISR   */

        if (!atomic_get(&capturing)) {
        continue;                         /* ignore stray events         */
    }

    /* Capture one line: wait for VSYNC event or previous DMA completion */
    arm_next_spis_transfer();


    /* Advance line index and write pointer for next line */
    atomic_inc(&line_idx);
    current_dst += IMG_WIDTH;

    /* If we've captured the last line, end frame */
    if (atomic_get(&line_idx) >= IMG_HEIGHT) {
        /* Copy the just-captured frame into a heap buffer and enqueue it */
        //printk("ploopt");
        frame_item_t *item = k_malloc(sizeof(frame_item_t));
        //uint8_t *frame_copy = k_malloc(IMAGE_SIZE);
        if (item) {
            //memcpy(frame_copy, image, IMAGE_SIZE);
            item->data = image;
            item->len = IMAGE_SIZE;
            k_fifo_put(cap_fifo, item);
        }
        atomic_clear(&capturing);
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
    atomic_set(&capturing, 1);
    atomic_set(&line_idx, 0);
    current_dst = image;
    
    
    k_sem_reset(&line_sem);                  /* flush stale grants        */
    /* Wake the line-capture thread so that it can start the very first
            * DMA transaction from thread context. Doing it here (ISR) would
            * violate Zephyr's rules because spi_read_dt() might sleep.        */
    //k_sem_give(&line_sem);
    k_work_submit(&start_capture_work);
    }
}


void stream_photos(void)
{
    while(1)
    {
        hm_i2c_write(REG_MODE_SELECT, 0x03);
        k_sem_take(&frame_sem, K_FOREVER);     /* wait for 1 frame */
        hm_i2c_write(REG_MODE_SELECT, 0x00);
        /* Frame is already enqueued to FIFO by the line thread */
    }

}

void take_single_photo(void)
{
    hm_i2c_write(REG_MODE_SELECT, 0x03);
    k_sem_take(&frame_sem, K_FOREVER);     /* wait for 1 frame */
    hm_i2c_write(REG_MODE_SELECT, 0x00);
    /* Frame is already enqueued to FIFO by the line thread */
}

void turn_off_cam(void)
{
    hm_i2c_write(REG_MODE_SELECT, 0x00); // go to standby mode
    hm_clk_enable(false); // disable the clock output
    atomic_clear(&capturing);
    k_sem_reset(&frame_sem);
    k_sem_reset(&line_sem);
    k_work_cancel_delayable(&start_capture_work);
    k_thread_abort(&line_thread_data);
    gpio_remove_callback(gpio0_dev, &vsync_cb);
}

void cam_standby(void)
{
    gpio_pin_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_DISABLE);
    hm_i2c_write(REG_MODE_SELECT, 0x00); // go to standby mode
}
