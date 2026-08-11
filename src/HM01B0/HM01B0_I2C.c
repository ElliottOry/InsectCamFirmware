#include "HM01B0_I2C.h"

void init_cam(void)
{


    uint8_t id_high = 0, id_low = 0;
    uint8_t reg_addr_high[2] = {0x00, 0x00};   // MODEL_ID_H address
    uint8_t reg_addr_low[2]  = {0x00, 0x01};   // MODEL_ID_L address
    /*
    // Read MODEL_ID_H
    int ret1 = i2c_write_read_dt(&dev_i2c, reg_addr_high, 2, &id_high, 1);
    // Read MODEL_ID_L
    int ret2 = i2c_write_read_dt(&dev_i2c, reg_addr_low, 2, &id_low, 1);
    if (ret1 != 0 || ret2 != 0) {
        printk("Failed to read chip ID! 0x%02x%02x\n", id_high, id_low);
        return;
    }
    printk("HM01B0 ID: 0x%02x%02x\n", id_high, id_low);
    */
    
    hm_i2c_write( REG_MODE_SELECT, 0x00);//go to stand by mode
    //hm_i2c_write( REG_ANA_REGISTER_17, 0x00);//register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm_i2c_write( REG_TEST_PATTERN_MODE, TEST_PATTERN_OFF);//Test pattern off, output real sensor image
    //hm_i2c_write(0x0101 , 0x03);
    
    hm_i2c_write( REG_BIN_MODE, 0x03);//VERTICAL BIN MODE
    hm_i2c_write( REG_QVGA_WIN_EN, 0x01);//Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
//    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);

    //100*100 optimization
    hm_i2c_write( REG_BIN_RDOUT_X, 0x03);//Horizontal Binning enable
    hm_i2c_write( REG_BIN_RDOUT_Y, 0x03);//vertical Binning enable => this register should be always 0x03 because we never go more than 160 for the height
    // Frame timing control at the QQVGA minimum (datasheet Table 8.2): line_length
    // 0x00D7 (215) x frame_length 0x0080 (128).  vt_pix_clk = MCLK/8 = 500 kHz, so
    // frame time = 128 x 215 / 500 kHz = 55 ms -> ~18.2 fps, the sensor-side ceiling.
    // AE MAX_INTG below is capped to frame_length - 2 so frames never stretch.
    hm_i2c_write(0x0342, 0x00);   // LINE_LENGTH_PCK_H
    hm_i2c_write(0x0343, 0xD7);   // LINE_LENGTH_PCK_L -> 0x00D7 (215)
    hm_i2c_write(0x0340, 0x00);   // FRAME_LENGTH_LINES_H
    hm_i2c_write(0x0341, 0x80);   // FRAME_LENGTH_LINES_L -> 0x0080 (128)

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);
    hm_i2c_write(0x0104 ,0x01);
    
    hm_i2c_write(0x3044,0x0A);
    hm_i2c_write(0x3045,0x00);
    hm_i2c_write(0x3047,0x0A);
    hm_i2c_write(0x3050,0xC0);
    hm_i2c_write(0x3051,0x42);
//    hm_i2c_write(0x3052,0x50);
    hm_i2c_write(0x3053,0x00);
    hm_i2c_write(0x3054,0x03);
    hm_i2c_write(0x3055,0xF7);
    hm_i2c_write(0x3056,0xF8);
    hm_i2c_write(0x3057,0x29);
    hm_i2c_write(0x3058,0x1F);
//    hm_i2c_write(0x3059,0x1E);//bit control
    hm_i2c_write(0x3064,0x00);
    hm_i2c_write(0x3065,0x04);

    //black level control
    hm_i2c_write(0x1000,0x43);
    hm_i2c_write(0x1001,0x40);
    hm_i2c_write(0x1002,0x32);
    hm_i2c_write(0x1003,0x14);   // BLC_TGT (was 0x08 — raised to reduce dark halos)
    hm_i2c_write(0x1006,0x01);
    hm_i2c_write(0x1007,0x14);   // BLC2_TGT must match BLC_TGT

    hm_i2c_write(0x0350,0x7F);
    

    //Sensor reserved
    /* DPC (defect pixel correction) OFF.  With DPC on, single pixels on the shoulder
     * of a blown-out highlight get flagged as defects and substituted, which draws a
     * 1-pixel black rim around any bright spot aimed at the sensor: a defect detector
     * cannot tell a genuinely steep edge from a bad pixel, and the saturation shoulder
     * is the steepest edge in the frame.  Measured on a lamp shone into the lens —
     * isolated pixels reading ~0 wedged between a 211-224 shoulder and the 255 plateau,
     * recurring at the same column for as many rows as the highlight edge stays
     * vertical.  The DEVEIL stretch in save_to_file_bt.py then clips that dip to solid
     * black, which is what makes it so obvious on screen.
     * 0x100B/0x100C are the hot/cold thresholds and only apply while DPC is on. */
    hm_i2c_write(0x1008,0x00);   // DPC_CTRL off (was 0x01 = option1-mono)
    hm_i2c_write(0x1009,0xA0);
    hm_i2c_write(0x100A,0x60);
    hm_i2c_write(0x100B,0x90);//default from lattice 0x90
    hm_i2c_write(0x100C,0x40);//default from lattice 0x40;

    //Vsync, hsync and pixel shift register
//    hm_i2c_write(0x1012,0x07);//changed by Ali
    hm_i2c_write(0x1012,0x00);//lattice value

    //Statistic control and read only
    hm_i2c_write(0x2000,0x07);
    // Full-frame AE metering (reset defaults). An outdoor day/night camera has no
    // fixed subject, so meter the whole frame instead of a biased sub-window — a
    // biased window made AE expose for a dark ROI and blow out the rest.
    // (Previous custom AE/MD ROI writes 0x2003-0x2018 removed.)

    // Auto exposure for an outdoor day/night camera. The AE loop converges across
    // many frames via the warm-up window in stream_photos() (HM01B0_CAPTURE.c).
    hm_i2c_write(0x2100, 0x00);   // AE off while updating

    hm_i2c_write(0x2101, 0x3C);   // AE_TARGET_MEAN: balanced mid-gray, protects daylight highlights
    // MAX_INTG must respect the datasheet constraint (8.4.B): integration <=
    // frame_length_lines - 2 = 126 (0x7E).  The previous 0xFE (254) let AE stretch the
    // frame to ~2x in dim light, halving fps to ~6; night exposure now comes from the
    // higher analog gain ceiling below instead, keeping a constant ~12 fps cadence.
    hm_i2c_write(0x2105, 0x00);   // MAX_INTG_H
    hm_i2c_write(0x2106, 0x7E);   // MAX_INTG_L = frame_length (128) - 2

    hm_i2c_write(0x2108, 0x03);   // MAX_AGAIN_FULL  = 8x (recovers the stop of night
    hm_i2c_write(0x2109, 0x03);   // MAX_AGAIN_BIN2  = 8x  sensitivity lost by capping
                                  // integration; noisier night frames, steady fps)
    hm_i2c_write(0x210A, 0x00);   // MIN_AGAIN       = 1× (daylight floor)
    hm_i2c_write(0x210B, 0x80);   // MAX_DGAIN       = 2× (8x×2x = 16x, datasheet 8.3 machine-version limit)
    hm_i2c_write(0x210C, 0x40);   // MIN_DGAIN       = 1×
    hm_i2c_write(0x2107, 0x02);   // MIN_INTG        = 2 lines (~0.43 ms, bright daylight floor)
    hm_i2c_write(0x210D, 0x20);   // DAMPING: faster convergence within the warm-up window
    hm_i2c_write(0x210E, 0x03);   // Anti-flicker enabled, 60 Hz

    /* The flicker step MUST match the actual pixel clock -- it is the quantum AE
     * rounds integration to, so a wrong value becomes AE's exposure floor.
     *   vt_pix_clk = MCLK / vt_sys_div = 4 MHz / 8 = 500 kHz
     *                (MCLK from HM01B0_CLK.c; vt_sys_div = 0x3060[1:0] = 00 -> /8)
     * Datasheet 8.4.1:
     *   step(60 Hz) = vt_pix_clk / line_length_pck / 120 = 500000/215/120 = 19 lines
     *   step(50 Hz) = vt_pix_clk / line_length_pck / 100 = 500000/215/100 = 23 lines
     * These were previously left at their reset defaults (60 and 50 lines), which
     * quantised integration to multiples of 60 -- an exposure floor of 25.8 ms
     * instead of 8.3 ms, i.e. 1.7 stops AE could not claw back.  Measured effect:
     * frame mean pinned at 126 against AE_TARGET_MEAN 0x3C (60). */
    hm_i2c_write(0x210F, 0x00);   // FS_60HZ_H
    hm_i2c_write(0x2110, 0x13);   // FS_60HZ_L = 19 lines
    hm_i2c_write(0x2111, 0x00);   // FS_50HZ_H
    hm_i2c_write(0x2112, 0x17);   // FS_50HZ_L = 23 lines

    hm_i2c_write(0x0202, 0x00);   // Seed exposure: start convergence from a non-blown point
    hm_i2c_write(0x0203, 0x20);   // INTEGRATION = 0x0020 (~6.9 ms)
    hm_i2c_write(0x0205, 0x00);   // Analog gain 1×
    hm_i2c_write(0x020E, 0x01);   // Digital 1×
    hm_i2c_write(0x020F, 0x00);

    hm_i2c_write(0x2100, 0x01);   // AE on

    // Keep motion detection control as before
    hm_i2c_write(0x2150, 0x03);

    // AE owns exposure/gain from here. Do not write manual integration after
    // enabling AE; values above the frame length are invalid per the datasheet.
    //frame timing control
//    hm_i2c_write(0x0340,0x02);//changed by Ali
//    hm_i2c_write(0x0341,0x32);//changed by Ali    
////    hm_i2c_write(0x0340,0x0C);
////    hm_i2c_write(0x0341,0x5C);
//
//    hm_i2c_write(0x0342,0x01);
//    hm_i2c_write(0x0343,0x78);//changed by Ali
//    hm_i2c_write(0x0343,0x78);

//    hm_i2c_write(0x3010,0x01); //done in lower lines
//    hm_i2c_write(0x0383,0x00); //done in lower lines
//    hm_i2c_write(0x0387,0x00); //done in lower lines
//    hm_i2c_write(0x0390,0x00); //done in lower lines
//    hm_i2c_write(0x3059,0x42); //done in lower lines
//    hm_i2c_write(0x3060,0x51); //done in lower lines
        //hm_i2c_write( REG_OSC_CLK_DIV, 0x30);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

        hm_i2c_write( REG_BIT_CONTROL, 0x20);//8-bit grayscale (1 byte per pixel)
        // IO drive strength left at default: scope shows D0 already reaches full swing,
        // so signal integrity isn't the limiter (max drive 0xFF only risked ringing).
        //hm_i2c_write(0x0202, 0x00);
        //hm_i2c_write(0x0203, 0x08);
        //hm_i2c_write(0x2101, 0x06);
        //hm_i2c_write(0x2105, 0x01);
        /* 0x3060 = 0x30: gated-clock mode + dividers, dvdd_ldo_en_d (bit 6)
         * left 0.  DVDD on this board is a bypass cap only (internal-LDO
         * arrangement) and the sensor demonstrably runs with this value, so
         * keep the proven configuration — the datasheet's bit-6 description
         * does not match observed behavior and is not to be trusted blindly. */
        hm_i2c_write(0x3060, 0x30);
        hm_i2c_write(0x1012, 0x03);
        //hm_i2c_write(0x3023, 0x05);
        //hm_i2c_write(0x0101 , 0x02);

        //hm_i2c_write( REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01);//set the number of frames to be sent out, it sends N frames
        //hm_clk_enable(true);
}

void hm_i2c_write(uint16_t reg, uint8_t val)
{
uint8_t buf[] = { reg >> 8, reg & 0xff, val };
i2c_write_dt(&dev_i2c, buf, sizeof buf);
}

int hm_cam_probe(void)
{
    uint8_t id = 0;
    uint8_t reg[2] = { 0x00, 0x00 };   /* MODEL_ID_H, reads 0x01 on a live HM01B0 */

    for (int attempt = 0; attempt < 3; attempt++) {
        /* Recover BEFORE the first transaction, not after a failure: on a wedged
         * bus (sensor holding SDA low) the TWIM transfer never completes and
         * i2c_write_read_dt blocks the calling thread FOREVER — recovery placed
         * after it would never run.  On a healthy bus this is a no-op. */
        int rec = i2c_recover_bus(dev_i2c.bus);
        if (rec != 0) {
            printk("I2C bus recovery returned %d\n", rec);
        }
        k_msleep(5);

        int ret = i2c_write_read_dt(&dev_i2c, reg, 2, &id, 1);
        if (ret == 0 && id == 0x01) {
            if (attempt > 0) {
                printk("HM01B0 responding after I2C bus recovery\n");
            }
            return 0;
        }
        printk("HM01B0 probe failed (ret=%d id=0x%02X); retrying\n", ret, id);
    }
    return -EIO;
}
