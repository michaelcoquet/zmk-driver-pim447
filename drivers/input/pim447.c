/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 *
 * Zephyr input driver for the Pimoroni PIM447 Trackball Breakout.
 *
 * The PIM447 has an onboard Nuvoton MCU that reads 4 Hall-effect sensors
 * and a dome switch, then exposes the data over I2C at address 0x0A:
 *
 *   Register  Description
 *   --------  -----------
 *   0x00      LED Red   (0-255, write)
 *   0x01      LED Green (0-255, write)
 *   0x02      LED Blue  (0-255, write)
 *   0x03      LED White (0-255, write)
 *   0x04      Left movement delta (read, clears on read)
 *   0x05      Right movement delta (read, clears on read)
 *   0x06      Up movement delta (read, clears on read)
 *   0x07      Down movement delta (read, clears on read)
 *   0x08      Switch state (bit 7 = pressed, bits 0-6 unused; clears on read)
 *
 * Reading 5 bytes starting from register 0x04 returns [left, right, up, down, switch]
 * atomically in a single I2C transaction.
 *
 * This driver polls at a configurable interval, computes net X/Y deltas,
 * optionally applies a configurable acceleration curve, and reports the
 * resulting movement as INPUT_REL_X / INPUT_REL_Y / INPUT_BTN_0 events
 * via the Zephyr input subsystem. ZMK picks these up through zmk,input-listener.
 *
 * Acceleration model (per-axis, integer math):
 *
 *   if |delta| <= accel-threshold:
 *       output = delta
 *   else:
 *       over    = |delta| - accel-threshold
 *       output  = delta * (ACCEL_BASE + over * accel-factor) / ACCEL_BASE
 *
 * With ACCEL_BASE = 16, the formula gives 1:1 below the threshold and
 * progressively boosts faster movements. With accel-factor = 4 and
 * accel-threshold = 1:
 *
 *   delta    1  2  3   4   5   6   7   8
 *   output   1  2  4   7  10  13  16  22
 *
 * Larger accel-factor values give more aggressive acceleration. Set
 * accel-factor = 0 to disable acceleration (pure linear).
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pim447, CONFIG_PIM447_LOG_LEVEL);

/* PIM447 I2C register addresses */
#define PIM447_REG_LED_RED   0x00
#define PIM447_REG_LED_GRN   0x01
#define PIM447_REG_LED_BLU   0x02
#define PIM447_REG_LED_WHT   0x03
#define PIM447_REG_LEFT      0x04
#define PIM447_REG_RIGHT     0x05
#define PIM447_REG_UP        0x06
#define PIM447_REG_DOWN      0x07
#define PIM447_REG_SWITCH    0x08

#define PIM447_MSK_SWITCH_STATE 0x80

/* Acceleration fixed-point base. Acceleration curve formula uses this as
 * the denominator. Higher values give finer control granularity in the
 * accel-factor devicetree property. */
#define ACCEL_BASE 16

/* Packed movement + switch data (5 bytes from reg 0x04) */
struct pim447_motion_data {
    uint8_t left;
    uint8_t right;
    uint8_t up;
    uint8_t down;
    uint8_t sw;
};

struct pim447_config {
    struct i2c_dt_spec i2c;
    uint32_t poll_interval_ms;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
    uint16_t accel_factor;
    uint16_t accel_threshold;
};

struct pim447_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
    bool prev_btn_state;
};

/**
 * Apply per-axis acceleration curve. Returns input unchanged when
 * acceleration is disabled (accel_factor == 0) or when |delta| is at or
 * below the threshold.
 */
static int16_t apply_acceleration(int16_t delta, const struct pim447_config *cfg)
{
    if (delta == 0 || cfg->accel_factor == 0) {
        return delta;
    }

    int32_t abs_d = (delta < 0) ? -(int32_t)delta : (int32_t)delta;
    if (abs_d <= (int32_t)cfg->accel_threshold) {
        return delta;
    }

    int32_t over = abs_d - (int32_t)cfg->accel_threshold;
    int32_t scaled = ((int32_t)delta * (ACCEL_BASE + over * (int32_t)cfg->accel_factor))
                     / ACCEL_BASE;

    /* Clamp to int16_t range */
    if (scaled > INT16_MAX) {
        scaled = INT16_MAX;
    } else if (scaled < INT16_MIN) {
        scaled = INT16_MIN;
    }

    return (int16_t)scaled;
}

/**
 * Read motion and switch data from the PIM447 in a single atomic
 * 5-byte burst read starting at register 0x04. The PIM447's internal
 * counters all clear on read, so reading them together guarantees the
 * four directional values were sampled at the same instant.
 */
static int pim447_read_motion(const struct device *dev, struct pim447_motion_data *motion)
{
    const struct pim447_config *cfg = dev->config;
    uint8_t reg = PIM447_REG_LEFT;
    uint8_t buf[5];
    int ret;

    ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read motion data: %d", ret);
        return ret;
    }

    motion->left  = buf[0];
    motion->right = buf[1];
    motion->up    = buf[2];
    motion->down  = buf[3];
    motion->sw    = buf[4];

    return 0;
}

/**
 * Set the RGBW LED color.
 * Writes 4 bytes to registers 0x00-0x03.
 */
int pim447_set_led(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    const struct pim447_config *cfg = dev->config;
    uint8_t buf[5] = { PIM447_REG_LED_RED, r, g, b, w };

    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/**
 * Periodic poll work handler.
 * Reads motion data, computes net deltas, applies acceleration,
 * and reports input events.
 */
static void pim447_poll_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pim447_data *data = CONTAINER_OF(dwork, struct pim447_data, poll_work);
    const struct device *dev = data->dev;
    const struct pim447_config *cfg = dev->config;
    struct pim447_motion_data motion;
    int ret;

    ret = pim447_read_motion(dev, &motion);
    if (ret < 0) {
        goto reschedule;
    }

    /* Compute signed deltas from the unsigned left/right/up/down counters */
    int16_t dx = (int16_t)motion.right - (int16_t)motion.left;
    int16_t dy = (int16_t)motion.down  - (int16_t)motion.up;

    /* Apply per-axis acceleration curve */
    dx = apply_acceleration(dx, cfg);
    dy = apply_acceleration(dy, cfg);

    /* Apply axis transformations */
    if (cfg->swap_xy) {
        int16_t tmp = dx;
        dx = dy;
        dy = tmp;
    }
    if (cfg->invert_x) {
        dx = -dx;
    }
    if (cfg->invert_y) {
        dy = -dy;
    }

    /* Report movement if non-zero */
    if (dx != 0 || dy != 0) {
        if (dx != 0) {
            input_report_rel(dev, INPUT_REL_X, dx, dy == 0, K_FOREVER);
        }
        if (dy != 0) {
            input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
        }
        LOG_DBG("dx=%d dy=%d", dx, dy);
    }

    /* Report button state changes */
    bool btn_pressed = (motion.sw & PIM447_MSK_SWITCH_STATE) != 0;
    if (btn_pressed != data->prev_btn_state) {
        input_report_key(dev, INPUT_BTN_0, btn_pressed ? 1 : 0, true, K_FOREVER);
        data->prev_btn_state = btn_pressed;
        LOG_DBG("btn=%d", btn_pressed);
    }

reschedule:
    k_work_reschedule(dwork, K_MSEC(cfg->poll_interval_ms));
}

/**
 * Device initialization.
 */
static int pim447_init(const struct device *dev)
{
    const struct pim447_config *cfg = dev->config;
    struct pim447_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->dev = dev;
    data->prev_btn_state = false;

    /* Set a dim white LED to indicate the trackball is active */
    ret = pim447_set_led(dev, 0, 0, 0, 30);
    if (ret < 0) {
        LOG_WRN("Failed to set initial LED: %d (continuing anyway)", ret);
    }

    /* Flush any stale motion data */
    struct pim447_motion_data dummy;
    pim447_read_motion(dev, &dummy);

    /* Start polling */
    k_work_init_delayable(&data->poll_work, pim447_poll_handler);
    k_work_reschedule(&data->poll_work, K_MSEC(cfg->poll_interval_ms));

    LOG_INF("PIM447 trackball initialized (poll=%dms, swap_xy=%d, inv_x=%d, inv_y=%d, "
            "accel_factor=%d, accel_threshold=%d)",
            cfg->poll_interval_ms, cfg->swap_xy, cfg->invert_x, cfg->invert_y,
            cfg->accel_factor, cfg->accel_threshold);

    return 0;
}

/* Devicetree instance macro */
#define PIM447_INST(inst)                                                    \
    static struct pim447_data pim447_data_##inst;                            \
    static const struct pim447_config pim447_config_##inst = {               \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                  \
        .poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms),            \
        .swap_xy = DT_INST_PROP(inst, swap_xy),                              \
        .invert_x = DT_INST_PROP(inst, invert_x),                            \
        .invert_y = DT_INST_PROP(inst, invert_y),                            \
        .accel_factor = DT_INST_PROP(inst, accel_factor),                    \
        .accel_threshold = DT_INST_PROP(inst, accel_threshold),              \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst, pim447_init, NULL,                            \
                          &pim447_data_##inst, &pim447_config_##inst,          \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INST)
