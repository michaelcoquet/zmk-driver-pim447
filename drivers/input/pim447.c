/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 *
 * Zephyr input driver for the Pimoroni PIM447 Trackball Breakout.
 *
 * MOVEMENT MODEL
 * --------------
 * The PIM447 is a low-resolution sensor. A vigorous swipe generates only
 * 5-10 counts. To make the trackball useful on large displays, we need
 * aggressive top-end scaling without losing precision for slow movements.
 *
 * The driver computes acceleration based on the COMBINED magnitude of dx
 * and dy (not per-axis) so that diagonal movements receive the same boost
 * as straight ones. The same scale factor is applied to both axes
 * proportionally - so a 45-degree swipe accelerates as much as a pure
 * horizontal swipe of the same speed.
 *
 * Formula (integer math):
 *   mag        = approx sqrt(dx^2 + dy^2)
 *   over       = max(0, mag - accel-divisor)
 *   scale_num  = base-scale + over^accel-exponent  (clamped)
 *   dx_out     = dx * scale_num / 16
 *   dy_out     = dy * scale_num / 16
 *
 * base-scale is in 1/16ths (i.e. base-scale=16 = 1.0x linear).
 *
 * Reasonable starting values for a 1080p display:
 *   base-scale = 32       (2.0x linear baseline)
 *   accel-divisor = 2     (start accelerating after 2 counts)
 *   accel-exponent = 2    (quadratic)
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pim447, CONFIG_PIM447_LOG_LEVEL);

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

/* Caps the maximum scale factor to prevent overflow on huge deltas. */
#define ACCEL_SCALE_MAX 16384

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
    uint16_t base_scale;
    uint16_t accel_divisor;
    uint8_t  accel_exponent;
};

struct pim447_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
    bool prev_btn_state;
};

/**
 * Integer approximation of sqrt(x^2 + y^2) using alpha-max-plus-beta-min
 * (alpha=15/16, beta=15/32). Accurate to ~3.5%, plenty for cursor accel.
 */
static inline uint32_t approx_magnitude(int32_t x, int32_t y)
{
    uint32_t ax = (x < 0) ? -x : x;
    uint32_t ay = (y < 0) ? -y : y;
    uint32_t max_v = (ax > ay) ? ax : ay;
    uint32_t min_v = (ax > ay) ? ay : ax;
    return ((max_v * 15) >> 4) + ((min_v * 15) >> 5);
}

/**
 * Integer power with overflow clamp.
 */
static inline uint32_t int_pow_clamped(uint32_t base, uint8_t exp)
{
    if (base == 0) {
        return (exp == 0) ? 1 : 0;
    }
    uint32_t result = 1;
    for (uint8_t i = 0; i < exp; i++) {
        if (result > ACCEL_SCALE_MAX) {
            return ACCEL_SCALE_MAX;
        }
        result *= base;
    }
    return (result > ACCEL_SCALE_MAX) ? ACCEL_SCALE_MAX : result;
}

/**
 * Magnitude-based acceleration. The same scale factor is applied to both
 * axes so diagonal swipes get the same boost as straight ones.
 */
static void apply_acceleration(int16_t dx, int16_t dy,
                               const struct pim447_config *cfg,
                               int16_t *dx_out, int16_t *dy_out)
{
    if (dx == 0 && dy == 0) {
        *dx_out = 0;
        *dy_out = 0;
        return;
    }

    uint32_t scale_num;
    if (cfg->accel_exponent == 0 || cfg->accel_divisor == 0) {
        scale_num = cfg->base_scale;
    } else {
        uint32_t mag = approx_magnitude(dx, dy);
        uint32_t over = (mag > cfg->accel_divisor)
                        ? (mag - cfg->accel_divisor) : 0;
        scale_num = cfg->base_scale + int_pow_clamped(over, cfg->accel_exponent);
        if (scale_num > ACCEL_SCALE_MAX) {
            scale_num = ACCEL_SCALE_MAX;
        }
    }

    /* base unit is 16, so divide by 16 after multiplying */
    int32_t sx = ((int32_t)dx * (int32_t)scale_num) / 16;
    int32_t sy = ((int32_t)dy * (int32_t)scale_num) / 16;

    *dx_out = (int16_t)CLAMP(sx, INT16_MIN, INT16_MAX);
    *dy_out = (int16_t)CLAMP(sy, INT16_MIN, INT16_MAX);
}

/**
 * Atomic 5-byte burst read of registers 0x04-0x08.
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

int pim447_set_led(const struct device *dev, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    const struct pim447_config *cfg = dev->config;
    uint8_t buf[5] = { PIM447_REG_LED_RED, r, g, b, w };

    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

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

    int16_t dx_raw = (int16_t)motion.right - (int16_t)motion.left;
    int16_t dy_raw = (int16_t)motion.down  - (int16_t)motion.up;

    int16_t dx, dy;
    apply_acceleration(dx_raw, dy_raw, cfg, &dx, &dy);

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

    if (dx != 0 || dy != 0) {
        if (dx != 0) {
            input_report_rel(dev, INPUT_REL_X, dx, dy == 0, K_FOREVER);
        }
        if (dy != 0) {
            input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
        }
        LOG_DBG("raw=(%d,%d) accel=(%d,%d)", dx_raw, dy_raw, dx, dy);
    }

    bool btn_pressed = (motion.sw & PIM447_MSK_SWITCH_STATE) != 0;
    if (btn_pressed != data->prev_btn_state) {
        input_report_key(dev, INPUT_BTN_0, btn_pressed ? 1 : 0, true, K_FOREVER);
        data->prev_btn_state = btn_pressed;
        LOG_DBG("btn=%d", btn_pressed);
    }

reschedule:
    k_work_reschedule(dwork, K_MSEC(cfg->poll_interval_ms));
}

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

    ret = pim447_set_led(dev, 0, 0, 0, 30);
    if (ret < 0) {
        LOG_WRN("Failed to set initial LED: %d (continuing anyway)", ret);
    }

    struct pim447_motion_data dummy;
    pim447_read_motion(dev, &dummy);

    k_work_init_delayable(&data->poll_work, pim447_poll_handler);
    k_work_reschedule(&data->poll_work, K_MSEC(cfg->poll_interval_ms));

    LOG_INF("PIM447 trackball initialized (poll=%dms, swap_xy=%d, inv_x=%d, inv_y=%d, "
            "base_scale=%d, accel_divisor=%d, accel_exponent=%d)",
            cfg->poll_interval_ms, cfg->swap_xy, cfg->invert_x, cfg->invert_y,
            cfg->base_scale, cfg->accel_divisor, cfg->accel_exponent);

    return 0;
}

#define PIM447_INST(inst)                                                    \
    static struct pim447_data pim447_data_##inst;                            \
    static const struct pim447_config pim447_config_##inst = {               \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                  \
        .poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms),            \
        .swap_xy = DT_INST_PROP(inst, swap_xy),                              \
        .invert_x = DT_INST_PROP(inst, invert_x),                            \
        .invert_y = DT_INST_PROP(inst, invert_y),                            \
        .base_scale = DT_INST_PROP(inst, base_scale),                        \
        .accel_divisor = DT_INST_PROP(inst, accel_divisor),                  \
        .accel_exponent = DT_INST_PROP(inst, accel_exponent),                \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst, pim447_init, NULL,                            \
                          &pim447_data_##inst, &pim447_config_##inst,          \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_INST)
