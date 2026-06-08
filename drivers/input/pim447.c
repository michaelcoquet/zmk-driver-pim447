/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 *
 * Zephyr input driver for the Pimoroni PIM447 Trackball Breakout.
 *
 * See drivers/input/pim447.h for the public API.
 *
 * READ MODE
 * ---------
 * Two modes of operation:
 *
 *   INTERRUPT-DRIVEN (preferred): if int-gpios is defined in devicetree,
 *   the driver configures the pin as a falling-edge interrupt with an
 *   internal pull-up. The PIM447's INT line is open-drain active-low; it
 *   pulls low when there is motion or button activity, and releases when
 *   the data registers are read. The driver reads on each interrupt edge,
 *   re-arming after each read. CPU is only used when there is actual input.
 *
 *   POLLING (fallback): if int-gpios is not defined, the driver falls back
 *   to periodic polling at poll-interval-ms. Higher idle CPU usage but no
 *   extra wiring needed.
 *
 * MOVEMENT MODEL
 * --------------
 * Magnitude-based acceleration applied per-read. Same scale factor applied
 * to both axes so diagonals get the same boost as straight movements.
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <drivers/input/pim447.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
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
    struct gpio_dt_spec int_gpio;
    bool has_int_gpio;
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
    struct k_work_delayable work;
    struct gpio_callback int_cb;
    bool prev_btn_state;
};

static inline uint32_t approx_magnitude(int32_t x, int32_t y)
{
    uint32_t ax = (x < 0) ? -x : x;
    uint32_t ay = (y < 0) ? -y : y;
    uint32_t max_v = (ax > ay) ? ax : ay;
    uint32_t min_v = (ax > ay) ? ay : ax;
    return ((max_v * 15) >> 4) + ((min_v * 15) >> 5);
}

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

    int32_t sx = ((int32_t)dx * (int32_t)scale_num) / 16;
    int32_t sy = ((int32_t)dy * (int32_t)scale_num) / 16;

    *dx_out = (int16_t)CLAMP(sx, INT16_MIN, INT16_MAX);
    *dy_out = (int16_t)CLAMP(sy, INT16_MIN, INT16_MAX);
}

/* Atomic 5-byte burst read of registers 0x04-0x08. Reading these registers
 * clears them on the chip, which also releases the INT line. */
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

/* Process one motion read and report input events. Shared between
 * interrupt and polling paths. */
static void pim447_process_motion(const struct device *dev, const struct pim447_motion_data *motion)
{
    const struct pim447_config *cfg = dev->config;
    struct pim447_data *data = dev->data;

    int16_t dx_raw = (int16_t)motion->right - (int16_t)motion->left;
    int16_t dy_raw = (int16_t)motion->down  - (int16_t)motion->up;

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

    bool btn_pressed = (motion->sw & PIM447_MSK_SWITCH_STATE) != 0;
    if (btn_pressed != data->prev_btn_state) {
        input_report_key(dev, INPUT_BTN_0, btn_pressed ? 1 : 0, true, K_FOREVER);
        data->prev_btn_state = btn_pressed;
        LOG_DBG("btn=%d", btn_pressed);
    }
}

/* Work handler. Called from system work queue.
 *
 * Interrupt mode: invoked by the GPIO callback. Reads once. The act of
 * reading clears the INT line on the chip; the GPIO is already armed for
 * the next edge.
 *
 * Polling mode: reschedules itself periodically. */
static void pim447_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pim447_data *data = CONTAINER_OF(dwork, struct pim447_data, work);
    const struct device *dev = data->dev;
    const struct pim447_config *cfg = dev->config;
    struct pim447_motion_data motion;
    int ret;

    ret = pim447_read_motion(dev, &motion);
    if (ret == 0) {
        pim447_process_motion(dev, &motion);
    }

    if (!cfg->has_int_gpio) {
        k_work_reschedule(dwork, K_MSEC(cfg->poll_interval_ms));
    }
}

/* GPIO interrupt callback. Runs in ISR context, so just dispatch work to
 * the system queue. I2C reads cannot run in ISR. */
static void pim447_int_handler(const struct device *port,
                               struct gpio_callback *cb,
                               gpio_port_pins_t pins)
{
    struct pim447_data *data = CONTAINER_OF(cb, struct pim447_data, int_cb);
    k_work_reschedule(&data->work, K_NO_WAIT);
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

    /* Bright green canary: this is a pure I2C write. If the LED lights at
     * boot, the I2C bus is acking and the problem is downstream (reads/INT).
     * If it stays dark, I2C is not getting through - check wiring/address. */
    ret = pim447_set_led(dev, 0, 60, 0, 0);
    if (ret < 0) {
        LOG_WRN("Failed to set initial LED: %d (continuing anyway)", ret);
    }

    /* Flush any stale data buffered on the chip. */
    struct pim447_motion_data dummy;
    pim447_read_motion(dev, &dummy);

    k_work_init_delayable(&data->work, pim447_work_handler);

    if (cfg->has_int_gpio) {
        if (!gpio_is_ready_dt(&cfg->int_gpio)) {
            LOG_ERR("INT GPIO not ready");
            return -ENODEV;
        }

        /* The PIM447 INT line is open-drain active-low. Add an internal
         * pull-up since the breakout does not include one. */
        ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO: %d", ret);
            return ret;
        }

        /* Chip drives the line low (logical active, pin is ACTIVE_LOW) when
         * there is motion or button activity, and holds it low until the
         * data registers are read. TO_ACTIVE is the falling edge here. */
        ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
                                              GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT interrupt: %d", ret);
            return ret;
        }

        gpio_init_callback(&data->int_cb, pim447_int_handler,
                           BIT(cfg->int_gpio.pin));
        ret = gpio_add_callback(cfg->int_gpio.port, &data->int_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add INT callback: %d", ret);
            return ret;
        }

        LOG_INF("PIM447 trackball initialized (interrupt mode, "
                "swap_xy=%d, inv_x=%d, inv_y=%d, "
                "base_scale=%d, accel_divisor=%d, accel_exponent=%d)",
                cfg->swap_xy, cfg->invert_x, cfg->invert_y,
                cfg->base_scale, cfg->accel_divisor, cfg->accel_exponent);
    } else {
        k_work_reschedule(&data->work, K_MSEC(cfg->poll_interval_ms));

        LOG_INF("PIM447 trackball initialized (polling mode, poll=%dms, "
                "swap_xy=%d, inv_x=%d, inv_y=%d, "
                "base_scale=%d, accel_divisor=%d, accel_exponent=%d)",
                cfg->poll_interval_ms, cfg->swap_xy, cfg->invert_x, cfg->invert_y,
                cfg->base_scale, cfg->accel_divisor, cfg->accel_exponent);
    }

    return 0;
}

#define PIM447_INT_GPIO_SPEC(inst)                                          \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios),                     \
                (GPIO_DT_SPEC_INST_GET(inst, int_gpios)),                   \
                ({0}))

#define PIM447_HAS_INT_GPIO(inst) DT_INST_NODE_HAS_PROP(inst, int_gpios)

#define PIM447_INST(inst)                                                    \
    static struct pim447_data pim447_data_##inst;                            \
    static const struct pim447_config pim447_config_##inst = {               \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                  \
        .int_gpio = PIM447_INT_GPIO_SPEC(inst),                              \
        .has_int_gpio = PIM447_HAS_INT_GPIO(inst),                           \
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
