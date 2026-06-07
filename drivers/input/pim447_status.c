/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 *
 * PIM447 status indicator.
 *
 * Drives the PIM447's onboard RGBW LED based on system events, providing
 * an unobtrusive status indicator that's visible even when underglow is
 * turned off.
 *
 * Current indicators (Phase 1, peripheral-local only):
 *
 *   Battery > 20%      | dim white solid    | normal idle
 *   Battery 11-20%     | orange solid       | low battery, charge soon
 *   Battery 6-10%      | red, slow blink    | critical, charge now
 *   Battery <= 5%      | red, fast blink    | dead soon
 *
 * Future indicators (Phase 2, requires central->peripheral data forwarding):
 *   BLE host disconnected
 *   Caps lock active
 *   Layer change
 *
 * The indicator does NOT interfere with the trackball driver itself. The
 * driver's polling and movement logic continues to work normally; we only
 * touch the LED via the public pim447_set_led() API.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

#include <drivers/input/pim447.h>

LOG_MODULE_REGISTER(pim447_status, CONFIG_PIM447_LOG_LEVEL);

/* Battery level thresholds (state-of-charge percentages) */
#define BAT_LEVEL_LOW       99
#define BAT_LEVEL_CRITICAL  96
#define BAT_LEVEL_DEAD       5

/* LED colors and brightness */
#define IDLE_WHITE  15      /* dim cool white for idle */
#define LOW_R       180     /* orange = R + G */
#define LOW_G        80
#define CRIT_R      200     /* pure red for critical */

/* Blink periods (full on+off cycle in ms) */
#define BLINK_SLOW_MS  1000
#define BLINK_FAST_MS   300

static const struct device *trackball_dev;

/* Blinking state */
static struct k_work_delayable blink_work;
static bool     blink_active;
static bool     blink_phase_on;
static uint8_t  blink_r, blink_g, blink_b;
static uint32_t blink_period_ms;

static int set_led_safe(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (!trackball_dev || !device_is_ready(trackball_dev)) {
        return -ENODEV;
    }
    return pim447_set_led(trackball_dev, r, g, b, w);
}

static void blink_work_handler(struct k_work *work)
{
    if (!blink_active) {
        return;
    }
    blink_phase_on = !blink_phase_on;
    if (blink_phase_on) {
        set_led_safe(blink_r, blink_g, blink_b, 0);
    } else {
        set_led_safe(0, 0, 0, 0);
    }
    k_work_reschedule(&blink_work, K_MSEC(blink_period_ms / 2));
}

static void start_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms)
{
    blink_active = true;
    blink_phase_on = false;
    blink_r = r;
    blink_g = g;
    blink_b = b;
    blink_period_ms = period_ms;
    k_work_reschedule(&blink_work, K_NO_WAIT);
}

static void stop_blink_and_set(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    blink_active = false;
    k_work_cancel_delayable(&blink_work);
    set_led_safe(r, g, b, w);
}

static void update_status(uint8_t battery_pct)
{
    LOG_DBG("battery: %u%%", battery_pct);

    if (battery_pct <= BAT_LEVEL_DEAD) {
        start_blink(CRIT_R, 0, 0, BLINK_FAST_MS);
    } else if (battery_pct <= BAT_LEVEL_CRITICAL) {
        start_blink(CRIT_R, 0, 0, BLINK_SLOW_MS);
    } else if (battery_pct <= BAT_LEVEL_LOW) {
        stop_blink_and_set(LOW_R, LOW_G, 0, 0);
    } else {
        stop_blink_and_set(0, 0, 0, IDLE_WHITE);
    }
}

static int battery_listener(const zmk_event_t *eh)
{
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev) {
        update_status(ev->state_of_charge);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(pim447_status_battery, battery_listener);
ZMK_SUBSCRIPTION(pim447_status_battery, zmk_battery_state_changed);

static int pim447_status_init(void)
{
    trackball_dev = DEVICE_DT_GET(DT_NODELABEL(trackball));

    if (!device_is_ready(trackball_dev)) {
        LOG_ERR("PIM447 trackball not ready - status indicator disabled");
        return -ENODEV;
    }

    k_work_init_delayable(&blink_work, blink_work_handler);

    /* Initial state: idle white. Will be updated by the first battery event. */
    set_led_safe(0, 0, 0, IDLE_WHITE);

    LOG_INF("PIM447 status indicator active");
    return 0;
}

SYS_INIT(pim447_status_init, APPLICATION, 99);
