/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: MIT
 *
 * Public API for the Pimoroni PIM447 trackball driver.
 *
 * This header exposes LED control to other modules (e.g. the status
 * indicator) without requiring direct access to the driver internals.
 */

#ifndef ZMK_DRIVERS_INPUT_PIM447_H_
#define ZMK_DRIVERS_INPUT_PIM447_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Set the RGBW LED on a PIM447 trackball device.
 *
 * The PIM447 has four independent LED channels (Red, Green, Blue, White)
 * each ranging 0-255. Channels mix additively. Setting all to 0 turns
 * the LED off.
 *
 * The white channel uses a separate cool-white LED which is more power
 * efficient than mixing R+G+B to produce white.
 *
 * @param dev Pointer to the device returned by DEVICE_DT_GET on the
 *            PIM447 node.
 * @param r Red intensity (0-255)
 * @param g Green intensity (0-255)
 * @param b Blue intensity (0-255)
 * @param w White intensity (0-255)
 * @return 0 on success, negative errno on I2C failure.
 */
int pim447_set_led(const struct device *dev,
                   uint8_t r, uint8_t g, uint8_t b, uint8_t w);

#ifdef __cplusplus
}
#endif

#endif /* ZMK_DRIVERS_INPUT_PIM447_H_ */
