# zmk-driver-pim447

Zephyr input driver module for the [Pimoroni PIM447 Trackball Breakout](https://shop.pimoroni.com/products/trackball-breakout), designed for use with [ZMK Firmware](https://zmk.dev).

## Features

- **Mouse pointer movement** via Zephyr input subsystem (`INPUT_REL_X`, `INPUT_REL_Y`)
- **Click button** reported as `INPUT_BTN_0`
- **RGBW LED control** via `pim447_set_led()` function
- **Configurable polling interval** (default 15ms / ~67Hz)
- **Axis swap and inversion** via devicetree properties
- Compatible with ZMK's `zmk,input-listener` and `zmk,input-split` for split keyboards

## Setup

### 1. Add to `config/west.yml`

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: YOUR_GITHUB_USERNAME
      url-base: https://github.com/YOUR_GITHUB_USERNAME
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-driver-pim447
      remote: YOUR_GITHUB_USERNAME
      revision: main
  self:
    path: config
```

### 2. Add to shield overlay (right half example)

```devicetree
&pro_micro_i2c {
    status = "okay";

    trackball: trackball@a {
        compatible = "pimoroni,pim447";
        reg = <0xa>;
        status = "okay";
        poll-interval-ms = <15>;
        /* swap-xy; */
        /* invert-x; */
        /* invert-y; */
    };
};
```

### 3. Add input listener to shared `.dtsi`

```devicetree
/ {
    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
    };
};
```

For split keyboards, use `zmk,input-split` to relay trackball data from the peripheral to the central half. See [ZMK pointing device docs](https://zmk.dev/docs/development/hardware-integration/pointing).

### 4. Enable in shield `.conf`

```ini
CONFIG_I2C=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PIM447=y
```

## I2C Register Map

| Register | Name    | R/W   | Description                              |
|----------|---------|-------|------------------------------------------|
| 0x00     | LED_RED | Write | Red LED brightness (0-255)               |
| 0x01     | LED_GRN | Write | Green LED brightness (0-255)             |
| 0x02     | LED_BLU | Write | Blue LED brightness (0-255)              |
| 0x03     | LED_WHT | Write | White LED brightness (0-255)             |
| 0x04     | LEFT    | Read  | Left movement delta (clears on read)     |
| 0x05     | RIGHT   | Read  | Right movement delta (clears on read)    |
| 0x06     | UP      | Read  | Up movement delta (clears on read)       |
| 0x07     | DOWN    | Read  | Down movement delta (clears on read)     |
| 0x08     | SWITCH  | Read  | Bit 7 = pressed (clears on read)         |

Default I2C address: `0x0A` (can be changed to `0x0B` by cutting a trace on the PCB).

## License

MIT
