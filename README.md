<!--
  SPDX-License-Identifier: Apache-2.0
-->

# LD2410 Zephyr Driver Module

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Zephyr](https://img.shields.io/badge/Zephyr-3.7+-brightgreen.svg)](https://www.zephyrproject.org)

Out-of-tree Zephyr RTOS driver module for the **HiLink LD2410** family of 24GHz FMCW mmWave human presence sensors.

## Features

- Full serial protocol implementation (19 commands)
- Zephyr device model integration (`DEVICE_DT_INST_DEFINE`)
- Devicetree binding with optional OUT pin GPIO
- Multi-instance support
- ISR-driven async UART reception with ring buffer
- Byte-by-byte state-machine frame parser
- Thread-safe command/ACK synchronisation
- Engineering mode with per-gate energy readout
- OUT pin GPIO support (poll & interrupt-driven)
- Auxiliary control: OUT pin polarity, light-sense gating (LD2410B/C)
- Bluetooth on/off, password, MAC address commands (LD2410B/C)
- Distance resolution configuration (0.75 m or 0.20 m per gate)
- 56 unit tests (ztest) validated against protocol documentation

## Supported Hardware

| Module      | UART | Bluetooth | Light Sensor | OUT Pin Config     |
|-------------|------|-----------|--------------|--------------------|
| LD2410      | Yes  | No        | No           | Fixed (active-high)|
| LD2410B     | Yes  | Yes       | Yes          | Configurable       |
| LD2410C     | Yes  | Yes       | Yes          | Configurable       |
| Seeed XIAO mmWave (101010001) | Yes  | Yes       | Yes          | Configurable       |

> **Note:** The **LD2410S** is **not supported** by this driver. Despite sharing
> the same frame delimiters, the LD2410S uses a completely different protocol
> (different command codes, data format, parameter model, and 16 gates instead
> of 9). It would require a separate driver implementation.

## Installation

### Option 1: West Module

Add to your `west.yml` manifest:

```yaml
manifest:
  projects:
    - name: ld2410
      url: https://github.com/sirfragles/ld2410-zephyr
      revision: main
      path: modules/lib/ld2410
```

Then run `west update`.

### Option 2: ZEPHYR_EXTRA_MODULES

```bash
west build -b <board> <app> -- -DZEPHYR_EXTRA_MODULES=/path/to/ld2410
```

## Baud Rate Configuration

The LD2410 factory default baud rate is **256000**, which is **not supported**
by the nRF52 UARTE peripheral. Before connecting the sensor to an nRF52-based
board (such as the XIAO BLE), you must change the baud rate to **9600** using
the HLKRadarTool app:

1. Install the app:
   - [Android (Google Play)](https://play.google.com/store/apps/details?id=com.hlk.hlkradartool)
   - [iOS (App Store)](https://apps.apple.com/us/app/hlkradartool/id1638651152)
2. Power the sensor and connect via Bluetooth (device name: `HLK-LD2410_xxxx`)
3. Go to **More > Settings** and change the serial port baud rate to **9600**
4. The sensor will restart with the new baud rate

The driver can also change the baud rate at runtime using `ld2410_set_baud_rate()`,
but this requires an initial connection at the current baud rate first.

For more details see the
[Seeed XIAO mmWave wiki](https://wiki.seeedstudio.com/mmwave_for_xiao/).

## Quick Start

### 1. Devicetree Overlay

Add the sensor as a child of your UART node:

```dts
&uart1 {
    status = "okay";
    current-speed = <9600>;  /* LD2410 default is 256000; see Baud Rate section */

    ld2410: ld2410 {
        compatible = "hilink,ld2410";
        out-gpios = <&gpio0 26 GPIO_ACTIVE_HIGH>;  /* optional */
    };
};
```

### 2. Project Configuration (`prj.conf`)

```ini
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_RING_BUFFER=y
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LD2410=y
```

### 3. Application Code

```c
#include <ld2410/ld2410.h>

static void on_data(const struct ld2410_frame *frame, void *ctx)
{
    printk("State: %s, distance: %u cm\n",
           ld2410_target_state_str(frame->target.state),
           frame->target.detection_distance_cm);
}

int main(void)
{
    const struct device *sensor = DEVICE_DT_GET_ANY(hilink_ld2410);

    if (!device_is_ready(sensor)) {
        return -ENODEV;
    }

    ld2410_register_callback(sensor, on_data, NULL);

    /* Configure sensor */
    ld2410_configure(sensor, my_config_fn, NULL);

    while (true) {
        k_sleep(K_FOREVER);
    }
}
```

## Kconfig Options

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_LD2410_RX_BUF_SIZE` | 256 | UART receive ring buffer size (bytes) |
| `CONFIG_LD2410_CMD_TIMEOUT_MS` | 1000 | ACK response timeout (ms) |
| `CONFIG_LD2410_THREAD_STACK_SIZE` | 1024 | RX processing thread stack |
| `CONFIG_LD2410_THREAD_PRIORITY` | 5 | RX processing thread priority |
| `CONFIG_LD2410_INIT_PRIORITY` | 90 | Driver initialization priority |
| `CONFIG_LD2410_LOG_LEVEL_*` | INF | Standard Zephyr log levels |

## API Reference

### Configuration Commands

All configuration commands must be called between `ld2410_config_enable()` and
`ld2410_config_end()`, or use the `ld2410_configure()` convenience wrapper.

| Function | Cmd | Description |
|----------|-----|-------------|
| `ld2410_register_callback()` | - | Register data frame callback |
| `ld2410_config_enable()` | 0x00FF | Enter configuration mode |
| `ld2410_config_end()` | 0x00FE | Exit configuration mode |
| `ld2410_configure()` | - | Wrapped config_enable + callback + config_end |
| `ld2410_set_max_gate_and_duration()` | 0x0060 | Set max detection gates and timeout |
| `ld2410_read_config()` | 0x0061 | Read all configuration parameters |
| `ld2410_engineering_mode_enable()` | 0x0062 | Enable per-gate energy output |
| `ld2410_engineering_mode_disable()` | 0x0063 | Disable engineering mode |
| `ld2410_set_gate_sensitivity()` | 0x0064 | Set gate sensitivity (individual or all) |
| `ld2410_read_firmware_version()` | 0x00A0 | Read firmware version |
| `ld2410_set_baud_rate()` | 0x00A1 | Set UART baud rate (needs restart) |
| `ld2410_factory_reset()` | 0x00A2 | Restore factory defaults (needs restart) |
| `ld2410_restart()` | 0x00A3 | Restart module |
| `ld2410_bluetooth_set()` | 0x00A4 | Enable/disable Bluetooth (B/C only) |
| `ld2410_get_mac_address()` | 0x00A5 | Get Bluetooth MAC address (B/C only) |
| `ld2410_bluetooth_authenticate()` | 0x00A8 | Authenticate via Bluetooth (B/C only) |
| `ld2410_bluetooth_set_password()` | 0x00A9 | Set Bluetooth password (B/C only) |
| `ld2410_set_distance_resolution()` | 0x00AA | Set distance resolution (B/C only) |
| `ld2410_get_distance_resolution()` | 0x00AB | Query distance resolution (B/C only) |
| `ld2410_set_aux_control()` | 0x00AD | Set OUT pin polarity and light-sense (B/C only) |
| `ld2410_get_aux_control()` | 0x00AE | Query auxiliary control (B/C only) |

### OUT Pin GPIO

Requires `out-gpios` property in devicetree.

| Function | Description |
|----------|-------------|
| `ld2410_out_pin_read()` | Poll OUT pin state (1=presence, 0=clear) |
| `ld2410_out_pin_set_interrupt()` | Register edge-triggered callback |

### Helpers

| Function | Description |
|----------|-------------|
| `ld2410_gate_to_cm()` | Convert gate index to distance in cm |
| `ld2410_target_state_str()` | Human-readable target state string |

## File Structure

```
ld2410/
├── CMakeLists.txt
├── Kconfig
├── LICENSE
├── README.md
├── zephyr/module.yml
├── drivers/sensor/ld2410/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── ld2410.c
│   └── ld2410_protocol.c
├── include/ld2410/
│   ├── ld2410.h
│   └── ld2410_protocol.h
├── dts/bindings/sensor/
│   └── hilink,ld2410.yaml
├── samples/basic/
│   ├── boards/xiao_ble.overlay
│   └── src/main.c
└── tests/protocol/
    ├── testcase.yaml
    └── src/main.c
```

## Running Tests

From your Zephyr workspace root:

```bash
west twister -T modules/lib/ld2410/tests \
  --extra-args ZEPHYR_EXTRA_MODULES=$(pwd)/modules/lib/ld2410 \
  -p native_sim
```

Or if cloned standalone:

```bash
west twister -T ld2410/tests \
  --extra-args ZEPHYR_EXTRA_MODULES=$(pwd)/ld2410 \
  -p native_sim
```

## Protocol References

- HLK-LD2410 Serial Communication Protocol V1.02
- HLK-LD2410B Serial Communication Protocol V1.06
- 24GHz mmWave Sensor for XIAO User Manual V1.00

## License

Apache-2.0 — See [LICENSE](LICENSE) for details.
