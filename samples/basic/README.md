# Seeed XIAO mmWave 24GHz LD2410 Basic Sample

Demonstrates reading presence data from an LD2410 mmWave sensor using
the out-of-tree Zephyr driver.

## Prerequisites

**Baud rate configuration:** The LD2410 factory default baud rate is 256000,
which is not supported by the nRF52 UARTE peripheral. Before connecting the
sensor to your board, you must change the baud rate to **9600** using the
HLKRadarTool app:

1. Install the app:
   - [Android (Google Play)](https://play.google.com/store/apps/details?id=com.hlk.hlkradartool)
   - [iOS (App Store)](https://apps.apple.com/us/app/hlkradartool/id1638651152)
2. Power the sensor and connect via Bluetooth (device name: `HLK-LD2410_xxxx`)
3. Go to **More > Settings** and change the serial port baud rate to **9600**
4. The sensor will restart with the new baud rate

For more details see the
[Seeed XIAO mmWave wiki](https://wiki.seeedstudio.com/mmwave_for_xiao/).

## Hardware

- Seeed XIAO BLE (nRF52840) or compatible board
- Seeed XIAO mmWave 24GHz, or 
  HiLink LD2410/LD2410B/LD2410C sensor module.


**Antenna orientation:** The sensor antenna must face outwards (towards the
detection area). Detection angle is ±60°, range 0.75–6 m.

### Wiring (XIAO BLE)

| XIAO Pin | nRF Pin | LD2410 Pin | Function |
|----------|---------|------------|----------|
| D2       | P0.28   | TX         | UART RX  |
| D3       | P0.29   | RX         | UART TX  |
| D10      | P0.26   | OUT        | GPIO (optional) |
| 3V3      | -       | VCC        | Power    |
| GND      | -       | GND        | Ground   |

## Installing the Driver

### Option 1: West Module

Add to your `west.yml` manifest:

```yaml
manifest:
  projects:
    - name: ld2410
      url: https://github.com/yourname/ld2410-zephyr
      revision: main
      path: modules/lib/ld2410
```

Then run `west update`.

### Option 2: Clone Standalone

```bash
cd ~/zephyrproject
git clone https://github.com/yourname/ld2410-zephyr.git ld2410
```

## Building

From your Zephyr workspace root (e.g. `~/zephyrproject`):

```bash
west build -b xiao_ble/nrf52840 \
  modules/lib/ld2410/samples/basic \
  -- -DZEPHYR_EXTRA_MODULES=$(pwd)/modules/lib/ld2410
```

Or if you cloned standalone (Option 2 above):

```bash
west build -b xiao_ble/nrf52840 ld2410/samples/basic \
  -- -DZEPHYR_EXTRA_MODULES=$(pwd)/ld2410
```

## Flashing

Enter the UF2 bootloader by double-tapping the reset button on the XIAO BLE,
then:

```bash
west flash
```

Or copy the generated `build/zephyr/zephyr.uf2` to the USB mass storage device
that appears.

## Expected Output

```
[00:00:03.000,000] <inf> app: LD2410 mmWave sensor ready
[00:00:06.000,000] <inf> app: Firmware: type=0x0001 V6.06091400
[00:00:06.500,000] <inf> app: Configuration complete
[00:00:06.500,000] <inf> app: Sensor ready — receiving data via callback
[00:00:06.600,000] <inf> app: [moving] mov=120cm(E45) sta=0cm(E0) det=120cm
```
