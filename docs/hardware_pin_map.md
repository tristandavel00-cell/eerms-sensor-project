# Production hardware pin map

Board: EERMS Bluetooth Sensor, Board2 / PCB4
MCU module: Raytac MDBT42V-P512KV2
Firmware baseline: 49d5d9f
Schematic revision date: 2026-05-25
BOM, pick-and-place, and Gerber date: 2026-06-23

## MCU signal map

| Firmware role | Schematic net | U6 pad | nRF52832 pin | Peripheral use | Board endpoint | Status |
|---|---|---:|---|---|---|---|
| Accelerometer I2C clock | SCL | 27 | P0.03 / AIN1 | Digital TWI SCL | U11 pin 1 | Needs bus pull-up |
| Accelerometer I2C data | SDA | 17 | P0.04 / AIN2 | Digital TWI SDA | U11 pin 4 | Needs bus pull-up |
| Accelerometer interrupt | INT1 | 28 | P0.11 | GPIO input | U11 pin 12 | Usable |
| U4 enable control | ON | 16 | P0.00 / XL1 | GPIO output | U4 pin 6 | Hardware review required |
| Original temperature route | ADC_TEMP | 25 | P0.25 | Digital GPIO only | R7, R8, and C11 midpoint | Must be isolated during prototype rework |
| Temperature measurement, prototype rework | ADC_TEMP | 23 | P0.29 / AIN5 | SAADC input | R7, R8, and C11 midpoint through bodge wire | Selected |
| Battery-divider enable | Q1_E | 12 | P0.08 | GPIO output | Q1 gate | Usable |
| Battery measurement | ADC | 19 | P0.30 / AIN6 | SAADC input | R4 and R5 midpoint | Usable |
| NFC antenna leg 1 | Unnamed NFC net | 11 | P0.09 / NFC1 | NFCT | U9 pin 1 | Deferred |
| NFC antenna leg 2 | Unnamed NFC net | 13 | P0.10 / NFC2 | NFCT | U9 pin 2 | Deferred |
| Analog rework candidate | Unused | 24 | P0.28 / AIN4 | Possible SAADC input | No routed load | Candidate |
| Analog rework candidate | Unused | 23 | P0.29 / AIN5 | Possible SAADC input | No routed load | Candidate |

## Accelerometer connections

| LIS2DW12 pin | Signal | Connection |
|---:|---|---|
| 1 | SCL | U6 P0.03 |
| 2 | CS | U4 VOUT through VDD_SENSOR |
| 3 | SDO / SA0 | GND |
| 4 | SDA | U6 P0.04 |
| 6 | GND | GND |
| 7 | RES | GND |
| 8 | GND | GND |
| 9 | VDD | VDD directly |
| 10 | VDDIO | VDD directly |
| 12 | INT1 | U6 P0.11 |

## Hardware issues

- H-001: ADC_TEMP was routed to P0.25, which is not an SAADC-capable pin. Prototype correction selected: isolate the P0.25 route and connect ADC_TEMP to P0.29/AIN5.
- H-002: U4 VBIAS is not connected. Prototype correction selected: connect VBIAS to the protected VDD rail.
- H-003: U4 VOUT is connected to the LIS2DW12 CS pin, not to the accelerometer supply. CS selects I2C or SPI; it does not power the accelerometer.
- H-004: No external I2C pull-up resistors are fitted. Prototype correction selected: add one pull-up from SCL to VDD and one from SDA to VDD.
- H-005: No 32.768 kHz crystal is fitted; firmware must use the LFRC clock source.
- H-006: The schematic and manufacturing files have different revision dates.

## Decisions still required

- Select the prototype and production correction for ADC_TEMP.
- Decide whether LIS2DW12 CS should be tied high or actively controlled.
- Decide the I2C pull-up resistance and physical placement.
- Confirm whether the assembled prototype can be safely reworked.
- Create a custom Zephyr board definition using the correct LFCLK source.

## Programming connector

H4 is a six-pin Tag-Connect SWD connector.

| H4 pin | Signal | Purpose |
|---:|---|---|
| 1 | VDD | Target voltage reference |
| 2 | SWDIO | Serial Wire Debug data |
| 3 | GND | Ground |
| 4 | SWDCLK | Serial Wire Debug clock |
| 5 | GND | Ground |
| 6 | RESET | Target reset |

## SWD bring-up result

The production PCB was successfully detected through the nRF52-DK P20 external debug interface using the TC2030-CTX cable and a custom remapping adapter.

J-Link connection settings:

- Device: NRF52832_XXAA
- Interface: SWD
- Speed: 100 kHz

Successful detection included:

- SW-DP ID: 0x2BA01477
- Cortex-M4 r0p1 detected
- Target core identified successfully

This confirms that target power reference, common ground, SWDIO, SWDCLK, cable orientation, and external-target selection are functioning.