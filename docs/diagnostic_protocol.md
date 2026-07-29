# EERMS Sensor Diagnostic Protocol

## Protocol version

Current protocol version: 1

## Status characteristic

UUID:

1234567b-1234-5678-1234-56789abcdef0

Packet length: 20 bytes

| Offset | Size | Field | Encoding |
|---:|---:|---|---|
| 0 | 1 | Protocol version | Unsigned |
| 1 | 1 | Packet length | Unsigned |
| 2 | 1 | Application mode | Code |
| 3 | 1 | Runtime state | Code |
| 4 | 1 | Last wake reason | Code |
| 5 | 1 | Bluetooth status flags | Bit field |
| 6 | 2 | Current error code | Little-endian unsigned |
| 8 | 4 | State-transition count | Little-endian unsigned |
| 12 | 4 | Uptime in seconds | Little-endian unsigned |
| 16 | 4 | Completed vibration windows | Little-endian unsigned |

## Application-mode codes

| Code | Name | Meaning |
|---:|---|---|
| 0x00 | NORMAL | Low-power monitoring operation |
| 0x01 | CONFIG | Configuration and provisioning operation |
| 0x02 | DIAGNOSTIC | Continuous measurement and development operation |

## Runtime-state codes

| Code | Name | Meaning |
|---:|---|---|
| 0x00 | BOOT | Firmware initialization |
| 0x01 | IDLE | Waiting for a wake event |
| 0x02 | MEASURING | Collecting a measurement window |
| 0x03 | REPORTING | Result available through Bluetooth |
| 0x04 | ERROR | Operation stopped due to an error |

## Wake-reason codes

| Code | Name | Meaning |
|---:|---|---|
| 0x00 | NONE | No wake reason recorded |
| 0x01 | BOOT | Startup initialization |
| 0x02 | SHAKE | LIS2DW12 motion event |
| 0x03 | PERIODIC | Scheduled periodic wake |
| 0x04 | NFC | NFC interaction |
| 0x05 | BLE_COMMAND | Bluetooth command |

## Bluetooth flag bits

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | 0x01 | Bluetooth connection active |
| 1 | 0x02 | Advertising requested by application policy |
| 2 | 0x04 | Advertising currently active |

## Error codes

| Code | Name | Meaning |
|---:|---|---|
| 0x0000 | APP_ERROR_NONE | No product error recorded |