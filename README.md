# Firmware cleanup and accelerometer settling step

This bundle is based on the source files uploaded on 2026-07-29.

## Purpose of this checkpoint

1. Correct runtime issues found during the code review.
2. Clean formatting and reduce duplicated event-handler logic.
3. Discard the first four accelerometer samples after switching from the
   low-rate motion profile to the 100 Hz measurement profile.
4. Keep this as a compile-only checkpoint. Do not flash the last battery yet.

At 100 Hz, four discarded samples represent approximately 40 ms. The discarded
samples are not included in the one-second vibration window and are not sent in
raw accelerometer notifications.

## Important functional corrections

- `sensor_accel_init()` is called once, not twice.
- Motion detection is configured before NORMAL mode tries to arm it.
- `motion_armed` starts as `false`; it changes to `true` only after a successful
  `sensor_trigger_set()` call.
- Raw BLE X/Y/Z storage now writes the correct three variables.
- Raw accelerometer notification returns `-ENOTCONN` when disconnected and
  checks whether notifications are enabled.
- Click, mode, and raw accelerometer notifications use UUID lookup instead of
  hard-coded GATT attribute indexes.
- The unused `accel_window_count_get` BLE callback was removed.
- NFC formatting and command parsing were cleaned without changing the command
  strings or payload purpose.

## Files

Copy the files from this bundle over the matching files in the project.

```text
CMakeLists.txt
prj.conf
src/main.c
src/app_ble.c
src/app_ble.h
src/app_nfc.c
src/app_nfc.h
src/app_types.h
src/sensor_accel.c
src/sensor_accel.h
```

Keep the existing `app.overlay`; it was not included in the uploaded review set
and this checkpoint does not require an overlay change.

## Build only

From the VS Code nRF Connect terminal:

```powershell
cd C:\Users\DELL\Documents\nrf_projects\nrf_build_clean
west build -d build
```

Do not flash this checkpoint yet.

## Deferred production work

- Stable BLE protocol version, runtime-state codes, wake-reason codes, and
  documented product error codes.
- Product Bluetooth name and production UUID allocation.
- Production logging configuration with unused console backends disabled.
- BLE advertising policy for low-power NORMAL mode.
- Moving vibration-window processing out of `main.c` into a measurement module.
- Data-ready/FIFO-based sampling if timer polling proves insufficient during
  validation.
