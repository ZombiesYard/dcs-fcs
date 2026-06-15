# AH-64D Auto Rudder

Windows C++ yaw-rate damper for the DCS AH-64D. It listens to DCS-BIOS telemetry and writes one final mixed rudder axis to a separate vJoy device.

## Input Chain

Use this topology:

```text
T300 pedals -> UCR -> vJoy #1 -> ah64d_auto_rudder -> vJoy #2 -> DCS AH-64D rudder
```

DCS should bind the AH-64D rudder axis only to vJoy #2. Do not also bind the T300 pedals or vJoy #1 to AH-64D rudder, or DCS will receive competing axes.

The app does not write rudder through DCS-BIOS. DCS-BIOS is used only for telemetry.

## Requirements

- Windows
- DCS-BIOS installed under `C:\Users\15423\Saved Games\DCS\Scripts\DCS-BIOS`
- vJoy installed, with vJoy #1 and vJoy #2 enabled
- UCR already mapping the T300 pedals to vJoy #1
- CMake 3.20+ and MSVC Build Tools / Visual Studio 2022

## Build

From a Developer PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

The executable will be under:

```text
build\Release\ah64d_auto_rudder.exe
```

If `vJoyInterface.dll` is not on `PATH`, copy it from the vJoy install folder next to the executable or add its folder to `PATH`.

## First Run

List devices first:

```powershell
.\build\Release\ah64d_auto_rudder.exe --list-devices
```

Check that the configured input device is the UCR/vJoy #1 pedal output. If the wrong device is selected, adjust these in `config.ini`:

```ini
input_vjoy_id=1
output_vjoy_id=2
input_device_name_contains=vJoy
axis_name=X
```

DCS-BIOS multicast defaults are also in `config.ini`. The default interface is loopback because DCS-BIOS exports to the local machine:

```ini
multicast_address=239.255.50.10
multicast_interface=127.0.0.1
udp_port=5010
```

## Optional High-Rate Telemetry

DCS-BIOS usually exports around 30 Hz. That is usable for a gentle damper, but a yaw-rate damper feels better with lower latency. This project includes an optional high-rate Lua export that sends only the data needed by this app over local UDP.

Install it from Windows:

```powershell
cd "L:\DCS FCS"
.\install_fast_export.bat
```

Then change `config.ini`:

```ini
telemetry_source=fast_export
fast_export_bind_address=127.0.0.1
fast_export_port=34380
```

Keep the existing DCS-BIOS line in `Export.lua`. The fast export script is chained after it and does not replace DCS-BIOS.

Then run without writing vJoy:

```powershell
.\build\Release\ah64d_auto_rudder.exe --dry-run
```

With DCS in the AH-64D, confirm the log shows:

- `acft=AH-64D...`
- `fresh=yes`
- pedal values changing when you move the pedals
- `yawZ` changing when the helicopter yaws

## Normal Run

```powershell
.\build\Release\ah64d_auto_rudder.exe
```

Press the configured hotkey, default `PAUSE`, to toggle assist. Use Ctrl+C to stop.

## Calibration

The default direction may need flipping. Use low-authority sign calibration only in a safe test mission:

```powershell
.\build\Release\ah64d_auto_rudder.exe --calibrate-sign
```

Keep pedals centered. The app tests `assist_sign=+1` and `assist_sign=-1`, then logs the lower mean absolute yaw-rate recommendation. Put that value into `config.ini`.

## Main Tuning Fields

- `assist_sign`: flip between `1` and `-1` if correction makes yaw worse.
- `kp`: yaw-rate correction strength.
- `max_assist`: maximum automatic rudder offset.
- `yaw_rate_deadband`: yaw rate ignored near zero.
- `pedal_override_threshold`: pedal deflection where user input overrides assist.
- `pedal_rate_override_threshold`: pedal movement speed where assist fades out.
- `fade_in_time` / `fade_out_time`: smoothing for automatic assist.
- `filter_time`: yaw-rate low-pass filter time.
- `stale_timeout`: disables assist if DCS-BIOS stops updating.

Start with the defaults. Increase `kp` slowly, and keep `max_assist` conservative until the direction is verified.

## Tests

On any C++20 toolchain:

```powershell
cmake -S . -B build
cmake --build build --target autorudder_tests
ctest --test-dir build
```

The tests cover DCS-BIOS frame parsing, JSON output reference extraction, and yaw-damper override/fallback behavior.
