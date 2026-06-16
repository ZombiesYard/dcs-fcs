# AH-64D Auto Rudder

Windows C++ external yaw FBW / auto-rudder for the DCS AH-64D. It listens to DCS telemetry and writes one final rudder axis to a separate vJoy device.

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
- `hdg` showing a number, not `NA`, for heading-hold mode

Before flying, verify the output vJoy binding:

```powershell
.\build\Release\ah64d_auto_rudder.exe --test-output
```

This sweeps `output_vjoy_id` / `axis_name` continuously. In DCS, bind AH-64D rudder to the vJoy column/axis that moves during this test. If RCtrl+Enter or the DCS axis setup does not show movement, the rudder binding is on the wrong device/axis.

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

## Tune Session

After the sign is correct and the helicopter is controllable, run a tune session:

```powershell
.\build\Release\ah64d_auto_rudder.exe --tune-session
```

This mode still writes the final rudder to vJoy #2, but it also prints a tuning summary every 10 seconds and once again when you exit with Ctrl+C. Use it in this order:

```text
1. Center pedals in hover or low-speed flight.
2. Move collective up/down several times, then hold it steady.
3. Let the aircraft settle in centered-pedal heading hold.
4. Ignore pedal turn-command segments; the analyzer ignores them too.
```

The analyzer first looks for steady collective feedforward error and recommends `collective_gain` changes when centered-pedal feedback is doing constant work. Then it looks at collective transients and may recommend `collective_rate_gain`. Finally it scores quiet centered-pedal heading hold and may recommend `kp` or, only for non-VRS normal segments, `heading_hold_max_assist`.

It deliberately excludes stale telemetry, non-AH-64D data, active pedal turns, and unstable/VRS-like segments. The recommendations are logged only; it does not edit `config.ini` automatically.

For live in-memory tuning, use:

```powershell
.\build\Release\ah64d_auto_rudder.exe --auto-tune
```

Use this with DCS already running, spawned into a safe hover/low-speed state. Keep pedals centered, use collective to keep the aircraft under control, and let the app tune rudder behavior. Every 10 seconds it applies at most one conservative parameter change, in this priority order:

```text
1. collective_gain
2. collective_rate_gain
3. kp
4. heading_hold_max_assist
```

The active values are only changed in memory. On exit, copy the final logged values into `config.ini` if they feel better. The app still controls only the final rudder vJoy axis; it does not take over collective or throttle bindings.

## Control Modes

The default mode is:

```ini
control_mode=heading_hold
```

In this mode the pedal axis is treated as yaw intent, not as direct anti-torque pedal position:

```text
pedal centered -> hold current heading
pedal deflected -> command yaw rate
pedal released -> capture current heading as the new reference
```

The old mixed-axis behavior is still available with:

```ini
control_mode=yaw_damper
```

That mode uses `final = physical_pedal + assist_offset` and fades assist out during pedal override.

## Main Tuning Fields

- `yaw_response_sign`: flip between `1` and `-1` if heading-hold correction makes yaw worse. If the log shows `yawZ` and `final` keep the same sign while yaw grows, flip this first.
- `yaw_rate_sign`: sign used when heading rate must fall back to `yawZ`; in the observed AH-64D export, positive rudder increased heading while `yawZ` often went negative, so the default is `-1`.
- `assist_sign`: old `yaw_damper` mode correction sign.
- `kp`: yaw-rate inner-loop correction strength.
- `heading_kp`: how strongly heading error commands yaw rate in `heading_hold`.
- `heading_rate_limit`: maximum automatic yaw-rate command while holding heading.
- `heading_hold_max_assist`: maximum feedback authority while pedals are centered. This keeps heading hold from slamming to full pedal and oscillating.
- `heading_error_deadband`: heading error ignored near zero to avoid small hold-mode hunting.
- `release_brake_*`: short high-authority damping after you release a turn command. This stops residual yaw rate before returning to normal heading-hold authority.
- `turn_rate_max`: logged yaw-rate intent scale. Active pedal turn output is direct, so the final vJoy axis is not limited by this value.
- `pedal_command_*`: deadzone, hysteresis, and sign for treating pedals as turn commands.
- `ki`: optional centered heading-hold integrator. Defaults to `0`; do not enable it until the proportional heading hold is stable.
- `integral_limit`: maximum steady hold contribution from `ki`.
- `max_assist`: maximum automatic rudder offset.
- `yaw_rate_deadband`: yaw rate ignored near zero.
- `pedal_override_threshold`: pedal deflection where user input overrides assist in old `yaw_damper` mode.
- `pedal_rate_override_threshold`: pedal movement speed where assist fades out.
- `trim_capture_*`: old `yaw_damper` mode manual trim capture. Defaults off because it can store a bad bias and cause runaway in FBW use.
- `collective_*`: collective feedforward. This is the part that reacts before yaw rate appears. Default `collective_source=auto`; it uses fast export collective if available, otherwise the configured fallback. For an Xbox controller, use `collective_source=xinput` and one of `LX`, `LY`, `RX`, `RY`, `LT`, `RT`. If the log shows `coll=NA`, run `--list-devices` while moving the collective and configure the axis that changes.
- `collective_transient_*`: fast path for sudden collective changes. It lets feedforward enter faster without making normal heading-hold feedback more aggressive.
- `fade_in_time` / `fade_out_time`: smoothing for automatic assist.
- `filter_time`: yaw-rate low-pass filter time.
- `stale_timeout`: disables assist if DCS-BIOS stops updating.

The shipped defaults are medium authority for the AH-64D fast export path. If correction direction is wrong, flip `assist_sign` before increasing authority. If it hunts or oscillates, lower `ki` first, then lower `kp`.

## Tests

On any C++20 toolchain:

```powershell
cmake -S . -B build
cmake --build build --target autorudder_tests
ctest --test-dir build
```

The tests cover DCS-BIOS frame parsing, JSON output reference extraction, and yaw-damper override/fallback behavior.
