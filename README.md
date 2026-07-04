# AH-64D Auto Rudder / F-14 Roll Assist

Windows C++ external control assist for DCS. The AH-64D mode listens to telemetry and writes one final rudder axis to a separate vJoy device. The F-14 mode adds a high-AoA roll-to-rudder mixer that can also reduce roll-axis output.

## Input Chain

Use this topology:

```text
T300 pedals -> UCR -> vJoy #1 -> ah64d_auto_rudder -> vJoy #2 -> DCS AH-64D rudder
```

DCS should bind the AH-64D rudder axis only to vJoy #2. Do not also bind the T300 pedals or vJoy #1 to AH-64D rudder, or DCS will receive competing axes.

The app does not write rudder through DCS-BIOS. DCS-BIOS is used only for telemetry.

For F-14 high-AoA roll assist, use this topology instead:

```text
Physical roll/rudder controls -> UCR -> vJoy #1 -> ah64d_auto_rudder -> vJoy #2 -> DCS F-14 roll/rudder
```

DCS should bind the F-14 roll and rudder axes only to vJoy #2. Do not leave the original stick/pedals or vJoy #1 also bound to the F-14 axes, or the assist cannot suppress high-AoA roll input cleanly.

## Requirements

- Windows
- DCS-BIOS installed under `C:\Users\15423\Saved Games\DCS\Scripts\DCS-BIOS` for AH-64D DCS-BIOS telemetry
- vJoy installed, with vJoy #1 and vJoy #2 enabled
- UCR already mapping the T300 pedals to vJoy #1
- CMake 3.20+ and MSVC Build Tools / Visual Studio 2022

## Build

From a Developer PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
```

The executables will be under:

```text
build\Release\ah64d_auto_rudder.exe
build\Release\f14_rudder_roll_assist.exe
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
rudder_input_center=0
rudder_input_deadzone=0
rudder_input_scale=1
```

`input_vjoy_id` is the DirectInput occurrence among devices matching `input_device_name_contains`. With two `vJoy Device` entries, use `--list-devices` and the logged `instance={...}` GUID to verify which one is UCR's output. `input_device_name_contains` can be set to part of that GUID for an exact match.

If the log shows a non-zero released pedal such as `rawPedal=-0.45`, set `rudder_input_center=-0.45` and add a small `rudder_input_deadzone`. The controller uses the corrected `pedal=` value, while `rawPedal=` remains in the log for diagnostics.

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

With DCS in the F-14 and `control_mode=f14_roll_assist`, confirm the log shows:

- `acft=F-14...`
- `fresh=yes`
- `roll` and `rudder` changing when you move the controls
- `rawAoa` showing the raw DCS export value
- `aoaU` showing a number, not `NA`
- `state` changing between `GROUND`, `LANDING`, `NORMAL_FLIGHT`, and `HIGH_AOA_COMBAT` as expected

Before flying, verify the output vJoy binding:

```powershell
.\build\Release\ah64d_auto_rudder.exe --test-output
```

This sweeps `output_vjoy_id` / `axis_name` continuously. In DCS, bind AH-64D rudder to the vJoy column/axis that moves during this test. If RCtrl+Enter or the DCS axis setup does not show movement, the rudder binding is on the wrong device/axis.

In F-14 mode, `--test-output` sweeps `f14_output_roll_axis_name` and `f14_output_rudder_axis_name` in opposite directions so both DCS bindings can be checked in one run.

## Normal Run

```powershell
.\build\Release\ah64d_auto_rudder.exe
.\build\Release\f14_rudder_roll_assist.exe
```

You can also use one executable with an explicit profile:

```powershell
.\build\Release\ah64d_auto_rudder.exe --ah64d
.\build\Release\ah64d_auto_rudder.exe --f14
.\build\Release\ah64d_auto_rudder.exe --profile f14
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

Use this with DCS already running, spawned into a safe hover/low-speed state. Keep pedals centered, use collective to keep the aircraft under control, and let the app tune rudder behavior. Every 10 seconds it can apply all valid recommendations from the same analysis window:

```text
- collective_gain for steady collective feedforward
- collective_rate_gain for sudden collective transients
- kp for centered yaw-rate damping
- heading_hold_max_assist when normal centered hold is authority-limited
```

The active values are only changed in memory. On exit, copy the final logged values into `config.ini` if they feel better. By default the app still controls only the final rudder vJoy axis; it does not take over collective or throttle bindings.

For faster tuning, the app can also drive a temporary vJoy collective axis:

```powershell
.\build\Release\ah64d_auto_rudder.exe --auto-tune --drive-collective
```

In that mode, temporarily bind AH-64D Pilot `总距` to vJoy #2 `Z` and remove the direct Warthog collective binding for the tuning flight. The app writes physical collective passthrough plus small scripted up/down moves around your Warthog RZ position. Keep the aircraft in a safe hover/low-speed state, keep pedals centered, and stop with Ctrl+C. Relevant config keys are `collective_output_axis_name`, `collective_output_invert`, `auto_tune_collective_amplitude`, `auto_tune_collective_period`, `auto_tune_collective_settle_time`, and `auto_tune_collective_rate_limit`.

If tuning is interrupted or vJoy appears to hold the last tune output, reset the output once:

```powershell
.\build\Release\ah64d_auto_rudder.exe --reset-output
```

This centers the configured rudder axis and writes the current physical collective to the configured vJoy collective axis.

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

For F-14 high-AoA roll assist:

```ini
control_mode=f14_roll_assist
telemetry_source=fast_export
f14_roll_axis_name=X
f14_rudder_axis_name=RZ
f14_roll_input_id=0
f14_roll_device_name_contains=
f14_rudder_input_id=0
f14_rudder_device_name_contains=
f14_output_roll_axis_name=X
f14_output_rudder_axis_name=RZ
```

This mode uses the fast export script and writes both final axes to `output_vjoy_id`. By default, roll and rudder are read from `input_vjoy_id`, but `f14_roll_input_id` / `f14_roll_device_name_contains` and `f14_rudder_input_id` / `f14_rudder_device_name_contains` can override each input separately. That is useful when roll is still on a physical stick but pedals are routed through UCR/vJoy. In flight, physical rudder is ignored by default; rudder becomes an internal actuator driven by roll intent, AoA, yaw rate, and beta/wing-rock guards. Ground mode still passes physical rudder through for steering.

```text
GROUND:
  roll_out = pilot_roll
  rudder_out = physical_rudder

LANDING:
  roll_out = pilot_roll
  rudder_out = light yaw damping / beta limiter

NORMAL_FLIGHT:
  roll_out = pilot_roll
  rudder_out = light coordination and damping

HIGH_AOA_COMBAT:
  roll_out = pilot_roll * scheduled_roll_keep
  rudder_out = scheduled roll-to-rudder + beta/r limiter
```

`LoGetAngleOfAttack()` is converted to approximate F-14 AoA units with `f14_aoa_units_offset + f14_aoa_units_per_radian * raw_aoa`. The field name is kept for compatibility, but current DCS/F-14 logs show the raw export behaves like degrees here, so the default is `1.5` AoA units per exported degree. If your export reports radians, use `85.9436692696`. Calibrate this mapping against the cockpit AoA gauge before increasing authority.

## Main Tuning Fields

- `yaw_response_sign`: flip between `1` and `-1` if heading-hold correction makes yaw worse. If the log shows `yawZ` and `final` keep the same sign while yaw grows, flip this first.
- `yaw_rate_sign`: sign used to convert angular-rate exports into the same convention as heading rate.
- `yaw_rate_source`: `heading` derives yaw rate from actual heading change and is the current AH-64D default because large cyclic movement can contaminate body-axis angular rates. `angular` trusts the exported angular rate directly. `auto` trusts angular rate but falls back to heading derivative when angular rate clearly underreports. `yawCtl` in the log is still the raw angular value, while `hRate` is the actual rate used by the controller.
- `assist_sign`: old `yaw_damper` mode correction sign.
- `kp`: yaw-rate inner-loop correction strength.
- `heading_kp`: how strongly heading error commands yaw rate in `heading_hold`.
- `heading_rate_limit`: maximum automatic yaw-rate command while holding heading.
- `heading_hold_max_assist`: maximum feedback authority while pedals are centered. This keeps heading hold from slamming to full pedal and oscillating.
- `heading_error_deadband`: heading error ignored near zero to avoid small hold-mode hunting.
- `heading_hold_leak_time`: optional softening that lets the held heading drift toward the current nose direction. Use `0` for the strictest heading hold; non-zero values can feel calmer but will allow slow self-yaw to persist.
- `release_brake_*`: short high-authority damping after you release a turn command. During this brake window the heading reference follows the current nose direction, so pedal release damps yaw rate instead of springing back to the exact release heading. The same authority is also used as a rate-rescue path when centered-pedal self-yaw is already large enough that normal heading-hold limits cannot overcome a stale feedforward bias.
- `turn_rate_max`: logged yaw-rate intent scale. Active pedal turn output is direct, so the final vJoy axis is not limited by this value.
- `pedal_command_*`: deadzone, hysteresis, and sign for treating pedals as turn commands.
- `ki`: optional centered heading-hold integrator. Defaults to `0`; do not enable it until the proportional heading hold is stable.
- `integral_limit`: maximum steady hold contribution from `ki`.
- `max_assist`: maximum automatic rudder offset.
- `yaw_rate_deadband`: yaw rate ignored near zero.
- `yaw_rate_error_exponent`: heading-hold inner-loop error shaping. Values below `1` make small yaw-rate errors produce a firmer correction without increasing full-authority output.
- `yaw_rate_integral_*`: a leaky centered-pedal yaw-rate hold term. It helps remove slow steady self-yaw that proportional damping leaves behind, and is cleared on active pedal turn, trim guard, stale telemetry, or heading relock.
- `pedal_override_threshold`: pedal deflection where user input overrides assist in old `yaw_damper` mode.
- `pedal_rate_override_threshold`: pedal movement speed where assist fades out.
- `trim_capture_*`: old `yaw_damper` mode manual trim capture. Defaults off because it can store a bad bias and cause runaway in FBW use.
- `power_feedforward_source`: selects the disturbance input used by the existing `collective_*` compensator. `collective` is the old direct collective-stick behavior. `fuel_flow` uses normalized fast-export fuel flow. `fuel_rpm` uses fuel flow plus an RPM-drop correction, which is the current default for AH-64D testing.
- `fuel_flow_min` / `fuel_flow_max` / `rpm_nominal` / `rpm_drop_full_scale` / `rpm_power_gain`: scaling for `fuel_flow` and `fuel_rpm`. In the log, `coll=` remains the physical collective input, `ffSrc=` shows the active disturbance source, `ffIn=` is the 0..1 value entering the compensator, and `cff=` is the rudder offset computed from it.
- `collective_*`: historical name for the power/disturbance compensator. With `power_feedforward_source=fuel_rpm`, these parameters no longer mean physical collective position; they tune how much rudder is applied for the normalized fuel/RPM proxy and its rate of change.
- `collective_transient_*`: fast path for sudden disturbance-input changes. The derivative term is gated by `collective_transient_rate_threshold`, so small proxy jitter does not add extra rudder.
- `tq`, `tqL`, `tqR` in the log: optional engine torque probe values from the fast Lua export. For AH-64D the exporter probes cockpit arguments `982`/`983` as left/right torque candidates, but ignores all-zero values for `tq` because flight torque cannot be zero. It then falls back to `LoGetEngineInfo()`/FM torque-like fields for `tq`. These values are logged only and are not used by the control law until verified against the in-cockpit ENG/FLT torque display. `NA` means that path did not provide a usable value.
- `fade_in_time` / `fade_out_time`: smoothing for automatic assist.
- `filter_time`: yaw-rate low-pass filter time. Lower values reduce damping latency but can expose more export noise.
- `stale_timeout`: disables assist if DCS-BIOS stops updating.
- `f14_aoa_*`: high-AoA gate and raw-AoA-to-units conversion for F-14 mode.
- `f14_deep_aoa_*`: second high-AoA stage for deeper roll-axis suppression.
- `f14_roll_washout` / `f14_deep_roll_washout` / `f14_roll_min_scale`: how much roll-axis output is removed in high-AoA combat mode.
- `f14_roll_to_rudder_*`: rudder mixer gain/sign from lateral stick demand.
- `f14_beta_limit` / `f14_beta_limiter_gain`: high-AoA beta limiter. It allows some sideslip for rudder roll and only corrects excess slip.
- `f14_ground_*` / `f14_landing_*`: state-machine thresholds for ground steering and landing-mode protection.
- `f14_rudder_max_assist` / `f14_rudder_rate_limit`: F-14 automatic rudder authority and slew rate.
- `f14_reversal_*`: reduces the roll-to-rudder mix after sustained opposite roll-rate response instead of commanding opposite full rudder.

The shipped defaults are medium authority for the AH-64D fast export path. If correction direction is wrong, flip `assist_sign` before increasing authority. If it hunts or oscillates, lower `ki` first, then lower `kp`.

## Tests

On any C++20 toolchain:

```powershell
cmake -S . -B build
cmake --build build --target autorudder_tests
ctest --test-dir build
```

The tests cover DCS-BIOS frame parsing, JSON output reference extraction, AH-64D yaw-damper behavior, and F-14 state-machine roll/rudder assist behavior.

## License

Project source code is licensed under the MIT License. Bundled tracker music
assets are not relicensed by the project license; see `THIRD_PARTY_NOTICES.md`.
