#include "config.h"
#include "dcs_bios_protocol.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
#include "tune_session.h"
#include "yaw_damper.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int failures = 0;

void expect_true(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void expect_near(double actual, double expected, double epsilon, const std::string& message) {
    if (std::abs(actual - expected) > epsilon) {
        ++failures;
        std::cerr << "FAIL: " << message << " actual=" << actual << " expected=" << expected << '\n';
    }
}

void test_protocol_parser_applies_writes() {
    autorudder::DcsBiosState state(2048);
    int frames = 0;
    autorudder::DcsBiosProtocolParser parser;
    parser.set_frame_callback([&] { ++frames; });
    parser.set_write_callback([&](const autorudder::BiosWrite& write) { state.apply_write(write); });

    const std::vector<std::uint8_t> packet = {
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x04, 0x00, 'A', 'H', 0, 0,
        0x5e, 0x04, 0x02, 0x00, 0x34, 0x12,
    };
    parser.feed(packet.data(), 5);
    parser.feed(packet.data() + 5, packet.size() - 5);

    expect_true(frames == 1, "parser detects one frame");
    expect_true(state.read_string(0x0400, 4) == "AH", "string write is applied");
    expect_true(state.read_u16(0x045e).value_or(0) == 0x1234, "integer write is applied");
}

void test_protocol_parser_handles_multiple_frames() {
    autorudder::DcsBiosState state(2048);
    int frames = 0;
    autorudder::DcsBiosProtocolParser parser;
    parser.set_frame_callback([&] { ++frames; });
    parser.set_write_callback([&](const autorudder::BiosWrite& write) { state.apply_write(write); });

    const std::vector<std::uint8_t> packet = {
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x02, 0x00, 'A', 0,
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x02, 0x00, 'B', 0,
    };
    parser.feed(packet.data(), packet.size());

    expect_true(frames == 2, "parser detects repeated frame sync");
    expect_true(state.read_string(0x0400, 2) == "B", "second frame write is applied");
}

void test_ref_extraction() {
    const std::string json = R"JSON(
{
  "Speed": {
    "ANGULAR_VELOCITY_Z": {
      "identifier": "ANGULAR_VELOCITY_Z",
      "outputs": [{"address": 1118, "max_length": 6, "type": "string"}]
    }
  },
  "Gauge": {
    "PLT_SAI_BALL": {
      "identifier": "PLT_SAI_BALL",
      "outputs": [{"address": 32828, "mask": 65535, "shift_by": 0, "max_value": 65535, "type": "integer"}]
    }
  }
}
)JSON";
    const auto yaw = autorudder::extract_output_ref(json, "ANGULAR_VELOCITY_Z");
    const auto ball = autorudder::extract_output_ref(json, "PLT_SAI_BALL");
    expect_true(yaw.has_value(), "yaw ref found");
    expect_true(ball.has_value(), "ball ref found");
    expect_true(yaw->address == 1118, "yaw address parsed");
    expect_true(yaw->max_length == 6, "yaw max length parsed");
    expect_true(ball->address == 32828, "ball address parsed");
}

autorudder::AppConfig test_config() {
    autorudder::AppConfig cfg;
    cfg.control_mode = "yaw_damper";
    cfg.assist_sign = 1.0;
    cfg.kp = 0.5;
    cfg.ki = 0.0;
    cfg.integral_limit = 0.0;
    cfg.max_assist = 0.2;
    cfg.trim_capture_enabled = 0.0;
    cfg.yaw_rate_deadband = 0.0;
    cfg.pedal_override_threshold = 0.12;
    cfg.pedal_rate_override_threshold = 10.0;
    cfg.fade_in_time = 0.1;
    cfg.fade_out_time = 0.1;
    cfg.filter_time = 0.0;
    return cfg;
}

autorudder::AppConfig heading_config() {
    autorudder::AppConfig cfg;
    cfg.control_mode = "heading_hold";
    cfg.yaw_response_sign = 1.0;
    cfg.yaw_rate_sign = -1.0;
    cfg.kp = 1.45;
    cfg.ki = 0.0;
    cfg.integral_limit = 0.0;
    cfg.max_assist = 0.85;
    cfg.heading_hold_max_assist = 0.35;
    cfg.release_brake_time = 1.8;
    cfg.release_brake_kp = 3.2;
    cfg.release_brake_max_assist = 0.85;
    cfg.yaw_rate_deadband = 0.0;
    cfg.heading_error_deadband = 0.0;
    cfg.heading_kp = 0.75;
    cfg.heading_rate_limit = 0.30;
    cfg.turn_rate_max = 0.45;
    cfg.pedal_command_sign = 1.0;
    cfg.pedal_command_deadzone = 0.06;
    cfg.pedal_command_exit_deadzone = 0.03;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    cfg.filter_time = 0.0;
    cfg.collective_gain = 0.0;
    cfg.collective_rate_gain = 0.0;
    return cfg;
}

void test_yaw_damper_assists_centered_pedals() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.3;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.assist_offset < -0.14, "positive yaw rate commands negative assist");
    expect_true(output.final_rudder < -0.14, "final rudder includes assist");
    expect_true(output.assist_active, "assist is active");
}

void test_yaw_damper_user_override() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.3;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    for (int i = 0; i < 10; ++i) {
        damper.update(input);
    }

    input.physical_rudder = 0.5;
    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.user_override, "pedal displacement triggers override");
    expect_near(output.assist_offset, 0.0, 0.001, "assist fades to zero during override");
    expect_near(output.final_rudder, 0.5, 0.001, "override passes pedal through");
}

void test_yaw_damper_stale_or_wrong_aircraft_passes_through() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = -0.2;
    input.yaw_rate_z = 1.0;
    input.telemetry_fresh = false;
    input.aircraft_is_ah64 = true;

    const auto output = damper.update(input);
    expect_near(output.final_rudder, -0.2, 0.001, "stale telemetry passes through");
    expect_near(output.assist_offset, 0.0, 0.001, "stale telemetry has no assist");
}

void test_yaw_damper_integrates_centered_steady_rate() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.integral_limit = 0.3;
    cfg.max_assist = 0.5;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = -0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    autorudder::YawDamperOutput output;
    for (int i = 0; i < 50; ++i) {
        output = damper.update(input);
    }
    expect_true(output.integral_assist < -0.25, "steady yaw rate builds hold assist");
    expect_true(output.final_rudder < -0.25, "integral assist contributes to final rudder");
}

void test_yaw_damper_override_resets_integral() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.integral_limit = 0.3;
    cfg.max_assist = 0.5;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = -0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;
    for (int i = 0; i < 20; ++i) {
        damper.update(input);
    }

    input.physical_rudder = -0.5;
    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.user_override, "manual pedal still overrides with integral enabled");
    expect_near(output.integral_assist, 0.0, 0.001, "manual pedal clears hold integral");
    expect_near(output.final_rudder, -0.5, 0.001, "manual pedal passes through after integral reset");
}

void test_yaw_damper_captures_manual_trim_on_release() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.trim_capture_enabled = 1.0;
    cfg.trim_capture_min_pedal = 0.2;
    cfg.trim_capture_yaw_rate = 0.02;
    cfg.trim_capture_pedal_rate = 0.5;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.physical_rudder = -0.65;
    input.yaw_rate_z = 0.0;
    damper.update(input);
    damper.update(input);

    input.physical_rudder = 0.0;
    damper.update(input);
    const auto output = damper.update(input);

    expect_near(output.trim_bias, -0.65, 0.001, "stable manual pedal is captured as trim");
    expect_near(output.final_rudder, -0.65, 0.001, "captured trim drives centered-pedal output");
}

void test_yaw_damper_collective_feedforward() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.0;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.0;
    input.collective = 0.60;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    const auto output = damper.update(input);
    expect_near(output.collective_feedforward, -0.42, 0.001, "collective feedforward commands left rudder");
    expect_near(output.final_rudder, -0.42, 0.001, "collective feedforward reaches final rudder");
}

void test_yaw_damper_trim_capture_subtracts_collective_feedforward() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.40;
    cfg.collective_rate_gain = 0.0;
    cfg.trim_capture_enabled = 1.0;
    cfg.trim_capture_min_pedal = 0.2;
    cfg.trim_capture_yaw_rate = 0.02;
    cfg.trim_capture_pedal_rate = 0.5;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.collective = 0.50;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.physical_rudder = -0.65;
    input.yaw_rate_z = 0.0;
    damper.update(input);
    damper.update(input);

    input.physical_rudder = 0.0;
    damper.update(input);
    const auto output = damper.update(input);

    expect_near(output.collective_feedforward, -0.20, 0.001, "collective feedforward remains active");
    expect_near(output.trim_bias, -0.45, 0.001, "captured trim stores only the remaining rudder");
    expect_near(output.final_rudder, -0.65, 0.001, "feedforward plus trim reproduces manual rudder");
}

void test_heading_hold_damps_heading_rate() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.02;
    const auto output = damper.update(input);

    expect_true(output.heading_rate > 0.19, "heading derivative reports positive heading rate");
    expect_true(output.final_rudder < -0.20, "positive heading rate commands braking rudder");
    expect_true(output.yaw_rate_command < 0.0, "positive heading error commands return yaw rate");
    expect_true(output.reason == "heading hold", "centered heading mode reports heading hold");
}

void test_heading_hold_recaptures_heading_after_turn_command() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = 0.5;
    input.heading = 0.4;
    auto output = damper.update(input);
    expect_true(output.yaw_rate_command > 0.10, "pedal deflection commands yaw rate");
    expect_true(output.reason == "turn command", "pedal deflection reports turn command");
    expect_near(output.final_rudder, 0.5, 0.001, "turn command maps pedal directly to output");

    input.heading = 0.4;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.0;
    output = damper.update(input);
    expect_near(output.heading_ref, 0.4, 0.001, "releasing pedal captures current heading");
    expect_near(output.heading_error, 0.0, 0.001, "new heading reference has no immediate error");
    expect_near(output.final_rudder, 0.0, 0.001, "release does not pull back toward old heading");
}

void test_heading_hold_uses_release_brake_after_turn_command() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = 0.1;
    input.heading = 0.1;
    damper.update(input);

    input.physical_rudder = 0.0;
    input.heading = 0.14;
    const auto output = damper.update(input);

    expect_true(output.reason == "release brake", "release from turn command enters brake mode");
    expect_true(output.final_rudder < -0.50, "release brake can exceed normal heading hold authority");
    expect_true(output.final_rudder >= -0.85, "release brake remains authority limited");
}

void test_heading_hold_allows_full_pedal_turn_output() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = -1.0;
    input.heading = -0.1;
    const auto output = damper.update(input);

    expect_true(output.reason == "turn command", "full pedal enters turn command");
    expect_near(output.final_rudder, -1.0, 0.001, "full pedal turn is not yaw-rate limited");
}

autorudder::TuneConfig tune_config() {
    autorudder::TuneConfig cfg;
    cfg.kp = 1.45;
    cfg.heading_hold_max_assist = 0.35;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.20;
    cfg.collective_sign = -1.0;
    return cfg;
}

void test_tune_session_recommends_collective_gain_increase() {
    const auto cfg = tune_config();
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.01;
    sample.heading_error = 0.01;
    sample.collective = 0.5;
    sample.collective_rate = 0.0;
    sample.collective_feedforward = -0.35;
    sample.final_rudder = -0.45;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.heading_hold_mode = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.recommended_collective_gain.has_value(), "steady feedback recommends collective gain");
    expect_true(*report.recommended_collective_gain > cfg.collective_gain, "collective gain increases when feedback adds same-direction rudder");
}

void test_tune_session_recommends_kp_increase_for_slow_damping() {
    const auto cfg = tune_config();
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.10;
    sample.heading_error = 0.02;
    sample.final_rudder = -0.10;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = false;
    sample.heading_hold_mode = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.recommended_kp.has_value(), "slow centered yaw damping recommends kp");
    expect_true(*report.recommended_kp > cfg.kp, "kp increases when hRate remains high without saturation");
}

void test_tune_session_excludes_unstable_segments() {
    autorudder::TuneSessionAnalyzer analyzer(tune_config());

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 2.0;
    sample.heading_error = 1.2;
    sample.final_rudder = -0.35;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.heading_hold_mode = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.usable_seconds < 0.001, "unstable segments are not usable tune data");
    expect_true(report.excluded_unstable_seconds > 5.0, "unstable segment time is reported");
    expect_true(!report.recommended_heading_hold_max_assist.has_value(), "unstable saturation does not recommend more authority");
}

void test_tune_update_prioritizes_collective_gain() {
    const auto cfg = tune_config();
    autorudder::TuneReport report;
    report.recommended_collective_gain = 0.85;
    report.recommended_collective_rate_gain = 0.30;
    report.recommended_kp = 1.60;

    const auto update = autorudder::choose_tune_update(cfg, report);

    expect_true(update.changed, "tune update applies a change");
    expect_near(update.config.collective_gain, 0.85, 0.001, "collective gain is first priority");
    expect_near(update.config.collective_rate_gain, cfg.collective_rate_gain, 0.001, "lower-priority rate gain is held");
    expect_near(update.config.kp, cfg.kp, 0.001, "lower-priority kp is held");
}

void test_tune_update_applies_kp_when_feedforward_has_no_change() {
    const auto cfg = tune_config();
    autorudder::TuneReport report;
    report.recommended_kp = 1.60;

    const auto update = autorudder::choose_tune_update(cfg, report);

    expect_true(update.changed, "tune update applies kp change");
    expect_near(update.config.kp, 1.60, 0.001, "kp changes when higher-priority stages have no update");
    expect_near(update.config.collective_gain, cfg.collective_gain, 0.001, "collective gain is held");
}

void test_yaw_damper_set_config_preserves_heading_reference() {
    auto cfg = heading_config();
    cfg.kp = 1.0;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    cfg.kp = 2.0;
    damper.set_config(cfg);
    input.heading = 1.10;
    const auto output = damper.update(input);

    expect_true(output.heading_error < -0.09, "heading reference survives config update");
    expect_true(output.final_rudder < -0.30, "updated kp is used without resetting damper state");
}

}  // namespace

int main() {
    test_protocol_parser_applies_writes();
    test_protocol_parser_handles_multiple_frames();
    test_ref_extraction();
    test_yaw_damper_assists_centered_pedals();
    test_yaw_damper_user_override();
    test_yaw_damper_stale_or_wrong_aircraft_passes_through();
    test_yaw_damper_integrates_centered_steady_rate();
    test_yaw_damper_override_resets_integral();
    test_yaw_damper_captures_manual_trim_on_release();
    test_yaw_damper_collective_feedforward();
    test_yaw_damper_trim_capture_subtracts_collective_feedforward();
    test_heading_hold_damps_heading_rate();
    test_heading_hold_recaptures_heading_after_turn_command();
    test_heading_hold_uses_release_brake_after_turn_command();
    test_heading_hold_allows_full_pedal_turn_output();
    test_tune_session_recommends_collective_gain_increase();
    test_tune_session_recommends_kp_increase_for_slow_damping();
    test_tune_session_excludes_unstable_segments();
    test_tune_update_prioritizes_collective_gain();
    test_tune_update_applies_kp_when_feedforward_has_no_change();
    test_yaw_damper_set_config_preserves_heading_reference();

    if (failures != 0) {
        std::cerr << failures << " test failure(s)\n";
        return EXIT_FAILURE;
    }
    std::cout << "All tests passed\n";
    return EXIT_SUCCESS;
}
