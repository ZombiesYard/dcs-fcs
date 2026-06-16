#include "config.h"
#include "dcs_bios_protocol.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
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
    cfg.kp = 2.2;
    cfg.ki = 0.0;
    cfg.integral_limit = 0.0;
    cfg.max_assist = 0.85;
    cfg.yaw_rate_deadband = 0.0;
    cfg.heading_kp = 2.0;
    cfg.heading_rate_limit = 0.35;
    cfg.turn_rate_max = 0.60;
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

void test_heading_hold_opposes_negative_yaw_rate() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = -0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    const auto output = damper.update(input);
    expect_true(output.final_rudder > 0.20, "negative yaw rate commands positive rudder in heading mode");
    expect_near(output.yaw_rate_command, 0.0, 0.001, "centered heading hold commands zero yaw rate when on heading");
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
    auto output = damper.update(input);
    expect_true(output.yaw_rate_command > 0.14, "pedal deflection commands yaw rate");
    expect_true(output.reason == "turn command", "pedal deflection reports turn command");

    input.heading = 0.4;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.0;
    output = damper.update(input);
    expect_near(output.heading_ref, 0.4, 0.001, "releasing pedal captures current heading");
    expect_near(output.heading_error, 0.0, 0.001, "new heading reference has no immediate error");
    expect_near(output.final_rudder, 0.0, 0.001, "release does not pull back toward old heading");
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
    test_heading_hold_opposes_negative_yaw_rate();
    test_heading_hold_recaptures_heading_after_turn_command();

    if (failures != 0) {
        std::cerr << failures << " test failure(s)\n";
        return EXIT_FAILURE;
    }
    std::cout << "All tests passed\n";
    return EXIT_SUCCESS;
}
